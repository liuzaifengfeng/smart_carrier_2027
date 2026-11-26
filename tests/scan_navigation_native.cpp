#include <assert.h>
#include <string.h>
#include <deque>
#include <vector>
#include "motion_sequence.h"
#include "motion_job.h"
#include "motion_types.h"
#include "task_code.h"

constexpr uint8_t SCAN_WAIT=4,SCAN_SWEEP=5,SCAN_WAIT_CODE=6,SCAN_RETREAT=7;

uint8_t stage,scanTurn,pathRetries;
uint32_t scanArrivedAt,scanTurnAt,now,trayTravelMs;
float scanTrayAngle;
bool taskReceived,scanRetreated,retreatActive;
unsigned retreatCommands;
TaskCode currentTask;
constexpr size_t SCANNER_BUF_LEN=64;
int taskCodeMux;
std::deque<const char *> scannerFrames;
NativeSerial Serial;
void portENTER_CRITICAL(int*){}
void portEXIT_CRITICAL(int*){}
bool waitScannerCode(char *text,size_t,uint32_t timeout){
    assert(timeout==0);
    if(scannerFrames.empty())return false;
    strcpy(text,scannerFrames.front());scannerFrames.pop_front();return true;
}
const char *failure;
MotionState state;
std::vector<float> trayAngles;
struct {
    void issue(const motion::Step &step){
        assert(step.kind==motion::StepKind::TRAY_ROTATE&&step.waitMs==1000);
        trayAngles.push_back(step.turret);trayTravelMs=step.waitMs;
    }
} hardware;

bool BeginPositionMove(float x,float y,float theta,float speed,uint8_t) {
    assert(x==0&&y==-100&&theta==0&&speed==200);
    ++retreatCommands;
    return retreatActive=true;
}
void TickPositionMove(){}
bool PositionMoveActive(){return retreatActive;}
uint32_t millis(){return now;}
void phase(const char *){}
void updateDisplay(const char *, const char *, const char *){}
void finish(MotionState result,const char *reason=nullptr){state=result;failure=reason;}
void startTray(float angle){assert(angle==0);}

enum class NodePathState;
NodePathState nodePathState;
int nodePathMux;
uint8_t nodePathBuffer[3]={4,3,2};
size_t nodePathLen=3;
uint32_t nodePathRequestedAt;

#include "scan_navigation_actual.inc"

void reset(float angle=0){
    stage=SCAN_WAIT;scanTurn=pathRetries=0;now=scanArrivedAt=100;scanTrayAngle=angle;
    scanRetreated=retreatActive=false;retreatCommands=0;
    taskReceived=false;state=MotionState::RUNNING;failure=nullptr;trayAngles.clear();scannerFrames.clear();
}
void completeRetreat(){retreatActive=false;scanAtNode();}
void completeTurn(){now=scanTurnAt+trayTravelMs;scanAtNode();}

int main(){
    reset();scannerFrames={"412+1\"3+124+123"};assert(!pollTaskCode());
    scannerFrames={"412+1\"3+124+123","412+123+124+123"};assert(pollTaskCode());
    scanAtNode();assert(state==MotionState::DONE&&trayAngles.empty());
    reset();now+=999;scanAtNode();assert(state==MotionState::RUNNING&&trayAngles.empty());
    taskReceived=true;scanAtNode();assert(state==MotionState::DONE&&trayAngles.empty());

    assert(retreatCommands==0&&!scanRetreated);

    reset();now+=1000;scanAtNode();
    assert(stage==SCAN_RETREAT&&retreatCommands==1&&trayAngles.empty()&&!scanRetreated);
    scannerFrames={"412+123+124+123"};scanAtNode();
    assert(taskReceived&&state==MotionState::RUNNING&&retreatCommands==1);
    completeRetreat();assert(state==MotionState::DONE&&scanRetreated&&trayAngles.empty());

    reset(270);now+=1000;scanAtNode();completeRetreat();
    assert(scanRetreated&&retreatCommands==1&&trayAngles==std::vector<float>{320});
    taskReceived=true;now+=999;scanAtNode();assert(state==MotionState::RUNNING&&trayAngles.size()==1);
    completeTurn();assert((trayAngles==std::vector<float>{320,270})&&state==MotionState::RUNNING);
    completeTurn();assert(state==MotionState::DONE);

    reset();now+=1000;scanAtNode();completeRetreat();
    for(size_t i=0;i<39;++i)completeTurn();
    assert(state==MotionState::RUNNING&&stage==SCAN_SWEEP&&trayAngles.size()==40);
    completeTurn();
    const float expectedRound[]={50,0,-50,0};
    assert(trayAngles.size()==40);
    for(size_t i=0;i<trayAngles.size();++i)assert(trayAngles[i]==expectedRound[i%4]);
    assert(state==MotionState::RUNNING&&stage==SCAN_WAIT_CODE);
    scannerFrames={"invalid"};scanAtNode();assert(state==MotionState::RUNNING);
    scannerFrames={"412+123+124+123"};scanAtNode();assert(state==MotionState::DONE);

    reset();stage=SCAN_SWEEP;scanTurn=40;scanTurnAt=now;now+=1000;
    scanAtNode(true);
    assert(stage==SCAN_SWEEP&&scanTurn==1&&retreatCommands==0&&trayAngles==std::vector<float>{50});
    scannerFrames={"412+123+124+123"};completeTurn();completeTurn();
    assert(state==MotionState::DONE);

    reset();Serial.text.clear();
    MotionRequest route;
    nodePathRequestedAt=now;
    nodePathState=NodePathState::WAITING;
    now+=1999;
    assert(!takeRequestedPath(route)&&state==MotionState::RUNNING&&Serial.text.empty());
    ++now;
    for(uint8_t retry=1;retry<=10;++retry) {
        Serial.text.clear();
        assert(!takeRequestedPath(route)&&state==MotionState::RUNNING&&pathRetries==retry);
        assert(Serial.text=="{EVT,NAV,ROUTE_WAITING}\n{EVT,NAV,ROUTE_REQUEST,0,0}\n");
        now+=2000;
    }
    Serial.text.clear();
    assert(!takeRequestedPath(route)&&state==MotionState::FAILED&&strcmp(failure,"PATH_TIMEOUT")==0);
    assert(nodePathState==NodePathState::FAILED&&Serial.text=="{EVT,NAV,ROUTE_FAILED,TIMEOUT}\n");

    reset();Serial.text.clear();
    route.kind=MotionKind::NAVIGATE;
    nodePathRequestedAt=now;
    nodePathState=NodePathState::WAITING;
    pathRetries=10;now+=2000;
    assert(!takeRequestedPath(route)&&state==MotionState::RUNNING&&pathRetries==0);
    assert(Serial.text=="{EVT,NAV,ROUTE_RETRY,TIMEOUT}\n{EVT,NAV,ROUTE_WAITING}\n{EVT,NAV,ROUTE_REQUEST,0,0}\n");

    reset();Serial.text.clear();
    nodePathRequestedAt=now;
    nodePathState=NodePathState::RECEIVING;
    now+=3000;
    assert(!takeRequestedPath(route)&&state==MotionState::RUNNING&&Serial.text.empty());
    nodePathState=NodePathState::RECEIVED;
    assert(takeRequestedPath(route)&&nodePathState==NodePathState::RUNNING&&state==MotionState::RUNNING);
    assert(route.pathCount==3&&memcmp(route.path,nodePathBuffer,3)==0);
}
