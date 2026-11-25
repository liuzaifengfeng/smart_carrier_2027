#include <assert.h>
#include <string.h>
#include <deque>
#include <vector>
#include "motion_sequence.h"
#include "motion_job.h"
#include "motion_types.h"
#include "task_code.h"

constexpr uint8_t SCAN_WAIT=4,SCAN_SWEEP=5;

uint8_t stage,scanTurn;
uint32_t scanArrivedAt,scanTurnAt,now,trayTravelMs;
float scanTrayAngle;
bool taskReceived;
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
        assert(step.kind==motion::StepKind::TRAY_ROTATE&&step.waitMs==250);
        trayAngles.push_back(step.turret);trayTravelMs=step.waitMs;
    }
} hardware;

uint32_t millis(){return now;}
void phase(const char *){}
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
    stage=SCAN_WAIT;scanTurn=0;now=scanArrivedAt=100;scanTrayAngle=angle;
    taskReceived=false;state=MotionState::RUNNING;failure=nullptr;trayAngles.clear();scannerFrames.clear();
}
void completeTurn(){now=scanTurnAt+trayTravelMs;scanAtNode();}

int main(){
    reset();scannerFrames={"412+1\"3+124+123"};assert(!pollTaskCode());
    scannerFrames={"412+1\"3+124+123","412+123+124+123"};assert(pollTaskCode());
    scanAtNode();assert(state==MotionState::DONE&&trayAngles.empty());
    reset();now+=999;scanAtNode();assert(state==MotionState::RUNNING&&trayAngles.empty());
    taskReceived=true;scanAtNode();assert(state==MotionState::DONE&&trayAngles.empty());

    reset(270);now+=1000;scanAtNode();assert(trayAngles==std::vector<float>{290});
    taskReceived=true;now+=249;scanAtNode();assert(state==MotionState::RUNNING&&trayAngles.size()==1);
    completeTurn();assert((trayAngles==std::vector<float>{290,270})&&state==MotionState::RUNNING);
    completeTurn();assert(state==MotionState::DONE);

    reset();now+=1000;scanAtNode();
    for(size_t i=0;i<19;++i)completeTurn();
    assert(state==MotionState::RUNNING);
    completeTurn();
    assert((trayAngles==std::vector<float>{20,0,-20,0,20,0,-20,0,20,0,-20,0,20,0,-20,0,20,0,-20,0}));
    assert(state==MotionState::FAILED&&strcmp(failure,"SCAN_TIMEOUT")==0);

    reset();Serial.text.clear();
    MotionRequest route;
    nodePathRequestedAt=now;
    nodePathState=NodePathState::WAITING;
    now+=2999;
    assert(!takeRequestedPath(route)&&state==MotionState::RUNNING&&Serial.text.empty());
    ++now;
    assert(!takeRequestedPath(route)&&state==MotionState::FAILED&&strcmp(failure,"PATH_TIMEOUT")==0);
    assert(nodePathState==NodePathState::FAILED&&Serial.text=="{EVT,NAV,ROUTE_FAILED,TIMEOUT}\n");

    reset();Serial.text.clear();
    nodePathRequestedAt=now;
    nodePathState=NodePathState::RECEIVING;
    now+=3000;
    assert(!takeRequestedPath(route)&&state==MotionState::RUNNING&&Serial.text.empty());
    nodePathState=NodePathState::RECEIVED;
    assert(takeRequestedPath(route)&&nodePathState==NodePathState::RUNNING&&state==MotionState::RUNNING);
    assert(route.pathCount==3&&memcmp(route.path,nodePathBuffer,3)==0);
}
