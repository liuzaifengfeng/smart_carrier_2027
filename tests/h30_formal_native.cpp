#include "../src/continuous_trial.h"
#include "../src/node_position_control.h"
#include "../src/motion_executor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
using namespace continuous;
enum State{IDLE,RUNNING,STOPPING,DONE,FAILED};
struct Snapshot{State state=IDLE;bool formal=false,async=false;uint32_t heartbeatMs=0;float total=0,x=0,y=0,heading=0;const char *reason="NONE";} status;
struct Pose{float x=0,y=0,theta=0;} currentPose;
static Route route;static Node nodes[25];
static node_position::Settings formalSettings;
static bool busy=false,taskMotionAborted=false,configured=true,needsCalibration=false,stopRequested=false;
static uint32_t failedVersion=0,referenceVersion=0;
static float initialHeading=0,THETA_PULSE=51.8f,X_PULSE=13.3f,Y_PULSE=13.6f;
static float NODE_PATH_ACCEL_MM_S2=425.5f;
static float NODE_PATH_TURN_DEG_S=150,NODE_PATH_TURN_ACCEL_DEG_S2=360;
static float NODE_PATH_HEADING_DEADBAND_DEG=1;
static bool scanPosePending=false;
constexpr uint8_t SCAN_AREA_NODE=2;
static bool initialized=true;static int trialMux=0,launches=0;static const char *sensorError=nullptr;
struct ConfigurationGuard{bool held=!busy;};
struct Logger{template<class... T>void printf(const char*,T...) {}} Serial;
static bool otaBusy(){return false;}
static uint32_t millis(){return 1000;}
static void portENTER_CRITICAL(int*){}
static void portEXIT_CRITICAL(int*){}
static Snapshot snapshot(){return status;}
bool ContinuousTrialActive(){return status.state==RUNNING||status.state==STOPPING;}
void ContinuousTrialLaunch(){++launches;}
static const char *H30UsbGetMotionSnapshot(H30Attitude &sample,float &heading,uint32_t &version){
    sample.receivedAtMs=1000;sample.fields=7;heading=180.855f;version=7;return sensorError;
}
static bool ChassisGetNodePosition(uint8_t n,float &x,float &y){
    if(n>=25)return false;
    const float xs[]={310,720,1200,1680,2050},ys[]={240,720,1200,1680,2020};
    int row=n/5,col=n%5;if(row%2)col=4-col;x=xs[col];y=ys[row];return true;
}
static bool adjacent(uint8_t a,uint8_t b){
    int ar=a/5,ac=a%5,br=b/5,bc=b%5;if(ar%2)ac=4-ac;if(br%2)bc=4-bc;
    return abs(ar-br)+abs(ac-bc)==1;
}
static void setAlignmentEnabled(bool){}
static int pdMS_TO_TICKS(int v){return v;}
static void vTaskDelay(int){status.state=DONE;}
// 从实际业务源提取正式导航入口，不复制待测逻辑。
#include "h30_formal_actual.inc"
#define CHECK(x) do{if(!(x)){fprintf(stderr,"formal line %d: %s\n",__LINE__,#x);return 2;}}while(0)
MotionRequest submitted;
uint32_t SubmitMotion(const MotionRequest &r,bool){submitted=r;return ++launches;}
bool WaitMotion(uint32_t id){return id!=0;}
int main(){
    uint8_t path[]={10,19,20,21,22};
    CHECK(H30PrepareNodePath(path,5,300,180,true));CHECK(status.formal&&status.async);
    CHECK(route.config.policy==2&&route.config.forwardPpm==Y_PULSE&&route.config.lateralPpm==X_PULSE);
    CHECK(route.config.accel==425.5f&&route.config.maxRpm==node_position::rpmFor(300,Y_PULSE));
    CHECK(formalSettings.speedMmS==300&&formalSettings.accelMmS2==425.5f);
    CHECK(formalSettings.turnDegS==150&&formalSettings.turnAccelDegS2==360&&formalSettings.headingDeadbandDeg==1);
    CHECK(referenceVersion==7&&initialHeading>180.8f&&!configured);
    for(size_t i=0;i<route.segmentCount;++i)CHECK(route.segments[i].kind!=CUBIC);
    NODE_PATH_ACCEL_MM_S2=600;CHECK(!H30PrepareNodePath(path,5,300,180,true));CHECK(route.config.accel==425.5f);
    status.state=DONE;sensorError="OFFLINE";currentPose.theta=180.855f;
    CHECK(H30PrepareNodePath(path,5,300,180,true));CHECK(referenceVersion==0);
    status.state=DONE;sensorError=nullptr;needsCalibration=true;failedVersion=7;
    CHECK(H30PrepareNodePath(path,5,300,180,false));CHECK(referenceVersion==0);
    status.state=DONE;needsCalibration=false;uint8_t bad[]={10,20};CHECK(!H30PrepareNodePath(bad,2,300,180,true));
    NODE_PATH_ACCEL_MM_S2=0;CHECK(!H30PrepareNodePath(path,5,120,180,true));
    NODE_PATH_ACCEL_MM_S2=4000;NODE_PATH_TURN_DEG_S=500;NODE_PATH_TURN_ACCEL_DEG_S2=720;
    CHECK(H30PrepareNodePath(path,5,5000,180,true));CHECK(route.config.accel==4000);
    status.state=DONE;scanPosePending=true;currentPose={1350,150,180};
    uint8_t scannedPath[]={2,1,0,9,10,19,20,21,22};
    CHECK(H30PrepareNodePath(scannedPath,9,300,180,true));
    CHECK(!scanPosePending&&currentPose.x==1350&&currentPose.y==150);
    CHECK(formalSettings.startOffset.x==150&&formalSettings.startOffset.y==-90);
    status.state=DONE;CHECK(H30PrepareNodePath(scannedPath,9,300,180,true));
    CHECK(formalSettings.startOffset.x==0&&formalSettings.startOffset.y==0&&currentPose.x==1200&&currentPose.y==240);
    CHECK(H30StartNodePath(path,5,300,180));CHECK(launches==1&&submitted.kind==MotionKind::NODE_PATH&&submitted.automatic);
    CHECK(submitted.pathCount==5&&submitted.path[4]==22&&submitted.speed==300&&submitted.theta==180);
    CHECK(H30RunNodePath(path,5,120,90));CHECK(launches==2&&!submitted.automatic&&submitted.theta==90);
    puts("formal preparation retains position policy, frozen parameters and scan offset; wrappers submit jobs");return 0;
}
