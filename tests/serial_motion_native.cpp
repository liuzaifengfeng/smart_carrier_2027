#include "serial_frame.h"
#include "motion_executor.h"
#include "motion_job.h"
#include "robot_state.h"
#include "chassis.h"
#include "material_transfer.h"
#include "continuous_trial.h"
#include "h30_usb.h"
#include "debug_release_button.h"
#include "disc_material_decision.h"
#include "task_code.h"
#include "led_pwm.h"
#include "node_route_heading.h"
#include <assert.h>
#include <float.h>
#include <vector>

NativeSerial Serial;
using TaskHandle_t=void *;
using TimerHandle_t=void *;
using portMUX_TYPE=int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
#define pdMS_TO_TICKS(x) (x)
constexpr int portMAX_DELAY=1,pdTRUE=1,pdFALSE=0,pdPASS=1,BOOT0_PIN=0,LOW=0;
uint32_t clockMs=100;
int inputLevel=1,taskCreations=0;
uint32_t millis(){return clockMs;}
int digitalRead(int){return inputLevel;}
int xAlignmentMotionMutex=1,xLidarPoseMutex=1,xAlignmentQueue=1,xVisualTaskQueue=1,taskCodeMux=0,nodePathMux=0;
int xSemaphoreTake(int,int){return pdTRUE;}
void xSemaphoreGive(int){}
void xQueueReset(int){}
template<class T>void xQueueOverwrite(int,T *){}
template<class T>void xQueueSend(int,T *,int){}
TaskHandle_t xTask_MainStateMachine_Handle=nullptr;
TimerHandle_t xHomeTimer=nullptr;
void Task_MainStateMachine(void *){}
void vHomeTimerCallback(TimerHandle_t){}
TimerHandle_t xTimerCreate(const char *,int,int,void *,void (*)(TimerHandle_t)){return reinterpret_cast<void *>(1);}
void xTimerDelete(TimerHandle_t,int){}
void xTimerStop(TimerHandle_t,int){}
int xTaskCreate(void (*)(void *),const char *,int,void *,int,TaskHandle_t *handle){++taskCreations;*handle=reinterpret_cast<void *>(1);return pdPASS;}
struct {void show(){}} FastLED;
struct CRGB {enum {Green=1};};int leds[1];
RobotState currentState=STATE_WAIT_START;
StartZone currentStartZone=START_ZONE_1;
volatile bool taskMotionAborted=false;
bool enableRun=false,nano_ready=false;
bool firstDiscGrabReady=false,discMessageResumeRequested=false,discMaterialActive=false,alignmentEnabled=false,alignmentStopPending=false;
uint8_t discMaterialColor=0;
int roundProgress=0;
uint32_t alignmentGeneration=0,alignmentFrameSequence=0;
struct VisualAlignmentFrame_t {float angleDeg,visualX,visualY;uint32_t sequence;};
struct VisualCmd_t {char cmd[20];float a,b,c;};
TaskCode currentTask{};bool taskReceived=false;
enum class NodePathState {IDLE,WAITING,RECEIVING,RECEIVED,RUNNING,DONE,FAILED};
NodePathState nodePathState=NodePathState::IDLE;
uint8_t nodePathBuffer[25];size_t nodePathLen=0;
constexpr uint8_t DISC_AREA_NODE=14,COARSE_AREA_NODE=10,TEMP_AREA_NODE=22;
constexpr float START_ZONE_HEADING=180;
float NODE_PATH_SPEED_MM_S=2500;
constexpr uint8_t SERVO_ID_BROADCAST=0xFE;
RobotPose currentPose{2250,150,180};ArmPose currentArm{};
CargoPlatform cargoPlatforms[3]={{MATERIAL_NONE},{MATERIAL_NONE},{MATERIAL_NONE}};
bool active=false,trialActive=false;
int submissions=0,launches=0,cancels=0,poseQueries=0,configQueries=0,calibrations=0;
MotionRequest captured;
MotionStatus result;
motion::LaunchGate gate;
uint32_t SubmitMotion(const MotionRequest &request,bool launch) {
    if(active)return 0;captured=request;active=true;++submissions;
    result=MotionStatus{};result.id=submissions;result.state=MotionState::QUEUED;result.phase="QUEUED";
    gate.prepare(result.id,launch);return result.id;
}
void LaunchMotion(uint32_t id) {
    assert(active&&id==result.id);
    assert(Serial.text.find("{RSP,")!=std::string::npos); // 实际分发函数已输出应答。
    gate.release(id);result.state=MotionState::RUNNING;++launches;
}
bool MotionExecutorActive(){return active;}
MotionStatus GetMotionStatus(uint32_t id){return id==result.id?result:MotionStatus{};}
bool WaitMotion(uint32_t id){return id&&result.state==MotionState::DONE;}
void CancelMotion(){++cancels;gate.cancel();active=false;result.state=MotionState::CANCELLED;}
void WakeMotionExecutor(){}
void requestVisionStop(VisionStartMode mode){if(mode==VisionStartMode::DISC_MATERIAL)discMaterialActive=false;else alignmentEnabled=false;}
void requestVisionStart(VisionStartMode mode){if(mode==VisionStartMode::DISC_MATERIAL)discMaterialActive=true;else alignmentEnabled=true;}
void setAlignmentEnabled(bool enabled,bool=false,bool=false,VisionStartMode=VisionStartMode::NONE,bool=false){alignmentEnabled=enabled;}
void ResetDiscAlignmentPid(){}
void ResetAlignmentWait(){}
void traceVisionFirstFrame(VisionStartMode){}
void traceVision(VisionStartMode,bool){}
bool updateDisplay(const char *,const char *,const char *){return true;}
void sendPoseQueryReply(){++poseQueries;Serial.println("{RSP,POSE,GET,OK,0,0,0,0,0,0,0}");}
void CalibrateH30MapHeading(float){++calibrations;}
const char *H30UsbGetAttitude(H30Attitude &,uint32_t &age){age=0;return nullptr;}
const char *H30UsbGetHeading(float &heading,uint32_t &age){heading=180;age=0;return nullptr;}
bool LedPwm_IsReady(){return true;}
bool LedPwm_SetBrightness(uint32_t){return true;}
uint8_t LedPwm_GetBrightness(){return 0;}
bool LedPwm_SetFrequency(uint32_t){return true;}
uint32_t LedPwm_GetFrequency(){return 8896;}
bool HandleRuntimeParameters(const char *const *,size_t,bool){++configQueries;return true;}
bool ContinuousTrialActive(){return trialActive;}
const char *ContinuousTrialBegin(const continuous::Config &){return nullptr;}
const char *ContinuousTrialPolicy(uint32_t,uint8_t,float,float,uint32_t){return nullptr;}
const char *ContinuousTrialNode(uint32_t,uint8_t,uint8_t,float,float,float){return nullptr;}
const char *ContinuousTrialPrepare(uint32_t){trialActive=true;return nullptr;}
void ContinuousTrialLaunch(){assert(Serial.text.find("TRIAL_START,ACK")!=std::string::npos);MotionRequest r;r.kind=MotionKind::TRIAL;uint32_t id=SubmitMotion(r,false);LaunchMotion(id);}
const char *ContinuousTrialStop(uint32_t){CancelMotion();trialActive=false;return nullptr;}
const char *ContinuousTrialStatus(uint32_t){Serial.println("{RSP,NAV,TRIAL_STATUS,OK}");return nullptr;}
bool busInitialized=false,serialInitialized=false;
int transactionDepth=0;
std::vector<int> startupCommands;
void MotorBusInit(){busInitialized=true;}
struct MotorBusGuard {MotorBusGuard(){++transactionDepth;}~MotorBusGuard(){--transactionDepth;}};
constexpr int SERIAL_8N1=0,SERIAL1_RXD_PIN=18,SERIAL1_TXD_PIN=17;
struct {void begin(int,int,int,int){serialInitialized=true;}void flush(){}} Serial1;
void recordCommand(int value){assert(busInitialized&&serialInitialized&&transactionDepth);startupCommands.push_back(value);}
void Emm_V5_Reset_Clog_Pro(uint8_t id){recordCommand(100+id);}
void Emm_V5_Reset_CurPos_To_Zero(uint8_t id){recordCommand(200+id);}
void Emm_V5_En_Control(uint8_t id,bool,bool){recordCommand(300+id);}
void Emm_V5_Origin_Trigger_Return(uint8_t id,uint8_t,bool){recordCommand(400+id);}
#include "motor_init_actual.inc"

// 编译实际运行时代码和完整串口分发；仅替换硬件及 FreeRTOS 边界。
#include "lidar_motion_actual.inc"
#include "serial_motion_actual.inc"

void dispatch(const char *command) {
    char text[160];strncpy(text,command,sizeof(text));text[159]=0;
    SerialFrame frame;assert(parseSerialFrame(text,frame));serialDispatch(frame);
}
void finish(bool success=true){active=false;result.state=success?MotionState::DONE:MotionState::FAILED;}
void grab() {
    serialDebugMode=false;currentState=STATE_GRAB_ROUND1;discMaterialActive=firstDiscGrabReady=true;
    currentTask=parseTaskCode("161+312+616+231");assert(currentTask.valid);
    dispatch("{CMD,VISION,GRAB,1,1}");assert(submissions==1&&launches==1&&active);
    assert(captured.kind==MotionKind::DISC_GRAB&&captured.automatic&&captured.color==1&&captured.cargo==1);
    assert(roundProgress==0&&cargoPlatforms[0].material==MATERIAL_NONE&&!firstDiscGrabReady);
    assert(Serial.text.find("{RSP,VISION,GRAB,ACK,1,1}")!=std::string::npos);
    dispatch("{CMD,POSE,GET}");dispatch("{CMD,CFG,GET,8}");dispatch("{CMD,MOTION,STATUS,1}");dispatch("{CMD,SYS,HELLO}");
    assert(poseQueries==1&&configQueries==1&&active&&submissions==1&&roundProgress==0);
    dispatch("{CMD,NAV,STOP}");assert(cancels==1&&!active&&result.state==MotionState::CANCELLED&&roundProgress==0);
}
void positions() {
    serialDebugMode=false;currentState=STATE_GRAB_ROUND2;discMaterialActive=firstDiscGrabReady=true;
    currentTask=parseTaskCode("161+312+616+231");
    dispatch("{CMD,VISION,DISC_POSITIONS,1,1,2,6,3,6}");
    assert(captured.position==2&&captured.color==6&&captured.cargo==1&&active);
    assert(!discMaterialActive&&!firstDiscGrabReady&&!roundProgress);
    const int submitted=submissions;dispatch("{CMD,VISION,DISC_POSITIONS,3,6}");assert(submitted==submissions);
    dispatch("{CMD,CHASSIS,MOVE,0,0,1}");assert(!active&&cancels==1&&!roundProgress);
}
void maintenance() {
    dispatch("{CMD,ARM,INIT_START}");assert(captured.kind==MotionKind::ARM_POSE&&active);
    dispatch("{CMD,IMU,GET}");dispatch("{CMD,POSE,GET}");dispatch("{CMD,CFG,GET,7}");assert(active&&poseQueries==1&&configQueries==1);
    dispatch("{CMD,ARM,DEMO1}");assert(submissions==1&&Serial.text.find("ERR,BUSY")!=std::string::npos);
    dispatch("{CMD,NAV,STOP}");finish();
    dispatch("{CMD,ARM,TRANSFER,2,3}");assert(captured.kind==MotionKind::TRANSFER&&captured.debug&&captured.cargo==2&&captured.work==3);
    assert(cargoPlatforms[1].material==MATERIAL_NONE);finish();
    dispatch("{CMD,ARM,RETRIEVE,3,2}");assert(captured.kind==MotionKind::RETRIEVE&&captured.color==MATERIAL_RED);finish();
    dispatch("{CMD,VISION,START_REQUEST,WORK_AREA_LOADED}");assert(captured.startVision&&!alignmentEnabled);
    assert(captured.vision==static_cast<uint8_t>(VisionStartMode::WORK_AREA_LOADED));
}
void lidar() {
    serialDebugMode=false;dispatch("{CMD,VISION,LIDAR_POSE}");
    assert(active&&submissions==1&&calibrations==1&&captured.kind==MotionKind::LIDAR_POSE);
    assert(SubmitLidarPoseAction(false,false)==result.id&&submissions==1); // 主流程接续同一作业。
    dispatch("{CMD,POSE,GET}");assert(poseQueries==1);
    finish();CompleteLidarPose(true,false);
    dispatch("{CMD,VISION,LIDAR_POSE}");assert(submissions==1&&calibrations==1);
    assert(Serial.text.find("LIDAR_POSE_DONE")!=std::string::npos);
}
void trial() {
    dispatch("{CMD,NAV,TRIAL_START,42}");assert(active&&trialActive&&captured.kind==MotionKind::TRIAL);
    dispatch("{CMD,NAV,TRIAL_STATUS,42}");dispatch("{CMD,POSE,GET}");assert(poseQueries==1&&active);
    dispatch("{CMD,NAV,TRIAL_STOP,42}");assert(!active&&!trialActive&&cancels==1);
}
void boot() {
    serialBootSetIdle(true);active=true;inputLevel=LOW;PollDebugBootButton();clockMs+=100;PollDebugBootButton();serialBootReleaseAtIdle(true);
    assert(serialDebugMode&&!taskCreations);
    inputLevel=1;clockMs+=100;PollDebugBootButton();active=false;serialBootSetIdle(true);clockMs+=100;PollDebugBootButton();
    inputLevel=LOW;clockMs+=100;PollDebugBootButton();clockMs+=100;PollDebugBootButton();serialBootReleaseAtIdle(true);
    assert(!serialDebugMode&&taskCreations==1&&!enableRun&&!taskMotionAborted);
}
void startup() {
    Emm_V5_Init();
    assert(startupCommands==std::vector<int>({100,201,202,203,204,305,405,306,406}));
    assert(!transactionDepth);
}
void zones() {
    dispatch("{CMD,NAV,START_ZONE,1}");assert(currentPose.x==2250&&currentPose.y==150&&currentPose.theta==180);
    dispatch("{CMD,NAV,START_ZONE,2}");assert(currentPose.x==150&&currentPose.y==150&&currentPose.theta==180);
    serialDebugMode=false;SubmitLidarPoseAction(false,false);
    assert(currentPose.x==150&&currentPose.y==150&&currentPose.theta==180&&captured.zone==2);
    const auto left=PlanNodeSegmentHeading(180,180,false,0),up=PlanNodeSegmentHeading(left.heading,90,false,0);
    assert(!left.reverse&&left.turn==0&&!up.reverse&&up.turn==-90);
}
int main(int argc,char **argv) {
    assert(argc==2);
    if(!strcmp(argv[1],"grab"))grab();else if(!strcmp(argv[1],"positions"))positions();
    else if(!strcmp(argv[1],"maintenance"))maintenance();else if(!strcmp(argv[1],"lidar"))lidar();
    else if(!strcmp(argv[1],"trial"))trial();else if(!strcmp(argv[1],"boot"))boot();
    else if(!strcmp(argv[1],"startup"))startup();else if(!strcmp(argv[1],"zones"))zones();else assert(false);
}
