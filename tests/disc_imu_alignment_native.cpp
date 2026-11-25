#include "Arduino.h"
#include "vision_alignment.h"
#include <cassert>
#include <cmath>

NativeSerial Serial;
enum class AutoAlignmentState { IDLE, WAITING, DONE, FAILED };
struct VisualAlignmentFrame_t {float angleDeg,visualX,visualY;uint32_t sequence;};
static uint32_t clockMs=1000;
uint32_t millis(){return clockMs;}
static int xAlignmentMotionMutex=1,xAlignmentQueue=1;
static const int portMAX_DELAY=0,pdTRUE=1;
void xSemaphoreTake(int,int){}
void xSemaphoreGive(int){}
static VisualAlignmentFrame_t pending;
static bool queued=false;
int xQueueReceive(int,VisualAlignmentFrame_t *frame,int){if(!queued)return 0;*frame=pending;queued=false;return pdTRUE;}
static bool alignmentEnabled=true,alignmentStopPending=false;
static uint32_t alignmentGeneration=0,autoAlignmentStartedMs=1000;
static AutoAlignmentState autoAlignmentState=AutoAlignmentState::WAITING;
static VisionStartMode alignmentOwner=VisionStartMode::DISC;
static float alignmentTargetHeading=NAN;
static struct {float theta=0;} currentPose;
static int headingCalibrations=0;
static float calibratedHeading=0;
static const char *calibrationError=nullptr;
const char *H30UsbCalibrateHeading(float heading,bool keepReferenceOnFailure) {
    assert(keepReferenceOnFailure);++headingCalibrations;calibratedHeading=heading;
    return calibrationError;
}
static const char *visionNames[]={"DISC","DISC_MATERIAL","WORK_AREA","WORK_AREA_LOADED","CORNER"};
#include "disc_alignment_timing_actual.inc"
constexpr uint32_t ALIGN_FEEDBACK_TIMEOUT_MS=300,ALIGN_LOG_INTERVAL_MS=500;
constexpr uint16_t ALIGN_PID_MAX_SPEED_RPM=50;
constexpr float DISC_AREA_HEADING=90;
void traceVision(VisionStartMode,bool start,bool reportMissingFrame){assert(!start&&!reportMissingFrame);}
void traceVisionFirstFrame(VisionStartMode){}
void updateVisionLight(){}
static float imuHeading=90;
static const char *imuError=nullptr;
static int imuReads=0;
const char *H30UsbGetHeading(float &heading,uint32_t &age){++imuReads;heading=imuHeading;age=0;return imuError;}
static float outputX=0,outputY=0,outputRotation=0;
static uint16_t outputSpeed=0;
void OmniMove(float x,float y,float rotation,uint16_t speed){outputX=x;outputY=y;outputRotation=rotation;outputSpeed=speed;}
bool AlignToVisionContinuous(VisionStartMode,float,float,float,float,uint16_t);
#include "disc_pid_actual.inc"
#include "disc_tick_actual.inc"

static void start(VisionStartMode mode=VisionStartMode::DISC){
    ++alignmentGeneration;alignmentOwner=mode;alignmentEnabled=true;alignmentStopPending=false;
    autoAlignmentState=AutoAlignmentState::WAITING;autoAlignmentStartedMs=clockMs;
    imuHeading=90;imuError=nullptr;imuReads=0;queued=false;ResetDiscAlignmentPid();ResetAlignmentWait();
    alignmentTargetHeading=NAN;headingCalibrations=0;calibrationError=nullptr;
}
static void feed(float visualAngle=0,float x=0,float y=0){
    clockMs+=50;pending={visualAngle,x,y,pending.sequence+1};queued=true;TickVisualAlignment();
}
int main(){
    assert(!IsWithinAlignmentDeadzone({1,2,.1f},0,NAN,0));
    // 视觉角度可大幅摆动；DISC仅按IMU角差转向，保留50RPM比例和视觉XY。
    start();imuHeading=89;feed(120,40,-30);
    assert(outputRotation>0&&outputX>0&&outputY<0&&outputSpeed==50);
    imuHeading=91;feed(-120,40,-30);assert(outputRotation<0);
    // 超过360度的差采用最短路径；避免359度读数被误当作正269度。
    start();imuHeading=359;feed(0);assert(outputRotation>0);
    start();imuHeading=450;feed(120);assert(outputRotation==0);
    // 完成角度条件也必须取IMU；视觉angle=0不能掩盖真实偏航。
    start();imuHeading=92;for(int i=0;i<6;++i)feed(0);
    assert(autoAlignmentState==AutoAlignmentState::WAITING);
    Serial.text.clear();
    imuHeading=90;feed(160);assert(autoAlignmentState==AutoAlignmentState::DONE);
    assert(Serial.text.find("{EVT,VISION,ALIGN_RESULT,DISC,ALIGNED,1,0.000,0.000,0.000}")!=std::string::npos);
    assert(Serial.text.find("{EVT,VISION,ALIGN_DONE}")!=std::string::npos);
    // XY越界仍阻止完成，并维持对应方向的视觉运动。
    start();for(int i=0;i<6;++i)feed(0,40,-30);
    assert(autoAlignmentState==AutoAlignmentState::WAITING&&outputX>0&&outputY<0&&outputRotation==0);
    // 各种IMU错误不得退回视觉角度或沿用旧旋转，恢复后的首个合格帧即可完成。
    for(const char *error:{"OFFLINE","STALE","NOT_CALIBRATED"}){
        start();imuError=error;feed(100,40,-30);
        assert(outputRotation==0&&outputX>0&&outputY<0&&outputSpeed==50);
        assert(autoAlignmentState==AutoAlignmentState::WAITING);
        for(int i=0;i<6;++i)feed(100);
        assert(autoAlignmentState==AutoAlignmentState::WAITING);
        imuError=nullptr;feed(100);assert(autoAlignmentState==AutoAlignmentState::DONE);
    }
    // 旧无模式入口仍映射DISC；其他模式保留视觉来源且不依赖H30。
    start(VisionStartMode::NONE);imuHeading=89;feed(100);assert(outputRotation>0&&imuReads==1);
    start(VisionStartMode::WORK_AREA);imuError="OFFLINE";feed(10);assert(outputRotation<0&&imuReads==0);
    for(int i=0;i<4;++i)feed(0,1,-1);
    assert(autoAlignmentState==AutoAlignmentState::WAITING);
    feed(0,1.01f,0); // 超过恢复后的 X 死区，应清零。
    for(int i=0;i<4;++i)feed(0,-1,1);
    assert(autoAlignmentState==AutoAlignmentState::WAITING);
    feed(0,1,-1);
    assert(autoAlignmentState==AutoAlignmentState::DONE);
    for(auto mode:{VisionStartMode::WORK_AREA_LOADED,VisionStartMode::CORNER}){
        start(mode);imuError="OFFLINE";feed(10);assert(outputRotation<0&&imuReads==0);
        feed();assert(autoAlignmentState==AutoAlignmentState::WAITING);
        feed();assert(autoAlignmentState==AutoAlignmentState::DONE);
    }
    // 5秒超时必须停车，只有新鲜且接近的误差可以继续。
    start();for(int i=0;i<10;++i)TickVisualAlignment();assert(imuReads==0);
    alignmentEnabled=false;feed(100);assert(imuReads==0);
    for(auto mode:{VisionStartMode::DISC,VisionStartMode::WORK_AREA,VisionStartMode::WORK_AREA_LOADED,VisionStartMode::CORNER}){
        start(mode);feed(10,40,-30);
        assert(outputSpeed==50);
        Serial.text.clear();
        clockMs=autoAlignmentStartedMs+4999;TickVisualAlignment();
        assert(alignmentEnabled&&autoAlignmentState==AutoAlignmentState::WAITING&&outputSpeed==50);
        ++clockMs;TickVisualAlignment();
        assert(!alignmentEnabled&&autoAlignmentState==AutoAlignmentState::FAILED&&outputSpeed==0);
        const std::string result=std::string("{EVT,VISION,ALIGN_RESULT,")
            +visionNames[static_cast<uint8_t>(mode)]+",TIMEOUT_STALE,1,"
            +(mode==VisionStartMode::DISC?"0.000":"10.000")+",40.000,-30.000}";
        assert(Serial.text.find(result)!=std::string::npos);
        assert(Serial.text.find("{EVT,VISION,ALIGN_DONE}")==std::string::npos);
        Serial.text.clear();feed(10,40,-30);
        assert(outputSpeed==0&&Serial.text.empty());
    }
    start();Serial.text.clear();clockMs+=5000;TickVisualAlignment();
    assert(!alignmentEnabled&&autoAlignmentState==AutoAlignmentState::FAILED&&imuReads==0);
    assert(Serial.text.find("{EVT,VISION,ALIGN_RESULT,DISC,TIMEOUT_NO_FRAME,0,NA,NA,NA}")!=std::string::npos);

    start();imuHeading=89;feed(120,40,-30);
    imuError="OFFLINE";clockMs=autoAlignmentStartedMs+4900;feed(100,12,-34);
    Serial.text.clear();clockMs=autoAlignmentStartedMs+5000;TickVisualAlignment();
    assert(!alignmentEnabled&&autoAlignmentState==AutoAlignmentState::FAILED&&outputSpeed==0);
    assert(Serial.text.find("{EVT,VISION,ALIGN_RESULT,DISC,TIMEOUT_ERROR,1,NA,12.000,-34.000}")!=std::string::npos);

    for(float idealHeading:{270.0f,180.0f}) {
        start(VisionStartMode::WORK_AREA);alignmentTargetHeading=idealHeading;currentPose.theta=idealHeading+4;
        feed(10);assert(headingCalibrations==0);
        for(int i=0;i<6;++i)feed();
        assert(headingCalibrations==1&&calibratedHeading==idealHeading&&currentPose.theta==idealHeading);
        assert(autoAlignmentState==AutoAlignmentState::DONE);
    }
    for(auto mode:{VisionStartMode::WORK_AREA,VisionStartMode::WORK_AREA_LOADED}) {
        start(mode);alignmentTargetHeading=180;currentPose.theta=184;
        clockMs=autoAlignmentStartedMs+4900;feed(0,mode==VisionStartMode::WORK_AREA?1.25f:3.25f,0);
        clockMs=autoAlignmentStartedMs+5000;TickVisualAlignment();
        const auto expected=mode==VisionStartMode::WORK_AREA?AutoAlignmentState::DONE:AutoAlignmentState::FAILED;
        assert(autoAlignmentState==expected&&headingCalibrations==0&&currentPose.theta==184);
    }

    start(VisionStartMode::WORK_AREA_LOADED);alignmentTargetHeading=180;calibrationError="STALE";
    for(int i=0;i<6;++i)feed();
    assert(autoAlignmentState==AutoAlignmentState::DONE&&headingCalibrations==1&&currentPose.theta==184);
}
