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
constexpr uint32_t AUTO_ALIGN_TIMEOUT_MS=15000,ALIGN_FEEDBACK_TIMEOUT_MS=300,ALIGN_LOG_INTERVAL_MS=500;
constexpr uint16_t ALIGN_PID_MAX_SPEED_RPM=50;
constexpr float DISC_AREA_HEADING=90;
void traceVision(VisionStartMode,bool){}
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
}
static void feed(float visualAngle=0,float x=0,float y=0){
    clockMs+=50;pending={visualAngle,x,y,pending.sequence+1};queued=true;TickVisualAlignment();
}
int main(){
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
    imuHeading=90;feed(160);assert(autoAlignmentState==AutoAlignmentState::DONE);
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
    for(auto mode:{VisionStartMode::WORK_AREA,VisionStartMode::WORK_AREA_LOADED,VisionStartMode::CORNER}){
        start(mode);imuError="OFFLINE";feed(10);assert(outputRotation<0&&imuReads==0);
        for(int i=0;i<5;++i)feed();assert(autoAlignmentState==AutoAlignmentState::DONE);
    }
    // 无视觉新帧不执行闭环或累计；停用和原有超时仍生效。
    start();for(int i=0;i<10;++i)TickVisualAlignment();assert(imuReads==0);
    alignmentEnabled=false;feed(100);assert(imuReads==0);
    start();clockMs+=AUTO_ALIGN_TIMEOUT_MS;TickVisualAlignment();
    assert(!alignmentEnabled&&autoAlignmentState==AutoAlignmentState::FAILED&&outputSpeed==0);
}
