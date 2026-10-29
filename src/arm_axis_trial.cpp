#include "arm_axis_trial.h"
#include "chassis.h"
#include "Emm_V5.h"
#include "motor_bus.h"
#include "node_position_hardware.h"
#include <math.h>

namespace {
constexpr float pulsesPerRev=3200.0f; // 沿用正式动作的16细分；编码器累计计数为65536/圈。
constexpr uint32_t queryGapMs=5;
struct AxisTrace {
    bool active=false;
    uint8_t motor=0;
    uint32_t id=0,started=0,lastSample=0,period=0,duration=0;
    float scale=0,start=0,target=0,measured=0,overshoot=0;
    const char *error=nullptr;
};
AxisTrace trace;
uint32_t nextTrace=0;

const char *readState(uint8_t id,float scale,uint16_t &encoder,int64_t &counts,
    int16_t &rpm,uint8_t &flags,float &position) {
    MotorBusGuard transaction;
    vTaskDelay(pdMS_TO_TICKS(queryGapMs));
    const char *error=ReadMotorEncoderSnapshot(id,encoder,counts,rpm,flags);
    if(error)return error;
    // 两个直线轴均以CCW为正位移；保留符号，反向越过零点时应显示负位置。
    position=-static_cast<double>(counts)*pulsesPerRev/65536.0/scale;
    return isfinite(position)?nullptr:"MOTOR_POSITION";
}

void finish(const char *result) {
    Serial.printf("{EVT,AXIS,RESULT,%u,%lu,%lu,%s,%.3f,%.3f,%.3f}\n",trace.motor,
        (unsigned long)trace.id,(unsigned long)(millis()-trace.started),result,
        trace.target,trace.measured,trace.overshoot);
    trace.active=false;
}
}

const char *ArmAxisTrialStart(uint8_t id,float positionMm,uint16_t rpm,uint16_t rampMs,
    uint32_t periodMs,uint32_t durationMs) {
    if(trace.active)return "BUSY";
    trace.error=nullptr;
    if((id!=5&&id!=6)||!isfinite(positionMm)||positionMm<0
        ||positionMm>(id==5?ARM_HEIGHT_LIMIT_MM:170.0f)||!rpm||!periodMs||!durationMs)
        return trace.error="RANGE";
    const float scale=id==5?HEIGHT_PULSE:LENGTH_PULSE;
    if(!isfinite(scale)||scale<=0)return trace.error="CALIBRATION";
    const double pulses=round(static_cast<double>(positionMm)*scale);
    if(pulses>UINT32_MAX)return trace.error="RANGE";
    MotorBusGuard transaction;
    uint16_t encoder=0;int64_t counts=0;int16_t measuredRpm=0;uint8_t flags=0;
    float start=0;
    const char *error=readState(id,scale,encoder,counts,measuredRpm,flags,start);
    if(error)return trace.error=error; // 初始反馈失败时不发送运动命令。
    // 与7号试验同一换算：每增加1 RPM耗时(256-acc)*50us，协议档位范围1~255。
    const int step=static_cast<int>(rampMs*20.0f/rpm+0.5f);
    const uint8_t acc=rampMs==0?0:step<1?255:step>255?1:256-step;
    Emm_V5_Pos_Control(id,1,rpm,acc,static_cast<uint32_t>(pulses),true,false);
    trace=AxisTrace{};trace.active=true;trace.motor=id;trace.id=++nextTrace;
    trace.started=trace.lastSample=millis();trace.period=periodMs;trace.duration=durationMs;
    trace.scale=scale;trace.start=trace.measured=start;trace.target=positionMm;
    Serial.printf("{EVT,AXIS,COMMAND,%u,%lu,%lu,%.3f,%.3f,%u,%u}\n",id,
        (unsigned long)trace.id,(unsigned long)trace.started,positionMm,start,rpm,acc);
    return nullptr;
}

void ArmAxisTrialPoll() {
    if(!trace.active)return;
    const uint32_t now=millis();
    if(now-trace.lastSample<trace.period&&now-trace.started<trace.duration)return;
    trace.lastSample=now;
    uint16_t encoder=0;int64_t counts=0;int16_t rpm=0;uint8_t flags=0;float position=0;
    trace.error=readState(trace.motor,trace.scale,encoder,counts,rpm,flags,position);
    if(!trace.error) {
        trace.measured=position;
        const float passed=(trace.target>=trace.start?1.0f:-1.0f)*(position-trace.target);
        if(passed>trace.overshoot)trace.overshoot=passed;
        Serial.printf("{EVT,AXIS,SAMPLE,%u,%lu,%lu,%.3f,%.3f,%ld,%u,%.3f,%u,%lld,%lu}\n",
            trace.motor,(unsigned long)trace.id,(unsigned long)millis(),trace.target,position,
            (long)-static_cast<int32_t>(rpm),flags,trace.overshoot,encoder,(long long)counts,
            (unsigned long)(millis()-now));
    }
    // 到位也不提前结束。瞬时丢帧允许重试，但结束时仍无有效反馈则报告错误。
    if(millis()-trace.started>=trace.duration)finish(trace.error?trace.error:"OBSERVED");
}

bool ArmAxisTrialActive(){return trace.active;}
void ArmAxisTrialCancel(){if(trace.active){trace.error=nullptr;finish("CANCELLED");}}
const char *ArmAxisTrialError(){return trace.error;}
uint32_t ArmAxisTrialNextDelay() {
    if(!trace.active)return 20;
    const uint32_t elapsed=millis()-trace.started,sinceSample=millis()-trace.lastSample;
    if(elapsed>=trace.duration||sinceSample>=trace.period)return 1;
    const uint32_t sample=trace.period-sinceSample,remaining=trace.duration-elapsed;
    return sample<remaining?sample:remaining;
}
