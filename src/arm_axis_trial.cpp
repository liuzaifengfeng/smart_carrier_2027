// 单轴调试采样。
// 试验入口建立本次动作与采样时间轴，轮询读取实际编码器位置、转速及状态，并在试验完成或取消时结束采样。

#include "arm_axis_trial.h"  // 引入 arm_axis_trial.h 中的声明和定义。
#include "chassis.h"  // 引入 chassis.h 中的声明和定义。
#include "Emm_V5.h"  // 引入 Emm_V5.h 中的声明和定义。
#include "motor_bus.h"  // 引入 motor_bus.h 中的声明和定义。
#include "node_position_hardware.h"  // 引入 node_position_hardware.h 中的声明和定义。
#include <math.h>  // 引入 math.h 中的声明和定义。

namespace {  // 开始匿名命名空间，本文件内部共享其中的定义。
constexpr float pulsesPerRev=3200.0f; // 沿用正式动作的16细分；编码器累计计数为65536/圈。
constexpr uint32_t queryGapMs=5;  // 将 5 写入或初始化 queryGapMs（queryGap毫秒）。
struct AxisTrace {  // 定义 AxisTrace 的类型成员或作用域。
    bool active=false;  // 将 false 写入或初始化 active（活跃）。
    uint8_t motor=0;  // 将 0 写入或初始化 motor。
    uint32_t id=0,started=0,lastSample=0,period=0,duration=0;  // 将 0,started=0,lastSample=0,period=0,duration=0 写入或初始化 id（编号）。
    float scale=0,start=0,target=0,measured=0,overshoot=0;  // 将 0,start=0,target=0,measured=0,overshoot=0 写入或初始化 scale。
    const char *error=nullptr;  // 将 nullptr 写入或初始化 error（错误）。
};  // 结束此类型定义。
AxisTrace trace;  // 声明 trace，类型和可用范围由所在声明决定。
uint32_t nextTrace=0;  // 将 0 写入或初始化 nextTrace（下一Trace）。

const char *readState(uint8_t id,float scale,uint16_t &encoder,int64_t &counts,  // 定义函数或成员 readState（read状态），参数由此处的签名确定。
    int16_t &rpm,uint8_t &flags,float &position) {  // 声明 flags（标志位），类型和可用范围由所在声明决定。
    MotorBusGuard transaction;  // 声明 transaction，类型和可用范围由所在声明决定。
    vTaskDelay(pdMS_TO_TICKS(queryGapMs));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    const char *error=ReadMotorEncoderSnapshot(id,encoder,counts,rpm,flags);  // 将 ReadMotorEncoderSnapshot(id,encoder,counts,rpm,flags) 写入或初始化 error（错误）。
    if(error)return error;  // 若 error，立即返回 error。
    // 两个直线轴均以CCW为正位移；保留符号，反向越过零点时应显示负位置。
    position=-static_cast<double>(counts)*pulsesPerRev/65536.0/scale;  // 将 -static_cast<double>(counts)*pulsesPerRev/65536.0/scale 写入或初始化 position（位置）。
    return isfinite(position)?nullptr:"MOTOR_POSITION";  // 结束当前函数并返回 isfinite(position)?nullptr:"MOTOR_POSITION"。
}  // 结束此函数或控制块。

void finish(const char *result) {  // 定义函数或成员 的入口 finish，参数由此处的签名确定。
    Serial.printf("{EVT,AXIS,RESULT,%u,%lu,%lu,%s,%.3f,%.3f,%.3f}\n",trace.motor,  // 通过串口执行 printf，输出内容或参数由本行给出。
        (unsigned long)trace.id,(unsigned long)(millis()-trace.started),result,  // 调用 millis，传入 )-trace.started),result,。
        trace.target,trace.measured,trace.overshoot);  // 续写当前声明或运算，提供 trace.target,trace.measured,trace.overshoot);。
    trace.active=false;  // 将 false 写入或初始化 trace.active。
}  // 结束此函数或控制块。
}  // 结束命名空间。

const char *ArmAxisTrialStart(uint8_t id,float positionMm,uint16_t rpm,uint16_t rampMs,  // 定义函数或成员 ArmAxisTrialStart（机械臂AxisTrial起点），参数由此处的签名确定。
    uint32_t periodMs,uint32_t durationMs) {  // 声明 periodMs（period毫秒），类型和可用范围由所在声明决定。
    if(trace.active)return "BUSY";  // 若 trace.active，立即返回 "BUSY"。
    trace.error=nullptr;  // 将 nullptr 写入或初始化 trace.error。
    if((id!=5&&id!=6)||!isfinite(positionMm)||positionMm<0  // 当 (id!=5&&id!=6)||!isfinite(positionMm)||positionMm<0 成立时进入此分支，否则继续后续流程。
        ||positionMm>(id==5?ARM_HEIGHT_LIMIT_MM:170.0f)||!rpm||!periodMs||!durationMs)  // 继续上一行的组合条件，本行提供 ||positionMm>(id==5?ARM_HEIGHT_LIMIT_MM:170.0f)||!rpm||!periodMs||!durationMs)。
        return trace.error="RANGE";  // 结束当前函数并返回 trace.error="RANGE"。
    const float scale=id==5?HEIGHT_PULSE:LENGTH_PULSE;  // 将 id==5?HEIGHT_PULSE:LENGTH_PULSE 写入或初始化 scale。
    if(!isfinite(scale)||scale<=0)return trace.error="CALIBRATION";  // 若 !isfinite(scale)||scale<=0，立即返回 trace.error="CALIBRATION"。
    const double pulses=round(static_cast<double>(positionMm)*scale);  // 将 round(static_cast<double>(positionMm)*scale) 写入或初始化 pulses（脉冲数）。
    if(pulses>UINT32_MAX)return trace.error="RANGE";  // 若 pulses>UINT32_MAX，立即返回 trace.error="RANGE"。
    MotorBusGuard transaction;  // 声明 transaction，类型和可用范围由所在声明决定。
    uint16_t encoder=0;int64_t counts=0;int16_t measuredRpm=0;uint8_t flags=0;  // 将 0 写入或初始化 encoder（编码器）。然后将 0 写入或初始化 counts。然后将 0 写入或初始化 measuredRpm（measured转速）。然后将 0 写入或初始化 flags（标志位）。
    float start=0;  // 将 0 写入或初始化 start（起点）。
    const char *error=readState(id,scale,encoder,counts,measuredRpm,flags,start);  // 将 readState(id,scale,encoder,counts,measuredRpm,flags,st… 写入或初始化 error（错误）。
    if(error)return trace.error=error; // 初始反馈失败时不发送运动命令。
    // 与7号试验同一换算：每增加1 RPM耗时(256-acc)*50us，协议档位范围1~255。
    const int step=static_cast<int>(rampMs*20.0f/rpm+0.5f);  // 将 static_cast<int>(rampMs*20.0f/rpm+0.5f) 写入或初始化 step。
    const uint8_t acc=rampMs==0?0:step<1?255:step>255?1:256-step;  // 将 rampMs==0?0:step<1?255:step>255?1:256-step 写入或初始化 acc（加速度档位）。
    Emm_V5_Pos_Control(id,1,rpm,acc,static_cast<uint32_t>(pulses),true,false);  // 调用 Emm_V5_Pos_Control（EmmV5位置Control），传入 id,1,rpm,acc,static_cast<uint32_t>(pulses),true,false。
    trace=AxisTrace{};trace.active=true;trace.motor=id;trace.id=++nextTrace;  // 将 AxisTrace{};trace.active=true;trace.motor=id;trace.id=… 写入或初始化 trace。
    trace.started=trace.lastSample=millis();trace.period=periodMs;trace.duration=durationMs;  // 将 trace.lastSample=millis() 写入或初始化 trace.started。然后将 periodMs 写入或初始化 trace.period。然后将 durationMs 写入或初始化 trace.duration。
    trace.scale=scale;trace.start=trace.measured=start;trace.target=positionMm;  // 将 scale 写入或初始化 trace.scale。然后将 trace.measured=start 写入或初始化 trace.start。然后将 positionMm 写入或初始化 trace.target。
    Serial.printf("{EVT,AXIS,COMMAND,%u,%lu,%lu,%.3f,%.3f,%u,%u}\n",id,  // 通过串口执行 printf，输出内容或参数由本行给出。
        (unsigned long)trace.id,(unsigned long)trace.started,positionMm,start,rpm,acc);  // 为当前列表或跨行表达式提供这一项 (unsigned long)trace.id,(unsigned long)trace.started,p…。
    return nullptr;  // 结束当前函数并返回 nullptr。
}  // 结束此函数或控制块。

void ArmAxisTrialPoll() {  // 定义函数或成员 的入口 ArmAxisTrialPoll（机械臂AxisTrialPoll），参数由此处的签名确定。
    if(!trace.active)return;  // 当 !trace.active 成立时进入此分支，否则继续后续流程。
    const uint32_t now=millis();  // 将 millis() 写入或初始化 now。
    if(now-trace.lastSample<trace.period&&now-trace.started<trace.duration)return;  // 当 now-trace.lastSample<trace.period&&now-trace.started<t… 成立时进入此分支，否则继续后续流程。
    trace.lastSample=now;  // 将 now 写入或初始化 trace.lastSample。
    uint16_t encoder=0;int64_t counts=0;int16_t rpm=0;uint8_t flags=0;float position=0;  // 将 0 写入或初始化 encoder（编码器）。然后将 0 写入或初始化 counts。然后将 0 写入或初始化 rpm（转速）。然后将 0 写入或初始化 flags（标志位）。然后将 0 写入或初始化 position（位置）。
    trace.error=readState(trace.motor,trace.scale,encoder,counts,rpm,flags,position);  // 将 readState(trace.motor,trace.scale,encoder,counts,rpm,f… 写入或初始化 trace.error。
    if(!trace.error) {  // 当 !trace.error 成立时进入此分支，否则继续后续流程。
        trace.measured=position;  // 将 position 写入或初始化 trace.measured。
        const float passed=(trace.target>=trace.start?1.0f:-1.0f)*(position-trace.target);  // 将 (trace.target>=trace.start?1.0f:-1.0f)*(position-trace… 写入或初始化 passed。
        if(passed>trace.overshoot)trace.overshoot=passed;  // 当 passed>trace.overshoot 成立时进入此分支，否则继续后续流程。
        Serial.printf("{EVT,AXIS,SAMPLE,%u,%lu,%lu,%.3f,%.3f,%ld,%u,%.3f,%u,%lld,%lu}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
            trace.motor,(unsigned long)trace.id,(unsigned long)millis(),trace.target,position,  // 调用 millis，传入 ),trace.target,position,。
            (long)-static_cast<int32_t>(rpm),flags,trace.overshoot,encoder,(long long)counts,  // 调用 int32_t>，传入 rpm),flags,trace.overshoot,encoder,(long long)counts,。
            (unsigned long)(millis()-now));  // 调用 millis，传入 )-now)。
    }  // 结束此条件分支。
    // 到位也不提前结束。瞬时丢帧允许重试，但结束时仍无有效反馈则报告错误。
    if(millis()-trace.started>=trace.duration)finish(trace.error?trace.error:"OBSERVED");  // 当 millis()-trace.started>=trace.duration 成立时进入此分支，否则继续后续流程。
}  // 结束此函数或控制块。

bool ArmAxisTrialActive(){return trace.active;}  // 定义 ArmAxisTrialActive（机械臂AxisTrial活跃），调用时返回 trace.active。
void ArmAxisTrialCancel(){if(trace.active){trace.error=nullptr;finish("CANCELLED");}}  // 将 nullptr;finish("CANCELLED");}} 写入或初始化 trace.error。
const char *ArmAxisTrialError(){return trace.error;}  // 定义函数或成员 ArmAxisTrialError（机械臂AxisTrial错误），参数由此处的签名确定。
uint32_t ArmAxisTrialNextDelay() {  // 定义函数或成员 的入口 ArmAxisTrialNextDelay（机械臂AxisTrial下一延时），参数由此处的签名确定。
    if(!trace.active)return 20;  // 若 !trace.active，立即返回 20。
    const uint32_t elapsed=millis()-trace.started,sinceSample=millis()-trace.lastSample;  // 将 millis()-trace.started,sinceSample=millis()-trace.last… 写入或初始化 elapsed（已经过）。
    if(elapsed>=trace.duration||sinceSample>=trace.period)return 1;  // 若 elapsed>=trace.duration||sinceSample>=trace.period，立即返回 1。
    const uint32_t sample=trace.period-sinceSample,remaining=trace.duration-elapsed;  // 将 trace.period-sinceSample,remaining=trace.duration-elap… 写入或初始化 sample（采样）。
    return sample<remaining?sample:remaining;  // 结束当前函数并返回 sample<remaining?sample:remaining。
}  // 结束此函数或控制块。
