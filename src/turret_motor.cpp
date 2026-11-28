// 机械臂转盘步进电机的角度与脉冲适配。
// 零点、编码器反馈、角度量化和试验采样在此统一管理。

#include "turret_motor.h"  // 引入 turret_motor.h 中的声明和定义。
#include "Emm_V5.h"  // 引入 Emm_V5.h 中的声明和定义。
#include "node_position_hardware.h"  // 引入 node_position_hardware.h 中的声明和定义。
#include "motor_bus.h"  // 引入 motor_bus.h 中的声明和定义。
#include <Preferences.h>  // 引入 Preferences.h 中的声明和定义。

namespace {  // 开始匿名命名空间，本文件内部共享其中的定义。
uint16_t zeroEncoder=0;  // 将 0 写入或初始化 zeroEncoder（零点编码器）。
bool zeroLoaded=false;  // 将 false 写入或初始化 zeroLoaded（零点Loaded）。
float motorOriginAngle=0,lastMeasuredAngle=0;  // 将 0,lastMeasuredAngle=0 写入或初始化 motorOriginAngle（motor原点角度）。
struct TurnTrace {bool active=false;uint32_t id=0,started=0,lastSample=0;uint8_t stableSamples=0;float start=0,target=0,overshoot=0;uint32_t periodMs=50,durationMs=0;};  // 定义 TurnTrace 的类型成员或作用域。
TurnTrace trace;  // 声明 trace，类型和可用范围由所在声明决定。
uint32_t nextTrace=0,lastCancelledTrace=0;  // 将 0,lastCancelledTrace=0 写入或初始化 nextTrace（下一Trace）。
void finishTrace(const char *reason) {  // 定义函数或成员 的入口 finishTrace，参数由此处的签名确定。
    Serial.printf("{EVT,TURRET,RESULT,%lu,%lu,%s,%.3f,%.3f,%.3f}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
        static_cast<unsigned long>(trace.id),static_cast<unsigned long>(millis()-trace.started),  // 定义函数或成员 long>，参数由此处的签名确定。
        reason,trace.target,lastMeasuredAngle,trace.overshoot);  // 续写当前声明或运算，提供 reason,trace.target,lastMeasuredAngle,trace.overshoot);。
    trace.active=false;  // 将 false 写入或初始化 trace.active。
}  // 结束此函数或控制块。

bool loadZero() {  // 定义函数或成员 的入口 loadZero（load零点），参数由此处的签名确定。
    if(zeroLoaded)return true;  // 若 zeroLoaded，立即返回 true。
    Preferences store;  // 声明 store，类型和可用范围由所在声明决定。
    if(!store.begin("turret",true))return false;  // 若 !store.begin("turret",true)，立即返回 false。
    zeroLoaded=store.isKey("zero_enc");  // 将 store.isKey("zero_enc") 写入或初始化 zeroLoaded（零点Loaded）。
    if(zeroLoaded)zeroEncoder=store.getUShort("zero_enc");  // 当 zeroLoaded 成立时进入此分支，否则继续后续流程。
    store.end();  // 调用 store.end。
    return zeroLoaded;  // 结束当前函数并返回 zeroLoaded。
}  // 结束此函数或控制块。
}  // 结束命名空间。


bool TurretMotorReadState(TurretMotorState &state) {  // 定义函数或成员 的入口 TurretMotorReadState（转台MotorRead状态），参数由此处的签名确定。
    MotorBusGuard transaction;  // 声明 transaction，类型和可用范围由所在声明决定。
    // 经用户确认，先验证连续通信的帧间隔；持锁避免其他任务插入命令。
    vTaskDelay(pdMS_TO_TICKS(TURRET_QUERY_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    if(!loadZero()) {  // 当 !loadZero() 成立时进入此分支，否则继续后续流程。
        Serial.println("[Turret] mechanical zero not calibrated; set ARM SERVO_ZERO 2");  // 调用 Serial.println，传入 "[Turret] mechanical zero not calibrated; set ARM SERV…。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。
    uint16_t encoder=0;  // 将 0 写入或初始化 encoder（编码器）。
    int64_t position=0;int16_t rpm=0;uint8_t flags=0;  // 将 0 写入或初始化 position（位置）。然后将 0 写入或初始化 rpm（转速）。然后将 0 写入或初始化 flags（标志位）。
    const char *error=ReadMotorEncoderSnapshot(TURRET_MOTOR_ID,encoder,position,rpm,flags);  // 将 ReadMotorEncoderSnapshot(TURRET_MOTOR_ID,encoder,posit… 写入或初始化 error（错误）。
    if(error){Serial.printf("[Turret] angle read failed: %s\n",error);return false;}  // 当 error 成立时进入此分支，否则继续后续流程。
    // 从同一帧反推出电机累计计数原点的单圈编码器位置。
    // 用户约定每次电机上电均位于机械0~180度，因此此原点的圈数唯一。
    const uint16_t originEncoder=static_cast<uint16_t>(int64_t(encoder)-position);  // 将 static_cast<uint16_t>(int64_t(encoder)-position) 写入或初始化 originEncoder（原点编码器）。
    const uint16_t originCounts=TURRET_MOTOR_POSITIVE_DIR==0  // 将 TURRET_MOTOR_POSITIVE_DIR==0 写入或初始化 originCounts（原点Counts）。
        ? static_cast<uint16_t>(originEncoder-zeroEncoder)  // 定义函数或成员 uint16_t>，参数由此处的签名确定。
        : static_cast<uint16_t>(zeroEncoder-originEncoder);  // 声明接口 uint16_t>，参数由此处的签名确定。
    // 按已约定的上电0~180度选择以90度为中心的圈数分支，零位附近的小偏差不会变成360度。
    const int32_t absoluteCounts=originCounts>49152?int32_t(originCounts)-65536:originCounts;  // 将 originCounts>49152?int32_t(originCounts)-65536:originC… 写入或初始化 absoluteCounts。
    motorOriginAngle=absoluteCounts*360.0f/65536.0f/TURRET_MOTOR_GEAR_RATIO;  // 将 absoluteCounts*360.0f/65536.0f/TURRET_MOTOR_GEAR_RATIO 写入或初始化 motorOriginAngle（motor原点角度）。
    const float sign=TURRET_MOTOR_POSITIVE_DIR==0?1.0f:-1.0f;  // 将 TURRET_MOTOR_POSITIVE_DIR==0?1.0f:-1.0f 写入或初始化 sign。
    state.angle=motorOriginAngle+static_cast<double>(position)*360.0/65536.0/TURRET_MOTOR_GEAR_RATIO*sign;  // 将 motorOriginAngle+static_cast<double>(position)*360.0/6… 写入或初始化 state.angle。
    state.rpm=TURRET_MOTOR_POSITIVE_DIR==0?rpm:-rpm;state.flags=flags;  // 将 TURRET_MOTOR_POSITIVE_DIR==0?rpm:-rpm 写入或初始化 state.rpm。然后将 flags 写入或初始化 state.flags。
    state.encoder=encoder;state.position=position;  // 将 encoder 写入或初始化 state.encoder。然后将 position 写入或初始化 state.position。
    lastMeasuredAngle=state.angle;  // 将 state.angle 写入或初始化 lastMeasuredAngle（最后Measured角度）。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

bool TurretMotorReadAngle(float &angle) {  // 定义函数或成员 的入口 TurretMotorReadAngle（转台MotorRead角度），参数由此处的签名确定。
    TurretMotorState state;if(!TurretMotorReadState(state))return false;  // 声明 state（状态），类型和可用范围由所在声明决定。然后若 !TurretMotorReadState(state)，立即返回 false。
    angle=state.angle;return true;  // 将 state.angle 写入或初始化 angle（角度）。然后结束当前函数并返回 true。
}  // 结束此函数或控制块。

static uint16_t motorRpm(float speedDegS) {  // 定义函数或成员 的入口 motorRpm（motor转速），参数由此处的签名确定。
    const uint16_t requestedRpm = static_cast<uint16_t>(speedDegS * TURRET_MOTOR_GEAR_RATIO / 6.0f + 0.5f);  // 将 static_cast<uint16_t>(speedDegS * TURRET_MOTOR_GEAR_RA… 写入或初始化 requestedRpm（requested转速）。
    return requestedRpm ? requestedRpm : 1; // 协议转速为整数RPM，最小非零档为1。
}  // 结束此函数或控制块。

TurretMotorPosition TurretMotorQuantizePosition(float angle) {  // 定义函数或成员 的入口 TurretMotorQuantizePosition（转台MotorQuantize位置），参数由此处的签名确定。
    // 将机构角度减去驱动器原点对应的机构角，再乘减速比得到电机角。
    // 协议方向位保存符号，脉冲字段保存四舍五入后的绝对值。
    const float motorAngle = (angle-motorOriginAngle) * TURRET_MOTOR_GEAR_RATIO;  // 将 (angle-motorOriginAngle) * TURRET_MOTOR_GEAR_RATIO 写入或初始化 motorAngle（motor角度）。
    const uint32_t pulses = static_cast<uint32_t>(fabsf(motorAngle) * TURRET_MOTOR_PULSES_PER_REV / 360.0f + 0.5f);  // 将 static_cast<uint32_t>(fabsf(motorAngle) * TURRET_MOTOR… 写入或初始化 pulses（脉冲数）。
    const uint8_t direction = motorAngle < 0 ? 1 - TURRET_MOTOR_POSITIVE_DIR : TURRET_MOTOR_POSITIVE_DIR;  // 将 motorAngle < 0 ? 1 - TURRET_MOTOR_POSITIVE_DIR : TURRE… 写入或初始化 direction（方向）。
    return {direction,pulses};  // 结束当前函数并返回 {direction,pulses}。
}  // 结束此函数或控制块。

static bool moveProfile(float angle,const TurretMotorPosition &position,uint16_t rpm,  // 定义函数或成员 moveProfile，参数由此处的签名确定。
                        uint8_t acceleration,uint32_t periodMs,uint32_t durationMs) {  // 声明 periodMs（period毫秒），类型和可用范围由所在声明决定。
    Emm_V5_Pos_Control(TURRET_MOTOR_ID,position.direction,rpm,acceleration,position.pulses,true,false);  // 调用 Emm_V5_Pos_Control（EmmV5位置Control），传入 TURRET_MOTOR_ID,position.direction,rpm,acceleration,po…。
    if(trace.active)finishTrace("REPLACED");  // 当 trace.active 成立时进入此分支，否则继续后续流程。
    trace=TurnTrace{};trace.active=true;trace.id=++nextTrace;trace.started=trace.lastSample=millis();  // 将 TurnTrace{};trace.active=true;trace.id=++nextTrace;tra… 写入或初始化 trace。
    trace.start=lastMeasuredAngle;trace.target=angle;trace.periodMs=periodMs;trace.durationMs=durationMs;  // 将 lastMeasuredAngle 写入或初始化 trace.start。然后将 angle 写入或初始化 trace.target。然后将 periodMs 写入或初始化 trace.periodMs。然后将 durationMs 写入或初始化 trace.durationMs。
    Serial.printf("{EVT,TURRET,COMMAND,%lu,%lu,%.3f,%.3f,%u,%u}\n",static_cast<unsigned long>(trace.id),  // 通过串口执行 printf，输出内容或参数由本行给出。
        static_cast<unsigned long>(trace.started),angle,trace.start,rpm,acceleration);  // 声明接口 long>，参数由此处的签名确定。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

bool TurretMotorMove(float angle,uint8_t acceleration,uint16_t rpm) {  // 定义函数或成员 的入口 TurretMotorMove（转台MotorMove），参数由此处的签名确定。
    return moveProfile(angle,TurretMotorQuantizePosition(angle),rpm,acceleration,50,0);  // 结束当前函数并返回 moveProfile(angle,TurretMotorQuantizePosition(angle),r…。
}  // 结束此函数或控制块。

bool TurretMotorMoveQuantized(float angle,const TurretMotorPosition &position,uint8_t acceleration,  // 定义函数或成员 TurretMotorMoveQuantized（转台MotorMoveQuantized），参数由此处的签名确定。
                             uint16_t rpm,uint32_t &commandId) {  // 声明 rpm（转速），类型和可用范围由所在声明决定。
    if(!moveProfile(angle,position,rpm,acceleration,50,0))return false;  // 若 !moveProfile(angle,position,rpm,acceleration,50,0)，立即返回 false。
    commandId=trace.id;  // 将 trace.id 写入或初始化 commandId（command编号）。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

bool TurretMotorTrial(float angle,float speedDegS,uint16_t rampMs,uint32_t periodMs,uint32_t durationMs) {  // 定义函数或成员 的入口 TurretMotorTrial（转台MotorTrial），参数由此处的签名确定。
    MotorBusGuard transaction;  // 声明 transaction，类型和可用范围由所在声明决定。
    TurretMotorState state;if(!TurretMotorReadState(state))return false;  // 声明 state（状态），类型和可用范围由所在声明决定。然后若 !TurretMotorReadState(state)，立即返回 false。
    const uint16_t rpm=motorRpm(speedDegS);  // 将 motorRpm(speedDegS) 写入或初始化 rpm（转速）。
    // 试验页继续按每增加1RPM耗时(256-acc)*50us换算档位。
    const int step=static_cast<int>(rampMs*20.0f/rpm+0.5f);  // 将 static_cast<int>(rampMs*20.0f/rpm+0.5f) 写入或初始化 step。
    const uint8_t acceleration=rampMs==0?0:step<1?255:step>255?1:256-step;  // 将 rampMs==0?0:step<1?255:step>255?1:256-step 写入或初始化 acceleration（加速度）。
    return moveProfile(angle,TurretMotorQuantizePosition(angle),rpm,acceleration,periodMs,durationMs);  // 结束当前函数并返回 moveProfile(angle,TurretMotorQuantizePosition(angle),r…。
}  // 结束此函数或控制块。

uint32_t TurretMotorNextDelay() {  // 定义函数或成员 的入口 TurretMotorNextDelay（转台Motor下一延时），参数由此处的签名确定。
    if(!trace.active)return 20;  // 若 !trace.active，立即返回 20。
    const uint32_t elapsed=millis()-trace.lastSample;  // 将 millis()-trace.lastSample 写入或初始化 elapsed（已经过）。
    return elapsed>=trace.periodMs?1:trace.periodMs-elapsed;  // 结束当前函数并返回 elapsed>=trace.periodMs?1:trace.periodMs-elapsed。
}  // 结束此函数或控制块。

bool TurretMotorSetZero(float &angle) {  // 定义函数或成员 的入口 TurretMotorSetZero（转台MotorSet零点），参数由此处的签名确定。
    MotorBusGuard transaction;  // 声明 transaction，类型和可用范围由所在声明决定。
    vTaskDelay(pdMS_TO_TICKS(TURRET_QUERY_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    uint16_t encoder=0;int64_t position=0;int16_t rpm=0;uint8_t flags=0;  // 将 0 写入或初始化 encoder（编码器）。然后将 0 写入或初始化 position（位置）。然后将 0 写入或初始化 rpm（转速）。然后将 0 写入或初始化 flags（标志位）。
    if(ReadMotorEncoderSnapshot(TURRET_MOTOR_ID,encoder,position,rpm,flags))return false;  // 若 ReadMotorEncoderSnapshot(TURRET_MOTOR_ID,encoder,posit…，立即返回 false。
    if(ZeroMotorEncoderPosition(TURRET_MOTOR_ID))return false;  // 若 ZeroMotorEncoderPosition(TURRET_MOTOR_ID)，立即返回 false。
    Preferences store;  // 声明 store，类型和可用范围由所在声明决定。
    if(!store.begin("turret",false)){Serial.println("[Turret] mechanical zero storage open failed");return false;}  // 当 !store.begin("turret",false) 成立时进入此分支，否则继续后续流程。
    const bool saved=store.putUShort("zero_enc",encoder)==sizeof(encoder);  // 将 store.putUShort("zero_enc",encoder)==sizeof(encoder) 写入或初始化 saved。
    store.end();  // 调用 store.end。
    if(!saved){Serial.println("[Turret] mechanical zero save failed");return false;}  // 当 !saved 成立时进入此分支，否则继续后续流程。
    zeroEncoder=encoder;zeroLoaded=true;  // 将 encoder 写入或初始化 zeroEncoder（零点编码器）。然后将 true 写入或初始化 zeroLoaded（零点Loaded）。
    Serial.printf("[Turret] mechanical zero saved: encoder=%u\n",zeroEncoder);  // 调用 Serial.printf，传入 "[Turret] mechanical zero saved: encoder=%u\n",zeroEnc…。
    return TurretMotorReadAngle(angle);  // 结束当前函数并返回 TurretMotorReadAngle(angle)。
}  // 结束此函数或控制块。

bool TurretMotorTracking(){return trace.active;}  // 定义 TurretMotorTracking（转台MotorTracking），调用时返回 trace.active。
bool TurretMotorCommandCancelled(uint32_t commandId){return commandId==lastCancelledTrace;}  // 定义函数或成员 TurretMotorCommandCancelled（转台MotorCommand已取消），参数由此处的签名确定。
bool TurretMotorCommandActive(uint32_t commandId){return trace.active&&trace.id==commandId;}  // 定义函数或成员 TurretMotorCommandActive（转台MotorCommand活跃），参数由此处的签名确定。
void TurretMotorCancelTrace(){  // 定义函数或成员 的入口 TurretMotorCancelTrace（转台MotorCancelTrace），参数由此处的签名确定。
    if(!trace.active)return;  // 当 !trace.active 成立时进入此分支，否则继续后续流程。
    lastCancelledTrace=trace.id;  // 将 trace.id 写入或初始化 lastCancelledTrace（最后已取消Trace）。
    finishTrace("CANCELLED");  // 调用 finishTrace，传入 "CANCELLED"。
}  // 结束此函数或控制块。
void TurretMotorPoll() {  // 定义函数或成员 的入口 TurretMotorPoll（转台MotorPoll），参数由此处的签名确定。
    if(!trace.active||millis()-trace.lastSample<trace.periodMs)return;  // 当 !trace.active||millis()-trace.lastSample<trace.periodMs 成立时进入此分支，否则继续后续流程。
    trace.lastSample=millis();  // 将 millis() 写入或初始化 trace.lastSample。
    const uint32_t queryStart=millis();  // 将 millis() 写入或初始化 queryStart（query起点）。
    TurretMotorState state;if(!TurretMotorReadState(state)) {  // 定义函数或成员 的入口 if，参数由此处的签名确定。
        if(trace.durationMs&&millis()-trace.started>=trace.durationMs)finishTrace("OBSERVED");  // 当 trace.durationMs&&millis()-trace.started>=trace.durati… 成立时进入此分支，否则继续后续流程。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此函数或控制块。
    const float error=state.angle-trace.target;  // 将 state.angle-trace.target 写入或初始化 error（错误）。
    const float passed=(trace.target>=trace.start?1.0f:-1.0f)*error;  // 将 (trace.target>=trace.start?1.0f:-1.0f)*error 写入或初始化 passed。
    if(passed>trace.overshoot)trace.overshoot=passed;  // 当 passed>trace.overshoot 成立时进入此分支，否则继续后续流程。
    if(trace.durationMs) {  // 当 trace.durationMs 成立时进入此分支，否则继续后续流程。
        Serial.printf("{EVT,TURRET,SAMPLE,%lu,%lu,%.3f,%.3f,%d,%u,%.3f,%u,%lld,%lu}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
            (unsigned long)trace.id,(unsigned long)millis(),trace.target,state.angle,state.rpm,state.flags,trace.overshoot,  // 调用 millis，传入 ),trace.target,state.angle,state.rpm,state.flags,trace…。
            state.encoder,(long long)state.position,(unsigned long)(millis()-queryStart));  // 调用 millis，传入 )-queryStart)。
        if(millis()-trace.started>=trace.durationMs)finishTrace("OBSERVED");  // 当 millis()-trace.started>=trace.durationMs 成立时进入此分支，否则继续后续流程。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    // 普通动作只记录指令及结果，保留内部采样用于计算超调和判断稳定。
    if((state.flags&2)&&state.rpm==0&&fabsf(error)<=1.0f)++trace.stableSamples;  // 当 (state.flags&2)&&state.rpm==0&&fabsf(error)<=1.0f 成立时进入此分支，否则继续后续流程。
    else trace.stableSamples=0;  // 前一个判断不成立时执行此分支。
    if(trace.stableSamples==3)finishTrace("SETTLED");  // 当 trace.stableSamples==3 成立时进入此分支，否则继续后续流程。
}  // 结束此函数或控制块。
