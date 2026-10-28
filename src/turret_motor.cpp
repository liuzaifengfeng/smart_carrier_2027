#include "turret_motor.h"
#include "Emm_V5.h"
#include "node_position_hardware.h"
#include "motor_bus.h"
#include <Preferences.h>

namespace {
uint16_t zeroEncoder=0;
bool zeroLoaded=false;
float motorOriginAngle=0,lastMeasuredAngle=0;
struct TurnTrace {bool active=false;uint32_t id=0,started=0,lastSample=0;uint8_t stableSamples=0;float start=0,target=0,overshoot=0;uint32_t periodMs=50,durationMs=0;};
TurnTrace trace;
uint32_t nextTrace=0;
void finishTrace(const char *reason) {
    Serial.printf("{EVT,TURRET,RESULT,%lu,%lu,%s,%.3f,%.3f,%.3f}\n",
        static_cast<unsigned long>(trace.id),static_cast<unsigned long>(millis()-trace.started),
        reason,trace.target,lastMeasuredAngle,trace.overshoot);
    trace.active=false;
}

bool loadZero() {
    if(zeroLoaded)return true;
    Preferences store;
    if(!store.begin("turret",true))return false;
    zeroLoaded=store.isKey("zero_enc");
    if(zeroLoaded)zeroEncoder=store.getUShort("zero_enc");
    store.end();
    return zeroLoaded;
}
}


bool TurretMotorReadState(TurretMotorState &state) {
    MotorBusGuard transaction;
    // 经用户确认，先验证连续通信的帧间隔；持锁避免其他任务插入命令。
    vTaskDelay(pdMS_TO_TICKS(TURRET_QUERY_GAP_MS));
    if(!loadZero()) {
        Serial.println("[Turret] mechanical zero not calibrated; set ARM SERVO_ZERO 2");
        return false;
    }
    uint16_t encoder=0;
    int64_t position=0;int16_t rpm=0;uint8_t flags=0;
    const char *error=ReadMotorEncoderSnapshot(TURRET_MOTOR_ID,encoder,position,rpm,flags);
    if(error){Serial.printf("[Turret] angle read failed: %s\n",error);return false;}
    // 从同一帧反推出电机累计计数原点的单圈编码器位置。
    // 用户约定每次电机上电均位于机械0~180度，因此此原点的圈数唯一。
    const uint16_t originEncoder=static_cast<uint16_t>(int64_t(encoder)-position);
    const uint16_t originCounts=TURRET_MOTOR_POSITIVE_DIR==0
        ? static_cast<uint16_t>(originEncoder-zeroEncoder)
        : static_cast<uint16_t>(zeroEncoder-originEncoder);
    // 按已约定的上电0~180度选择以90度为中心的圈数分支，零位附近的小偏差不会变成360度。
    const int32_t absoluteCounts=originCounts>49152?int32_t(originCounts)-65536:originCounts;
    motorOriginAngle=absoluteCounts*360.0f/65536.0f/TURRET_MOTOR_GEAR_RATIO;
    const float sign=TURRET_MOTOR_POSITIVE_DIR==0?1.0f:-1.0f;
    state.angle=motorOriginAngle+static_cast<double>(position)*360.0/65536.0/TURRET_MOTOR_GEAR_RATIO*sign;
    state.rpm=TURRET_MOTOR_POSITIVE_DIR==0?rpm:-rpm;state.flags=flags;
    state.encoder=encoder;state.position=position;
    lastMeasuredAngle=state.angle;
    return true;
}

bool TurretMotorReadAngle(float &angle) {
    TurretMotorState state;if(!TurretMotorReadState(state))return false;
    angle=state.angle;return true;
}

static uint16_t motorRpm(float speedDegS) {
    const uint16_t requestedRpm = static_cast<uint16_t>(speedDegS * TURRET_MOTOR_GEAR_RATIO / 6.0f + 0.5f);
    return requestedRpm ? requestedRpm : 1; // 协议转速为整数RPM，最小非零档为1。
}

static bool moveProfile(float angle,uint16_t rpm,uint8_t acceleration,uint32_t periodMs,uint32_t durationMs) {
    const float motorAngle = (angle-motorOriginAngle) * TURRET_MOTOR_GEAR_RATIO;
    const uint32_t pulses = static_cast<uint32_t>(fabsf(motorAngle) * TURRET_MOTOR_PULSES_PER_REV / 360.0f + 0.5f);
    Emm_V5_Pos_Control(TURRET_MOTOR_ID, motorAngle < 0 ? 1 - TURRET_MOTOR_POSITIVE_DIR : TURRET_MOTOR_POSITIVE_DIR,
                       rpm, acceleration, pulses, true, false);
    if(trace.active)finishTrace("REPLACED");
    trace=TurnTrace{};trace.active=true;trace.id=++nextTrace;trace.started=trace.lastSample=millis();
    trace.start=lastMeasuredAngle;trace.target=angle;trace.periodMs=periodMs;trace.durationMs=durationMs;
    Serial.printf("{EVT,TURRET,COMMAND,%lu,%lu,%.3f,%.3f,%u,%u}\n",static_cast<unsigned long>(trace.id),
        static_cast<unsigned long>(trace.started),angle,trace.start,rpm,acceleration);
    return true;
}

bool TurretMotorMove(float angle,uint16_t /*rampMs*/) {
    return moveProfile(angle,motorRpm(TURRET_MOTOR_SPEED_DEG_S),TURRET_MOTOR_ACCELERATION,50,0);
}

bool TurretMotorTrial(float angle,float speedDegS,uint16_t rampMs,uint32_t periodMs,uint32_t durationMs) {
    MotorBusGuard transaction;
    TurretMotorState state;if(!TurretMotorReadState(state))return false;
    const uint16_t rpm=motorRpm(speedDegS);
    // 试验页继续按每增加1RPM耗时(256-acc)*50us换算档位。
    const int step=static_cast<int>(rampMs*20.0f/rpm+0.5f);
    const uint8_t acceleration=rampMs==0?0:step<1?255:step>255?1:256-step;
    return moveProfile(angle,rpm,acceleration,periodMs,durationMs);
}

uint32_t TurretMotorNextDelay() {
    if(!trace.active)return 20;
    const uint32_t elapsed=millis()-trace.lastSample;
    return elapsed>=trace.periodMs?1:trace.periodMs-elapsed;
}

bool TurretMotorSetZero(float &angle) {
    MotorBusGuard transaction;
    vTaskDelay(pdMS_TO_TICKS(TURRET_QUERY_GAP_MS));
    uint16_t encoder=0;int64_t position=0;int16_t rpm=0;uint8_t flags=0;
    if(ReadMotorEncoderSnapshot(TURRET_MOTOR_ID,encoder,position,rpm,flags))return false;
    if(ZeroMotorEncoderPosition(TURRET_MOTOR_ID))return false;
    Preferences store;
    if(!store.begin("turret",false)){Serial.println("[Turret] mechanical zero storage open failed");return false;}
    const bool saved=store.putUShort("zero_enc",encoder)==sizeof(encoder);
    store.end();
    if(!saved){Serial.println("[Turret] mechanical zero save failed");return false;}
    zeroEncoder=encoder;zeroLoaded=true;
    Serial.printf("[Turret] mechanical zero saved: encoder=%u\n",zeroEncoder);
    return TurretMotorReadAngle(angle);
}

bool TurretMotorTracking(){return trace.active;}
void TurretMotorCancelTrace(){if(trace.active)finishTrace("CANCELLED");}
void TurretMotorPoll() {
    if(!trace.active||millis()-trace.lastSample<trace.periodMs)return;
    trace.lastSample=millis();
    const uint32_t queryStart=millis();
    TurretMotorState state;if(!TurretMotorReadState(state)) {
        if(trace.durationMs&&millis()-trace.started>=trace.durationMs)finishTrace("OBSERVED");
        return;
    }
    const float error=state.angle-trace.target;
    const float passed=(trace.target>=trace.start?1.0f:-1.0f)*error;
    if(passed>trace.overshoot)trace.overshoot=passed;
    if(trace.durationMs) {
        Serial.printf("{EVT,TURRET,SAMPLE,%lu,%lu,%.3f,%.3f,%d,%u,%.3f,%u,%lld,%lu}\n",
            (unsigned long)trace.id,(unsigned long)millis(),trace.target,state.angle,state.rpm,state.flags,trace.overshoot,
            state.encoder,(long long)state.position,(unsigned long)(millis()-queryStart));
        if(millis()-trace.started>=trace.durationMs)finishTrace("OBSERVED");
        return;
    }
    Serial.printf("{EVT,TURRET,SAMPLE,%lu,%lu,%.3f,%.3f,%d,%u,%.3f}\n",(unsigned long)trace.id,
        (unsigned long)millis(),trace.target,state.angle,state.rpm,state.flags,trace.overshoot);
    // 仅结束后台记录，不参与动作放行；原有±5度放行及动作重叠保持不变。
    if((state.flags&2)&&state.rpm==0&&fabsf(error)<=1.0f)++trace.stableSamples;
    else trace.stableSamples=0;
    if(trace.stableSamples==3)finishTrace("SETTLED");
}
