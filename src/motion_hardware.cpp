#include "motion_hardware.h"
#include "motion_executor.h"
#include "robot_runtime.h"
#include "node_position_hardware.h"
#include "arm_placement_log.h"

using namespace motion;
namespace {
bool readServoAngle(uint8_t id,float &angle,uint32_t timeoutMs=50) {
    return Servo_QueryAngleMTurn(id,angle,timeoutMs)||Servo_QueryAngleMTurn(id,angle,timeoutMs);
}
const char *readExtension(float &length) {
    int64_t encoder=0;
    if(const char *error=ReadMotorEncoderPosition(6,encoder))return error;
    length=-static_cast<double>(encoder)*3200.0/65536.0/LENGTH_PULSE;
    return nullptr;
}
}
/**
 * @brief 读取系统毫秒计时。
 */
uint32_t MotionHardware::now(){return millis();}
/**
 * @brief 检查总任务及当前作业的取消状态。
 */
bool MotionHardware::cancelled(){return taskMotionAborted||(cancel&&cancel());}
void MotionHardware::stopTray() {
    trayPreselected=false;
    if(!trayMoving)return;
    Servo_HoldPosition(SERVO_ID_CARGO_TRAY);
    trayMoving=false;
}

void MotionHardware::stopRingTurret() {
    if(!ringTurretRunning)return;
    TurretMotorSetVelocity(0,ring::TURRET_CORRECTION_ACCELERATION);
    ringTurretRunning=false;
}

void MotionHardware::stopRingVision() {
    stopRingTurret();
    if(!ringVisionActive)return;
    ring::input().close();
    ringVisionActive=false;
    LedPwm_SetRingVisionActive(false);
    Serial.println("{EVT,VISION,STOP_REQUEST,RING_CORRECTION}");
}

void MotionHardware::preselectTray(float angle) {
    Step step;step.kind=StepKind::TRAY_START;step.turret=angle;
    if(issue(step))return;
    preselectedTrayAngle=angle;
    trayPreselected=true;
}
/**
 * @brief 按步骤类型调用 MoveArm、BeginPositionMove 等硬件入口；发令后立即返回。
 * @return nullptr 成功，否则返回执行错误。
 */
const char *MotionHardware::issue(const Step &s) {
    if(cancelled())return "STOPPED";
    switch(s.kind) {
    case StepKind::ARM:
        if(CargoTrayNeedsWork3Limit(s.trayWork3LimitCargo)) {
            trayPreselected=false;
            Servo_SetAngleMTurn(SERVO_ID_CARGO_TRAY,WORK_POSITION_3_TRAY_TARGET_DEG,0,0);
            trayMoving=true;
        }
        if(s.ringVisionPosition) {
            ringVisionActive=true;
            LedPwm_SetRingVisionActive(true);
            Serial.printf("{EVT,VISION,RING_TARGET,%u}\n",static_cast<unsigned>(s.ringVisionPosition));
            Serial.println("{EVT,VISION,START_REQUEST,RING_CORRECTION}");
        }
        if(!(s.planned?MoveArmPlanned(s.high,s.length,s.turret,s.pawl,s.speed,s.turretAcceleration,s.slowLower):
             MoveArm(s.high,s.length,s.turret,s.pawl,s.speed,s.turretAcceleration,s.slowLower,
                s.lengthAcceleration,s.slowLowerAcceleration)))return "EXECUTION";
        if(s.turretArrival)turretTarget=currentArm.turret_angle;
        if(CargoTrayNeedsClearance(s.trayClearanceCargo)) {
            trayPreselected=false;
            Servo_SetAngleMTurn(SERVO_ID_CARGO_TRAY,
                CargoTrayClearanceAngle(s.trayClearanceCargo),0,0);
            trayMoving=true;
        }
        return nullptr;
    case StepKind::RING_CORRECT: {
        float length=0,turret=0;
        if(const char *error=readExtension(length))return error;
        if(!ReadArmTurretAngle(turret))return "MOTOR_READ";
        const float margin=ring::TURRET_CORRECTION_CABLE_MARGIN_DEG;
        ringMinTurret=fmaxf(TURRET_CABLE_MIN_DEG+margin,turret-ring::MAX_TURRET_CORRECTION_DEG);
        ringMaxTurret=fminf(TURRET_CABLE_MAX_DEG-margin,turret+ring::MAX_TURRET_CORRECTION_DEG);
        // 伸缩仍用绝对位置指令，给脉冲向下取整留出余量。
        const float minLength=length-ring::MAX_EXTENSION_CORRECTION_MM;
        ringMinLength=minLength>0?minLength+1.0f/LENGTH_PULSE:0.0f;
        ringMaxLength=fminf(170.0f,length+ring::MAX_EXTENSION_CORRECTION_MM);
        ringX=ring::Pid{};
        ringY=ring::Pid{};
        ringHasLastFrame=false;
        ringSettings=ring::input().settings();
        ringDeadzone=ring::configuredDeadzone();
        ringTimeoutMs=ring::configuredTimeoutMs();
        ringStartedMs=now();
        Serial.printf("{EVT,VISION,RING_READY,%lu}\n",
            static_cast<unsigned long>(ring::input().ready(ring::requiredStableFrames(),ringDeadzone)));
        return nullptr;
    }
    case StepKind::POSITION: return BeginPositionMove(s.high,s.length,s.turret,s.speed,s.positionAcceleration)?nullptr:"EXECUTION";
    case StepKind::GOTO: GotoPose(s.high,s.length,s.turret,true);break;
    case StepKind::TURRET_READY: turretStarted=millis();break;
    case StepKind::COMMIT: cargoPlatforms[s.cargo-1].material=static_cast<MaterialType>(s.color);break;
    case StepKind::RAW_SERVO:
        if(s.cargo==SERVO_ID_CARGO_TRAY)trayPreselected=false;
        Servo_SetAngle(s.cargo,s.turret,0,0);break;
    case StepKind::TRAY_ROTATE:
        trayPreselected=false;
        Servo_SetAngleMTurn(SERVO_ID_CARGO_TRAY,s.turret,s.waitMs,s.trayPowerMw);
        trayMoving=true;
        break;
    case StepKind::TRAY_START:
        if(trayPreselected) {
            trayPreselected=false;
            if(s.turret==preselectedTrayAngle)break; // 首件沿用已下发的选盘，后续仍检查实测角度。
        }
        Servo_MoveCargoTray(s.turret);
        trayMoving=true;
        break;
    case StepKind::ENABLE: Emm_V5_En_Control_all(s.cargo);break;
    case StepKind::VELOCITY: MovePose(s.cargo,s.speed,s.color);break;
    default: break;
    }
    return nullptr;
}
/**
 * @brief 判断当前步骤是否使用反馈完成；POSITION、TURRET_READY 或显式开启的机械臂反馈模式使用反馈。
 */
bool MotionHardware::feedback(const Step &s) {
    return s.kind==StepKind::RING_CORRECT||s.kind==StepKind::POSITION||s.kind==StepKind::TURRET_READY||s.kind==StepKind::TRAY_ROTATE||s.kind==StepKind::TRAY_READY||
        (s.kind==StepKind::ARM&&(s.liftArrival||s.workPlaceLiftArrival||s.turretArrival||ARM_FEEDBACK_ENABLED)&&!s.timed);
}

uint32_t MotionHardware::feedbackDelay(const Step &s) {
    if(s.kind!=StepKind::RING_CORRECT)return SequenceIO::feedbackDelay(s);
    if(!ringTurretRunning)return 20;
    const int32_t remaining=static_cast<int32_t>(ringTurretStopMs-now());
    return remaining<=0?0:remaining<20?remaining:20;
}

const char *MotionHardware::correctRing(const Step &s,bool &done) {
    ring::Frame frame;
    const bool received=ring::input().take(frame,done);
    if(!done&&ringTimeoutMs&&now()-ringStartedMs>=ringTimeoutMs) {
        Serial.printf("{EVT,VISION,RING_TIMEOUT,%lu}\n",static_cast<unsigned long>(ringTimeoutMs));
        done=true;
    }
    if(done) {
        stopRingVision();
        return nullptr;
    }
    // 新帧重算速度，失帧时则在本次限时运行结束后停车。
    if(received||(ringTurretRunning&&static_cast<int32_t>(now()-ringTurretStopMs)>=0))stopRingTurret();
    if(!received)return nullptr;
    if(ring::withinDeadzone(frame,ringDeadzone)) {
        ringX=ring::Pid{};
        ringY=ring::Pid{};
        return nullptr;
    }
    const float dt=ringHasLastFrame?(frame.timeMs-ringLastFrameMs)*0.001f:0.0f;
    ringHasLastFrame=true;
    ringLastFrameMs=frame.timeMs;
    float targetLength=-1;
    if(ring::withinDeadzone(frame.y,ringDeadzone.y))ringY=ring::Pid{};
    else {
        float length=0;
        if(const char *error=readExtension(length))return error;
        targetLength=fmaxf(ringMinLength,fminf(ringMaxLength,length+ringY.update(frame.y,dt,ringSettings.extension)));
    }
    if(!MoveArm(-1,targetLength,-1,-1,s.speed,TURRET_ACCELERATION_SLOW))return "EXECUTION";
    if(ring::withinDeadzone(frame.x,ringDeadzone.x)) {ringX=ring::Pid{};return nullptr;}

    float turret=0;
    if(!ReadArmTurretAngle(turret))return "MOTOR_READ";
    currentArm.turret_angle=turret;
    const float correction=-ringX.update(frame.x,dt,ringSettings.turret);
    if(!isfinite(correction)) {
        ringX=ring::Pid{};
        return nullptr;
    }
    const float room=correction>0?ringMaxTurret-turret:turret-ringMinTurret;
    const float travel=fminf(fabsf(correction),room);
    if(travel<=0)return nullptr;

    // 保留 PID 的角度单位。转速向上取整，以缩短运行时间实现小于 1 RPM 的平均修正。
    const float rpmMs=travel*TURRET_MOTOR_GEAR_RATIO*1000.0f/6.0f;
    const auto rpm=static_cast<int16_t>(fminf(ceilf(rpmMs/ring::TURRET_CORRECTION_MAX_RUN_MS),
                                            ring::TURRET_CORRECTION_MAX_RPM));
    const uint32_t commandAt=now();
    const uint32_t age=commandAt-frame.timeMs;
    if(age>=ring::TURRET_CORRECTION_MAX_RUN_MS)return nullptr;
    uint32_t budget=ring::TURRET_CORRECTION_MAX_RUN_MS-age;
    if(ringTimeoutMs) {
        const uint32_t elapsed=commandAt-ringStartedMs;
        if(elapsed>=ringTimeoutMs)return nullptr;
        if(ringTimeoutMs-elapsed<budget)budget=ringTimeoutMs-elapsed;
    }
    const auto duration=static_cast<uint32_t>(fminf(rpmMs/rpm,budget));
    if(!duration)return nullptr;
    if(cancelled())return "STOPPED";
    TurretMotorSetVelocity(correction>0?rpm:-rpm,ring::TURRET_CORRECTION_ACCELERATION);
    ringTurretStopMs=commandAt+duration;
    ringTurretRunning=true;
    return nullptr;
}
/**
 * @brief 检查位置移动、转台或机械臂实测到位条件，并通过 done 返回本次结果。
 * @return nullptr 查询正常，否则返回错误原因。
 */
const char *MotionHardware::ready(const Step &s,bool &done) {
    done=false;if(cancelled())return "STOPPED";
    if(s.kind==StepKind::RING_CORRECT)return correctRing(s,done);
    if(s.kind==StepKind::POSITION){TickPositionMove();done=!PositionMoveActive();return nullptr;}
    if(s.kind==StepKind::TRAY_ROTATE||s.kind==StepKind::TRAY_READY) {
        float angle=0;
        if(!readServoAngle(SERVO_ID_CARGO_TRAY,angle))return "SERVO_READ";
        done=fabsf(angle-s.turret)<=s.trayArrivalToleranceDeg;
        if(done)trayMoving=false;
        return nullptr;
    }
    if(s.kind==StepKind::TURRET_READY) {
        float angle=0;
        if(ReadArmTurretAngle(angle)&&fabsf(angle-s.turret)<=5){done=true;return nullptr;}
        return millis()-turretStarted>=s.waitMs?"EXECUTION":nullptr;
    }
    if(s.liftArrival||s.workPlaceLiftArrival) {
        node_position::Motor state;
        const char *error=ReadMotorFeedback(5,state);if(error)return error;
        // 升降正方向为CCW，位置反馈为负；保留符号，不把反向位置当作抬升到位。
        const double measured=-static_cast<double>(state.position)*3200.0/65536.0/HEIGHT_PULSE;
        done=s.workPlaceLiftArrival ? fabs(measured-s.high)<10.0
            : (state.flags&2)&&fabs(measured-s.high)<=1.0;
        if(done&&s.lengthArrival) {
            if(const char *lengthError=ReadMotorFeedback(6,state))return lengthError;
            const double length=-static_cast<double>(state.position)*3200.0/65536.0/LENGTH_PULSE;
            done=(state.flags&2)&&fabs(length-s.length)<=1.0;
        }
        if(done)Serial.printf("{EVT,MOTION,LIFT_ARRIVED,%lu,%.3f,%.3f}\n",
            static_cast<unsigned long>(job),s.high,measured);
        return nullptr;
    }
    if(s.turretArrival) {
        float measured=0;if(!ReadArmTurretAngle(measured))return "EXECUTION";
        done=fabsf(measured-turretTarget)<=5.0f;
        if(done&&s.lengthArrival) {
            node_position::Motor state;
            const char *error=ReadMotorFeedback(6,state);if(error)return error;
            const double length=fabs(static_cast<double>(state.position))*3200/65536/LENGTH_PULSE;
            done=(state.flags&2)&&fabs(length-s.length)<=1.0;
        }
        if(done)Serial.printf("{EVT,MOTION,TURRET_ARRIVED,%lu,%.3f,%.3f}\n",
            static_cast<unsigned long>(job),turretTarget,measured);
        return nullptr;
    }
    // 反馈模式需在实车核验驱动器到位标志及编码器单位后显式开启。
    // currentArm 仅保存目标，只有下列实读反馈才能结束动作。
    for(uint8_t id=5;id<=6;++id) {
        const float target=id==5?s.high:s.length;if(target==-1)continue;
        node_position::Motor state;const char *error=ReadMotorFeedback(id,state);if(error)return error;
        const double measured=fabs(static_cast<double>(state.position))*3200/65536/(id==5?HEIGHT_PULSE:LENGTH_PULSE);
        if(!(state.flags&2)||fabs(measured-target)>1)return nullptr;
    }
    if(s.planned||s.turret!=-1) {
        float measured=0;if(!ReadArmTurretAngle(measured))return "EXECUTION";
        if(fabsf(measured-currentArm.turret_angle)>5)return nullptr;
    }
    if(s.pawl!=-1) {
        float measured=0;if(!readServoAngle(1,measured,100))return "EXECUTION";
        if(fabsf(continuous::turn(measured,s.pawl))>5)return nullptr;
    }
    done=true;return nullptr;
}
/**
 * @brief 输出当前作业的 STEP 开始/结束日志，index 为步骤下标。
 */
void MotionHardware::event(size_t index,const Step &step,bool begin) {
    if(!begin&&step.materialEvent) {
        char message[48];
        snprintf(message,sizeof(message),"%s %u",step.materialEvent,step.materialPosition);
        Serial.printf("{EVT,MATERIAL,%s,%u,%lu}\n",step.materialEvent,step.materialPosition,static_cast<unsigned long>(job));
        updateDisplay("DISPLAY","DEBUG",message);
    }
    if(!begin&&step.placementPosition)
        LogArmPlacementReadback(job,step.placementPosition,currentArm.turret_angle,currentArm.length);
    Serial.printf("{EVT,MOTION,STEP,%lu,%lu,%u,%s,%s}\n",static_cast<unsigned long>(job),
        static_cast<unsigned long>(millis()),static_cast<unsigned>(index),step.name,begin?"BEGIN":"END");
}
