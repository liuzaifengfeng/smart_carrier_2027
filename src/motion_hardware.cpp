#include "motion_hardware.h"
#include "motion_executor.h"
#include "robot_runtime.h"
#include "node_position_hardware.h"

using namespace motion;
/**
 * @brief 读取系统毫秒计时。
 */
uint32_t MotionHardware::now(){return millis();}
/**
 * @brief 检查总任务及当前作业的取消状态。
 */
bool MotionHardware::cancelled(){return taskMotionAborted||(cancel&&cancel());}
/**
 * @brief 按步骤类型调用 MoveArm、BeginPositionMove 等硬件入口；发令后立即返回。
 * @return nullptr 成功，否则返回执行错误。
 */
const char *MotionHardware::issue(const Step &s) {
    if(cancelled())return "STOPPED";
    switch(s.kind) {
    case StepKind::ARM:
        return (s.planned?MoveArmPlanned(s.high,s.length,s.turret,s.pawl,s.speed,s.turretRampMs,s.slowLower):
            MoveArm(s.high,s.length,s.turret,s.pawl,s.speed,s.turretRampMs,s.slowLower))?nullptr:"EXECUTION";
    case StepKind::POSITION: return BeginPositionMove(s.high,s.length,s.turret,s.speed)?nullptr:"EXECUTION";
    case StepKind::GOTO: GotoPose(s.high,s.length,s.turret,true);break;
    case StepKind::TURRET_READY: turretStarted=millis();break;
    case StepKind::COMMIT: cargoPlatforms[s.cargo-1].material=static_cast<MaterialType>(s.color);break;
    case StepKind::RAW_SERVO: Servo_SetAngle(s.cargo,s.turret,0,0);break;
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
    return s.kind==StepKind::POSITION||s.kind==StepKind::TURRET_READY||
        (s.kind==StepKind::ARM&&ARM_FEEDBACK_ENABLED&&!s.timed);
}
/**
 * @brief 检查位置移动、转台或机械臂实测到位条件，并通过 done 返回本次结果。
 * @return nullptr 查询正常，否则返回错误原因。
 */
const char *MotionHardware::ready(const Step &s,bool &done) {
    done=false;if(cancelled())return "STOPPED";
    if(s.kind==StepKind::POSITION){TickPositionMove();done=!PositionMoveActive();return nullptr;}
    if(s.kind==StepKind::TURRET_READY) {
        float angle=0;
        if(Servo_QueryAngleMTurn(2,angle,100)&&isfinite(angle)&&fabsf(angle-s.turret)<=5){done=true;return nullptr;}
        return millis()-turretStarted>=s.waitMs?"EXECUTION":nullptr;
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
        float measured=0;if(!Servo_QueryAngleMTurn(1,measured,100))return "EXECUTION";
        if(fabsf(continuous::turn(measured,s.pawl))>5)return nullptr;
    }
    done=true;return nullptr;
}
/**
 * @brief 输出当前作业的 STEP 开始/结束日志，index 为步骤下标。
 */
void MotionHardware::event(size_t index,const Step &step,bool begin) {
    Serial.printf("{EVT,MOTION,STEP,%lu,%lu,%u,%s,%s}\n",static_cast<unsigned long>(job),
        static_cast<unsigned long>(millis()),static_cast<unsigned>(index),step.name,begin?"BEGIN":"END");
}
