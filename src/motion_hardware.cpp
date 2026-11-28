// 将动作步骤映射到物理设备。
// issue 发出命令，ready 轮询动作条件，event 记录时间线。
// 圆环对齐需要关联当前视觉会话与步骤状态。

#include "motion_hardware.h"  // 引入 motion_hardware.h 中的声明和定义。
#include "motion_executor.h"  // 引入 motion_executor.h 中的声明和定义。
#include "robot_runtime.h"  // 引入 robot_runtime.h 中的声明和定义。
#include "node_position_hardware.h"  // 引入 node_position_hardware.h 中的声明和定义。
#include "arm_placement_log.h"  // 引入 arm_placement_log.h 中的声明和定义。

using namespace motion;  // 引入名称或类型别名 namespace motion。
namespace {  // 开始匿名命名空间，本文件内部共享其中的定义。
constexpr float turretPulseDegrees=360.0f/(TURRET_MOTOR_GEAR_RATIO*TURRET_MOTOR_PULSES_PER_REV);  // 将 360.0f/(TURRET_MOTOR_GEAR_RATIO*TURRET_MOTOR_PULSES_PE… 写入或初始化 turretPulseDegrees（转台脉冲Degrees）。

bool readServoAngle(uint8_t id,float &angle,uint32_t timeoutMs=50) {  // 定义函数或成员 的入口 readServoAngle（readServo角度），参数由此处的签名确定。
    return Servo_QueryAngleMTurn(id,angle,timeoutMs)||Servo_QueryAngleMTurn(id,angle,timeoutMs);  // 结束当前函数并返回 Servo_QueryAngleMTurn(id,angle,timeoutMs)||Servo_Query…。
}  // 结束此初始化列表或局部作用域。
const char *readExtension(float &length) {  // 定义函数或成员 的入口 readExtension（read伸缩），参数由此处的签名确定。
    int64_t encoder=0;  // 将 0 写入或初始化 encoder（编码器）。
    if(const char *error=ReadMotorEncoderPosition(6,encoder))return error;  // 若 const char *error=ReadMotorEncoderPosition(6,encoder)，立即返回 error。
    length=-static_cast<double>(encoder)*3200.0/65536.0/LENGTH_PULSE;  // 将 -static_cast<double>(encoder)*3200.0/65536.0/LENGTH_PU… 写入或初始化 length（长度）。
    return nullptr;  // 结束当前函数并返回 nullptr。
}  // 结束此函数或控制块。
}  // 结束命名空间。
/**
 * @brief 读取系统毫秒计时。
 */
uint32_t MotionHardware::now(){return millis();}  // 定义函数或成员 MotionHardware::now，参数由此处的签名确定。
/**
 * @brief 检查总任务及当前作业的取消状态。
 */
bool MotionHardware::cancelled(){return taskMotionAborted||(cancel&&cancel());}  // 定义函数或成员 MotionHardware::cancelled（MotionHardware已取消），参数由此处的签名确定。
void MotionHardware::stopTray() {  // 定义函数或成员 的入口 MotionHardware::stopTray（MotionHardware停止料盘），参数由此处的签名确定。
    trayPreselected=false;  // 将 false 写入或初始化 trayPreselected（料盘Preselected）。
    if(!trayMoving)return;  // 当 !trayMoving 成立时进入此分支，否则继续后续流程。
    Servo_HoldPosition(SERVO_ID_CARGO_TRAY);  // 调用 Servo_HoldPosition（Servo保持位置），传入 SERVO_ID_CARGO_TRAY。
    trayMoving=false;  // 将 false 写入或初始化 trayMoving（料盘Moving）。
}  // 结束此函数或控制块。

void MotionHardware::stopRingVision() {  // 定义函数或成员 的入口 MotionHardware::stopRingVision（MotionHardware停止圆环视觉），参数由此处的签名确定。
    lastRingXCommand={};  // 将 {} 写入或初始化 lastRingXCommand（最后圆环横向Command）。
    if(!ringVisionActive)return;  // 当 !ringVisionActive 成立时进入此分支，否则继续后续流程。
    ring::input().close();  // 调用 ring::input（圆环输入），传入 ).close(。
    ringVisionActive=false;  // 将 false 写入或初始化 ringVisionActive（圆环视觉活跃）。
    LedPwm_SetRingVisionActive(false);  // 调用 LedPwm_SetRingVisionActive（Led占空比输出Set圆环视觉活跃），传入 false。
    Serial.println("{EVT,VISION,STOP_REQUEST,RING_CORRECTION}");  // 调用 Serial.println，传入 "{EVT,VISION,STOP_REQUEST,RING_CORRECTION}"。
}  // 结束此函数或控制块。

void MotionHardware::preselectTray(float angle) {  // 定义函数或成员 的入口 MotionHardware::preselectTray（MotionHardwarepreselect料盘），参数由此处的签名确定。
    Step step;step.kind=StepKind::TRAY_START;step.turret=angle;  // 声明 step，类型和可用范围由所在声明决定。然后将 StepKind::TRAY_START 写入或初始化 step.kind。然后将 angle 写入或初始化 step.turret。
    if(issue(step))return;  // 当 issue(step) 成立时进入此分支，否则继续后续流程。
    preselectedTrayAngle=angle;  // 将 angle 写入或初始化 preselectedTrayAngle（preselected料盘角度）。
    trayPreselected=true;  // 将 true 写入或初始化 trayPreselected（料盘Preselected）。
}  // 结束此函数或控制块。
/**
 * @brief 按步骤类型调用 MoveArm、BeginPositionMove 等硬件入口；发令后立即返回。
 * @return nullptr 成功，否则返回执行错误。
 */
const char *MotionHardware::issue(const Step &s) {  // 定义函数或成员 的入口 MotionHardware::issue，参数由此处的签名确定。
    if(cancelled())return "STOPPED";  // 若 cancelled()，立即返回 "STOPPED"。
    switch(s.kind) {  // 按 s.kind 的值选择对应分支。
    case StepKind::ARM:  // 当前值匹配 StepKind::ARM 时从这里执行。
        if(CargoTrayNeedsWork3Limit(s.trayWork3LimitCargo)) {  // 当 CargoTrayNeedsWork3Limit(s.trayWork3LimitCargo) 成立时进入此分支，否则继续后续流程。
            trayPreselected=false;  // 将 false 写入或初始化 trayPreselected（料盘Preselected）。
            Servo_SetAngleMTurn(SERVO_ID_CARGO_TRAY,WORK_POSITION_3_TRAY_TARGET_DEG,0,0);  // 调用 Servo_SetAngleMTurn（ServoSet角度MTurn），传入 SERVO_ID_CARGO_TRAY,WORK_POSITION_3_TRAY_TARGET_DEG,0,0。
            trayMoving=true;  // 将 true 写入或初始化 trayMoving（料盘Moving）。
        }  // 结束此条件分支。
        if(s.ringVisionPosition) {  // 当 s.ringVisionPosition 成立时进入此分支，否则继续后续流程。
            ringVisionActive=true;  // 将 true 写入或初始化 ringVisionActive（圆环视觉活跃）。
            LedPwm_SetRingVisionActive(true);  // 调用 LedPwm_SetRingVisionActive（Led占空比输出Set圆环视觉活跃），传入 true。
            Serial.printf("{EVT,VISION,RING_TARGET,%u}\n",static_cast<unsigned>(s.ringVisionPosition));  // 调用 Serial.printf，传入 "{EVT,VISION,RING_TARGET,%u}\n",static_cast<unsigned>(…。
            Serial.println("{EVT,VISION,START_REQUEST,RING_CORRECTION}");  // 调用 Serial.println，传入 "{EVT,VISION,START_REQUEST,RING_CORRECTION}"。
        }  // 结束此条件分支。
        if(!(s.planned?MoveArmPlanned(s.high,s.length,s.turret,s.pawl,s.speed,s.turretAcceleration,s.slowLower):  // 当 !(s.planned?MoveArmPlanned(s.high,s.length,s.turret,s.… 成立时进入此分支，否则继续后续流程。
             MoveArm(s.high,s.length,s.turret,s.pawl,s.speed,s.turretAcceleration,s.slowLower,  // 调用 MoveArm（Move机械臂），传入 s.high,s.length,s.turret,s.pawl,s.speed,s.turretAccele…。
                s.lengthAcceleration,s.slowLowerAcceleration)))return "EXECUTION";  // 续写当前声明或运算，提供 s.lengthAcceleration,s.slowLowerAcceleration)))return …。
        if(s.turretArrival)turretTarget=currentArm.turret_angle;  // 当 s.turretArrival 成立时进入此分支，否则继续后续流程。
        if(CargoTrayNeedsClearance(s.trayClearanceCargo)) {  // 当 CargoTrayNeedsClearance(s.trayClearanceCargo) 成立时进入此分支，否则继续后续流程。
            trayPreselected=false;  // 将 false 写入或初始化 trayPreselected（料盘Preselected）。
            Servo_SetAngleMTurn(SERVO_ID_CARGO_TRAY,  // 调用 Servo_SetAngleMTurn（ServoSet角度MTurn），传入 SERVO_ID_CARGO_TRAY,。
                CargoTrayClearanceAngle(s.trayClearanceCargo),0,0);  // 调用 CargoTrayClearanceAngle（货位料盘Clearance角度），传入 s.trayClearanceCargo),0,0。
            trayMoving=true;  // 将 true 写入或初始化 trayMoving（料盘Moving）。
        }  // 结束此条件分支。
        return nullptr;  // 结束当前函数并返回 nullptr。
    case StepKind::RING_CORRECT: {  // 当前值匹配 StepKind::RING_CORRECT 时从这里执行。
        float length=0,turret=0;  // 将 0,turret=0 写入或初始化 length（长度）。
        if(const char *error=readExtension(length))return error;  // 若 const char *error=readExtension(length)，立即返回 error。
        if(!ReadArmTurretAngle(turret))return "MOTOR_READ";  // 若 !ReadArmTurretAngle(turret)，立即返回 "MOTOR_READ"。
        // 转台四舍五入，伸缩脉冲向下取整。给脉冲换算留出向内的余量。
        constexpr float halfTurretPulse=turretPulseDegrees*.5f;  // 将 turretPulseDegrees*.5f 写入或初始化 halfTurretPulse（half转台脉冲）。
        ringMinTurret=fmaxf(TURRET_CABLE_MIN_DEG,turret-ring::MAX_TURRET_CORRECTION_DEG)+halfTurretPulse;  // 将 fmaxf(TURRET_CABLE_MIN_DEG,turret-ring::MAX_TURRET_COR… 写入或初始化 ringMinTurret（圆环最小转台）。
        ringMaxTurret=fminf(TURRET_CABLE_MAX_DEG,turret+ring::MAX_TURRET_CORRECTION_DEG)-halfTurretPulse;  // 将 fminf(TURRET_CABLE_MAX_DEG,turret+ring::MAX_TURRET_COR… 写入或初始化 ringMaxTurret（圆环最大转台）。
        const float minLength=length-ring::MAX_EXTENSION_CORRECTION_MM;  // 将 length-ring::MAX_EXTENSION_CORRECTION_MM 写入或初始化 minLength（最小长度）。
        ringMinLength=minLength>0?minLength+1.0f/LENGTH_PULSE:0.0f;  // 将 minLength>0?minLength+1.0f/LENGTH_PULSE:0.0f 写入或初始化 ringMinLength（圆环最小长度）。
        ringMaxLength=fminf(170.0f,length+ring::MAX_EXTENSION_CORRECTION_MM);  // 将 fminf(170.0f,length+ring::MAX_EXTENSION_CORRECTION_MM) 写入或初始化 ringMaxLength（圆环最大长度）。
        ringX=ring::Pid{};  // 将 ring::Pid{} 写入或初始化 ringX（圆环横向）。
        ringY=ring::Pid{};  // 将 ring::Pid{} 写入或初始化 ringY（圆环纵向）。
        lastRingXCommand={};  // 将 {} 写入或初始化 lastRingXCommand（最后圆环横向Command）。
        ringHasLastFrame=false;  // 将 false 写入或初始化 ringHasLastFrame（圆环Has最后帧）。
        ringSettings=ring::input().settings();  // 将 ring::input().settings() 写入或初始化 ringSettings（圆环设置）。
        ringDeadzone=ring::configuredDeadzone();  // 将 ring::configuredDeadzone() 写入或初始化 ringDeadzone（圆环Deadzone）。
        ringTimeoutMs=ring::configuredTimeoutMs();  // 将 ring::configuredTimeoutMs() 写入或初始化 ringTimeoutMs（圆环超时毫秒）。
        ringStartedMs=now();  // 将 now() 写入或初始化 ringStartedMs（圆环Started毫秒）。
        Serial.printf("{EVT,VISION,RING_READY,%lu}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
            static_cast<unsigned long>(ring::input().ready(ring::requiredStableFrames(),ringDeadzone)));  // 声明接口 long>，参数由此处的签名确定。
        return nullptr;  // 结束当前函数并返回 nullptr。
    }  // 结束此条件分支。
    case StepKind::POSITION: return BeginPositionMove(s.high,s.length,s.turret,s.speed,s.positionAcceleration)?nullptr:"EXECUTION";  // 当前值匹配 StepKind::POSITION 时从这里执行。
    case StepKind::GOTO: GotoPose(s.high,s.length,s.turret,true);break;  // 当前值匹配 StepKind::GOTO 时从这里执行。
    case StepKind::TURRET_READY: turretStarted=millis();break;  // 当前值匹配 StepKind::TURRET_READY 时从这里执行。
    case StepKind::COMMIT: cargoPlatforms[s.cargo-1].material=static_cast<MaterialType>(s.color);break;  // 当前值匹配 StepKind::COMMIT 时从这里执行。
    case StepKind::RAW_SERVO:  // 当前值匹配 StepKind::RAW_SERVO 时从这里执行。
        if(s.cargo==SERVO_ID_CARGO_TRAY)trayPreselected=false;  // 当 s.cargo==SERVO_ID_CARGO_TRAY 成立时进入此分支，否则继续后续流程。
        Servo_SetAngle(s.cargo,s.turret,0,0);break;  // 调用 Servo_SetAngle（ServoSet角度），传入 s.cargo,s.turret,0,0。然后退出当前循环或 switch 分支。
    case StepKind::TRAY_ROTATE:  // 当前值匹配 StepKind::TRAY_ROTATE 时从这里执行。
        trayPreselected=false;  // 将 false 写入或初始化 trayPreselected（料盘Preselected）。
        Servo_SetAngleMTurn(SERVO_ID_CARGO_TRAY,s.turret,s.waitMs,s.trayPowerMw);  // 调用 Servo_SetAngleMTurn（ServoSet角度MTurn），传入 SERVO_ID_CARGO_TRAY,s.turret,s.waitMs,s.trayPowerMw。
        trayMoving=true;  // 将 true 写入或初始化 trayMoving（料盘Moving）。
        break;  // 退出当前循环或 switch 分支。
    case StepKind::TRAY_START:  // 当前值匹配 StepKind::TRAY_START 时从这里执行。
        if(trayPreselected) {  // 当 trayPreselected 成立时进入此分支，否则继续后续流程。
            trayPreselected=false;  // 将 false 写入或初始化 trayPreselected（料盘Preselected）。
            if(s.turret==preselectedTrayAngle)break; // 首件沿用已下发的选盘，后续仍检查实测角度。
        }  // 结束此条件分支。
        Servo_MoveCargoTray(s.turret);  // 调用 Servo_MoveCargoTray（ServoMove货位料盘），传入 s.turret。
        trayMoving=true;  // 将 true 写入或初始化 trayMoving（料盘Moving）。
        break;  // 退出当前循环或 switch 分支。
    case StepKind::ENABLE: Emm_V5_En_Control_all(s.cargo);break;  // 当前值匹配 StepKind::ENABLE 时从这里执行。
    case StepKind::VELOCITY: MovePose(s.cargo,s.speed,s.color);break;  // 当前值匹配 StepKind::VELOCITY 时从这里执行。
    default: break;  // 没有匹配的 case 时走此默认分支。
    }  // 结束此函数或控制块。
    return nullptr;  // 结束当前函数并返回 nullptr。
}  // 结束此函数或控制块。
/**
 * @brief 判断当前步骤是否使用反馈完成；POSITION、TURRET_READY 或显式开启的机械臂反馈模式使用反馈。
 */
bool MotionHardware::feedback(const Step &s) {  // 定义函数或成员 的入口 MotionHardware::feedback（MotionHardware反馈），参数由此处的签名确定。
    return s.kind==StepKind::RING_CORRECT||s.kind==StepKind::POSITION||s.kind==StepKind::TURRET_READY||s.kind==StepKind::TRAY_ROTATE||s.kind==StepKind::TRAY_READY||  // 结束当前函数并返回 s.kind==StepKind::RING_CORRECT||s.kind==StepKind::POSI…。
        (s.kind==StepKind::ARM&&(s.liftArrival||s.workPlaceLiftArrival||s.turretArrival||ARM_FEEDBACK_ENABLED)&&!s.timed);  // 继续上一行的组合条件，本行提供 (s.kind==StepKind::ARM&&(s.liftArrival||s.workPlaceLiftArrival||s.turretArrival||ARM_FEED…。
}  // 结束此函数或控制块。

const char *MotionHardware::correctRing(const Step &s,bool &done) {  // 定义函数或成员 的入口 MotionHardware::correctRing（MotionHardwarecorrect圆环），参数由此处的签名确定。
    ring::Frame frame;  // 声明 frame（帧），类型和可用范围由所在声明决定。
    // 视觉输入将是否取到新帧与是否达到结束条件分开返回。
    // 超时时这一步也会结束，并在返回前停掉本次圆环视觉会话。
    const bool received=ring::input().take(frame,done);  // 将 ring::input().take(frame,done) 写入或初始化 received（已接收）。
    if(!done&&ringTimeoutMs&&now()-ringStartedMs>=ringTimeoutMs) {  // 当 !done&&ringTimeoutMs&&now()-ringStartedMs>=ringTimeout… 成立时进入此分支，否则继续后续流程。
        Serial.printf("{EVT,VISION,RING_TIMEOUT,%lu}\n",static_cast<unsigned long>(ringTimeoutMs));  // 调用 Serial.printf，传入 "{EVT,VISION,RING_TIMEOUT,%lu}\n",static_cast<unsigned…。
        done=true;  // 将 true 写入或初始化 done（完成）。
    }  // 结束此条件分支。
    if(done) {  // 当 done 成立时进入此分支，否则继续后续流程。
        stopRingVision();  // 调用 stopRingVision（停止圆环视觉），使用无参形式推进相应操作。
        return nullptr;  // 结束当前函数并返回 nullptr。
    }  // 结束此条件分支。
    if(!received)return nullptr;  // 若 !received，立即返回 nullptr。
    if(ring::withinDeadzone(frame,ringDeadzone)) {  // 当 ring::withinDeadzone(frame,ringDeadzone) 成立时进入此分支，否则继续后续流程。
        ringX=ring::Pid{};  // 将 ring::Pid{} 写入或初始化 ringX（圆环横向）。
        ringY=ring::Pid{};  // 将 ring::Pid{} 写入或初始化 ringY（圆环纵向）。
        return nullptr;  // 结束当前函数并返回 nullptr。
    }  // 结束此条件分支。
    const float dt=ringHasLastFrame?(frame.timeMs-ringLastFrameMs)*0.001f:0.0f;  // 将 ringHasLastFrame?(frame.timeMs-ringLastFrameMs)*0.001f… 写入或初始化 dt（时间差）。
    ringHasLastFrame=true;  // 将 true 写入或初始化 ringHasLastFrame（圆环Has最后帧）。
    ringLastFrameMs=frame.timeMs;  // 将 frame.timeMs 写入或初始化 ringLastFrameMs（圆环最后帧毫秒）。
    // Y 误差用于伸缩轴，X 误差用于转台。
    // 轴处于死区时重置该方向的 PID，其中 -1 表示本步保留伸缩目标。
    float targetLength=-1;  // 将 -1 写入或初始化 targetLength（目标长度）。
    if(ring::withinDeadzone(frame.y,ringDeadzone.y))ringY=ring::Pid{};  // 当 ring::withinDeadzone(frame.y,ringDeadzone.y) 成立时进入此分支，否则继续后续流程。
    else {  // 前一个判断不成立时执行此分支。
        float length=0;  // 将 0 写入或初始化 length（长度）。
        if(const char *error=readExtension(length))return error;  // 若 const char *error=readExtension(length)，立即返回 error。
        targetLength=fmaxf(ringMinLength,fminf(ringMaxLength,length+ringY.update(frame.y,dt,ringSettings.extension)));  // 将 fmaxf(ringMinLength,fminf(ringMaxLength,length+ringY.u… 写入或初始化 targetLength（目标长度）。
    }  // 结束此条件分支。
    if(ring::withinDeadzone(frame.x,ringDeadzone.x)) {  // 当 ring::withinDeadzone(frame.x,ringDeadzone.x) 成立时进入此分支，否则继续后续流程。
        ringX=ring::Pid{};  // 将 ring::Pid{} 写入或初始化 ringX（圆环横向）。
        return MoveArm(-1,targetLength,-1,-1,s.speed,TURRET_ACCELERATION_SLOW)?nullptr:"EXECUTION";  // 结束当前函数并返回 MoveArm(-1,targetLength,-1,-1,s.speed,TURRET_ACCELERAT…。
    }  // 结束此条件分支。
    float turret=0;  // 将 0 写入或初始化 turret（转台）。
    if(!ReadArmTurretAngle(turret))return "MOTOR_READ";  // 若 !ReadArmTurretAngle(turret)，立即返回 "MOTOR_READ"。
    const float correction=-ringX.update(frame.x,dt,ringSettings.turret);  // 将 -ringX.update(frame.x,dt,ringSettings.turret) 写入或初始化 correction（修正量）。
    // 保留原规划发令前的实测范围检查和 Y 轴先下发的顺序。
    if(!ReadArmTurretAngle(turret)||!MoveArm(-1,targetLength,-1,-1,s.speed,TURRET_ACCELERATION_SLOW))  // 当 !ReadArmTurretAngle(turret)||!MoveArm(-1,targetLength,… 成立时进入此分支，否则继续后续流程。
        return "EXECUTION";  // 结束当前函数并返回 "EXECUTION"。
    if(correction==0)return nullptr;  // 若 correction==0，立即返回 nullptr。
    float targetTurret=fmaxf(ringMinTurret,fminf(ringMaxTurret,turret+correction));  // 将 fmaxf(ringMinTurret,fminf(ringMaxTurret,turret+correct… 写入或初始化 targetTurret（目标转台）。
    auto position=TurretMotorQuantizePosition(targetTurret);  // 将 TurretMotorQuantizePosition(targetTurret) 写入或初始化 position（位置）。
    const auto current=TurretMotorQuantizePosition(turret);  // 将 TurretMotorQuantizePosition(turret) 写入或初始化 current（当前）。
    const auto signedPulses=[](const TurretMotorPosition &p) -> int64_t {  // 将 [](const TurretMotorPosition &p) -> int64_t { 写入或初始化 signedPulses（signed脉冲数）。
        return p.direction==TURRET_MOTOR_POSITIVE_DIR?int64_t(p.pulses):-int64_t(p.pulses);  // 结束当前函数并返回 p.direction==TURRET_MOTOR_POSITIVE_DIR?int64_t(p.pulse…。
    };  // 结束此初始化列表或局部作用域。
    if(signedPulses(position)==signedPulses(current)) {  // 当 signedPulses(position)==signedPulses(current) 成立时进入此分支，否则继续后续流程。
        // 非零 PID 输出量化后若没有位移，就沿修正方向补一个脉冲。
        const bool positive=correction>0;  // 将 correction>0 写入或初始化 positive。
        const int64_t next=signedPulses(current)+(positive?1:-1);  // 将 signedPulses(current)+(positive?1:-1) 写入或初始化 next（下一）。
        const float limit=positive?ringMaxTurret:ringMinTurret;  // 将 positive?ringMaxTurret:ringMinTurret 写入或初始化 limit（限值）。
        const int64_t limitPulses=signedPulses(TurretMotorQuantizePosition(limit));  // 将 signedPulses(TurretMotorQuantizePosition(limit)) 写入或初始化 limitPulses（限值脉冲数）。
        if(positive?next>limitPulses:next<limitPulses)return nullptr;  // 若 positive?next>limitPulses:next<limitPulses，立即返回 nullptr。
        position={static_cast<uint8_t>(next<0?1-TURRET_MOTOR_POSITIVE_DIR:TURRET_MOTOR_POSITIVE_DIR),  // 将 {static_cast<uint8_t>(next<0?1-TURRET_MOTOR_POSITIVE_D… 写入或初始化 position（位置）。
                  static_cast<uint32_t>(next<0?-next:next)};  // 声明接口 uint32_t>，参数由此处的签名确定。
        targetTurret=fmaxf(ringMinTurret,fminf(ringMaxTurret,turret+copysignf(turretPulseDegrees,correction)));  // 将 fmaxf(ringMinTurret,fminf(ringMaxTurret,turret+copysig… 写入或初始化 targetTurret（目标转台）。
    }  // 结束此条件分支。
    // 细调阶段引用上一次转台命令的编号、下发时间和量化位置。
    // 被撤销的指令先清除，接着以时间间隔和同目标指令的活动状态判断本帧是否发令。
    if(lastRingXCommand.id&&TurretMotorCommandCancelled(lastRingXCommand.id))lastRingXCommand={};  // 当 lastRingXCommand.id&&TurretMotorCommandCancelled(lastR… 成立时进入此分支，否则继续后续流程。
    if(fabsf(frame.x)<=ring::RING_X_FINE_PX&&lastRingXCommand.id) {  // 当 fabsf(frame.x)<=ring::RING_X_FINE_PX&&lastRingXCommand… 成立时进入此分支，否则继续后续流程。
        if(now()-lastRingXCommand.sentAtMs<ring::RING_X_FINE_SEND_MS||  // 当 now()-lastRingXCommand.sentAtMs<ring::RING_X_FINE_SEND… 成立时进入此分支，否则继续后续流程。
           (signedPulses(position)==signedPulses(lastRingXCommand.position)&&  // 继续上一行的组合条件，本行提供 (signedPulses(position)==signedPulses(lastRingXCommand.position)&&。
            TurretMotorCommandActive(lastRingXCommand.id)))return nullptr;  // 继续上一行的组合条件，本行提供 TurretMotorCommandActive(lastRingXCommand.id)))return nullptr;。
    }  // 结束此条件分支。
    uint32_t commandId=0;  // 将 0 写入或初始化 commandId（command编号）。
    if(!TurretMotorMoveQuantized(targetTurret,position,ring::TURRET_CORRECTION_ACCELERATION,  // 当 !TurretMotorMoveQuantized(targetTurret,position,ring::… 成立时进入此分支，否则继续后续流程。
                                ring::TURRET_CORRECTION_RPM,commandId)) {  // 续写当前声明或运算，提供 ring::TURRET_CORRECTION_RPM,commandId)) {。
        lastRingXCommand={};  // 将 {} 写入或初始化 lastRingXCommand（最后圆环横向Command）。
        return "EXECUTION";  // 结束当前函数并返回 "EXECUTION"。
    }  // 结束此函数或控制块。
    currentArm.turret_angle=targetTurret;  // 将 targetTurret 写入或初始化 currentArm.turret_angle。
    lastRingXCommand.id=commandId;  // 将 commandId 写入或初始化 lastRingXCommand.id。
    lastRingXCommand.sentAtMs=now();  // 将 now() 写入或初始化 lastRingXCommand.sentAtMs。
    lastRingXCommand.position=position;  // 将 position 写入或初始化 lastRingXCommand.position。
    return taskMotionAborted?"EXECUTION":nullptr;  // 结束当前函数并返回 taskMotionAborted?"EXECUTION":nullptr。
}  // 结束此函数或控制块。
/**
 * @brief 检查位置移动、转台或机械臂实测到位条件，并通过 done 返回本次结果。
 * @return nullptr 查询正常，否则返回错误原因。
 */
// 轮询阶段可根据步骤类型检查不同设备。
// 返回值描述查询是否出错，done 表示查询正常时的到位结果，两者需要一起判断。
const char *MotionHardware::ready(const Step &s,bool &done) {  // 定义函数或成员 的入口 MotionHardware::ready（MotionHardware就绪），参数由此处的签名确定。
    done=false;if(cancelled())return "STOPPED";  // 将 false 写入或初始化 done（完成）。然后若 cancelled()，立即返回 "STOPPED"。
    if(s.kind==StepKind::RING_CORRECT)return correctRing(s,done);  // 若 s.kind==StepKind::RING_CORRECT，立即返回 correctRing(s,done)。
    if(s.kind==StepKind::POSITION){TickPositionMove();done=!PositionMoveActive();return nullptr;}  // 当 s.kind==StepKind::POSITION 成立时进入此分支，否则继续后续流程。
    if(s.kind==StepKind::TRAY_ROTATE||s.kind==StepKind::TRAY_READY) {  // 当 s.kind==StepKind::TRAY_ROTATE||s.kind==StepKind::TRAY_… 成立时进入此分支，否则继续后续流程。
        float angle=0;  // 将 0 写入或初始化 angle（角度）。
        if(!readServoAngle(SERVO_ID_CARGO_TRAY,angle))return "SERVO_READ";  // 若 !readServoAngle(SERVO_ID_CARGO_TRAY,angle)，立即返回 "SERVO_READ"。
        done=fabsf(angle-s.turret)<=s.trayArrivalToleranceDeg;  // 将 fabsf(angle-s.turret)<=s.trayArrivalToleranceDeg 写入或初始化 done（完成）。
        if(done)trayMoving=false;  // 当 done 成立时进入此分支，否则继续后续流程。
        return nullptr;  // 结束当前函数并返回 nullptr。
    }  // 结束此条件分支。
    if(s.kind==StepKind::TURRET_READY) {  // 当 s.kind==StepKind::TURRET_READY 成立时进入此分支，否则继续后续流程。
        float angle=0;  // 将 0 写入或初始化 angle（角度）。
        if(ReadArmTurretAngle(angle)&&fabsf(angle-s.turret)<=5){done=true;return nullptr;}  // 当 ReadArmTurretAngle(angle)&&fabsf(angle-s.turret)<=5 成立时进入此分支，否则继续后续流程。
        return millis()-turretStarted>=s.waitMs?"EXECUTION":nullptr;  // 结束当前函数并返回 millis()-turretStarted>=s.waitMs?"EXECUTION":nullptr。
    }  // 结束此条件分支。
    if(s.liftArrival||s.workPlaceLiftArrival) {  // 当 s.liftArrival||s.workPlaceLiftArrival 成立时进入此分支，否则继续后续流程。
        node_position::Motor state;  // 声明 state（状态），类型和可用范围由所在声明决定。
        const char *error=ReadMotorFeedback(5,state);if(error)return error;  // 将 ReadMotorFeedback(5,state) 写入或初始化 error（错误）。然后若 error，立即返回 error。
        // 升降正方向为CCW，位置反馈为负；保留符号，不把反向位置当作抬升到位。
        const double measured=-static_cast<double>(state.position)*3200.0/65536.0/HEIGHT_PULSE;  // 将 -static_cast<double>(state.position)*3200.0/65536.0/HE… 写入或初始化 measured。
        done=s.workPlaceLiftArrival ? fabs(measured-s.high)<10.0  // 将 s.workPlaceLiftArrival ? fabs(measured-s.high)<10.0 写入或初始化 done（完成）。
            : (state.flags&2)&&fabs(measured-s.high)<=1.0;  // 调用 fabs，传入 measured-s.high)<=1.0。
        if(done&&s.lengthArrival) {  // 当 done&&s.lengthArrival 成立时进入此分支，否则继续后续流程。
            if(const char *lengthError=ReadMotorFeedback(6,state))return lengthError;  // 若 const char *lengthError=ReadMotorFeedback(6,state)，立即返回 lengthError。
            const double length=-static_cast<double>(state.position)*3200.0/65536.0/LENGTH_PULSE;  // 将 -static_cast<double>(state.position)*3200.0/65536.0/LE… 写入或初始化 length（长度）。
            done=(state.flags&2)&&fabs(length-s.length)<=1.0;  // 将 (state.flags&2)&&fabs(length-s.length)<=1.0 写入或初始化 done（完成）。
        }  // 结束此条件分支。
        if(done)Serial.printf("{EVT,MOTION,LIFT_ARRIVED,%lu,%.3f,%.3f}\n",  // 当 done 成立时进入此分支，否则继续后续流程。
            static_cast<unsigned long>(job),s.high,measured);  // 声明接口 long>，参数由此处的签名确定。
        return nullptr;  // 结束当前函数并返回 nullptr。
    }  // 结束此条件分支。
    if(s.turretArrival) {  // 当 s.turretArrival 成立时进入此分支，否则继续后续流程。
        float measured=0;if(!ReadArmTurretAngle(measured))return "MOTOR_READ";  // 将 0 写入或初始化 measured。然后若 !ReadArmTurretAngle(measured)，立即返回 "MOTOR_READ"。
        done=fabsf(measured-turretTarget)<=5.0f;  // 将 fabsf(measured-turretTarget)<=5.0f 写入或初始化 done（完成）。
        if(done&&s.lengthArrival) {  // 当 done&&s.lengthArrival 成立时进入此分支，否则继续后续流程。
            node_position::Motor state;  // 声明 state（状态），类型和可用范围由所在声明决定。
            const char *error=ReadMotorFeedback(6,state);if(error)return error;  // 将 ReadMotorFeedback(6,state) 写入或初始化 error（错误）。然后若 error，立即返回 error。
            const double length=fabs(static_cast<double>(state.position))*3200/65536/LENGTH_PULSE;  // 将 fabs(static_cast<double>(state.position))*3200/65536/L… 写入或初始化 length（长度）。
            const double errorMm=fabs(length-s.length);  // 将 fabs(length-s.length) 写入或初始化 errorMm（错误毫米）。
            done=s.lengthArrivalByPosition ? errorMm<=10.0 : (state.flags&2)&&errorMm<=1.0;  // 将 s.lengthArrivalByPosition ? errorMm<=10.0 : (state.fla… 写入或初始化 done（完成）。
        }  // 结束此条件分支。
        if(done)Serial.printf("{EVT,MOTION,TURRET_ARRIVED,%lu,%.3f,%.3f}\n",  // 当 done 成立时进入此分支，否则继续后续流程。
            static_cast<unsigned long>(job),turretTarget,measured);  // 声明接口 long>，参数由此处的签名确定。
        return nullptr;  // 结束当前函数并返回 nullptr。
    }  // 结束此条件分支。
    // 反馈模式需在实车核验驱动器到位标志及编码器单位后显式开启。
    // currentArm 仅保存目标，只有下列实读反馈才能结束动作。
    for(uint8_t id=5;id<=6;++id) {  // 按 uint8_t id=5;id<=6;++id 组织循环的初值、继续条件和步进。
        const float target=id==5?s.high:s.length;if(target==-1)continue;  // 将 id==5?s.high:s.length 写入或初始化 target（目标）。然后若 target==-1，直接进入下一轮循环。
        node_position::Motor state;const char *error=ReadMotorFeedback(id,state);if(error)return error;  // 声明 state（状态），类型和可用范围由所在声明决定。然后将 ReadMotorFeedback(id,state) 写入或初始化 error（错误）。然后若 error，立即返回 error。
        const double measured=fabs(static_cast<double>(state.position))*3200/65536/(id==5?HEIGHT_PULSE:LENGTH_PULSE);  // 将 fabs(static_cast<double>(state.position))*3200/65536/(… 写入或初始化 measured。
        if(!(state.flags&2)||fabs(measured-target)>1)return nullptr;  // 若 !(state.flags&2)||fabs(measured-target)>1，立即返回 nullptr。
    }  // 结束此循环体。
    if(s.planned||s.turret!=-1) {  // 当 s.planned||s.turret!=-1 成立时进入此分支，否则继续后续流程。
        float measured=0;if(!ReadArmTurretAngle(measured))return "MOTOR_READ";  // 将 0 写入或初始化 measured。然后若 !ReadArmTurretAngle(measured)，立即返回 "MOTOR_READ"。
        if(fabsf(measured-currentArm.turret_angle)>5)return nullptr;  // 若 fabsf(measured-currentArm.turret_angle)>5，立即返回 nullptr。
    }  // 结束此条件分支。
    if(s.pawl!=-1) {  // 当 s.pawl!=-1 成立时进入此分支，否则继续后续流程。
        float measured=0;if(!readServoAngle(1,measured,100))return "SERVO_READ";  // 将 0 写入或初始化 measured。然后若 !readServoAngle(1,measured,100)，立即返回 "SERVO_READ"。
        if(fabsf(continuous::turn(measured,s.pawl))>5)return nullptr;  // 若 fabsf(continuous::turn(measured,s.pawl))>5，立即返回 nullptr。
    }  // 结束此条件分支。
    done=true;return nullptr;  // 将 true 写入或初始化 done（完成）。然后结束当前函数并返回 nullptr。
}  // 结束此函数或控制块。
/**
 * @brief 输出当前作业的 STEP 开始/结束日志，index 为步骤下标。
 */
void MotionHardware::event(size_t index,const Step &step,bool begin) {  // 定义函数或成员 的入口 MotionHardware::event，参数由此处的签名确定。
    if(!begin&&step.materialEvent) {  // 当 !begin&&step.materialEvent 成立时进入此分支，否则继续后续流程。
        char message[48];  // 声明 message，类型和可用范围由所在声明决定。
        snprintf(message,sizeof(message),"%s %u",step.materialEvent,step.materialPosition);  // 调用 snprintf，传入 message,sizeof(message),"%s %u",step.materialEvent,ste…。
        Serial.printf("{EVT,MATERIAL,%s,%u,%lu}\n",step.materialEvent,step.materialPosition,static_cast<unsigned long>(job));  // 调用 Serial.printf，传入 "{EVT,MATERIAL,%s,%u,%lu}\n",step.materialEvent,step.m…。
        updateDisplay("DISPLAY","DEBUG",message);  // 调用 updateDisplay，传入 "DISPLAY","DEBUG",message。
    }  // 结束此条件分支。
    if(!begin&&step.placementPosition)  // 当 !begin&&step.placementPosition 成立时进入此分支，否则继续后续流程。
        LogArmPlacementReadback(job,step.placementPosition,currentArm.turret_angle,currentArm.length);  // 调用 LogArmPlacementReadback（Log机械臂PlacementReadback），传入 job,step.placementPosition,currentArm.turret_angle,cur…。
    Serial.printf("{EVT,MOTION,STEP,%lu,%lu,%u,%s,%s}\n",static_cast<unsigned long>(job),  // 通过串口执行 printf，输出内容或参数由本行给出。
        static_cast<unsigned long>(millis()),static_cast<unsigned>(index),step.name,begin?"BEGIN":"END");  // 声明接口 long>，参数由此处的签名确定。
}  // 结束此函数或控制块。
