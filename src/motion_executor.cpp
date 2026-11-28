// 运动作业的调度任务。
// 提交方写入请求，执行器依动作类型推进子状态并发布完成或阶段事件；作业编号用于关联提交、接续和结果。

#include "motion_executor.h"  // 引入 motion_executor.h 中的声明和定义。
#include "motion_hardware.h"  // 引入 motion_hardware.h 中的声明和定义。
#include "motion_job.h"  // 引入 motion_job.h 中的声明和定义。
#include "arm_motion.h"  // 引入 arm_motion.h 中的声明和定义。
#include "robot_runtime.h"  // 引入 robot_runtime.h 中的声明和定义。
#include "continuous_trial.h"  // 引入 continuous_trial.h 中的声明和定义。
#include "node_position_hardware.h"  // 引入 node_position_hardware.h 中的声明和定义。
#include "motor_bus.h"  // 引入 motor_bus.h 中的声明和定义。
#include "turret_motor.h"  // 引入 turret_motor.h 中的声明和定义。
#include "arm_axis_trial.h"  // 引入 arm_axis_trial.h 中的声明和定义。
#include <float.h>  // 引入 float.h 中的声明和定义。

// 保留旧参数注册位置以兼容上位机编号；观察姿态现按路径阶段提前触发。
float MOTION_LOOK_LEAD_MM=1500,MOTION_LOOK2_LEAD_MM=1500;  // 将 1500,MOTION_LOOK2_LEAD_MM=1500 写入或初始化 MOTION_LOOK_LEAD_MM（MOTIONLOOKLEAD毫米）。
uint32_t ARM_FEEDBACK_ENABLED=0,ARM_GRIP_HOLD_MS=0,ARM_RELEASE_HOLD_MS=0;  // 将 0,ARM_GRIP_HOLD_MS=0,ARM_RELEASE_HOLD_MS=0 写入或初始化 ARM_FEEDBACK_ENABLED（机械臂反馈已启用）。
namespace {  // 开始匿名命名空间，本文件内部共享其中的定义。
TaskHandle_t worker=nullptr;  // 将 nullptr 写入或初始化 worker。
QueueHandle_t queue=nullptr;  // 将 nullptr 写入或初始化 queue。
portMUX_TYPE jobMux=portMUX_INITIALIZER_UNLOCKED;  // 将 portMUX_INITIALIZER_UNLOCKED 写入或初始化 jobMux（job临界区锁）。
uint32_t nextId=0;  // 将 0 写入或初始化 nextId（下一编号）。
motion::LaunchGate launchGate;  // 声明 launchGate，类型和可用范围由所在声明决定。
bool ota=false,stopPending=false;  // 将 false,stopPending=false 写入或初始化 ota（固件更新）。
MotionStatus status,history[8];  // 声明 status（状态），类型和可用范围由所在声明决定。
size_t historyIndex=0;  // 将 0 写入或初始化 historyIndex（history下标）。
struct Job {MotionRequest request;uint32_t id=0,generation=0;RobotState state=STATE_WAIT_START;int progress=0;} job;  // 定义 Job 的类型成员或作用域。
motion::Program program;  // 声明 program，类型和可用范围由所在声明决定。
motion::Sequence arm;  // 声明 arm（机械臂），类型和可用范围由所在声明决定。
motion::Join join;  // 声明 join，类型和可用范围由所在声明决定。
MotionHardware hardware;  // 声明 hardware，类型和可用范围由所在声明决定。
bool running=false;  // 将 false 写入或初始化 running。
bool armSpanOpen=false;  // 将 false 写入或初始化 armSpanOpen（机械臂SpanOpen）。
bool materialPaused=false,resumeMaterial=false;  // 将 false,resumeMaterial=false 写入或初始化 materialPaused（物料Paused）。
size_t materialFirstStep=0;  // 将 0 写入或初始化 materialFirstStep（物料首个Step）。
size_t materialRetriedStep=motion::Program::CAPACITY;  // 将 motion::Program::CAPACITY 写入或初始化 materialRetriedStep（物料RetriedStep）。
uint32_t materialCheckToken=0;  // 将 0 写入或初始化 materialCheckToken（物料CheckToken）。
uint8_t stage=0,scanTurn=0,pathRetries=0;  // 将 0,scanTurn=0,pathRetries=0 写入或初始化 stage（阶段）。
constexpr uint8_t SCAN_WAIT=4,SCAN_SWEEP=5,SCAN_WAIT_CODE=6,SCAN_RETREAT=7;  // 将 4,SCAN_SWEEP=5,SCAN_WAIT_CODE=6,SCAN_RETREAT=7 写入或初始化 SCAN_WAIT（扫码WAIT）。
uint32_t scanArrivedAt=0,scanTurnAt=0;  // 将 0,scanTurnAt=0 写入或初始化 scanArrivedAt（扫码ArrivedAt）。
float scanTrayAngle=0;  // 将 0 写入或初始化 scanTrayAngle（扫码料盘角度）。
bool scanRetreated=false;  // 将 false 写入或初始化 scanRetreated（扫码Retreated）。
int64_t discBefore[4]={};  // 将 {} 写入或初始化 discBefore[4]。
float discBeforeHeading=0,discMeasuredYOffset=0;  // 将 0,discMeasuredYOffset=0 写入或初始化 discBeforeHeading（圆盘Before航向）。
bool discMeasuredYOffsetValid=false;  // 将 false 写入或初始化 discMeasuredYOffsetValid（圆盘Measured纵向偏移有效）。
int64_t discReturnTargets[4]={};  // 将 {} 写入或初始化 discReturnTargets[4]。
const char *activeSpan=nullptr;  // 将 nullptr 写入或初始化 activeSpan（活跃Span）。

void publishBaseEstimate(bool force=false) {  // 定义函数或成员 的入口 publishBaseEstimate，参数由此处的签名确定。
    static RobotPose previous{NAN,NAN,NAN};  // 声明 previous（上一），类型和可用范围由所在声明决定。
    const auto pose=currentPose;  // 将 currentPose 写入或初始化 pose。
    if(!force&&pose.x==previous.x&&pose.y==previous.y&&pose.theta==previous.theta)return;  // 当 !force&&pose.x==previous.x&&pose.y==previous.y&&pose.t… 成立时进入此分支，否则继续后续流程。
    previous=pose;  // 将 pose 写入或初始化 previous（上一）。
    Serial.printf("{EVT,POSE,ESTIMATE,%.2f,%.2f,%.3f}\n",pose.x,pose.y,pose.theta);  // 调用 Serial.printf，传入 "{EVT,POSE,ESTIMATE,%.2f,%.2f,%.3f}\n",pose.x,pose.y,p…。
}  // 结束此初始化列表或局部作用域。

void publishArmEstimate() {  // 定义函数或成员 的入口 publishArmEstimate（publish机械臂Estimate），参数由此处的签名确定。
    static ArmPose previous{NAN,NAN,NAN,NAN};  // 声明 previous（上一），类型和可用范围由所在声明决定。
    const auto pose=currentArm;  // 将 currentArm 写入或初始化 pose。
    if(pose.high==previous.high&&pose.length==previous.length&&  // 当 pose.high==previous.high&&pose.length==previous.length… 成立时进入此分支，否则继续后续流程。
       pose.turret_angle==previous.turret_angle&&pose.pawl_angle==previous.pawl_angle)return;  // 继续上一行的组合条件，本行提供 pose.turret_angle==previous.turret_angle&&pose.pawl_angle==previous.pawl_angle)return;。
    previous=pose;  // 将 pose 写入或初始化 previous（上一）。
    Serial.printf("{EVT,ARM,POSE,%.2f,%.2f,%.3f,%.3f}\n",pose.high,pose.length,pose.turret_angle,pose.pawl_angle);  // 调用 Serial.printf，传入 "{EVT,ARM,POSE,%.2f,%.2f,%.3f,%.3f}\n",pose.high,pose.…。
}  // 结束此函数或控制块。

/**
 * @brief 在临界区内复制当前作业状态，供其他任务读取。
 */
MotionStatus snapshot(){portENTER_CRITICAL(&jobMux);auto s=status;portEXIT_CRITICAL(&jobMux);return s;}  // 将 status;portEXIT_CRITICAL(&jobMux);return s;} 写入或初始化 s。
/**
 * @brief 检查OTA、取消代次与总任务取消标志，供当前运动控制器轮询。
 */
bool cancelled(){portENTER_CRITICAL(&jobMux);bool c=ota||launchGate.cancelled(job.generation);portEXIT_CRITICAL(&jobMux);return c||taskMotionAborted;}  // 将 ota||launchGate.cancelled(job.generation);portEXIT_CRI… 写入或初始化 c。
/**
 * @brief 输出当前作业某个分项的BEGIN或END计时事件。
 */
void span(const char *name,bool begin) {  // 定义函数或成员 的入口 span，参数由此处的签名确定。
    Serial.printf("{EVT,MOTION,SPAN,%lu,%lu,%s,%s}\n",static_cast<unsigned long>(job.id),  // 通过串口执行 printf，输出内容或参数由本行给出。
        static_cast<unsigned long>(millis()),name,begin?"BEGIN":"END");  // 声明接口 long>，参数由此处的签名确定。
}  // 结束此函数或控制块。
/**
 * @brief 结束上一分项并开始新分项，同时更新当前作业阶段名称。
 */
void phase(const char *name) {  // 定义函数或成员 的入口 phase（阶段），参数由此处的签名确定。
    if(activeSpan&&strcmp(activeSpan,name)==0)return;  // 当 activeSpan&&strcmp(activeSpan,name)==0 成立时进入此分支，否则继续后续流程。
    if(activeSpan)span(activeSpan,false);activeSpan=name;span(name,true);  // 当 activeSpan 成立时进入此分支，否则继续后续流程。
    portENTER_CRITICAL(&jobMux);status.phase=name;portEXIT_CRITICAL(&jobMux);  // 进入临界区，保护共享状态的本段访问。然后将 name 写入或初始化 status.phase。然后离开临界区，保护共享状态的本段访问。
}  // 结束此函数或控制块。
/**
 * @brief 输出当前作业的单次完成标志事件，用于分析底盘和机械臂重叠。
 */
void flag(const char *name) {  // 定义函数或成员 的入口 flag，参数由此处的签名确定。
    Serial.printf("{EVT,MOTION,FLAG,%lu,%lu,%s,1}\n",static_cast<unsigned long>(job.id),  // 通过串口执行 printf，输出内容或参数由本行给出。
        static_cast<unsigned long>(millis()),name);  // 声明接口 long>，参数由此处的签名确定。
}  // 结束此函数或控制块。
/**
 * @brief 取消路径与相对位置运动并停止升降/伸缩/转盘；保留夹爪当前命令。
 */
void stopHardware() {  // 定义函数或成员 的入口 stopHardware（停止Hardware），参数由此处的签名确定。
    ContinuousMotionCancel();  // 调用 ContinuousMotionCancel，使用无参形式推进相应操作。
    CancelPositionMove();  // 调用 CancelPositionMove（Cancel位置Move），使用无参形式推进相应操作。
    hardware.stopTray();  // 调用 hardware.stopTray。
    MotorBusGuard transaction;  // 声明 transaction，类型和可用范围由所在声明决定。
    for(uint8_t id=5;id<=6;++id)Emm_V5_Stop_Now(id,false);  // 按 uint8_t id=5;id<=6;++id)Emm_V5_Stop_Now(id,false); 组织循环的初值、继续条件和步进。
    Emm_V5_Stop_Now(TURRET_MOTOR_ID,false);  // 调用 Emm_V5_Stop_Now（EmmV5停止Now），传入 TURRET_MOTOR_ID,false。
    TurretMotorCancelTrace();  // 调用 TurretMotorCancelTrace（转台MotorCancelTrace），使用无参形式推进相应操作。
    ArmAxisTrialCancel();  // 调用 ArmAxisTrialCancel（机械臂AxisTrialCancel），使用无参形式推进相应操作。
    // 保留夹爪当前命令，不在未知持料状态下松手。
}  // 结束此函数或控制块。
/**
 * @brief 根据动作类型发送完成应答，并在圆盘抓取成功后推进物料登记进度。
 */
void completedEvent(const MotionStatus &result) {  // 定义函数或成员 的入口 completedEvent，参数由此处的签名确定。
    const auto &r=job.request;const bool success=result.state==MotionState::DONE;  // 将 job.request 写入或初始化 r。然后将 result.state==MotionState::DONE 写入或初始化 success（成功）。
    if(r.kind==MotionKind::LIDAR_POSE&&r.category[0])CompleteLidarPose(success,r.debug);  // 当 r.kind==MotionKind::LIDAR_POSE&&r.category[0] 成立时进入此分支，否则继续后续流程。
    if(r.startVision) {  // 当 r.startVision 成立时进入此分支，否则继续后续流程。
        const auto mode=static_cast<VisionStartMode>(r.vision);  // 将 static_cast<VisionStartMode>(r.vision) 写入或初始化 mode（模式）。
        if(success&&currentState==job.state&&!taskMotionAborted)requestVisionStart(mode);  // 当 success&&currentState==job.state&&!taskMotionAborted 成立时进入此分支，否则继续后续流程。
        else Serial.printf("{EVT,VISION,START_FAILED,%s,ARM}\n",mode==VisionStartMode::WORK_AREA?"WORK_AREA":"WORK_AREA_LOADED");  // 前一个判断不成立时执行此分支。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    if(r.kind==MotionKind::DISC_GRAB) {  // 当 r.kind==MotionKind::DISC_GRAB 成立时进入此分支，否则继续后续流程。
        if(r.position>=1&&r.position<=3)Serial.printf("{EVT,VISION,DISC_POSITIONS,%s,%u,%u,%u}\n",success?"DONE":"FAILED",r.position,r.color,r.cargo);  // 当 r.position>=1&&r.position<=3 成立时进入此分支，否则继续后续流程。
        else Serial.printf("{EVT,VISION,%s,%s,%u,%u}\n",r.force?"FORCE_GRAB":"GRAB",success?"DONE":"FAILED",r.color,r.cargo);  // 前一个判断不成立时执行此分支。
        if(r.automatic&&currentState==job.state) {  // 当 r.automatic&&currentState==job.state 成立时进入此分支，否则继续后续流程。
            if(!success) {  // 当 !success 成立时进入此分支，否则继续后续流程。
                requestVisionStop(VisionStartMode::DISC_MATERIAL);  // 调用 requestVisionStop（请求视觉停止），传入 VisionStartMode::DISC_MATERIAL。
                if(xHomeTimer)xTimerStop(xHomeTimer,0);  // 当 xHomeTimer 成立时进入此分支，否则继续后续流程。
                currentState=STATE_TRANSFER_FAILED;  // 将 STATE_TRANSFER_FAILED 写入或初始化 currentState（当前状态）。
                updateDisplay("DISPLAY","DEBUG","DISC GRAB ERR");  // 调用 updateDisplay，传入 "DISPLAY","DEBUG","DISC GRAB ERR"。
            } else {  // 前一个判断不成立时执行此分支。
                const int next=AdvanceDiscGrabProgress(job.progress,r.cargo,true);  // 将 AdvanceDiscGrabProgress(job.progress,r.cargo,true) 写入或初始化 next（下一）。
                if(next==3)requestVisionStop(VisionStartMode::DISC_MATERIAL);  // 当 next==3 成立时进入此分支，否则继续后续流程。
                roundProgress=next;  // 将 next 写入或初始化 roundProgress（轮次Progress）。
            }  // 结束此条件分支。
        }  // 结束此条件分支。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    if(r.category[0]) {  // 当 r.category[0] 成立时进入此分支，否则继续后续流程。
        if(r.kind==MotionKind::TRANSFER)Serial.printf("{EVT,ARM,TRANSFER,%s,%u,%u}\n",success?"DONE":"FAILED",r.cargo,r.work);  // 当 r.kind==MotionKind::TRANSFER 成立时进入此分支，否则继续后续流程。
        else if(r.kind==MotionKind::RETRIEVE)Serial.printf("{EVT,ARM,RETRIEVE,%s,%u,%u}\n",success?"DONE":"FAILED",r.work,r.cargo);  // 仅在先前条件均不成立时，当 r.kind==MotionKind::RETRIEVE 成立时进入此分支，否则继续后续流程。
        else if(r.kind==MotionKind::AXIS_TRIAL)Serial.printf("{EVT,EMM,AXIS_TRIAL,%s,%u}\n",success?"DONE":"FAILED",r.axisTrialMotor);  // 仅在先前条件均不成立时，当 r.kind==MotionKind::AXIS_TRIAL 成立时进入此分支，否则继续后续流程。
        else if(r.kind==MotionKind::LIDAR_POSE)Serial.println(success?"{EVT,VISION,LIDAR_POSE_DONE}":"{EVT,VISION,LIDAR_POSE_FAILED}");  // 仅在先前条件均不成立时，当 r.kind==MotionKind::LIDAR_POSE 成立时进入此分支，否则继续后续流程。
        else if(r.kind==MotionKind::POSITION)Serial.println(success?"{EVT,CHASSIS,MOVE_POSITION,DONE,ESTIMATED}":"{EVT,CHASSIS,MOVE_POSITION,FAILED}");  // 仅在先前条件均不成立时，当 r.kind==MotionKind::POSITION 成立时进入此分支，否则继续后续流程。
        else Serial.printf("{EVT,%s,%s,%s}\n",r.category,r.command,  // 前一个判断不成立时执行此分支。
            success?(r.kind==MotionKind::ARM_RAW||r.kind==MotionKind::ARM_POSE||r.kind==MotionKind::SERVO_RAW?"ISSUED":"DONE"):"FAILED");  // 续写当前声明或运算，提供 success?(r.kind==MotionKind::ARM_RAW||r.kind==MotionKi…。
    }  // 结束此条件分支。
}  // 结束此函数或控制块。
/**
 * @brief 结束当前作业，记录终态、计时和错误，发送业务事件并保存历史快照。
 */
void finish(MotionState state,const char *reason=nullptr) {  // 定义函数或成员 的入口 finish，参数由此处的签名确定。
    hardware.stopRingVision();  // 调用 hardware.stopRingVision。
    if(activeSpan){span(activeSpan,false);activeSpan=nullptr;}  // 当 activeSpan 成立时进入此分支，否则继续后续流程。
    if(armSpanOpen){span("ARM_PROGRAM",false);armSpanOpen=false;}  // 当 armSpanOpen 成立时进入此分支，否则继续后续流程。
    if(job.request.kind==MotionKind::NAVIGATE||job.request.kind==MotionKind::SCAN_TASK) {  // 当 job.request.kind==MotionKind::NAVIGATE||job.request.ki… 成立时进入此分支，否则继续后续流程。
        portENTER_CRITICAL(&nodePathMux);nodePathState=state==MotionState::DONE?NodePathState::DONE:NodePathState::FAILED;portEXIT_CRITICAL(&nodePathMux);  // 进入临界区，保护共享状态的本段访问。然后将 state==MotionState::DONE?NodePathState::DONE:NodePathS… 写入或初始化 nodePathState（节点路径状态）。然后离开临界区，保护共享状态的本段访问。
    }  // 结束此条件分支。
    if(job.request.kind==MotionKind::LIDAR_POSE&&state==MotionState::DONE&&!job.request.debug)  // 当 job.request.kind==MotionKind::LIDAR_POSE&&state==Motio… 成立时进入此分支，否则继续后续流程。
        enableRun=false; // 位姿完成前收到的 START 不算 Nano 扫描完成。
    const uint32_t now=millis();  // 将 millis() 写入或初始化 now。
    portENTER_CRITICAL(&jobMux);  // 进入临界区，保护共享状态的本段访问。
    status.finishedMs=now;status.elapsedMs=now-status.startedMs;status.state=state;status.reason=reason;  // 将 now 写入或初始化 status.finishedMs。然后将 now-status.startedMs 写入或初始化 status.elapsedMs。然后将 state 写入或初始化 status.state。然后将 reason 写入或初始化 status.reason。
    status.chassisDone=join.chassisDone;status.armStarted=join.armStarted;status.armReady=join.armReady;  // 将 join.chassisDone 写入或初始化 status.chassisDone。然后将 join.armStarted 写入或初始化 status.armStarted。然后将 join.armReady 写入或初始化 status.armReady。
    materialPaused=resumeMaterial=false;  // 将 resumeMaterial=false 写入或初始化 materialPaused（物料Paused）。
    const auto result=status;  // 将 status 写入或初始化 result（结果）。
    portEXIT_CRITICAL(&jobMux);  // 离开临界区，保护共享状态的本段访问。
    Serial.printf("{EVT,MOTION,RESULT,%lu,%lu,%s,%lu}\n",static_cast<unsigned long>(job.id),  // 通过串口执行 printf，输出内容或参数由本行给出。
        static_cast<unsigned long>(now),MotionStateName(state),static_cast<unsigned long>(result.elapsedMs));  // 声明接口 long>，参数由此处的签名确定。
    if(reason)Serial.printf("{EVT,MOTION,ERROR,%lu,%lu,%s}\n",static_cast<unsigned long>(job.id),static_cast<unsigned long>(now),reason);  // 当 reason 成立时进入此分支，否则继续后续流程。
    publishBaseEstimate(true);  // 调用 publishBaseEstimate，传入 true。
    publishArmEstimate();  // 调用 publishArmEstimate（publish机械臂Estimate），使用无参形式推进相应操作。
    completedEvent(result);  // 调用 completedEvent，传入 result。
    portENTER_CRITICAL(&jobMux);history[historyIndex++%8]=result;portEXIT_CRITICAL(&jobMux);  // 进入临界区，保护共享状态的本段访问。然后将 result 写入或初始化 history[historyIndex++%8]。然后离开临界区，保护共享状态的本段访问。
    running=false;  // 将 false 写入或初始化 running。
}  // 结束此初始化列表或局部作用域。
/**
 * @brief 调用BuildArmProgram构建步骤并启动机械臂Sequence；失败时结束当前作业。
 * @return true 序列已启动，false 构建失败。
 */
bool beginArm(const MotionRequest &request) {  // 定义函数或成员 的入口 beginArm（begin机械臂），参数由此处的签名确定。
    portENTER_CRITICAL(&jobMux);status.phase="ARM_PREPARE";portEXIT_CRITICAL(&jobMux);  // 进入临界区，保护共享状态的本段访问。然后将 "ARM_PREPARE" 写入或初始化 status.phase。然后离开临界区，保护共享状态的本段访问。
    const char *error=BuildArmProgram(request,program);  // 将 BuildArmProgram(request,program) 写入或初始化 error（错误）。
    if(error){stopHardware();finish(MotionState::FAILED,error);return false;}  // 当 error 成立时进入此分支，否则继续后续流程。
    arm.begin(program,hardware);join.armStarted=true;join.armReady=false;  // 调用 arm.begin，传入 program,hardware。然后将 true 写入或初始化 join.armStarted。然后将 false 写入或初始化 join.armReady。
    span("ARM_PROGRAM",true);armSpanOpen=true;flag("ARM_STARTED");return true;  // 调用 span，传入 "ARM_PROGRAM",true。然后将 true 写入或初始化 armSpanOpen（机械臂SpanOpen）。然后调用 flag，传入 "ARM_STARTED"。然后结束当前函数并返回 true。
}  // 结束此函数或控制块。
/**
 * @brief 推进一次机械臂步骤序列，结束时设置ARM_READY；仍在执行或失败时返回false。
 */
bool tickArm() {  // 定义函数或成员 的入口 tickArm（轮询机械臂），参数由此处的签名确定。
    arm.tick();publishArmEstimate();if(arm.active())return false;  // 调用 arm.tick。然后调用 publishArmEstimate（publish机械臂Estimate），使用无参形式推进相应操作。然后若 arm.active()，立即返回 false。
    if(arm.failure()){  // 当 arm.failure() 成立时进入此分支，否则继续后续流程。
        const auto &request=job.request;  // 将 job.request 写入或初始化 request（请求）。
        const auto kind=request.kind;  // 将 request.kind 写入或初始化 kind。
        const bool materialTransfer=kind==MotionKind::PLACE_ROUND||kind==MotionKind::RETRIEVE_ROUND  // 将 kind==MotionKind::PLACE_ROUND||kind==MotionKind::RETRI… 写入或初始化 materialTransfer（物料Transfer）。
            ||kind==MotionKind::DISC_GRAB;  // 继续上一行的组合条件，本行提供 ||kind==MotionKind::DISC_GRAB;。
        const char *error=arm.failure();  // 将 arm.failure() 写入或初始化 error（错误）。
        if(request.automatic&&materialTransfer&&currentState==job.state&&!cancelled()) {  // 当 request.automatic&&materialTransfer&&currentState==job… 成立时进入此分支，否则继续后续流程。
            stopHardware();  // 调用 stopHardware（停止Hardware），使用无参形式推进相应操作。
            hardware.stopRingVision();  // 调用 hardware.stopRingVision。
            if(armSpanOpen){span("ARM_PROGRAM",false);armSpanOpen=false;}  // 当 armSpanOpen 成立时进入此分支，否则继续后续流程。
            materialFirstStep=0;  // 将 0 写入或初始化 materialFirstStep（物料首个Step）。
            unsigned completed=0;  // 将 0 写入或初始化 completed。
            for(size_t i=0;i<arm.stepIndex();++i)if(program.steps[i].kind==motion::StepKind::COMMIT) {  // 按 size_t i=0;i<arm.stepIndex();++i)if(program.steps[i].k… 组织循环的初值、继续条件和步进。
                ++completed;  // 续写当前声明或运算，提供 ++completed;。
                materialFirstStep=i+1;  // 将 i+1 写入或初始化 materialFirstStep（物料首个Step）。
            }  // 结束此循环体。
            bool beforeGrip=true;  // 将 true 写入或初始化 beforeGrip。
            for(size_t i=materialFirstStep;i<=arm.stepIndex();++i) {  // 按 size_t i=materialFirstStep;i<=arm.stepIndex();++i 组织循环的初值、继续条件和步进。
                const char *name=program.steps[i].name;  // 将 program.steps[i].name 写入或初始化 name。
                if(!strcmp(name,"PICK_GRIP")||!strcmp(name,"DISC_GRIP")) {  // 当 !strcmp(name,"PICK_GRIP")||!strcmp(name,"DISC_GRIP") 成立时进入此分支，否则继续后续流程。
                    beforeGrip=false;break;  // 将 false 写入或初始化 beforeGrip。然后退出当前循环或 switch 分支。
                }  // 结束此条件分支。
            }  // 结束此循环体。
            if(beforeGrip&&motion::retryableFeedback(error)&&materialRetriedStep!=materialFirstStep) {  // 当 beforeGrip&&motion::retryableFeedback(error)&&material… 成立时进入此分支，否则继续后续流程。
                materialRetriedStep=materialFirstStep;  // 将 materialFirstStep 写入或初始化 materialRetriedStep（物料RetriedStep）。
                Serial.printf("{EVT,MOTION,MATERIAL_RETRY,%lu,%s,%s}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                    static_cast<unsigned long>(job.id),program.steps[arm.stepIndex()].name,error);  // 声明接口 long>，参数由此处的签名确定。
                arm.begin(program,hardware,materialFirstStep);  // 调用 arm.begin，传入 program,hardware,materialFirstStep。
                span("ARM_PROGRAM",true);armSpanOpen=true;  // 调用 span，传入 "ARM_PROGRAM",true。然后将 true 写入或初始化 armSpanOpen（机械臂SpanOpen）。
                return false;  // 结束当前函数并返回 false。
            }  // 结束此条件分支。
            const bool disc=kind==MotionKind::DISC_GRAB;  // 将 kind==MotionKind::DISC_GRAB 写入或初始化 disc（圆盘）。
            const unsigned cargo=disc?request.cargo:CargoForGrabIndex(completed);  // 将 disc?request.cargo:CargoForGrabIndex(completed) 写入或初始化 cargo（货位）。
            const unsigned work=disc?request.position:request.positions[completed];  // 将 disc?request.position:request.positions[completed] 写入或初始化 work（工位）。
            portENTER_CRITICAL(&jobMux);  // 进入临界区，保护共享状态的本段访问。
            materialPaused=true;  // 将 true 写入或初始化 materialPaused（物料Paused）。
            const uint32_t token=++materialCheckToken;  // 将 ++materialCheckToken 写入或初始化 token。
            status.phase="MATERIAL_CHECK";  // 将 "MATERIAL_CHECK" 写入或初始化 status.phase。
            status.reason=error;  // 将 error 写入或初始化 status.reason。
            portEXIT_CRITICAL(&jobMux);  // 离开临界区，保护共享状态的本段访问。
            Serial.printf("{EVT,MOTION,MATERIAL_CHECK,%lu,%lu,%s,%u,%u,%s,%s}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                static_cast<unsigned long>(job.id),static_cast<unsigned long>(token),disc?"DISC_GRAB":kind==MotionKind::RETRIEVE_ROUND?"RETRIEVE":"PLACE",  // 定义函数或成员 long>，参数由此处的签名确定。
                cargo,work,program.steps[arm.stepIndex()].name,error);  // 调用 arm.stepIndex（机械臂step下标），传入 )].name,error。
            updateDisplay("DISPLAY","DEBUG","CHECK MATERIAL");  // 调用 updateDisplay，传入 "DISPLAY","DEBUG","CHECK MATERIAL"。
            return false;  // 结束当前函数并返回 false。
        }  // 结束此条件分支。
        portENTER_CRITICAL(&jobMux);status.phase="ARM_FAILED";portEXIT_CRITICAL(&jobMux);  // 进入临界区，保护共享状态的本段访问。然后将 "ARM_FAILED" 写入或初始化 status.phase。然后离开临界区，保护共享状态的本段访问。
        stopHardware();finish(MotionState::FAILED,error);return false;  // 调用 stopHardware（停止Hardware），使用无参形式推进相应操作。然后调用 finish，传入 MotionState::FAILED,error。然后结束当前函数并返回 false。
    }  // 结束此条件分支。
    if(!join.armReady){join.armReady=true;span("ARM_PROGRAM",false);armSpanOpen=false;flag("ARM_READY");}  // 当 !join.armReady 成立时进入此分支，否则继续后续流程。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。
/**
 * @brief 选择导航终点航向：显式theta优先，否则按功能区节点确定。
 */
float endpointHeading(const MotionRequest &r) {  // 定义函数或成员 的入口 endpointHeading（endpoint航向），参数由此处的签名确定。
    if(r.kind==MotionKind::SCAN_TASK)return r.path[r.pathCount-2]==7?270.0f:NAN;  // 若 r.kind==MotionKind::SCAN_TASK，立即返回 r.path[r.pathCount-2]==7?270.0f:NAN。
    if(isfinite(r.theta))return r.theta;  // 若 isfinite(r.theta)，立即返回 r.theta。
    const uint8_t end=r.kind==MotionKind::NAVIGATE?r.endNode:r.path[r.pathCount-1];  // 将 r.kind==MotionKind::NAVIGATE?r.endNode:r.path[r.pathCo… 写入或初始化 end（终点）。
    return end==DISC_AREA_NODE?DISC_AREA_HEADING:end==COARSE_AREA_NODE?COARSE_AREA_HEADING:end==TEMP_AREA_NODE?TEMP_AREA_HEADING:NAN;  // 结束当前函数并返回 end==DISC_AREA_NODE?DISC_AREA_HEADING:end==COARSE_AREA…。
}  // 结束此函数或控制块。
void startTray(float angle) {  // 定义函数或成员 的入口 startTray（起点料盘），参数由此处的签名确定。
    motion::Step tray;  // 声明 tray（料盘），类型和可用范围由所在声明决定。
    tray.kind=motion::StepKind::TRAY_START;  // 将 motion::StepKind::TRAY_START 写入或初始化 tray.kind。
    tray.turret=angle;  // 将 angle 写入或初始化 tray.turret。
    hardware.issue(tray);  // 调用 hardware.issue，传入 tray。
}  // 结束此函数或控制块。

/**
 * @brief 准备正式节点路径并启动控制器，设置导航及机械臂合流条件；返回是否启动成功。
 */
void requestAnotherPath(const char *reason) {  // 定义函数或成员 的入口 requestAnotherPath（请求Another路径），参数由此处的签名确定。
    portENTER_CRITICAL(&nodePathMux);  // 进入临界区，保护共享状态的本段访问。
    nodePathState=NodePathState::WAITING;  // 将 NodePathState::WAITING 写入或初始化 nodePathState（节点路径状态）。
    nodePathRequestedAt=millis();  // 将 millis() 写入或初始化 nodePathRequestedAt（节点路径RequestedAt）。
    nodePathLen=0;  // 将 0 写入或初始化 nodePathLen（节点路径Len）。
    portEXIT_CRITICAL(&nodePathMux);  // 离开临界区，保护共享状态的本段访问。
    phase("PATH_WAIT");  // 调用 phase（阶段），传入 "PATH_WAIT"。
    stage=1;  // 将 1 写入或初始化 stage（阶段）。
    Serial.printf("{EVT,NAV,ROUTE_RETRY,%s}\n",reason);  // 调用 Serial.printf，传入 "{EVT,NAV,ROUTE_RETRY,%s}\n",reason。
    // 保留回传窗口。下一次 ROUTE_REQUEST 由 takeRequestedPath 发出。
}  // 结束此函数或控制块。

// 将作业请求中的路径和目标交给节点路径准备入口。
// 起始 Y 偏移随路径准备传入，用于衔接离开圆盘的行程。
bool beginPath(float startYOffset=0) {  // 定义函数或成员 的入口 beginPath（begin路径），参数由此处的签名确定。
    const auto &r=job.request;  // 将 job.request 写入或初始化 r。
    const bool scan=r.kind==MotionKind::SCAN_TASK;  // 将 r.kind==MotionKind::SCAN_TASK 写入或初始化 scan（扫码）。
    const bool preserveStartPosition=scan||(scanRetreated&&r.path[0]==SCAN_AREA_NODE);  // 将 scan||(scanRetreated&&r.path[0]==SCAN_AREA_NODE) 写入或初始化 preserveStartPosition（preserve起点位置）。
    float plannedHeading=0;  // 将 0 写入或初始化 plannedHeading（planned航向）。
    if(!H30PrepareNodePath(r.path,r.pathCount,r.kind==MotionKind::NODE_PATH?r.speed:NODE_PATH_SPEED_MM_S,  // 当 !H30PrepareNodePath(r.path,r.pathCount,r.kind==MotionK… 成立时进入此分支，否则继续后续流程。
        endpointHeading(r),r.automatic,startYOffset,scan?&plannedHeading:nullptr,preserveStartPosition)) {  // 调用 endpointHeading（endpoint航向），传入 r),r.automatic,startYOffset,scan?&plannedHeading:nullp…。
        if(RetryFormalRoute(r.kind,r.automatic)&&stage==1&&!cancelled())requestAnotherPath("START");  // 当 RetryFormalRoute(r.kind,r.automatic)&&stage==1&&!cance… 成立时进入此分支，否则继续后续流程。
        else {Serial.println("{EVT,NAV,ROUTE_FAILED,START}");finish(MotionState::FAILED,"START");}  // 前一个判断不成立时执行此分支。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此函数或控制块。
    char route[sizeof("{EVT,NAV,ROUTE_PATH,4294967295}")+MAX_NODE_PATH_LENGTH*3];  // 调用 sizeof，传入 "{EVT,NAV,ROUTE_PATH,4294967295}")+MAX_NODE_PATH_LENGT…。
    size_t length=snprintf(route,sizeof(route),"{EVT,NAV,ROUTE_PATH,%lu",static_cast<unsigned long>(job.id));  // 将 snprintf(route,sizeof(route),"{EVT,NAV,ROUTE_PATH,%lu"… 写入或初始化 length（长度）。
    for(uint8_t i=0;i<r.pathCount;++i)  // 按 uint8_t i=0;i<r.pathCount;++i 组织循环的初值、继续条件和步进。
        length+=snprintf(route+length,sizeof(route)-length,",%u",static_cast<unsigned>(r.path[i]));  // 把 snprintf(route+length,sizeof(route)-length,",%u",stati… 累加到 length（长度）。
    snprintf(route+length,sizeof(route)-length,"}");  // 调用 snprintf，传入 route+length,sizeof(route)-length,"}"。
    Serial.println(route);  // 调用 Serial.println，传入 route。
    phase("NAVIGATION");Serial.println("{EVT,NAV,ROUTE_RUNNING}");  // 调用 phase（阶段），传入 "NAVIGATION");Serial.println("{EVT,NAV,ROUTE_RUNNING}"。
    if(scan)xQueueReset(xScannerQueue);  // 当 scan 成立时进入此分支，否则继续后续流程。
    // 路上就转好扫码舵盘；终点航向沿用路径规划器选定的方向。
    if(scan)scanTrayAngle=continuous::wrap(plannedHeading+90);  // 当 scan 成立时进入此分支，否则继续后续流程。
    startTray(scan?scanTrayAngle:0);  // 调用 startTray（起点料盘），传入 scan?scanTrayAngle:0。
    ContinuousMotionBegin();stage=2;join.armReady=r.observation==ArmObservation::NONE;  // 调用 ContinuousMotionBegin，使用无参形式推进相应操作。然后将 2 写入或初始化 stage（阶段）。然后将 r.observation==ArmObservation::NONE 写入或初始化 join.armReady。
    return true;  // 结束当前函数并返回 true。
}  // 结束此初始化列表或局部作用域。
/**
 * @brief 非阻塞读取扫码队列，用parseTaskCode解析并登记currentTask；也接受已由串口登记的任务码。
 * @return true 已取得有效任务码，false 仍未收到。
 */
bool pollTaskCode() {  // 定义函数或成员 的入口 pollTaskCode（poll任务Code），参数由此处的签名确定。
    char code[SCANNER_BUF_LEN];  // 声明 code，类型和可用范围由所在声明决定。
    while(!taskReceived&&waitScannerCode(code,sizeof(code),0)) {  // 当 !taskReceived&&waitScannerCode(code,sizeof(code),0) 成立时继续循环。
        NormalizeScannerTaskCode(code);
        const auto task=parseTaskCode(code);  // 将 parseTaskCode(code) 写入或初始化 task（任务）。
        portENTER_CRITICAL(&taskCodeMux);if(!taskReceived&&task.valid){currentTask=task;taskReceived=true;}portEXIT_CRITICAL(&taskCodeMux);  // 将 task;taskReceived=true;}portEXIT_CRITICAL(&taskCodeMux) 写入或初始化 currentTask（当前任务）。
        Serial.printf("[SCANNER] task code %s\n",task.valid?"OK":"ERR");  // 调用 Serial.printf，传入 "[SCANNER] task code %s\n",task.valid?"OK":"ERR"。
    }  // 结束此循环体。
    return taskReceived;  // 结束当前函数并返回 taskReceived。
}  // 结束此函数或控制块。
void finishScan() {  // 定义函数或成员 的入口 finishScan（finish扫码），参数由此处的签名确定。
    startTray(0);  // 调用 startTray（起点料盘），传入 0。
    finish(MotionState::DONE);  // 调用 finish，传入 MotionState::DONE。
}  // 结束此函数或控制块。
/**
 * @brief 节点到达后等待有效任务码，必要时让料盘围绕扫码角最多摆动十轮。
 */
void scanAtNode(bool automatic=false) {  // 定义函数或成员 的入口 scanAtNode（扫码At节点），参数由此处的签名确定。
    const bool received=pollTaskCode();  // 将 pollTaskCode() 写入或初始化 received（已接收）。
    if(stage==SCAN_WAIT_CODE) {  // 当 stage==SCAN_WAIT_CODE 成立时进入此分支，否则继续后续流程。
        if(received)finishScan();  // 当 received 成立时进入此分支，否则继续后续流程。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    if(stage==SCAN_WAIT) {  // 当 stage==SCAN_WAIT 成立时进入此分支，否则继续后续流程。
        if(received){finishScan();return;}  // 当 received 成立时进入此分支，否则继续后续流程。
        if(millis()-scanArrivedAt<1000)return;  // 当 millis()-scanArrivedAt<1000 成立时进入此分支，否则继续后续流程。
        if(!BeginPositionMove(0,-100,0,200)){finish(MotionState::FAILED,"EXECUTION");return;}  // 当 !BeginPositionMove(0,-100,0,200) 成立时进入此分支，否则继续后续流程。
        stage=SCAN_RETREAT;  // 将 SCAN_RETREAT 写入或初始化 stage（阶段）。
        phase("SCAN_RETREAT");  // 调用 phase（阶段），传入 "SCAN_RETREAT"。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    if(stage==SCAN_RETREAT) {  // 当 stage==SCAN_RETREAT 成立时进入此分支，否则继续后续流程。
        TickPositionMove();  // 调用 TickPositionMove（轮询位置Move），使用无参形式推进相应操作。
        if(PositionMoveActive())return;  // 当 PositionMoveActive() 成立时进入此分支，否则继续后续流程。
        scanRetreated=true;  // 将 true 写入或初始化 scanRetreated（扫码Retreated）。
        if(received){finishScan();return;}  // 当 received 成立时进入此分支，否则继续后续流程。
        stage=SCAN_SWEEP;  // 将 SCAN_SWEEP 写入或初始化 stage（阶段）。
        phase("SCAN_SWEEP");  // 调用 phase（阶段），传入 "SCAN_SWEEP"。
    }  // 结束此条件分支。
    // 每段摆动留出 1000 ms，期间持续监听扫码。
    constexpr uint32_t travelMs=1000;  // 将 1000 写入或初始化 travelMs（travel毫秒）。
    if(scanTurn&&millis()-scanTurnAt<travelMs)return;  // 当 scanTurn&&millis()-scanTurnAt<travelMs 成立时进入此分支，否则继续后续流程。
    if(received&&scanTurn%2==0){finishScan();return;}  // 当 received&&scanTurn%2==0 成立时进入此分支，否则继续后续流程。
    constexpr uint8_t stepsPerRound=4,maxRounds=10;  // 将 4,maxRounds=10 写入或初始化 stepsPerRound（stepsPer轮次）。
    if(scanTurn==stepsPerRound*maxRounds) {  // 当 scanTurn==stepsPerRound*maxRounds 成立时进入此分支，否则继续后续流程。
        if(automatic) {  // 当 automatic 成立时进入此分支，否则继续后续流程。
            scanTurn=0;  // 将 0 写入或初始化 scanTurn（扫码Turn）。
            Serial.println("{EVT,TASK,SCAN_RETRY,RESWEEP}");  // 调用 Serial.println，传入 "{EVT,TASK,SCAN_RETRY,RESWEEP}"。
            updateDisplay("DISPLAY","DEBUG","QR RESCAN");  // 调用 updateDisplay，传入 "DISPLAY","DEBUG","QR RESCAN"。
        } else {  // 前一个判断不成立时执行此分支。
            stage=SCAN_WAIT_CODE;  // 将 SCAN_WAIT_CODE 写入或初始化 stage（阶段）。
            phase("SCAN_WAIT_CODE");  // 调用 phase（阶段），传入 "SCAN_WAIT_CODE"。
            Serial.println("{EVT,TASK,SCAN_TIMEOUT,WAIT_CODE}");  // 调用 Serial.println，传入 "{EVT,TASK,SCAN_TIMEOUT,WAIT_CODE}"。
            updateDisplay("DISPLAY","DEBUG","NO QR - WAIT CODE");  // 调用 updateDisplay，传入 "DISPLAY","DEBUG","NO QR - WAIT CODE"。
            return;  // 结束当前函数，将执行权交回调用者。
        }  // 结束此条件分支。
    }  // 结束此条件分支。

    constexpr float offsets[stepsPerRound]={50,0,-50,0};  // 将 {50,0,-50,0} 写入或初始化 offsets[stepsPerRound]。
    motion::Step tray;  // 声明 tray（料盘），类型和可用范围由所在声明决定。
    tray.kind=motion::StepKind::TRAY_ROTATE;  // 将 motion::StepKind::TRAY_ROTATE 写入或初始化 tray.kind。
    tray.turret=scanTrayAngle+offsets[scanTurn%stepsPerRound];  // 将 scanTrayAngle+offsets[scanTurn%stepsPerRound] 写入或初始化 tray.turret。
    tray.waitMs=travelMs;  // 将 travelMs 写入或初始化 tray.waitMs。
    hardware.issue(tray);  // 调用 hardware.issue，传入 tray。
    scanTurnAt=millis();  // 将 millis() 写入或初始化 scanTurnAt（扫码TurnAt）。
    ++scanTurn;  // 续写当前声明或运算，提供 ++scanTurn;。
}  // 结束此初始化列表或局部作用域。
bool discReturnArrived() {  // 定义函数或成员 的入口 discReturnArrived（圆盘ReturnArrived），参数由此处的签名确定。
    bool arrived=true;  // 将 true 写入或初始化 arrived。
    for(uint8_t id=1;id<=4;++id) {  // 按 uint8_t id=1;id<=4;++id 组织循环的初值、继续条件和步进。
        node_position::Motor motor;  // 声明 motor，类型和可用范围由所在声明决定。
        const char *error=ReadDiscReturnMotorFeedback(id,motor,cancelled);  // 将 ReadDiscReturnMotorFeedback(id,motor,cancelled) 写入或初始化 error（错误）。
        if(!error&&!(motor.flags&1))error="MOTOR_DISABLED";  // 当 !error&&!(motor.flags&1) 成立时进入此分支，否则继续后续流程。
        if(!error&&node_position::MOTOR_STALL_CHECK_ENABLED&&(motor.flags&12))error="MOTOR_STALL";  // 当 !error&&node_position::MOTOR_STALL_CHECK_ENABLED&&(mot… 成立时进入此分支，否则继续后续流程。
        if(error) {  // 当 error 成立时进入此分支，否则继续后续流程。
            Serial.printf("[DiscReturn] feedback motor=%u error=%s\n",id,error);  // 调用 Serial.printf，传入 "[DiscReturn] feedback motor=%u error=%s\n",id,error。
            stopHardware();  // 调用 stopHardware（停止Hardware），使用无参形式推进相应操作。
            finish(MotionState::FAILED,error);  // 调用 finish，传入 MotionState::FAILED,error。
            return false;  // 结束当前函数并返回 false。
        }  // 结束此条件分支。
        arrived=arrived&&node_position::encoderReturnArrived(motor,discReturnTargets[id-1]);  // 将 arrived&&node_position::encoderReturnArrived(motor,dis… 写入或初始化 arrived。
    }  // 结束此循环体。
    return arrived;  // 结束当前函数并返回 arrived。
}  // 结束此函数或控制块。
/**
 * @brief 消费已接收的正式路径，或结束超过回传期限的作业。
 */
void sendRouteRequest(const MotionRequest &request) {  // 定义函数或成员 的入口 sendRouteRequest（send路线请求），参数由此处的签名确定。
    Serial.println("{EVT,NAV,ROUTE_WAITING}");  // 调用 Serial.println，传入 "{EVT,NAV,ROUTE_WAITING}"。
    Serial.printf("{EVT,NAV,ROUTE_REQUEST,%u,%u}\n",request.startNode,request.endNode);  // 调用 Serial.printf，传入 "{EVT,NAV,ROUTE_REQUEST,%u,%u}\n",request.startNode,re…。
}  // 结束此函数或控制块。

bool takeRequestedPath(MotionRequest &request) {  // 定义函数或成员 的入口 takeRequestedPath（takeRequested路径），参数由此处的签名确定。
    portENTER_CRITICAL(&nodePathMux);  // 进入临界区，保护共享状态的本段访问。
    const auto state=nodePathState;  // 将 nodePathState 写入或初始化 state（状态）。
    const bool timedOut=state==NodePathState::WAITING  // 将 state==NodePathState::WAITING 写入或初始化 timedOut。
        && millis()-nodePathRequestedAt>=ROUTE_REPLY_TIMEOUT_MS;  // 继续上一行的组合条件，本行提供 && millis()-nodePathRequestedAt>=ROUTE_REPLY_TIMEOUT_MS;。
    const bool newBatch=timedOut&&RetryFormalRoute(request.kind,request.automatic)&&pathRetries==ROUTE_REPLY_MAX_RETRIES;  // 将 timedOut&&RetryFormalRoute(request.kind,request.automa… 写入或初始化 newBatch。
    const bool retry=timedOut&&(pathRetries<ROUTE_REPLY_MAX_RETRIES||newBatch);  // 将 timedOut&&(pathRetries<ROUTE_REPLY_MAX_RETRIES||newBat… 写入或初始化 retry（重试）。
    if(retry) {  // 当 retry 成立时进入此分支，否则继续后续流程。
        pathRetries=newBatch?0:pathRetries+1;  // 将 newBatch?0:pathRetries+1 写入或初始化 pathRetries（路径Retries）。
        nodePathRequestedAt=millis();  // 将 millis() 写入或初始化 nodePathRequestedAt（节点路径RequestedAt）。
    } else if(timedOut)nodePathState=NodePathState::FAILED;  // 仅在先前条件均不成立时，当 timedOut 成立时进入此分支，否则继续后续流程。
    if(state==NodePathState::RECEIVED) {  // 当 state==NodePathState::RECEIVED 成立时进入此分支，否则继续后续流程。
        request.pathCount=nodePathLen;  // 将 nodePathLen 写入或初始化 request.pathCount。
        memcpy(request.path,nodePathBuffer,nodePathLen);  // 调用 memcpy，传入 request.path,nodePathBuffer,nodePathLen。
        nodePathState=NodePathState::RUNNING;  // 将 NodePathState::RUNNING 写入或初始化 nodePathState（节点路径状态）。
    }  // 结束此条件分支。
    portEXIT_CRITICAL(&nodePathMux);  // 离开临界区，保护共享状态的本段访问。
    if(retry) {  // 当 retry 成立时进入此分支，否则继续后续流程。
        if(newBatch)Serial.println("{EVT,NAV,ROUTE_RETRY,TIMEOUT}");  // 当 newBatch 成立时进入此分支，否则继续后续流程。
        sendRouteRequest(request);  // 调用 sendRouteRequest（send路线请求），传入 request。
    } else if(timedOut) {  // 仅在先前条件均不成立时，当 timedOut 成立时进入此分支，否则继续后续流程。
        Serial.println("{EVT,NAV,ROUTE_FAILED,TIMEOUT}");  // 调用 Serial.println，传入 "{EVT,NAV,ROUTE_FAILED,TIMEOUT}"。
        finish(MotionState::FAILED,"PATH_TIMEOUT");  // 调用 finish，传入 MotionState::FAILED,"PATH_TIMEOUT"。
    } else if(state==NodePathState::FAILED) {  // 仅在先前条件均不成立时，当 state==NodePathState::FAILED 成立时进入此分支，否则继续后续流程。
        finish(MotionState::FAILED,"STOPPED");  // 调用 finish，传入 MotionState::FAILED,"STOPPED"。
    }  // 结束此条件分支。
    return state==NodePathState::RECEIVED;  // 结束当前函数并返回 state==NodePathState::RECEIVED。
}  // 结束此函数或控制块。
/**
 * @brief 推进导航作业：请求节点路径、执行底盘运动、按进站阶段提前启动观察姿态，等待两者合流完成。
 */
void navigationTick() {  // 定义函数或成员 的入口 navigationTick（navigation轮询），参数由此处的签名确定。
    auto &r=job.request;  // 将 job.request 写入或初始化 r。
    const bool scan=r.kind==MotionKind::SCAN_TASK;  // 将 r.kind==MotionKind::SCAN_TASK 写入或初始化 scan（扫码）。
    if(scan&&stage>=SCAN_WAIT){scanAtNode(r.automatic);return;}  // 当 scan&&stage>=SCAN_WAIT 成立时进入此分支，否则继续后续流程。
    if(stage==0&&(r.kind==MotionKind::NAVIGATE||scan)) {  // 当 stage==0&&(r.kind==MotionKind::NAVIGATE||scan) 成立时进入此分支，否则继续后续流程。
        portENTER_CRITICAL(&nodePathMux);  // 进入临界区，保护共享状态的本段访问。
        nodePathRequestedAt=millis();  // 将 millis() 写入或初始化 nodePathRequestedAt（节点路径RequestedAt）。
        nodePathState=NodePathState::WAITING;nodePathLen=0;  // 将 NodePathState::WAITING 写入或初始化 nodePathState（节点路径状态）。然后将 0 写入或初始化 nodePathLen（节点路径Len）。
        portEXIT_CRITICAL(&nodePathMux);  // 离开临界区，保护共享状态的本段访问。
        pathRetries=0;  // 将 0 写入或初始化 pathRetries（路径Retries）。
        sendRouteRequest(r);  // 调用 sendRouteRequest（send路线请求），传入 r。
        phase("PATH_WAIT");stage=1;return;  // 调用 phase（阶段），传入 "PATH_WAIT"。然后将 1 写入或初始化 stage（阶段）。然后结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    if(stage==1) {  // 当 stage==1 成立时进入此分支，否则继续后续流程。
        if(!takeRequestedPath(r))return;  // 当 !takeRequestedPath(r) 成立时进入此分支，否则继续后续流程。
        if(r.path[0]!=r.startNode||r.path[r.pathCount-1]!=r.endNode) {  // 当 r.path[0]!=r.startNode||r.path[r.pathCount-1]!=r.endNo… 成立时进入此分支，否则继续后续流程。
            if(RetryFormalRoute(r.kind,r.automatic))requestAnotherPath("ENDPOINT");  // 当 RetryFormalRoute(r.kind,r.automatic) 成立时进入此分支，否则继续后续流程。
            else {Serial.println("{EVT,NAV,ROUTE_FAILED,ENDPOINT}");finish(MotionState::FAILED,"ENDPOINT");}  // 前一个判断不成立时执行此分支。
            return;  // 结束当前函数，将执行权交回调用者。
        }  // 结束此条件分支。
        float startYOffset=0;  // 将 0 写入或初始化 startYOffset（起点纵向偏移）。
        if(r.discDeparture) {  // 当 r.discDeparture 成立时进入此分支，否则继续后续流程。
            if(!MergeDiscYIntoFirstStraight(r.path[1])) {  // 当 !MergeDiscYIntoFirstStraight(r.path[1]) 成立时进入此分支，否则继续后续流程。
                phase("DISC_RETURN_POSITION");  // 调用 phase（阶段），传入 "DISC_RETURN_POSITION"。
                if(const char *error=BeginDiscYReturn(discBefore,discBeforeHeading,ALIGN_PID_MAX_SPEED_RPM,  // 当 const char *error=BeginDiscYReturn(discBefore,discBefo… 成立时进入此分支，否则继续后续流程。
                    discReturnTargets,cancelled)) {  // 续写当前声明或运算，提供 discReturnTargets,cancelled)) {。
                    stopHardware();  // 调用 stopHardware（停止Hardware），使用无参形式推进相应操作。
                    finish(MotionState::FAILED,error);  // 调用 finish，传入 MotionState::FAILED,error。
                } else stage=3;  // 前一个判断不成立时执行此分支。
                return;  // 结束当前函数，将执行权交回调用者。
            }  // 结束此条件分支。
            phase("DISC_Y_OFFSET");  // 调用 phase（阶段），传入 "DISC_Y_OFFSET"。
            if(const char *error=ReadDiscYOffset(discBefore,discBeforeHeading,startYOffset,cancelled)) {  // 当 const char *error=ReadDiscYOffset(discBefore,discBefor… 成立时进入此分支，否则继续后续流程。
                if(!discMeasuredYOffsetValid||cancelled()) {  // 当 !discMeasuredYOffsetValid||cancelled() 成立时进入此分支，否则继续后续流程。
                    finish(MotionState::FAILED,error);  // 调用 finish，传入 MotionState::FAILED,error。
                    return;  // 结束当前函数，将执行权交回调用者。
                }  // 结束此条件分支。
                startYOffset=discMeasuredYOffset;  // 将 discMeasuredYOffset 写入或初始化 startYOffset（起点纵向偏移）。
                Serial.printf("{EVT,NAV,DISC_Y_OFFSET,CACHED,%.2f}\n",startYOffset);  // 调用 Serial.printf，传入 "{EVT,NAV,DISC_Y_OFFSET,CACHED,%.2f}\n",startYOffset。
            } else {  // 前一个判断不成立时执行此分支。
                discMeasuredYOffset=startYOffset;  // 将 startYOffset 写入或初始化 discMeasuredYOffset（圆盘Measured纵向偏移）。
                discMeasuredYOffsetValid=true;  // 将 true 写入或初始化 discMeasuredYOffsetValid（圆盘Measured纵向偏移有效）。
            }  // 结束此条件分支。
            Serial.printf("[Route] Disc start Y offset=%.1f mm, next node=%u\n",startYOffset,r.path[1]);  // 调用 Serial.printf，传入 "[Route] Disc start Y offset=%.1f mm, next node=%u\n",…。
        }  // 结束此条件分支。
        if(!beginPath(startYOffset))return;  // 当 !beginPath(startYOffset) 成立时进入此分支，否则继续后续流程。
    } else if(stage==3) {  // 仅在先前条件均不成立时，当 stage==3 成立时进入此分支，否则继续后续流程。
        if(!discReturnArrived()||!beginPath())return;  // 当 !discReturnArrived()||!beginPath() 成立时进入此分支，否则继续后续流程。
    } else if(stage==0) {if(!beginPath())return;}  // 仅在先前条件均不成立时，当 stage==0 成立时进入此分支，否则继续后续流程。
    if(!join.chassisDone) {  // 当 !join.chassisDone 成立时进入此分支，否则继续后续流程。
        if(scan)pollTaskCode();  // 当 scan 成立时进入此分支，否则继续后续流程。
        ContinuousMotionTick();  // 调用 ContinuousMotionTick（ContinuousMotion轮询），使用无参形式推进相应操作。
        if(!ContinuousMotionActive()) {  // 当 !ContinuousMotionActive() 成立时进入此分支，否则继续后续流程。
            if(ContinuousMotionFailure()){  // 当 ContinuousMotionFailure() 成立时进入此分支，否则继续后续流程。
                portENTER_CRITICAL(&jobMux);status.phase="NAVIGATION";portEXIT_CRITICAL(&jobMux);  // 进入临界区，保护共享状态的本段访问。然后将 "NAVIGATION" 写入或初始化 status.phase。然后离开临界区，保护共享状态的本段访问。
                finish(MotionState::FAILED,ContinuousMotionFailure());return;  // 调用 finish，传入 MotionState::FAILED,ContinuousMotionFailure()。然后结束当前函数，将执行权交回调用者。
            }  // 结束此条件分支。
            scanRetreated=false;  // 将 false 写入或初始化 scanRetreated（扫码Retreated）。
            join.chassisDone=true;flag("CHASSIS_DONE");  // 将 true 写入或初始化 join.chassisDone。然后调用 flag，传入 "CHASSIS_DONE"。
            if(activeSpan){span(activeSpan,false);activeSpan=nullptr;}  // 当 activeSpan 成立时进入此分支，否则继续后续流程。
            if(scan) {  // 当 scan 成立时进入此分支，否则继续后续流程。
                scanArrivedAt=millis();scanTurn=0;  // 将 millis() 写入或初始化 scanArrivedAt（扫码ArrivedAt）。然后将 0 写入或初始化 scanTurn（扫码Turn）。
                stage=SCAN_WAIT;phase("SCAN_WAIT");scanAtNode(r.automatic);return;  // 将 SCAN_WAIT 写入或初始化 stage（阶段）。然后调用 phase（阶段），传入 "SCAN_WAIT"。然后调用 scanAtNode（扫码At节点），传入 r.automatic。然后结束当前函数，将执行权交回调用者。
            }  // 结束此条件分支。
            if(r.kind==MotionKind::NAVIGATE&&r.cargo) {  // 当 r.kind==MotionKind::NAVIGATE&&r.cargo 成立时进入此分支，否则继续后续流程。
                hardware.preselectTray(cargoTrayAngles[r.cargo-1]);  // 调用 hardware.preselectTray，传入 cargoTrayAngles[r.cargo-1]。
                flag("TRAY_PRESELECT");  // 调用 flag，传入 "TRAY_PRESELECT"。
            }  // 结束此条件分支。
        }  // 结束此条件分支。
    }  // 结束此条件分支。
    auto &base=PositionNodeRunner();  // 将 PositionNodeRunner() 写入或初始化 base。
    const bool visualPose=r.observation==ArmObservation::LOOK||r.observation==ArmObservation::LOOK2;  // 将 r.observation==ArmObservation::LOOK||r.observation==Ar… 写入或初始化 visualPose。
    // 圆盘到粗加工横穿时保持载物台放料后的姿态，等末尾90度转向前再展开LOOK。
    const bool coarseArrivalLook=r.observation==ArmObservation::LOOK  // 将 r.observation==ArmObservation::LOOK 写入或初始化 coarseArrivalLook。
        &&r.path[0]==DISC_AREA_NODE&&r.path[r.pathCount-1]==COARSE_AREA_NODE;  // 继续上一行的组合条件，本行提供 &&r.path[0]==DISC_AREA_NODE&&r.path[r.pathCount-1]==COARSE_AREA_NODE;。
    // 返家LOOK4同样与终点转向并行；其他视觉姿态仍提前准备。
    const bool prepareArm=(r.observation==ArmObservation::LOOK4||coarseArrivalLook)  // 将 (r.observation==ArmObservation::LOOK4||coarseArrivalLo… 写入或初始化 prepareArm（prepare机械臂）。
        ?base.finalTurnApproach():visualPose&&base.observationApproach();  // 声明接口 base.finalTurnApproach，参数由此处的签名确定。
    if(r.observation!=ArmObservation::NONE&&join.shouldStartArm(join.chassisDone,prepareArm)) {  // 当 r.observation!=ArmObservation::NONE&&join.shouldStartA… 成立时进入此分支，否则继续后续流程。
        MotionRequest pose;pose.kind=MotionKind::ARM_POSE;pose.observation=r.observation;pose.zone=r.zone;  // 声明 pose，类型和可用范围由所在声明决定。然后将 MotionKind::ARM_POSE 写入或初始化 pose.kind。然后将 r.observation 写入或初始化 pose.observation。然后将 r.zone 写入或初始化 pose.zone。
        if(!beginArm(pose))return;  // 当 !beginArm(pose) 成立时进入此分支，否则继续后续流程。
    }  // 结束此条件分支。
    if(join.armStarted&&!join.armReady)tickArm();  // 当 join.armStarted&&!join.armReady 成立时进入此分支，否则继续后续流程。
    if(!running)return;  // 当 !running 成立时进入此分支，否则继续后续流程。
    portENTER_CRITICAL(&jobMux);  // 进入临界区，保护共享状态的本段访问。
    status.chassisDone=join.chassisDone;status.armStarted=join.armStarted;status.armReady=join.armReady;  // 将 join.chassisDone 写入或初始化 status.chassisDone。然后将 join.armStarted 写入或初始化 status.armStarted。然后将 join.armReady 写入或初始化 status.armReady。
    status.lastTranslation=base.lastTranslation();status.remainingMm=base.remainingDistance();status.terminalTurn=base.hasTerminalTurn();status.progress=base.progress();  // 将 base.lastTranslation() 写入或初始化 status.lastTranslation。然后将 base.remainingDistance() 写入或初始化 status.remainingMm。然后将 base.hasTerminalTurn() 写入或初始化 status.terminalTurn。然后将 base.progress() 写入或初始化 status.progress。
    portEXIT_CRITICAL(&jobMux);  // 离开临界区，保护共享状态的本段访问。
    if(join.complete())finish(MotionState::DONE);  // 当 join.complete() 成立时进入此分支，否则继续后续流程。
}  // 结束此函数或控制块。
/**
 * @brief 统一运动任务循环：领取已放行作业，推进导航、机械臂和视觉闭环。
 */
void task(void *) {  // 定义函数或成员 的入口 task（任务），参数由此处的签名确定。
    for(;;) {  // 按 ;; 组织循环的初值、继续条件和步进。
        if(!running) {  // 当 !running 成立时进入此分支，否则继续后续流程。
            portENTER_CRITICAL(&jobMux);bool stop=stopPending;stopPending=false;portEXIT_CRITICAL(&jobMux);  // 进入临界区，保护共享状态的本段访问。然后将 stopPending 写入或初始化 stop（停止）。然后将 false 写入或初始化 stopPending（停止Pending）。然后离开临界区，保护共享状态的本段访问。
            if(stop)stopHardware();  // 当 stop 成立时进入此分支，否则继续后续流程。
            Job pending;  // 声明 pending，类型和可用范围由所在声明决定。
            const bool queued=xQueuePeek(queue,&pending,0)==pdTRUE;  // 将 xQueuePeek(queue,&pending,0)==pdTRUE 写入或初始化 queued。
            portENTER_CRITICAL(&jobMux);  // 进入临界区，保护共享状态的本段访问。
            const bool released=queued&&launchGate.runnable(pending.id,pending.generation,ota,taskMotionAborted);  // 将 queued&&launchGate.runnable(pending.id,pending.generat… 写入或初始化 released。
            portEXIT_CRITICAL(&jobMux);  // 离开临界区，保护共享状态的本段访问。
            if(!released||xQueueReceive(queue,&job,0)!=pdTRUE) {  // 当 !released||xQueueReceive(queue,&job,0)!=pdTRUE 成立时进入此分支，否则继续后续流程。
                TurretMotorPoll();  // 调用 TurretMotorPoll（转台MotorPoll），使用无参形式推进相应操作。
                TickVisualAlignment();  // 调用 TickVisualAlignment（轮询VisualAlignment），使用无参形式推进相应操作。
                ulTaskNotifyTake(pdTRUE,(alignmentEnabled||TurretMotorTracking())?pdMS_TO_TICKS(TurretMotorNextDelay()):portMAX_DELAY);continue;  // 调用 ulTaskNotifyTake（ul任务NotifyTake），传入 pdTRUE,(alignmentEnabled||TurretMotorTracking())?pdMS_…。然后跳过本次剩余处理，开始下一轮循环。
            }  // 结束此条件分支。
            running=true;stage=0;join=motion::Join{};activeSpan=nullptr;armSpanOpen=false;  // 将 true;stage=0;join=motion::Join{};activeSpan=nullptr;ar… 写入或初始化 running。
            materialRetriedStep=motion::Program::CAPACITY;  // 将 motion::Program::CAPACITY 写入或初始化 materialRetriedStep（物料RetriedStep）。
            hardware.configure(job.id,cancelled);  // 调用 hardware.configure，传入 job.id,cancelled。
            setAlignmentEnabled(false);  // 调用 setAlignmentEnabled（setAlignment已启用），传入 false。
            TickVisualAlignment(); // 旧视觉停车在新位置目标装载前完成，不能在同步触发后覆盖它。
            portENTER_CRITICAL(&jobMux);status.state=MotionState::RUNNING;status.startedMs=millis();portEXIT_CRITICAL(&jobMux);  // 进入临界区，保护共享状态的本段访问。然后将 MotionState::RUNNING 写入或初始化 status.state。然后将 millis() 写入或初始化 status.startedMs。然后离开临界区，保护共享状态的本段访问。
        }  // 结束此条件分支。
        if(cancelled()){  // 当 cancelled() 成立时进入此分支，否则继续后续流程。
            portENTER_CRITICAL(&jobMux);stopPending=false;portEXIT_CRITICAL(&jobMux);  // 进入临界区，保护共享状态的本段访问。然后将 false 写入或初始化 stopPending（停止Pending）。然后离开临界区，保护共享状态的本段访问。
            stopHardware();finish(MotionState::CANCELLED,taskMotionAborted?"ABORTED":"STOPPED");continue;  // 调用 stopHardware（停止Hardware），使用无参形式推进相应操作。然后调用 finish，传入 MotionState::CANCELLED,taskMotionAborted?"ABORTED":"ST…。然后跳过本次剩余处理，开始下一轮循环。
        }  // 结束此条件分支。
        if(materialPaused) {  // 当 materialPaused 成立时进入此分支，否则继续后续流程。
            portENTER_CRITICAL(&jobMux);  // 进入临界区，保护共享状态的本段访问。
            const bool authorized=resumeMaterial;  // 将 resumeMaterial 写入或初始化 authorized。
            if(authorized) {  // 当 authorized 成立时进入此分支，否则继续后续流程。
                materialPaused=resumeMaterial=false;  // 将 resumeMaterial=false 写入或初始化 materialPaused（物料Paused）。
                status.phase="ARM_PREPARE";  // 将 "ARM_PREPARE" 写入或初始化 status.phase。
                status.reason=nullptr;  // 将 nullptr 写入或初始化 status.reason。
            }  // 结束此条件分支。
            portEXIT_CRITICAL(&jobMux);  // 离开临界区，保护共享状态的本段访问。
            if(!authorized) {  // 当 !authorized 成立时进入此分支，否则继续后续流程。
                ulTaskNotifyTake(pdTRUE,portMAX_DELAY);  // 调用 ulTaskNotifyTake（ul任务NotifyTake），传入 pdTRUE,portMAX_DELAY。
                continue;  // 跳过本次剩余处理，开始下一轮循环。
            }  // 结束此条件分支。
            discMeasuredYOffsetValid=false;  // 将 false 写入或初始化 discMeasuredYOffsetValid（圆盘Measured纵向偏移有效）。
            updateDisplay("DISPLAY","DEBUG","RESUME MATERIAL");  // 调用 updateDisplay，传入 "DISPLAY","DEBUG","RESUME MATERIAL"。
            arm.begin(program,hardware,materialFirstStep);  // 调用 arm.begin，传入 program,hardware,materialFirstStep。
            span("ARM_PROGRAM",true);armSpanOpen=true;  // 调用 span，传入 "ARM_PROGRAM",true。然后将 true 写入或初始化 armSpanOpen（机械臂SpanOpen）。
        }  // 结束此条件分支。
        TurretMotorPoll();  // 调用 TurretMotorPoll（转台MotorPoll），使用无参形式推进相应操作。
        const auto kind=job.request.kind;  // 将 job.request.kind 写入或初始化 kind。
        const bool navigation=kind==MotionKind::NAVIGATE||kind==MotionKind::NODE_PATH||kind==MotionKind::SCAN_TASK;  // 将 kind==MotionKind::NAVIGATE||kind==MotionKind::NODE_PAT… 写入或初始化 navigation。
        if(navigation)navigationTick();  // 当 navigation 成立时进入此分支，否则继续后续流程。
        else if(kind==MotionKind::DISC_RECORD_POSITION) {  // 仅在先前条件均不成立时，当 kind==MotionKind::DISC_RECORD_POSITION 成立时进入此分支，否则继续后续流程。
            phase("DISC_RECORD_POSITION");  // 调用 phase（阶段），传入 "DISC_RECORD_POSITION"。
            discMeasuredYOffsetValid=false;  // 将 false 写入或初始化 discMeasuredYOffsetValid（圆盘Measured纵向偏移有效）。
            const char *error=RecordDiscEncoderPositions(discBefore,cancelled);  // 将 RecordDiscEncoderPositions(discBefore,cancelled) 写入或初始化 error（错误）。
            if(!error)discBeforeHeading=currentPose.theta;  // 当 !error 成立时进入此分支，否则继续后续流程。
            finish(error?MotionState::FAILED:MotionState::DONE,error);  // 调用 finish，传入 error?MotionState::FAILED:MotionState::DONE,error。
        }  // 结束此条件分支。
        else if(kind==MotionKind::TURRET_TRIAL) {  // 仅在先前条件均不成立时，当 kind==MotionKind::TURRET_TRIAL 成立时进入此分支，否则继续后续流程。
            if(stage==0) {  // 当 stage==0 成立时进入此分支，否则继续后续流程。
                phase("TURRET_TRIAL");  // 调用 phase（阶段），传入 "TURRET_TRIAL"。
                const auto &r=job.request;  // 将 job.request 写入或初始化 r。
                if(!TurretMotorTrial(r.turret,r.speed,r.turretTrialRampMs,r.turretTrialPeriodMs,r.turretTrialDurationMs))finish(MotionState::FAILED,"MOTOR_READ");  // 当 !TurretMotorTrial(r.turret,r.speed,r.turretTrialRampMs… 成立时进入此分支，否则继续后续流程。
                else {currentArm.turret_angle=r.turret;stage=1;}  // 前一个判断不成立时执行此分支。
            } else if(!TurretMotorTracking())finish(MotionState::DONE);  // 仅在先前条件均不成立时，当 !TurretMotorTracking() 成立时进入此分支，否则继续后续流程。
        }  // 结束此条件分支。
        else if(kind==MotionKind::AXIS_TRIAL) {  // 仅在先前条件均不成立时，当 kind==MotionKind::AXIS_TRIAL 成立时进入此分支，否则继续后续流程。
            if(stage==0) {  // 当 stage==0 成立时进入此分支，否则继续后续流程。
                phase("AXIS_TRIAL");  // 调用 phase（阶段），传入 "AXIS_TRIAL"。
                const auto &r=job.request;  // 将 job.request 写入或初始化 r。
                const float target=r.axisTrialMotor==5?r.high:r.length;  // 将 r.axisTrialMotor==5?r.high:r.length 写入或初始化 target（目标）。
                const char *error=ArmAxisTrialStart(r.axisTrialMotor,target,r.axisTrialRpm,  // 将 ArmAxisTrialStart(r.axisTrialMotor,target,r.axisTrialR… 写入或初始化 error（错误）。
                    r.axisTrialRampMs,r.axisTrialPeriodMs,r.axisTrialDurationMs);  // 续写当前声明或运算，提供 r.axisTrialRampMs,r.axisTrialPeriodMs,r.axisTrialDurat…。
                if(error)finish(MotionState::FAILED,error);  // 当 error 成立时进入此分支，否则继续后续流程。
                else {  // 前一个判断不成立时执行此分支。
                    if(r.axisTrialMotor==5)currentArm.high=target;else currentArm.length=target;  // 当 r.axisTrialMotor==5 成立时进入此分支，否则继续后续流程。
                    stage=1;  // 将 1 写入或初始化 stage（阶段）。
                }  // 结束此条件分支。
            } else {  // 前一个判断不成立时执行此分支。
                ArmAxisTrialPoll();  // 调用 ArmAxisTrialPoll（机械臂AxisTrialPoll），使用无参形式推进相应操作。
                if(!ArmAxisTrialActive()) {  // 当 !ArmAxisTrialActive() 成立时进入此分支，否则继续后续流程。
                    const char *error=ArmAxisTrialError();  // 将 ArmAxisTrialError() 写入或初始化 error（错误）。
                    finish(error?MotionState::FAILED:MotionState::DONE,error);  // 调用 finish，传入 error?MotionState::FAILED:MotionState::DONE,error。
                }  // 结束此条件分支。
            }  // 结束此条件分支。
        }  // 结束此条件分支。
        else if(kind==MotionKind::TRIAL) {  // 仅在先前条件均不成立时，当 kind==MotionKind::TRIAL 成立时进入此分支，否则继续后续流程。
            if(stage==0){ContinuousMotionBegin();stage=1;phase("TRIAL");}  // 当 stage==0 成立时进入此分支，否则继续后续流程。
            ContinuousMotionTick();  // 调用 ContinuousMotionTick（ContinuousMotion轮询），使用无参形式推进相应操作。
            if(!ContinuousMotionActive())finish(ContinuousMotionFailure()?MotionState::FAILED:MotionState::DONE,ContinuousMotionFailure());  // 当 !ContinuousMotionActive() 成立时进入此分支，否则继续后续流程。
        } else {  // 前一个判断不成立时执行此分支。
            if(stage==0) {  // 当 stage==0 成立时进入此分支，否则继续后续流程。
                if(job.request.force&&kind==MotionKind::DISC_GRAB&&cargoPlatforms[job.request.cargo-1].material!=MATERIAL_NONE)  // 当 job.request.force&&kind==MotionKind::DISC_GRAB&&cargoP… 成立时进入此分支，否则继续后续流程。
                    Serial.printf("{EVT,VISION,FORCE_GRAB,WARN,OCCUPIED,%u,%u}\n",cargoPlatforms[job.request.cargo-1].material,job.request.cargo);  // 调用 Serial.printf，传入 "{EVT,VISION,FORCE_GRAB,WARN,OCCUPIED,%u,%u}\n",cargoP…。
                if(kind==MotionKind::DISC_GRAB&&job.request.automatic&&job.progress==0) {  // 当 kind==MotionKind::DISC_GRAB&&job.request.automatic&&jo… 成立时进入此分支，否则继续后续流程。
                    const char *error=ReadDiscYOffset(discBefore,discBeforeHeading,discMeasuredYOffset,cancelled);  // 将 ReadDiscYOffset(discBefore,discBeforeHeading,discMeasu… 写入或初始化 error（错误）。
                    discMeasuredYOffsetValid=!error;  // 将 !error 写入或初始化 discMeasuredYOffsetValid（圆盘Measured纵向偏移有效）。
                    if(error)Serial.printf("{EVT,NAV,DISC_Y_OFFSET,ERR,%s}\n",error);  // 当 error 成立时进入此分支，否则继续后续流程。
                }  // 结束此条件分支。
                if(beginArm(job.request))stage=1;  // 当 beginArm(job.request) 成立时进入此分支，否则继续后续流程。
            } else if(tickArm()&&running)finish(MotionState::DONE);  // 仅在先前条件均不成立时，当 tickArm()&&running 成立时进入此分支，否则继续后续流程。
        }  // 结束此条件分支。
        if(!running||materialPaused)continue;  // 若 !running||materialPaused，直接进入下一轮循环。
        if(kind!=MotionKind::NAVIGATE&&kind!=MotionKind::NODE_PATH)publishBaseEstimate();  // 当 kind!=MotionKind::NAVIGATE&&kind!=MotionKind::NODE_PATH 成立时进入此分支，否则继续后续流程。
        if(join.armStarted&&!join.armReady){portENTER_CRITICAL(&jobMux);status.phase=arm.phase();portEXIT_CRITICAL(&jobMux);}  // 当 join.armStarted&&!join.armReady 成立时进入此分支，否则继续后续流程。
        TickVisualAlignment();  // 调用 TickVisualAlignment（轮询VisualAlignment），使用无参形式推进相应操作。
        uint32_t delay=20;  // 将 20 写入或初始化 delay（延时）。
        if(kind==MotionKind::TURRET_TRIAL)delay=TurretMotorNextDelay();  // 当 kind==MotionKind::TURRET_TRIAL 成立时进入此分支，否则继续后续流程。
        if(kind==MotionKind::AXIS_TRIAL)delay=ArmAxisTrialNextDelay();  // 当 kind==MotionKind::AXIS_TRIAL 成立时进入此分支，否则继续后续流程。
        if(navigation&&stage==2&&!join.chassisDone)delay=ContinuousMotionNextDelay();  // 当 navigation&&stage==2&&!join.chassisDone 成立时进入此分支，否则继续后续流程。
        if(join.armStarted&&!join.armReady){const uint32_t next=arm.nextDelay();if(next<delay)delay=next;}  // 当 join.armStarted&&!join.armReady 成立时进入此分支，否则继续后续流程。
        if(kind==MotionKind::TRIAL)delay=ContinuousMotionNextDelay();  // 当 kind==MotionKind::TRIAL 成立时进入此分支，否则继续后续流程。
        // 通知可由新视觉帧、取消或控制请求提前唤醒；没有新增稳定窗口。
        ulTaskNotifyTake(pdTRUE,delay?pdMS_TO_TICKS(delay):1);  // 调用 ulTaskNotifyTake（ul任务NotifyTake），传入 pdTRUE,delay?pdMS_TO_TICKS(delay):1。
    }  // 结束此循环体。
}  // 结束此函数或控制块。
}  // 结束命名空间。
/**
 * @brief 创建统一运动任务及作业队列；后续底盘、机械臂和视觉闭环由该任务推进。
 * @return true 初始化成功，false 队列或任务创建失败。
 */
bool MotionExecutorInit() {  // 定义函数或成员 的入口 MotionExecutorInit，参数由此处的签名确定。
    if(worker)return true;  // 若 worker，立即返回 true。
    queue=xQueueCreate(1,sizeof(Job));if(!queue)return false;  // 将 xQueueCreate(1,sizeof(Job)) 写入或初始化 queue。然后若 !queue，立即返回 false。
    return xTaskCreate(task,"MotionExecutor",12288,nullptr,6,&worker)==pdPASS;  // 结束当前函数并返回 xTaskCreate(task,"MotionExecutor",12288,nullptr,6,&wor…。
}  // 结束此函数或控制块。
/**
 * @brief 通知运动任务立即检查待执行作业、取消请求或新视觉反馈。
 */
void WakeMotionExecutor(){if(worker)xTaskNotifyGive(worker);}  // 定义函数或成员 WakeMotionExecutor，参数由此处的签名确定。
/**
 * @brief 读取当前运动作业编号，供运动任务内部输出分项日志。
 */
uint32_t CurrentMotionId(){return job.id;}  // 定义 CurrentMotionId（当前Motion编号），调用时返回 job.id。
/**
 * @brief 按值复制动作请求到队列，不等待动作执行完成。
 * @param request 动作类型、目标位置及本轮任务数据。
 * @param launch true 立即允许执行；false 等串口回复 ACK 后调用 LaunchMotion 放行。
 * @return 非零作业编号；0 表示未提交。提交成功不等于动作完成。
 */
uint32_t SubmitMotion(const MotionRequest &request,bool launch) {  // 定义函数或成员 的入口 SubmitMotion，参数由此处的签名确定。
    return SubmitMotion(request,launch,nullptr);  // 结束当前函数并返回 SubmitMotion(request,launch,nullptr)。
}  // 结束此函数或控制块。
uint32_t SubmitMotion(const MotionRequest &request,bool launch,MotionSubmitError *error) {  // 定义函数或成员 的入口 SubmitMotion，参数由此处的签名确定。
    if(error)*error=MotionSubmitError::NONE;  // 当 error 成立时进入此分支，否则继续后续流程。
    Job queued;queued.request=request;queued.state=currentState;queued.progress=roundProgress;  // 声明 queued，类型和可用范围由所在声明决定。然后将 request 写入或初始化 queued.request。然后将 currentState 写入或初始化 queued.state。然后将 roundProgress 写入或初始化 queued.progress。
    portENTER_CRITICAL(&jobMux);  // 进入临界区，保护共享状态的本段访问。
    if(!worker||ota||taskMotionAborted) {  // 当 !worker||ota||taskMotionAborted 成立时进入此分支，否则继续后续流程。
        portEXIT_CRITICAL(&jobMux);  // 离开临界区，保护共享状态的本段访问。
        if(error)*error=MotionSubmitError::UNAVAILABLE;  // 当 error 成立时进入此分支，否则继续后续流程。
        return 0;  // 结束当前函数并返回 0。
    }  // 结束此条件分支。
    if(status.state==MotionState::RUNNING||status.state==MotionState::QUEUED) {  // 当 status.state==MotionState::RUNNING||status.state==Moti… 成立时进入此分支，否则继续后续流程。
        portEXIT_CRITICAL(&jobMux);  // 离开临界区，保护共享状态的本段访问。
        if(error)*error=MotionSubmitError::BUSY;  // 当 error 成立时进入此分支，否则继续后续流程。
        return 0;  // 结束当前函数并返回 0。
    }  // 结束此条件分支。
    queued.id=++nextId;queued.generation=launchGate.generation;status=MotionStatus{};status.id=queued.id;  // 将 ++nextId;queued.generation=launchGate.generation;statu… 写入或初始化 queued.id。
    status.submittedMs=millis();status.state=MotionState::QUEUED;status.phase="QUEUED";  // 将 millis() 写入或初始化 status.submittedMs。然后将 MotionState::QUEUED 写入或初始化 status.state。然后将 "QUEUED" 写入或初始化 status.phase。
    launchGate.prepare(queued.id,launch);  // 调用 launchGate.prepare，传入 queued.id,launch。
    portEXIT_CRITICAL(&jobMux);  // 离开临界区，保护共享状态的本段访问。
    if(xQueueSend(queue,&queued,0)!=pdTRUE) {  // 当 xQueueSend(queue,&queued,0)!=pdTRUE 成立时进入此分支，否则继续后续流程。
        portENTER_CRITICAL(&jobMux);  // 进入临界区，保护共享状态的本段访问。
        status.state=MotionState::FAILED;status.reason="QUEUE";  // 将 MotionState::FAILED 写入或初始化 status.state。然后将 "QUEUE" 写入或初始化 status.reason。
        portEXIT_CRITICAL(&jobMux);  // 离开临界区，保护共享状态的本段访问。
        if(error)*error=MotionSubmitError::QUEUE;  // 当 error 成立时进入此分支，否则继续后续流程。
        return 0;  // 结束当前函数并返回 0。
    }  // 结束此条件分支。
    WakeMotionExecutor();return queued.id;  // 调用 WakeMotionExecutor，使用无参形式推进相应操作。然后结束当前函数并返回 queued.id。
}  // 结束此函数或控制块。
/**
 * @brief 放行已入队的指定作业，并唤醒运动任务；用于保证串口 ACK 先于动作执行。
 * @param id SubmitMotion 返回的作业编号。
 */
void LaunchMotion(uint32_t id){portENTER_CRITICAL(&jobMux);if(status.id==id&&status.state==MotionState::QUEUED)launchGate.release(id);portEXIT_CRITICAL(&jobMux);WakeMotionExecutor();}  // 定义函数或成员 LaunchMotion，参数由此处的签名确定。
// 接续请求同时携带作业编号和本次等待的令牌，用来匹配当前等待窗口，避免把旧接续信号用于后续动作。
bool ResumeMaterialMotion(uint32_t id,uint32_t token) {  // 定义函数或成员 的入口 ResumeMaterialMotion（Resume物料Motion），参数由此处的签名确定。
    portENTER_CRITICAL(&jobMux);  // 进入临界区，保护共享状态的本段访问。
    const bool accepted=materialPaused&&!resumeMaterial&&status.id==id&&token==materialCheckToken  // 将 materialPaused&&!resumeMaterial&&status.id==id&&token=… 写入或初始化 accepted。
        &&status.state==MotionState::RUNNING  // 继续上一行的组合条件，本行提供 &&status.state==MotionState::RUNNING。
        &&!ota&&!taskMotionAborted&&!launchGate.cancelled(job.generation);  // 继续上一行的组合条件，本行提供 &&!ota&&!taskMotionAborted&&!launchGate.cancelled(job.generation);。
    if(accepted)resumeMaterial=true;  // 当 accepted 成立时进入此分支，否则继续后续流程。
    portEXIT_CRITICAL(&jobMux);  // 离开临界区，保护共享状态的本段访问。
    if(accepted)WakeMotionExecutor();  // 当 accepted 成立时进入此分支，否则继续后续流程。
    return accepted;  // 结束当前函数并返回 accepted。
}  // 结束此函数或控制块。
/**
 * @brief 读取指定作业的状态快照，可查询当前作业及最近保留的终态。
 * @param id 提交动作时取得的作业编号。
 * @return 含执行状态、阶段、错误原因及底盘/机械臂完成标志的快照。
 */
MotionStatus GetMotionStatus(uint32_t id) {  // 定义函数或成员 的入口 GetMotionStatus（GetMotion状态），参数由此处的签名确定。
    portENTER_CRITICAL(&jobMux);MotionStatus value;  // 进入临界区，保护共享状态的本段访问。然后声明 value（测值），类型和可用范围由所在声明决定。
    if(status.id==id)value=status;else for(const auto &entry:history)if(entry.id==id){value=entry;break;}  // 当 status.id==id 成立时进入此分支，否则继续后续流程。
    if(value.state==MotionState::RUNNING)value.elapsedMs=millis()-value.startedMs;  // 当 value.state==MotionState::RUNNING 成立时进入此分支，否则继续后续流程。
    portEXIT_CRITICAL(&jobMux);return value;  // 离开临界区，保护共享状态的本段访问。然后结束当前函数并返回 value。
}  // 结束此函数或控制块。
/**
 * @brief 判断是否存在排队中或执行中的运动作业。
 * @return true 表示执行器忙碌。
 */
bool MotionExecutorActive(){const auto s=snapshot();return s.state==MotionState::QUEUED||s.state==MotionState::RUNNING;}  // 将 snapshot();return s.state==MotionState::QUEUED||s.stat… 写入或初始化 s。
/**
 * @brief 取消当前作业代次并请求工作任务停车；旧作业不会在取消后继续执行。
 */
void CancelMotion(){  // 定义函数或成员 的入口 CancelMotion，参数由此处的签名确定。
    portENTER_CRITICAL(&jobMux);launchGate.cancel();stopPending=true;portEXIT_CRITICAL(&jobMux);  // 进入临界区，保护共享状态的本段访问。然后调用 launchGate.cancel。然后将 true 写入或初始化 stopPending（停止Pending）。然后离开临界区，保护共享状态的本段访问。
    ContinuousTrialCancel();setAlignmentEnabled(false);  // 调用 ContinuousTrialCancel，使用无参形式推进相应操作。然后调用 setAlignmentEnabled（setAlignment已启用），传入 false。
    if(discMaterialActive){firstDiscGrabReady=false;requestVisionStop(VisionStartMode::DISC_MATERIAL);}  // 当 discMaterialActive 成立时进入此分支，否则继续后续流程。
    // 显式停止/OTA 结束当前业务，不能在两个作业之间恢复旧阶段。
    // 沿用已有故障返回状态，不创建故障锁或自动重试。
    if(enableRun) {  // 当 enableRun 成立时进入此分支，否则继续后续流程。
        const RobotState state=currentState;  // 将 currentState 写入或初始化 state（状态）。
        if(state==STATE_READ_TASK)currentState=STATE_SCAN_FAILED;  // 当 state==STATE_READ_TASK 成立时进入此分支，否则继续后续流程。
        else if(state==STATE_GRAB_ROUND1||state==STATE_GRAB_ROUND2||state==STATE_PLACE_COARSE1||  // 仅在先前条件均不成立时，当 state==STATE_GRAB_ROUND1||state==STATE_GRAB_ROUND2||st… 成立时进入此分支，否则继续后续流程。
            state==STATE_PLACE_COARSE2||state==STATE_PLACE_TEMP1||state==STATE_STACK_TEMP2||state==STATE_RETURN_HOME)  // 继续上一行的组合条件，本行提供 state==STATE_PLACE_COARSE2||state==STATE_PLACE_TEMP1||state==STATE_STACK_TEMP2||state==ST…。
            currentState=STATE_TRANSFER_FAILED;  // 将 STATE_TRANSFER_FAILED 写入或初始化 currentState（当前状态）。
    }  // 结束此条件分支。
    WakeMotionExecutor();  // 调用 WakeMotionExecutor，使用无参形式推进相应操作。
}  // 结束此函数或控制块。
/**
 * @brief 设置 OTA 占用状态；进入 OTA 时取消运动作业。
 * @param active true 进入 OTA，false 退出 OTA。
 */
void MotionExecutorSetOta(bool active){portENTER_CRITICAL(&jobMux);ota=active;portEXIT_CRITICAL(&jobMux);if(active)CancelMotion();}  // 将 active;portEXIT_CRITICAL(&jobMux);if(active)CancelMoti… 写入或初始化 ota（固件更新）。
/**
 * @brief 阻塞当前调用任务，轮询指定作业直至终态；只供旧调试接口使用。
 * @param id 待等待的作业编号。
 * @return true 作业 DONE，false 编号无效或作业失败、取消。
 */
bool WaitMotion(uint32_t id){if(!id)return false;for(;;){const auto s=GetMotionStatus(id);if(s.complete())return s.state==MotionState::DONE;vTaskDelay(pdMS_TO_TICKS(10));}}  // 将 GetMotionStatus(id);if(s.complete())return s.state==Mo… 写入或初始化 s。
