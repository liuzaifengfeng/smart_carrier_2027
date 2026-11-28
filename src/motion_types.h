// 定义调度器与请求端共享的运动类型。
// 请求描述要做什么，状态对象用于追踪作业编号、阶段和终态。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include <stdint.h>  // 引入 stdint.h 中的声明和定义。
#include <stddef.h>  // 引入 stddef.h 中的声明和定义。
#include "position_move_timing.h"  // 引入 position_move_timing.h 中的声明和定义。

enum class MotionSubmitError : uint8_t { NONE, BUSY, UNAVAILABLE, QUEUE };  // 定义 MotionSubmitError（MotionSubmit错误） 的类型成员或作用域。

enum class MotionState : uint8_t { QUEUED, RUNNING, DONE, FAILED, CANCELLED };  // 定义 MotionState（Motion状态） 的类型成员或作用域。
enum class MotionKind : uint8_t {  // 定义 MotionKind 的类型成员或作用域。
    NAVIGATE, NODE_PATH, ARM_POSE, DISC_GRAB, LOAD_ROUND, HELD_DISC, PLACE_ROUND, RETRIEVE_ROUND,  // 声明 RETRIEVE_ROUND（RETRIEVE轮次），类型和可用范围由所在声明决定。
    TRANSFER, RETRIEVE, DEMO_PLACE, DEMO_RETRIEVE, SCAN_TASK, LIDAR_POSE,  // 声明 LIDAR_POSE，类型和可用范围由所在声明决定。
    HOME_OBSERVE, HOME_RETURN, POSITION, GOTO, VELOCITY, ARM_RAW, SERVO_RAW, ENABLE, TRIAL, TURRET_TRIAL, AXIS_TRIAL, TRAY_ROTATE, RING_PREPARE,  // 声明 RING_PREPARE（圆环PREPARE），类型和可用范围由所在声明决定。
    DISC_RECORD_POSITION  // 续写当前声明或运算，提供 DISC_RECORD_POSITION。
};  // 结束此类型定义。
inline bool RetryFormalRoute(MotionKind kind,bool automatic) {  // 定义函数或成员 的入口 RetryFormalRoute（重试Formal路线），参数由此处的签名确定。
    return kind==MotionKind::NAVIGATE||(kind==MotionKind::SCAN_TASK&&automatic);  // 结束当前函数并返回 kind==MotionKind::NAVIGATE||(kind==MotionKind::SCAN_TA…。
}  // 结束此函数或控制块。
inline bool MergeDiscYIntoFirstStraight(uint8_t nextNode) {  // 定义函数或成员 的入口 MergeDiscYIntoFirstStraight（Merge圆盘纵向Into首个Straight），参数由此处的签名确定。
    return nextNode==5||nextNode==15;  // 结束当前函数并返回 nextNode==5||nextNode==15。
}  // 结束此函数或控制块。
enum class ArmObservation : uint8_t { NONE, START, LOOK, LOOK2, LOOK3, LOOK4 };  // 定义 ArmObservation（机械臂Observation） 的类型成员或作用域。

// 提交时按 kind 解释请求字段。
// 构造某类请求可以对照串口命令或比赛流程入口，再到执行器查看字段怎样传给子动作。
struct MotionRequest {  // 定义 MotionRequest（Motion请求） 的类型成员或作用域。
    MotionKind kind=MotionKind::ARM_POSE;  // 将 MotionKind::ARM_POSE 写入或初始化 kind。
    ArmObservation observation=ArmObservation::NONE;  // 将 ArmObservation::NONE 写入或初始化 observation。
    uint8_t startNode=0,endNode=0,path[25]={},pathCount=0;  // 将 0,endNode=0,path[25]={},pathCount=0 写入或初始化 startNode（起点节点）。
    uint8_t color=0,cargo=0,work=0,position=0,layer=1,zone=0; // NAVIGATE 的 cargo 是到站预选盘号。
    int colors[3]={},positions[3]={};  // 将 {},positions[3]={} 写入或初始化 colors[3]。
    float x=0,y=0,theta=0,speed=0,high=-1,length=-1,turret=-1,pawl=-1;  // 将 0,y=0,theta=0,speed=0,high=-1,length=-1,turret=-1,pawl… 写入或初始化 x（横向）。
    bool force=false,automatic=false,debug=false,planned=false;  // 将 false,automatic=false,debug=false,planned=false 写入或初始化 force。
    bool startVision=false;  // 将 false 写入或初始化 startVision（起点视觉）。
    bool discDeparture=false; // 正式圆盘抓取结束后，按路径首段决定如何补偿地图 Y。
    bool ringCorrection=false; // 确认圆环预放位，超时参数非零时启用视觉纠偏。
    bool armPrepared=false; // 返航导航已完成 LOOK4，HOME_OBSERVE 跳过机械臂准备。
    uint8_t vision=0;  // 将 0 写入或初始化 vision（视觉）。
    uint8_t positionAcceleration=POSITION_MOVE_DEFAULT_ACCELERATION;  // 将 POSITION_MOVE_DEFAULT_ACCELERATION 写入或初始化 positionAcceleration（位置加速度）。
    uint16_t turretTrialRampMs=100;  // 将 100 写入或初始化 turretTrialRampMs（转台TrialRamp毫秒）。
    uint32_t turretTrialPeriodMs=20,turretTrialDurationMs=3000;  // 将 20,turretTrialDurationMs=3000 写入或初始化 turretTrialPeriodMs（转台TrialPeriod毫秒）。
    uint8_t axisTrialMotor=0;  // 将 0 写入或初始化 axisTrialMotor。
    uint16_t axisTrialRpm=0,axisTrialRampMs=100;  // 将 0,axisTrialRampMs=100 写入或初始化 axisTrialRpm（axisTrial转速）。
    uint32_t axisTrialPeriodMs=20,axisTrialDurationMs=3000;  // 将 20,axisTrialDurationMs=3000 写入或初始化 axisTrialPeriodMs（axisTrialPeriod毫秒）。
    uint32_t trayIntervalMs=0;  // 将 0 写入或初始化 trayIntervalMs（料盘Interval毫秒）。
    uint16_t trayPowerMw=0;  // 将 0 写入或初始化 trayPowerMw（料盘功率Mw）。
    // 应答文本按值复制，不能保存串口栈上 frame.fields 的指针。
    char category[8]={},command[24]={};  // 将 {},command[24]={} 写入或初始化 category[8]。
};  // 结束此类型定义。
// 状态用于在提交端与执行任务之间传递进度，终态可由 complete() 统一判断，具体成功或失败再看 state。
struct MotionStatus {  // 定义 MotionStatus（Motion状态） 的类型成员或作用域。
    uint32_t id=0,submittedMs=0,startedMs=0,finishedMs=0,elapsedMs=0;  // 将 0,submittedMs=0,startedMs=0,finishedMs=0,elapsedMs=0 写入或初始化 id（编号）。
    MotionState state=MotionState::CANCELLED;  // 将 MotionState::CANCELLED 写入或初始化 state（状态）。
    const char *phase="IDLE",*reason=nullptr;  // 将 "IDLE",*reason=nullptr 写入或初始化 phase（阶段）。
    bool chassisDone=false,armStarted=false,armReady=false;  // 将 false,armStarted=false,armReady=false 写入或初始化 chassisDone（chassis完成）。
    bool lastTranslation=false,terminalTurn=false;  // 将 false,terminalTurn=false 写入或初始化 lastTranslation（最后Translation）。
    float remainingMm=0,progress=0;  // 将 0,progress=0 写入或初始化 remainingMm（剩余毫米）。
    /**
     * @brief 判断作业是否进入 DONE、FAILED 或 CANCELLED 终态；终态不一定表示成功。
     */
    bool complete() const {return state==MotionState::DONE||state==MotionState::FAILED||state==MotionState::CANCELLED;}  // 定义 complete，调用时返回 state==MotionState::DONE||state==MotionState::FAILED||…。
};  // 结束此类型定义。
/**
 * @brief 将运动状态转换为日志使用的固定英文名称。
 */
inline const char *MotionStateName(MotionState state) {  // 定义函数或成员 的入口 MotionStateName（Motion状态Name），参数由此处的签名确定。
    static const char *names[]={"QUEUED","RUNNING","DONE","FAILED","CANCELLED"};  // 声明 names，类型和可用范围由所在声明决定。
    return names[static_cast<uint8_t>(state)];  // 结束当前函数并返回 names[static_cast<uint8_t>(state)]。
}  // 结束此函数或控制块。
