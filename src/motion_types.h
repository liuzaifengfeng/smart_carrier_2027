#pragma once
#include <stdint.h>
#include <stddef.h>

enum class MotionState : uint8_t { QUEUED, RUNNING, DONE, FAILED, CANCELLED };
enum class MotionKind : uint8_t {
    NAVIGATE, NODE_PATH, ARM_POSE, DISC_GRAB, LOAD_ROUND, HELD_DISC, PLACE_ROUND, RETRIEVE_ROUND,
    TRANSFER, RETRIEVE, DEMO_PLACE, DEMO_RETRIEVE, SCAN_TASK, LIDAR_POSE,
    HOME_OBSERVE, HOME_RETURN, POSITION, GOTO, VELOCITY, ARM_RAW, SERVO_RAW, ENABLE, TRIAL, TURRET_TRIAL
};
enum class ArmObservation : uint8_t { NONE, START, LOOK, LOOK2, LOOK3, LOOK4 };

struct MotionRequest {
    MotionKind kind=MotionKind::ARM_POSE;
    ArmObservation observation=ArmObservation::NONE;
    uint8_t startNode=0,endNode=0,path[25]={},pathCount=0;
    uint8_t color=0,cargo=0,work=0,position=0,layer=1,zone=0;
    int colors[3]={},positions[3]={};
    float x=0,y=0,theta=0,speed=0,high=-1,length=-1,turret=-1,pawl=-1;
    bool force=false,automatic=false,debug=false,planned=false;
    bool startVision=false;
    bool armPrepared=false; // 返航导航已完成 LOOK4，HOME_OBSERVE 只执行原有航向修正。
    uint8_t vision=0;
    uint16_t turretTrialRampMs=100;
    uint32_t turretTrialPeriodMs=20,turretTrialDurationMs=3000;
    // 应答文本按值复制，不能保存串口栈上 frame.fields 的指针。
    char category[8]={},command[24]={};
};
struct MotionStatus {
    uint32_t id=0,submittedMs=0,startedMs=0,finishedMs=0,elapsedMs=0;
    MotionState state=MotionState::CANCELLED;
    const char *phase="IDLE",*reason=nullptr;
    bool chassisDone=false,armStarted=false,armReady=false;
    bool lastTranslation=false,terminalTurn=false;
    float remainingMm=0,progress=0;
    /**
     * @brief 判断作业是否进入 DONE、FAILED 或 CANCELLED 终态；终态不一定表示成功。
     */
    bool complete() const {return state==MotionState::DONE||state==MotionState::FAILED||state==MotionState::CANCELLED;}
};
/**
 * @brief 将运动状态转换为日志使用的固定英文名称。
 */
inline const char *MotionStateName(MotionState state) {
    static const char *names[]={"QUEUED","RUNNING","DONE","FAILED","CANCELLED"};
    return names[static_cast<uint8_t>(state)];
}
