#pragma once
#include "motion_types.h"

bool MotionExecutorInit();
uint32_t SubmitMotion(const MotionRequest &request,bool launch=true);
void LaunchMotion(uint32_t id);
MotionStatus GetMotionStatus(uint32_t id);
bool MotionExecutorActive();
void CancelMotion();
void MotionExecutorSetOta(bool active);
void WakeMotionExecutor();
uint32_t CurrentMotionId(); // 仅供运动任务内部的分项日志使用。
// 同步接口保留给旧调试代码；正式状态机和串口任务不调用此入口。
bool WaitMotion(uint32_t id);
// 视觉任务只投递反馈；闭环发令由同一运动任务完成。
void TickVisualAlignment();

extern float MOTION_LOOK_LEAD_MM,MOTION_LOOK2_LEAD_MM;
extern uint32_t ARM_FEEDBACK_ENABLED,ARM_GRIP_HOLD_MS,ARM_RELEASE_HOLD_MS;
