#pragma once
#include "continuous_route.h"

bool ContinuousTrialInit();
bool ContinuousTrialActive();
const char *ContinuousTrialBegin(const continuous::Config &config);
const char *ContinuousTrialPolicy(uint32_t id,uint8_t policy,float outset,float margin,uint32_t geometry);
const char *ContinuousTrialNode(uint32_t id,uint8_t index,uint8_t node,float radius,float expectedX,float expectedY);
// Prepare 之后即为 RUNNING，串口先回复 ACK，再 Launch 通知工作任务。
const char *ContinuousTrialPrepare(uint32_t id);
void ContinuousTrialLaunch();
const char *ContinuousTrialStop(uint32_t id);
const char *ContinuousTrialStatus(uint32_t id); // 输出状态，同时续约 1500ms 心跳
// OTA 等外部取消源；不在调用方操作电机。
void ContinuousTrialCancel();
void ContinuousTrialSetOta(bool active);
