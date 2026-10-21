#pragma once
#include "node_position_control.h"
#include "scan_motion_control.h"
const char *RunContinuousScan(const ScanMotionSettings &settings,bool (*received)(),float &forwardTravel);
bool ReadChassisEncoderPositions(int64_t (&positions)[4]);
// 调用者持有底盘运动锁；不会用估计时间替代驱动器反馈宣布成功。
const char *RunPositionNodeRoute(const continuous::Route &route,const node_position::Settings &settings,
    uint32_t referenceVersion,bool (*cancelled)(),node_position::Result &result);
void BeginPositionNodeRoute(const continuous::Route &route,const node_position::Settings &settings,
    uint32_t referenceVersion,bool (*cancelled)(),node_position::Result &result);
void TickPositionNodeRoute();
node_position::Runner &PositionNodeRunner();
const char *ReadMotorFeedback(uint8_t id,node_position::Motor &state);
void BeginContinuousScan(const ScanMotionSettings &settings,bool (*received)());
void TickContinuousScan();
ScanMotionRunner &ContinuousScanRunner();
