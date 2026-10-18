#pragma once
#include "node_position_control.h"
// 调用者持有底盘运动锁；不会用估计时间替代驱动器反馈宣布成功。
const char *RunPositionNodeRoute(const continuous::Route &route,const node_position::Settings &settings,
    uint32_t referenceVersion,bool (*cancelled)(),node_position::Result &result);
