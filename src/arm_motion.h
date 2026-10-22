#pragma once
#include "motion_types.h"
#include "motion_sequence.h"

// 只构建动作，不发令、不等待；角度规划所需的实测回读在运动任务内进行。
/**
 * @brief 将动作请求展开为机械臂步骤序列；不下发运动命令、不等待动作结束，转台路径规划会回读实际角度。
 * @param request 抓放料、观察姿态或原始控制请求。
 * @param program 输出步骤序列，后续由 motion::Sequence 和 MotionHardware 执行。
 * @return nullptr 构建成功，否则返回错误原因字符串。
 */
const char *BuildArmProgram(const MotionRequest &request,motion::Program &program);
