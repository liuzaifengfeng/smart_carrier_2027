#pragma once
#include "motion_types.h"
#include "motion_sequence.h"

// 只构建动作，不发令、不等待；角度规划所需的实测回读在运动任务内进行。
const char *BuildArmProgram(const MotionRequest &request,motion::Program &program);
