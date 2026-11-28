// 机械臂动作编排入口。
// 请求描述动作意图，Program 保存展开后的步骤，执行器随后按步骤推进。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include "motion_types.h"  // 引入 motion_types.h 中的声明和定义。
#include "motion_sequence.h"  // 引入 motion_sequence.h 中的声明和定义。

// 只构建动作，不发令、不等待；角度规划所需的实测回读在运动任务内进行。
/**
 * @brief 将动作请求展开为机械臂步骤序列；不下发运动命令、不等待动作结束，转台路径规划会回读实际角度。
 * @param request 抓放料、观察姿态或原始控制请求。
 * @param program 输出步骤序列，后续由 motion::Sequence 和 MotionHardware 执行。
 * @return nullptr 构建成功，否则返回错误原因字符串。
 */
const char *BuildArmProgram(const MotionRequest &request,motion::Program &program);  // 声明接口 BuildArmProgram（Build机械臂Program），参数由此处的签名确定。
