// 集中表达视觉对齐启动策略。
// 修改启动条件时可先检查这里的纯判断，再追踪 robot_runtime 中负责切换会话的调用。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include "vision_alignment.h"  // 引入 vision_alignment.h 中的声明和定义。

enum class AlignmentStartAction { RESET, KEEP, BUSY };  // 定义 AlignmentStartAction（Alignment起点Action） 的类型成员或作用域。

// NONE 表示旧版无模式命令；自动对齐期间沿用 owner，不切回 DISC。
/**
 * @brief 根据当前自动对齐及拥有者，决定新视觉对齐请求是复用、拒绝还是重新启动；不操作硬件。
 */
constexpr AlignmentStartAction DecideAlignmentStart(bool automaticActive, bool automaticFailed, VisionStartMode owner, VisionStartMode requested) {  // 定义函数或成员 的入口 DecideAlignmentStart（DecideAlignment起点），参数由此处的签名确定。
    return !automaticActive ? AlignmentStartAction::RESET  // 结束当前函数并返回 !automaticActive ? AlignmentStartAction::RESET。
        : automaticFailed ? AlignmentStartAction::BUSY  // 续写上一行表达式的条件取值分支。
        : (requested == VisionStartMode::NONE || requested == owner)  // 续写上一行表达式的条件取值分支。
            ? AlignmentStartAction::KEEP : AlignmentStartAction::BUSY;  // 续写上一行表达式的条件取值分支。
}  // 结束此函数或控制块。
