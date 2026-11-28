// 根据当前取料阶段、颜色和货位状态决定能否抓取。
// 这里保留判断结果供串口层回传，机械臂动作由后续作业执行。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。

#include <stdint.h>  // 引入 stdint.h 中的声明和定义。
#include <stddef.h>  // 引入 stddef.h 中的声明和定义。

struct DiscPositionCandidate {  // 定义 DiscPositionCandidate（圆盘位置Candidate） 的类型成员或作用域。
    uint8_t position;  // 声明 position（位置），类型和可用范围由所在声明决定。
    uint8_t color;  // 声明 color（颜色），类型和可用范围由所在声明决定。
};  // 结束此类型定义。

// 一帧是一次可抓快照；按下一件任务颜色选择，重复颜色时优先位置号较小者。
// 返回候选下标，-1 表示无匹配。位置号与载物台编号互相独立。
/**
 * @brief 按下一件任务颜色从视觉候选中选择圆盘位置，重复颜色优先位置号小者。
 * @return 候选数组下标，-1表示无匹配或候选数据无效。
 */
inline int SelectDiscPosition(const DiscPositionCandidate *candidates, size_t count, const int *colors, int progress, bool taskValid) {  // 定义函数或成员 的入口 SelectDiscPosition（Select圆盘位置），参数由此处的签名确定。
    if (!candidates || count == 0 || count > 3 || !colors || !taskValid  // 当 !candidates || count == 0 || count > 3 || !colors || !… 成立时进入此分支，否则继续后续流程。
            || progress < 0 || progress >= 3) return -1;  // 继续上一行的组合条件，本行提供 || progress < 0 || progress >= 3) return -1;。
    int selected = -1;  // 将 -1 写入或初始化 selected。
    for (size_t i = 0; i < count; ++i) {  // 按 size_t i = 0; i < count; ++i 组织循环的初值、继续条件和步进。
        if (candidates[i].position < 1 || candidates[i].position > 3  // 当 candidates[i].position < 1 || candidates[i].position >… 成立时进入此分支，否则继续后续流程。
                || candidates[i].color < 1 || candidates[i].color > 6) return -1;  // 继续上一行的组合条件，本行提供 || candidates[i].color < 1 || candidates[i].color > 6) return -1;。
        for (size_t j = 0; j < i; ++j)  // 按 size_t j = 0; j < i; ++j 组织循环的初值、继续条件和步进。
            if (candidates[j].position == candidates[i].position) return -1;  // 若 candidates[j].position == candidates[i].position，立即返回 -1。
        if (candidates[i].color == colors[progress]  // 当 candidates[i].color == colors[progress] 成立时进入此分支，否则继续后续流程。
                && (selected < 0 || candidates[i].position < candidates[selected].position))  // 继续上一行的组合条件，本行提供 && (selected < 0 || candidates[i].position < candidates[selected].position))。
            selected = static_cast<int>(i);  // 将 static_cast<int>(i) 写入或初始化 selected。
    }  // 结束此循环体。
    return selected;  // 结束当前函数并返回 selected。
}  // 结束此函数或控制块。

// 任务码规定抓取顺序，只匹配当前轮次下一件待抓颜色。
/**
 * @brief 判断识别颜色是否匹配当前轮下一件任务；不操作机械臂。
 */
constexpr bool ShouldGrabTaskColor(const int *colors, int progress, uint8_t color, bool taskValid) {  // 定义函数或成员 的入口 ShouldGrabTaskColor（ShouldGrab任务颜色），参数由此处的签名确定。
    return taskValid && colors != nullptr && progress >= 0 && progress < 3  // 结束当前函数并返回 taskValid && colors != nullptr && progress >= 0 && pro…。
        && color >= 1 && color <= 6 && colors[progress] == color;  // 继续上一行的组合条件，本行提供 && color >= 1 && color <= 6 && colors[progress] == color;。
}  // 结束此函数或控制块。

enum class DiscColorDecision {  // 定义 DiscColorDecision（圆盘颜色Decision） 的类型成员或作用域。
    GRAB, COLOR_MISMATCH, NO_TASK, NOT_WAITING, ROUND_COMPLETE,  // 声明 ROUND_COMPLETE（轮次COMPLETE），类型和可用范围由所在声明决定。
    ALIGN_ACTIVE, NOT_READY, OCCUPIED  // 声明 OCCUPIED，类型和可用范围由所在声明决定。
};  // 结束此类型定义。

// 每轮任务第 1、2、3 件分别对应 3、2、1 号载物台。
constexpr uint8_t CargoForGrabIndex(int index) { return 3 - index; }  // 定义函数或成员 CargoForGrabIndex（货位ForGrab下标），参数由此处的签名确定。

/**
 * @brief 仅当预期载物台的抓放作业完成时将进度加1，否则保持原进度。
 */
constexpr int AdvanceDiscGrabProgress(int progress, uint8_t cargo, bool completed) {  // 定义函数或成员 的入口 AdvanceDiscGrabProgress（Advance圆盘GrabProgress），参数由此处的签名确定。
    return completed && progress >= 0 && progress < 3 && cargo == CargoForGrabIndex(progress)  // 结束当前函数并返回 completed && progress >= 0 && progress < 3 && cargo ==…。
        ? progress + 1 : progress;  // 续写上一行表达式的条件取值分支。
}  // 结束此函数或控制块。

/**
 * @brief 结合任务、颜色、对齐、观察就绪和载物台占用状态，给出抓取或拒绝原因。
 */
constexpr DiscColorDecision EvaluateDiscColor(const int *colors, int progress, uint8_t color, bool taskValid, bool alignmentActive, bool ready, bool occupied) {  // 定义函数或成员 的入口 EvaluateDiscColor（Evaluate圆盘颜色），参数由此处的签名确定。
    return !taskValid ? DiscColorDecision::NO_TASK  // 结束当前函数并返回 !taskValid ? DiscColorDecision::NO_TASK。
        : colors == nullptr ? DiscColorDecision::NOT_WAITING  // 续写上一行表达式的条件取值分支。
        : progress < 0 || progress >= 3 ? DiscColorDecision::ROUND_COMPLETE  // 续写上一行表达式的条件取值分支。
        : !ShouldGrabTaskColor(colors, progress, color, taskValid) ? DiscColorDecision::COLOR_MISMATCH  // 定义函数或成员 ShouldGrabTaskColor（ShouldGrab任务颜色），参数由此处的签名确定。
        : alignmentActive ? DiscColorDecision::ALIGN_ACTIVE  // 续写上一行表达式的条件取值分支。
        : !ready ? DiscColorDecision::NOT_READY  // 续写上一行表达式的条件取值分支。
        : occupied ? DiscColorDecision::OCCUPIED  // 续写上一行表达式的条件取值分支。
        : DiscColorDecision::GRAB;  // 续写上一行表达式的条件取值分支。
}  // 结束此函数或控制块。
