#pragma once

#include <stdint.h>
#include <stddef.h>

struct DiscPositionCandidate {
    uint8_t position;
    uint8_t color;
};

// 一帧是一次可抓快照；按下一件任务颜色选择，重复颜色时优先位置号较小者。
// 返回候选下标，-1 表示无匹配。位置号与载物台编号互相独立。
/**
 * @brief 按下一件任务颜色从视觉候选中选择圆盘位置，重复颜色优先位置号小者。
 * @return 候选数组下标，-1表示无匹配或候选数据无效。
 */
inline int SelectDiscPosition(const DiscPositionCandidate *candidates, size_t count, const int *colors, int progress, bool taskValid) {
    if (!candidates || count == 0 || count > 3 || !colors || !taskValid
            || progress < 0 || progress >= 3) return -1;
    int selected = -1;
    for (size_t i = 0; i < count; ++i) {
        if (candidates[i].position < 1 || candidates[i].position > 3
                || candidates[i].color < 1 || candidates[i].color > 6) return -1;
        for (size_t j = 0; j < i; ++j)
            if (candidates[j].position == candidates[i].position) return -1;
        if (candidates[i].color == colors[progress]
                && (selected < 0 || candidates[i].position < candidates[selected].position))
            selected = static_cast<int>(i);
    }
    return selected;
}

// 任务码规定抓取顺序，只匹配当前轮次下一件待抓颜色。
/**
 * @brief 判断识别颜色是否匹配当前轮下一件任务；不操作机械臂。
 */
constexpr bool ShouldGrabTaskColor(const int *colors, int progress, uint8_t color, bool taskValid) {
    return taskValid && colors != nullptr && progress >= 0 && progress < 3
        && color >= 1 && color <= 6 && colors[progress] == color;
}

enum class DiscColorDecision {
    GRAB, COLOR_MISMATCH, NO_TASK, NOT_WAITING, ROUND_COMPLETE,
    ALIGN_ACTIVE, NOT_READY, OCCUPIED
};

/**
 * @brief 仅当预期载物台的抓放作业完成时将进度加1，否则保持原进度。
 */
constexpr int AdvanceDiscGrabProgress(int progress, uint8_t cargo, bool completed) {
    return completed && progress >= 0 && progress < 3 && cargo == progress + 1
        ? progress + 1 : progress;
}

/**
 * @brief 结合任务、颜色、对齐、观察就绪和载物台占用状态，给出抓取或拒绝原因。
 */
constexpr DiscColorDecision EvaluateDiscColor(const int *colors, int progress, uint8_t color, bool taskValid, bool alignmentActive, bool ready, bool occupied) {
    return !taskValid ? DiscColorDecision::NO_TASK
        : colors == nullptr ? DiscColorDecision::NOT_WAITING
        : progress < 0 || progress >= 3 ? DiscColorDecision::ROUND_COMPLETE
        : !ShouldGrabTaskColor(colors, progress, color, taskValid) ? DiscColorDecision::COLOR_MISMATCH
        : alignmentActive ? DiscColorDecision::ALIGN_ACTIVE
        : !ready ? DiscColorDecision::NOT_READY
        : occupied ? DiscColorDecision::OCCUPIED
        : DiscColorDecision::GRAB;
}
