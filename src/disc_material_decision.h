#pragma once

#include <stdint.h>

// 任务码规定抓取顺序，只匹配当前轮次下一件待抓颜色。
constexpr bool ShouldGrabTaskColor(const int *colors, int progress,
                                   uint8_t color, bool taskValid) {
    return taskValid && colors != nullptr && progress >= 0 && progress < 3
        && color >= 1 && color <= 6 && colors[progress] == color;
}

enum class DiscColorDecision {
    GRAB, COLOR_MISMATCH, NO_TASK, NOT_WAITING, ROUND_COMPLETE,
    ALIGN_ACTIVE, NOT_READY, OCCUPIED
};

constexpr int AdvanceDiscGrabProgress(int progress, uint8_t cargo, bool completed) {
    return completed && progress >= 0 && progress < 3 && cargo == progress + 1
        ? progress + 1 : progress;
}

constexpr DiscColorDecision EvaluateDiscColor(const int *colors, int progress,
                                            uint8_t color, bool taskValid,
                                            bool alignmentActive, bool ready,
                                            bool occupied) {
    return !taskValid ? DiscColorDecision::NO_TASK
        : colors == nullptr ? DiscColorDecision::NOT_WAITING
        : progress < 0 || progress >= 3 ? DiscColorDecision::ROUND_COMPLETE
        : !ShouldGrabTaskColor(colors, progress, color, taskValid) ? DiscColorDecision::COLOR_MISMATCH
        : alignmentActive ? DiscColorDecision::ALIGN_ACTIVE
        : !ready ? DiscColorDecision::NOT_READY
        : occupied ? DiscColorDecision::OCCUPIED
        : DiscColorDecision::GRAB;
}
