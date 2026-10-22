#pragma once
#include "vision_alignment.h"

enum class AlignmentStartAction { RESET, KEEP, BUSY };

// NONE 表示旧版无模式命令；自动对齐期间沿用 owner，不切回 DISC。
/**
 * @brief 根据当前自动对齐及拥有者，决定新视觉对齐请求是复用、拒绝还是重新启动；不操作硬件。
 */
constexpr AlignmentStartAction DecideAlignmentStart(bool automaticActive, bool automaticFailed, VisionStartMode owner, VisionStartMode requested) {
    return !automaticActive ? AlignmentStartAction::RESET
        : automaticFailed ? AlignmentStartAction::BUSY
        : (requested == VisionStartMode::NONE || requested == owner)
            ? AlignmentStartAction::KEEP : AlignmentStartAction::BUSY;
}
