#pragma once
#include "vision_alignment.h"

enum class AlignmentStartAction { RESET, KEEP, BUSY };

// NONE 表示旧版无模式命令；自动对齐期间沿用 owner，不切回 DISC。
constexpr AlignmentStartAction DecideAlignmentStart(bool automaticActive,
        bool automaticFailed, VisionStartMode owner, VisionStartMode requested) {
    return !automaticActive ? AlignmentStartAction::RESET
        : automaticFailed ? AlignmentStartAction::BUSY
        : (requested == VisionStartMode::NONE || requested == owner)
            ? AlignmentStartAction::KEEP : AlignmentStartAction::BUSY;
}
