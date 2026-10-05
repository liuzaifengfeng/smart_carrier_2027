#pragma once

#include <stdint.h>
#include <math.h>

enum class VisionStartMode : uint8_t {
    DISC, DISC_MATERIAL, WORK_AREA, WORK_AREA_LOADED, CORNER, NONE
};

constexpr bool VisionModeUsesAlignment(VisionStartMode mode) {
    return mode == VisionStartMode::DISC || mode == VisionStartMode::WORK_AREA
        || mode == VisionStartMode::WORK_AREA_LOADED || mode == VisionStartMode::CORNER;
}

struct VisionAlignmentDeadzone {
    float visualX;
    float visualY;
    float angleDeg;
};

constexpr bool IsWithinAlignmentDeadzone(const VisionAlignmentDeadzone &zone,
                                     float angle, float x, float y) {
    return isfinite(angle) && isfinite(x) && isfinite(y)
        && isfinite(zone.visualX) && isfinite(zone.visualY) && isfinite(zone.angleDeg)
        && zone.visualX >= 0 && zone.visualY >= 0 && zone.angleDeg >= 0
        && fabsf(x) <= zone.visualX && fabsf(y) <= zone.visualY
        && fabsf(angle) <= zone.angleDeg;
}

constexpr uint8_t NextAlignmentFrameCount(uint8_t count, bool withinDeadzone,
                                         bool sameMode) {
    return !withinDeadzone ? 0 : !sameMode ? 1 : count < 5 ? count + 1 : 5;
}

// 每个新反馈帧调用一次；重复轮询不得调用 update。
struct AlignmentFrameWaiter {
    static constexpr uint8_t REQUIRED_FRAMES = 5;
    uint8_t consecutiveFrames = 0;
    VisionStartMode mode = VisionStartMode::NONE;

    void reset() {
        consecutiveFrames = 0;
        mode = VisionStartMode::NONE;
    }

    bool update(VisionStartMode newMode, bool withinDeadzone) {
        consecutiveFrames = NextAlignmentFrameCount(consecutiveFrames,
            newMode != VisionStartMode::NONE && withinDeadzone, mode == newMode);
        mode = newMode;
        return consecutiveFrames >= REQUIRED_FRAMES;
    }
};
