#pragma once

#include <stdint.h>
#include <math.h>

enum class VisionStartMode : uint8_t {
    DISC, DISC_MATERIAL, WORK_AREA, WORK_AREA_LOADED, CORNER, NONE
};

/**
 * @brief 判断该视觉模式是否需要底盘对齐；单件圆盘识别不属于底盘整体对齐。
 */
constexpr bool VisionModeUsesAlignment(VisionStartMode mode) {
    return mode == VisionStartMode::DISC || mode == VisionStartMode::WORK_AREA
        || mode == VisionStartMode::WORK_AREA_LOADED || mode == VisionStartMode::CORNER;
}

struct VisionAlignmentDeadzone {
    float visualX;
    float visualY;
    float angleDeg;
};

/**
 * @brief 检查视觉X、Y及角度误差是否同时进入指定死区；XY单位沿用视觉输出。
 */
constexpr bool IsWithinAlignmentDeadzone(const VisionAlignmentDeadzone &zone, float angle, float x, float y) {
    return (x<0?-x:x) <= zone.visualX && (y<0?-y:y) <= zone.visualY
        && (angle<0?-angle:angle) <= zone.angleDeg;
}

constexpr uint8_t ALIGNMENT_REQUIRED_FRAMES = 2;

/**
 * @brief 根据本帧死区判定和模式是否连续，更新连续到位帧数。
 */
constexpr uint8_t NextAlignmentFrameCount(uint8_t count, bool withinDeadzone, bool sameMode) {
    return !withinDeadzone ? 0 : !sameMode ? 1
        : count < ALIGNMENT_REQUIRED_FRAMES ? count + 1 : ALIGNMENT_REQUIRED_FRAMES;
}

// 每个新反馈帧调用一次；重复轮询不得调用 update。
struct AlignmentFrameWaiter {
    static constexpr uint8_t REQUIRED_FRAMES = ALIGNMENT_REQUIRED_FRAMES;
    uint8_t consecutiveFrames = 0;
    VisionStartMode mode = VisionStartMode::NONE;

    /**
     * @brief 清空连续到位帧数与视觉模式，开始新一次对齐判定。
     */
    void reset() {
        consecutiveFrames = 0;
        mode = VisionStartMode::NONE;
    }

    /**
     * @brief 处理一帧新的视觉反馈，更新连续到位帧数；重复轮询同一帧不得调用。
     * @return true 连续到位帧数达到要求。
     */
    bool update(VisionStartMode newMode, bool withinDeadzone) {
        consecutiveFrames = NextAlignmentFrameCount(consecutiveFrames,
            newMode != VisionStartMode::NONE && withinDeadzone, mode == newMode);
        mode = newMode;
        return consecutiveFrames >= REQUIRED_FRAMES;
    }
};
