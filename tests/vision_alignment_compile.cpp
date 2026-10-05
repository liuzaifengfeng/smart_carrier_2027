// 用 ESP32 编译器编译即可执行这些编译期断言，无需刷写硬件。
#include "vision_alignment.h"
#include <limits>

static_assert(VisionModeUsesAlignment(VisionStartMode::DISC), "DISC couples alignment");
static_assert(VisionModeUsesAlignment(VisionStartMode::WORK_AREA), "WORK_AREA couples alignment");
static_assert(VisionModeUsesAlignment(VisionStartMode::WORK_AREA_LOADED), "loaded area couples alignment");
static_assert(VisionModeUsesAlignment(VisionStartMode::CORNER), "CORNER couples alignment");
static_assert(!VisionModeUsesAlignment(VisionStartMode::DISC_MATERIAL), "material recognition stays separate");
static_assert(!VisionModeUsesAlignment(VisionStartMode::NONE), "no mode stays separate");

constexpr uint8_t feed(unsigned frames, uint8_t count = 0) {
    return frames == 0 ? count : feed(frames - 1, NextAlignmentFrameCount(count, true, true));
}
static_assert(feed(4) == 4, "four frames must not finish");
static_assert(feed(5) == AlignmentFrameWaiter::REQUIRED_FRAMES, "fifth frame finishes");
static_assert(feed(20) == 5, "counter saturates");
static_assert(NextAlignmentFrameCount(4, false, true) == 0, "outlier resets");
static_assert(NextAlignmentFrameCount(5, false, true) == 0, "outlier after completion resets");
static_assert(NextAlignmentFrameCount(4, true, false) == 1, "new mode starts at one");
static_assert(feed(4, NextAlignmentFrameCount(4, false, true)) == 4, "four new frames still wait");
constexpr VisionAlignmentDeadzone small{1, 2, 0.1f};
constexpr VisionAlignmentDeadzone large{3, 4, 0.2f};
static_assert(IsWithinAlignmentDeadzone(small, -0.1f, -1, 2), "inclusive signed boundaries");
static_assert(!IsWithinAlignmentDeadzone(small, 0, 1.01f, 0), "X outlier");
static_assert(!IsWithinAlignmentDeadzone(small, 0, 0, -2.01f), "Y outlier");
static_assert(!IsWithinAlignmentDeadzone(small, 0.11f, 0, 0), "angle outlier");
static_assert(IsWithinAlignmentDeadzone(large, 0.15f, 2, 3), "larger independent profile");
static_assert(!IsWithinAlignmentDeadzone(small, 0.15f, 2, 3), "smaller independent profile");
static_assert(IsWithinAlignmentDeadzone({0, 0, 0}, 0, 0, 0), "zero deadzone exact match");
static_assert(!IsWithinAlignmentDeadzone(small, std::numeric_limits<float>::infinity(), 0, 0), "infinity rejected");
static_assert(!IsWithinAlignmentDeadzone(small, 0, std::numeric_limits<float>::quiet_NaN(), 0), "NaN rejected");
static_assert(!IsWithinAlignmentDeadzone({-1, 2, 0.1f}, 0, 0, 0), "invalid deadzone rejected");
