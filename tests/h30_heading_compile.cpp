#include "../src/h30_heading.h"
constexpr bool verifyHeading() {
    H30HeadingReference ref;
    float angle = -1;
    if (ref.get(10, angle)) return false; // 不自动绑定第一帧
    if (!ref.calibrate(-68, 180) || !ref.get(-68, angle) || angle != 180) return false;
    if (!ref.get(22, angle) || angle != 270) return false;
    if (!ref.calibrate(179, 90) || !ref.get(-179, angle) || angle != 92) return false;
    if (!ref.get(177, angle) || angle != 88) return false;
    if (!ref.calibrate(10, 360) || !ref.get(9, angle) || angle != 359) return false;
    if (!ref.calibrate(0, 0) || !ref.get(-0.000001f, angle) || angle < 0 || angle >= 360) return false;
    ref.reset();
    if (ref.get(9, angle)) return false; // 拔出后旧参考失效
    if (ref.calibrate(NAN, 0) || ref.get(0, angle)) return false;
    if (ref.calibrate(0, INFINITY) || ref.get(0, angle)) return false;
    if (!ref.calibrate(50, 180)) return false;
    if (ref.calibrate(50, NAN) || ref.get(50, angle)) return false;
    return true;
}
static_assert(verifyHeading(), "heading anchor, wraparound, reset and invalid inputs");
