#pragma once
#include <math.h>
#include <stdint.h>
#if __cplusplus >= 201402L
#define H30_HEADING_CONSTEXPR constexpr
#else
#define H30_HEADING_CONSTEXPR
#endif

// 平面航向参考。绕 Y 倒装不会自动要求将欧拉 Yaw 取负。
// 以标准 Yaw 正方向为正；实车需验证左转时地图航向递增。
class H30HeadingReference {
public:
    H30_HEADING_CONSTEXPR void reset() { calibrated_ = false; }
    H30_HEADING_CONSTEXPR bool calibrate(float sensorYaw, float mapHeading) {
        reset();
        if (!validYaw(sensorYaw) || !isfinite(mapHeading)
                || mapHeading < 0 || mapHeading > 360) return false;
        sensorAtReference_ = normalize(sensorYaw);
        mapAtReference_ = normalize(mapHeading);
        calibrated_ = true;
        return true;
    }
    H30_HEADING_CONSTEXPR bool get(float sensorYaw, float &heading) const {
        if (!calibrated_ || !validYaw(sensorYaw)) return false;
        heading = normalize(mapAtReference_ + normalize(sensorYaw) - sensorAtReference_);
        return true;
    }
private:
    bool calibrated_ = false;
    float sensorAtReference_ = 0, mapAtReference_ = 0;
    static H30_HEADING_CONSTEXPR bool validYaw(float yaw) {
        // YIS int32 * 1e-6 的整个可表示范围；也使下方整数商转换有界。
        return isfinite(yaw) && yaw >= -2148 && yaw <= 2148;
    }
    static H30_HEADING_CONSTEXPR float normalize(float angle) {
        const float wrapped = angle - int32_t(angle / 360.0f) * 360.0f;
        const float positive = wrapped < 0 ? wrapped + 360.0f : wrapped;
        return positive >= 360.0f ? 0.0f : positive;
    }
};
#undef H30_HEADING_CONSTEXPR
