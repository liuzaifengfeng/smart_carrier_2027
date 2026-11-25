#pragma once
#include <algorithm>
#include <math.h>
#include <stdint.h>
#include "h30_motion.h"
#if __cplusplus >= 201402L
#define H30_HEADING_CONSTEXPR constexpr
#else
#define H30_HEADING_CONSTEXPR
#endif

// 平面航向参考。绕 Y 倒装不会自动要求将欧拉 Yaw 取负。
// 以标准 Yaw 正方向为正；实车需验证左转时地图航向递增。
class H30HeadingReference {
public:
    /**
     * @brief 清除地图航向标定状态，不保留旧参考继续换算。
     */
    H30_HEADING_CONSTEXPR void reset() { calibrated_ = false; }
    /**
     * @brief 将传感器Yaw与已知地图航向绑定为同一时刻的参考；角度单位deg。
     * @return true 参考建立成功，false 输入无效。
     */
    H30_HEADING_CONSTEXPR bool calibrate(float sensorYaw, float mapHeading) {
        reset();
        if (!validYaw(sensorYaw) || !isfinite(mapHeading)
                || mapHeading < 0 || mapHeading > 360) return false;
        sensorAtReference_ = normalize(sensorYaw);
        mapAtReference_ = normalize(mapHeading);
        calibrated_ = true;
        return true;
    }
    /**
     * @brief 使用当前参考将原始Yaw换算为0~360度地图航向，结果写入heading。
     * @return true 换算成功，false 尚未标定或输入无效。
     */
    H30_HEADING_CONSTEXPR bool get(float sensorYaw, float &heading) const {
        if (!calibrated_ || !validYaw(sensorYaw)) return false;
        heading = normalize(mapAtReference_ + normalize(sensorYaw) - sensorAtReference_);
        return true;
    }
private:
    bool calibrated_ = false;
    float sensorAtReference_ = 0, mapAtReference_ = 0;
    /**
     * @brief 检查原始Yaw是否有限且处于YIS角度字段可表示范围。
     */
    static H30_HEADING_CONSTEXPR bool validYaw(float yaw) {
        // YIS int32 * 1e-6 的整个可表示范围；也使下方整数商转换有界。
        return isfinite(yaw) && yaw >= -2148 && yaw <= 2148;
    }
    /**
     * @brief 将角度归一化到0~360度范围。
     */
    static H30_HEADING_CONSTEXPR float normalize(float angle) {
        const float wrapped = angle - int32_t(angle / 360.0f) * 360.0f;
        const float positive = wrapped < 0 ? wrapped + 360.0f : wrapped;
        return positive >= 360.0f ? 0.0f : positive;
    }
};
#undef H30_HEADING_CONSTEXPR

/** @brief 在固定3秒窗口内筛选近静止样本，保留圆周中位数附近的航向求圆周平均。 */
class H30HeadingAverage {
    // 460800 baud、8N1、每个完整惯性帧至少49字节，3秒约2822帧，再留接收缓冲余量。
    static constexpr uint32_t sampleCapacity = 3072;
    static constexpr float maxYawDeviation = 5.0f;
    static constexpr float maxGyro = 5.0f; // deg/s
    static constexpr float minAccel = 7.8f, maxAccel = 11.8f; // m/s²，含重力
    static constexpr double radiansPerDegree = 0.017453292519943295;

    float yaws_[sampleCapacity];
    uint32_t started_ = 0, sampleCount_ = 0;
    uint16_t sequence_ = 0;
    bool seen_ = false;

    static bool nearStill(const H30Attitude &sample) {
        if ((sample.fields & 7) != 7) return false;
        float gyroSquared = 0, accelSquared = 0;
        for (int axis = 0; axis < 3; ++axis) {
            gyroSquared += sample.gyro[axis] * sample.gyro[axis];
            accelSquared += sample.accel[axis] * sample.accel[axis];
        }
        return gyroSquared <= maxGyro * maxGyro
            && accelSquared >= minAccel * minAccel && accelSquared <= maxAccel * maxAccel;
    }

public:
    struct Result {
        uint32_t count;
        float yaw;
    };

    void begin(uint32_t now, uint32_t settleMs = 0) {
        started_ = now + settleMs;
        seen_ = false;
        sampleCount_ = 0;
    }

    void add(const H30Attitude &sample) {
        if (!collecting(sample.receivedAtMs) || elapsed(sample.receivedAtMs)) return;
        if (seen_ && sample.sequence == sequence_) return;
        seen_ = true;
        sequence_ = sample.sequence;
        if (!nearStill(sample) || !isfinite(sample.yaw)) return;

        float yaw = fmodf(sample.yaw, 360.0f);
        if (yaw < 0) yaw += 360.0f;
        if (sampleCount_ < sampleCapacity) yaws_[sampleCount_++] = yaw;
    }

    bool collecting(uint32_t now) const { return static_cast<int32_t>(now - started_) >= 0; }
    bool elapsed(uint32_t now) const { return static_cast<int32_t>(now - started_) >= 3000; }

    Result result() {
        if (!sampleCount_) return {0, 0};

        std::sort(yaws_, yaws_ + sampleCount_);
        const float median = circularMedian();
        uint32_t count = 0;
        double sine = 0, cosine = 0;
        for (uint32_t i = 0; i < sampleCount_; ++i) {
            const float yaw = yaws_[i];
            const float delta = fabsf(yaw - median);
            if (fminf(delta, 360.0f - delta) > maxYawDeviation) continue;
            const double radians = yaw * radiansPerDegree;
            sine += sin(radians);
            cosine += cos(radians);
            ++count;
        }
        return {count, static_cast<float>(atan2(sine, cosine) / radiansPerDegree)};
    }

private:
    // 圆周绝对角差之和的最小值可以在样本角度中取得。
    float circularMedian() const {
        double total = 0;
        for (uint32_t i = 0; i < sampleCount_; ++i) total += yaws_[i];

        double prefix = 0, leftSum = 0, rightSum = total;
        uint32_t left = 0, right = 0;
        double bestCost = INFINITY;
        float median = yaws_[0];
        for (uint32_t i = 0; i < sampleCount_; ++i) {
            const double yaw = yaws_[i];
            while (left < sampleCount_ && yaws_[left] < yaw - 180.0) leftSum += yaws_[left++];
            while (right < sampleCount_ && yaws_[right] <= yaw + 180.0) rightSum -= yaws_[right++];

            // 先求直线绝对角差，再将超出180°的距离 d 折回 360°-d。
            const double linearCost = (2.0 * i - sampleCount_) * yaw + total - 2.0 * prefix;
            const double leftCorrection = 2.0 * ((yaw - 180.0) * left - leftSum);
            const double rightCorrection = 2.0 * (rightSum - (yaw + 180.0) * (sampleCount_ - right));
            const double cost = linearCost - leftCorrection - rightCorrection;
            if (cost < bestCost) {
                bestCost = cost;
                median = yaws_[i];
            }
            prefix += yaw;
        }
        return median;
    }
};
