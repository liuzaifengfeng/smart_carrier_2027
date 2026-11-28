// 将 H30 姿态航向绑定到地图航向，并提供跨角度边界的处理。
// 读取角度时应区分传感器参考系与已校准的地图参考系。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include <algorithm>  // 引入 algorithm 中的声明和定义。
#include <math.h>  // 引入 math.h 中的声明和定义。
#include <stdint.h>  // 引入 stdint.h 中的声明和定义。
#include "h30_motion.h"  // 引入 h30_motion.h 中的声明和定义。
#if __cplusplus >= 201402L  // 在编译阶段判断是否纳入此分支。
#define H30_HEADING_CONSTEXPR constexpr  // 为后续文本展开定义宏 H30_HEADING_CONSTEXPR constexpr。
#else  // 前面的编译条件均未选中时使用此分支。
#define H30_HEADING_CONSTEXPR  // 为后续文本展开定义宏 H30_HEADING_CONSTEXPR。
#endif  // 结束本组条件编译。

// 平面航向参考。绕 Y 倒装不会自动要求将欧拉 Yaw 取负。
// 以标准 Yaw 正方向为正；实车需验证左转时地图航向递增。
// 保存原始传感器航向到地图航向的对应关系。
// 校准更新参考，读取路径使用参考把实时角度映射到地图。
class H30HeadingReference {  // 定义 H30HeadingReference（H30航向Reference） 的类型成员或作用域。
public:  // 此后的成员采用 public 访问权限，直到下一个权限标签。
    /**
     * @brief 清除地图航向标定状态，不保留旧参考继续换算。
     */
    H30_HEADING_CONSTEXPR void reset() { calibrated_ = false; }  // 将 false; } 写入或初始化 calibrated_。
    /**
     * @brief 将传感器Yaw与已知地图航向绑定为同一时刻的参考；角度单位deg。
     * @return true 参考建立成功，false 输入无效。
     */
    H30_HEADING_CONSTEXPR bool calibrate(float sensorYaw, float mapHeading) {  // 定义函数或成员 的入口 calibrate，参数由此处的签名确定。
        reset();  // 调用 reset，使用无参形式推进相应操作。
        if (!validYaw(sensorYaw) || !isfinite(mapHeading)  // 当 !validYaw(sensorYaw) || !isfinite(mapHeading 成立时进入此分支，否则继续后续流程。
                || mapHeading < 0 || mapHeading > 360) return false;  // 继续上一行的组合条件，本行提供 || mapHeading < 0 || mapHeading > 360) return false;。
        sensorAtReference_ = normalize(sensorYaw);  // 将 normalize(sensorYaw) 写入或初始化 sensorAtReference_。
        mapAtReference_ = normalize(mapHeading);  // 将 normalize(mapHeading) 写入或初始化 mapAtReference_。
        calibrated_ = true;  // 将 true 写入或初始化 calibrated_。
        return true;  // 结束当前函数并返回 true。
    }  // 结束此函数或控制块。
    /**
     * @brief 使用当前参考将原始Yaw换算为0~360度地图航向，结果写入heading。
     * @return true 换算成功，false 尚未标定或输入无效。
     */
    H30_HEADING_CONSTEXPR bool get(float sensorYaw, float &heading) const {  // 定义函数或成员 的入口 get，参数由此处的签名确定。
        if (!calibrated_ || !validYaw(sensorYaw)) return false;  // 若 !calibrated_ || !validYaw(sensorYaw)，立即返回 false。
        heading = normalize(mapAtReference_ + normalize(sensorYaw) - sensorAtReference_);  // 将 normalize(mapAtReference_ + normalize(sensorYaw) - sen… 写入或初始化 heading（航向）。
        return true;  // 结束当前函数并返回 true。
    }  // 结束此函数或控制块。
private:  // 此后的成员采用 private 访问权限，直到下一个权限标签。
    bool calibrated_ = false;  // 将 false 写入或初始化 calibrated_。
    float sensorAtReference_ = 0, mapAtReference_ = 0;  // 将 0, mapAtReference_ = 0 写入或初始化 sensorAtReference_。
    /**
     * @brief 检查原始Yaw是否有限且处于YIS角度字段可表示范围。
     */
    static H30_HEADING_CONSTEXPR bool validYaw(float yaw) {  // 定义函数或成员 的入口 validYaw（有效偏航），参数由此处的签名确定。
        // YIS int32 * 1e-6 的整个可表示范围；也使下方整数商转换有界。
        return isfinite(yaw) && yaw >= -2148 && yaw <= 2148;  // 结束当前函数并返回 isfinite(yaw) && yaw >= -2148 && yaw <= 2148。
    }  // 结束此函数或控制块。
    /**
     * @brief 将角度归一化到0~360度范围。
     */
    static H30_HEADING_CONSTEXPR float normalize(float angle) {  // 定义函数或成员 的入口 normalize，参数由此处的签名确定。
        const float wrapped = angle - int32_t(angle / 360.0f) * 360.0f;  // 将 angle - int32_t(angle / 360.0f) * 360.0f 写入或初始化 wrapped。
        const float positive = wrapped < 0 ? wrapped + 360.0f : wrapped;  // 将 wrapped < 0 ? wrapped + 360.0f : wrapped 写入或初始化 positive。
        return positive >= 360.0f ? 0.0f : positive;  // 结束当前函数并返回 positive >= 360.0f ? 0.0f : positive。
    }  // 结束此函数或控制块。
};  // 结束此类型定义。
#undef H30_HEADING_CONSTEXPR  // 撤销宏 H30_HEADING_CONSTEXPR，限定其作用范围。

/** @brief 在固定3秒窗口内筛选近静止样本，保留圆周中位数附近的航向求圆周平均。 */
// 采样阶段持续累积航向供最后绑定地图角度。
// 排查标定效果时可分别看采样窗口、角度平均和参考更新时机。
class H30HeadingAverage {  // 定义 H30HeadingAverage（H30航向Average） 的类型成员或作用域。
    // 460800 baud、8N1、每个完整惯性帧至少49字节，3秒约2822帧，再留接收缓冲余量。
    static constexpr uint32_t sampleCapacity = 3072;  // 将 3072 写入或初始化 sampleCapacity（采样Capacity）。
    static constexpr float maxYawDeviation = 5.0f;  // 将 5.0f 写入或初始化 maxYawDeviation（最大偏航Deviation）。
    static constexpr float maxGyro = 5.0f; // deg/s
    static constexpr float minAccel = 7.8f, maxAccel = 11.8f; // m/s²，含重力
    static constexpr double radiansPerDegree = 0.017453292519943295;  // 将 0.017453292519943295 写入或初始化 radiansPerDegree。

    float yaws_[sampleCapacity];  // 声明 yaws_，类型和可用范围由所在声明决定。
    uint32_t started_ = 0, sampleCount_ = 0;  // 将 0, sampleCount_ = 0 写入或初始化 started_。
    uint16_t sequence_ = 0;  // 将 0 写入或初始化 sequence_。
    bool seen_ = false;  // 将 false 写入或初始化 seen_。

    static bool nearStill(const H30Attitude &sample) {  // 定义函数或成员 的入口 nearStill，参数由此处的签名确定。
        if ((sample.fields & 7) != 7) return false;  // 若 (sample.fields & 7) != 7，立即返回 false。
        float gyroSquared = 0, accelSquared = 0;  // 将 0, accelSquared = 0 写入或初始化 gyroSquared。
        for (int axis = 0; axis < 3; ++axis) {  // 按 int axis = 0; axis < 3; ++axis 组织循环的初值、继续条件和步进。
            gyroSquared += sample.gyro[axis] * sample.gyro[axis];  // 把 sample.gyro[axis] * sample.gyro[axis] 累加到 gyroSquared。
            accelSquared += sample.accel[axis] * sample.accel[axis];  // 把 sample.accel[axis] * sample.accel[axis] 累加到 accelSquared。
        }  // 结束此循环体。
        return gyroSquared <= maxGyro * maxGyro  // 结束当前函数并返回 gyroSquared <= maxGyro * maxGyro。
            && accelSquared >= minAccel * minAccel && accelSquared <= maxAccel * maxAccel;  // 继续上一行的组合条件，本行提供 && accelSquared >= minAccel * minAccel && accelSquared <= maxAccel * maxAccel;。
    }  // 结束此函数或控制块。

public:  // 此后的成员采用 public 访问权限，直到下一个权限标签。
    struct Result {  // 定义 Result（结果） 的类型成员或作用域。
        uint32_t count;  // 声明 count（计数），类型和可用范围由所在声明决定。
        float yaw;  // 声明 yaw（偏航），类型和可用范围由所在声明决定。
    };  // 结束此类型定义。

    void begin(uint32_t now, uint32_t settleMs = 0) {  // 定义函数或成员 的入口 begin，参数由此处的签名确定。
        started_ = now + settleMs;  // 将 now + settleMs 写入或初始化 started_。
        seen_ = false;  // 将 false 写入或初始化 seen_。
        sampleCount_ = 0;  // 将 0 写入或初始化 sampleCount_（采样计数）。
    }  // 结束此初始化列表或局部作用域。

    void add(const H30Attitude &sample) {  // 定义函数或成员 的入口 add，参数由此处的签名确定。
        if (!collecting(sample.receivedAtMs) || elapsed(sample.receivedAtMs)) return;  // 当 !collecting(sample.receivedAtMs) || elapsed(sample.rec… 成立时进入此分支，否则继续后续流程。
        if (seen_ && sample.sequence == sequence_) return;  // 当 seen_ && sample.sequence == sequence_ 成立时进入此分支，否则继续后续流程。
        seen_ = true;  // 将 true 写入或初始化 seen_。
        sequence_ = sample.sequence;  // 将 sample.sequence 写入或初始化 sequence_。
        if (!nearStill(sample) || !isfinite(sample.yaw)) return;  // 当 !nearStill(sample) || !isfinite(sample.yaw) 成立时进入此分支，否则继续后续流程。

        float yaw = fmodf(sample.yaw, 360.0f);  // 将 fmodf(sample.yaw, 360.0f) 写入或初始化 yaw（偏航）。
        if (yaw < 0) yaw += 360.0f;  // 当 yaw < 0 成立时进入此分支，否则继续后续流程。
        if (sampleCount_ < sampleCapacity) yaws_[sampleCount_++] = yaw;  // 当 sampleCount_ < sampleCapacity 成立时进入此分支，否则继续后续流程。
    }  // 结束此函数或控制块。

    bool collecting(uint32_t now) const { return static_cast<int32_t>(now - started_) >= 0; }  // 定义函数或成员 collecting，参数由此处的签名确定。
    bool elapsed(uint32_t now) const { return static_cast<int32_t>(now - started_) >= 3000; }  // 定义函数或成员 elapsed（已经过），参数由此处的签名确定。

    Result result() {  // 定义函数或成员 的入口 result（结果），参数由此处的签名确定。
        if (!sampleCount_) return {0, 0};  // 当 !sampleCount_ 成立时进入此分支，否则继续后续流程。

        std::sort(yaws_, yaws_ + sampleCount_);  // 调用 std::sort，传入 yaws_, yaws_ + sampleCount_。
        const float median = circularMedian();  // 将 circularMedian() 写入或初始化 median。
        uint32_t count = 0;  // 将 0 写入或初始化 count（计数）。
        double sine = 0, cosine = 0;  // 将 0, cosine = 0 写入或初始化 sine。
        for (uint32_t i = 0; i < sampleCount_; ++i) {  // 按 uint32_t i = 0; i < sampleCount_; ++i 组织循环的初值、继续条件和步进。
            const float yaw = yaws_[i];  // 将 yaws_[i] 写入或初始化 yaw（偏航）。
            const float delta = fabsf(yaw - median);  // 将 fabsf(yaw - median) 写入或初始化 delta。
            if (fminf(delta, 360.0f - delta) > maxYawDeviation) continue;  // 若 fminf(delta, 360.0f - delta) > maxYawDeviation，直接进入下一轮循环。
            const double radians = yaw * radiansPerDegree;  // 将 yaw * radiansPerDegree 写入或初始化 radians。
            sine += sin(radians);  // 把 sin(radians) 累加到 sine。
            cosine += cos(radians);  // 把 cos(radians) 累加到 cosine。
            ++count;  // 续写当前声明或运算，提供 ++count;。
        }  // 结束此循环体。
        return {count, static_cast<float>(atan2(sine, cosine) / radiansPerDegree)};  // 结束当前函数并返回 {count, static_cast<float>(atan2(sine, cosine) / radia…。
    }  // 结束此函数或控制块。

private:  // 此后的成员采用 private 访问权限，直到下一个权限标签。
    // 圆周绝对角差之和的最小值可以在样本角度中取得。
    float circularMedian() const {  // 定义函数或成员 的入口 circularMedian，参数由此处的签名确定。
        double total = 0;  // 将 0 写入或初始化 total（总计）。
        for (uint32_t i = 0; i < sampleCount_; ++i) total += yaws_[i];  // 按 uint32_t i = 0; i < sampleCount_; ++i) total += yaws_[… 组织循环的初值、继续条件和步进。

        double prefix = 0, leftSum = 0, rightSum = total;  // 将 0, leftSum = 0, rightSum = total 写入或初始化 prefix。
        uint32_t left = 0, right = 0;  // 将 0, right = 0 写入或初始化 left（左）。
        double bestCost = INFINITY;  // 将 INFINITY 写入或初始化 bestCost。
        float median = yaws_[0];  // 将 yaws_[0] 写入或初始化 median。
        for (uint32_t i = 0; i < sampleCount_; ++i) {  // 按 uint32_t i = 0; i < sampleCount_; ++i 组织循环的初值、继续条件和步进。
            const double yaw = yaws_[i];  // 将 yaws_[i] 写入或初始化 yaw（偏航）。
            while (left < sampleCount_ && yaws_[left] < yaw - 180.0) leftSum += yaws_[left++];  // 当 left < sampleCount_ && yaws_[left] < yaw - 180.0 成立时继续循环。
            while (right < sampleCount_ && yaws_[right] <= yaw + 180.0) rightSum -= yaws_[right++];  // 当 right < sampleCount_ && yaws_[right] <= yaw + 180.0 成立时继续循环。

            // 先求直线绝对角差，再将超出180°的距离 d 折回 360°-d。
            const double linearCost = (2.0 * i - sampleCount_) * yaw + total - 2.0 * prefix;  // 将 (2.0 * i - sampleCount_) * yaw + total - 2.0 * prefix 写入或初始化 linearCost。
            const double leftCorrection = 2.0 * ((yaw - 180.0) * left - leftSum);  // 将 2.0 * ((yaw - 180.0) * left - leftSum) 写入或初始化 leftCorrection（左修正量）。
            const double rightCorrection = 2.0 * (rightSum - (yaw + 180.0) * (sampleCount_ - right));  // 将 2.0 * (rightSum - (yaw + 180.0) * (sampleCount_ - righ… 写入或初始化 rightCorrection（右修正量）。
            const double cost = linearCost - leftCorrection - rightCorrection;  // 将 linearCost - leftCorrection - rightCorrection 写入或初始化 cost。
            if (cost < bestCost) {  // 当 cost < bestCost 成立时进入此分支，否则继续后续流程。
                bestCost = cost;  // 将 cost 写入或初始化 bestCost。
                median = yaws_[i];  // 将 yaws_[i] 写入或初始化 median。
            }  // 结束此条件分支。
            prefix += yaw;  // 把 yaw 累加到 prefix。
        }  // 结束此循环体。
        return median;  // 结束当前函数并返回 median。
    }  // 结束此函数或控制块。
};  // 结束此类型定义。
