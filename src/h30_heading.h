#pragma once
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

/** @brief 等待结束后，在固定3秒窗口内对每个新Yaw求圆周平均，不再筛选静止或离群样本。 */
class H30HeadingAverage {
    uint32_t started_=0,count_=0;
    double sine_=0,cosine_=0;
    uint16_t sequence_=0;
    bool seen_=false;
public:
    void begin(uint32_t now,uint32_t settleMs=0){*this=H30HeadingAverage{};started_=now+settleMs;}
    void add(const H30Attitude &sample) {
        const uint32_t now=sample.receivedAtMs;
        const int32_t elapsed=static_cast<int32_t>(now-started_);
        if(elapsed<0||elapsed>=3000)return;
        if(seen_&&sample.sequence==sequence_)return;
        seen_=true;sequence_=sample.sequence;
        if(!isfinite(sample.yaw))return;
        const double radians=sample.yaw*0.017453292519943295;
        sine_+=sin(radians);cosine_+=cos(radians);++count_;
    }
    bool collecting(uint32_t now)const{return static_cast<int32_t>(now-started_)>=0;}
    bool elapsed(uint32_t now)const{return static_cast<int32_t>(now-started_)>=3000;}
    uint32_t count()const{return count_;}
    float yaw()const{return atan2(sine_,cosine_)*57.29577951308232;}
};
