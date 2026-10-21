#pragma once
#include <stdint.h>
#include <math.h>

// EMM V5位置模式，1~255加速度档位：每增加1 RPM需(256-acc)*50us。
// 从静止加速、减速到静止；短行程达不到设定轮速时按三角形速度曲线计算。
inline double PositionMoveDurationMs(uint32_t pulses,uint16_t rpm,uint8_t acceleration) {
    const double rpmPerSecond=20000.0/(256-acceleration);
    const double distanceRpmSeconds=static_cast<double>(pulses)*60.0/3200.0;
    const double rampDistance=static_cast<double>(rpm)*rpm/rpmPerSecond;
    const double seconds=distanceRpmSeconds>=rampDistance
        ? distanceRpmSeconds/rpm+rpm/rpmPerSecond
        : 2.0*sqrt(distanceRpmSeconds/rpmPerSecond);
    return seconds*1000.0;
}
