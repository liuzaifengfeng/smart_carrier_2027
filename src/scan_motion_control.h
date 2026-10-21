#pragma once
#include "node_position_control.h"

struct ScanMotionSettings {
    float approachMm,searchMm,forwardPpm;
    uint16_t approachRpm,searchRpm;
};
// 一次连续行程：接近->降速扫码。编码器用于行程推进，不增加故障门槛。
const char *RunScanMotion(const ScanMotionSettings &settings,node_position::IO &io, bool (*received)(),float &forwardTravel);
