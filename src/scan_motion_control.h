#pragma once
#include "node_position_control.h"

struct ScanMotionSettings {
    float approachMm,searchMm,forwardPpm;
    uint16_t approachRpm,searchRpm;
};
// 一次连续行程：接近->降速扫码。编码器用于行程推进，不增加故障门槛。
const char *RunScanMotion(const ScanMotionSettings &settings,node_position::IO &io, bool (*received)(),float &forwardTravel);
class ScanMotionRunner {
    ScanMotionSettings config{};
    node_position::IO *io=nullptr;
    bool (*received)()=nullptr;
    int64_t origin[4]={},positions[4]={};
    bool running=false,initialized=false;
    const char *error=nullptr;
    float speed=0,travel=0;
    uint32_t previous=0;
    void finish();
public:
    void begin(const ScanMotionSettings &settings,node_position::IO &hardware,bool (*poll)());
    void tick();
    bool active() const{return running;}
    const char *failure() const{return error;}
    float forwardTravel() const{return travel;}
};
