#include "scan_motion_control.h"

const char *RunScanMotion(const ScanMotionSettings &cfg,node_position::IO &io,
                         bool (*received)(),float &forwardTravel) {
    int64_t origin[4],positions[4];
    for(uint8_t id=1;id<=4;++id) {
        const char *error=io.readPosition(id,origin[id-1]);
        if(error)return error;
    }
    const float approach=fabsf(cfg.approachMm),total=approach+fabsf(cfg.searchMm);
    const float rpmPerSecond=20000.0f/(256-180);
    const float rpmSecondsPerMm=cfg.forwardPpm*60.0f/3200;
    const float direction=cfg.approachMm>=0?1:-1;
    float speed=0;
    uint32_t previous=io.now();
    const char *error=nullptr;
    for(;;) {
        double units=0;
        for(uint8_t id=1;id<=4;++id) {
            error=io.readPosition(id,positions[id-1]);
            if(error)break;
            units+=fabs(static_cast<double>(positions[id-1]-origin[id-1]))/4;
        }
        if(error)break;
        const float travel=units*3200/65536/cfg.forwardPpm;
        if(received()||travel>=total)break;
        // 提前平滑降至扫码轮速；在整段终点才降至零速。
        const float entryCap=travel<approach
            ? sqrtf(cfg.searchRpm*float(cfg.searchRpm)+2*rpmPerSecond*rpmSecondsPerMm*(approach-travel))
            : cfg.searchRpm;
        const float endCap=sqrtf(2*rpmPerSecond*rpmSecondsPerMm*(total-travel));
        const float cap=fminf(cfg.approachRpm,fminf(entryCap,endCap));
        const uint32_t now=io.now();
        const float step=rpmPerSecond*(now-previous)*.001f;previous=now;
        speed+=continuous::clamp(cap-speed,-step,step);
        const int16_t rpm=static_cast<int16_t>(fmaxf(1,lroundf(speed)))*direction;
        continuous::WheelCommand command;
        command.rpm[0]=command.rpm[2]=rpm;
        command.rpm[1]=command.rpm[3]=-rpm;
        error=io.velocity(command);if(error)break;
    }
    io.halt();
    double units=0;
    for(uint8_t id=1;id<=4;++id) {
        const char *readError=io.readPosition(id,positions[id-1]);
        if(readError){if(!error)error=readError;break;}
        units+=fabs(static_cast<double>(positions[id-1]-origin[id-1]))/4;
    }
    forwardTravel=direction*units*3200/65536/cfg.forwardPpm;
    return error;
}
