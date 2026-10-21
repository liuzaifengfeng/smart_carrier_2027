#include "scan_motion_control.h"

void ScanMotionRunner::begin(const ScanMotionSettings &settings,node_position::IO &hardware,bool (*poll)()) {
    *this=ScanMotionRunner{};config=settings;io=&hardware;received=poll;running=true;
}
void ScanMotionRunner::finish() {
    io->halt();double units=0;
    for(uint8_t id=1;id<=4;++id) {
        const char *failure=io->readPosition(id,positions[id-1]);
        if(failure){if(!error)error=failure;break;}
        units+=fabs(static_cast<double>(positions[id-1]-origin[id-1]))/4;
    }
    travel=(config.approachMm>=0?1:-1)*units*3200/65536/config.forwardPpm;running=false;
}
void ScanMotionRunner::tick() {
    if(!running)return;
    if(io->cancelled()){error="STOPPED";finish();return;}
    if(!initialized) {
        for(uint8_t id=1;id<=4;++id){error=io->readPosition(id,origin[id-1]);if(error){running=false;return;}}
        previous=io->now();initialized=true;return;
    }
    double units=0;
    for(uint8_t id=1;id<=4;++id) {
        error=io->readPosition(id,positions[id-1]);if(error){finish();return;}
        units+=fabs(static_cast<double>(positions[id-1]-origin[id-1]))/4;
    }
    const float approach=fabsf(config.approachMm),total=approach+fabsf(config.searchMm);
    const float distance=units*3200/65536/config.forwardPpm;
    if(received()||distance>=total){finish();return;}
    const float rpmPerSecond=20000.0f/(256-180),rpmSecondsPerMm=config.forwardPpm*60/3200;
    const float entryCap=distance<approach?sqrtf(config.searchRpm*float(config.searchRpm)+2*rpmPerSecond*rpmSecondsPerMm*(approach-distance)):config.searchRpm;
    const float endCap=sqrtf(2*rpmPerSecond*rpmSecondsPerMm*(total-distance));
    const float cap=fminf(config.approachRpm,fminf(entryCap,endCap));
    const uint32_t now=io->now();const float step=rpmPerSecond*(now-previous)*.001f;previous=now;
    speed+=continuous::clamp(cap-speed,-step,step);
    const int16_t rpm=static_cast<int16_t>(fmaxf(1,lroundf(speed)))*(config.approachMm>=0?1:-1);
    continuous::WheelCommand command;command.rpm[0]=command.rpm[2]=rpm;command.rpm[1]=command.rpm[3]=-rpm;
    error=io->velocity(command);if(error)finish();
}
const char *RunScanMotion(const ScanMotionSettings &settings,node_position::IO &io,bool (*received)(),float &travel) {
    ScanMotionRunner runner;runner.begin(settings,io,received);
    while(runner.active())runner.tick();
    travel=runner.forwardTravel();return runner.failure();
}
