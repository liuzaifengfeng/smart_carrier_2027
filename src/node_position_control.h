#pragma once
#include "continuous_route.h"

namespace node_position {
constexpr float PULSES_PER_REV=3200; // 要求底盘驱动配置为16细分，实车验收时核对
struct Settings {float speedMmS=1200,accelMmS2=1600,turnDegS=150,turnAccelDegS2=360,headingDeadbandDeg=3;};
struct Motor {uint8_t flags=0;int64_t position=0;int16_t rpm=0;}; // 位置：65536单位/圈
struct Result {continuous::Vec estimated;float heading=0,s=0;};
// 瞬时制动扰动不等于持续偏航；与0.4度的主动纠偏启动阈值分开。
class HeadingGuard {
    bool softActive=false,hardActive=false,brakeActive=false;
    uint32_t softSince=0,hardSince=0,brakeSince=0;
public:
    bool update(float error,uint32_t stamp,bool braking,bool stopped,bool available=true) {
        if(!available){*this=HeadingGuard{};return false;}
        if(braking&&!brakeActive)brakeSince=stamp;
        brakeActive=braking;
        float magnitude=fabsf(error);
        if(magnitude>30) {
            if(!hardActive){hardActive=true;hardSince=stamp;}
            if(stamp-hardSince>=200)return true;
        } else hardActive=false;
        bool grace=braking&&stamp-brakeSince<1200;
        if(magnitude>15&&!stopped&&!grace) {
            if(!softActive){softActive=true;softSince=stamp;}
            if(stamp-softSince>=800)return true;
        } else softActive=false;
        return false;
    }
};
struct IO {
    virtual ~IO()=default;
    virtual uint32_t now()=0;
    virtual void wait(uint32_t ms)=0;
    virtual bool cancelled()=0;
    virtual const char *imu(H30Attitude &sample,float &heading)=0;
    virtual const char *read(uint8_t id,Motor &state)=0;
    virtual const char *zero(uint8_t id)=0;
    // 同一绝对目标只更新速度，绝不反复追加相对行程；启动前已核对零点。
    virtual const char *target(uint8_t id,bool positive,uint16_t rpm,uint32_t pulses)=0;
    virtual const char *sync()=0;
    virtual void halt()=0;
    virtual void report(const char *stage,const Result &result,float reference)=0;
    virtual void beginAction(){}
    virtual void motorDiagnostic(uint8_t,const char *,double,const Motor &){}
};
inline uint16_t rpmFor(float speed,float ppm) {
    float rpm=floorf(fabsf(speed)*ppm*60/PULSES_PER_REV);
    return static_cast<uint16_t>(continuous::clamp(rpm,1,5000));
}
const char *run(const continuous::Route &route,const Settings &settings,IO &io,Result &result);
}
