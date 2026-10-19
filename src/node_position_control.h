#pragma once
#include "continuous_route.h"

namespace node_position {
constexpr bool MOTOR_STALL_CHECK_ENABLED=false; // 临时禁用所有底盘电机的固件堵转检查；恢复时改为true
constexpr float PULSES_PER_REV=3200; // 要求底盘驱动配置为16细分，实车验收时核对
constexpr double STRAIGHT_ARRIVAL_ENCODER_TOLERANCE=5000; // 每轮65536单位/圈坐标中的允许误差
struct Settings {float speedMmS=2000,accelMmS2=2000,turnDegS=200,turnAccelDegS2=400,headingDeadbandDeg=3;};
struct Motor {uint8_t flags=0;int64_t position=0;int16_t rpm=0;}; // 位置：65536单位/圈
struct Result {continuous::Vec estimated;float heading=0,s=0;};
// 直行期间仅监测持续偏航；航向校正在到位、近零转速后独立执行。
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
    virtual const char *readPosition(uint8_t id,int64_t &position) {
        Motor state;const char *error=read(id,state);if(!error)position=state.position;return error;
    }
    virtual const char *zero(uint8_t id)=0;
    // 每个动作只装载一次绝对目标、最大轮速和加速度，由驱动器完成加减速。
    virtual const char *target(uint8_t id,bool positive,uint16_t rpm,uint8_t acceleration,uint32_t pulses)=0;
    virtual const char *sync()=0;
    virtual const char *velocity(const continuous::WheelCommand &command)=0;
    virtual void halt()=0;
    virtual void report(const char *stage,const Result &result,float reference)=0;
    virtual void beginAction(){}
    virtual void timing(const char *){}
    virtual void arrivalDiagnostic(float,const Motor (&)[4]){}
    virtual void motorDiagnostic(uint8_t,const char *,double,const Motor &){}
};
inline uint16_t rpmFor(float speed,float ppm) {
    float rpm=floorf(fabsf(speed)*ppm*60/PULSES_PER_REV);
    return static_cast<uint16_t>(continuous::clamp(rpm,1,5000));
}
// EMM V5: 每增加1 RPM用时(256-acc)*50us；0会关闭曲线，不能表示物理加速度。
// 将路径的距离/角度加速度结合当前脉冲系数换算为最接近的1~255档位。
inline uint8_t driverAccelerationFor(float acceleration,float pulsesPerUnit) {
    float rpmPerSecond=fabsf(acceleration)*pulsesPerUnit*60/PULSES_PER_REV;
    if(!isfinite(rpmPerSecond)||rpmPerSecond<=0)return 1;
    float level=roundf(256-20000/rpmPerSecond);
    return static_cast<uint8_t>(continuous::clamp(level,1,255));
}
const char *run(const continuous::Route &route,const Settings &settings,IO &io,Result &result);
}
