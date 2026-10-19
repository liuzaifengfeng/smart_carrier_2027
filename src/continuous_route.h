#pragma once
#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include <initializer_list>
#include "h30_motion.h"

namespace continuous {
constexpr float PI_F = 3.14159265358979323846f;
constexpr size_t MAX_NODES = 25, MAX_SEGMENTS = 49, MAX_SPEED_SAMPLES = 4096;
struct Vec { float x = 0, y = 0; Vec() = default; Vec(float a, float b): x(a), y(b) {} };
inline Vec operator+(Vec a, Vec b) { return {a.x+b.x, a.y+b.y}; }
inline Vec operator-(Vec a, Vec b) { return {a.x-b.x, a.y-b.y}; }
inline Vec operator*(Vec a, float s) { return {a.x*s, a.y*s}; }
inline float dot(Vec a, Vec b) { return a.x*b.x+a.y*b.y; }
inline float cross(Vec a, Vec b) { return a.x*b.y-a.y*b.x; }
inline float norm(Vec a) { return hypotf(a.x,a.y); }
inline float clamp(float x,float a,float b) { return fmaxf(a,fminf(b,x)); }
inline float wrap(float a) { a=fmodf(a,360); return a<0?a+360:a; }
inline float turn(float a,float b) { float d=wrap(b-a); return d>180?d-360:d; }
// theta=0 时车头(+Y)指向地图+X，车体+X指向地图-Y。
inline Vec worldToBody(Vec v,float theta) {
    float a=theta*PI_F/180; return {v.x*sinf(a)-v.y*cosf(a),v.x*cosf(a)+v.y*sinf(a)};
}
inline Vec bodyToWorld(Vec v,float theta) {
    float a=theta*PI_F/180; return {v.x*sinf(a)+v.y*cosf(a),-v.x*cosf(a)+v.y*sinf(a)};
}
struct Node { Vec p; float radius = 0; };
struct Config {
    uint32_t id=0;
    uint8_t count=0;
    float startHeading=0,endHeading=0,maxRpm=80,accel=200,lateralAccel=300;
    float lateralPpm=13.3f,forwardPpm=13.6f,rotationPpd=51.8f;
    float kp=2,maxOmega=45,correction=20,omegaAccel=90;
    uint8_t policy=0; // 0旧模型；试跑1自动/2停转；3正式外圈连续、内侧位置模式。
    float outset=60,margin=20;
};
struct Pose { Vec p,tangent; float curvature=0; };
enum SegmentKind { LINE=0, CUBIC=1, ROTATE=2 };
struct Segment {
    Vec start,end,center;
    float offset=0,length=0,radius=0,angle=0,sweep=0;
    uint8_t kind=LINE;
    Vec control1,control2;
    float h0=0,h1=0;
    float lut[65]={};
};

class Route {
public:
    Config config;
    Segment segments[MAX_SEGMENTS];
    size_t segmentCount=0,speedCount=0;
    float length=0,headingDelta=0,spacing=5,plannedSeconds=0;
    const char *build(const Node *nodes,const Config &cfg);
    Pose at(float s) const;
    float heading(float s) const;
    float headingSlope(float s) const; // deg/mm
    float speedCap(float s) const;
    bool inside(float s) const;
    Pose phaseAt(size_t index,float local) const;
    float phaseHeading(size_t index,float local) const;
    float phaseSlope(size_t index,float local) const;
    size_t movingPhase(float s) const;
    float nextStop(size_t index) const;
private:
    float speeds[MAX_SPEED_SAMPLES] = {};
    float geometricCap(float s) const;
    void line(Vec a,Vec b);
    const char *buildHybrid(const Node *nodes,const Config &cfg);
    const char *validatePhase(size_t index) const;
    bool addRotation(Vec p,float from,float to);
};

struct WheelCommand {
    int16_t rpm[4] = {};
    Vec body; float omega=0;
};
WheelCommand wheels(Vec body,float omega,const Config &cfg);
struct Output {
    WheelCommand wheels;
    float reference=0,error=0;
    bool finished=false;
    const char *failure=nullptr;
};
// 纯计算状态机，硬件任务只负责提供样本、下发命令和记录同步启动时间。
class Tracker {
public:
    float s=0,speed=0,omega=0;
    Vec estimated;
    bool finishing=false;
    size_t phaseIndex=0;
    bool requireInertial=false;
    bool movingBlock=false; // 正式外圈只复用移动段，不接管内侧原地转向或引入试跑故障门槛。
    const char *stage="MOVE";
    H30Attitude motionSample;
    void observeMotion(const H30Attitude &sample){motionSample=sample;}
    void begin(const Route &route,float actualHeading);
    Output update(const Route &route,float actualHeading,float dt,uint32_t sampleMs);
    void integrate(const Route &route,float actualHeading,float dt);
    void commit(const WheelCommand &command,uint32_t atMs=0) {
        bool zero=true;for(int i=0;i<4;++i)if(command.rpm[i])zero=false;
        if(zero&&!zeroCommanded)zeroSince=atMs;
        zeroCommanded=zero;applied=command;
    }
private:
    WheelCommand applied;
    float badSeconds=0,finishSeconds=0,stableSeconds=0;
    uint32_t previousSample=0;
    float previousActual=0,turnSeconds=0;
    bool rateReady=false;
    bool phaseStopped=false;
    H30MotionQuiet quiet;
    H30HeadingHold hold;
    bool zeroCommanded=false;uint32_t zeroSince=0;
    Output updateHybrid(const Route &route,float actual,float dt,uint32_t sampleMs);
};
} // namespace continuous
