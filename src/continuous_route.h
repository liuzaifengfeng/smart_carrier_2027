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
/**
 * @brief 计算两个二维向量的点积。
 */
inline float dot(Vec a, Vec b) { return a.x*b.x+a.y*b.y; }
/**
 * @brief 计算两个二维向量的有符号叉积，用于判断转弯方向。
 */
inline float cross(Vec a, Vec b) { return a.x*b.y-a.y*b.x; }
/**
 * @brief 计算二维向量长度。
 */
inline float norm(Vec a) { return hypotf(a.x,a.y); }
/**
 * @brief 将数值限制在闭区间a到b之间。
 */
inline float clamp(float x,float a,float b) { return fmaxf(a,fminf(b,x)); }
/**
 * @brief 将角度归一化为0~360度。
 */
inline float wrap(float a) { a=fmodf(a,360); return a<0?a+360:a; }
/**
 * @brief 计算航向a到b的最短有符号转角(deg)。
 */
inline float turn(float a,float b) { float d=wrap(b-a); return d>180?d-360:d; }
// theta=0 时车头(+Y)指向地图+X，车体+X指向地图-Y。
/**
 * @brief 按地图航向theta将世界坐标向量转换到车体坐标，theta单位deg。
 */
inline Vec worldToBody(Vec v,float theta) {
    float a=theta*PI_F/180; return {v.x*sinf(a)-v.y*cosf(a),v.x*cosf(a)+v.y*sinf(a)};
}
/**
 * @brief 按地图航向theta将车体坐标向量转换到世界坐标，theta单位deg。
 */
inline Vec bodyToWorld(Vec v,float theta) {
    float a=theta*PI_F/180; return {v.x*sinf(a)+v.y*cosf(a),-v.x*cosf(a)+v.y*sinf(a)};
}
struct Node { Vec p; float radius = 0; bool middle=false; }; // 正式路径按节点编号标记中间道路。
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
    bool middle=false; // 平移段使用独立的中间道路速度和加速度。
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
    /**
     * @brief 根据节点及配置构建路径段和速度规划；成功返回nullptr，否则为几何或配置错误原因。
     */
    const char *build(const Node *nodes,const Config &cfg);
    /**
     * @brief 查询累计路径距离s(mm)处的位置、切向及曲率。
     */
    Pose at(float s) const;
    /**
     * @brief 查询累计路径距离s(mm)处的计划地图航向(deg)。
     */
    float heading(float s) const;
    /**
     * @brief 查询路径航向相对行程的变化率(deg/mm)。
     */
    float headingSlope(float s) const; // deg/mm
    /**
     * @brief 查询路径距离s(mm)处结合几何、加速度及停止约束后的速度上限(mm/s)。
     */
    float speedCap(float s) const;
    /**
     * @brief 检查路径距离s处的车体四角是否位于旧模型的场地边界内。
     */
    bool inside(float s) const;
    /**
     * @brief 查询指定路径段内局部行程local处的几何位置、切向和曲率。
     */
    Pose phaseAt(size_t index,float local) const;
    /**
     * @brief 查询指定路径段内局部行程local处的计划航向(deg)。
     */
    float phaseHeading(size_t index,float local) const;
    /**
     * @brief 查询指定路径段内航向随行程变化率(deg/mm)。
     */
    float phaseSlope(size_t index,float local) const;
    /**
     * @brief 根据累计距离s(mm)寻找其所在的移动段下标。
     */
    size_t movingPhase(float s) const;
    /**
     * @brief 查询从指定路径段开始向后的下一个停车距离(mm)。
     */
    float nextStop(size_t index) const;
private:
    float speeds[MAX_SPEED_SAMPLES] = {};
    /**
     * @brief 结合四轮最大转速、曲率及航向变化计算当前位置的几何速度上限(mm/s)。
     */
    float geometricCap(float s) const;
    /**
     * @brief 向路径追加起点a到终点b的直线段，更新累计行程。
     */
    void line(Vec a,Vec b);
    /**
     * @brief 构建直线、外圈连续曲线及原地转向混合路径，成功返回nullptr。
     */
    const char *buildHybrid(const Node *nodes,const Config &cfg);
    /**
     * @brief 检查指定路径段的车体几何及障碍间距，成功返回nullptr。
     */
    const char *validatePhase(size_t index) const;
    /**
     * @brief 追加指定位置p处from到to的原地转向段，角度单位deg；返回是否成功加入。
     */
    bool addRotation(Vec p,float from,float to);
};

struct WheelCommand {
    int16_t rpm[4] = {};
    Vec body; float omega=0;
};
/**
 * @brief 将车体平移速度(mm/s)及角速度(deg/s)换算为四轮RPM，并按配置上限等比例缩放。
 */
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
    /**
     * @brief 保存本周期惯性样本，供Tracker停稳和航向控制使用。
     */
    void observeMotion(const H30Attitude &sample){motionSample=sample;}
    /**
     * @brief 初始化路径跟踪状态；actualHeading为开始时实测地图航向(deg)。
     */
    void begin(const Route &route,float actualHeading);
    /**
     * @brief 计算一个控制周期的四轮命令、航向误差及完成结果；dt为秒，sampleMs为样本毫秒数。
     */
    Output update(const Route &route,float actualHeading,float dt,uint32_t sampleMs);
    /**
     * @brief 按已下发轮速、实测航向及时间dt(s)更新估算位置，不代表外部定位。
     */
    void integrate(const Route &route,float actualHeading,float dt);
    /**
     * @brief 记录实际应用的四轮命令和时间，供积分及零轮速停稳判断使用。
     */
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
    /**
     * @brief 推进混合路径当前段的速度、转向及停稳判定，返回本周期控制输出。
     */
    Output updateHybrid(const Route &route,float actual,float dt,uint32_t sampleMs);
};
} // namespace continuous
