// 连续路径规划与跟踪使用的数据结构。
// 节点是输入，段保存几何结果，跟踪器记录当前进度；排查问题时可按这三层定位。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include <math.h>  // 引入 math.h 中的声明和定义。
#include <stdint.h>  // 引入 stdint.h 中的声明和定义。
#include <stddef.h>  // 引入 stddef.h 中的声明和定义。
#include <initializer_list>  // 引入 initializer_list 中的声明和定义。
#include "h30_motion.h"  // 引入 h30_motion.h 中的声明和定义。

namespace continuous {  // 定义 continuous 的类型成员或作用域。
constexpr float PI_F = 3.14159265358979323846f;  // 将 3.14159265358979323846f 写入或初始化 PI_F。
constexpr size_t MAX_NODES = 25, MAX_SEGMENTS = 49, MAX_SPEED_SAMPLES = 4096;  // 将 25, MAX_SEGMENTS = 49, MAX_SPEED_SAMPLES = 4096 写入或初始化 MAX_NODES（最大节点序列）。
struct Vec { float x = 0, y = 0; Vec() = default; Vec(float a, float b): x(a), y(b) {} };  // 定义 Vec 的类型成员或作用域。
inline Vec operator+(Vec a, Vec b) { return {a.x+b.x, a.y+b.y}; }  // 续写当前声明或运算，提供 inline Vec operator+(Vec a, Vec b) { return {a.x+b.x, …。
inline Vec operator-(Vec a, Vec b) { return {a.x-b.x, a.y-b.y}; }  // 续写当前声明或运算，提供 inline Vec operator-(Vec a, Vec b) { return {a.x-b.x, …。
inline Vec operator*(Vec a, float s) { return {a.x*s, a.y*s}; }  // 续写当前声明或运算，提供 inline Vec operator*(Vec a, float s) { return {a.x*s, …。
/**
 * @brief 计算两个二维向量的点积。
 */
inline float dot(Vec a, Vec b) { return a.x*b.x+a.y*b.y; }  // 定义函数或成员 dot，参数由此处的签名确定。
/**
 * @brief 计算两个二维向量的有符号叉积，用于判断转弯方向。
 */
inline float cross(Vec a, Vec b) { return a.x*b.y-a.y*b.x; }  // 定义函数或成员 cross，参数由此处的签名确定。
/**
 * @brief 计算二维向量长度。
 */
inline float norm(Vec a) { return hypotf(a.x,a.y); }  // 定义函数或成员 norm，参数由此处的签名确定。
/**
 * @brief 将数值限制在闭区间a到b之间。
 */
inline float clamp(float x,float a,float b) { return fmaxf(a,fminf(b,x)); }  // 定义函数或成员 clamp，参数由此处的签名确定。
/**
 * @brief 将角度归一化为0~360度。
 */
inline float wrap(float a) { a=fmodf(a,360); return a<0?a+360:a; }  // 将 fmodf(a,360); return a<0?a+360:a; } 写入或初始化 a。
/**
 * @brief 计算航向a到b的最短有符号转角(deg)。
 */
inline float turn(float a,float b) { float d=wrap(b-a); return d>180?d-360:d; }  // 将 wrap(b-a); return d>180?d-360:d; } 写入或初始化 d。
// theta=0 时车头(+Y)指向地图+X，车体+X指向地图-Y。
/**
 * @brief 按地图航向theta将世界坐标向量转换到车体坐标，theta单位deg。
 */
inline Vec worldToBody(Vec v,float theta) {  // 定义函数或成员 的入口 worldToBody（地图To车体），参数由此处的签名确定。
    float a=theta*PI_F/180; return {v.x*sinf(a)-v.y*cosf(a),v.x*cosf(a)+v.y*sinf(a)};  // 将 theta*PI_F/180; return {v.x*sinf(a)-v.y*cosf(a),v.x*co… 写入或初始化 a。
}  // 结束此函数或控制块。
/**
 * @brief 按地图航向theta将车体坐标向量转换到世界坐标，theta单位deg。
 */
inline Vec bodyToWorld(Vec v,float theta) {  // 定义函数或成员 的入口 bodyToWorld（车体To地图），参数由此处的签名确定。
    float a=theta*PI_F/180; return {v.x*sinf(a)+v.y*cosf(a),-v.x*cosf(a)+v.y*sinf(a)};  // 将 theta*PI_F/180; return {v.x*sinf(a)+v.y*cosf(a),-v.x*c… 写入或初始化 a。
}  // 结束此函数或控制块。
struct Node { Vec p; float radius = 0; bool middle=false; }; // 正式路径按节点编号标记中间道路。
struct Config {  // 定义 Config（配置） 的类型成员或作用域。
    uint32_t id=0;  // 将 0 写入或初始化 id（编号）。
    uint8_t count=0;  // 将 0 写入或初始化 count（计数）。
    float startHeading=0,endHeading=0,maxRpm=80,accel=200,lateralAccel=300;  // 将 0,endHeading=0,maxRpm=80,accel=200,lateralAccel=300 写入或初始化 startHeading（起点航向）。
    float lateralPpm=13.3f,forwardPpm=13.6f,rotationPpd=49.353889f;  // 将 13.3f,forwardPpm=13.6f,rotationPpd=49.353889f 写入或初始化 lateralPpm（横向每毫米脉冲数）。
    float kp=2,maxOmega=45,correction=20,omegaAccel=90;  // 将 2,maxOmega=45,correction=20,omegaAccel=90 写入或初始化 kp。
    uint8_t policy=0; // 0旧模型；试跑1自动/2停转；3正式外圈连续、内侧位置模式。
    float outset=60,margin=20;  // 将 60,margin=20 写入或初始化 outset。
    size_t slowApproachEdge=MAX_NODES;  // 将 MAX_NODES 写入或初始化 slowApproachEdge。
};  // 结束此类型定义。
struct Pose { Vec p,tangent; float curvature=0; };  // 定义 Pose 的类型成员或作用域。
enum SegmentKind { LINE=0, CUBIC=1, ROTATE=2 };  // 定义 SegmentKind 的类型成员或作用域。
// 段内位置用局部沿程表示，offset 指向该段在整条路线中的起点。
// 向跟踪器传坐标前要确认用的是全局还是局部沿程。
struct Segment {  // 定义 Segment 的类型成员或作用域。
    bool middle=false; // 平移段使用独立的中间道路速度和加速度。
    bool slowApproach=false;  // 将 false 写入或初始化 slowApproach。
    Vec start,end,center;  // 声明 start（起点），类型和可用范围由所在声明决定。
    float offset=0,length=0,radius=0,angle=0,sweep=0;  // 将 0,length=0,radius=0,angle=0,sweep=0 写入或初始化 offset（偏移）。
    uint8_t kind=LINE;  // 将 LINE 写入或初始化 kind。
    Vec control1,control2;  // 声明 control1，类型和可用范围由所在声明决定。
    float h0=0,h1=0;  // 将 0,h1=0 写入或初始化 h0。
    float lut[65]={};  // 将 {} 写入或初始化 lut[65]。
};  // 结束此类型定义。

class Route {  // 定义 Route（路线） 的类型成员或作用域。
public:  // 此后的成员采用 public 访问权限，直到下一个权限标签。
    Config config;  // 声明 config（配置），类型和可用范围由所在声明决定。
    Segment segments[MAX_SEGMENTS];  // 声明 segments，类型和可用范围由所在声明决定。
    size_t segmentCount=0,speedCount=0;  // 将 0,speedCount=0 写入或初始化 segmentCount（segment计数）。
    float length=0,headingDelta=0,spacing=5,plannedSeconds=0;  // 将 0,headingDelta=0,spacing=5,plannedSeconds=0 写入或初始化 length（长度）。
    /**
     * @brief 根据节点及配置构建路径段和速度规划；成功返回nullptr，否则为几何或配置错误原因。
     */
    const char *build(const Node *nodes,const Config &cfg);  // 声明接口 build，参数由此处的签名确定。
    /**
     * @brief 查询累计路径距离s(mm)处的位置、切向及曲率。
     */
    Pose at(float s) const;  // 声明接口 at，参数由此处的签名确定。
    /**
     * @brief 查询累计路径距离s(mm)处的计划地图航向(deg)。
     */
    float heading(float s) const;  // 声明接口 heading（航向），参数由此处的签名确定。
    /**
     * @brief 查询路径航向相对行程的变化率(deg/mm)。
     */
    float headingSlope(float s) const; // deg/mm
    /**
     * @brief 查询路径距离s(mm)处结合几何、加速度及停止约束后的速度上限(mm/s)。
     */
    float speedCap(float s) const;  // 声明接口 speedCap（速度Cap），参数由此处的签名确定。
    /**
     * @brief 检查路径距离s处的车体四角是否位于旧模型的场地边界内。
     */
    bool inside(float s) const;  // 声明接口 inside，参数由此处的签名确定。
    /**
     * @brief 查询指定路径段内局部行程local处的几何位置、切向和曲率。
     */
    Pose phaseAt(size_t index,float local) const;  // 声明接口 phaseAt（阶段At），参数由此处的签名确定。
    /**
     * @brief 查询指定路径段内局部行程local处的计划航向(deg)。
     */
    float phaseHeading(size_t index,float local) const;  // 声明接口 phaseHeading（阶段航向），参数由此处的签名确定。
    /**
     * @brief 查询指定路径段内航向随行程变化率(deg/mm)。
     */
    float phaseSlope(size_t index,float local) const;  // 声明接口 phaseSlope（阶段斜率），参数由此处的签名确定。
    /**
     * @brief 根据累计距离s(mm)寻找其所在的移动段下标。
     */
    size_t movingPhase(float s) const;  // 声明接口 movingPhase（moving阶段），参数由此处的签名确定。
    /**
     * @brief 查询从指定路径段开始向后的下一个停车距离(mm)。
     */
    float nextStop(size_t index) const;  // 声明接口 nextStop（下一停止），参数由此处的签名确定。
private:  // 此后的成员采用 private 访问权限，直到下一个权限标签。
    float speeds[MAX_SPEED_SAMPLES] = {};  // 将 {} 写入或初始化 speeds[MAX_SPEED_SAMPLES]。
    /**
     * @brief 结合四轮最大转速、曲率及航向变化计算当前位置的几何速度上限(mm/s)。
     */
    float geometricCap(float s) const;  // 声明接口 geometricCap，参数由此处的签名确定。
    /**
     * @brief 向路径追加起点a到终点b的直线段，更新累计行程。
     */
    void line(Vec a,Vec b);  // 声明接口 line，参数由此处的签名确定。
    /**
     * @brief 构建直线、外圈连续曲线及原地转向混合路径，成功返回nullptr。
     */
    const char *buildHybrid(const Node *nodes,const Config &cfg);  // 声明接口 buildHybrid，参数由此处的签名确定。
    /**
     * @brief 检查指定路径段的车体几何及障碍间距，成功返回nullptr。
     */
    const char *validatePhase(size_t index) const;  // 声明接口 validatePhase（validate阶段），参数由此处的签名确定。
    /**
     * @brief 追加指定位置p处from到to的原地转向段，角度单位deg；返回是否成功加入。
     */
    bool addRotation(Vec p,float from,float to);  // 声明接口 addRotation，参数由此处的签名确定。
};  // 结束此类型定义。

// 车体运动经轮组运动学转换后的驱动命令。
// 上层跟踪器输出这个结构，实机任务再把轮命令发往总线。
struct WheelCommand {  // 定义 WheelCommand（轮Command） 的类型成员或作用域。
    int16_t rpm[4] = {};  // 将 {} 写入或初始化 rpm[4]。
    Vec body; float omega=0;  // 声明 body（车体），类型和可用范围由所在声明决定。然后将 0 写入或初始化 omega。
};  // 结束此类型定义。
/**
 * @brief 将车体平移速度(mm/s)及角速度(deg/s)换算为四轮RPM，并按配置上限等比例缩放。
 */
WheelCommand wheels(Vec body,float omega,const Config &cfg);  // 声明接口 wheels（四轮），参数由此处的签名确定。
struct Output {  // 定义 Output（输出） 的类型成员或作用域。
    WheelCommand wheels;  // 声明 wheels（四轮），类型和可用范围由所在声明决定。
    float reference=0,error=0;  // 将 0,error=0 写入或初始化 reference。
    bool finished=false;  // 将 false 写入或初始化 finished。
    const char *failure=nullptr;  // 将 nullptr 写入或初始化 failure。
};  // 结束此类型定义。
// 纯计算状态机，硬件任务只负责提供样本、下发命令和记录同步启动时间。
class Tracker {  // 定义 Tracker 的类型成员或作用域。
public:  // 此后的成员采用 public 访问权限，直到下一个权限标签。
    float s=0,speed=0,omega=0;  // 将 0,speed=0,omega=0 写入或初始化 s。
    Vec estimated;  // 声明 estimated，类型和可用范围由所在声明决定。
    bool finishing=false;  // 将 false 写入或初始化 finishing。
    size_t phaseIndex=0;  // 将 0 写入或初始化 phaseIndex（阶段下标）。
    bool requireInertial=false;  // 将 false 写入或初始化 requireInertial。
    bool movingBlock=false; // 正式外圈只复用移动段，不接管内侧原地转向或引入试跑故障门槛。
    const char *stage="MOVE";  // 将 "MOVE" 写入或初始化 stage（阶段）。
    H30Attitude motionSample;  // 声明 motionSample（motion采样），类型和可用范围由所在声明决定。
    /**
     * @brief 保存本周期惯性样本，供Tracker停稳和航向控制使用。
     */
    void observeMotion(const H30Attitude &sample){motionSample=sample;}  // 将 sample;} 写入或初始化 motionSample（motion采样）。
    /**
     * @brief 初始化路径跟踪状态；actualHeading为开始时实测地图航向(deg)。
     */
    void begin(const Route &route,float actualHeading);  // 声明接口 begin，参数由此处的签名确定。
    /**
     * @brief 计算一个控制周期的四轮命令、航向误差及完成结果；dt为秒，sampleMs为样本毫秒数。
     */
    Output update(const Route &route,float actualHeading,float dt,uint32_t sampleMs);  // 声明接口 update，参数由此处的签名确定。
    /**
     * @brief 按已下发轮速、实测航向及时间dt(s)更新估算位置，不代表外部定位。
     */
    void integrate(const Route &route,float actualHeading,float dt);  // 声明接口 integrate，参数由此处的签名确定。
    /**
     * @brief 记录实际应用的四轮命令和时间，供积分及零轮速停稳判断使用。
     */
    void commit(const WheelCommand &command,uint32_t atMs=0) {  // 定义函数或成员 的入口 commit，参数由此处的签名确定。
        bool zero=true;for(int i=0;i<4;++i)if(command.rpm[i])zero=false;  // 将 true 写入或初始化 zero（零点）。然后按 int i=0;i<4;++i)if(command.rpm[i])zero=false; 组织循环的初值、继续条件和步进。
        if(zero&&!zeroCommanded)zeroSince=atMs;  // 当 zero&&!zeroCommanded 成立时进入此分支，否则继续后续流程。
        zeroCommanded=zero;applied=command;  // 将 zero 写入或初始化 zeroCommanded（零点Commanded）。然后将 command 写入或初始化 applied。
    }  // 结束此初始化列表或局部作用域。
private:  // 此后的成员采用 private 访问权限，直到下一个权限标签。
    WheelCommand applied;  // 声明 applied，类型和可用范围由所在声明决定。
    float badSeconds=0,finishSeconds=0,stableSeconds=0;  // 将 0,finishSeconds=0,stableSeconds=0 写入或初始化 badSeconds。
    uint32_t previousSample=0;  // 将 0 写入或初始化 previousSample（上一采样）。
    float previousActual=0,turnSeconds=0;  // 将 0,turnSeconds=0 写入或初始化 previousActual（上一实测）。
    bool rateReady=false;  // 将 false 写入或初始化 rateReady（rate就绪）。
    bool phaseStopped=false;  // 将 false 写入或初始化 phaseStopped（阶段已停止）。
    H30MotionQuiet quiet;  // 声明 quiet，类型和可用范围由所在声明决定。
    H30HeadingHold hold;  // 声明 hold（保持），类型和可用范围由所在声明决定。
    bool zeroCommanded=false;uint32_t zeroSince=0;  // 将 false 写入或初始化 zeroCommanded（零点Commanded）。然后将 0 写入或初始化 zeroSince（零点Since）。
    /**
     * @brief 推进混合路径当前段的速度、转向及停稳判定，返回本周期控制输出。
     */
    Output updateHybrid(const Route &route,float actual,float dt,uint32_t sampleMs);  // 声明接口 updateHybrid，参数由此处的签名确定。
};  // 结束此类型定义。
} // namespace continuous
