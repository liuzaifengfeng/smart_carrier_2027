// 节点位置控制所需的硬件抽象、配置及运行数据。
// 控制算法只通过 IO 参数读反馈和发运动指令，硬件实现另见 node_position_hardware。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include "continuous_route.h"  // 引入 continuous_route.h 中的声明和定义。

namespace node_position {  // 定义 node_position（节点位置） 的类型成员或作用域。
constexpr bool MOTOR_STALL_CHECK_ENABLED=false; // 临时禁用所有底盘电机的固件堵转检查；恢复时改为true
constexpr float PULSES_PER_REV=3200; // 要求底盘驱动配置为16细分，实车验收时核对
constexpr double STRAIGHT_ARRIVAL_ENCODER_TOLERANCE=5000; // 每轮65536单位/圈坐标中的允许误差
constexpr float STRAIGHT_HEADING_KP=5; // 航向偏差1度给5度/秒修正，再叠加原积分和阻尼。
constexpr float STRAIGHT_LATERAL_KP=3; // 横向位置比例增益(1/s)：偏离10mm时给30mm/s平移修正。
struct Settings {  // 定义 Settings（设置） 的类型成员或作用域。
    float speedMmS=2000,accelMmS2=2000,turnDegS=200,turnAccelDegS2=400,headingDeadbandDeg=1;  // 将 2000,accelMmS2=2000,turnDegS=200,turnAccelDegS2=400,he… 写入或初始化 speedMmS（速度毫米S）。
    float middleSpeedMmS=2000,middleAccelMmS2=2000;  // 将 2000,middleAccelMmS2=2000 写入或初始化 middleSpeedMmS（middle速度毫米S）。
    continuous::Vec startOffset; // 起始估算位置相对路径起点的偏移，并入首段四轮位置目标。
    bool alignStartToRoad=false; // 先横移到道路中心，再按剩余纵向距离走首段。
    float startYOffset=0; // 圆盘离站时合入首条直线的地图 Y 偏移。
    bool recoverSmallResidual=false; // 正式路径按场地余量低速补偿一次小残差。
    bool outerStraightVelocity=false; // 正式节点导航启用，保留同步位置模式入口。
};  // 结束此类型定义。
struct Motor {uint8_t flags=0;int64_t position=0;int16_t rpm=0;}; // 位置：65536单位/圈
// 四轮有符号编码器行程只用于恢复地图 Y，不恢复 X 或航向。
// 四轮位移先换算为车体横向和前后分量，再根据进入圆盘区时的航向投影到地图 Y 轴。
inline float discYOffset(const float (&travel)[4],float lateralPpm,float forwardPpm,  // 定义函数或成员 discYOffset（圆盘纵向偏移），参数由此处的签名确定。
    float beforeHeading,float heading) {  // 声明 beforeHeading（before航向），类型和可用范围由所在声明决定。
    const continuous::Vec bodyTravel{  // 声明 bodyTravel（车体Travel），类型和可用范围由所在声明决定。
        (-travel[0]-travel[1]+travel[2]+travel[3])/(4*lateralPpm),  // 为当前列表或跨行表达式提供这一项 (-travel[0]-travel[1]+travel[2]+travel[3])/(4*lateralP…。
        (travel[0]-travel[1]+travel[2]-travel[3])/(4*forwardPpm)};  // 为当前列表或跨行表达式提供这一项 (travel[0]-travel[1]+travel[2]-travel[3])/(4*forwardPp…。
    const float middleHeading=beforeHeading+continuous::turn(beforeHeading,heading)*.5f;  // 将 beforeHeading+continuous::turn(beforeHeading,heading)*… 写入或初始化 middleHeading（middle航向）。
    return continuous::bodyToWorld(bodyTravel,middleHeading).y;  // 结束当前函数并返回 continuous::bodyToWorld(bodyTravel,middleHeading).y。
}  // 结束此函数或控制块。
// 从地图 Y 位移反算车体位移，再分配给四轮。
// 调用方据此发出对应的轮脉冲命令。
inline void discReturnWheelPulses(float worldY,float lateralPpm,float forwardPpm,float heading,int64_t (&pulses)[4]) {  // 定义函数或成员 的入口 discReturnWheelPulses（圆盘Return轮脉冲数），参数由此处的签名确定。
    const auto correction=continuous::worldToBody({0,-worldY},heading);  // 将 continuous::worldToBody({0,-worldY},heading) 写入或初始化 correction（修正量）。
    const float lateral=lateralPpm*correction.x,forward=forwardPpm*correction.y;  // 将 lateralPpm*correction.x,forward=forwardPpm*correction.y 写入或初始化 lateral（横向）。
    pulses[0]=lroundf(-lateral+forward);  // 将 lroundf(-lateral+forward) 写入或初始化 pulses[0]。
    pulses[1]=lroundf(-lateral-forward);  // 将 lroundf(-lateral-forward) 写入或初始化 pulses[1]。
    pulses[2]=-pulses[1];  // 将 -pulses[1] 写入或初始化 pulses[2]。
    pulses[3]=-pulses[0];  // 将 -pulses[0] 写入或初始化 pulses[3]。
}  // 结束此函数或控制块。
// 圆盘 Y 回位依据实测转速和编码器残差判定。
inline bool encoderReturnArrived(const Motor &motor,int64_t target) {  // 定义函数或成员 的入口 encoderReturnArrived（编码器ReturnArrived），参数由此处的签名确定。
    return motor.rpm>=-3&&motor.rpm<=3&&  // 结束当前函数并返回 motor.rpm>=-3&&motor.rpm<=3&&。
        fabs(double(motor.position-target))<=STRAIGHT_ARRIVAL_ENCODER_TOLERANCE;  // 继续上一行的组合条件，本行提供 fabs(double(motor.position-target))<=STRAIGHT_ARRIVAL_ENCODER_TOLERANCE;。
}  // 结束此函数或控制块。
struct Result {continuous::Vec estimated;float heading=0,s=0;};  // 定义 Result（结果） 的类型成员或作用域。
/** @brief 停轮后连续三帧新IMU航向的极差不超过0.2度；重复帧和停轮前数据不计数。 */
// 保存不同时间戳的航向观测，用于判断停车后的角度稳定性。
// 调用处提供样本以及判定所需条件。
class HeadingStable3 {  // 定义 HeadingStable3（航向稳定3） 的类型成员或作用域。
    uint32_t stoppedAt=0,lastStamp=0;  // 将 0,lastStamp=0 写入或初始化 stoppedAt（已停止At）。
    uint16_t lastSequence=0;  // 将 0 写入或初始化 lastSequence（最后Sequence）。
    uint8_t count=0;  // 将 0 写入或初始化 count（计数）。
    float headings[3]={};  // 将 {} 写入或初始化 headings[3]。
    uint8_t next=0;  // 将 0 写入或初始化 next（下一）。
public:  // 此后的成员采用 public 访问权限，直到下一个权限标签。
    void begin(uint32_t now){*this=HeadingStable3{};stoppedAt=now;}  // 将 HeadingStable3{};stoppedAt=now;} 写入或初始化 this。
    bool update(const H30Attitude &sample,float heading) {  // 定义函数或成员 的入口 update，参数由此处的签名确定。
        if(static_cast<int32_t>(sample.receivedAtMs-stoppedAt)<=0)return false;  // 若 static_cast<int32_t>(sample.receivedAtMs-stoppedAt)<=0，立即返回 false。
        if(count&&(sample.sequence==lastSequence||sample.receivedAtMs==lastStamp))return false;  // 若 count&&(sample.sequence==lastSequence||sample.received…，立即返回 false。
        lastSequence=sample.sequence;lastStamp=sample.receivedAtMs;  // 将 sample.sequence 写入或初始化 lastSequence（最后Sequence）。然后将 sample.receivedAtMs 写入或初始化 lastStamp（最后Stamp）。
        headings[next]=heading;next=(next+1)%3;  // 将 heading 写入或初始化 headings[next]。然后将 (next+1)%3 写入或初始化 next（下一）。
        if(count<3)++count;  // 当 count<3 成立时进入此分支，否则继续后续流程。
        if(count<3)return false;  // 若 count<3，立即返回 false。
        float low=0,high=0;  // 将 0,high=0 写入或初始化 low。
        for(uint8_t i=1;i<3;++i){const float angle=continuous::turn(headings[0],headings[i]);low=fminf(low,angle);high=fmaxf(high,angle);}  // 按 uint8_t i=1;i<3;++i 组织循环的初值、继续条件和步进。
        return high-low<=0.2f;  // 结束当前函数并返回 high-low<=0.2f。
    }  // 结束此函数或控制块。
};  // 结束此类型定义。
// 保留原有持续偏航检查；中间直行另外叠加实时航向闭环，外围直行仍停轮后校角。
class HeadingGuard {  // 定义 HeadingGuard（航向Guard） 的类型成员或作用域。
    bool softActive=false,hardActive=false,brakeActive=false;  // 将 false,hardActive=false,brakeActive=false 写入或初始化 softActive（soft活跃）。
    uint32_t softSince=0,hardSince=0,brakeSince=0;  // 将 0,hardSince=0,brakeSince=0 写入或初始化 softSince。
public:  // 此后的成员采用 public 访问权限，直到下一个权限标签。
    /**
     * @brief 累计直行偏航持续时间并判断是否超出既有阈值；无有效航向时清除累计状态。
     * @return true 表示持续偏航达到现有失败条件。
     */
    bool update(float error,uint32_t stamp,bool braking,bool stopped,bool available=true) {  // 定义函数或成员 的入口 update，参数由此处的签名确定。
        if(!available){*this=HeadingGuard{};return false;}  // 当 !available 成立时进入此分支，否则继续后续流程。
        if(braking&&!brakeActive)brakeSince=stamp;  // 当 braking&&!brakeActive 成立时进入此分支，否则继续后续流程。
        brakeActive=braking;  // 将 braking 写入或初始化 brakeActive（brake活跃）。
        float magnitude=fabsf(error);  // 将 fabsf(error) 写入或初始化 magnitude。
        if(magnitude>30) {  // 当 magnitude>30 成立时进入此分支，否则继续后续流程。
            if(!hardActive){hardActive=true;hardSince=stamp;}  // 当 !hardActive 成立时进入此分支，否则继续后续流程。
            if(stamp-hardSince>=200)return true;  // 若 stamp-hardSince>=200，立即返回 true。
        } else hardActive=false;  // 前一个判断不成立时执行此分支。
        bool grace=braking&&stamp-brakeSince<1200;  // 将 braking&&stamp-brakeSince<1200 写入或初始化 grace。
        if(magnitude>15&&!stopped&&!grace) {  // 当 magnitude>15&&!stopped&&!grace 成立时进入此分支，否则继续后续流程。
            if(!softActive){softActive=true;softSince=stamp;}  // 当 !softActive 成立时进入此分支，否则继续后续流程。
            if(stamp-softSince>=800)return true;  // 若 stamp-softSince>=800，立即返回 true。
        } else softActive=false;  // 前一个判断不成立时执行此分支。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此初始化列表或局部作用域。
};  // 结束此类型定义。
// 路线算法依赖的设备接口。
// 硬件时钟、读轮状态、发令及停车通过这里注入，实机封包和串口细节由硬件实现处理。
struct IO {  // 定义 IO（设备接口） 的类型成员或作用域。
    virtual ~IO()=default;  // 声明接口 IO（设备接口），参数由此处的签名确定。
    /**
     * @brief 读取节点控制使用的设备毫秒数。
     */
    virtual uint32_t now()=0;  // 声明接口 now，参数由此处的签名确定。
    /**
     * @brief 让出当前任务指定毫秒数，供同步兼容入口使用。
     */
    virtual void wait(uint32_t ms)=0;  // 声明接口 wait，参数由此处的签名确定。
    /**
     * @brief 查询当前路径是否被取消。
     */
    virtual bool cancelled()=0;  // 声明接口 cancelled（已取消），参数由此处的签名确定。
    /**
     * @brief 读取同一帧惯性数据与地图航向；成功返回 nullptr，否则返回原因。
     */
    virtual const char *imu(H30Attitude &sample,float &heading)=0;  // 声明接口 imu（惯性传感器），参数由此处的签名确定。
    /**
     * @brief 读取单个电机的状态标志、编码器位置和RPM；成功返回 nullptr。
     */
    virtual const char *read(uint8_t id,Motor &state)=0;  // 声明接口 read，参数由此处的签名确定。
    /** @brief 仅查询一个电机的转速(RPM)，供位置动作运行中的单轮轮询使用。 */
    virtual const char *readSpeed(uint8_t id,int16_t &rpm)=0;  // 声明接口 readSpeed（read速度），参数由此处的签名确定。
    /**
     * @brief 读取指定电机编码器位置，单位65536/圈；成功返回 nullptr。
     */
    virtual const char *readPosition(uint8_t id,int64_t &position) {  // 定义函数或成员 的入口 readPosition（read位置），参数由此处的签名确定。
        Motor state;const char *error=read(id,state);if(!error)position=state.position;return error;  // 声明 state（状态），类型和可用范围由所在声明决定。然后将 read(id,state) 写入或初始化 error（错误）。然后当 !error 成立时进入此分支，否则继续后续流程。然后结束当前函数并返回 error。
    }  // 结束此函数或控制块。
    /**
     * @brief 将指定电机当前位置设为相对运动起点；成功返回 nullptr。
     */
    virtual const char *zero(uint8_t id)=0;  // 声明接口 zero（零点），参数由此处的签名确定。
    // 每个动作只装载一次绝对目标、最大轮速和加速度，由驱动器完成加减速。
    /**
     * @brief 装载指定电机的绝对脉冲目标、方向、最大RPM和加速度档位，等待 sync 同步启动。
     */
    virtual const char *target(uint8_t id,bool positive,uint16_t rpm,uint8_t acceleration,uint32_t pulses)=0;  // 声明接口 target（目标），参数由此处的签名确定。
    /**
     * @brief 同步启动此前装载的四轮位置动作；成功返回 nullptr。
     */
    virtual const char *sync()=0;  // 声明接口 sync，参数由此处的签名确定。
    /**
     * @brief 下发四轮轮速并同步执行，供连续曲线及扫码运动使用。
     */
    virtual const char *velocity(const continuous::WheelCommand &command)=0;  // 声明接口 velocity（速度），参数由此处的签名确定。
    /**
     * @brief 立即停止当前底盘运动。
     */
    virtual void halt()=0;  // 声明接口 halt，参数由此处的签名确定。
    /**
     * @brief 输出节点控制阶段、估算位置与参考航向诊断。
     */
    virtual void report(const char *stage,const Result &result,float reference)=0;  // 声明接口 report，参数由此处的签名确定。
    /**
     * @brief 通知硬件层开始一个位置动作，用于重置单次动作的诊断状态。
     */
    virtual void beginAction(){}  // 定义函数或成员 beginAction，参数由此处的签名确定。
    /**
     * @brief 输出当前动作的计时节点名称。
     */
    virtual void timing(const char *){}  // 定义函数或成员 timing，参数由此处的签名确定。
    /** @brief 停轮后连续三帧实测航向稳定，仅用于记录停稳结果。 */
    virtual void headingStable(float){}  // 定义函数或成员 headingStable（航向稳定），参数由此处的签名确定。
    /** @brief 每200ms记录中间直行的目标/实测航向、纠偏角速度和规划线速度。 */
    virtual void headingControl(float,float,float,float){}  // 定义函数或成员 headingControl（航向Control），参数由此处的签名确定。
    /**
     * @brief 输出四轮到位残差及实际状态，供定位到位耗时问题。
     */
    virtual void arrivalDiagnostic(float,const Motor (&)[4]){}  // 定义函数或成员 arrivalDiagnostic，参数由此处的签名确定。
    /** @brief 输出代表轮停转后的四轮实测对比；残差单位为直行mm或转向deg，时间单位ms。 */
    virtual void arrivalComparison(uint8_t representative,uint32_t readMs,uint32_t waitMs,  // 定义函数或成员 arrivalComparison，参数由此处的签名确定。
        bool arrived,bool rotating,const float (&residual)[4],const Motor (&states)[4])=0;  // 声明接口 float，参数由此处的签名确定。
    /**
     * @brief 输出指定电机的目标、实际反馈及失败原因。
     */
    virtual void motorDiagnostic(uint8_t,const char *,double,const Motor &){}  // 定义函数或成员 motorDiagnostic，参数由此处的签名确定。
};  // 结束此类型定义。
/**
 * @brief 把路径速度与脉冲换算系数转换为驱动器最大转速RPM。
 */
inline uint16_t rpmFor(float speed,float ppm) {  // 定义函数或成员 的入口 rpmFor（转速For），参数由此处的签名确定。
    float rpm=floorf(fabsf(speed)*ppm*60/PULSES_PER_REV);  // 将 floorf(fabsf(speed)*ppm*60/PULSES_PER_REV) 写入或初始化 rpm（转速）。
    return static_cast<uint16_t>(continuous::clamp(rpm,1,5000));  // 结束当前函数并返回 static_cast<uint16_t>(continuous::clamp(rpm,1,5000))。
}  // 结束此函数或控制块。
// EMM V5: 每增加1 RPM用时(256-acc)*50us；0会关闭曲线，不能表示物理加速度。
// 将路径的距离/角度加速度结合当前脉冲系数换算为最接近的1~255档位。
/**
 * @brief 把距离/角度加速度换算为EMM V5的1~255加速度档位；使用当前脉冲系数。
 */
inline uint8_t driverAccelerationFor(float acceleration,float pulsesPerUnit) {  // 定义函数或成员 的入口 driverAccelerationFor（driver加速度For），参数由此处的签名确定。
    float rpmPerSecond=fabsf(acceleration)*pulsesPerUnit*60/PULSES_PER_REV;  // 将 fabsf(acceleration)*pulsesPerUnit*60/PULSES_PER_REV 写入或初始化 rpmPerSecond（转速PerSecond）。
    if(!isfinite(rpmPerSecond)||rpmPerSecond<=0)return 1;  // 若 !isfinite(rpmPerSecond)||rpmPerSecond<=0，立即返回 1。
    float level=roundf(256-20000/rpmPerSecond);  // 将 roundf(256-20000/rpmPerSecond) 写入或初始化 level。
    return static_cast<uint8_t>(continuous::clamp(level,1,255));  // 结束当前函数并返回 static_cast<uint8_t>(continuous::clamp(level,1,255))。
}  // 结束此函数或控制块。
/** @brief 将 EMM V5 的非零加速度档位换算为速度曲线所用的单位/s²。 */
inline float motionAccelerationFor(uint8_t level,float pulsesPerUnit) {  // 定义函数或成员 的入口 motionAccelerationFor（motion加速度For），参数由此处的签名确定。
    return 20000.0f*PULSES_PER_REV/((256-level)*60*pulsesPerUnit);  // 结束当前函数并返回 20000.0f*PULSES_PER_REV/((256-level)*60*pulsesPerUnit)。
}  // 结束此函数或控制块。
/**
 * @brief 同步推进节点位置路径直至结束；result 保存估算位置和航向。
 * @return nullptr 成功，否则返回错误原因。
 */
const char *run(const continuous::Route &route,const Settings &settings,IO &io,Result &result);  // 声明接口 run，参数由此处的签名确定。

// 与 run 共用控制算法；每次 tick 只推进一次控制/通信周期，不等待物理行程结束。
class Runner {  // 定义 Runner 的类型成员或作用域。
    enum Stage { IDLE, START, QUIET, SETTLE, SELECT, ALIGN, ZERO, PREPARE, MOVE, STRAIGHT_VELOCITY, OUTER, OUTER_QUIET, FINISHED } stage=IDLE;  // 定义 Stage（阶段） 的类型成员或作用域。
    Stage settleNext=SELECT;  // 将 SELECT 写入或初始化 settleNext（settle下一）。
    HeadingStable3 headingStable;  // 声明 headingStable（航向稳定），类型和可用范围由所在声明决定。
    enum Action { ROTATION, START_LATERAL, STRAIGHT, CURVE, FINAL } action=FINAL;  // 定义 Action 的类型成员或作用域。
    const continuous::Route *route=nullptr;  // 将 nullptr 写入或初始化 route（路线）。
    const Settings *settings=nullptr;  // 将 nullptr 写入或初始化 settings（设置）。
    IO *io=nullptr;  // 将 nullptr 写入或初始化 io（设备接口）。
    Result *output=nullptr;  // 将 nullptr 写入或初始化 output（输出）。
    const char *error=nullptr;  // 将 nullptr 写入或初始化 error（错误）。
    size_t index=0,after=0,curveBegin=0,observationIndex=0;  // 将 0,after=0,curveBegin=0,observationIndex=0 写入或初始化 index（下标）。
    continuous::Vec offset,end,start;  // 声明 offset（偏移），类型和可用范围由所在声明决定。
    float startYOffset=0;  // 将 0 写入或初始化 startYOffset（起点纵向偏移）。
    float distance=0,targetHeading=0,baseS=0,delta=0,ppm=0,maximum=0,acceleration=0,startHeading=0;  // 将 0,targetHeading=0,baseS=0,delta=0,ppm=0,maximum=0,acce… 写入或初始化 distance（距离）。
    double unitsPerDistance=0,tolerance=0,hardLimit=0;  // 将 0,tolerance=0,hardLimit=0 写入或初始化 unitsPerDistance（unitsPer距离）。
    uint32_t due=0,quietStarted=0,started=0,previous=0,deadline=0,lastReport=0,lastArrivalReport=0;  // 将 0,quietStarted=0,started=0,previous=0,deadline=0,lastR… 写入或初始化 due。
    bool prepared=false,valid=false,rotating=false,scanExit=false,sent=false,braking=false,turningForAlignment=false;  // 将 false,valid=false,rotating=false,scanExit=false,sent=f… 写入或初始化 prepared。
    bool startLateralDone=false;  // 将 false 写入或初始化 startLateralDone（起点横向完成）。
    uint8_t zeroId=1,attempts=0,movingWheels=0;  // 将 1,attempts=0,movingWheels=0 写入或初始化 zeroId（零点编号）。
    uint16_t peakMeasuredRpm=0;  // 将 0 写入或初始化 peakMeasuredRpm（peakMeasured转速）。
    uint8_t representative=1;  // 将 1 写入或初始化 representative。
    int16_t previousRepresentativeRpm=0;  // 将 0 写入或初始化 previousRepresentativeRpm（上一Representative转速）。
    double representativePulses=0;  // 将 0 写入或初始化 representativePulses（representative脉冲数）。
    uint32_t representativeSampleAt=0,representativeStopAt=0,lastComparison=0;  // 将 0,representativeStopAt=0,lastComparison=0 写入或初始化 representativeSampleAt（representative采样At）。
    bool representativeMoved=false,representativeStopped=false,comparisonReported=false;  // 将 false,representativeStopped=false,comparisonReported=f… 写入或初始化 representativeMoved。
    Motor stopped[4];  // 声明 stopped（已停止），类型和可用范围由所在声明决定。
    int encoderSign[4]={};  // 将 {} 写入或初始化 encoderSign[4]。
    int64_t origin[4]={},goals[4]={},positions[4]={};  // 将 {},goals[4]={},positions[4]={} 写入或初始化 origin[4]。
    uint32_t travelPulses[4]={};  // 将 {} 写入或初始化 travelPulses[4]。
    bool directions[4]={};  // 将 {} 写入或初始化 directions[4]。
    HeadingGuard headingGuard;  // 声明 headingGuard（航向Guard），类型和可用范围由所在声明决定。
    H30HeadingHold headingHold;  // 声明 headingHold（航向保持），类型和可用范围由所在声明决定。
    float straightSpeed=0,straightTravel=0;  // 将 0,straightTravel=0 写入或初始化 straightSpeed（straight速度）。
    float approachStart=INFINITY;  // 将 INFINITY 写入或初始化 approachStart（approach起点）。
    bool straightStopping=false;  // 将 false 写入或初始化 straightStopping。
    bool quietSeen=false,residualRetried=false,headingRecoveryUsed=false;  // 将 false,residualRetried=false,headingRecoveryUsed=false 写入或初始化 quietSeen。
    uint32_t residualSince=0;  // 将 0 写入或初始化 residualSince。
    uint8_t quietCount=0;  // 将 0 写入或初始化 quietCount（quiet计数）。
    int64_t quietAnchor[4]={};  // 将 {} 写入或初始化 quietAnchor[4]。
    continuous::Tracker tracker;  // 声明 tracker，类型和可用范围由所在声明决定。
    continuous::WheelCommand applied;  // 声明 applied，类型和可用范围由所在声明决定。
    float curveStop=0;  // 将 0 写入或初始化 curveStop（curve停止）。
    bool finalTranslation=false,terminalTurn=false;  // 将 false,terminalTurn=false 写入或初始化 finalTranslation。
    /**
     * @brief 读取惯性数据并处理当前路径的航向参考可用性；成功返回 nullptr。
     */
    const char *sample(H30Attitude &motion);  // 声明接口 sample（采样），参数由此处的签名确定。
    /**
     * @brief 记录路径失败原因，切到结束阶段并停车。
     */
    void fail(const char *reason);  // 声明接口 fail，参数由此处的签名确定。
    /**
     * @brief 准备当前直行或转向动作的四轮目标与速度曲线；参数表示是否为到位校角。
     */
    void startMove(bool turnForAlignment);  // 声明接口 startMove（起点Move），参数由此处的签名确定。
    /**
     * @brief 结束当前航向对齐，转入后续路径选择阶段。
     */
    void aligned();  // 声明接口 aligned，参数由此处的签名确定。
    /**
     * @brief 完成当前移动段并推进路径索引和估算结果。
     */
    void arrived();  // 声明接口 arrived，参数由此处的签名确定。
    /**
     * @brief 处理当前四轮位置动作的一次反馈、偏航及到位检查。
     */
    void moveTick();  // 声明接口 moveTick（move轮询），参数由此处的签名确定。
    /** @brief 速度模式直行：四轮/H30里程计、横向位置P补偿及航向PI+D纠偏。 */
    void straightVelocityTick();  // 声明接口 straightVelocityTick（straight速度轮询），参数由此处的签名确定。
    /** @brief 按估算或最终回读进度更新位置、编码器航向及既有偏航检查；不作为到位判据。 */
    const char *updateProgress(float progress,const H30Attitude &sample,bool idle);  // 声明接口 updateProgress，参数由此处的签名确定。
    /**
     * @brief 根据四轮反馈更新本次动作行程和到位残差；成功返回 nullptr。
     */
    const char *displacement(const Motor (&states)[4]);  // 声明接口 displacement，参数由此处的签名确定。
public:  // 此后的成员采用 public 访问权限，直到下一个权限标签。
    /**
     * @brief 初始化整条节点路径，记录配置、硬件接口和输出结果；后续由 tick 推进。
     */
    void begin(const continuous::Route &path,const Settings &config,IO &hardware,Result &result);  // 声明接口 begin，参数由此处的签名确定。
    /**
     * @brief 推进当前节点控制阶段一次，包括起步、段选择、位置目标下发及到位反馈。
     */
    void tick();  // 声明接口 tick（轮询），参数由此处的签名确定。
    /**
     * @brief 判断节点路径是否仍在运行。
     */
    bool active() const { return stage!=IDLE&&stage!=FINISHED; }  // 定义函数或成员 active（活跃），参数由此处的签名确定。
    /**
     * @brief 读取节点路径错误原因；nullptr 表示未记录错误。
     */
    const char *failure() const { return error; }  // 定义函数或成员 failure，参数由此处的签名确定。
    /**
     * @brief 返回到下次推进所需等待毫秒数，供统一运动任务调度。
     */
    uint32_t nextDelay() const;  // 声明接口 nextDelay（下一延时），参数由此处的签名确定。
    /**
     * @brief 判断当前是否正在执行路径的最后一段连续平移。
     */
    bool lastTranslation() const { return finalTranslation&&(stage==MOVE||stage==STRAIGHT_VELOCITY)&&!rotating; }  // 定义函数或成员 lastTranslation（最后Translation），参数由此处的签名确定。
    /**
     * @brief 判断是否进入最终进站平移前的准备阶段，用于提前展开视觉观察姿态。
     */
    bool observationApproach() const { return active()&&stage!=START&&stage!=QUIET&&stage!=SETTLE&&index>=observationIndex; }  // 将 observationIndex; } 写入或初始化 index>。
    /** @brief 最后平移完成，在路径末尾转向或终点校角开始前同步展开角点观察姿态。 */
    bool finalTurnApproach() const {  // 定义函数或成员 的入口 finalTurnApproach，参数由此处的签名确定。
        if(!active()||stage!=ALIGN)return false;  // 若 !active()||stage!=ALIGN，立即返回 false。
        if(action==FINAL)return true;  // 若 action==FINAL，立即返回 true。
        if(action!=ROTATION)return false;  // 若 action!=ROTATION，立即返回 false。
        for(size_t i=after;i<route->segmentCount;++i)if(route->segments[i].kind!=continuous::ROTATE)return false;  // 按 size_t i=after;i<route->segmentCount;++i)if(route->seg… 组织循环的初值、继续条件和步进。
        return true;  // 结束当前函数并返回 true。
    }  // 结束此函数或控制块。
    /**
     * @brief 返回当前平移动作剩余距离(mm)；运行中由代表轮转速估算，结束时由四轮位置校准。
     */
    float remainingDistance() const;  // 声明接口 remainingDistance（剩余距离），参数由此处的签名确定。
    /**
     * @brief 判断最后平移之后是否还有终点转向。
     */
    bool hasTerminalTurn() const { return terminalTurn; }  // 定义函数或成员 hasTerminalTurn，参数由此处的签名确定。
    /**
     * @brief 读取路径输出累计行程(mm)，无绑定输出时返回0。
     */
    float progress() const {return output?output->s:0;}  // 定义 progress，调用时返回 output?output->s:0。
    // 与节点绝对位置指令使用相同的编码器至命令方向映射。
    int encoderToCommandSign(uint8_t wheel) const {return encoderSign[wheel]?encoderSign[wheel]:1;}  // 定义函数或成员 encoderToCommandSign（编码器ToCommandSign），参数由此处的签名确定。
};  // 结束此类型定义。
}  // 结束命名空间。
