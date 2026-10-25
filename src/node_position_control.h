#pragma once
#include "continuous_route.h"

namespace node_position {
constexpr bool MOTOR_STALL_CHECK_ENABLED=false; // 临时禁用所有底盘电机的固件堵转检查；恢复时改为true
constexpr float PULSES_PER_REV=3200; // 要求底盘驱动配置为16细分，实车验收时核对
constexpr double STRAIGHT_ARRIVAL_ENCODER_TOLERANCE=5000; // 每轮65536单位/圈坐标中的允许误差
struct Settings {
    float speedMmS=2000,accelMmS2=2000,turnDegS=200,turnAccelDegS2=400,headingDeadbandDeg=1;
    float middleSpeedMmS=2000,middleAccelMmS2=2000;
    continuous::Vec startOffset; // 扫码停车位置相对首节点的偏移，并入首段四轮位置目标。
};
struct Motor {uint8_t flags=0;int64_t position=0;int16_t rpm=0;}; // 位置：65536单位/圈
struct Result {continuous::Vec estimated;float heading=0,s=0;};
// 直行期间仅监测持续偏航；航向校正在到位、近零转速后独立执行。
class HeadingGuard {
    bool softActive=false,hardActive=false,brakeActive=false;
    uint32_t softSince=0,hardSince=0,brakeSince=0;
public:
    /**
     * @brief 累计直行偏航持续时间并判断是否超出既有阈值；无有效航向时清除累计状态。
     * @return true 表示持续偏航达到现有失败条件。
     */
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
    /**
     * @brief 读取节点控制使用的设备毫秒数。
     */
    virtual uint32_t now()=0;
    /**
     * @brief 让出当前任务指定毫秒数，供同步兼容入口使用。
     */
    virtual void wait(uint32_t ms)=0;
    /**
     * @brief 查询当前路径是否被取消。
     */
    virtual bool cancelled()=0;
    /**
     * @brief 读取同一帧惯性数据与地图航向；成功返回 nullptr，否则返回原因。
     */
    virtual const char *imu(H30Attitude &sample,float &heading)=0;
    /**
     * @brief 读取单个电机的状态标志、编码器位置和RPM；成功返回 nullptr。
     */
    virtual const char *read(uint8_t id,Motor &state)=0;
    /** @brief 仅查询一个电机的转速(RPM)，供位置动作运行中的单轮轮询使用。 */
    virtual const char *readSpeed(uint8_t id,int16_t &rpm)=0;
    /**
     * @brief 读取指定电机编码器位置，单位65536/圈；成功返回 nullptr。
     */
    virtual const char *readPosition(uint8_t id,int64_t &position) {
        Motor state;const char *error=read(id,state);if(!error)position=state.position;return error;
    }
    /**
     * @brief 将指定电机当前位置设为相对运动起点；成功返回 nullptr。
     */
    virtual const char *zero(uint8_t id)=0;
    // 每个动作只装载一次绝对目标、最大轮速和加速度，由驱动器完成加减速。
    /**
     * @brief 装载指定电机的绝对脉冲目标、方向、最大RPM和加速度档位，等待 sync 同步启动。
     */
    virtual const char *target(uint8_t id,bool positive,uint16_t rpm,uint8_t acceleration,uint32_t pulses)=0;
    /**
     * @brief 同步启动此前装载的四轮位置动作；成功返回 nullptr。
     */
    virtual const char *sync()=0;
    /**
     * @brief 下发四轮轮速并同步执行，供连续曲线及扫码运动使用。
     */
    virtual const char *velocity(const continuous::WheelCommand &command)=0;
    /**
     * @brief 立即停止当前底盘运动。
     */
    virtual void halt()=0;
    /**
     * @brief 输出节点控制阶段、估算位置与参考航向诊断。
     */
    virtual void report(const char *stage,const Result &result,float reference)=0;
    /**
     * @brief 通知硬件层开始一个位置动作，用于重置单次动作的诊断状态。
     */
    virtual void beginAction(){}
    /**
     * @brief 输出当前动作的计时节点名称。
     */
    virtual void timing(const char *){}
    /**
     * @brief 输出四轮到位残差及实际状态，供定位到位耗时问题。
     */
    virtual void arrivalDiagnostic(float,const Motor (&)[4]){}
    /** @brief 输出代表轮停转后的四轮实测对比；残差单位为直行mm或转向deg，时间单位ms。 */
    virtual void arrivalComparison(uint8_t representative,uint32_t readMs,uint32_t waitMs,
        bool arrived,bool rotating,const float (&residual)[4],const Motor (&states)[4])=0;
    /**
     * @brief 输出指定电机的目标、实际反馈及失败原因。
     */
    virtual void motorDiagnostic(uint8_t,const char *,double,const Motor &){}
};
/**
 * @brief 把路径速度与脉冲换算系数转换为驱动器最大转速RPM。
 */
inline uint16_t rpmFor(float speed,float ppm) {
    float rpm=floorf(fabsf(speed)*ppm*60/PULSES_PER_REV);
    return static_cast<uint16_t>(continuous::clamp(rpm,1,5000));
}
// EMM V5: 每增加1 RPM用时(256-acc)*50us；0会关闭曲线，不能表示物理加速度。
// 将路径的距离/角度加速度结合当前脉冲系数换算为最接近的1~255档位。
/**
 * @brief 把距离/角度加速度换算为EMM V5的1~255加速度档位；使用当前脉冲系数。
 */
inline uint8_t driverAccelerationFor(float acceleration,float pulsesPerUnit) {
    float rpmPerSecond=fabsf(acceleration)*pulsesPerUnit*60/PULSES_PER_REV;
    if(!isfinite(rpmPerSecond)||rpmPerSecond<=0)return 1;
    float level=roundf(256-20000/rpmPerSecond);
    return static_cast<uint8_t>(continuous::clamp(level,1,255));
}
/**
 * @brief 同步推进节点位置路径直至结束；result 保存估算位置和航向。
 * @return nullptr 成功，否则返回错误原因。
 */
const char *run(const continuous::Route &route,const Settings &settings,IO &io,Result &result);

// 与 run 共用控制算法；每次 tick 只推进一次控制/通信周期，不等待物理行程结束。
class Runner {
    enum Stage { IDLE, START, QUIET, SELECT, ALIGN, ZERO, PREPARE, MOVE, OUTER, OUTER_QUIET, FINISHED } stage=IDLE;
    enum Action { ROTATION, STRAIGHT, CURVE, FINAL } action=FINAL;
    const continuous::Route *route=nullptr;
    const Settings *settings=nullptr;
    IO *io=nullptr;
    Result *output=nullptr;
    const char *error=nullptr;
    size_t index=0,after=0,curveBegin=0,observationIndex=0;
    continuous::Vec offset,end,start;
    float distance=0,targetHeading=0,baseS=0,delta=0,ppm=0,maximum=0,acceleration=0,startHeading=0;
    double unitsPerDistance=0,tolerance=0,hardLimit=0;
    uint32_t due=0,quietStarted=0,started=0,previous=0,deadline=0,lastReport=0,lastArrivalReport=0;
    bool prepared=false,valid=false,rotating=false,scanExit=false,sent=false,braking=false,turningForAlignment=false;
    uint8_t zeroId=1,attempts=0,movingWheels=0;
    uint16_t peakMeasuredRpm=0;
    uint8_t representative=1;
    int16_t previousRepresentativeRpm=0;
    double representativePulses=0;
    uint32_t representativeSampleAt=0,representativeStopAt=0,lastComparison=0;
    bool representativeMoved=false,representativeStopped=false,comparisonReported=false;
    Motor stopped[4];
    int encoderSign[4]={};
    int64_t origin[4]={},goals[4]={},positions[4]={};
    uint32_t travelPulses[4]={};
    bool directions[4]={};
    HeadingGuard headingGuard;
    bool quietSeen=false;
    uint32_t residualSince=0;
    uint8_t quietCount=0;
    int64_t quietAnchor[4]={};
    continuous::Tracker tracker;
    continuous::WheelCommand applied;
    float curveStop=0;
    bool finalTranslation=false,terminalTurn=false;
    /**
     * @brief 读取惯性数据并处理当前路径的航向参考可用性；成功返回 nullptr。
     */
    const char *sample(H30Attitude &motion);
    /**
     * @brief 记录路径失败原因，切到结束阶段并停车。
     */
    void fail(const char *reason);
    /**
     * @brief 准备当前直行或转向动作的四轮目标与速度曲线；参数表示是否为到位校角。
     */
    void startMove(bool turnForAlignment);
    /**
     * @brief 结束当前航向对齐，转入后续路径选择阶段。
     */
    void aligned();
    /**
     * @brief 完成当前移动段并推进路径索引和估算结果。
     */
    void arrived();
    /**
     * @brief 处理当前四轮位置动作的一次反馈、偏航及到位检查。
     */
    void moveTick();
    /** @brief 按估算或最终回读进度更新位置、编码器航向及既有偏航检查；不作为到位判据。 */
    const char *updateProgress(float progress,const H30Attitude &sample,bool idle);
    /**
     * @brief 根据四轮反馈更新本次动作行程和到位残差；成功返回 nullptr。
     */
    const char *displacement(const Motor (&states)[4]);
public:
    /**
     * @brief 初始化整条节点路径，记录配置、硬件接口和输出结果；后续由 tick 推进。
     */
    void begin(const continuous::Route &path,const Settings &config,IO &hardware,Result &result);
    /**
     * @brief 推进当前节点控制阶段一次，包括起步、段选择、位置目标下发及到位反馈。
     */
    void tick();
    /**
     * @brief 判断节点路径是否仍在运行。
     */
    bool active() const { return stage!=IDLE&&stage!=FINISHED; }
    /**
     * @brief 读取节点路径错误原因；nullptr 表示未记录错误。
     */
    const char *failure() const { return error; }
    /**
     * @brief 返回到下次推进所需等待毫秒数，供统一运动任务调度。
     */
    uint32_t nextDelay() const;
    /**
     * @brief 判断当前是否正在执行路径的最后一段连续平移。
     */
    bool lastTranslation() const { return finalTranslation&&stage==MOVE&&!rotating; }
    /**
     * @brief 判断是否进入最终进站平移前的准备阶段，用于提前展开视觉观察姿态。
     */
    bool observationApproach() const { return active()&&stage!=START&&stage!=QUIET&&index>=observationIndex; }
    /** @brief 最后平移完成，在路径末尾转向或终点校角开始前同步展开角点观察姿态。 */
    bool finalTurnApproach() const {
        if(!active()||stage!=ALIGN)return false;
        if(action==FINAL)return true;
        if(action!=ROTATION)return false;
        for(size_t i=after;i<route->segmentCount;++i)if(route->segments[i].kind!=continuous::ROTATE)return false;
        return true;
    }
    /**
     * @brief 返回当前平移动作剩余距离(mm)；运行中由代表轮转速估算，结束时由四轮位置校准。
     */
    float remainingDistance() const;
    /**
     * @brief 判断最后平移之后是否还有终点转向。
     */
    bool hasTerminalTurn() const { return terminalTurn; }
    /**
     * @brief 读取路径输出累计行程(mm)，无绑定输出时返回0。
     */
    float progress() const {return output?output->s:0;}
};
}
