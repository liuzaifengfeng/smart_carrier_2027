// 位置路线状态机。
// 依次处理起步静止、阶段选择、校角和轮位移，直线速度阶段还将四轮编码器变化分解为车体位移。

#include "node_position_control.h"  // 引入 node_position_control.h 中的声明和定义。
#include "field_geometry.h"  // 引入 field_geometry.h 中的声明和定义。
#include <stdlib.h>  // 引入 stdlib.h 中的声明和定义。

namespace node_position {  // 定义 node_position（节点位置） 的类型成员或作用域。
using namespace continuous;  // 引入名称或类型别名 namespace continuous。
namespace {  // 开始匿名命名空间，本文件内部共享其中的定义。
constexpr int64_t ENCODER_DIRECTION_MIN_COUNTS=128;  // 将 128 写入或初始化 ENCODER_DIRECTION_MIN_COUNTS（编码器方向最小COUNTS）。

const char *health(IO &io,uint8_t id,const Motor &m,double target=0) {  // 定义函数或成员 的入口 health，参数由此处的签名确定。
    const char *error=!(m.flags&1)?"MOTOR_DISABLED":  // 将 !(m.flags&1)?"MOTOR_DISABLED": 写入或初始化 error（错误）。
        MOTOR_STALL_CHECK_ENABLED&&(m.flags&12)?"MOTOR_STALL":nullptr;  // 续写当前声明或运算，提供 MOTOR_STALL_CHECK_ENABLED&&(m.flags&12)?"MOTOR_STALL":…。
    if(error)io.motorDiagnostic(id,error,target,m);  // 当 error 成立时进入此分支，否则继续后续流程。
    return error;  // 结束当前函数并返回 error。
}  // 结束此初始化列表或局部作用域。
}  // 结束命名空间。
/**
 * @brief 读取惯性数据并处理当前路径的航向参考可用性；成功返回 nullptr。
 */
const char *Runner::sample(H30Attitude &motion) {  // 定义函数或成员 的入口 Runner::sample（Runner采样），参数由此处的签名确定。
    if(io->cancelled())return "STOPPED";  // 若 io->cancelled()，立即返回 "STOPPED"。
    float measured=output->heading;  // 将 output->heading 写入或初始化 measured。
    const char *failure=io->imu(motion,measured);  // 将 io->imu(motion,measured) 写入或初始化 failure。
    if(failure||(motion.fields&7)!=7||!isfinite(measured))motion=H30Attitude{};  // 当 failure||(motion.fields&7)!=7||!isfinite(measured) 成立时进入此分支，否则继续后续流程。
    else output->heading=measured;  // 前一个判断不成立时执行此分支。
    return io->cancelled()?"STOPPED":nullptr;  // 结束当前函数并返回 io->cancelled()?"STOPPED":nullptr。
}  // 结束此函数或控制块。
/**
 * @brief 记录路径失败原因，切到结束阶段并停车。
 */
void Runner::fail(const char *reason) {error=reason;stage=FINISHED;io->halt();}  // 将 reason;stage=FINISHED;io->halt();} 写入或初始化 error（错误）。
/**
 * @brief 初始化整条节点路径，记录配置、硬件接口和输出结果；后续由 tick 推进。
 */
void Runner::begin(const Route &path,const Settings &config,IO &hardware,Result &result) {  // 定义函数或成员 的入口 Runner::begin，参数由此处的签名确定。
    *this=Runner{};route=&path;settings=&config;io=&hardware;output=&result;  // 将 Runner{};route=&path;settings=&config;io=&hardware;out… 写入或初始化 this。
    offset=config.startOffset;startYOffset=config.startYOffset;  // 将 config.startOffset 写入或初始化 offset（偏移）。然后将 config.startYOffset 写入或初始化 startYOffset（起点纵向偏移）。
    result.estimated=path.segments[0].start+offset+Vec{0,startYOffset};  // 将 path.segments[0].start+offset+Vec{0,startYOffset} 写入或初始化 result.estimated。
    // 最后一组连续平移前的转向阶段即开始准备观察姿态；单段路径从起步准备。
    size_t finalBlock=path.segmentCount;  // 将 path.segmentCount 写入或初始化 finalBlock。
    while(finalBlock&&path.segments[finalBlock-1].kind==ROTATE)--finalBlock;  // 当 finalBlock&&path.segments[finalBlock-1].kind==ROTATE 成立时继续循环。
    while(finalBlock&&path.segments[finalBlock-1].kind!=ROTATE)--finalBlock;  // 当 finalBlock&&path.segments[finalBlock-1].kind!=ROTATE 成立时继续循环。
    observationIndex=finalBlock?finalBlock-1:0;  // 将 finalBlock?finalBlock-1:0 写入或初始化 observationIndex（observation下标）。
    stage=START;due=hardware.now();  // 将 START 写入或初始化 stage（阶段）。然后将 hardware.now() 写入或初始化 due。
}  // 结束此函数或控制块。
/**
 * @brief 返回到下次推进所需等待毫秒数，供统一运动任务调度。
 */
uint32_t Runner::nextDelay() const {  // 定义函数或成员 的入口 Runner::nextDelay（Runner下一延时），参数由此处的签名确定。
    const int32_t left=static_cast<int32_t>(due-io->now());return left>0?left:0;  // 将 static_cast<int32_t>(due-io->now()) 写入或初始化 left（左）。然后结束当前函数并返回 left>0?left:0。
}  // 结束此函数或控制块。
/**
 * @brief 返回当前平移动作剩余距离(mm)；运行中由代表轮转速估算，结束时由四轮位置校准。
 */
float Runner::remainingDistance() const {  // 定义函数或成员 的入口 Runner::remainingDistance（Runner剩余距离），参数由此处的签名确定。
    return fmaxf(0,fabsf(distance)-(output->s-baseS));  // 结束当前函数并返回 fmaxf(0,fabsf(distance)-(output->s-baseS))。
}  // 结束此函数或控制块。
/**
 * @brief 准备当前直行或转向动作的四轮目标与速度曲线；参数表示是否为到位校角。
 */
void Runner::startMove(bool alignment) {  // 定义函数或成员 的入口 Runner::startMove（Runner起点Move），参数由此处的签名确定。
    turningForAlignment=alignment;rotating=alignment;  // 将 alignment 写入或初始化 turningForAlignment。然后将 alignment 写入或初始化 rotating。
    H30Attitude m;const char *failure=sample(m);if(failure){fail(failure);return;}  // 将 sample(m);if(failure){fail(failure);return;} 写入或初始化 failure。
    delta=rotating?turn(output->heading,targetHeading):distance;  // 将 rotating?turn(output->heading,targetHeading):distance 写入或初始化 delta。
    ppm=rotating?route->config.rotationPpd:route->config.forwardPpm;  // 将 rotating?route->config.rotationPpd:route->config.forwa… 写入或初始化 ppm（每毫米脉冲数）。
    const uint32_t pulses=static_cast<uint32_t>(lroundf(fabsf(delta)*ppm));  // 将 static_cast<uint32_t>(lroundf(fabsf(delta)*ppm)) 写入或初始化 pulses（脉冲数）。
    if(!pulses){arrived();return;}  // 当 !pulses 成立时进入此分支，否则继续后续流程。
    Vec body=scanExit&&!rotating?worldToBody(end-output->estimated,action==START_LATERAL?output->heading:targetHeading):Vec{};  // 将 scanExit&&!rotating?worldToBody(end-output->estimated,… 写入或初始化 body（车体）。
    const float px=body.x*route->config.lateralPpm,py=body.y*route->config.forwardPpm;  // 将 body.x*route->config.lateralPpm,py=body.y*route->confi… 写入或初始化 px。
    const float scanPulses[4]={-px+py,-px-py,px+py,px-py};  // 将 {-px+py,-px-py,px+py,px-py} 写入或初始化 scanPulses[4]。
    movingWheels=0;  // 将 0 写入或初始化 movingWheels（moving四轮）。
    for(int i=0;i<4;++i) {  // 按 int i=0;i<4;++i 组织循环的初值、继续条件和步进。
        travelPulses[i]=scanExit&&!rotating?static_cast<uint32_t>(lroundf(fabsf(scanPulses[i]))):pulses;  // 将 scanExit&&!rotating?static_cast<uint32_t>(lroundf(fabs… 写入或初始化 travelPulses[i]。
        if(travelPulses[i])++movingWheels;  // 当 travelPulses[i] 成立时进入此分支，否则继续后续流程。
        directions[i]=rotating?delta>=0:((distance>=0)==(i%2==0));  // 将 rotating?delta>=0:((distance>=0)==(i%2==0)) 写入或初始化 directions[i]。
        if(scanExit&&!rotating)directions[i]=scanPulses[i]>=0;  // 当 scanExit&&!rotating 成立时进入此分支，否则继续后续流程。
    }  // 结束此循环体。
    zeroId=1;stage=prepared?PREPARE:ZERO;  // 将 1 写入或初始化 zeroId（零点编号）。然后将 prepared?PREPARE:ZERO 写入或初始化 stage（阶段）。
}  // 结束此函数或控制块。
/**
 * @brief 结束当前航向对齐，转入后续路径选择阶段。
 */
void Runner::aligned() {  // 定义函数或成员 的入口 Runner::aligned，参数由此处的签名确定。
    if(action==FINAL){stage=FINISHED;io->halt();return;}  // 当 action==FINAL 成立时进入此分支，否则继续后续流程。
    if(action==ROTATION){index=after;stage=SELECT;return;}  // 当 action==ROTATION 成立时进入此分支，否则继续后续流程。
    if(action==STRAIGHT||action==START_LATERAL){startMove(false);return;}  // 当 action==STRAIGHT||action==START_LATERAL 成立时进入此分支，否则继续后续流程。
    tracker=Tracker{};tracker.begin(*route,output->heading);tracker.movingBlock=true;  // 将 Tracker{};tracker.begin(*route,output->heading);tracke… 写入或初始化 tracker。
    tracker.phaseIndex=curveBegin;tracker.s=route->segments[curveBegin].offset;tracker.estimated=output->estimated;  // 将 curveBegin 写入或初始化 tracker.phaseIndex。然后将 route->segments[curveBegin].offset 写入或初始化 tracker.s。然后将 output->estimated 写入或初始化 tracker.estimated。
    const Segment &last=route->segments[after-1];curveStop=last.offset+last.length;  // 将 route->segments[after-1] 写入或初始化 last（最后）。然后将 last.offset+last.length 写入或初始化 curveStop（curve停止）。
    for(int i=0;i<4;++i)positions[i]=stopped[i].position;  // 按 int i=0;i<4;++i)positions[i]=stopped[i].position; 组织循环的初值、继续条件和步进。
    applied=WheelCommand{};previous=io->now();lastReport=0;valid=false;  // 将 WheelCommand{};previous=io->now();lastReport=0;valid=f… 写入或初始化 applied。
    io->beginAction();io->timing("OUTER_DRIFT_SENT");stage=OUTER;  // 调用 io->beginAction。然后调用 io->timing，传入 "OUTER_DRIFT_SENT"。然后将 OUTER 写入或初始化 stage（阶段）。
}  // 结束此函数或控制块。
/**
 * @brief 完成当前移动段并推进路径索引和估算结果。
 */
void Runner::arrived() {  // 定义函数或成员 的入口 Runner::arrived，参数由此处的签名确定。
    if(turningForAlignment){stage=ALIGN;return;}  // 当 turningForAlignment 成立时进入此分支，否则继续后续流程。
    if(action==START_LATERAL){startLateralDone=true;stage=SELECT;return;}  // 当 action==START_LATERAL 成立时进入此分支，否则继续后续流程。
    offset={};index=after;stage=SELECT;  // 将 {};index=after;stage=SELECT 写入或初始化 offset（偏移）。
}  // 结束此函数或控制块。
const char *Runner::updateProgress(float progress,const H30Attitude &m,bool idle) {  // 定义函数或成员 的入口 Runner::updateProgress，参数由此处的签名确定。
    if(rotating&&(m.fields&7)!=7)output->heading=wrap(startHeading+delta*progress);  // 当 rotating&&(m.fields&7)!=7 成立时进入此分支，否则继续后续流程。
    const float headingError=turn(output->heading,targetHeading);  // 将 turn(output->heading,targetHeading) 写入或初始化 headingError（航向错误）。
    if(!rotating&&headingGuard.update(headingError,m.receivedAtMs,braking,idle,(m.fields&7)==7)) {  // 当 !rotating&&headingGuard.update(headingError,m.received… 成立时进入此分支，否则继续后续流程。
        io->report("HEADING_ERROR",*output,targetHeading);return "HEADING_ERROR";  // 上报当前控制状态，传入 "HEADING_ERROR",*output,targetHeading。然后结束当前函数并返回 "HEADING_ERROR"。
    }  // 结束此条件分支。
    if(!rotating) {  // 当 !rotating 成立时进入此分支，否则继续后续流程。
        if(stage!=STRAIGHT_VELOCITY)output->estimated=start+(end-start)*progress;  // 当 stage!=STRAIGHT_VELOCITY 成立时进入此分支，否则继续后续流程。
        output->s=baseS+fabsf(delta)*progress;  // 将 baseS+fabsf(delta)*progress 写入或初始化 output->s。
        return field::check(output->estimated.x,output->estimated.y,output->heading,0,false);  // 结束当前函数并返回 field::check(output->estimated.x,output->estimated.y,o…。
    }  // 结束此条件分支。
    return nullptr;  // 结束当前函数并返回 nullptr。
}  // 结束此函数或控制块。
/**
 * @brief 运动中只读代表轮转速；近零时回读四轮，保留原位置/转速到位条件并输出对比。
 */
void Runner::moveTick() {  // 定义函数或成员 的入口 Runner::moveTick（Runnermove轮询），参数由此处的签名确定。
    const uint32_t cycleAt=io->now();  // 将 io->now() 写入或初始化 cycleAt。
    if(cycleAt-started>deadline){fail("MOTOR_ARRIVAL_TIMEOUT");return;}  // 当 cycleAt-started>deadline 成立时进入此分支，否则继续后续流程。
    const float dt=(cycleAt-previous)*.001f;previous=cycleAt;  // 将 (cycleAt-previous)*.001f 写入或初始化 dt（时间差）。然后将 cycleAt 写入或初始化 previous（上一）。
    if(dt>.5f){fail("CONTROL_OVERRUN");return;}  // 当 dt>.5f 成立时进入此分支，否则继续后续流程。
    H30Attitude m;const char *failure=sample(m);if(failure){fail(failure);return;}  // 将 sample(m);if(failure){fail(failure);return;} 写入或初始化 failure。
    if(sent) {  // 当 sent 成立时进入此分支，否则继续后续流程。
        int16_t rpm=0;  // 将 0 写入或初始化 rpm（转速）。
        failure=io->readSpeed(representative,rpm);if(failure){fail(failure);return;}  // 将 io->readSpeed(representative,rpm);if(failure){fail(fai… 写入或初始化 failure。
        const uint32_t now=io->now();  // 将 io->now() 写入或初始化 now。
        representativePulses+=(abs(previousRepresentativeRpm)+abs(rpm))*.5  // 把 (abs(previousRepresentativeRpm)+abs(rpm))*.5 累加到 representativePulses（representative脉冲数）。
            *PULSES_PER_REV*(now-representativeSampleAt)/60000.0;  // 续写当前声明或运算，提供 *PULSES_PER_REV*(now-representativeSampleAt)/60000.0;。
        previousRepresentativeRpm=rpm;representativeSampleAt=now;  // 将 rpm 写入或初始化 previousRepresentativeRpm（上一Representative转速）。然后将 now 写入或初始化 representativeSampleAt（representative采样At）。
        if(abs(rpm)>3)representativeMoved=true;  // 当 abs(rpm)>3 成立时进入此分支，否则继续后续流程。
        if(peakMeasuredRpm>abs(rpm)+3)braking=true;  // 当 peakMeasuredRpm>abs(rpm)+3 成立时进入此分支，否则继续后续流程。
        peakMeasuredRpm=static_cast<uint16_t>(fmaxf(peakMeasuredRpm,abs(rpm)));  // 将 static_cast<uint16_t>(fmaxf(peakMeasuredRpm,abs(rpm))) 写入或初始化 peakMeasuredRpm（peakMeasured转速）。
        if(abs(rpm)>3) {  // 当 abs(rpm)>3 成立时进入此分支，否则继续后续流程。
            // 中途位置仅用于进度估计；最终是否到位仍由下面四轮实测决定。
            const float progress=static_cast<float>(fmin(1,representativePulses/travelPulses[representative-1]));  // 将 static_cast<float>(fmin(1,representativePulses/travelP… 写入或初始化 progress。
            failure=updateProgress(progress,m,false);if(failure){fail(failure);return;}  // 将 updateProgress(progress,m,false);if(failure){fail(fail… 写入或初始化 failure。
            if(now-lastReport>=200){io->report(rotating?"POSITION_TURN":"POSITION_STRAIGHT",*output,targetHeading);lastReport=now;}  // 当 now-lastReport>=200 成立时进入此分支，否则继续后续流程。
            due=io->now()+(braking?1:20);return;  // 将 io->now()+(braking?1:20) 写入或初始化 due。然后结束当前函数，将执行权交回调用者。
        }  // 结束此条件分支。
        if(representativeMoved&&!representativeStopped){representativeStopped=true;representativeStopAt=now;}  // 当 representativeMoved&&!representativeStopped 成立时进入此分支，否则继续后续流程。
        // 起步时也可能读到0：只触发完整检查，不据此直接接续下一动作。
    }  // 结束此条件分支。
    const uint32_t checkStarted=io->now();  // 将 io->now() 写入或初始化 checkStarted。
    bool arrivedNow=sent,allIdle=true;double progress=0,maxResidual=0;Motor measured[4];  // 将 sent,allIdle=true 写入或初始化 arrivedNow。然后将 0,maxResidual=0 写入或初始化 progress。然后声明 measured，类型和可用范围由所在声明决定。
    float residuals[4],remainingPulses[4];  // 声明 residuals，类型和可用范围由所在声明决定。
    uint16_t measuredRpm=0;  // 将 0 写入或初始化 measuredRpm（measured转速）。
    for(uint8_t id=1;id<=4;++id) {  // 按 uint8_t id=1;id<=4;++id 组织循环的初值、继续条件和步进。
        Motor state=stopped[id-1];  // 将 stopped[id-1] 写入或初始化 state（状态）。
        const double goal=static_cast<double>(travelPulses[id-1])*65536/PULSES_PER_REV;  // 将 static_cast<double>(travelPulses[id-1])*65536/PULSES_P… 写入或初始化 goal。
        if(sent){failure=io->read(id,state);if(failure){fail(failure);return;}}  // 当 sent 成立时进入此分支，否则继续后续流程。
        const int64_t rawDelta=state.position-origin[id-1];  // 将 state.position-origin[id-1] 写入或初始化 rawDelta。
        const int sign=encoderSign[id-1]?encoderSign[id-1]:1;  // 将 encoderSign[id-1]?encoderSign[id-1]:1 写入或初始化 sign。
        const double absoluteGoal=static_cast<double>(goals[id-1])*sign*65536/PULSES_PER_REV;  // 将 static_cast<double>(goals[id-1])*sign*65536/PULSES_PER… 写入或初始化 absoluteGoal。
        failure=health(*io,id,state,absoluteGoal);if(failure){fail(failure);return;}  // 将 health(*io,id,state,absoluteGoal);if(failure){fail(fai… 写入或初始化 failure。
        measured[id-1]=state;  // 将 state 写入或初始化 measured[id-1]。
        const double current=encoderSign[id-1]?static_cast<double>(rawDelta)*sign*(directions[id-1]?1:-1):  // 将 encoderSign[id-1]?static_cast<double>(rawDelta)*sign*(… 写入或初始化 current（当前）。
            static_cast<double>(llabs(rawDelta));  // 声明接口 double>，参数由此处的签名确定。
        const double residual=fabs(current-goal);maxResidual=fmax(maxResidual,residual);  // 将 fabs(current-goal) 写入或初始化 residual。然后将 fmax(maxResidual,residual) 写入或初始化 maxResidual（最大Residual）。
        residuals[id-1]=static_cast<float>((current-goal)/unitsPerDistance);  // 将 static_cast<float>((current-goal)/unitsPerDistance) 写入或初始化 residuals[id-1]。
        remainingPulses[id-1]=(goal-current)*PULSES_PER_REV/65536*(directions[id-1]?1:-1);  // 将 (goal-current)*PULSES_PER_REV/65536*(directions[id-1]?… 写入或初始化 remainingPulses[id-1]。
        if(current>goal+hardLimit){io->motorDiagnostic(id,"MOTOR_POSITION_OVERRUN",absoluteGoal,state);fail("MOTOR_POSITION_OVERRUN");return;}  // 当 current>goal+hardLimit 成立时进入此分支，否则继续后续流程。
        measuredRpm=static_cast<uint16_t>(fmaxf(measuredRpm,abs(state.rpm)));  // 将 static_cast<uint16_t>(fmaxf(measuredRpm,abs(state.rpm)… 写入或初始化 measuredRpm（measured转速）。
        if(abs(state.rpm)>3)allIdle=false;  // 当 abs(state.rpm)>3 成立时进入此分支，否则继续后续流程。
        const double arrivalResidual=rotating?residual:static_cast<double>(llround(residual));  // 将 rotating?residual:static_cast<double>(llround(residual… 写入或初始化 arrivalResidual。
        if(arrivalResidual>tolerance||abs(state.rpm)>3)arrivedNow=false;  // 当 arrivalResidual>tolerance||abs(state.rpm)>3 成立时进入此分支，否则继续后续流程。
        if(goal>0)progress+=fmax(0,fmin(1,current/goal))/movingWheels;  // 当 goal>0 成立时进入此分支，否则继续后续流程。
    }  // 结束此循环体。
    const uint32_t readMs=io->now()-checkStarted;  // 将 io->now()-checkStarted 写入或初始化 readMs（read毫秒）。
    if(sent) {  // 当 sent 成立时进入此分支，否则继续后续流程。
        representativePulses=fabs(static_cast<double>(measured[representative-1].position-origin[representative-1]))*PULSES_PER_REV/65536;  // 将 fabs(static_cast<double>(measured[representative-1].po… 写入或初始化 representativePulses（representative脉冲数）。
        representativeSampleAt=io->now();previousRepresentativeRpm=measured[representative-1].rpm;  // 将 io->now() 写入或初始化 representativeSampleAt（representative采样At）。然后将 measured[representative-1].rpm 写入或初始化 previousRepresentativeRpm（上一Representative转速）。
        if((representativeMoved||arrivedNow)&&(arrivedNow||!comparisonReported||io->now()-lastComparison>=200)) {  // 当 (representativeMoved||arrivedNow)&&(arrivedNow||!compa… 成立时进入此分支，否则继续后续流程。
            const uint32_t waitMs=representativeStopped?io->now()-representativeStopAt:readMs;  // 将 representativeStopped?io->now()-representativeStopAt:r… 写入或初始化 waitMs（wait毫秒）。
            io->arrivalComparison(representative,readMs,waitMs,arrivedNow,rotating,residuals,measured);  // 调用 io->arrivalComparison，传入 representative,readMs,waitMs,arrivedNow,rotating,resid…。
            comparisonReported=true;lastComparison=io->now();  // 将 true 写入或初始化 comparisonReported。然后将 io->now() 写入或初始化 lastComparison（最后Comparison）。
        }  // 结束此条件分支。
    }  // 结束此条件分支。
    failure=sample(m);if(failure){fail(failure);return;}  // 将 sample(m);if(failure){fail(failure);return;} 写入或初始化 failure。
    if(sent&&peakMeasuredRpm>measuredRpm+3)braking=true;  // 当 sent&&peakMeasuredRpm>measuredRpm+3 成立时进入此分支，否则继续后续流程。
    peakMeasuredRpm=static_cast<uint16_t>(fmaxf(peakMeasuredRpm,measuredRpm));  // 将 static_cast<uint16_t>(fmaxf(peakMeasuredRpm,measuredRp… 写入或初始化 peakMeasuredRpm（peakMeasured转速）。
    failure=updateProgress(static_cast<float>(progress),m,allIdle);if(failure){fail(failure);return;}  // 将 updateProgress(static_cast<float>(progress),m,allIdle)… 写入或初始化 failure。
    if(arrivedNow) {  // 当 arrivedNow 成立时进入此分支，否则继续后续流程。
        for(int i=0;i<4;++i) {  // 按 int i=0;i<4;++i 组织循环的初值、继续条件和步进。
            const int64_t rawDelta=measured[i].position-origin[i];  // 将 measured[i].position-origin[i] 写入或初始化 rawDelta。
            // 微小位置动作的计数波动不能用于确定后续直行的编码器方向。
            if(!encoderSign[i]&&travelPulses[i]&&llabs(rawDelta)>ENCODER_DIRECTION_MIN_COUNTS)  // 当 !encoderSign[i]&&travelPulses[i]&&llabs(rawDelta)>ENCO… 成立时进入此分支，否则继续后续流程。
                encoderSign[i]=(rawDelta>=0?1:-1)*(directions[i]?1:-1);  // 将 (rawDelta>=0?1:-1)*(directions[i]?1:-1) 写入或初始化 encoderSign[i]。
            stopped[i]=measured[i];  // 将 measured[i] 写入或初始化 stopped[i]。
        }  // 结束此循环体。
        io->timing(rotating?"TURN_ARRIVED":"STRAIGHT_ARRIVED");valid=true;arrived();  // 调用 io->timing，传入 rotating?"TURN_ARRIVED":"STRAIGHT_ARRIVED");valid=true…。
        // 四轮已停且位置到位，再等停轮后的三帧航向稳定，才用角度校角或接续直行。
        settleNext=stage;headingStable.begin(io->now());stage=SETTLE;return;  // 将 stage 写入或初始化 settleNext（settle下一）。然后调用 headingStable.begin，传入 io->now()。然后将 SETTLE 写入或初始化 stage（阶段）。然后结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    if(sent&&allIdle&&io->now()-lastArrivalReport>=200) {  // 当 sent&&allIdle&&io->now()-lastArrivalReport>=200 成立时进入此分支，否则继续后续流程。
        io->arrivalDiagnostic(static_cast<float>(maxResidual/unitsPerDistance),measured);lastArrivalReport=io->now();  // 调用 io->arrivalDiagnostic，传入 static_cast<float>(maxResidual/unitsPerDistance),measu…。
    }  // 结束此条件分支。
    // 沿用既有静止残差故障条件；正常到位不增加稳定等待。
    const bool enabled=sent&&allIdle&&maxResidual<=hardLimit;  // 将 sent&&allIdle&&maxResidual<=hardLimit 写入或初始化 enabled（已启用）。
    bool calm=enabled;  // 将 enabled 写入或初始化 calm。
    for(int i=0;i<4;++i)if(abs(measured[i].rpm)>3||(quietSeen&&llabs(measured[i].position-quietAnchor[i])>128))calm=false;  // 按 int i=0;i<4;++i)if(abs(measured[i].rpm)>3||(quietSeen&… 组织循环的初值、继续条件和步进。
    if(!enabled){quietSeen=false;quietCount=0;}  // 当 !enabled 成立时进入此分支，否则继续后续流程。
    else {  // 前一个判断不成立时执行此分支。
        if(!quietSeen||!calm){quietSeen=true;residualSince=io->now();quietCount=0;for(int i=0;i<4;++i)quietAnchor[i]=measured[i].position;}  // 当 !quietSeen||!calm 成立时进入此分支，否则继续后续流程。
        if(calm&&quietCount<255)++quietCount;  // 当 calm&&quietCount<255 成立时进入此分支，否则继续后续流程。
        if(calm&&quietCount>=2&&io->now()-residualSince>=500) {  // 当 calm&&quietCount>=2&&io->now()-residualSince>=500 成立时进入此分支，否则继续后续流程。
            for(uint8_t id=1;id<=4;++id)io->motorDiagnostic(id,"MOTOR_RESIDUAL_TIMEOUT",  // 按 uint8_t id=1;id<=4;++id)io->motorDiagnostic(id,"MOTOR_… 组织循环的初值、继续条件和步进。
                static_cast<double>(goals[id-1])*(encoderSign[id-1]?encoderSign[id-1]:1)*65536/PULSES_PER_REV,measured[id-1]);  // 声明接口 double>，参数由此处的签名确定。
            const Vec body{  // 声明 body（车体），类型和可用范围由所在声明决定。
                (-remainingPulses[0]-remainingPulses[1]+remainingPulses[2]+remainingPulses[3])/(4*route->config.lateralPpm),  // 为当前列表或跨行表达式提供这一项 (-remainingPulses[0]-remainingPulses[1]+remainingPulse…。
                (remainingPulses[0]-remainingPulses[1]+remainingPulses[2]-remainingPulses[3])/(4*route->config.forwardPpm)  // 为当前列表或跨行表达式提供这一项 (remainingPulses[0]-remainingPulses[1]+remainingPulses…。
            };  // 结束此初始化列表或局部作用域。
            const float angle=(remainingPulses[0]+remainingPulses[1]+remainingPulses[2]+remainingPulses[3])/(4*route->config.rotationPpd);  // 将 (remainingPulses[0]+remainingPulses[1]+remainingPulses… 写入或初始化 angle（角度）。
            bool clear=settings->recoverSmallResidual&&!residualRetried&&(!rotating||!headingRecoveryUsed)  // 将 settings->recoverSmallResidual&&!residualRetried&&(!ro… 写入或初始化 clear。
                &&norm(body)<=10&&fabsf(angle)<=3;  // 继续上一行的组合条件，本行提供 &&norm(body)<=10&&fabsf(angle)<=3;。
            const Vec world=bodyToWorld(body,output->heading+angle*.5f);  // 将 bodyToWorld(body,output->heading+angle*.5f) 写入或初始化 world（地图）。
            for(int i=0;clear&&i<=10;++i) {  // 按 int i=0;clear&&i<=10;++i 组织循环的初值、继续条件和步进。
                const float ratio=i/10.0f;  // 将 i/10.0f 写入或初始化 ratio。
                const Vec pose=output->estimated+world*ratio;  // 将 output->estimated+world*ratio 写入或初始化 pose。
                clear=!field::check(pose.x,pose.y,output->heading+angle*ratio,route->config.margin);  // 将 !field::check(pose.x,pose.y,output->heading+angle*rati… 写入或初始化 clear。
            }  // 结束此循环体。
            if(!clear){fail("MOTOR_RESIDUAL_TIMEOUT");return;}  // 当 !clear 成立时进入此分支，否则继续后续流程。
            residualRetried=true;  // 将 true 写入或初始化 residualRetried。
            headingRecoveryUsed|=rotating;  // 用 rotating 对 headingRecoveryUsed（航向RecoveryUsed） 应用 |= 更新。
            sent=quietSeen=false;  // 将 quietSeen=false 写入或初始化 sent（已发送）。
            quietCount=0;  // 将 0 写入或初始化 quietCount（quiet计数）。
            io->timing("POSITION_RESIDUAL_RETRY");  // 调用 io->timing，传入 "POSITION_RESIDUAL_RETRY"。
        }  // 结束此条件分支。
    }  // 结束此条件分支。
    if(!sent) {  // 当 !sent 成立时进入此分支，否则继续后续流程。
        const float speed=residualRetried?fminf(maximum*.25f,rotating?30.0f:200.0f):maximum;  // 将 residualRetried?fminf(maximum*.25f,rotating?30.0f:200.… 写入或初始化 speed（速度）。
        const uint16_t rpm=rpmFor(speed,ppm);  // 将 rpmFor(speed,ppm) 写入或初始化 rpm（转速）。
        for(uint8_t id=1;id<=4;++id) {  // 按 uint8_t id=1;id<=4;++id 组织循环的初值、继续条件和步进。
            const float wheelPpm=scanExit&&!rotating?travelPulses[id-1]/fabsf(delta):ppm;  // 将 scanExit&&!rotating?travelPulses[id-1]/fabsf(delta):ppm 写入或初始化 wheelPpm（轮每毫米脉冲数）。
            failure=io->target(id,goals[id-1]>=0,scanExit&&!rotating?rpmFor(speed,wheelPpm):rpm,  // 将 io->target(id,goals[id-1]>=0,scanExit&&!rotating?rpmFo… 写入或初始化 failure。
                driverAccelerationFor(acceleration,wheelPpm),static_cast<uint32_t>(llabs(goals[id-1])));  // 调用 driverAccelerationFor（driver加速度For），传入 acceleration,wheelPpm),static_cast<uint32_t>(llabs(goa…。
            if(failure){fail(failure);return;}  // 当 failure 成立时进入此分支，否则继续后续流程。
        }  // 结束此循环体。
        if(io->cancelled()){fail("STOPPED");return;}  // 当 io->cancelled() 成立时进入此分支，否则继续后续流程。
        failure=io->sync();if(failure){fail(failure);return;}  // 将 io->sync();if(failure){fail(failure);return;} 写入或初始化 failure。
        sent=true;representativeSampleAt=io->now();io->timing(rotating?"TURN_SENT":"STRAIGHT_SENT");  // 将 true 写入或初始化 sent（已发送）。然后将 io->now() 写入或初始化 representativeSampleAt（representative采样At）。然后调用 io->timing，传入 rotating?"TURN_SENT":"STRAIGHT_SENT"。
    }  // 结束此条件分支。
    if(io->now()-lastReport>=200){io->report(rotating?"POSITION_TURN":"POSITION_STRAIGHT",*output,targetHeading);lastReport=io->now();}  // 当 io->now()-lastReport>=200 成立时进入此分支，否则继续后续流程。
    due=io->now()+(allIdle||maxResidual<=unitsPerDistance*(rotating?2.0:10.0)?1:20);  // 将 io->now()+(allIdle||maxResidual<=unitsPerDistance*(rot… 写入或初始化 due。
}  // 结束此函数或控制块。
/** @brief 速度模式直行同时下发航向修正与平移分量。 */
void Runner::straightVelocityTick() {  // 定义函数或成员 的入口 Runner::straightVelocityTick（Runnerstraight速度轮询），参数由此处的签名确定。
    const uint32_t cycleAt=io->now();  // 将 io->now() 写入或初始化 cycleAt。
    const float dt=(cycleAt-previous)*.001f;previous=cycleAt;  // 将 (cycleAt-previous)*.001f 写入或初始化 dt（时间差）。然后将 cycleAt 写入或初始化 previous（上一）。
    if(cycleAt-started>deadline){fail("MOTOR_ARRIVAL_TIMEOUT");return;}  // 当 cycleAt-started>deadline 成立时进入此分支，否则继续后续流程。
    const char *failure=nullptr;  // 将 nullptr 写入或初始化 failure。
    bool idle=true;  // 将 true 写入或初始化 idle。
    bool encoderDirectionsKnown=true;  // 将 true 写入或初始化 encoderDirectionsKnown（编码器DirectionsKnown）。
    int64_t measured[4];  // 声明 measured，类型和可用范围由所在声明决定。
    for(uint8_t id=1;id<=4;++id) {  // 按 uint8_t id=1;id<=4;++id 组织循环的初值、继续条件和步进。
        const int i=id-1;  // 将 id-1 写入或初始化 i。
        if(straightStopping) {  // 当 straightStopping 成立时进入此分支，否则继续后续流程。
            failure=io->read(id,stopped[i]);measured[i]=stopped[i].position;  // 将 io->read(id,stopped[i]) 写入或初始化 failure。然后将 stopped[i].position 写入或初始化 measured[i]。
            if(abs(stopped[i].rpm)>3)idle=false;  // 当 abs(stopped[i].rpm)>3 成立时进入此分支，否则继续后续流程。
        } else failure=io->readPosition(id,measured[i]);  // 前一个判断不成立时执行此分支。
        if(failure){fail(failure);return;}  // 当 failure 成立时进入此分支，否则继续后续流程。
        // 未发出轮速时刷新静止基准。起步后沿用基准，按累计位移辨识轮向。
        if(!encoderSign[i]&&!applied.rpm[i])positions[i]=measured[i];  // 当 !encoderSign[i]&&!applied.rpm[i] 成立时进入此分支，否则继续后续流程。
        const int64_t change=measured[i]-positions[i];  // 将 measured[i]-positions[i] 写入或初始化 change。
        if(!encoderSign[i]&&llabs(change)>ENCODER_DIRECTION_MIN_COUNTS&&applied.rpm[i])  // 当 !encoderSign[i]&&llabs(change)>ENCODER_DIRECTION_MIN_C… 成立时进入此分支，否则继续后续流程。
            encoderSign[i]=(change>=0?1:-1)*(applied.rpm[i]>=0?1:-1);  // 将 (change>=0?1:-1)*(applied.rpm[i]>=0?1:-1) 写入或初始化 encoderSign[i]。
        encoderDirectionsKnown=encoderDirectionsKnown&&encoderSign[i]!=0;  // 将 encoderDirectionsKnown&&encoderSign[i]!=0 写入或初始化 encoderDirectionsKnown（编码器DirectionsKnown）。
    }  // 结束此循环体。
    // 四轮方向齐备后一起累计起步位移，避免只读到部分轮子时产生虚假横移。
    float wheelTravel[4]={};  // 将 {} 写入或初始化 wheelTravel[4]。
    if(encoderDirectionsKnown) {  // 当 encoderDirectionsKnown 成立时进入此分支，否则继续后续流程。
        for(int i=0;i<4;++i) {  // 按 int i=0;i<4;++i 组织循环的初值、继续条件和步进。
            wheelTravel[i]=static_cast<float>((measured[i]-positions[i])*encoderSign[i])*PULSES_PER_REV/65536;  // 将 static_cast<float>((measured[i]-positions[i])*encoderS… 写入或初始化 wheelTravel[i]。
            positions[i]=measured[i];  // 将 measured[i] 写入或初始化 positions[i]。
        }  // 结束此循环体。
    }  // 结束此条件分支。
    // 解算车体横向/前后位移，四轮同向旋转分量在组合中抵消。
    const float lateral=(-wheelTravel[0]-wheelTravel[1]+wheelTravel[2]+wheelTravel[3])/(4*route->config.lateralPpm);  // 将 (-wheelTravel[0]-wheelTravel[1]+wheelTravel[2]+wheelTr… 写入或初始化 lateral（横向）。
    const float forward=(wheelTravel[0]-wheelTravel[1]+wheelTravel[2]-wheelTravel[3])/(4*ppm);  // 将 (wheelTravel[0]-wheelTravel[1]+wheelTravel[2]-wheelTra… 写入或初始化 forward（前向）。
    const float oldHeading=output->heading;  // 将 output->heading 写入或初始化 oldHeading（old航向）。
    H30Attitude m;failure=sample(m);if(failure){fail(failure);return;}  // 将 sample(m);if(failure){fail(failure);return;} 写入或初始化 failure。
    output->estimated=output->estimated+bodyToWorld(Vec{lateral,forward},oldHeading+turn(oldHeading,output->heading)*.5f);  // 将 output->estimated+bodyToWorld(Vec{lateral,forward},old… 写入或初始化 output->estimated。
    const Vec lineStart=route->segments[index].start;  // 将 route->segments[index].start 写入或初始化 lineStart（line起点）。
    const Vec tangent=(end-lineStart)*(1/norm(end-lineStart));  // 将 (end-lineStart)*(1/norm(end-lineStart)) 写入或初始化 tangent（切向）。
    const Vec normal{-tangent.y,tangent.x};  // 声明 normal（法向），类型和可用范围由所在声明决定。
    straightTravel=dot(output->estimated-start,tangent);  // 将 dot(output->estimated-start,tangent) 写入或初始化 straightTravel。
    const float progress=clamp(straightTravel/fabsf(distance),0,1);  // 将 clamp(straightTravel/fabsf(distance),0,1) 写入或初始化 progress。
    failure=updateProgress(progress,m,straightStopping&&idle);if(failure){fail(failure);return;}  // 将 updateProgress(progress,m,straightStopping&&idle);if(f… 写入或初始化 failure。
    if(straightStopping) {  // 当 straightStopping 成立时进入此分支，否则继续后续流程。
        if(idle) {  // 当 idle 成立时进入此分支，否则继续后续流程。
            io->timing("STRAIGHT_ARRIVED");valid=true;arrived();  // 调用 io->timing，传入 "STRAIGHT_ARRIVED");valid=true;arrived(。
            settleNext=stage;headingStable.begin(io->now());stage=SETTLE;  // 将 stage 写入或初始化 settleNext（settle下一）。然后调用 headingStable.begin，传入 io->now()。然后将 SETTLE 写入或初始化 stage（阶段）。
        } else due=io->now()+1;  // 前一个判断不成立时执行此分支。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    const float remaining=fabsf(distance)-straightTravel;  // 将 fabsf(distance)-straightTravel 写入或初始化 remaining（剩余）。
    if(remaining<=1) {  // 当 remaining<=1 成立时进入此分支，否则继续后续流程。
        failure=io->velocity(WheelCommand{});if(failure){fail(failure);return;}  // 将 io->velocity(WheelCommand{});if(failure){fail(failure)… 写入或初始化 failure。
        applied={};straightStopping=true;due=io->now();return;  // 将 {};straightStopping=true;due=io->now();return 写入或初始化 applied。
    }  // 结束此条件分支。
    // 制动距离包含两个40ms周期，覆盖四轮依次采样、装载及同步的延迟；不增加停车等待。
    float cap=fminf(maximum,acceleration*(sqrtf(.08f*.08f+2*remaining/acceleration)-.08f));  // 将 fminf(maximum,acceleration*(sqrtf(.08f*.08f+2*remainin… 写入或初始化 cap。
    if(isfinite(approachStart)) {  // 当 isfinite(approachStart) 成立时进入此分支，否则继续后续流程。
        const float toApproach=approachStart-dot(output->estimated-lineStart,tangent);  // 将 approachStart-dot(output->estimated-lineStart,tangent) 写入或初始化 toApproach。
        const float halfSpeed=maximum*.5f;  // 将 maximum*.5f 写入或初始化 halfSpeed（half速度）。
        // 给同一直线的限速点预留制动距离，无需在节点停车换速。
        const float brakingDistance=fmaxf(0,toApproach-straightSpeed*.08f);  // 将 fmaxf(0,toApproach-straightSpeed*.08f) 写入或初始化 brakingDistance（braking距离）。
        cap=fminf(cap,sqrtf(halfSpeed*halfSpeed+2*acceleration*brakingDistance));  // 将 fminf(cap,sqrtf(halfSpeed*halfSpeed+2*acceleration*bra… 写入或初始化 cap。
    }  // 结束此条件分支。
    braking=cap<straightSpeed;  // 将 cap<straightSpeed 写入或初始化 braking。
    straightSpeed+=clamp(cap-straightSpeed,-acceleration*dt,acceleration*dt);  // 把 clamp(cap-straightSpeed,-acceleration*dt,acceleration*… 累加到 straightSpeed（straight速度）。
    float omega=0;  // 将 0 写入或初始化 omega。
    if(encoderDirectionsKnown&&(m.fields&7)==7)omega=headingHold.update(output->heading,turn(output->heading,targetHeading),  // 当 encoderDirectionsKnown&&(m.fields&7)==7 成立时进入此分支，否则继续后续流程。
        m.receivedAtMs,STRAIGHT_HEADING_KP,route->config.maxOmega);  // 续写当前声明或运算，提供 m.receivedAtMs,STRAIGHT_HEADING_KP,route->config.maxOm…。
    else headingHold.reset();  // 前一个判断不成立时执行此分支。
    Config wheelConfig=route->config;  // 将 route->config 写入或初始化 wheelConfig（轮配置）。
    wheelConfig.maxRpm=5000; // 沿用节点位置模式的驱动轮速上限，不套用连续试跑的maxRpm。
    const float lateralError=dot(lineStart-output->estimated,normal);  // 将 dot(lineStart-output->estimated,normal) 写入或初始化 lateralError（横向错误）。
    const Vec velocity=tangent*straightSpeed+normal*(STRAIGHT_LATERAL_KP*lateralError);  // 将 tangent*straightSpeed+normal*(STRAIGHT_LATERAL_KP*late… 写入或初始化 velocity（速度）。
    // 首批位移可能滞后于命令。辨识期间用固定轮向起步，避免纠偏换向导致编码器符号判反。
    const Vec bodyVelocity=encoderDirectionsKnown?worldToBody(velocity,output->heading):Vec{0,copysignf(straightSpeed,distance)};  // 将 encoderDirectionsKnown?worldToBody(velocity,output->he… 写入或初始化 bodyVelocity（车体速度）。
    const WheelCommand command=wheels(bodyVelocity,omega,wheelConfig);  // 将 wheels(bodyVelocity,omega,wheelConfig) 写入或初始化 command。
    failure=io->velocity(command);if(failure){fail(failure);return;}  // 将 io->velocity(command);if(failure){fail(failure);return… 写入或初始化 failure。
    applied=command;  // 将 command 写入或初始化 applied。
    if(!sent){sent=true;io->timing("STRAIGHT_HOLD_SENT");}  // 当 !sent 成立时进入此分支，否则继续后续流程。
    if(io->now()-lastReport>=200) {  // 当 io->now()-lastReport>=200 成立时进入此分支，否则继续后续流程。
        io->report(route->segments[index].middle?"MIDDLE_HEADING_HOLD":"OUTER_HEADING_HOLD",*output,targetHeading);  // 上报当前控制状态，传入 route->segments[index].middle?"MIDDLE_HEADING_HOLD":"O…。
        io->headingControl(targetHeading,output->heading,command.omega,straightSpeed);  // 调用 io->headingControl，传入 targetHeading,output->heading,command.omega,straightSp…。
        lastReport=io->now();  // 将 io->now() 写入或初始化 lastReport（最后Report）。
    }  // 结束此条件分支。
    due=cycleAt+40;  // 将 cycleAt+40 写入或初始化 due。
}  // 结束此函数或控制块。
/**
 * @brief 根据四轮反馈更新本次动作行程和到位残差；成功返回 nullptr。
 */
const char *Runner::displacement(const Motor (&states)[4]) {  // 定义函数或成员 的入口 Runner::displacement，参数由此处的签名确定。
    float pulses[4];  // 声明 pulses（脉冲数），类型和可用范围由所在声明决定。
    for(int i=0;i<4;++i) {  // 按 int i=0;i<4;++i 组织循环的初值、继续条件和步进。
        const int64_t change=states[i].position-positions[i];  // 将 states[i].position-positions[i] 写入或初始化 change。
        if(!encoderSign[i]&&llabs(change)>ENCODER_DIRECTION_MIN_COUNTS&&applied.rpm[i])encoderSign[i]=(change>=0?1:-1)*(applied.rpm[i]>=0?1:-1);  // 当 !encoderSign[i]&&llabs(change)>ENCODER_DIRECTION_MIN_C… 成立时进入此分支，否则继续后续流程。
        pulses[i]=encoderSign[i]?static_cast<float>(change*encoderSign[i])*PULSES_PER_REV/65536:0;  // 将 encoderSign[i]?static_cast<float>(change*encoderSign[i… 写入或初始化 pulses[i]。
        positions[i]=states[i].position;  // 将 states[i].position 写入或初始化 positions[i]。
    }  // 结束此循环体。
    Vec body{(-pulses[0]-pulses[1]+pulses[2]+pulses[3])/(4*route->config.lateralPpm),  // 声明 body（车体），类型和可用范围由所在声明决定。
              (pulses[0]-pulses[1]+pulses[2]-pulses[3])/(4*route->config.forwardPpm)};  // 为当前列表或跨行表达式提供这一项 (pulses[0]-pulses[1]+pulses[2]-pulses[3])/(4*route->co…。
    const float oldHeading=output->heading;H30Attitude m;const char *failure=sample(m);if(failure)return failure;  // 将 output->heading 写入或初始化 oldHeading（old航向）。然后声明 m，类型和可用范围由所在声明决定。然后将 sample(m) 写入或初始化 failure。然后若 failure，立即返回 failure。
    if((m.fields&7)!=7)output->heading=wrap(oldHeading+(pulses[0]+pulses[1]+pulses[2]+pulses[3])/(4*route->config.rotationPpd));  // 当 (m.fields&7)!=7 成立时进入此分支，否则继续后续流程。
    const Vec world=bodyToWorld(body,oldHeading+turn(oldHeading,output->heading)*.5f);  // 将 bodyToWorld(body,oldHeading+turn(oldHeading,output->he… 写入或初始化 world（地图）。
    output->estimated=output->estimated+world;tracker.estimated=output->estimated;  // 将 output->estimated+world 写入或初始化 output->estimated。然后将 output->estimated 写入或初始化 tracker.estimated。
    const float window=norm(world)*2+20,lo=fmaxf(route->segments[curveBegin].offset,tracker.s-window),hi=fminf(curveStop,tracker.s+window);  // 将 norm(world)*2+20,lo=fmaxf(route->segments[curveBegin].… 写入或初始化 window。
    auto distanceAt=[&](float s){size_t i=route->movingPhase(s);return norm(route->phaseAt(i,s-route->segments[i].offset).p-output->estimated);};  // 将 [&](float s){size_t i=route->movingPhase(s);return nor… 写入或初始化 distanceAt（距离At）。
    float best=lo,bestDistance=distanceAt(lo);  // 将 lo,bestDistance=distanceAt(lo) 写入或初始化 best。
    for(float s=lo;s<=hi;s+=5){float d=distanceAt(s);if(d<bestDistance){best=s;bestDistance=d;}}  // 按 float s=lo;s<=hi;s+=5 组织循环的初值、继续条件和步进。
    if(distanceAt(hi)<bestDistance)best=hi;  // 当 distanceAt(hi)<bestDistance 成立时进入此分支，否则继续后续流程。
    float a=fmaxf(lo,best-5),b=fminf(hi,best+5);  // 将 fmaxf(lo,best-5),b=fminf(hi,best+5) 写入或初始化 a。
    for(int i=0;i<12;++i){float u=a+(b-a)/3,v=b-(b-a)/3;if(distanceAt(u)<distanceAt(v))b=v;else a=u;}  // 按 int i=0;i<12;++i 组织循环的初值、继续条件和步进。
    tracker.s=fmaxf(tracker.s,(a+b)*.5f);output->s=tracker.s;return nullptr;  // 将 fmaxf(tracker.s,(a+b)*.5f) 写入或初始化 tracker.s。然后将 tracker.s 写入或初始化 output->s。然后结束当前函数并返回 nullptr。
}  // 结束此函数或控制块。
/**
 * @brief 推进当前节点控制阶段一次，包括起步、段选择、位置目标下发及到位反馈。
 */
void Runner::tick() {  // 定义函数或成员 的入口 Runner::tick（Runner轮询），参数由此处的签名确定。
    if(!active())return;  // 当 !active() 成立时进入此分支，否则继续后续流程。
    if(io->cancelled()){fail("STOPPED");return;}  // 当 io->cancelled() 成立时进入此分支，否则继续后续流程。
    if(nextDelay())return;  // 距离下次可推进的时刻仍有延时，本轮提前返回。
    const char *failure=nullptr;H30Attitude m;  // 将 nullptr 写入或初始化 failure。然后声明 m，类型和可用范围由所在声明决定。
    switch(stage) {  // 按 stage 的值选择对应分支。
    case START: io->halt();quietStarted=io->now();stage=QUIET;break;  // 当前值匹配 START 时从这里执行。
    case QUIET:  // 当前值匹配 QUIET 时从这里执行。
    case OUTER_QUIET: {  // 当前值匹配 OUTER_QUIET 时从这里执行。
        if(io->now()-quietStarted>3000){fail("MOTOR_STOP_TIMEOUT");break;}  // 当 io->now()-quietStarted>3000 成立时进入此分支，否则继续后续流程。
        failure=sample(m);if(failure)break;  // 将 sample(m) 写入或初始化 failure。然后若 failure，从当前循环或分支退出。
        bool idle=true;  // 将 true 写入或初始化 idle。
        for(uint8_t id=1;id<=4;++id){failure=io->read(id,stopped[id-1]);if(failure)break;  // 按 uint8_t id=1;id<=4;++id 组织循环的初值、继续条件和步进。
            failure=health(*io,id,stopped[id-1]);if(failure)break;if(abs(stopped[id-1].rpm)>3)idle=false;}  // 将 health(*io,id,stopped[id-1]);if(failure)break;if(abs(s… 写入或初始化 failure。
        if(failure)break;  // 若 failure，从当前循环或分支退出。
        failure=sample(m);if(failure)break;  // 将 sample(m) 写入或初始化 failure。然后若 failure，从当前循环或分支退出。
        if(idle){valid=true;  // 当 idle 成立时进入此分支，否则继续后续流程。
            if(stage==OUTER_QUIET){failure=displacement(stopped);prepared=true;io->timing("OUTER_DRIFT_ARRIVED");index=after;}  // 当 stage==OUTER_QUIET 成立时进入此分支，否则继续后续流程。
            settleNext=SELECT;headingStable.begin(io->now());stage=SETTLE;  // 将 SELECT 写入或初始化 settleNext（settle下一）。然后调用 headingStable.begin，传入 io->now()。然后将 SETTLE 写入或初始化 stage（阶段）。
        } else due=io->now()+5;  // 前一个判断不成立时执行此分支。
        break;  // 退出当前循环或 switch 分支。
    }  // 结束此条件分支。
    case SETTLE:  // 当前值匹配 SETTLE 时从这里执行。
        failure=sample(m);if(failure)break;  // 将 sample(m) 写入或初始化 failure。然后若 failure，从当前循环或分支退出。
        // 沿用现有IMU不可用时的编码器控制；不拿估算角度凑三帧实测。
        if((m.fields&7)!=7)stage=settleNext;  // 当 (m.fields&7)!=7 成立时进入此分支，否则继续后续流程。
        else if(headingStable.update(m,output->heading)){io->headingStable(output->heading);stage=settleNext;}  // 仅在先前条件均不成立时，当 headingStable.update(m,output->heading) 成立时进入此分支，否则继续后续流程。
        else due=io->now()+1;  // 前一个判断不成立时执行此分支。
        break;  // 退出当前循环或 switch 分支。
    // 选择接下来要运行的路线阶段。
    // 若在此处改变分支顺序，应继续检查 ALIGN、PREPARE 和对应的移动阶段怎样接收选择结果。
    case SELECT: {  // 当前值匹配 SELECT 时从这里执行。
        finalTranslation=false;terminalTurn=false;  // 将 false 写入或初始化 finalTranslation。然后将 false 写入或初始化 terminalTurn。
        if(index>=route->segmentCount){action=FINAL;targetHeading=route->config.endHeading;attempts=0;headingRecoveryUsed=false;stage=ALIGN;break;}  // 当 index>=route->segmentCount 成立时进入此分支，否则继续后续流程。
        const Segment &phase=route->segments[index];after=index+1;  // 将 route->segments[index] 写入或初始化 phase（阶段）。然后将 index+1 写入或初始化 after。
        if(phase.kind==ROTATE) {  // 当 phase.kind==ROTATE 成立时进入此分支，否则继续后续流程。
            if(norm(offset)>0&&!settings->alignStartToRoad){++index;break;}  // 当 norm(offset)>0&&!settings->alignStartToRoad 成立时进入此分支，否则继续后续流程。
            action=ROTATION;targetHeading=phase.h1;attempts=0;headingRecoveryUsed=false;stage=ALIGN;break;  // 将 ROTATION 写入或初始化 action。然后将 phase.h1 写入或初始化 targetHeading（目标航向）。然后将 0 写入或初始化 attempts。然后将 false 写入或初始化 headingRecoveryUsed（航向RecoveryUsed）。然后将 ALIGN 写入或初始化 stage（阶段）。然后退出当前循环或 switch 分支。
        }  // 结束此条件分支。
        size_t block=index;bool curved=false;  // 将 index 写入或初始化 block。然后将 false 写入或初始化 curved。
        while(block<route->segmentCount&&route->segments[block].kind!=ROTATE){curved|=route->segments[block].kind==CUBIC;++block;}  // 当 block<route->segmentCount&&route->segments[block].kind… 成立时继续循环。
        targetHeading=phase.h0;attempts=0;headingRecoveryUsed=false;  // 将 phase.h0 写入或初始化 targetHeading（目标航向）。然后将 0 写入或初始化 attempts。然后将 false 写入或初始化 headingRecoveryUsed（航向RecoveryUsed）。
        if(curved){action=CURVE;curveBegin=index;after=block;stage=ALIGN;break;}  // 当 curved 成立时进入此分支，否则继续后续流程。
        action=STRAIGHT;end=phase.end;float length=phase.length;  // 将 STRAIGHT 写入或初始化 action。然后将 phase.end 写入或初始化 end（终点）。然后将 phase.length 写入或初始化 length（长度）。
        approachStart=INFINITY;  // 将 INFINITY 写入或初始化 approachStart（approach起点）。
        const bool velocityStraight=(phase.middle||settings->outerStraightVelocity)  // 将 (phase.middle||settings->outerStraightVelocity) 写入或初始化 velocityStraight（速度Straight）。
            &&(norm(offset)==0||settings->alignStartToRoad);  // 继续上一行的组合条件，本行提供 &&(norm(offset)==0||settings->alignStartToRoad);。
        while(after<route->segmentCount) {  // 当 after<route->segmentCount 成立时继续循环。
            const auto &next=route->segments[after];  // 将 route->segments[after] 写入或初始化 next（下一）。
            if(next.kind!=LINE||next.middle!=phase.middle||fabsf(turn(phase.h0,next.h0))>.01f||dot(end-phase.start,next.end-next.start)<=0)break;  // 若 next.kind!=LINE||next.middle!=phase.middle||fabsf(turn…，从当前循环或分支退出。
            if(next.slowApproach&&!phase.slowApproach) {  // 当 next.slowApproach&&!phase.slowApproach 成立时进入此分支，否则继续后续流程。
                if(!velocityStraight)break; // 位置模式整段使用同一个速度。
                if(!isfinite(approachStart))approachStart=length;  // 当 !isfinite(approachStart) 成立时进入此分支，否则继续后续流程。
            }  // 结束此条件分支。
            end=next.end;length+=next.length;++after;  // 将 next.end 写入或初始化 end（终点）。然后把 next.length 累加到 length（长度）。然后续写当前声明或运算，提供 ++after;。
        }  // 结束此循环体。
        const Vec direction=(end-phase.start)*(1/length);  // 将 (end-phase.start)*(1/length) 写入或初始化 direction（方向）。
        const float sign=dot(direction,Vec(cosf(phase.h0*PI_F/180),sinf(phase.h0*PI_F/180)))>=0?1:-1;  // 将 dot(direction,Vec(cosf(phase.h0*PI_F/180),sinf(phase.h… 写入或初始化 sign。
        scanExit=norm(offset)>0;  // 将 norm(offset)>0 写入或初始化 scanExit（扫码Exit）。
        if(scanExit&&settings->alignStartToRoad) {  // 当 scanExit&&settings->alignStartToRoad 成立时进入此分支，否则继续后续流程。
            const Vec normal{-direction.y,direction.x};  // 声明 normal（法向），类型和可用范围由所在声明决定。
            const float lateral=dot(output->estimated-phase.start,normal);  // 将 dot(output->estimated-phase.start,normal) 写入或初始化 lateral（横向）。
            if(!startLateralDone&&lroundf(fabsf(lateral)*route->config.lateralPpm)>0) {  // 当 !startLateralDone&&lroundf(fabsf(lateral)*route->confi… 成立时进入此分支，否则继续后续流程。
                action=START_LATERAL;end=output->estimated-normal*lateral;  // 将 START_LATERAL 写入或初始化 action。然后将 output->estimated-normal*lateral 写入或初始化 end（终点）。
                distance=fabsf(lateral);stage=ALIGN;break;  // 将 fabsf(lateral) 写入或初始化 distance（距离）。然后将 ALIGN 写入或初始化 stage（阶段）。然后退出当前循环或 switch 分支。
            }  // 结束此条件分支。
            scanExit=false;  // 将 false 写入或初始化 scanExit（扫码Exit）。
            distance=sign*dot(end-output->estimated,direction);  // 将 sign*dot(end-output->estimated,direction) 写入或初始化 distance（距离）。
        } else distance=sign*(scanExit?norm(end-output->estimated):length-startYOffset*direction.y);  // 前一个判断不成立时执行此分支。
        startYOffset=0;  // 将 0 写入或初始化 startYOffset（起点纵向偏移）。
        finalTranslation=true;  // 将 true 写入或初始化 finalTranslation。
        for(size_t i=after;i<route->segmentCount;++i)if(route->segments[i].kind!=ROTATE)finalTranslation=false;  // 按 size_t i=after;i<route->segmentCount;++i)if(route->seg… 组织循环的初值、继续条件和步进。
        terminalTurn=after<route->segmentCount||fabsf(turn(targetHeading,route->config.endHeading))>settings->headingDeadbandDeg;  // 将 after<route->segmentCount||fabsf(turn(targetHeading,ro… 写入或初始化 terminalTurn。
        if(scanExit){targetHeading=output->heading;startMove(false);}else stage=ALIGN;  // 当 scanExit 成立时进入此分支，否则继续后续流程。
        break;  // 退出当前循环或 switch 分支。
    }  // 结束此条件分支。
    case ALIGN: {  // 当前值匹配 ALIGN 时从这里执行。
        failure=sample(m);if(failure)break;  // 将 sample(m) 写入或初始化 failure。然后若 failure，从当前循环或分支退出。
        const float angle=turn(output->heading,targetHeading);  // 将 turn(output->heading,targetHeading) 写入或初始化 angle（角度）。
        if(fabsf(angle)<=settings->headingDeadbandDeg){aligned();break;}  // 当 fabsf(angle)<=settings->headingDeadbandDeg 成立时进入此分支，否则继续后续流程。
        const bool slowCorrection=attempts==3&&settings->recoverSmallResidual&&!headingRecoveryUsed&&fabsf(angle)<=3;  // 将 attempts==3&&settings->recoverSmallResidual&&!headingR… 写入或初始化 slowCorrection（slow修正量）。
        if(attempts>=3&&!slowCorrection){failure="TURN_HEADING_ERROR";break;}  // 当 attempts>=3&&!slowCorrection 成立时进入此分支，否则继续后续流程。
        const float margin=slowCorrection?route->config.margin:0;  // 将 slowCorrection?route->config.margin:0 写入或初始化 margin（裕量）。
        for(int i=0;i<=90;++i)if(field::check(output->estimated.x,output->estimated.y,wrap(output->heading+angle*i/90),margin)){failure="TURN_SPACE";break;}  // 按 int i=0;i<=90;++i)if(field::check(output->estimated.x,… 组织循环的初值、继续条件和步进。
        if(!failure){headingRecoveryUsed|=slowCorrection;++attempts;startMove(true);}break;  // 当 !failure 成立时进入此分支，否则继续后续流程。
    }  // 结束此条件分支。
    case ZERO:  // 当前值匹配 ZERO 时从这里执行。
        failure=io->zero(zeroId);if(failure)break;  // 将 io->zero(zeroId) 写入或初始化 failure。然后若 failure，从当前循环或分支退出。
        failure=io->readPosition(zeroId,stopped[zeroId-1].position);if(failure)break;  // 将 io->readPosition(zeroId,stopped[zeroId-1].position) 写入或初始化 failure。然后若 failure，从当前循环或分支退出。
        if(llabs(stopped[zeroId-1].position)>64){failure="MOTOR_ZERO_FAILED";break;}  // 当 llabs(stopped[zeroId-1].position)>64 成立时进入此分支，否则继续后续流程。
        if(++zeroId>4){prepared=true;stage=PREPARE;}break;  // 当 ++zeroId>4 成立时进入此分支，否则继续后续流程。
    case PREPARE:  // 当前值匹配 PREPARE 时从这里执行。
        for(int i=0;i<4;++i) {  // 按 int i=0;i<4;++i 组织循环的初值、继续条件和步进。
            origin[i]=stopped[i].position;  // 将 stopped[i].position 写入或初始化 origin[i]。
            const int sign=encoderToCommandSign(i);  // 将 encoderToCommandSign(i) 写入或初始化 sign。
            goals[i]=llround(static_cast<double>(origin[i])*sign*PULSES_PER_REV/65536)+(directions[i]?int64_t(travelPulses[i]):-int64_t(travelPulses[i]));  // 将 llround(static_cast<double>(origin[i])*sign*PULSES_PER… 写入或初始化 goals[i]。
            if(llabs(goals[i])>UINT32_MAX){failure="MOTOR_TARGET_RANGE";break;}  // 当 llabs(goals[i])>UINT32_MAX 成立时进入此分支，否则继续后续流程。
        }  // 结束此循环体。
        if(failure)break;  // 若 failure，从当前循环或分支退出。
        valid=false;io->beginAction();start=output->estimated;baseS=output->s;  // 将 false 写入或初始化 valid（有效）。然后调用 io->beginAction。然后将 output->estimated 写入或初始化 start（起点）。然后将 output->s 写入或初始化 baseS。
        maximum=rotating?settings->turnDegS:route->segments[index].middle?settings->middleSpeedMmS:settings->speedMmS;  // 将 rotating?settings->turnDegS:route->segments[index].mid… 写入或初始化 maximum。
        if(!rotating&&action==STRAIGHT&&route->segments[index].slowApproach)maximum*=.5f;  // 当 !rotating&&action==STRAIGHT&&route->segments[index].sl… 成立时进入此分支，否则继续后续流程。
        if(rotating&&attempts>3)maximum=fminf(maximum*.25f,30.0f);  // 当 rotating&&attempts>3 成立时进入此分支，否则继续后续流程。
        acceleration=rotating?settings->turnAccelDegS2:route->segments[index].middle?settings->middleAccelMmS2:settings->accelMmS2;  // 将 rotating?settings->turnAccelDegS2:route->segments[inde… 写入或初始化 acceleration（加速度）。
        started=previous=io->now();lastReport=lastArrivalReport=0;  // 将 previous=io->now() 写入或初始化 started。然后将 lastArrivalReport=0 写入或初始化 lastReport（最后Report）。
        deadline=static_cast<uint32_t>((fabsf(delta)/maximum+maximum/acceleration+5)*3000);  // 将 static_cast<uint32_t>((fabsf(delta)/maximum+maximum/ac… 写入或初始化 deadline（截止时间）。
        sent=braking=quietSeen=residualRetried=false;quietCount=0;peakMeasuredRpm=0;headingGuard=HeadingGuard{};startHeading=output->heading;  // 将 braking=quietSeen=residualRetried=false;quietCount=0;p… 写入或初始化 sent（已发送）。
        representative=1;  // 将 1 写入或初始化 representative。
        for(uint8_t id=2;id<=4;++id)if(travelPulses[id-1]>travelPulses[representative-1])representative=id;  // 按 uint8_t id=2;id<=4;++id)if(travelPulses[id-1]>travelPu… 组织循环的初值、继续条件和步进。
        representativePulses=0;previousRepresentativeRpm=0;  // 将 0 写入或初始化 representativePulses（representative脉冲数）。然后将 0 写入或初始化 previousRepresentativeRpm（上一Representative转速）。
        representativeMoved=representativeStopped=comparisonReported=false;  // 将 representativeStopped=comparisonReported=false 写入或初始化 representativeMoved。
        unitsPerDistance=ppm*65536/PULSES_PER_REV;tolerance=rotating?unitsPerDistance*.4:STRAIGHT_ARRIVAL_ENCODER_TOLERANCE;  // 将 ppm*65536/PULSES_PER_REV 写入或初始化 unitsPerDistance（unitsPer距离）。然后将 rotating?unitsPerDistance*.4:STRAIGHT_ARRIVAL_ENCODER_… 写入或初始化 tolerance（容差）。
        hardLimit=rotating?unitsPerDistance*3:fmax(unitsPerDistance*10,2*STRAIGHT_ARRIVAL_ENCODER_TOLERANCE);  // 将 rotating?unitsPerDistance*3:fmax(unitsPerDistance*10,2… 写入或初始化 hardLimit（hard限值）。
        stage=MOVE;  // 将 MOVE 写入或初始化 stage（阶段）。
        if(!rotating&&!scanExit&&(route->segments[index].middle||settings->outerStraightVelocity)) {  // 当 !rotating&&!scanExit&&(route->segments[index].middle||… 成立时进入此分支，否则继续后续流程。
            headingHold.reset();straightSpeed=straightTravel=0;straightStopping=false;applied={};  // 将 straightTravel=0;straightStopping=false;applied={} 写入或初始化 straightSpeed（straight速度）。
            for(int i=0;i<4;++i)positions[i]=stopped[i].position;  // 按 int i=0;i<4;++i)positions[i]=stopped[i].position; 组织循环的初值、继续条件和步进。
            stage=STRAIGHT_VELOCITY;  // 将 STRAIGHT_VELOCITY 写入或初始化 stage（阶段）。
        }  // 结束此条件分支。
        break;  // 退出当前循环或 switch 分支。
    case MOVE: moveTick();break;  // 当前值匹配 MOVE 时从这里执行。
    case STRAIGHT_VELOCITY: straightVelocityTick();break;  // 当前值匹配 STRAIGHT_VELOCITY 时从这里执行。
    case OUTER: {  // 当前值匹配 OUTER 时从这里执行。
        const uint32_t now=io->now();const float dt=fmaxf(.001f,(now-previous)*.001f);previous=now;  // 将 io->now() 写入或初始化 now。然后将 fmaxf(.001f,(now-previous)*.001f) 写入或初始化 dt（时间差）。然后将 now 写入或初始化 previous（上一）。
        Motor states[4];for(uint8_t id=1;id<=4;++id){failure=io->read(id,states[id-1]);if(failure)break;failure=health(*io,id,states[id-1]);if(failure)break;}  // 调用 for，传入 uint8_t id=1;id<=4;++id){failure=io->read(id,states[id…。
        if(failure)break;  // 若 failure，从当前循环或分支退出。
        failure=displacement(states);if(failure)break;  // 将 displacement(states) 写入或初始化 failure。然后若 failure，从当前循环或分支退出。
        if(tracker.s>=curveStop-.5f){io->halt();quietStarted=io->now();stage=OUTER_QUIET;break;}  // 当 tracker.s>=curveStop-.5f 成立时进入此分支，否则继续后续流程。
        failure=sample(m);if(failure)break;  // 将 sample(m) 写入或初始化 failure。然后若 failure，从当前循环或分支退出。
        Output out=tracker.update(*route,output->heading,dt,m.receivedAtMs);  // 将 tracker.update(*route,output->heading,dt,m.receivedAtM… 写入或初始化 out。
        if(out.failure){failure=out.failure;break;}  // 当 out.failure 成立时进入此分支，否则继续后续流程。
        if(out.finished){io->halt();quietStarted=io->now();stage=OUTER_QUIET;break;}  // 当 out.finished 成立时进入此分支，否则继续后续流程。
        failure=io->velocity(out.wheels);if(failure)break;  // 将 io->velocity(out.wheels) 写入或初始化 failure。然后若 failure，从当前循环或分支退出。
        applied=out.wheels;tracker.commit(applied,io->now());  // 将 out.wheels 写入或初始化 applied。然后调用 tracker.commit，传入 applied,io->now()。
        if(now-lastReport>=200){io->report(route->segments[tracker.phaseIndex].kind==CUBIC?"OUTER_CURVE":"OUTER_DRIFT",*output,out.reference);lastReport=now;}  // 当 now-lastReport>=200 成立时进入此分支，否则继续后续流程。
        due=io->now()+20;break;  // 将 io->now()+20 写入或初始化 due。然后退出当前循环或 switch 分支。
    }  // 结束此条件分支。
    default: break;  // 没有匹配的 case 时走此默认分支。
    }  // 结束此函数或控制块。
    if(failure)fail(failure);  // 当 failure 成立时进入此分支，否则继续后续流程。
}  // 结束此函数或控制块。
}  // 结束命名空间。

namespace node_position {  // 定义 node_position（节点位置） 的类型成员或作用域。
// 同步兼容入口仅供旧调用和原生仿真；正式运动任务直接交替调用 tick。
/**
 * @brief 同步推进节点位置路径直至结束；result 保存估算位置和航向。
 * @return nullptr 成功，否则返回错误原因。
 */
const char *run(const continuous::Route &route,const Settings &settings,IO &io,Result &result) {  // 定义函数或成员 的入口 run，参数由此处的签名确定。
    Runner runner;runner.begin(route,settings,io,result);  // 声明 runner，类型和可用范围由所在声明决定。然后调用 runner.begin，传入 route,settings,io,result。
    while(runner.active()) {runner.tick();if(runner.active())io.wait(runner.nextDelay());}  // 当 runner.active() 成立时继续循环。
    return runner.failure();  // 结束当前函数并返回 runner.failure()。
}  // 结束此函数或控制块。
}  // 结束命名空间。
