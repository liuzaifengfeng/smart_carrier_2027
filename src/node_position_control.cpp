#include "node_position_control.h"
#include "field_geometry.h"
#include <stdlib.h>

namespace node_position {
using namespace continuous;
namespace {
const char *health(IO &io,uint8_t id,const Motor &m,double target=0) {
    const char *error=!(m.flags&1)?"MOTOR_DISABLED":
        MOTOR_STALL_CHECK_ENABLED&&(m.flags&12)?"MOTOR_STALL":nullptr;
    if(error)io.motorDiagnostic(id,error,target,m);
    return error;
}
}
/**
 * @brief 读取惯性数据并处理当前路径的航向参考可用性；成功返回 nullptr。
 */
const char *Runner::sample(H30Attitude &motion) {
    if(io->cancelled())return "STOPPED";
    float measured=output->heading;
    const char *failure=io->imu(motion,measured);
    if(failure||(motion.fields&7)!=7||!isfinite(measured))motion=H30Attitude{};
    else output->heading=measured;
    return io->cancelled()?"STOPPED":nullptr;
}
/**
 * @brief 记录路径失败原因，切到结束阶段并停车。
 */
void Runner::fail(const char *reason) {error=reason;stage=FINISHED;io->halt();}
/**
 * @brief 初始化整条节点路径，记录配置、硬件接口和输出结果；后续由 tick 推进。
 */
void Runner::begin(const Route &path,const Settings &config,IO &hardware,Result &result) {
    *this=Runner{};route=&path;settings=&config;io=&hardware;output=&result;
    offset=config.startOffset;result.estimated=path.segments[0].start+offset;
    // 最后一组连续平移前的转向阶段即开始准备观察姿态；单段路径从起步准备。
    size_t finalBlock=path.segmentCount;
    while(finalBlock&&path.segments[finalBlock-1].kind==ROTATE)--finalBlock;
    while(finalBlock&&path.segments[finalBlock-1].kind!=ROTATE)--finalBlock;
    observationIndex=finalBlock?finalBlock-1:0;
    stage=START;due=hardware.now();
}
/**
 * @brief 返回到下次推进所需等待毫秒数，供统一运动任务调度。
 */
uint32_t Runner::nextDelay() const {
    const int32_t left=static_cast<int32_t>(due-io->now());return left>0?left:0;
}
/**
 * @brief 返回当前平移动作剩余距离(mm)；运行中由代表轮转速估算，结束时由四轮位置校准。
 */
float Runner::remainingDistance() const {
    return fmaxf(0,fabsf(distance)-(output->s-baseS));
}
/**
 * @brief 准备当前直行或转向动作的四轮目标与速度曲线；参数表示是否为到位校角。
 */
void Runner::startMove(bool alignment) {
    turningForAlignment=alignment;rotating=alignment;
    H30Attitude m;const char *failure=sample(m);if(failure){fail(failure);return;}
    delta=rotating?turn(output->heading,targetHeading):distance;
    ppm=rotating?route->config.rotationPpd:route->config.forwardPpm;
    const uint32_t pulses=static_cast<uint32_t>(lroundf(fabsf(delta)*ppm));
    if(!pulses){arrived();return;}
    Vec body=scanExit&&!rotating?worldToBody(end-output->estimated,targetHeading):Vec{};
    const float px=body.x*route->config.lateralPpm,py=body.y*route->config.forwardPpm;
    const float scanPulses[4]={-px+py,-px-py,px+py,px-py};
    movingWheels=0;
    for(int i=0;i<4;++i) {
        travelPulses[i]=scanExit&&!rotating?static_cast<uint32_t>(lroundf(fabsf(scanPulses[i]))):pulses;
        if(travelPulses[i])++movingWheels;
        directions[i]=rotating?delta>=0:((distance>=0)==(i%2==0));
        if(scanExit&&!rotating)directions[i]=scanPulses[i]>=0;
    }
    zeroId=1;stage=prepared?PREPARE:ZERO;
}
/**
 * @brief 结束当前航向对齐，转入后续路径选择阶段。
 */
void Runner::aligned() {
    if(action==FINAL){stage=FINISHED;io->halt();return;}
    if(action==ROTATION){index=after;stage=SELECT;return;}
    if(action==STRAIGHT){startMove(false);return;}
    tracker=Tracker{};tracker.begin(*route,output->heading);tracker.movingBlock=true;
    tracker.phaseIndex=curveBegin;tracker.s=route->segments[curveBegin].offset;tracker.estimated=output->estimated;
    const Segment &last=route->segments[after-1];curveStop=last.offset+last.length;
    for(int i=0;i<4;++i)positions[i]=stopped[i].position;
    applied=WheelCommand{};previous=io->now();lastReport=0;valid=false;
    io->beginAction();io->timing("OUTER_DRIFT_SENT");stage=OUTER;
}
/**
 * @brief 完成当前移动段并推进路径索引和估算结果。
 */
void Runner::arrived() {
    if(turningForAlignment){stage=ALIGN;return;}
    offset={};index=after;stage=SELECT;
}
const char *Runner::updateProgress(float progress,const H30Attitude &m,bool idle) {
    if(rotating&&(m.fields&7)!=7)output->heading=wrap(startHeading+delta*progress);
    const float headingError=turn(output->heading,targetHeading);
    if(!rotating&&headingGuard.update(headingError,m.receivedAtMs,braking,idle,(m.fields&7)==7)) {
        io->report("HEADING_ERROR",*output,targetHeading);return "HEADING_ERROR";
    }
    if(!rotating) {
        output->estimated=start+(end-start)*progress;
        output->s=baseS+fabsf(delta)*progress;
        return field::check(output->estimated.x,output->estimated.y,output->heading,0);
    }
    return nullptr;
}
/**
 * @brief 运动中只读代表轮转速；近零时回读四轮，保留原位置/转速到位条件并输出对比。
 */
void Runner::moveTick() {
    const uint32_t cycleAt=io->now();
    if(cycleAt-started>deadline){fail("MOTOR_ARRIVAL_TIMEOUT");return;}
    const float dt=(cycleAt-previous)*.001f;previous=cycleAt;
    if(dt>.5f){fail("CONTROL_OVERRUN");return;}
    H30Attitude m;const char *failure=sample(m);if(failure){fail(failure);return;}
    if(sent) {
        int16_t rpm=0;
        failure=io->readSpeed(representative,rpm);if(failure){fail(failure);return;}
        const uint32_t now=io->now();
        representativePulses+=(abs(previousRepresentativeRpm)+abs(rpm))*.5
            *PULSES_PER_REV*(now-representativeSampleAt)/60000.0;
        previousRepresentativeRpm=rpm;representativeSampleAt=now;
        if(abs(rpm)>3)representativeMoved=true;
        if(peakMeasuredRpm>abs(rpm)+3)braking=true;
        peakMeasuredRpm=static_cast<uint16_t>(fmaxf(peakMeasuredRpm,abs(rpm)));
        if(abs(rpm)>3) {
            // 中途位置仅用于进度估计；最终是否到位仍由下面四轮实测决定。
            const float progress=static_cast<float>(fmin(1,representativePulses/travelPulses[representative-1]));
            failure=updateProgress(progress,m,false);if(failure){fail(failure);return;}
            if(now-lastReport>=200){io->report(rotating?"POSITION_TURN":"POSITION_STRAIGHT",*output,targetHeading);lastReport=now;}
            due=io->now()+(braking?1:20);return;
        }
        if(representativeMoved&&!representativeStopped){representativeStopped=true;representativeStopAt=now;}
        // 起步时也可能读到0：只触发完整检查，不据此直接接续下一动作。
    }
    const uint32_t checkStarted=io->now();
    bool arrivedNow=sent,allIdle=true;double progress=0,maxResidual=0;Motor measured[4];
    float residuals[4];
    uint16_t measuredRpm=0;
    for(uint8_t id=1;id<=4;++id) {
        Motor state=stopped[id-1];
        const double goal=static_cast<double>(travelPulses[id-1])*65536/PULSES_PER_REV;
        if(sent){failure=io->read(id,state);if(failure){fail(failure);return;}}
        const int64_t rawDelta=state.position-origin[id-1];
        const int sign=encoderSign[id-1]?encoderSign[id-1]:1;
        const double absoluteGoal=static_cast<double>(goals[id-1])*sign*65536/PULSES_PER_REV;
        failure=health(*io,id,state,absoluteGoal);if(failure){fail(failure);return;}
        measured[id-1]=state;
        const double current=encoderSign[id-1]?static_cast<double>(rawDelta)*sign*(directions[id-1]?1:-1):
            static_cast<double>(llabs(rawDelta));
        const double residual=fabs(current-goal);maxResidual=fmax(maxResidual,residual);
        residuals[id-1]=static_cast<float>((current-goal)/unitsPerDistance);
        if(current>goal+hardLimit){io->motorDiagnostic(id,"MOTOR_POSITION_OVERRUN",absoluteGoal,state);fail("MOTOR_POSITION_OVERRUN");return;}
        measuredRpm=static_cast<uint16_t>(fmaxf(measuredRpm,abs(state.rpm)));
        if(abs(state.rpm)>3)allIdle=false;
        const double arrivalResidual=rotating?residual:static_cast<double>(llround(residual));
        if(arrivalResidual>tolerance||abs(state.rpm)>3)arrivedNow=false;
        if(goal>0)progress+=fmax(0,fmin(1,current/goal))/movingWheels;
    }
    const uint32_t readMs=io->now()-checkStarted;
    if(sent) {
        representativePulses=fabs(static_cast<double>(measured[representative-1].position-origin[representative-1]))*PULSES_PER_REV/65536;
        representativeSampleAt=io->now();previousRepresentativeRpm=measured[representative-1].rpm;
        if((representativeMoved||arrivedNow)&&(arrivedNow||!comparisonReported||io->now()-lastComparison>=200)) {
            const uint32_t waitMs=representativeStopped?io->now()-representativeStopAt:readMs;
            io->arrivalComparison(representative,readMs,waitMs,arrivedNow,rotating,residuals,measured);
            comparisonReported=true;lastComparison=io->now();
        }
    }
    failure=sample(m);if(failure){fail(failure);return;}
    if(sent&&peakMeasuredRpm>measuredRpm+3)braking=true;
    peakMeasuredRpm=static_cast<uint16_t>(fmaxf(peakMeasuredRpm,measuredRpm));
    failure=updateProgress(static_cast<float>(progress),m,allIdle);if(failure){fail(failure);return;}
    if(arrivedNow) {
        for(int i=0;i<4;++i) {
            const int64_t rawDelta=measured[i].position-origin[i];
            if(!encoderSign[i]&&travelPulses[i]&&rawDelta)encoderSign[i]=(rawDelta>=0?1:-1)*(directions[i]?1:-1);
            stopped[i]=measured[i];
        }
        io->timing(rotating?"TURN_ARRIVED":"STRAIGHT_ARRIVED");valid=true;arrived();return;
    }
    if(sent&&allIdle&&io->now()-lastArrivalReport>=200) {
        io->arrivalDiagnostic(static_cast<float>(maxResidual/unitsPerDistance),measured);lastArrivalReport=io->now();
    }
    // 沿用既有静止残差故障条件；正常到位不增加稳定等待。
    const bool enabled=sent&&allIdle&&maxResidual<=hardLimit;
    bool calm=enabled;
    for(int i=0;i<4;++i)if(abs(measured[i].rpm)>3||(quietSeen&&llabs(measured[i].position-quietAnchor[i])>128))calm=false;
    if(!enabled){quietSeen=false;quietCount=0;}
    else {
        if(!quietSeen||!calm){quietSeen=true;residualSince=io->now();quietCount=0;for(int i=0;i<4;++i)quietAnchor[i]=measured[i].position;}
        if(calm&&quietCount<255)++quietCount;
        if(calm&&quietCount>=2&&io->now()-residualSince>=500) {
            for(uint8_t id=1;id<=4;++id)io->motorDiagnostic(id,"MOTOR_RESIDUAL_TIMEOUT",
                static_cast<double>(goals[id-1])*(encoderSign[id-1]?encoderSign[id-1]:1)*65536/PULSES_PER_REV,measured[id-1]);
            fail("MOTOR_RESIDUAL_TIMEOUT");return;
        }
    }
    if(!sent) {
        const uint16_t rpm=rpmFor(maximum,ppm);
        for(uint8_t id=1;id<=4;++id) {
            const float wheelPpm=scanExit&&!rotating?travelPulses[id-1]/fabsf(delta):ppm;
            failure=io->target(id,goals[id-1]>=0,scanExit&&!rotating?rpmFor(maximum,wheelPpm):rpm,
                driverAccelerationFor(acceleration,wheelPpm),static_cast<uint32_t>(llabs(goals[id-1])));
            if(failure){fail(failure);return;}
        }
        if(io->cancelled()){fail("STOPPED");return;}
        failure=io->sync();if(failure){fail(failure);return;}
        sent=true;representativeSampleAt=io->now();io->timing(rotating?"TURN_SENT":"STRAIGHT_SENT");
    }
    if(io->now()-lastReport>=200){io->report(rotating?"POSITION_TURN":"POSITION_STRAIGHT",*output,targetHeading);lastReport=io->now();}
    due=io->now()+(allIdle||maxResidual<=unitsPerDistance*(rotating?2.0:10.0)?1:20);
}
/**
 * @brief 根据四轮反馈更新本次动作行程和到位残差；成功返回 nullptr。
 */
const char *Runner::displacement(const Motor (&states)[4]) {
    float pulses[4];
    for(int i=0;i<4;++i) {
        const int64_t change=states[i].position-positions[i];
        if(!encoderSign[i]&&llabs(change)>128&&applied.rpm[i])encoderSign[i]=(change>=0?1:-1)*(applied.rpm[i]>=0?1:-1);
        pulses[i]=encoderSign[i]?static_cast<float>(change*encoderSign[i])*PULSES_PER_REV/65536:0;
        positions[i]=states[i].position;
    }
    Vec body{(-pulses[0]-pulses[1]+pulses[2]+pulses[3])/(4*route->config.lateralPpm),
              (pulses[0]-pulses[1]+pulses[2]-pulses[3])/(4*route->config.forwardPpm)};
    const float oldHeading=output->heading;H30Attitude m;const char *failure=sample(m);if(failure)return failure;
    if((m.fields&7)!=7)output->heading=wrap(oldHeading+(pulses[0]+pulses[1]+pulses[2]+pulses[3])/(4*route->config.rotationPpd));
    const Vec world=bodyToWorld(body,oldHeading+turn(oldHeading,output->heading)*.5f);
    output->estimated=output->estimated+world;tracker.estimated=output->estimated;
    const float window=norm(world)*2+20,lo=fmaxf(route->segments[curveBegin].offset,tracker.s-window),hi=fminf(curveStop,tracker.s+window);
    auto distanceAt=[&](float s){size_t i=route->movingPhase(s);return norm(route->phaseAt(i,s-route->segments[i].offset).p-output->estimated);};
    float best=lo,bestDistance=distanceAt(lo);
    for(float s=lo;s<=hi;s+=5){float d=distanceAt(s);if(d<bestDistance){best=s;bestDistance=d;}}
    if(distanceAt(hi)<bestDistance)best=hi;
    float a=fmaxf(lo,best-5),b=fminf(hi,best+5);
    for(int i=0;i<12;++i){float u=a+(b-a)/3,v=b-(b-a)/3;if(distanceAt(u)<distanceAt(v))b=v;else a=u;}
    tracker.s=fmaxf(tracker.s,(a+b)*.5f);output->s=tracker.s;return nullptr;
}
/**
 * @brief 推进当前节点控制阶段一次，包括起步、段选择、位置目标下发及到位反馈。
 */
void Runner::tick() {
    if(!active())return;
    if(io->cancelled()){fail("STOPPED");return;}
    if(nextDelay())return;
    const char *failure=nullptr;H30Attitude m;
    switch(stage) {
    case START: io->halt();quietStarted=io->now();stage=QUIET;break;
    case QUIET:
    case OUTER_QUIET: {
        if(io->now()-quietStarted>3000){fail("MOTOR_STOP_TIMEOUT");break;}
        failure=sample(m);if(failure)break;
        bool idle=true;
        for(uint8_t id=1;id<=4;++id){failure=io->read(id,stopped[id-1]);if(failure)break;
            failure=health(*io,id,stopped[id-1]);if(failure)break;if(abs(stopped[id-1].rpm)>3)idle=false;}
        if(failure)break;
        failure=sample(m);if(failure)break;
        if(idle){valid=true;
            if(stage==OUTER_QUIET){failure=displacement(stopped);prepared=true;io->timing("OUTER_DRIFT_ARRIVED");index=after;}
            stage=SELECT;
        } else due=io->now()+5;
        break;
    }
    case SELECT: {
        finalTranslation=false;terminalTurn=false;
        if(index>=route->segmentCount){action=FINAL;targetHeading=route->config.endHeading;attempts=0;stage=ALIGN;break;}
        const Segment &phase=route->segments[index];after=index+1;
        if(phase.kind==ROTATE) {
            if(norm(offset)>0){++index;break;}
            action=ROTATION;targetHeading=phase.h1;attempts=0;stage=ALIGN;break;
        }
        size_t block=index;bool curved=false;
        while(block<route->segmentCount&&route->segments[block].kind!=ROTATE){curved|=route->segments[block].kind==CUBIC;++block;}
        targetHeading=phase.h0;attempts=0;
        if(curved){action=CURVE;curveBegin=index;after=block;stage=ALIGN;break;}
        action=STRAIGHT;end=phase.end;float length=phase.length;
        while(after<route->segmentCount) {
            const auto &next=route->segments[after];
            if(next.kind!=LINE||fabsf(turn(phase.h0,next.h0))>.01f||dot(end-phase.start,next.end-next.start)<=0)break;
            end=next.end;length+=next.length;++after;
        }
        scanExit=norm(offset)>0;const Vec direction=(end-phase.start)*(1/length);
        const float sign=dot(direction,Vec(cosf(phase.h0*PI_F/180),sinf(phase.h0*PI_F/180)))>=0?1:-1;
        distance=sign*(scanExit?norm(end-output->estimated):length);
        finalTranslation=true;
        for(size_t i=after;i<route->segmentCount;++i)if(route->segments[i].kind!=ROTATE)finalTranslation=false;
        terminalTurn=after<route->segmentCount||fabsf(turn(targetHeading,route->config.endHeading))>settings->headingDeadbandDeg;
        if(scanExit){targetHeading=output->heading;startMove(false);}else stage=ALIGN;
        break;
    }
    case ALIGN: {
        failure=sample(m);if(failure)break;
        const float angle=turn(output->heading,targetHeading);
        if(fabsf(angle)<=settings->headingDeadbandDeg){aligned();break;}
        if(attempts>=3){failure="TURN_HEADING_ERROR";break;}
        for(int i=0;i<=90;++i)if(field::check(output->estimated.x,output->estimated.y,wrap(output->heading+angle*i/90),0)){failure="TURN_SPACE";break;}
        if(!failure){++attempts;startMove(true);}break;
    }
    case ZERO:
        failure=io->zero(zeroId);if(failure)break;
        failure=io->readPosition(zeroId,stopped[zeroId-1].position);if(failure)break;
        if(llabs(stopped[zeroId-1].position)>64){failure="MOTOR_ZERO_FAILED";break;}
        if(++zeroId>4){prepared=true;stage=PREPARE;}break;
    case PREPARE:
        for(int i=0;i<4;++i) {
            origin[i]=stopped[i].position;
            const int sign=encoderSign[i]?encoderSign[i]:1;
            goals[i]=llround(static_cast<double>(origin[i])*sign*PULSES_PER_REV/65536)+(directions[i]?int64_t(travelPulses[i]):-int64_t(travelPulses[i]));
            if(llabs(goals[i])>UINT32_MAX){failure="MOTOR_TARGET_RANGE";break;}
        }
        if(failure)break;
        valid=false;io->beginAction();start=output->estimated;baseS=output->s;
        maximum=rotating?settings->turnDegS:settings->speedMmS;
        acceleration=rotating?settings->turnAccelDegS2:settings->accelMmS2;
        started=previous=io->now();lastReport=lastArrivalReport=0;
        deadline=static_cast<uint32_t>((fabsf(delta)/maximum+maximum/acceleration+5)*3000);
        sent=braking=quietSeen=false;quietCount=0;peakMeasuredRpm=0;headingGuard=HeadingGuard{};startHeading=output->heading;
        representative=1;
        for(uint8_t id=2;id<=4;++id)if(travelPulses[id-1]>travelPulses[representative-1])representative=id;
        representativePulses=0;previousRepresentativeRpm=0;
        representativeMoved=representativeStopped=comparisonReported=false;
        unitsPerDistance=ppm*65536/PULSES_PER_REV;tolerance=rotating?unitsPerDistance*.25:STRAIGHT_ARRIVAL_ENCODER_TOLERANCE;
        hardLimit=rotating?unitsPerDistance*3:fmax(unitsPerDistance*10,2*STRAIGHT_ARRIVAL_ENCODER_TOLERANCE);
        stage=MOVE;break;
    case MOVE: moveTick();break;
    case OUTER: {
        const uint32_t now=io->now();const float dt=fmaxf(.001f,(now-previous)*.001f);previous=now;
        Motor states[4];for(uint8_t id=1;id<=4;++id){failure=io->read(id,states[id-1]);if(failure)break;failure=health(*io,id,states[id-1]);if(failure)break;}
        if(failure)break;
        failure=displacement(states);if(failure)break;
        if(tracker.s>=curveStop-.5f){io->halt();quietStarted=io->now();stage=OUTER_QUIET;break;}
        failure=sample(m);if(failure)break;
        Output out=tracker.update(*route,output->heading,dt,m.receivedAtMs);
        if(out.failure){failure=out.failure;break;}
        if(out.finished){io->halt();quietStarted=io->now();stage=OUTER_QUIET;break;}
        failure=io->velocity(out.wheels);if(failure)break;
        applied=out.wheels;tracker.commit(applied,io->now());
        if(now-lastReport>=200){io->report(route->segments[tracker.phaseIndex].kind==CUBIC?"OUTER_CURVE":"OUTER_DRIFT",*output,out.reference);lastReport=now;}
        due=io->now()+20;break;
    }
    default: break;
    }
    if(failure)fail(failure);
}
}

namespace node_position {
// 同步兼容入口仅供旧调用和原生仿真；正式运动任务直接交替调用 tick。
/**
 * @brief 同步推进节点位置路径直至结束；result 保存估算位置和航向。
 * @return nullptr 成功，否则返回错误原因。
 */
const char *run(const continuous::Route &route,const Settings &settings,IO &io,Result &result) {
    Runner runner;runner.begin(route,settings,io,result);
    while(runner.active()) {runner.tick();if(runner.active())io.wait(runner.nextDelay());}
    return runner.failure();
}
}
