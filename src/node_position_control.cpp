#include "node_position_control.h"
#include "field_geometry.h"
#include <stdlib.h>

namespace node_position {
using namespace continuous;
namespace {
const char *health(const Motor &m) {
    if(!(m.flags&1))return "MOTOR_DISABLED";
    if(MOTOR_STALL_CHECK_ENABLED&&(m.flags&12))return "MOTOR_STALL";
    return nullptr;
}
const char *health(IO &io,uint8_t id,const Motor &m,double target=0) {
    const char *error=health(m);
    if(error)io.motorDiagnostic(id,error,target,m);
    return error;
}
const char *sample(IO &io,H30Attitude &m,float &heading) {
    if(io.cancelled())return "STOPPED";
    float measured=heading;const char *error=io.imu(m,measured);
    // IMU只是辅助；失效时保留编码器/规划航向，不伪造惯性样本。
    if(error||(m.fields&7)!=7||!isfinite(measured)) {
        m=H30Attitude{};return io.cancelled()?"STOPPED":nullptr;
    }
    heading=measured;return nullptr;
}
struct Context {
    bool prepared=false,valid=false;Motor stopped[4];
    int encoderSign[4]={}; // 首次有限动作学习各轮反馈符号与指令坐标的对应关系
};
// 仅用于异常静止残差的500ms故障判定；正常起步和接续不等待稳定窗口。
class MotorQuiet {
    bool seen=false;uint32_t since=0;uint8_t count=0;int64_t anchor[4]={};
public:
    bool update(const Motor (&state)[4],uint32_t now,uint32_t window,bool enabled=true) {
        if(!enabled){seen=false;count=0;return false;}
        bool calm=enabled;
        for(int i=0;i<4;++i)if(abs(state[i].rpm)>3||(seen&&llabs(state[i].position-anchor[i])>128))calm=false;
        if(!seen||!calm) {
            seen=true;since=now;count=0;
            for(int i=0;i<4;++i)anchor[i]=state[i].position;
        }
        if(!calm)return false;
        if(count<255)++count;
        return count>=2&&now-since>=window;
    }
};
const char *quiet(IO &io,Result &result,Context &context) {
    uint32_t started=io.now();context.valid=false;
    while(io.now()-started<=3000) {
        H30Attitude m;const char *error=sample(io,m,result.heading);if(error)return error;
        bool idle=true;Motor states[4];
        for(uint8_t id=1;id<=4;++id) {
            Motor state;error=io.read(id,state);if(error)return error;
            error=health(io,id,state);if(error)return error;
            states[id-1]=state;
            if(abs(state.rpm)>3)idle=false;
        }
        error=sample(io,m,result.heading);if(error)return error;
        if(idle) {
            for(int i=0;i<4;++i)context.stopped[i]=states[i];
            context.valid=true;return nullptr;
        }
        io.wait(5);
    }
    return "MOTOR_STOP_TIMEOUT";
}
// 每个动作只发送一次带加速度的有限位置目标，由驱动板执行加减速。
const char *move(IO &io,const Settings &settings,const Config &cfg,Result &result,
                 Vec end,float distance,float targetHeading,bool rotating,Context &context) {
    H30Attitude m;const char *error=sample(io,m,result.heading);if(error)return error;
    float delta=rotating?turn(result.heading,targetHeading):distance;
    float ppm=rotating?cfg.rotationPpd:cfg.forwardPpm;
    uint32_t pulses=static_cast<uint32_t>(lroundf(fabsf(delta)*ppm));
    if(!pulses)return nullptr;
    if(!context.valid) {
        error=quiet(io,result,context);if(error)return error;
    }
    if(!context.prepared) {
    // 复用起步四轮近零转速反馈，不再重复整组及逐轮完整查询。
    // 清零仍等ACK，并只读位置验证；不改变H30参考或5/6号机械臂。
    for(uint8_t id=1;id<=4;++id) {
        error=io.zero(id);if(error)return error;
        error=io.readPosition(id,context.stopped[id-1].position);if(error)return error;
        if(llabs(context.stopped[id-1].position)>64)return "MOTOR_ZERO_FAILED";
    }
    context.prepared=true;
    }
    int64_t origin[4],goals[4];bool directions[4];
    for(int i=0;i<4;++i) {
        origin[i]=context.stopped[i].position;
        directions[i]=rotating?delta>=0:((distance>=0)==(i%2==0));
        int sign=context.encoderSign[i]?context.encoderSign[i]:1;
        goals[i]=llround(static_cast<double>(origin[i])*sign*PULSES_PER_REV/65536)+(directions[i]?int64_t(pulses):-int64_t(pulses));
        if(llabs(goals[i])>UINT32_MAX)return "MOTOR_TARGET_RANGE";
    }
    context.valid=false;io.beginAction();
    Vec start=result.estimated;float baseS=result.s;
    float maximum=rotating?settings.turnDegS:settings.speedMmS;
    float acceleration=rotating?settings.turnAccelDegS2:settings.accelMmS2;
    uint8_t driverAcceleration=driverAccelerationFor(acceleration,ppm);
    uint32_t started=io.now(),previous=started,lastReport=0,lastArrivalReport=0;
    float amount=fabsf(delta);
    uint32_t deadline=static_cast<uint32_t>((amount/maximum+maximum/acceleration+5)*3000);
    bool sent=false,braking=false;
    HeadingGuard headingGuard;
    uint16_t peakMeasuredRpm=0;
    MotorQuiet residualQuiet;float startHeading=result.heading;
    const double goal=static_cast<double>(pulses)*65536/PULSES_PER_REV;
    const double unitsPerDistance=ppm*65536/PULSES_PER_REV;
    const double tolerance=rotating?unitsPerDistance*.25:STRAIGHT_ARRIVAL_ENCODER_TOLERANCE;
    // 越界保护必须在允许到位误差之外，避免合格的正向残差先触发超限。
    const double hardLimit=rotating?unitsPerDistance*3.0:
        fmax(unitsPerDistance*10.0,2*STRAIGHT_ARRIVAL_ENCODER_TOLERANCE);
    while(io.now()-started<=deadline) {
        uint32_t cycleAt=io.now();float dt=(cycleAt-previous)*.001f;previous=cycleAt;
        if(dt>.5f)return "CONTROL_OVERRUN";
        error=sample(io,m,result.heading);if(error)return error;
        float headingError=turn(result.heading,targetHeading);
        bool arrived=sent,allIdle=true;double progress=0,maxResidual=0;Motor measured[4];
        uint16_t measuredRpm=0;
        for(uint8_t id=1;id<=4;++id) {
            // 第一轮直接使用起步/上一动作到位反馈，发令后恢复实时完整查询。
            Motor state=context.stopped[id-1];
            if(sent){error=io.read(id,state);if(error)return error;}
            int64_t rawDelta=state.position-origin[id-1];
            if(sent&&!context.encoderSign[id-1]&&llabs(rawDelta)>128)
                context.encoderSign[id-1]=(rawDelta>=0?1:-1)*(directions[id-1]?1:-1);
            int sign=context.encoderSign[id-1]?context.encoderSign[id-1]:1;
            double absoluteGoal=static_cast<double>(goals[id-1])*sign*65536/PULSES_PER_REV;
            error=health(io,id,state,absoluteGoal);if(error)return error;
            measured[id-1]=state;
            double current=context.encoderSign[id-1]?static_cast<double>(rawDelta)*sign*(directions[id-1]?1:-1):static_cast<double>(llabs(rawDelta));
            double residual=fabs(current-goal);maxResidual=fmax(maxResidual,residual);
            if(current>goal+hardLimit) {
                io.motorDiagnostic(id,"MOTOR_POSITION_OVERRUN",absoluteGoal,state);
                return "MOTOR_POSITION_OVERRUN";
            }
            if(current < -hardLimit)return "MOTOR_DIRECTION_ERROR";
            measuredRpm=static_cast<uint16_t>(fmaxf(measuredRpm,abs(state.rpm)));
            if(abs(state.rpm)>3)allIdle=false;
            // 接续以每轮实际位置和转速为准，不等待驱动器更严格的到位标志。
            // flags仍用于使能检查和诊断；堵转检查由独立开关控制，不能仅凭标志或平均进度推进。
            // 直行按整数编码器单位比较，消除脉冲目标换算产生的不足1单位舍入差。
            const double arrivalResidual=rotating?residual:static_cast<double>(llround(residual));
            if(arrivalResidual>tolerance||abs(state.rpm)>3)arrived=false;
            progress+=fmax(0,fmin(1,current/goal))/4;
        }
        // 轮询后更新航向，仅用于偏航保护及停止后的校角，不在直行中差动调速。
        error=sample(io,m,result.heading);if(error)return error;
        if(rotating&&(m.fields&7)!=7)result.heading=wrap(startHeading+delta*static_cast<float>(progress));
        // 仅从反馈识别减速供偏航保护使用，不改变已下发的轮速或位置目标。
        if(sent&&peakMeasuredRpm>measuredRpm+3)braking=true;
        peakMeasuredRpm=static_cast<uint16_t>(fmaxf(peakMeasuredRpm,measuredRpm));
        headingError=turn(result.heading,targetHeading);
        if(!rotating&&headingGuard.update(headingError,m.receivedAtMs,braking,allIdle,(m.fields&7)==7)) {
            io.report("HEADING_ERROR",result,targetHeading);return "HEADING_ERROR";
        }
        if(!rotating) {
            result.estimated=start+(end-start)*static_cast<float>(progress);
            result.s=baseS+amount*progress;
            error=field::check(result.estimated.x,result.estimated.y,result.heading,0);if(error)return error;
        }
        // 四轮各自进入位置容差且近零转速即接续，不附加稳定窗口或到位标志等待。
        if(arrived) {
            for(int i=0;i<4;++i)context.stopped[i]=measured[i];
            io.timing(rotating?"TURN_ARRIVED":"STRAIGHT_ARRIVED");
            context.valid=true;return nullptr;
        }
        if(sent&&allIdle&&io.now()-lastArrivalReport>=200) {
            // 车已近零转速但尚未通过到位条件，输出原因而不插入等待或重发。
            io.arrivalDiagnostic(static_cast<float>(maxResidual/unitsPerDistance),measured);
            lastArrivalReport=io.now();
        }
        // 停止后的小残差留500ms供驱动闭环收敛；仍未到位就报错，不自动重发。
        if(residualQuiet.update(measured,io.now(),500,sent&&allIdle&&maxResidual<=hardLimit)) {
            for(uint8_t id=1;id<=4;++id)
                io.motorDiagnostic(id,"MOTOR_RESIDUAL_TIMEOUT",static_cast<double>(goals[id-1])
                    *(context.encoderSign[id-1]?context.encoderSign[id-1]:1)*65536/PULSES_PER_REV,measured[id-1]);
            return "MOTOR_RESIDUAL_TIMEOUT";
        }
        if(!sent) {
            const uint16_t rpm=rpmFor(maximum,ppm);
            for(uint8_t id=1;id<=4;++id) {
                error=io.target(id,goals[id-1]>=0,rpm,driverAcceleration,
                                static_cast<uint32_t>(llabs(goals[id-1])));if(error)return error;
            }
            if(io.cancelled())return "STOPPED";
            error=io.sync();if(error)return error;
            sent=true;
            io.timing(rotating?"TURN_SENT":"STRAIGHT_SENT");
        }
        if(io.now()-lastReport>=200){io.report(rotating?"POSITION_TURN":"POSITION_STRAIGHT",result,targetHeading);lastReport=io.now();}
        // 末端密集查询，避免已停止却还空等原来的40ms调度间隔。
        io.wait(allIdle||maxResidual<=unitsPerDistance*(rotating?2.0:10.0)?1:20);
    }
    return "MOTOR_ARRIVAL_TIMEOUT";
}
const char *align(IO &io,const Settings &settings,const Config &cfg,Result &result,float heading,Context &context) {
    for(int attempt=0;attempt<3;++attempt) {
        H30Attitude m;const char *error=sample(io,m,result.heading);if(error)return error;
        if(fabsf(turn(result.heading,heading))<=settings.headingDeadbandDeg)return nullptr;
        float delta=turn(result.heading,heading);
        for(int i=0;i<=90;++i) {
            error=field::check(result.estimated.x,result.estimated.y,wrap(result.heading+delta*i/90),0);
            if(error)return "TURN_SPACE";
        }
        error=move(io,settings,cfg,result,result.estimated,0,heading,true,context);if(error)return error;
    }
    H30Attitude m;const char *error=sample(io,m,result.heading);if(error)return error;
    return fabsf(turn(result.heading,heading))<=settings.headingDeadbandDeg?nullptr:"TURN_HEADING_ERROR";
}
}
const char *run(const Route &route,const Settings &settings,IO &io,Result &result) {
    if(!isfinite(settings.speedMmS)||settings.speedMmS<10||settings.speedMmS>2000
        ||!isfinite(settings.accelMmS2)||settings.accelMmS2<1||settings.accelMmS2>4000
        ||!isfinite(settings.turnDegS)||settings.turnDegS<10||settings.turnDegS>240
        ||!isfinite(settings.turnAccelDegS2)||settings.turnAccelDegS2<30||settings.turnAccelDegS2>720
        ||!isfinite(settings.headingDeadbandDeg)||settings.headingDeadbandDeg<1||settings.headingDeadbandDeg>8)return "RANGE";
    result.estimated=route.segments[0].start;
    Context context;io.halt();const char *error=quiet(io,result,context);if(error){io.halt();return error;}
    for(size_t i=0;i<route.segmentCount;++i) {
        const Segment &phase=route.segments[i];
        if(phase.kind==CUBIC){io.halt();return "POSITION_ONLY";}
        if(phase.kind==ROTATE)error=align(io,settings,route.config,result,phase.h1,context);
        else {
            // 同向共线节点不各停一次；合并成同一有限脉冲目标。
            Vec end=phase.end;float length=phase.length;
            while(i+1<route.segmentCount) {
                const auto &next=route.segments[i+1];
                if(next.kind!=LINE||fabsf(turn(phase.h0,next.h0))>.01f
                    ||dot(end-phase.start,next.end-next.start)<=0)break;
                end=next.end;length+=next.length;++i;
            }
            error=align(io,settings,route.config,result,phase.h0,context);
            Vec direction=(end-phase.start)*(1/length);
            float sign=dot(direction,Vec(cosf(phase.h0*PI_F/180),sinf(phase.h0*PI_F/180)))>=0?1:-1;
            if(!error)error=move(io,settings,route.config,result,end,sign*length,phase.h0,false,context);
            // 到位后直接使用下一动作的目标航向校角；不要先修旧航向再转弯。
            // 下一段直行前的align、ROTATE或最终align仍只在停车后执行。
        }
        if(error){io.halt();return error;}
    }
    error=align(io,settings,route.config,result,route.config.endHeading,context);
    io.halt();return error;
}
}
