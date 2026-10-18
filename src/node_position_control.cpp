#include "node_position_control.h"
#include "field_geometry.h"
#include <stdlib.h>

namespace node_position {
using namespace continuous;
namespace {
const char *health(const Motor &m) {
    if(!(m.flags&1))return "MOTOR_DISABLED";
    if(m.flags&12)return "MOTOR_STALL";
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
    bool prepared=false,valid=false;uint32_t at=0;Motor stopped[4];
    int encoderSign[4]={}; // 首次有限动作学习各轮反馈符号与指令坐标的对应关系
};
// 两次独立的四轮位置/转速反馈覆盖100ms；不是只等时间就认为停稳。
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
    MotorQuiet motors;uint32_t started=io.now();context.valid=false;
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
        if(motors.update(states,io.now(),100,idle)) {
            for(int i=0;i<4;++i)context.stopped[i]=states[i];
            context.valid=true;context.at=io.now();return nullptr;
        }
        io.wait(5);
    }
    return "MOTOR_STOP_TIMEOUT";
}
// 一次有固定脉冲终点的位置动作。进度只读编码器，时钟只用于加速和超时。
const char *move(IO &io,const Settings &settings,const Config &cfg,Result &result,
                 Vec end,float distance,float targetHeading,bool rotating,Context &context) {
    H30Attitude m;const char *error=sample(io,m,result.heading);if(error)return error;
    float delta=rotating?turn(result.heading,targetHeading):distance;
    float ppm=rotating?cfg.rotationPpd:cfg.forwardPpm;
    uint32_t pulses=static_cast<uint32_t>(lroundf(fabsf(delta)*ppm));
    if(!pulses)return nullptr;
    if(!context.valid||io.now()-context.at>100) {
        error=quiet(io,result,context);if(error)return error;
    }
    if(!context.prepared) {
    // 再检出现瞬时转速时重新确认整组停止，不能凭一次波动拒绝，也不能边转边清零。
    for(int attempt=0;attempt<3;++attempt) {
        bool idle=true;
        for(uint8_t id=1;id<=4;++id) {
            Motor state;error=io.read(id,state);if(error)return error;
            error=health(io,id,state);if(error)return error;
            if(abs(state.rpm)>3)idle=false;
        }
        if(idle)break;
        io.report("POSITION_SETTLE",result,targetHeading);io.halt();
        error=quiet(io,result,context);if(error)return error;
        if(attempt==2)return "MOTOR_STOP_TIMEOUT";
    }
    // 全组停止后才清零，不改变H30地图参考，也不清零5/6号机械臂。
    for(uint8_t id=1;id<=4;++id) {
        Motor state;
        for(int retry=0;;++retry) {
            error=io.read(id,state);if(error)return error;
            error=health(io,id,state);if(error)return error;
            if(abs(state.rpm)<=3)break;
            io.motorDiagnostic(id,"POSITION_SETTLE",0,state);
            io.halt();error=quiet(io,result,context);if(error)return error;
            if(retry>=2)return "MOTOR_STOP_TIMEOUT";
        }
        error=io.zero(id);if(error)return error;
        error=io.read(id,state);if(error)return error;
        if(llabs(state.position)>64)return "MOTOR_ZERO_FAILED";
        context.stopped[id-1]=state;
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
    float speed=0;uint32_t started=io.now(),previous=started,lastReport=0;
    float amount=fabsf(delta);
    uint32_t deadline=static_cast<uint32_t>((amount/maximum+maximum/acceleration+5)*3000);
    bool sent=false,braking=false;
    HeadingGuard headingGuard;
    uint8_t retries=0;uint16_t lastRpm[4]={};H30HeadingHold headingHold;
    MotorQuiet motorQuiet,recoveryQuiet;float startHeading=result.heading;
    const double goal=static_cast<double>(pulses)*65536/PULSES_PER_REV;
    const double unitsPerDistance=ppm*65536/PULSES_PER_REV;
    const double tolerance=unitsPerDistance*(rotating?.25:1.0);
    const double hardLimit=unitsPerDistance*(rotating?3.0:10.0);
    while(io.now()-started<=deadline) {
        uint32_t cycleAt=io.now();float dt=(cycleAt-previous)*.001f;previous=cycleAt;
        error=sample(io,m,result.heading);if(error)return error;
        float headingError=turn(result.heading,targetHeading);
        bool arrived=sent,allIdle=true;double progress=0,maxResidual=0;Motor measured[4];
        for(uint8_t id=1;id<=4;++id) {
            Motor state;error=io.read(id,state);if(error)return error;
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
            if(abs(state.rpm)>3)allIdle=false;
            if(!(state.flags&2)||residual>tolerance||abs(state.rpm)>3)arrived=false;
            progress+=fmax(0,fmin(1,current/goal))/4;
        }
        // 轮询四轮串口有延时，纠偏使用轮询之后的最新IMU快照。
        error=sample(io,m,result.heading);if(error)return error;
        if(rotating&&(m.fields&7)!=7)result.heading=wrap(startHeading+delta*static_cast<float>(progress));
        headingError=turn(result.heading,targetHeading);
        if(!rotating&&headingGuard.update(headingError,m.receivedAtMs,braking,allIdle,(m.fields&7)==7)) {
            io.report("HEADING_ERROR",result,targetHeading);return "HEADING_ERROR";
        }
        if(!rotating) {
            result.estimated=start+(end-start)*static_cast<float>(progress);
            result.s=baseS+amount*progress;
            error=field::check(result.estimated.x,result.estimated.y,result.heading,0);if(error)return error;
        }
        // 四轮反馈到目标、转速近零，且编码器位置稳定100ms，才推进动作。
        if(motorQuiet.update(measured,io.now(),100,arrived)) {
            for(int i=0;i<4;++i)context.stopped[i]=measured[i];
            context.valid=true;context.at=io.now();return nullptr;
        }
        // 到位标志、位置与转速共同确认；小幅超调留给驱动闭环收敛，不立即失败。
        // 静止残差在有限范围内时仅低速重试原绝对目标，最多两次。
        bool retry=false;
        if(recoveryQuiet.update(measured,io.now(),500,sent&&allIdle&&!arrived&&maxResidual<=hardLimit)) {
                if(retries>=2)return "MOTOR_RESIDUAL_TIMEOUT";
                ++retries;retry=true;recoveryQuiet=MotorQuiet{};
                io.report("POSITION_RECOVER",result,targetHeading);
                for(uint8_t id=1;id<=4;++id)io.motorDiagnostic(id,"POSITION_RECOVER",static_cast<double>(goals[id-1])*(context.encoderSign[id-1]?context.encoderSign[id-1]:1)*65536/PULSES_PER_REV,measured[id-1]);
        }
        if(!arrived&&(!sent||!allIdle||retry||maxResidual>hardLimit)) {
            if(dt>.5f)return "CONTROL_OVERRUN";
            // dt含上次串口下发、轮询和等待的完整周期，而非只算空闲部分。
            float remaining=amount*(1-static_cast<float>(progress));
            // 预留完整周期的通信预算；有限位置驱动负责最后几毫米/小角度的准确停靠。
            // 保留低速收尾下限，避免v≈剩余距离/延迟的渐近爬行直到1RPM。
            float latency=clamp(dt,.04f,.25f);
            float finishSpeed=fminf(maximum,rotating?6.0f:25.0f);
            float target=fminf(maximum,fmaxf(finishSpeed,sqrtf(acceleration*acceleration*latency*latency+2*acceleration*fmaxf(0,remaining))-acceleration*latency));
            if(sent&&target<speed)braking=true;
            speed=fminf(target,speed+acceleration*fmaxf(dt,.02f));
            if(retry||retries)speed=fminf(speed,rotating?5.0f:20.0f);
            uint16_t rpm=rpmFor(speed,ppm);
            // 固定四轮有限终点，仅差动调整轮速；绝不以速度模式覆盖位置约束。
            // 转向校正预算不超过平移轮速的25%，末端15mm停止叠加并由align收尾。
            float omega=0;
            if((m.fields&7)==7&&!rotating&&remaining>15&&fabsf(headingError)>.4f) {
                float limit=fminf(8,speed*ppm*.25f/cfg.rotationPpd);
                omega=headingHold.update(result.heading,headingError,m.receivedAtMs,2,limit);
            }
            uint16_t requested[4];bool changed=!sent||retry;
            for(uint8_t id=1;id<=4;++id) {
                bool positive=rotating?delta>=0:((distance>=0)==(id%2==1));
                float correction=omega*cfg.rotationPpd*60/PULSES_PER_REV*(positive?1:-1);
                requested[id-1]=static_cast<uint16_t>(clamp(roundf(rpm+correction),1,rpmFor(maximum,ppm)));
                if(requested[id-1]!=lastRpm[id-1])changed=true;
            }
            if(changed) {
                for(uint8_t id=1;id<=4;++id) {
                    error=io.target(id,goals[id-1]>=0,requested[id-1],static_cast<uint32_t>(llabs(goals[id-1])));if(error)return error;
                    lastRpm[id-1]=requested[id-1];
                }
                if(io.cancelled())return "STOPPED";
                error=io.sync();if(error)return error;sent=true;
            }
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
    if(!isfinite(settings.speedMmS)||settings.speedMmS<10||settings.speedMmS>1600
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
            // 差动轮速仍受原位置终点约束，最终残余航向由有限角度校正收尾。
        }
        if(error){io.halt();return error;}
    }
    error=align(io,settings,route.config,result,route.config.endHeading,context);
    io.halt();return error;
}
}
