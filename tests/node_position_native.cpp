#include "../src/node_position_control.h"
#include "../src/emm_feedback.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
using namespace continuous;
using namespace node_position;
#define CHECK(x) do{if(!(x)){fprintf(stderr,"position line %d: %s\n",__LINE__,#x);exit(2);}}while(0)
static Route route;
struct Sim : IO {
    uint32_t time=1000;double actual[4]={},goal[4]={},pending[4]={};
    uint16_t rpm[4]={},stagedRpm[4]={},peak=0;
    bool frozen=false,falseArrived=false,cancel=false,missing=false,reject=false;
    bool disturb=false,injected=false,correctedMoving=false,differential=false;
    bool residualTest=false,residualInjected=false,residualActive=false,hardOverrun=false;
    bool persistentResidual=false;float disturbanceReference=0,disturbanceAngle=3;
    uint32_t cancelAt=0,missingAt=0;
    uint32_t rippleAt=0,firstZeroAt=0;bool rippleInjected=false;
    bool neverStops=false;
    uint32_t latencyMs=0;
    int recoveries=0,diagnostics=0;
    int actions=0,updates=0,syncs=0,halts=0;float heading=0;Vec location={310,1200};
    uint32_t now()override{return time;}
    bool cancelled()override{return cancel||(cancelAt&&time>=cancelAt);}
    void wait(uint32_t ms)override {
        double pulses[4]={};
        for(int i=0;i<4;++i) {
            double amount=rpm[i]*PULSES_PER_REV*ms/60000.0;
            double delta=fmax(-amount,fmin(amount,goal[i]-actual[i]));
            if(frozen)delta=0;
            actual[i]+=delta;pulses[i]=delta;
        }
        float angle=(pulses[0]+pulses[1]+pulses[2]+pulses[3])/(4*51.8f);
        Vec body={static_cast<float>((-pulses[0]-pulses[1]+pulses[2]+pulses[3])/(4*13.3)),
                  static_cast<float>((pulses[0]-pulses[1]+pulses[2]-pulses[3])/(4*13.6))};
        location=location+bodyToWorld(body,heading+angle/2);heading=wrap(heading+angle);time+=ms;
        if(disturb&&!injected&&location.x>500){heading=wrap(heading+disturbanceAngle);injected=true;}
        if(injected&&location.x>550&&location.x<1000&&fabsf(turn(heading,disturbanceReference))<1)correctedMoving=true;
    }
    const char *imu(H30Attitude &sample,float &value)override {
        if(missing||(missingAt&&time>=missingAt))return "MOTION_FIELDS_REQUIRED";
        sample.fields=7;sample.sequence=time/10;sample.receivedAtMs=time;sample.accel[2]=9.78f;value=heading;
        return nullptr;
    }
    const char *read(uint8_t id,Motor &m)override {
        int i=id-1;wait(1+3*latencyMs);bool done=fabs(goal[i]-actual[i])<.001;
        m.position=llround(actual[i]*65536/PULSES_PER_REV);
        if(residualTest&&done&&pending[i]!=0&&!residualInjected){residualInjected=true;residualActive=true;}
        if(done&&pending[i]!=0&&residualActive)m.position+=(actual[i]>=0?1:-1)*(hardOverrun?5000:410);
        m.flags=1|((done||falseArrived)?2:0);m.rpm=(done||falseArrived)?0:rpm[i];
        if(neverStops){m.rpm=2;m.position+=time*10;}
        if(rippleAt&&time>=rippleAt&&!rippleInjected&&actions==0){m.rpm=6;rippleInjected=true;}
        return nullptr;
    }
    const char *zero(uint8_t id)override {
        int i=id-1;CHECK(fabs(goal[i]-actual[i])<.01);actual[i]=goal[i]=pending[i]=0;
        if(id==1){if(!firstZeroAt)firstZeroAt=time;++actions;}return nullptr;
    }
    const char *target(uint8_t id,bool positive,uint16_t speed,uint32_t count)override {
        if(reject)return "MOTOR_COMMAND_REJECTED";
        if(latencyMs)wait(5+latencyMs);
        int i=id-1;double next=positive?count:-double(count);
        if(pending[i]!=0)CHECK(next==pending[i]); // 速度更新不能增加目标距离
        pending[i]=next;stagedRpm[i]=speed;peak=fmaxf(peak,speed);++updates;return nullptr;
    }
    const char *sync()override {
        if(residualActive&&!persistentResidual)residualActive=false;
        if(stagedRpm[0]!=stagedRpm[1])differential=true;
        for(int i=0;i<4;++i){goal[i]=pending[i];rpm[i]=stagedRpm[i];}++syncs;return nullptr;
    }
    void halt()override {for(int i=0;i<4;++i){goal[i]=actual[i];pending[i]=0;rpm[i]=0;}++halts;}
    void report(const char *stage,const Result&,float)override{if(!strcmp(stage,"POSITION_RECOVER"))++recoveries;}
    void motorDiagnostic(uint8_t,const char*,double,const Motor&)override{++diagnostics;}
};
int main() {
    HeadingGuard guard;
    CHECK(!guard.update(12,1000,false,false));CHECK(!guard.update(12,2000,false,false));
    CHECK(!guard.update(18,2100,false,false));CHECK(!guard.update(18,2899,false,false));
    CHECK(guard.update(18,2900,false,false));
    guard=HeadingGuard{};
    CHECK(!guard.update(20,1000,true,false));CHECK(!guard.update(20,2199,true,false));
    CHECK(!guard.update(20,2200,true,false));CHECK(!guard.update(20,2999,true,false));
    CHECK(guard.update(20,3000,true,false));
    guard=HeadingGuard{};
    CHECK(!guard.update(20,1000,true,false));CHECK(!guard.update(20,1800,true,true));
    CHECK(!guard.update(20,3000,true,true)); // 停稳小偏角交由有限角度修正，不触发持续偏航
    CHECK(!guard.update(35,3100,true,true));CHECK(guard.update(35,3300,true,true));
    CHECK(!guard.update(35,3500,true,true,false)); // IMU降级不累计保护计时
    CHECK(rpmFor(200,13.6f)==51);CHECK(rpmFor(400,13.6f)==102);
    EmmFeedbackParser parser;EmmReply reply;
    const uint8_t state[]={1,0x3A,3,0x6B};
    for(int i=0;i<3;++i)CHECK(!parser.feed(state[i],reply));CHECK(parser.feed(state[3],reply));
    CHECK(reply.id==1&&reply.data[2]==3);
    const uint8_t pos[]={2,0x36,1,0,1,0,0,0x6B};
    for(int i=0;i<7;++i)CHECK(!parser.feed(pos[i],reply));CHECK(parser.feed(pos[7],reply));
    CHECK(EmmFeedbackParser::position(reply)==-65536);
    const uint8_t velocity[]={2,0x35,0,0,0x6B,0x6B}; // 载荷中的6B不是帧结束
    for(int i=0;i<5;++i)CHECK(!parser.feed(velocity[i],reply));CHECK(parser.feed(velocity[5],reply));
    CHECK(EmmFeedbackParser::rpm(reply)==107);
    Config cfg;cfg.policy=2;cfg.count=4;cfg.startHeading=0;cfg.endHeading=90;cfg.forwardPpm=13.6f;
    Node nodes[]={{{310,1200},0},{{720,1200},0},{{1200,1200},0},{{1200,1680},0}};
    CHECK(!route.build(nodes,cfg));Settings settings;CHECK(settings.speedMmS==1200&&settings.accelMmS2==1600&&settings.turnDegS==150&&settings.turnAccelDegS2==360);
    settings.speedMmS=200;settings.accelMmS2=200;settings.turnDegS=20;settings.turnAccelDegS2=30;Result result;Sim sim;
    const char *error=run(route,settings,sim,result);if(error)fprintf(stderr,"run: %s\n",error);CHECK(!error);
    CHECK(sim.actions==3); // 两条直线（共线节点合并）+一次转向
    CHECK(sim.updates>sim.actions*4);CHECK(sim.peak<=51);
    CHECK(norm(sim.location-Vec(1200,1680))<2);CHECK(fabsf(turn(sim.heading,90))<=1);
    CHECK(norm(result.estimated-Vec(1200,1680))<1);
    Sim ripple;ripple.rippleAt=sim.firstZeroAt;
    error=run(route,settings,ripple,result);CHECK(!error&&ripple.rippleInjected&&ripple.actions==3);
    CHECK(ripple.firstZeroAt>sim.firstZeroAt); // 起步再检6RPM波动：重等稳定而非直接报错
    Sim moving;moving.neverStops=true;
    error=run(route,settings,moving,result);CHECK(error&&!strcmp(error,"MOTOR_STOP_TIMEOUT")&&moving.actions==0);
    Sim disturbed;disturbed.disturb=true;
    error=run(route,settings,disturbed,result);if(error)fprintf(stderr,"disturb: %s\n",error);CHECK(!error);
    CHECK(disturbed.injected&&disturbed.differential&&disturbed.correctedMoving);
    CHECK(fabsf(turn(disturbed.heading,90))<=1);
    Sim fast;fast.disturb=true;Settings fastSettings=settings;fastSettings.speedMmS=400;
    error=run(route,fastSettings,fast,result);CHECK(!error);
    CHECK(fast.correctedMoving&&fast.differential&&fast.peak<=102);
    Sim residual;residual.residualTest=true;
    error=run(route,settings,residual,result);CHECK(!error);
    CHECK(residual.recoveries>=1); // ~1.47mm静止残差不是立刻判超限
    Sim persistent;persistent.residualTest=true;persistent.persistentResidual=true;
    error=run(route,settings,persistent,result);
    CHECK(error&&!strcmp(error,"MOTOR_RESIDUAL_TIMEOUT")&&persistent.recoveries==2&&persistent.actions==1);
    Sim overrun;overrun.residualTest=true;overrun.hardOverrun=true;
    error=run(route,settings,overrun,result);
    CHECK(error&&!strcmp(error,"MOTOR_POSITION_OVERRUN")&&overrun.diagnostics==1);
    Sim reverse;reverse.heading=180;
    cfg.startHeading=180;cfg.endHeading=180;cfg.count=3;
    CHECK(!route.build(nodes,cfg));
    error=run(route,settings,reverse,result);CHECK(!error);
    CHECK(reverse.actions==1); // 保持车头，倒退通过共线节点
    CHECK(norm(reverse.location-Vec(1200,1200))<2);
    Sim reverseDisturbed;reverseDisturbed.heading=180;reverseDisturbed.disturb=true;reverseDisturbed.disturbanceReference=180;
    error=run(route,fastSettings,reverseDisturbed,result);CHECK(!error);
    CHECK(reverseDisturbed.correctedMoving&&fabsf(turn(reverseDisturbed.heading,180))<=1);
    cfg.startHeading=0;cfg.endHeading=90;cfg.count=4;
    CHECK(!route.build(nodes,cfg));
    Sim excessive;excessive.disturb=true;excessive.disturbanceAngle=35;
    error=run(route,settings,excessive,result);
    CHECK(error&&(!strcmp(error,"HEADING_ERROR")||!strcmp(error,"OBSTACLE")||!strcmp(error,"BOUNDARY"))&&excessive.actions==1);
    Sim stopDuring;stopDuring.disturb=true;stopDuring.cancelAt=3500;
    error=run(route,settings,stopDuring,result);CHECK(error&&!strcmp(error,"STOPPED")&&stopDuring.actions==1);
    Sim lostDuring;lostDuring.disturb=true;lostDuring.missingAt=3500;
    error=run(route,settings,lostDuring,result);CHECK(!error&&lostDuring.actions>=3);
    Sim blocked;blocked.frozen=true;blocked.falseArrived=true;
    error=run(route,settings,blocked,result);CHECK(error&&strcmp(error,"MOTOR_ARRIVAL_TIMEOUT")==0);
    CHECK(blocked.actions==1&&blocked.halts>=2);CHECK(norm(blocked.location-Vec(310,1200))==0);
    Sim missing;missing.missing=true;error=run(route,settings,missing,result);
    CHECK(!error&&missing.actions==3); // H30离线仍按有限脉冲/编码器完成，不伪造IMU到位
    Sim rejected;rejected.reject=true;error=run(route,settings,rejected,result);
    CHECK(error&&strcmp(error,"MOTOR_COMMAND_REJECTED")==0&&rejected.syncs==0);
    Sim cancelled;cancelled.cancel=true;error=run(route,settings,cancelled,result);
    CHECK(error&&strcmp(error,"STOPPED")==0&&cancelled.actions==0);
    // 与实车日志相同的1850mm+90度路线，加上串口查询/发令耗时作配置对比。
    Node benchNodes[]={{{1200,240},0},{{1680,240},0},{{2090,240},0},{{2090,720},0},{{2090,1200},0}};
    cfg.count=5;cfg.startHeading=180;cfg.endHeading=90;
    CHECK(!route.build(benchNodes,cfg));
    Settings oldProfile=settings;oldProfile.speedMmS=400;
    Sim previous,upgraded;previous.location=upgraded.location={1200,240};previous.heading=upgraded.heading=180;
    previous.latencyMs=upgraded.latencyMs=4;
    error=run(route,oldProfile,previous,result);CHECK(!error);
    Settings newProfile;error=run(route,newProfile,upgraded,result);if(error)fprintf(stderr,"bench: %s\n",error);CHECK(!error);
    CHECK(upgraded.time<previous.time*.75f);
    CHECK(upgraded.peak<=node_position::rpmFor(1200,13.6f));
    CHECK(norm(upgraded.location-Vec(2090,1200))<3&&fabsf(turn(upgraded.heading,90))<=1);
    for(int field=0;field<4;++field) {
        Settings invalid;float *values[]={&invalid.speedMmS,&invalid.accelMmS2,&invalid.turnDegS,&invalid.turnAccelDegS2};
        const float maxima[]={1600,4000,240,720};*values[field]=maxima[field]+1;
        Sim untouched;error=run(route,invalid,untouched,result);
        CHECK(error&&!strcmp(error,"RANGE")&&untouched.syncs==0);
    }
    printf("same-route model: old profile %lums, new profile %lums\n",(unsigned long)(previous.time-1000),(unsigned long)(upgraded.time-1000));
    puts("fixed position targets, feedback gating, units, merging and faults OK");return 0;
}
