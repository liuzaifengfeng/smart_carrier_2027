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
    uint16_t rpm[4]={},stagedRpm[4]={},peak=0;uint8_t lastAcceleration=0;
    bool frozen=false,falseArrived=false,cancel=false,missing=false,reject=false;
    bool disturb=false,injected=false,correctedMoving=false,differential=false;
    bool residualTest=false,residualInjected=false,residualActive=false,hardOverrun=false;
    bool persistentResidual=false;float disturbanceReference=0,disturbanceAngle=3;
    int residualEncoderUnits=6000;
    uint32_t cancelAt=0,missingAt=0;
    uint32_t rippleAt=0,firstZeroAt=0;bool rippleInjected=false;
    bool neverStops=false,badZero=false,stall=false;
    uint8_t stallMotor=4,stallFlags=133;
    int startupQueries=0;bool targetSent=false;
    uint32_t latencyMs=0;
    uint32_t flagDelayMs=0,rpmTailMs=0,doneAt[4]={};
    int sentMarks=0,arrivalMarks=0,waitReports=0;
    int recoveries=0,diagnostics=0;
    int actions=0,updates=0,syncs=0,halts=0,zeroGroups=0;float heading=0;Vec location={310,1200};
    uint32_t arrivalAt=0,maxGap=0;bool wasMoving=false,newAction=false;
    int encoderPolarity[4]={1,1,1,1};
    bool velocityMode=false;int16_t velocityRpm[4]={};int velocityUpdates=0,curveReports=0;
    uint32_t now()override{return time;}
    bool cancelled()override{return cancel||(cancelAt&&time>=cancelAt);}
    void wait(uint32_t ms)override {
        double pulses[4]={};
        for(int i=0;i<4;++i) {
            double amount=rpm[i]*PULSES_PER_REV*ms/60000.0;
            double delta=fmax(-amount,fmin(amount,goal[i]-actual[i]));
            if(velocityMode)delta=velocityRpm[i]*PULSES_PER_REV*ms/60000.0;
            if(frozen)delta=0;
            actual[i]+=delta;pulses[i]=delta;
        }
        float angle=(pulses[0]+pulses[1]+pulses[2]+pulses[3])/(4*51.8f);
        Vec body={static_cast<float>((-pulses[0]-pulses[1]+pulses[2]+pulses[3])/(4*13.3)),
                  static_cast<float>((pulses[0]-pulses[1]+pulses[2]-pulses[3])/(4*13.6))};
        location=location+bodyToWorld(body,heading+angle/2);heading=wrap(heading+angle);time+=ms;
        bool moving=false;for(int i=0;i<4;++i)if(fabs(goal[i]-actual[i])>.001)moving=true;
        if(wasMoving&&!moving)arrivalAt=time;wasMoving=moving;
        if(disturb&&!injected&&location.x>500){heading=wrap(heading+disturbanceAngle);injected=true;}
        if(injected&&location.x>550&&location.x<1000&&fabsf(turn(heading,disturbanceReference))<1)correctedMoving=true;
    }
    const char *imu(H30Attitude &sample,float &value)override {
        if(missing||(missingAt&&time>=missingAt))return "MOTION_FIELDS_REQUIRED";
        sample.fields=7;sample.sequence=time/10;sample.receivedAtMs=time;sample.accel[2]=9.78f;value=heading;
        return nullptr;
    }
    const char *read(uint8_t id,Motor &m)override {
        if(!targetSent)startupQueries+=3;
        int i=id-1;wait(1+3*latencyMs);bool done=fabs(goal[i]-actual[i])<.001;
        m.position=llround(actual[i]*65536/PULSES_PER_REV);
        if(residualTest&&done&&pending[i]!=0&&!residualInjected){residualInjected=true;residualActive=true;}
        if(done&&pending[i]!=0&&residualActive)m.position+=(actual[i]>=0?1:-1)*(hardOverrun?12000:residualEncoderUnits);
        m.position*=encoderPolarity[i];
        if(done&&pending[i]!=0&&!doneAt[i])doneAt[i]=time;
        bool flagReady=done&&(!flagDelayMs||time-doneAt[i]>=flagDelayMs);
        m.flags=1|((flagReady||falseArrived)?2:0);m.rpm=(done||falseArrived)?0:rpm[i];
        if(done&&pending[i]!=0&&time-doneAt[i]<rpmTailMs)m.rpm=6;
        if(stall&&actions>0&&id==stallMotor)m.flags=stallFlags;
        if(neverStops){m.rpm=6;m.position+=time*10;}
        if(rippleAt&&time>=rippleAt&&!rippleInjected&&actions==0){m.rpm=6;rippleInjected=true;}
        return nullptr;
    }
    const char *readPosition(uint8_t id,int64_t &position)override {
        if(!targetSent)++startupQueries;
        wait(1+latencyMs);
        position=badZero?100:llround(actual[id-1]*65536/PULSES_PER_REV)*encoderPolarity[id-1];
        return nullptr;
    }
    const char *zero(uint8_t id)override {
        int i=id-1;CHECK(fabs(goal[i]-actual[i])<.01);actual[i]=goal[i]=pending[i]=0;
        if(id==1){if(!firstZeroAt)firstZeroAt=time;++zeroGroups;}return nullptr;
    }
    const char *target(uint8_t id,bool positive,uint16_t speed,uint8_t acceleration,uint32_t count)override {
        CHECK(acceleration>0); // 每条位置命令必须携带驱动板加减速曲线档位
        targetSent=true;
        if(reject)return "MOTOR_COMMAND_REJECTED";
        if(latencyMs)wait(latencyMs); // ACK后硬件适配器不再固定等待5ms
        int i=id-1;double next=positive?count:-double(count);
        CHECK(pending[i]==0); // 同一动作禁止重复装载位置指令，含低速补位重发
        pending[i]=next;stagedRpm[i]=speed;lastAcceleration=acceleration;peak=fmaxf(peak,speed);++updates;return nullptr;
    }
    const char *sync()override {
        velocityMode=false;
        if(newAction){if(arrivalAt&&time-arrivalAt>maxGap)maxGap=time-arrivalAt;newAction=false;}
        if(residualActive&&!persistentResidual)residualActive=false;
        if(stagedRpm[0]!=stagedRpm[1])differential=true;
        for(int i=0;i<4;++i){goal[i]=pending[i];rpm[i]=stagedRpm[i];}++syncs;return nullptr;
    }
    const char *velocity(const WheelCommand &command)override {
        if(cancelled())return "STOPPED";
        if(reject)return "MOTOR_COMMAND_REJECTED";
        velocityMode=true;targetSent=true;++velocityUpdates;
        for(int i=0;i<4;++i){velocityRpm[i]=command.rpm[i];rpm[i]=abs(command.rpm[i]);}
        return nullptr;
    }
    void beginAction()override{
        // 新动作（包括停止后校角）不能在上一动作仍运动时插入。
        for(int i=0;i<4;++i)CHECK(fabs(goal[i]-actual[i])<.01);
        ++actions;newAction=true;for(int i=0;i<4;++i){pending[i]=0;doneAt[i]=0;}
    }
    void timing(const char *stage)override {
        if(strstr(stage,"_SENT"))++sentMarks;
        if(strstr(stage,"_ARRIVED"))++arrivalMarks;
    }
    void arrivalDiagnostic(float,const Motor (&m)[4])override {
        for(int i=0;i<4;++i)CHECK(abs(m[i].rpm)<=3);
        ++waitReports;
    }
    void halt()override {velocityMode=false;for(int i=0;i<4;++i){goal[i]=actual[i];pending[i]=0;rpm[i]=0;}++halts;}
    void report(const char *stage,const Result&,float)override{
        if(!strcmp(stage,"POSITION_RECOVER"))++recoveries;
        if(!strcmp(stage,"OUTER_CURVE"))++curveReports;
    }
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
    CHECK(driverAccelerationFor(1600,13.6f)==207);
    CHECK(driverAccelerationFor(360,51.8f)==199);
    CHECK(driverAccelerationFor(1,13.6f)==1);
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
    CHECK(!route.build(nodes,cfg));Settings settings;CHECK(settings.speedMmS==2000&&settings.accelMmS2==2000&&settings.turnDegS==200&&settings.turnAccelDegS2==400);
    settings.speedMmS=200;settings.accelMmS2=200;settings.turnDegS=20;settings.turnAccelDegS2=30;Result result;Sim sim;
    const char *error=run(route,settings,sim,result);if(error)fprintf(stderr,"run: %s\n",error);CHECK(!error);
    CHECK(sim.actions==3); // 两条直线（共线节点合并）+一次转向
    CHECK(sim.zeroGroups==1&&sim.halts==2); // 仅起步清零一次，动作间不重复halt/清零
    CHECK(sim.maxGap<100); // 到位后不附加100ms窗口或重复整组查询
    CHECK(sim.startupQueries==16&&sim.firstZeroAt==1004); // 一组12次查询+4次仅位置验证
    CHECK(sim.updates==sim.actions*4&&sim.syncs==sim.actions);CHECK(sim.peak==51);
    CHECK(sim.sentMarks==sim.actions&&sim.arrivalMarks==sim.actions&&sim.waitReports==0);
    CHECK(sim.lastAcceleration==1); // 200mm/s²低于驱动器最小非零档，使用最慢曲线档1
    CHECK(norm(sim.location-Vec(1200,1680))<2);CHECK(fabsf(turn(sim.heading,90))<=1);
    CHECK(norm(result.estimated-Vec(1200,1680))<1);
    Sim delayed;delayed.flagDelayMs=60000; // 到位标志整条路径均未置位
    error=run(route,settings,delayed,result);CHECK(!error);
    CHECK(delayed.waitReports==0&&delayed.time==sim.time); // 实际位置/转速到位即接续
    CHECK(delayed.updates==delayed.actions*4&&delayed.syncs==delayed.actions);
    Sim coasting;coasting.rpmTailMs=300;
    error=run(route,settings,coasting,result);CHECK(!error);
    CHECK(coasting.time>sim.time+750); // 位置已到但仍有转速，不能提前转向
    Sim ripple;ripple.rippleAt=1000;
    error=run(route,settings,ripple,result);CHECK(!error&&ripple.rippleInjected&&ripple.actions==3);
    CHECK(ripple.firstZeroAt>sim.firstZeroAt); // 起步6RPM波动仍重新读近零转速
    Sim zeroFailed;zeroFailed.badZero=true;
    error=run(route,settings,zeroFailed,result);
    CHECK(error&&!strcmp(error,"MOTOR_ZERO_FAILED")&&zeroFailed.updates==0);
    Sim moving;moving.neverStops=true;
    error=run(route,settings,moving,result);CHECK(error&&!strcmp(error,"MOTOR_STOP_TIMEOUT")&&moving.actions==0);
    Sim disturbed;disturbed.disturb=true;disturbed.disturbanceAngle=5;
    error=run(route,settings,disturbed,result);if(error)fprintf(stderr,"disturb: %s\n",error);CHECK(!error);
    CHECK(disturbed.injected&&!disturbed.differential&&!disturbed.correctedMoving&&disturbed.actions==3);
    CHECK(fabsf(turn(disturbed.heading,90))<=1);
    Sim fast;fast.disturb=true;fast.disturbanceAngle=5;Settings fastSettings=settings;fastSettings.speedMmS=400;
    error=run(route,fastSettings,fast,result);CHECK(!error);
    CHECK(!fast.correctedMoving&&!fast.differential&&fast.actions==3&&fast.peak<=102);
    // 直行允许每轮5000编码器单位误差，覆盖实车1181及正负边界；转向门槛独立。
    Config straightCfg=cfg;straightCfg.count=3;straightCfg.endHeading=0;
    Route straight;CHECK(!straight.build(nodes,straightCfg));
    const int acceptedResiduals[]={1181,5000,-1181,-5000};
    for(int units:acceptedResiduals) {
        Sim accepted;accepted.residualTest=true;accepted.residualEncoderUnits=units;
        error=run(straight,settings,accepted,result);
        if(error||accepted.actions!=1)fprintf(stderr,"residual %d: %s, actions %d\n",units,error?error:"OK",accepted.actions);
        CHECK(!error&&accepted.actions==1&&accepted.updates==4&&accepted.syncs==1);
        CHECK(accepted.waitReports==0);
    }
    Sim outside;outside.residualTest=true;outside.residualEncoderUnits=5001;
    error=run(straight,settings,outside,result);
    CHECK(error&&!strcmp(error,"MOTOR_RESIDUAL_TIMEOUT")&&outside.actions==1);
    Sim residual;residual.residualTest=true;
    error=run(route,settings,residual,result);
    CHECK(error&&!strcmp(error,"MOTOR_RESIDUAL_TIMEOUT"));
    CHECK(residual.recoveries==0&&residual.updates==4&&residual.syncs==1); // 残差超时也不补位重发
    Sim persistent;persistent.residualTest=true;persistent.persistentResidual=true;
    error=run(route,settings,persistent,result);
    CHECK(error&&!strcmp(error,"MOTOR_RESIDUAL_TIMEOUT")&&persistent.recoveries==0&&persistent.actions==1&&persistent.updates==4);
    Sim overrun;overrun.residualTest=true;overrun.hardOverrun=true;
    error=run(route,settings,overrun,result);
    CHECK(error&&!strcmp(error,"MOTOR_POSITION_OVERRUN")&&overrun.diagnostics==1);
    Sim reverse;reverse.heading=180;
    cfg.startHeading=180;cfg.endHeading=180;cfg.count=3;
    CHECK(!route.build(nodes,cfg));
    error=run(route,settings,reverse,result);CHECK(!error);
    CHECK(reverse.actions==1); // 保持车头，倒退通过共线节点
    CHECK(norm(reverse.location-Vec(1200,1200))<2);
    Sim reverseDisturbed;reverseDisturbed.heading=180;reverseDisturbed.disturb=true;reverseDisturbed.disturbanceReference=180;reverseDisturbed.disturbanceAngle=5;
    error=run(route,fastSettings,reverseDisturbed,result);CHECK(!error);
    CHECK(!reverseDisturbed.correctedMoving&&!reverseDisturbed.differential&&reverseDisturbed.actions==2);
    CHECK(fabsf(turn(reverseDisturbed.heading,180))<=fastSettings.headingDeadbandDeg);
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
    // 四轮的两类堵转位均不再触发固件中止，其他到位条件仍须满足。
    for(uint8_t id=1;id<=4;++id)for(uint8_t flags:{uint8_t(133),uint8_t(137),uint8_t(141)}) {
        Sim stalled;stalled.stall=true;stalled.stallMotor=id;stalled.stallFlags=flags;
        error=run(route,settings,stalled,result);
        CHECK(!error&&stalled.diagnostics==0&&stalled.actions==3);
    }
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
    CHECK(upgraded.peak<=node_position::rpmFor(2000,13.6f));
    CHECK(norm(upgraded.location-Vec(2090,1200))<3&&fabsf(turn(upgraded.heading,90))<=1);
    Sim opposite;opposite.location={1200,240};opposite.heading=180;opposite.encoderPolarity[0]=opposite.encoderPolarity[2]=-1;
    error=run(route,newProfile,opposite,result);CHECK(!error&&opposite.zeroGroups==1);
    CHECK(norm(opposite.location-Vec(2090,1200))<3);
    printf("serial model max transition gap=%ums\n",upgraded.maxGap);
    CHECK(upgraded.zeroGroups==1&&upgraded.maxGap<=120); // 逐轮反馈跨越到位时刻可再占一轮查询，加上目标ACK
    Node shortNodes[]={{{310,1200},0},{{410,1200},0}};cfg.count=2;cfg.startHeading=0;cfg.endHeading=0;
    CHECK(!route.build(shortNodes,cfg));
    Sim tolerant;tolerant.heading=2;error=run(route,settings,tolerant,result);CHECK(!error&&tolerant.actions==1);
    Settings tight=settings;tight.headingDeadbandDeg=1;Sim precise;precise.heading=2;
    error=run(route,tight,precise,result);CHECK(!error&&precise.actions>tolerant.actions);
    Node reversal[]={{{310,1200},0},{{720,1200},0},{{310,1200},0},{{720,1200},0}};
    cfg.count=4;CHECK(!route.build(reversal,cfg));Sim returning;
    error=run(route,settings,returning,result);CHECK(!error&&returning.zeroGroups==1&&returning.actions==3);
    CHECK(norm(returning.location-Vec(720,1200))<2);
    for(int field=0;field<4;++field) {
        Settings invalid;float *values[]={&invalid.speedMmS,&invalid.accelMmS2,&invalid.turnDegS,&invalid.turnAccelDegS2};
        const float maxima[]={2000,4000,240,720};*values[field]=maxima[field]+1;
        Sim untouched;error=run(route,invalid,untouched,result);
        CHECK(error&&!strcmp(error,"RANGE")&&untouched.syncs==0);
    }
    printf("same-route model: old profile %lums, new profile %lums\n",(unsigned long)(previous.time-1000),(unsigned long)(upgraded.time-1000));
    // 正式外圈2-3-4-5-14：4号角连续外绕，块内不插入位置动作或停车。
    Config mixed=cfg;mixed.policy=3;mixed.count=5;mixed.startHeading=180;mixed.endHeading=90;
    mixed.maxRpm=rpmFor(newProfile.speedMmS,13.6f);mixed.accel=newProfile.accelMmS2;
    Route outer;CHECK(!outer.build(benchNodes,mixed));bool hasCurve=false;
    for(size_t i=0;i<outer.segmentCount;++i)hasCurve|=outer.segments[i].kind==CUBIC;CHECK(hasCurve);
    Sim drift;drift.location={1200,240};drift.heading=180;
    error=run(outer,newProfile,drift,result);
    if(error)fprintf(stderr,"outer: %s\n",error);CHECK(!error);
    printf("outer end %.2f %.2f heading %.2f, actions=%d velocity=%d\n",drift.location.x,drift.location.y,drift.heading,drift.actions,drift.velocityUpdates);
    CHECK(drift.velocityUpdates>0&&drift.curveReports>0&&drift.updates==0&&drift.actions==1&&drift.halts==3);
    CHECK(norm(drift.location-Vec(2090,1200))<15&&fabsf(turn(drift.heading,90))<=newProfile.headingDeadbandDeg);
    Sim noImu;noImu.location={1200,240};noImu.heading=180;noImu.missing=true;
    noImu.encoderPolarity[0]=noImu.encoderPolarity[3]=-1;
    Result noImuResult;noImuResult.heading=180;
    error=run(outer,newProfile,noImu,noImuResult);CHECK(!error&&noImu.velocityUpdates>0);
    CHECK(norm(noImu.location-Vec(2090,1200))<15);
    Sim cancelCurve;cancelCurve.location={1200,240};cancelCurve.heading=180;cancelCurve.cancelAt=3000;
    error=run(outer,newProfile,cancelCurve,result);CHECK(error&&!strcmp(error,"STOPPED")&&!cancelCurve.velocityMode);
    Node enterCenter[]={{{1200,240},0},{{1680,240},0},{{2090,240},0},{{2090,720},0},
                         {{2090,1200},0},{{1680,1200},0},{{1200,1200},0}};
    mixed.count=7;mixed.endHeading=180;Route entering;
    CHECK(!entering.build(enterCenter,mixed));Sim enter;enter.location={1200,240};enter.heading=180;
    enter.encoderPolarity[1]=enter.encoderPolarity[3]=-1;
    error=run(entering,newProfile,enter,result);if(error)fprintf(stderr,"mixed: %s\n",error);CHECK(!error);
    CHECK(enter.velocityUpdates>0&&enter.updates==8&&enter.actions==3);
    CHECK(norm(enter.location-Vec(1200,1200))<15);
    Node upperLeft[]={{{310,1200},0},{{310,1680},0},{{310,2020},0},{{720,2020},0},{{1200,2020},0}};
    mixed.count=5;mixed.startHeading=180.855f;mixed.endHeading=180;Route aroundLeft;
    CHECK(!aroundLeft.build(upperLeft,mixed));Sim left;left.heading=180.855f;
    error=run(aroundLeft,newProfile,left,result);if(error)fprintf(stderr,"upper left: %s\n",error);CHECK(!error);
    CHECK(left.velocityUpdates>0&&left.updates==4&&norm(left.location-Vec(1200,2020))<15);
    // 中间十字11-12-17：新旧规划的分段与朝向一致，依旧纯位置模式。
    Node crossNodes[]={{{720,1200},0},{{1200,1200},0},{{1200,1680},0}};
    mixed.count=3;mixed.startHeading=0;mixed.endHeading=90;Route center,oldCenter;
    CHECK(!center.build(crossNodes,mixed));Config stoppedConfig=mixed;stoppedConfig.policy=2;
    CHECK(!oldCenter.build(crossNodes,stoppedConfig)&&center.segmentCount==oldCenter.segmentCount);
    for(size_t i=0;i<center.segmentCount;++i) {
        CHECK(center.segments[i].kind==oldCenter.segments[i].kind&&center.segments[i].kind!=CUBIC);
        CHECK(center.segments[i].h0==oldCenter.segments[i].h0&&center.segments[i].h1==oldCenter.segments[i].h1);
    }
    Sim crossing;crossing.location={720,1200};error=run(center,newProfile,crossing,result);
    CHECK(!error&&crossing.velocityUpdates==0&&crossing.actions==3);
    puts("fixed position targets, feedback gating, units, merging and faults OK");return 0;
}
