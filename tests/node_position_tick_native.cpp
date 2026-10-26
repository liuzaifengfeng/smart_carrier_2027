#include "node_position_sim.h"
#include "motion_sequence.h"
#include "motion_job.h"
#include <assert.h>
#include <vector>
struct ProfileSim : Sim {
    std::vector<uint16_t> speeds;
    std::vector<uint8_t> accelerations;
    int stableReports=0;
    void headingStable(float)override{++stableReports;}
    const char *target(uint8_t id,bool positive,uint16_t speed,uint8_t acceleration,uint32_t count)override {
        if(id==1){speeds.push_back(speed);accelerations.push_back(acceleration);}
        return Sim::target(id,positive,speed,acceleration,count);
    }
};
struct ArmClock : motion::SequenceIO {
    Sim &base;int issued=0;
    explicit ArmClock(Sim &clock):base(clock){}
    uint32_t now()override{return base.now();}
    bool cancelled()override{return base.cancelled();}
    const char *issue(const motion::Step &)override{++issued;return nullptr;}
    const char *ready(const motion::Step &,bool &ready)override{ready=false;return nullptr;}
    bool feedback(const motion::Step &)override{return false;}
    void event(size_t,const motion::Step &,bool)override{}
};
void nodeTick() {
    Config cfg;cfg.policy=2;cfg.count=3;cfg.startHeading=0;cfg.endHeading=90;
    Node points[]={{{310,1200},0},{{720,1200},0},{{1200,1200},0}};
    Route path;assert(!path.build(points,cfg));
    Settings settings;settings.speedMmS=400;Sim base;Result result;
    Runner chassis;chassis.begin(path,settings,base,result);
    ArmClock io(base);motion::Program program;motion::Step move;move.kind=motion::StepKind::ARM;move.waitMs=1000;program.append(move);
    motion::Sequence arm;arm.begin(program,io);
    motion::Join joined;int ticks=0,starts=0,progressWhileArmWait=0;
    while(chassis.active()||arm.active()) {
        const float before=result.s;chassis.tick();arm.tick();
        if(arm.active()&&result.s>before)++progressWhileArmWait;
        starts+=joined.shouldStartArm(!chassis.active(),chassis.observationApproach());
        base.wait(1);assert(++ticks<100000);
    }
    assert(!chassis.failure()&&!arm.failure()&&io.issued==1&&starts==1&&progressWhileArmWait>0);
    assert(base.zeroGroups==1&&base.actions==2); // 两条共线段合并，一次终点转向。
    assert(norm(result.estimated-Vec(1200,1200))<3);
    // 最终直线无论长短，都在它前面的转向阶段准备，不再等待距离阈值。
    for(bool explicitTurn:{false,true})for(float endY:{1300.0f,2020.0f}) {
        Node cornerPoints[]={{{310,1200},0},{{2100,1200},0},{{2100,endY},0}};
        Config cornerConfig=cfg;cornerConfig.endHeading=180;
        Route corner;assert(!corner.build(cornerPoints,cornerConfig));
        assert(corner.segments[corner.segmentCount-1].kind==ROTATE);
        if(!explicitTurn)--corner.segmentCount; // 同时覆盖显式末尾转向和路径结束后的隐式校角。
        Sim car;Result measured;Runner runner;runner.begin(corner,settings,car,measured);
        bool triggered=false,cornerTriggered=false;int cycles=0;
        while(runner.active()) {
            runner.tick();
            if(!triggered&&runner.observationApproach()) {
                assert(!runner.lastTranslation());
                assert(norm(measured.estimated-Vec(2100,1200))<3);
                assert(fabsf(turn(car.heading,0))<3); // 进入转向前即触发。
                triggered=true;
            }
            if(!cornerTriggered&&runner.finalTurnApproach()) {
                assert(triggered); // 普通视觉已经提前展开，角点姿态仍等到末段直行完成。
                assert(norm(measured.estimated-Vec(2100,endY))<3);
                assert(fabsf(turn(car.heading,90))<3); // 此时尚未开始最终90→180度旋转。
                cornerTriggered=true;
            }
            car.wait(1);assert(++cycles<100000);
        }
        assert(triggered&&cornerTriggered&&!runner.failure());
    }
}
void middleProfile() {
    for(bool reverse:{false,true}) {
        Node points[]={{{310,1200},0},{{600,1200},0},{{900,1200},0,true},{{1200,1200},0}};
        if(reverse)for(int i=0;i<2;++i){Node temp=points[i];points[i]=points[3-i];points[3-i]=temp;}
        Config cfg;cfg.policy=2;cfg.count=4;cfg.startHeading=cfg.endHeading=reverse?180:0;
        Route path;assert(!path.build(points,cfg));
        assert(path.segments[0].middle==reverse&&path.segments[1].middle&&path.segments[2].middle!=reverse);
        Settings settings;settings.speedMmS=400;settings.accelMmS2=2000;
        settings.middleSpeedMmS=150;settings.middleAccelMmS2=500;
        ProfileSim car;car.heading=cfg.startHeading;car.location=points[0].p;
        Result result;assert(!run(path,settings,car,result));
        // 同速区间可合并，跨内外层边界不可把整条共线道路当成同一种曲线。
        assert(car.actions==2&&car.speeds.size()==1&&car.stableReports==3);
        assert(car.speeds[0]==rpmFor(400,cfg.forwardPpm));
        assert(car.accelerations[0]==driverAccelerationFor(2000,cfg.forwardPpm));
        assert(car.velocityUpdates>10); // 中间道路独立规划线速度并实时纠偏，外围仍只下发一次位置目标。
        assert(norm(result.estimated-points[3].p)<3);
    }
}
struct HoldSim : ProfileSim {
    Vec entry;float biasRate=0,kick=0,maxError=0,finalMovingError=0,peakSpeed=0;
    int holdReports=0;bool kicked=false,sawNegativeCorrection=false,sawPositiveCorrection=false;
    uint32_t lastHoldAt=0;bool stop=false;float maxAcceleration=0,previousSpeed=0;
    void wait(uint32_t ms)override {
        Sim::wait(ms);
        if(velocityMode&&!stop) {
            heading=wrap(heading+biasRate*ms*.001f);
            if(!kicked&&norm(location-entry)>300){heading=wrap(heading+kick);kicked=true;}
        }
    }
    const char *velocity(const WheelCommand &command)override {
        wait(20); // 实际适配器四轮装载各5ms，最后同步生效。
        const bool moving=command.rpm[0]||command.rpm[1]||command.rpm[2]||command.rpm[3];
        if(moving) {
            peakSpeed=fmaxf(peakSpeed,fabsf(command.body.y));
            const float error=turn(heading,0);maxError=fmaxf(maxError,fabsf(error));finalMovingError=error;
            if(error<-.1f&&command.omega<0)sawNegativeCorrection=true;
            if(error>.1f&&command.omega>0)sawPositiveCorrection=true;
        } else if(velocityUpdates)stop=true;
        return Sim::velocity(command);
    }
    void headingControl(float,float,float,float speed)override {
        if(lastHoldAt)maxAcceleration=fmaxf(maxAcceleration,fabsf(speed-previousSpeed)*1000/(time-lastHoldAt));
        previousSpeed=speed;lastHoldAt=time;peakSpeed=fmaxf(peakSpeed,speed);++holdReports;
    }
};
void middleHold() {
    for(bool reverse:{false,true})for(float disturbance:{-3.0f,3.0f})for(int polarity:{-1,1}) {
        Node points[]={{{reverse?1900.0f:400.0f,1200},0,true},{{reverse?400.0f:1900.0f,1200},0,true}};
        Config cfg;cfg.policy=2;cfg.count=2;cfg.startHeading=cfg.endHeading=0;
        Route path;assert(!path.build(points,cfg));
        Settings settings;settings.middleSpeedMmS=700;settings.middleAccelMmS2=600;
        HoldSim car;car.location=car.entry=points[0].p;car.kick=disturbance;car.biasRate=disturbance/3;
        for(int i=0;i<4;++i)car.encoderPolarity[i]=polarity;
        Result result;const char *failure=run(path,settings,car,result);
        if(failure)fprintf(stderr,"middle hold: %s reverse=%d disturbance=%f polarity=%d\n",failure,reverse,disturbance,polarity);
        assert(!failure&&car.kicked&&car.holdReports>5&&car.speeds.empty());
        assert(fabsf(car.finalMovingError)<1&&norm(result.estimated-points[1].p)<2);
        assert(norm(car.location-points[1].p)<45); // 注入侧滑/偏航后仍在行驶中拉回航向，而非等停车校角。
        assert(disturbance>0?car.sawNegativeCorrection:car.sawPositiveCorrection);
        assert(car.peakSpeed<=701&&car.maxAcceleration<=610);
    }
    for(float length:{100.0f,1730.0f}) {
        Node points[]={{{320,1200},0,true},{{320+length,1200},0,true}};
        Config cfg;cfg.policy=2;cfg.count=2;Route path;assert(!path.build(points,cfg));
        HoldSim car;car.location=car.entry=points[0].p;car.latencyMs=3;
        Settings settings;settings.middleSpeedMmS=3000;settings.middleAccelMmS2=1000;
        Result result;assert(!run(path,settings,car,result));
        if(norm(car.location-points[1].p)>=3||car.time>=7000)
            fprintf(stderr,"hold length=%f actual=%f,%f estimated=%f time=%u peak=%f\n",length,car.location.x,car.location.y,result.estimated.x,car.time,car.peakSpeed);
        assert(norm(car.location-points[1].p)<3&&car.time<7000);
        assert(car.peakSpeed>(length>1000?1000:150)); // 长段不受试跑默认80RPM限速；短段也正常起步、减速。
    }
    // IMU暂不可用沿用编码器行程；取消时直接停车，不进入额外等待。
    for(bool cancel:{false,true}) {
        Node points[]={{{400,1200},0,true},{{1200,1200},0,true}};
        Config cfg;cfg.policy=2;cfg.count=2;Route path;assert(!path.build(points,cfg));
        HoldSim car;car.location=car.entry=points[0].p;car.missing=true;car.cancelAt=cancel?1600:0;
        Settings settings;settings.middleSpeedMmS=500;Result result;const char *failure=run(path,settings,car,result);
        if(cancel)assert(failure&&!strcmp(failure,"STOPPED"));else assert(!failure&&norm(result.estimated-points[1].p)<2);
        for(int i=0;i<4;++i)assert(car.rpm[i]==0);
    }
}
void stableFrames() {
    HeadingStable3 stable;stable.begin(100);
    H30Attitude sample;sample.fields=7;
    auto frame=[&](uint16_t sequence,uint32_t stamp,float heading){sample.sequence=sequence;sample.receivedAtMs=stamp;return stable.update(sample,heading);};
    assert(!frame(1,99,30)); // 停轮前旧帧不计数。
    assert(!frame(2,110,359.95f));
    assert(!frame(2,110,359.95f)); // 重复读取不能充当第二帧。
    assert(!frame(3,120,.03f));
    assert(frame(4,130,.08f)); // 跨0度，三帧总摆幅0.13度。
    stable.begin(200);
    assert(!frame(5,210,0));assert(!frame(6,220,.1f));
    assert(!frame(7,230,.25f)); // 极差0.25度，仍不能宣布停稳。
    assert(frame(8,240,.28f)); // 最新三帧0.10/0.25/0.28度，极差0.18度。
    stable.begin(300);
    assert(!frame(9,310,5));assert(!frame(10,320,5.1f));
    assert(!frame(11,330,5.4f));assert(!frame(12,340,5.6f));
    assert(!frame(13,350,5.9f)); // 持续转动不能仅靠累计三帧通过。
}
void scanTick() {
    for(float direction:{1.0f,-1.0f}) {
        ScanSim car;car.location=car.start={1200,275};car.heading=180;car.stopAt=850;scanCar=&car;
        ScanMotionSettings settings{direction*700,direction*400,13.6f,2000,60};
        ScanMotionRunner runner;runner.begin(settings,car,scanReceived);int ticks=0;
        while(runner.active()){runner.tick();assert(++ticks<100000);}
        assert(!runner.failure()&&ticks>10&&car.halts==1);
        assert(fabsf(runner.forwardTravel()-direction*850)<3);
        assert(car.crossed&&!car.zeroWhileMoving&&car.fastRpm>100);
    }
}
void representativePolling() {
    Config cfg;cfg.policy=2;cfg.count=2;cfg.startHeading=cfg.endHeading=0;
    Node nodes[]={{{310,1200},0},{{1200,1200},0}};
    Route path;assert(!path.build(nodes,cfg));Settings settings;settings.speedMmS=200;
    Sim car;car.latencyMs=3;Result result;Runner runner;runner.begin(path,settings,car,result);
    bool speedOnly=false;
    while(runner.active()) {
        const int full=car.fullReads,speeds=car.speedReads;
        runner.tick();
        if(car.speedReads>speeds&&car.fullReads==full)speedOnly=true;
        car.wait(1);
    }
    assert(!runner.failure()&&speedOnly&&car.speedReads>car.fullReads*5);
    assert(car.passedComparisons==1&&car.waitingComparisons==0&&norm(result.estimated-Vec(1200,1200))<1);
    // 代表轮停转后，4号轮仍有6RPM尾速：必须打印WAIT，等四轮都停转才打印PASS并接续。
    Sim lagged;lagged.rpmTailByWheel[3]=300;Result delayed;Runner waitAll;waitAll.begin(path,settings,lagged,delayed);
    while(waitAll.active()){waitAll.tick();lagged.wait(1);}
    assert(!waitAll.failure()&&lagged.waitingComparisons>0&&lagged.passedComparisons==1);
    assert(lagged.comparisonWaitMs>=250&&lagged.time>car.time);
    // 即使代表轮尚未起步时读到0，也不能在四轮实际行程未完成前宣布到位。
    Sim initialZero;initialZero.falseArrived=true;Result unfinished;Runner verify;verify.begin(path,settings,initialZero,unfinished);
    for(int i=0;i<30&&verify.active();++i){verify.tick();initialZero.wait(1);}
    assert(verify.active()&&initialZero.passedComparisons==0);
}
int main(int argc,char **argv){assert(argc==2);if(!strcmp(argv[1],"scan"))scanTick();else if(!strcmp(argv[1],"stable"))stableFrames();else if(!strcmp(argv[1],"hold"))middleHold();else if(!strcmp(argv[1],"middle"))middleProfile();else if(!strcmp(argv[1],"representative"))representativePolling();else nodeTick();}
