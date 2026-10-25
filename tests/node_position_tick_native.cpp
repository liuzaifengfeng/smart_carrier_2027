#include "node_position_sim.h"
#include "motion_sequence.h"
#include "motion_job.h"
#include <assert.h>
#include <vector>
struct ProfileSim : Sim {
    std::vector<uint16_t> speeds;
    std::vector<uint8_t> accelerations;
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
        assert(car.actions==2&&car.speeds.size()==2);
        for(int i=0;i<2;++i){bool middle=reverse?i==0:i==1;
            assert(car.speeds[i]==rpmFor(middle?150:400,cfg.forwardPpm));
            assert(car.accelerations[i]==driverAccelerationFor(middle?500:2000,cfg.forwardPpm));
        }
        assert(norm(result.estimated-points[3].p)<3);
    }
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
int main(int argc,char **argv){assert(argc==2);if(!strcmp(argv[1],"scan"))scanTick();else if(!strcmp(argv[1],"middle"))middleProfile();else if(!strcmp(argv[1],"representative"))representativePolling();else nodeTick();}
