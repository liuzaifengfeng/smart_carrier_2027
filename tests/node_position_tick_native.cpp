#include "node_position_sim.h"
#include "motion_sequence.h"
#include "motion_job.h"
#include <assert.h>
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
        starts+=joined.shouldStartArm(!chassis.active(),chassis.lastTranslation(),chassis.remainingDistance(),500);
        base.wait(1);assert(++ticks<100000);
    }
    assert(!chassis.failure()&&!arm.failure()&&io.issued==1&&starts==1&&progressWhileArmWait>0);
    assert(base.zeroGroups==1&&base.actions==2); // 两条共线段合并，一次终点转向。
    assert(norm(result.estimated-Vec(1200,1200))<3);
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
int main(int argc,char **argv){assert(argc==2);if(!strcmp(argv[1],"scan"))scanTick();else nodeTick();}
