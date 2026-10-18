#include "../src/continuous_route.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
using namespace continuous;
#define CHECK(x) do{if(!(x)){fprintf(stderr,"motion line %d: %s\n",__LINE__,#x);exit(2);}}while(0)
static H30Attitude frame(uint32_t ms,float rate=0) {
    H30Attitude v;v.fields=7;v.sequence=ms/40;v.receivedAtMs=ms;v.accel[2]=-9.78f;v.gyro[2]=rate;return v;
}
static Route route;
int main() {
    H30MotionQuiet quiet;
    for(int i=0;i<20;++i)CHECK(!quiet.update(frame(1000),true));
    CHECK(!quiet.update(frame(1040),true));CHECK(!quiet.update(frame(1080),true));
    CHECK(quiet.update(frame(1120),true));CHECK(quiet.stableMs()==120);
    CHECK(!quiet.update(frame(1160),false));
    for(int i=0;i<10;++i)CHECK(!quiet.update(frame(1200+i*40,3),true));
    quiet.reset();
    for(int i=0;i<10;++i){auto v=frame(2000+i*40);v.accel[0]=i%2?.4f:0;CHECK(!quiet.update(v,true));}
    quiet.reset();CHECK(!quiet.update(frame(2000),true));CHECK(!quiet.update(frame(2200),true));
    auto missing=frame(2240);missing.fields=4;CHECK(!quiet.update(missing,true));
    H30HeadingHold hold;
    CHECK(hold.update(359,1,1000,2,20)>0);
    CHECK(fabsf(hold.update(1,-1,1040,2,20))<10); // 跨0/360的D不能产生巨大尖峰
    float old=hold.update(1,-1,1040,2,20);
    for(int i=0;i<20;++i)CHECK(hold.update(1,-1,1040,2,20)==old);
    for(int i=0;i<1000;++i)CHECK(fabsf(hold.update(1,4,1080+i*40,2,20))<=20);
    // 300RPM直行，注入持续偏转和短时3度扰动，共用控制器最终回正并惯性安静后完成。
    Config c;c.count=2;c.policy=2;c.startHeading=0;c.endHeading=0;c.maxRpm=300;
    Node nodes[]={{{310,1200},0},{{2090,1200},0}};
    CHECK(!route.build(nodes,c));Tracker tracker;tracker.begin(route,0);tracker.requireInertial=true;
    tracker.commit(WheelCommand{},1000);float actual=0,velocity=0;WheelCommand applied;
    bool finished=false,settled=false;uint32_t stamp=1000;float peak=0;
    for(int i=0;i<2000;++i) {
        stamp+=40;velocity+=(applied.omega+(.1f+norm(applied.body)*.003f)-velocity)*.35f;
        if(norm(applied.body)<.01f&&fabsf(applied.omega)<.01f)velocity*=.4f;
        actual=wrap(actual+velocity*.04f);
        if(i==40)actual=wrap(actual+3);
        tracker.integrate(route,actual,.04f);auto sample=frame(stamp,velocity);tracker.observeMotion(sample);
        auto out=tracker.update(route,actual,.04f,stamp);
        if(out.failure)fprintf(stderr,"failure %s err %f\n",out.failure,out.error);
        CHECK(!out.failure);peak=fmaxf(peak,fabsf(out.error));
        if(strcmp(tracker.stage,"SETTLE_BEFORE_TURN")==0)settled=true;
        if(out.finished){finished=true;break;}
        applied=out.wheels;tracker.commit(applied,stamp);
    }
    CHECK(finished&&settled);CHECK(peak>2);CHECK(fabsf(turn(actual,0))<=1);
    // 停稳门控：没有零指令不能完成，零指令之前的旧样本也不计数。
    tracker.begin(route,0);tracker.requireInertial=true;tracker.s=route.length;
    for(int i=0;i<10;++i){tracker.observeMotion(frame(1000+i*40));CHECK(!tracker.update(route,0,.04f,1000+i*40).finished);}
    tracker.commit(WheelCommand{},2000);
    for(int i=0;i<10;++i){tracker.observeMotion(frame(1400+i*40));CHECK(!tracker.update(route,0,.04f,1400+i*40).finished);}
    bool done=false;
    for(int i=0;i<5;++i){tracker.observeMotion(frame(2000+i*40));done=tracker.update(route,0,.04f,2000+i*40).finished;if(done)break;}
    CHECK(done);
    // 中心路口：先停平移并等惯性安静，再转向；转向停稳后才恢复下一段。
    Node corner[]={{{720,1200},0},{{1200,1200},0},{{1200,1680},0}};
    c.count=3;c.endHeading=90;CHECK(!route.build(corner,c));
    tracker.begin(route,0);tracker.requireInertial=true;tracker.commit(WheelCommand{},1000);
    actual=velocity=0;applied=WheelCommand{};stamp=1000;finished=false;
    bool before=false,after=false,next=false;
    for(int i=0;i<2500;++i) {
        stamp+=40;velocity+=(applied.omega-velocity)*.5f;actual=wrap(actual+velocity*.04f);
        tracker.integrate(route,actual,.04f);tracker.observeMotion(frame(stamp,velocity));
        auto out=tracker.update(route,actual,.04f,stamp);CHECK(!out.failure);
        if(strcmp(tracker.stage,"SETTLE_BEFORE_TURN")==0&&tracker.s<route.length-1)before=true;
        if(strcmp(tracker.stage,"SETTLE_AFTER_TURN")==0)after=true;
        if(tracker.phaseIndex>=2&&norm(out.wheels.body)>1){CHECK(before&&after);next=true;}
        if(out.finished){finished=true;break;}applied=out.wheels;tracker.commit(applied,stamp);
    }
    CHECK(finished&&next);CHECK(fabsf(turn(actual,90))<=1);
    // 持续振动不允许通过停稳门，必须报超时。
    tracker.begin(route,0);tracker.requireInertial=true;tracker.s=480;tracker.commit(WheelCommand{},1000);
    Output timeout;
    for(int i=0;i<100&&!timeout.failure;++i){stamp=1040+i*40;tracker.observeMotion(frame(stamp,10));timeout=tracker.update(route,0,.04f,stamp);}
    CHECK(timeout.failure&&strcmp(timeout.failure,"SETTLE_TIMEOUT")==0);
    puts("inertial settling and heading hold OK");return 0;
}
