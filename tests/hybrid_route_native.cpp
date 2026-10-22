#include "../src/continuous_route.h"
#include "../src/field_geometry.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
using namespace continuous;
#define CHECK(x) do{if(!(x)){fprintf(stderr,"hybrid line %d: %s\n",__LINE__,#x);exit(2);}}while(0)
static Route route;
static void simulate(const Node *nodes,Config cfg) {
    const char *error=route.build(nodes,cfg);if(error)fprintf(stderr,"build: %s\n",error);CHECK(!error);
    Tracker tracker;tracker.begin(route,cfg.startHeading);float actual=cfg.startHeading;
    WheelCommand previous;bool finished=false;uint32_t sample=1000;
    for(int tick=0;tick<10000;++tick) {
        actual=wrap(actual+previous.omega*.04f);sample+=40;
        tracker.integrate(route,actual,.04f);
        Output out=tracker.update(route,actual,.04f,sample);
        if(out.failure)fprintf(stderr,"update: %s s=%f h=%f ref=%f\n",out.failure,tracker.s,actual,out.reference);
        CHECK(!out.failure);
        CHECK(!field::check(tracker.estimated.x,tracker.estimated.y,actual,0));
        if(out.finished){finished=true;break;}
        if(tracker.phaseIndex<route.segmentCount&&route.segments[tracker.phaseIndex].kind==ROTATE)
            CHECK(norm(out.wheels.body)<.01f);
        tracker.commit(out.wheels);previous=out.wheels;
    }
    CHECK(finished);CHECK(fabsf(turn(actual,cfg.endHeading))<=1.1f);
    CHECK(norm(tracker.estimated-nodes[cfg.count-1].p)<15);
}
int main() {
    Config c;c.count=5;c.startHeading=90;c.endHeading=180;c.policy=1;
    Node outside[]={{{2050,1200},0},{{2050,1680},0},{{2050,2020},0},{{1680,2020},0},{{1200,2020},0}};
    simulate(outside,c);int curves=0;
    for(size_t i=0;i<route.segmentCount;++i)if(route.segments[i].kind==CUBIC)++curves;
    CHECK(curves==1);
    printf("{\"signature\":%u,\"length\":%.6f,\"samples\":[",field::signature(),route.length);
    for(int i=0;i<=10;++i){float s=route.length*i/10;Pose p=route.at(s);printf("%s[%.6f,%.6f,%.6f]",i?",":"",p.p.x,p.p.y,route.heading(s));}
    printf("],\"regression\":{");
    Node left[]={{{310,1200},0},{{310,1680},0},{{310,2020},0},{{720,2020},0},{{1200,2020},0}};
    Config regression=c;regression.startHeading=180.855f;regression.maxRpm=300;
    regression.accel=500;regression.maxOmega=90;
    simulate(left,regression);CHECK(route.segments[3].kind==CUBIC);
    CHECK(route.segments[3].h0==270&&route.segments[3].h1==180);
    CHECK(route.segments[4].h0==180&&route.segments[5].h0==180);
    printf("\"length\":%.6f,\"samples\":[",route.length);
    for(int i=0;i<=10;++i){float s=route.length*i/10;Pose p=route.at(s);printf("%s[%.6f,%.6f,%.6f]",i?",":"",p.p.x,p.p.y,route.heading(s));}
    printf("]}}");
    for(float heading:{179.9f,180.0f,180.1f,181.0f,270.0f,90.0f}) {
        regression.startHeading=heading;CHECK(!route.build(left,regression));int count=0;
        for(size_t i=0;i<route.segmentCount;++i)if(route.segments[i].kind==CUBIC)++count;
        CHECK(count==1);
    }
    Node loop[]={{{2050,1200},0},{{2050,1680},0},{{2050,2020},0},{{1680,2020},0},{{1200,2020},0},
        {{720,2020},0},{{310,2020},0},{{310,1680},0},{{310,1200},0},{{310,720},0},{{310,240},0},
        {{720,240},0},{{1200,240},0},{{1680,240},0},{{2050,240},0},{{2050,720},0},{{2050,1200},0}};
    c.count=17;c.endHeading=90;simulate(loop,c);
    c.count=5;c.endHeading=180;
    c.policy=2;simulate(outside,c);
    for(size_t i=0;i<route.segmentCount;++i)CHECK(route.segments[i].kind!=CUBIC);
    Node center[]={{{720,1200},0},{{1200,1200},0},{{1200,1680},0}};
    c.policy=1;c.count=3;c.startHeading=0;c.endHeading=90;simulate(center,c);
    CHECK(route.segments[1].kind==ROTATE);
    c.startHeading=90;CHECK(strcmp(route.build(center,c),"INITIAL_TURN_SPACE")==0);
    Node forbidden[]={{{310,240},0},{{720,240},0},{{720,720},0}};
    c.startHeading=0;c.endHeading=90;CHECK(route.build(forbidden,c));
    CHECK(field::check(720,1200,45,20));CHECK(!field::check(1200,1200,45,20));
    Node back[]={{{1200,1200},0},{{1200,1680},0},{{1200,1200},0}};
    c.startHeading=90;c.endHeading=90;simulate(back,c);
    // 重复的H30样本不能把转向完成；即使已到目标角也必须等待新样本。
    c.startHeading=0;c.endHeading=90;c.count=2;
    Node initial[]={{{1200,1200},0},{{1200,1680},0}};
    CHECK(!route.build(initial,c));Tracker tracker;tracker.begin(route,0);
    for(int i=0;i<20;++i){auto out=tracker.update(route,90,.04f,1000);CHECK(!out.finished);CHECK(tracker.phaseIndex==0);}
    for(int i=1;i<=6;++i)tracker.update(route,90,.04f,1000+40*i);
    CHECK(tracker.phaseIndex==1);
    tracker.begin(route,0);Output out;
    for(int i=0;i<190&&!out.failure;++i)out=tracker.update(route,0,.04f,1000+40*i);
    CHECK(out.failure&&strcmp(out.failure,"TURN_TIMEOUT")==0);
    tracker.begin(route,0);out=tracker.update(route,0,.121f,1000);
    CHECK(out.failure&&strcmp(out.failure,"CONTROL_OVERRUN")==0);
    c.count=3;c.startHeading=0;c.endHeading=90;CHECK(!route.build(center,c));
    tracker.begin(route,0);out=Output{};
    for(int i=0;i<7&&!out.failure;++i)out=tracker.update(route,7,.04f,1000+40*i);
    CHECK(out.failure&&strcmp(out.failure,"HEADING_ERROR")==0);
    return 0;
}
