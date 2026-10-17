#include "../src/continuous_route.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#undef assert
// 主机编译器可能默认 NDEBUG；测试条件必须始终执行，失败不弹出 Windows CRT 对话框。
#define assert(expr) do { if(!(expr)) { fprintf(stderr,"CHECK failed at line %d: %s\n",__LINE__,#expr); exit(2); } } while(0)
using namespace continuous;

static Node point(float x,float y,float r=0){Node n;n.p=Vec(x,y);n.radius=r;return n;}
static void near(float actual,float expected,float tolerance=0.01f){
    if(fabsf(actual-expected)>tolerance)fprintf(stderr,"near actual=%f expected=%f tolerance=%f\n",actual,expected,tolerance);
    assert(fabsf(actual-expected)<=tolerance);
}
static void simulation(Route &route,bool disturb=false) {
    Tracker tracker;tracker.begin(route,route.config.startHeading);
    float heading=route.config.startHeading;WheelCommand previous;bool done=false;
    float maxError=0;int interiorStops=0;
    for(uint32_t i=1;i<20000;++i) {
        heading=wrap(heading+previous.omega*0.04f);
        if(disturb&&i==50)heading=wrap(heading+10);
        tracker.integrate(route,heading,0.04f);
        Output out=tracker.update(route,heading,0.04f,i*40);
        if(out.failure){fprintf(stderr,"simulation failure: %s s=%f / %f heading=%f ref=%f\n",out.failure,tracker.s,route.length,heading,out.reference);assert(false);}
        maxError=fmaxf(maxError,fabsf(out.error));
        if(out.finished){done=true;break;}
        for(int j=0;j<4;++j)assert(abs(out.wheels.rpm[j])<=route.config.maxRpm+0.5f);
        if(tracker.s>20&&route.length-tracker.s>20&&norm(out.wheels.body)<0.1f)++interiorStops;
        tracker.commit(out.wheels);previous=out.wheels;
    }
    assert(done);assert(interiorStops==0);
    near(turn(heading,route.config.endHeading),0,5);
    if(!disturb)assert(maxError<=5);
    // 无滑移离散模型，误差主要是40ms积分和圆弧离散误差。
    assert(norm(tracker.estimated-route.at(route.length).p)<35);
}
int main() {
    Config c;c.count=3;c.startHeading=180;c.endHeading=270;
    Node nodes[]={point(310,240),point(720,240,120),point(720,720)};
    Route route;assert(!route.build(nodes,c));
    near(route.length,410+480-240+120*PI_F/2);
    near(route.at(0).p.x,310);near(route.at(route.length).p.y,720);
    near(route.heading(0),180);near(route.heading(route.length),270);
    for(size_t i=1;i<route.segmentCount;++i) {
        auto a=route.at(route.segments[i].offset-0.001f),b=route.at(route.segments[i].offset+0.001f);
        assert(norm(a.p-b.p)<0.01f);assert(dot(a.tangent,b.tangent)>0.999f);
    }
    for(float theta:{0.f,90.f,180.f,270.f,359.f}) {
        Vec v(123,-64),b=worldToBody(v,theta),w=bodyToWorld(b,theta);
        near(v.x,w.x);near(v.y,w.y);
    }
    near(worldToBody(Vec(100,0),0).y,100);near(worldToBody(Vec(0,-100),0).x,100);
    WheelCommand command=wheels(Vec(200,100),20,c);
    for(int16_t v:command.rpm)assert(abs(v)<=80);
    // 饱和后必须由下发轮速反解，不能继续使用请求速度推进路程。
    assert(norm(command.body)<hypotf(200,100));
    Tracker t;t.begin(route,180);t.commit(command);t.integrate(route,180,0.04f);
    near(t.s,fmaxf(0,dot(bodyToWorld(command.body,180),route.at(0).tangent))*0.04f);
    simulation(route);simulation(route,true);
    c.startHeading=350;c.endHeading=10;assert(!route.build(nodes,c));simulation(route);
    c.startHeading=0;c.endHeading=180;nodes[0]=point(310,720);nodes[1]=point(720,720,120);nodes[2]=point(720,240);
    assert(!route.build(nodes,c));simulation(route);
    c.startHeading=c.endHeading=180;Node straight[]={point(310,720),point(720,720),point(1200,720)};
    assert(!route.build(straight,c));simulation(route);
    c.count=4;Node multi[]={point(310,720),point(720,720,120),point(720,1200,120),point(1200,1200)};
    c.startHeading=180;c.endHeading=90;assert(!route.build(multi,c));simulation(route);
    c.maxRpm=160;assert(!route.build(multi,c));simulation(route);
    c.maxRpm=400;assert(!route.build(multi,c));simulation(route);
    multi[1].radius=240;multi[2].radius=240;
    assert(strcmp(route.build(multi,c),"RADIUS_OVERLAP")==0);
    c.count=3;nodes[0]=point(310,240);nodes[1]=point(720,240,0);nodes[2]=point(720,720);
    assert(strcmp(route.build(nodes,c),"CORNER_NOT_ALLOWED")==0);
    nodes[1].radius=120;nodes[2]=nodes[0];assert(strcmp(route.build(nodes,c),"TURN_NOT_90")==0);
    nodes[2]=point(720,720);c.startHeading=45;nodes[0]=point(150,150);
    assert(route.build(nodes,c)!=nullptr);
    nodes[0]=point(310,240);c.startHeading=180;c.endHeading=270;assert(!route.build(nodes,c));
    t.begin(route,180);auto out=t.update(route,180,0.121f,40);assert(strcmp(out.failure,"CONTROL_OVERRUN")==0);
    t.begin(route,180);
    for(int i=0;i<6;++i)out=t.update(route,150,0.04f,40*(i+1));
    assert(out.failure&&strcmp(out.failure,"HEADING_ERROR")==0);
    t.begin(route,180);t.s=route.length;
    out=Output{};
    for(int i=0;i<78&&!out.failure;++i)out=t.update(route,260,0.04f,40*(i+1));
    assert(out.failure&&strcmp(out.failure,"FINAL_HEADING_TIMEOUT")==0);
    t.begin(route,180);t.s=route.length;
    for(int i=0;i<20;++i){out=t.update(route,270,0.04f,40);assert(!out.finished);}
    c.startHeading=NAN;assert(strcmp(route.build(nodes,c),"RANGE")==0);
    c.startHeading=180;c.endHeading=270;assert(!route.build(nodes,c));
    printf("{\"length\":%.6f,\"samples\":[",route.length);
    for(int i=0;i<=10;++i) {
        float s=route.length*i/10;auto p=route.at(s);
        printf("%s[%.6f,%.6f,%.6f]",i?",":"",p.p.x,p.p.y,route.heading(s));
    }
    printf("]}\n");return 0;
}
