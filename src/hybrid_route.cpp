#include "continuous_route.h"
#include "field_geometry.h"

namespace continuous {
namespace {
Vec cubic(const Segment &v,float t) {
    float u=1-t;return v.start*(u*u*u)+v.control1*(3*u*u*t)+v.control2*(3*u*t*t)+v.end*(t*t*t);
}
float parameter(const Segment &v,float local) {
    local=clamp(local,0,v.length);size_t i=1;
    while(i<64&&v.lut[i]<local)++i;
    float span=v.lut[i]-v.lut[i-1];
    return (i-1+(span>0?(local-v.lut[i-1])/span:0))/64.0f;
}
Pose sample(const Segment &v,float local) {
    Pose p;p.p=v.start;
    if(v.kind==ROTATE)return p;
    if(v.kind==LINE){p.tangent=(v.end-v.start)*(1/v.length);p.p=v.start+p.tangent*clamp(local,0,v.length);return p;}
    float t=parameter(v,local),u=1-t;
    p.p=cubic(v,t);
    Vec d=(v.control1-v.start)*(3*u*u)+(v.control2-v.control1)*(6*u*t)+(v.end-v.control2)*(3*t*t);
    Vec dd=(v.control2-v.control1*2+v.start)*(6*u)+(v.end-v.control2*2+v.control1)*(6*t);
    float n=norm(d);if(n>0.001f){p.tangent=d*(1/n);p.curvature=cross(d,dd)/(n*n*n);}
    return p;
}
float smooth(float u){return u*u*u*(10+u*(-15+6*u));}
float phaseHeading(const Segment &v,float local) {
    float u=v.length>0?clamp(local/v.length,0,1):0;
    return wrap(v.h0+turn(v.h0,v.h1)*smooth(u));
}
const char *validate(const Segment &v,float margin) {
    float delta=turn(v.h0,v.h1);
    size_t count=static_cast<size_t>(ceilf(fmaxf(v.length/2,fabsf(delta)*1.875f)));
    if(count<1)count=1;
    for(size_t i=0;i<=count;++i) {
        float u=static_cast<float>(i)/count;
        Pose p=sample(v,v.length*u);
        float heading=v.kind==ROTATE?wrap(v.h0+delta*u):phaseHeading(v,v.length*u);
        const char *error=field::check(p.p.x,p.p.y,heading,margin);if(error)return error;
    }
    return nullptr;
}
float roadHeading(float current,Vec direction) {
    float forward=wrap(atan2f(direction.y,direction.x)*180/PI_F),back=wrap(forward+180);
    return fabsf(turn(current,back))<fabsf(turn(current,forward))-0.01f?back:forward;
}
Segment outsideCurve(Vec p,Vec incoming,Vec outgoing,float trim,float outset,float from,float to) {
    Segment v;v.kind=CUBIC;v.start=p-incoming*trim;v.end=p+outgoing*trim;
    // 与内切圆弧相反：控制柄超过原节点，曲线中部向远离黄色角点的一侧鼓出。
    float handle=(4*trim+8*outset)/3;
    v.control1=v.start+incoming*handle;v.control2=v.end-outgoing*handle;
    v.h0=from;v.h1=to;Vec previous=v.start;
    for(size_t i=1;i<=64;++i){Vec next=cubic(v,i/64.0f);v.length+=norm(next-previous);v.lut[i]=v.length;previous=next;}
    return v;
}
}

bool Route::addRotation(Vec p,float from,float to) {
    if(segmentCount>=MAX_SEGMENTS)return false;
    Segment &v=segments[segmentCount++];v=Segment{};
    v.kind=ROTATE;v.start=v.end=p;v.offset=length;v.h0=from;v.h1=to;
    return !validate(v,config.margin);
}
const char *Route::buildHybrid(const Node *nodes,const Config &cfg) {
    if(cfg.policy!=1&&cfg.policy!=2)return "POLICY";
    if(!isfinite(cfg.outset)||cfg.outset<0||cfg.outset>100
        ||!isfinite(cfg.margin)||cfg.margin<10||cfg.margin>40)return "POLICY_RANGE";
    Vec direction[MAX_NODES];float distance[MAX_NODES]={},trim[MAX_NODES]={},outsets[MAX_NODES]={};
    float heads[MAX_NODES]={};
    for(size_t i=0;i<cfg.count;++i) {
        if(!isfinite(nodes[i].p.x)||!isfinite(nodes[i].p.y))return "NODE";
        if(i+1<cfg.count){distance[i]=norm(nodes[i+1].p-nodes[i].p);if(distance[i]<20)return "SHORT_SEGMENT";
            direction[i]=(nodes[i+1].p-nodes[i].p)*(1/distance[i]);}
    }
    heads[0]=roadHeading(cfg.startHeading,direction[0]);
    if(fabsf(turn(cfg.startHeading,heads[0]))>0.05f&&!addRotation(nodes[0].p,cfg.startHeading,heads[0]))return "INITIAL_TURN_SPACE";
    for(size_t i=1;i+1<cfg.count;++i) {
        heads[i]=roadHeading(heads[i-1],direction[i]);
        float d=dot(direction[i-1],direction[i]);
        if(d>0.9999f)continue;
        if(fabsf(d)>0.001f&&d>-0.9999f)return "TURN_NOT_90";
        Vec p=nodes[i].p;
        bool outside=(p.x<550||p.x>1850)&&(p.y<550||p.y>1850);
        if(cfg.policy==1&&outside&&fabsf(d)<0.001f) {
            float amount=fminf(180,fminf((distance[i-1]-20)*0.45f,(distance[i]-20)*0.45f));
            if(amount>=30) {
                const float offsets[]={cfg.outset,80,100,40,20,0};
                const float headings[]={heads[i],wrap(heads[i]+180)};
                for(float offset:offsets) {
                    // 外绕轨迹与出弯朝向联合校验，正向失败还要尝试倒退出弯。
                    for(float heading:headings) {
                        Segment curve=outsideCurve(p,direction[i-1],direction[i],amount,offset,heads[i-1],heading);
                        if(!validate(curve,cfg.margin)){trim[i]=amount;outsets[i]=offset;heads[i]=heading;break;}
                    }
                    if(trim[i]>0)break;
                }
            }
        }
    }
    Vec cursor=nodes[0].p;
    for(size_t edge=0;edge+1<cfg.count;++edge) {
        size_t i=edge+1;Vec entry=nodes[i].p-direction[edge]*trim[i];
        if(norm(entry-cursor)>0.001f){
            if(segmentCount>=MAX_SEGMENTS)return "PATH_TOO_LONG";
            line(cursor,entry);segments[segmentCount-1].h0=segments[segmentCount-1].h1=heads[edge];
        }
        if(i+1<cfg.count) {
            if(trim[i]>0) {
                if(segmentCount>=MAX_SEGMENTS)return "PATH_TOO_LONG";
                Segment curve=outsideCurve(nodes[i].p,direction[edge],direction[i],trim[i],outsets[i],heads[edge],heads[i]);
                curve.offset=length;segments[segmentCount++]=curve;length+=curve.length;cursor=curve.end;
            } else {
                if(dot(direction[edge],direction[i])<0.9999f
                    &&!addRotation(nodes[i].p,heads[edge],heads[i]))return "CORNER_TURN_SPACE";
                cursor=nodes[i].p;
            }
        }
    }
    if(!addRotation(nodes[cfg.count-1].p,heads[cfg.count-2],cfg.endHeading))return "FINAL_TURN_SPACE";
    for(size_t i=0;i<segmentCount;++i){const char *error=validatePhase(i);if(error)return error;}
    speedCount=static_cast<size_t>(ceilf(length/5))+1;
    if(speedCount>MAX_SPEED_SAMPLES)return "PATH_TOO_LONG";
    spacing=length/(speedCount-1);
    for(size_t i=0;i<speedCount;++i){
        float s=i*spacing;speeds[i]=geometricCap(s);
        for(size_t j=0;j<segmentCount;++j)if(segments[j].kind==ROTATE)
            speeds[i]=fminf(speeds[i],sqrtf(2*cfg.accel*fabsf(s-segments[j].offset)));
    }
    speeds[0]=0;speeds[speedCount-1]=0;
    for(size_t i=1;i<speedCount;++i)speeds[i]=fminf(speeds[i],sqrtf(speeds[i-1]*speeds[i-1]+2*cfg.accel*spacing));
    for(size_t i=speedCount-1;i>0;--i)speeds[i-1]=fminf(speeds[i-1],sqrtf(speeds[i]*speeds[i]+2*cfg.accel*spacing));
    for(size_t i=1;i<speedCount;++i){float sum=speeds[i]+speeds[i-1];if(sum<0.001f)return "NO_SPEED";plannedSeconds+=2*spacing/sum;}
    for(size_t i=0;i<segmentCount;++i)if(segments[i].kind==ROTATE)
        plannedSeconds+=fabsf(turn(segments[i].h0,segments[i].h1))/cfg.maxOmega+1;
    headingDelta=turn(cfg.startHeading,cfg.endHeading);return nullptr;
}
const char *Route::validatePhase(size_t index) const {return validate(segments[index],config.margin);}
size_t Route::movingPhase(float s) const {
    size_t last=0;
    for(size_t i=0;i<segmentCount;++i)if(segments[i].kind!=ROTATE){last=i;if(s<segments[i].offset+segments[i].length-0.0001f)return i;}
    return last;
}
Pose Route::phaseAt(size_t index,float local) const {return sample(segments[index],local);}
float Route::phaseHeading(size_t index,float local) const {return continuous::phaseHeading(segments[index],local);}
float Route::phaseSlope(size_t index,float local) const {
    const auto &v=segments[index];if(v.kind!=CUBIC)return 0;
    float u=clamp(local/v.length,0,1);return turn(v.h0,v.h1)*30*u*u*(1-u)*(1-u)/v.length;
}
float Route::nextStop(size_t index) const {
    for(size_t i=index;i<segmentCount;++i)if(segments[i].kind==ROTATE)return segments[i].offset;
    return length;
}
Output Tracker::updateHybrid(const Route &route,float actual,float dt,uint32_t sampleMs) {
    Output out;const Config &c=route.config;
    if(!isfinite(dt)||dt<=0||dt>0.120f){out.failure="CONTROL_OVERRUN";return out;}
    while(phaseIndex<route.segmentCount&&route.segments[phaseIndex].kind!=ROTATE
        &&s>=route.segments[phaseIndex].offset+route.segments[phaseIndex].length-0.5f) {
        if(phaseIndex+1<route.segmentCount&&route.segments[phaseIndex+1].kind==ROTATE){s=route.segments[phaseIndex+1].offset;speed=0;}
        ++phaseIndex;stableSeconds=turnSeconds=badSeconds=0;rateReady=false;
    }
    if(phaseIndex>=route.segmentCount){out.reference=c.endHeading;out.error=turn(actual,out.reference);out.finished=true;return out;}
    const Segment &phase=route.segments[phaseIndex];bool rotating=phase.kind==ROTATE;
    float reference=rotating?phase.h1:route.phaseHeading(phaseIndex,s-phase.offset);
    out.reference=reference;out.error=turn(actual,reference);
    float targetOmega=0;
    if(rotating) {
        speed=0;turnSeconds+=dt;finishing=phaseIndex+1==route.segmentCount;
        if(sampleMs!=previousSample) {
            float sampleDt=(sampleMs-previousSample)/1000.0f;
            float rate=sampleDt>0&&sampleDt<=0.2f?fabsf(turn(previousActual,actual))/sampleDt:1000;
            if(rateReady&&fabsf(out.error)<=1&&rate<=3)stableSeconds+=fminf(sampleDt,0.12f);else stableSeconds=0;
            rateReady=true;previousSample=sampleMs;previousActual=actual;
        }
        if(stableSeconds>=0.12f){++phaseIndex;omega=0;stableSeconds=turnSeconds=0;rateReady=false;
            if(phaseIndex==route.segmentCount)out.finished=true;return out;}
        float deadline=fabsf(turn(phase.h0,phase.h1))/fminf(45,c.maxOmega)*2+3;
        if(turnSeconds>deadline){out.failure="TURN_TIMEOUT";return out;}
        targetOmega=clamp(c.kp*out.error,-fminf(45,c.maxOmega),fminf(45,c.maxOmega));
    } else {
        finishing=false;
        badSeconds=fabsf(out.error)>5?badSeconds+dt:0;
        if(badSeconds>=0.2f){out.failure="HEADING_ERROR";return out;}
        float remaining=fmaxf(0,route.nextStop(phaseIndex)-s);
        float target=fminf(route.speedCap(s),sqrtf(2*c.accel*remaining+c.accel*c.accel*dt*dt)-c.accel*dt);
        target*=clamp(1-(fabsf(out.error)-1)/4,0.2f,1);
        float acceleration=c.accel;
        if(phase.kind==CUBIC) {
            float slope=fabsf(turn(phase.h0,phase.h1))*1.875f/phase.length;
            if(slope>0.000001f)acceleration=fminf(acceleration,30/slope);
        }
        speed=clamp(target,fmaxf(0,speed-acceleration*dt),speed+acceleration*dt);
        targetOmega=clamp(route.phaseSlope(phaseIndex,s-phase.offset)*speed+clamp(c.kp*out.error,-c.correction,c.correction),-c.maxOmega,c.maxOmega);
    }
    omega=clamp(targetOmega,omega-90*dt,omega+90*dt);
    Vec body=rotating?Vec{}:worldToBody(route.phaseAt(phaseIndex,s-phase.offset).tangent*speed,actual);
    out.wheels=wheels(body,omega,c);return out;
}
} // namespace continuous
