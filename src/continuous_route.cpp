#include "continuous_route.h"

namespace continuous {
void Route::line(Vec a,Vec b) {
    float n=norm(b-a); if(n<0.001f)return;
    Segment &v=segments[segmentCount++]; v=Segment{};
    v.start=a;v.end=b;v.offset=length;v.length=n;length+=n;
}
const char *Route::build(const Node *nodes,const Config &cfg) {
    config=cfg;segmentCount=0;speedCount=0;length=0;plannedSeconds=0;
    if(!nodes||cfg.count<2||cfg.count>MAX_NODES)return "COUNT";
    const float values[]={cfg.startHeading,cfg.endHeading,cfg.maxRpm,cfg.accel,cfg.lateralAccel,
        cfg.lateralPpm,cfg.forwardPpm,cfg.rotationPpd,cfg.kp,cfg.maxOmega,cfg.correction};
    for(float v:values)if(!isfinite(v))return "RANGE";
    if(cfg.startHeading<0||cfg.startHeading>=360||cfg.endHeading<0||cfg.endHeading>=360
        ||cfg.maxRpm<1||cfg.maxRpm>5000||cfg.maxRpm!=floorf(cfg.maxRpm)||cfg.accel<1||cfg.accel>(cfg.policy==2?4000:2000)
        ||cfg.lateralAccel<1||cfg.lateralAccel>3000||cfg.lateralPpm<0.01f||cfg.lateralPpm>1000
        ||cfg.forwardPpm<0.01f||cfg.forwardPpm>1000||cfg.rotationPpd<=0||cfg.rotationPpd>10000
        ||cfg.kp<0.1f||cfg.kp>10||cfg.maxOmega<5||cfg.maxOmega>90
        ||cfg.correction<1||cfg.correction>=cfg.maxOmega)return "RANGE";
    if(cfg.policy)return buildHybrid(nodes,cfg);
    Vec direction[MAX_NODES];float distance[MAX_NODES]={},trim[MAX_NODES]={};
    for(size_t i=0;i<cfg.count;++i) {
        if(!isfinite(nodes[i].p.x)||!isfinite(nodes[i].p.y)||!isfinite(nodes[i].radius)
            ||nodes[i].radius<0||nodes[i].radius>1000)return "NODE";
        if(i+1<cfg.count) {
            distance[i]=norm(nodes[i+1].p-nodes[i].p);
            if(distance[i]<20)return "SHORT_SEGMENT";
            direction[i]=(nodes[i+1].p-nodes[i].p)*(1/distance[i]);
        }
    }
    if(nodes[0].radius!=0||nodes[cfg.count-1].radius!=0)return "END_RADIUS";
    float minRadius=100000;
    for(size_t i=1;i+1<cfg.count;++i) {
        float d=dot(direction[i-1],direction[i]);
        if(d>0.9999f)continue;
        if(fabsf(d)>0.001f)return "TURN_NOT_90";
        if(nodes[i].radius<20)return "CORNER_NOT_ALLOWED";
        trim[i]=nodes[i].radius;minRadius=fminf(minRadius,trim[i]);
    }
    for(size_t i=0;i+1<cfg.count;++i)
        if(trim[i]+trim[i+1]>distance[i]-20)return "RADIUS_OVERLAP";
    Vec cursor=nodes[0].p;
    for(size_t i=1;i<cfg.count;++i) {
        Vec entry=nodes[i].p-direction[i-1]*trim[i];line(cursor,entry);
        if(trim[i]>0) {
            Segment &v=segments[segmentCount++];v=Segment{};
            v.start=entry;v.end=nodes[i].p+direction[i]*trim[i];
            v.center=entry+direction[i]*trim[i];v.radius=trim[i];
            v.angle=atan2f(entry.y-v.center.y,entry.x-v.center.x);
            v.sweep=cross(direction[i-1],direction[i])>0?PI_F/2:-PI_F/2;
            v.length=trim[i]*PI_F/2;v.offset=length;length+=v.length;
            cursor=v.end;
        } else cursor=nodes[i].p;
    }
    headingDelta=turn(cfg.startHeading,cfg.endHeading);
    // 空间与姿态轮廓验算，最多 5mm / 1deg，包含端点。
    float ds=fminf(5,minRadius*PI_F/180);
    if(fabsf(headingDelta)>0.001f)ds=fminf(ds,length/(fabsf(headingDelta)*1.875f));
    if(ds<=0||length/ds>200000)return "PATH_TOO_LONG";
    size_t checks=static_cast<size_t>(ceilf(length/ds));
    for(size_t i=0;i<=checks;++i)if(!inside(length*i/checks))return "BOUNDARY";
    speedCount=static_cast<size_t>(ceilf(length/5))+1;
    if(speedCount>MAX_SPEED_SAMPLES)return "PATH_TOO_LONG";
    spacing=length/(speedCount-1);
    for(size_t i=0;i<speedCount;++i)speeds[i]=geometricCap(i*spacing);
    speeds[0]=0;speeds[speedCount-1]=0;
    for(size_t i=1;i<speedCount;++i)
        speeds[i]=fminf(speeds[i],sqrtf(speeds[i-1]*speeds[i-1]+2*cfg.accel*spacing));
    for(size_t i=speedCount-1;i>0;--i)
        speeds[i-1]=fminf(speeds[i-1],sqrtf(speeds[i]*speeds[i]+2*cfg.accel*spacing));
    for(size_t i=1;i<speedCount;++i) {
        float sum=speeds[i-1]+speeds[i];if(sum<0.01f)return "NO_SPEED";
        plannedSeconds+=2*spacing/sum;
    }
    // 初始位置和当前导航位置是否一致，由运行时再次校验。
    return nullptr;
}
Pose Route::at(float s) const {
    if(config.policy){size_t i=movingPhase(s);return phaseAt(i,s-segments[i].offset);}
    s=clamp(s,0,length);size_t i=0;
    while(i+1<segmentCount&&s>=segments[i].offset+segments[i].length)++i;
    const Segment &v=segments[i];float t=clamp((s-v.offset)/v.length,0,1);Pose p;
    if(v.radius==0){p.tangent=(v.end-v.start)*(1/v.length);p.p=v.start+(v.end-v.start)*t;}
    else {float a=v.angle+v.sweep*t,sign=v.sweep>0?1:-1;
        p.p=v.center+Vec(cosf(a),sinf(a))*v.radius;
        p.tangent=Vec(-sinf(a)*sign,cosf(a)*sign);p.curvature=sign/v.radius;}
    return p;
}
float Route::heading(float s) const {
    if(config.policy){size_t i=movingPhase(s);return phaseHeading(i,s-segments[i].offset);}
    float u=clamp(s/length,0,1);return wrap(config.startHeading+headingDelta*u*u*u*(10+u*(-15+6*u)));
}
float Route::headingSlope(float s) const {
    if(config.policy){size_t i=movingPhase(s);return phaseSlope(i,s-segments[i].offset);}
    float u=clamp(s/length,0,1);return headingDelta*30*u*u*(1-u)*(1-u)/length;
}
bool Route::inside(float s) const {
    Pose p=at(s);
    for(int x:{-1,1})for(int y:{-1,1}) {
        Vec q=p.p+bodyToWorld(Vec(x*150.0f,y*150.0f),heading(s));
        if(q.x<20||q.y<20||q.x>2380||q.y>2380)return false;
    }
    return true;
}
float Route::geometricCap(float s) const {
    Pose p=at(s);Vec body=worldToBody(p.tangent,heading(s));
    float px=body.x*config.lateralPpm,py=body.y*config.forwardPpm,pr=headingSlope(s)*config.rotationPpd;
    const float rates[]={-px+py+pr,-px-py+pr,px+py+pr,px-py+pr};
    float peak=0;for(float v:rates)peak=fmaxf(peak,fabsf(v)*60/3200);
    float cap=config.maxRpm*0.75f/fmaxf(peak,0.0001f); // 给航向纠偏预留轮速
    if(fabsf(p.curvature)>0)cap=fminf(cap,sqrtf(config.lateralAccel/fabsf(p.curvature)));
    if(fabsf(headingSlope(s))>0.000001f)
        cap=fminf(cap,(config.maxOmega-config.correction)/fabsf(headingSlope(s)));
    if(config.policy) {
        const Segment &phase=segments[movingPhase(s)];
        if(phase.kind==CUBIC) {
            // 五次姿态插值的 |d²theta/ds²| 峰值为 10/sqrt(3)*|delta|/L²。
            // 90deg/s²执行器斜率限制中，给几何变化45、平移加速30，保留15纠偏。
            float peak=fabsf(turn(phase.h0,phase.h1))*5.773503f/(phase.length*phase.length);
            if(peak>0.000001f)cap=fminf(cap,sqrtf(45/peak));
        }
    }
    return cap;
}
float Route::speedCap(float s) const {
    size_t next=static_cast<size_t>(s/spacing)+1;if(next>=speedCount)next=speedCount-1;
    float remain=fmaxf(0,next*spacing-s);
    return fminf(geometricCap(s),sqrtf(speeds[next]*speeds[next]+2*config.accel*remain));
}
WheelCommand wheels(Vec body,float omega,const Config &cfg) {
    WheelCommand out;
    float px=body.x*cfg.lateralPpm,py=body.y*cfg.forwardPpm,pr=omega*cfg.rotationPpd;
    float values[]={-px+py+pr,-px-py+pr,px+py+pr,px-py+pr};
    float peak=0;for(float &v:values){v*=60/3200.0f;peak=fmaxf(peak,fabsf(v));}
    float scale=peak>cfg.maxRpm?cfg.maxRpm/peak:1;
    float p[4];for(int i=0;i<4;++i){out.rpm[i]=static_cast<int16_t>(lroundf(values[i]*scale));p[i]=out.rpm[i]*3200/60.0f;}
    out.body.x=(-p[0]-p[1]+p[2]+p[3])/(4*cfg.lateralPpm);
    out.body.y=(p[0]-p[1]+p[2]-p[3])/(4*cfg.forwardPpm);
    out.omega=(p[0]+p[1]+p[2]+p[3])/(4*cfg.rotationPpd);
    return out;
}
void Tracker::begin(const Route &route,float actual) {
    *this=Tracker{};estimated=route.at(0).p;(void)actual;
}
void Tracker::integrate(const Route &route,float actual,float dt) {
    Vec world=bodyToWorld(applied.body,actual);estimated=estimated+world*dt;
    if(route.config.policy) {
        if(phaseIndex<route.segmentCount&&route.segments[phaseIndex].kind!=ROTATE) {
            Vec tangent=route.phaseAt(phaseIndex,s-route.segments[phaseIndex].offset).tangent;
            s=fminf(route.nextStop(phaseIndex),s+fmaxf(0,dot(world,tangent))*dt);
        }
        return;
    }
    if(!finishing)s=fminf(route.length,s+fmaxf(0,dot(world,route.at(s).tangent))*dt);
}
Output Tracker::update(const Route &route,float actual,float dt,uint32_t sampleMs) {
    if(route.config.policy)return updateHybrid(route,actual,dt,sampleMs);
    Output out;const Config &c=route.config;
    out.reference=route.heading(s);out.error=turn(actual,out.reference);
    if(!isfinite(dt)||dt<=0||dt>0.120f){out.failure="CONTROL_OVERRUN";return out;}
    badSeconds=fabsf(out.error)>20?badSeconds+dt:0;
    if(badSeconds>=0.2f){out.failure="HEADING_ERROR";return out;}
    if(route.length-s<=0.5f)finishing=true;
    float targetOmega=0;
    if(finishing) {
        finishSeconds+=dt;out.reference=c.endHeading;out.error=turn(actual,c.endHeading);speed=0;
        if(sampleMs!=previousSample) {
            stableSeconds=fabsf(out.error)<=5?stableSeconds+dt:0;previousSample=sampleMs;
        }
        if(stableSeconds>=0.2f){out.finished=true;return out;}
        if(finishSeconds>3){out.failure="FINAL_HEADING_TIMEOUT";return out;}
        targetOmega=clamp(c.kp*out.error,-fminf(20,c.maxOmega),fminf(20,c.maxOmega));
    } else {
        float target=route.speedCap(s);
        target=fminf(target,sqrtf(2*c.accel*(route.length-s)+c.accel*c.accel*dt*dt)-c.accel*dt);
        target*=clamp(1-(fabsf(out.error)-5)/15,0.2f,1);
        speed=clamp(target,fmaxf(0,speed-c.accel*dt),speed+c.accel*dt);
        float correction=clamp(c.kp*out.error,-c.correction,c.correction);
        targetOmega=clamp(route.headingSlope(s)*speed+correction,-c.maxOmega,c.maxOmega);
    }
    omega=clamp(targetOmega,omega-90*dt,omega+90*dt);
    Vec body=finishing?Vec{}:worldToBody(route.at(s).tangent*speed,actual);
    out.wheels=wheels(body,omega,c);return out;
}
} // namespace continuous
