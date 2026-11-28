// 混合路径把平移、外绕曲线和原地转向组织成阶段。
// 曲线的几何采样、航向插值和跟踪阶段切换分别处理。

#include "continuous_route.h"  // 引入 continuous_route.h 中的声明和定义。
#include "field_geometry.h"  // 引入 field_geometry.h 中的声明和定义。

namespace continuous {  // 定义 continuous 的类型成员或作用域。
namespace {  // 开始匿名命名空间，本文件内部共享其中的定义。
Vec cubic(const Segment &v,float t) {  // 定义函数或成员 的入口 cubic，参数由此处的签名确定。
    float u=1-t;return v.start*(u*u*u)+v.control1*(3*u*u*t)+v.control2*(3*u*t*t)+v.end*(t*t*t);  // 将 1-t 写入或初始化 u。然后结束当前函数并返回 v.start*(u*u*u)+v.control1*(3*u*u*t)+v.control2*(3*u*t…。
}  // 结束此函数或控制块。
float parameter(const Segment &v,float local) {  // 定义函数或成员 的入口 parameter，参数由此处的签名确定。
    local=clamp(local,0,v.length);size_t i=1;  // 将 clamp(local,0,v.length) 写入或初始化 local。然后将 1 写入或初始化 i。
    while(i<64&&v.lut[i]<local)++i;  // 当 i<64&&v.lut[i]<local 成立时继续循环。
    float span=v.lut[i]-v.lut[i-1];  // 将 v.lut[i]-v.lut[i-1] 写入或初始化 span。
    return (i-1+(span>0?(local-v.lut[i-1])/span:0))/64.0f;  // 结束当前函数并返回 (i-1+(span>0?(local-v.lut[i-1])/span:0))/64.0f。
}  // 结束此函数或控制块。
Pose sample(const Segment &v,float local) {  // 定义函数或成员 的入口 sample（采样），参数由此处的签名确定。
    Pose p;p.p=v.start;  // 声明 p，类型和可用范围由所在声明决定。然后将 v.start 写入或初始化 p.p。
    if(v.kind==ROTATE)return p;  // 若 v.kind==ROTATE，立即返回 p。
    if(v.kind==LINE){p.tangent=(v.end-v.start)*(1/v.length);p.p=v.start+p.tangent*clamp(local,0,v.length);return p;}  // 当 v.kind==LINE 成立时进入此分支，否则继续后续流程。
    float t=parameter(v,local),u=1-t;  // 将 parameter(v,local),u=1-t 写入或初始化 t。
    p.p=cubic(v,t);  // 将 cubic(v,t) 写入或初始化 p.p。
    Vec d=(v.control1-v.start)*(3*u*u)+(v.control2-v.control1)*(6*u*t)+(v.end-v.control2)*(3*t*t);  // 将 (v.control1-v.start)*(3*u*u)+(v.control2-v.control1)*(… 写入或初始化 d。
    Vec dd=(v.control2-v.control1*2+v.start)*(6*u)+(v.end-v.control2*2+v.control1)*(6*t);  // 将 (v.control2-v.control1*2+v.start)*(6*u)+(v.end-v.contr… 写入或初始化 dd。
    float n=norm(d);if(n>0.001f){p.tangent=d*(1/n);p.curvature=cross(d,dd)/(n*n*n);}  // 将 norm(d);if(n>0.001f){p.tangent=d*(1/n);p.curvature=cro… 写入或初始化 n。
    return p;  // 结束当前函数并返回 p。
}  // 结束此函数或控制块。
float smooth(float u){return u*u*u*(10+u*(-15+6*u));}  // 定义函数或成员 smooth，参数由此处的签名确定。
float phaseHeading(const Segment &v,float local) {  // 定义函数或成员 的入口 phaseHeading（阶段航向），参数由此处的签名确定。
    float u=v.length>0?clamp(local/v.length,0,1):0;  // 将 v.length>0?clamp(local/v.length,0,1):0 写入或初始化 u。
    return wrap(v.h0+turn(v.h0,v.h1)*smooth(u));  // 结束当前函数并返回 wrap(v.h0+turn(v.h0,v.h1)*smooth(u))。
}  // 结束此函数或控制块。
const char *validate(const Segment &v,float margin) {  // 定义函数或成员 的入口 validate，参数由此处的签名确定。
    float delta=turn(v.h0,v.h1);  // 将 turn(v.h0,v.h1) 写入或初始化 delta。
    size_t count=static_cast<size_t>(ceilf(fmaxf(v.length/2,fabsf(delta)*1.875f)));  // 将 static_cast<size_t>(ceilf(fmaxf(v.length/2,fabsf(delta… 写入或初始化 count（计数）。
    if(count<1)count=1;  // 当 count<1 成立时进入此分支，否则继续后续流程。
    for(size_t i=0;i<=count;++i) {  // 按 size_t i=0;i<=count;++i 组织循环的初值、继续条件和步进。
        float u=static_cast<float>(i)/count;  // 将 static_cast<float>(i)/count 写入或初始化 u。
        Pose p=sample(v,v.length*u);  // 将 sample(v,v.length*u) 写入或初始化 p。
        float heading=v.kind==ROTATE?wrap(v.h0+delta*u):phaseHeading(v,v.length*u);  // 将 v.kind==ROTATE?wrap(v.h0+delta*u):phaseHeading(v,v.len… 写入或初始化 heading（航向）。
        const char *error=field::check(p.p.x,p.p.y,heading,margin);if(error)return error;  // 将 field::check(p.p.x,p.p.y,heading,margin) 写入或初始化 error（错误）。然后若 error，立即返回 error。
    }  // 结束此循环体。
    return nullptr;  // 结束当前函数并返回 nullptr。
}  // 结束此函数或控制块。
float roadHeading(float current,Vec direction) {  // 定义函数或成员 的入口 roadHeading（road航向），参数由此处的签名确定。
    float forward=wrap(atan2f(direction.y,direction.x)*180/PI_F),back=wrap(forward+180);  // 将 wrap(atan2f(direction.y,direction.x)*180/PI_F),back=wr… 写入或初始化 forward（前向）。
    return fabsf(turn(current,back))<fabsf(turn(current,forward))-0.01f?back:forward;  // 结束当前函数并返回 fabsf(turn(current,back))<fabsf(turn(current,forward))…。
}  // 结束此函数或控制块。
// 全停转路径同时考虑终点朝向，避免局部90度平局选错后在终点再掉头。
void stoppedHeadings(const Vec *directions,size_t edges,float start,float end,float *heads) {  // 定义函数或成员 的入口 stoppedHeadings（已停止Headings），参数由此处的签名确定。
    float candidates[MAX_NODES][2],cost[MAX_NODES][2];  // 声明 candidates，类型和可用范围由所在声明决定。
    for(size_t i=0;i<edges;++i) {  // 按 size_t i=0;i<edges;++i 组织循环的初值、继续条件和步进。
        candidates[i][0]=wrap(atan2f(directions[i].y,directions[i].x)*180/PI_F);  // 调用 wrap，传入 atan2f(directions[i].y,directions[i].x)*180/PI_F。
        candidates[i][1]=wrap(candidates[i][0]+180);  // 调用 wrap，传入 candidates[i][0]+180。
    }  // 结束此循环体。
    for(int i=static_cast<int>(edges)-1;i>=0;--i)for(int k=0;k<2;++k) {  // 按 int i=static_cast<int>(edges)-1;i>=0;--i)for(int k=0;k… 组织循环的初值、继续条件和步进。
        if(i==static_cast<int>(edges)-1)cost[i][k]=fabsf(turn(candidates[i][k],end));  // 当 i==static_cast<int>(edges)-1 成立时进入此分支，否则继续后续流程。
        else cost[i][k]=fminf(fabsf(turn(candidates[i][k],candidates[i+1][0]))+cost[i+1][0],  // 前一个判断不成立时执行此分支。
                             fabsf(turn(candidates[i][k],candidates[i+1][1]))+cost[i+1][1]);  // 调用 fabsf，传入 turn(candidates[i][k],candidates[i+1][1]))+cost[i+1][1]。
    }  // 结束此循环体。
    float current=start;  // 将 start 写入或初始化 current（当前）。
    for(size_t i=0;i<edges;++i) {  // 按 size_t i=0;i<edges;++i 组织循环的初值、继续条件和步进。
        float a=fabsf(turn(current,candidates[i][0])),b=fabsf(turn(current,candidates[i][1]));  // 将 fabsf(turn(current,candidates[i][0])),b=fabsf(turn(cur… 写入或初始化 a。
        float ca=a+cost[i][0],cb=b+cost[i][1];  // 将 a+cost[i][0],cb=b+cost[i][1] 写入或初始化 ca。
        int k=cb<ca-.01f||(fabsf(cb-ca)<=.01f&&b<a-.01f)?1:0;  // 将 cb<ca-.01f||(fabsf(cb-ca)<=.01f&&b<a-.01f)?1:0 写入或初始化 k。
        heads[i]=candidates[i][k];current=heads[i];  // 将 candidates[i][k] 写入或初始化 heads[i]。然后将 heads[i] 写入或初始化 current（当前）。
    }  // 结束此循环体。
}  // 结束此函数或控制块。
Segment outsideCurve(Vec p,Vec incoming,Vec outgoing,float trim,float outset,float from,float to) {  // 定义函数或成员 的入口 outsideCurve，参数由此处的签名确定。
    Segment v;v.kind=CUBIC;v.start=p-incoming*trim;v.end=p+outgoing*trim;  // 声明 v，类型和可用范围由所在声明决定。然后将 CUBIC 写入或初始化 v.kind。然后将 p-incoming*trim 写入或初始化 v.start。然后将 p+outgoing*trim 写入或初始化 v.end。
    // 与内切圆弧相反：控制柄超过原节点，曲线中部向远离黄色角点的一侧鼓出。
    float handle=(4*trim+8*outset)/3;  // 将 (4*trim+8*outset)/3 写入或初始化 handle。
    v.control1=v.start+incoming*handle;v.control2=v.end-outgoing*handle;  // 将 v.start+incoming*handle 写入或初始化 v.control1。然后将 v.end-outgoing*handle 写入或初始化 v.control2。
    v.h0=from;v.h1=to;Vec previous=v.start;  // 将 from 写入或初始化 v.h0。然后将 to 写入或初始化 v.h1。然后将 v.start 写入或初始化 previous（上一）。
    for(size_t i=1;i<=64;++i){Vec next=cubic(v,i/64.0f);v.length+=norm(next-previous);v.lut[i]=v.length;previous=next;}  // 按 size_t i=1;i<=64;++i 组织循环的初值、继续条件和步进。
    return v;  // 结束当前函数并返回 v。
}  // 结束此函数或控制块。
}  // 结束命名空间。

bool Route::addRotation(Vec p,float from,float to) {  // 定义函数或成员 的入口 Route::addRotation（路线addRotation），参数由此处的签名确定。
    if(segmentCount>=MAX_SEGMENTS)return false;  // 若 segmentCount>=MAX_SEGMENTS，立即返回 false。
    Segment &v=segments[segmentCount++];v=Segment{};  // 将 segments[segmentCount++];v=Segment{} 写入或初始化 v。
    v.kind=ROTATE;v.start=v.end=p;v.offset=length;v.h0=from;v.h1=to;  // 将 ROTATE 写入或初始化 v.kind。然后将 v.end=p 写入或初始化 v.start。然后将 length 写入或初始化 v.offset。然后将 from 写入或初始化 v.h0。然后将 to 写入或初始化 v.h1。
    return !validate(v,config.margin);  // 结束当前函数并返回 !validate(v,config.margin)。
}  // 结束此函数或控制块。
const char *Route::buildHybrid(const Node *nodes,const Config &cfg) {  // 定义函数或成员 的入口 Route::buildHybrid（路线buildHybrid），参数由此处的签名确定。
    if(cfg.policy!=1&&cfg.policy!=2&&cfg.policy!=3)return "POLICY";  // 若 cfg.policy!=1&&cfg.policy!=2&&cfg.policy!=3，立即返回 "POLICY"。
    if(!isfinite(cfg.outset)||cfg.outset<0||cfg.outset>100  // 当 !isfinite(cfg.outset)||cfg.outset<0||cfg.outset>100 成立时进入此分支，否则继续后续流程。
        ||!isfinite(cfg.margin)||cfg.margin<10||cfg.margin>40)return "POLICY_RANGE";  // 继续上一行的组合条件，本行提供 ||!isfinite(cfg.margin)||cfg.margin<10||cfg.margin>40)return "POLICY_RANGE";。
    Vec direction[MAX_NODES];float distance[MAX_NODES]={},trim[MAX_NODES]={},outsets[MAX_NODES]={};  // 将 {},trim[MAX_NODES]={},outsets[MAX_NODES]={} 写入或初始化 distance[MAX_NODES]。
    float heads[MAX_NODES]={};  // 将 {} 写入或初始化 heads[MAX_NODES]。
    for(size_t i=0;i<cfg.count;++i) {  // 按 size_t i=0;i<cfg.count;++i 组织循环的初值、继续条件和步进。
        if(!isfinite(nodes[i].p.x)||!isfinite(nodes[i].p.y))return "NODE";  // 若 !isfinite(nodes[i].p.x)||!isfinite(nodes[i].p.y)，立即返回 "NODE"。
        if(i+1<cfg.count){distance[i]=norm(nodes[i+1].p-nodes[i].p);if(distance[i]<20)return "SHORT_SEGMENT";  // 当 i+1<cfg.count 成立时进入此分支，否则继续后续流程。
            direction[i]=(nodes[i+1].p-nodes[i].p)*(1/distance[i]);}  // 将 (nodes[i+1].p-nodes[i].p)*(1/distance[i]);} 写入或初始化 direction[i]。
    }  // 结束此循环体。
    if(cfg.policy==2)stoppedHeadings(direction,cfg.count-1,cfg.startHeading,cfg.endHeading,heads);  // 当 cfg.policy==2 成立时进入此分支，否则继续后续流程。
    else heads[0]=roadHeading(cfg.startHeading,direction[0]);  // 前一个判断不成立时执行此分支。
    if(fabsf(turn(cfg.startHeading,heads[0]))>0.05f&&!addRotation(nodes[0].p,cfg.startHeading,heads[0]))return "INITIAL_TURN_SPACE";  // 若 fabsf(turn(cfg.startHeading,heads[0]))>0.05f&&!addRota…，立即返回 "INITIAL_TURN_SPACE"。
    for(size_t i=1;i+1<cfg.count;++i) {  // 按 size_t i=1;i+1<cfg.count;++i 组织循环的初值、继续条件和步进。
        if(cfg.policy!=2)heads[i]=roadHeading(heads[i-1],direction[i]);  // 当 cfg.policy!=2 成立时进入此分支，否则继续后续流程。
        float d=dot(direction[i-1],direction[i]);  // 将 dot(direction[i-1],direction[i]) 写入或初始化 d。
        if(d>0.9999f)continue;  // 若 d>0.9999f，直接进入下一轮循环。
        if(fabsf(d)>0.001f&&d>-0.9999f)return "TURN_NOT_90";  // 若 fabsf(d)>0.001f&&d>-0.9999f，立即返回 "TURN_NOT_90"。
        Vec p=nodes[i].p;  // 将 nodes[i].p 写入或初始化 p。
        bool outside=(p.x<550||p.x>1850)&&(p.y<550||p.y>1850);  // 将 (p.x<550||p.x>1850)&&(p.y<550||p.y>1850) 写入或初始化 outside。
        if((cfg.policy==1||cfg.policy==3)&&outside&&fabsf(d)<0.001f) {  // 当 (cfg.policy==1||cfg.policy==3)&&outside&&fabsf(d)<0.00… 成立时进入此分支，否则继续后续流程。
            float amount=fminf(180,fminf((distance[i-1]-20)*0.45f,(distance[i]-20)*0.45f));  // 将 fminf(180,fminf((distance[i-1]-20)*0.45f,(distance[i]-… 写入或初始化 amount。
            if(amount>=30) {  // 当 amount>=30 成立时进入此分支，否则继续后续流程。
                const float offsets[]={cfg.outset,80,100,40,20,0};  // 声明 offsets，类型和可用范围由所在声明决定。
                const float headings[]={heads[i],wrap(heads[i]+180)};  // 调用 wrap，传入 heads[i]+180)}。
                for(float offset:offsets) {  // 按 float offset:offsets 组织循环的初值、继续条件和步进。
                    // 外绕轨迹与出弯朝向联合校验，正向失败还要尝试倒退出弯。
                    for(float heading:headings) {  // 按 float heading:headings 组织循环的初值、继续条件和步进。
                        Segment curve=outsideCurve(p,direction[i-1],direction[i],amount,offset,heads[i-1],heading);  // 将 outsideCurve(p,direction[i-1],direction[i],amount,offs… 写入或初始化 curve。
                        if(!validate(curve,cfg.margin)){trim[i]=amount;outsets[i]=offset;heads[i]=heading;break;}  // 当 !validate(curve,cfg.margin) 成立时进入此分支，否则继续后续流程。
                    }  // 结束此循环体。
                    if(trim[i]>0)break;  // 若 trim[i]>0，从当前循环或分支退出。
                }  // 结束此循环体。
            }  // 结束此条件分支。
        }  // 结束此条件分支。
    }  // 结束此循环体。
    Vec cursor=nodes[0].p;  // 将 nodes[0].p 写入或初始化 cursor。
    for(size_t edge=0;edge+1<cfg.count;++edge) {  // 按 size_t edge=0;edge+1<cfg.count;++edge 组织循环的初值、继续条件和步进。
        size_t i=edge+1;Vec entry=nodes[i].p-direction[edge]*trim[i];  // 将 edge+1 写入或初始化 i。然后将 nodes[i].p-direction[edge]*trim[i] 写入或初始化 entry。
        if(norm(entry-cursor)>0.001f){  // 当 norm(entry-cursor)>0.001f 成立时进入此分支，否则继续后续流程。
            if(segmentCount>=MAX_SEGMENTS)return "PATH_TOO_LONG";  // 若 segmentCount>=MAX_SEGMENTS，立即返回 "PATH_TOO_LONG"。
            line(cursor,entry);segments[segmentCount-1].h0=segments[segmentCount-1].h1=heads[edge];  // 调用 line，传入 cursor,entry。然后将 segments[segmentCount-1].h1=heads[edge] 写入或初始化 h0。
            segments[segmentCount-1].middle=nodes[edge].middle||nodes[i].middle;  // 将 nodes[edge].middle||nodes[i].middle 写入或初始化 middle。
            segments[segmentCount-1].slowApproach=edge>=cfg.slowApproachEdge;  // 将 edge>=cfg.slowApproachEdge 写入或初始化 slowApproach。
        }  // 结束此条件分支。
        if(i+1<cfg.count) {  // 当 i+1<cfg.count 成立时进入此分支，否则继续后续流程。
            if(trim[i]>0) {  // 当 trim[i]>0 成立时进入此分支，否则继续后续流程。
                if(segmentCount>=MAX_SEGMENTS)return "PATH_TOO_LONG";  // 若 segmentCount>=MAX_SEGMENTS，立即返回 "PATH_TOO_LONG"。
                Segment curve=outsideCurve(nodes[i].p,direction[edge],direction[i],trim[i],outsets[i],heads[edge],heads[i]);  // 将 outsideCurve(nodes[i].p,direction[edge],direction[i],t… 写入或初始化 curve。
                curve.offset=length;segments[segmentCount++]=curve;length+=curve.length;cursor=curve.end;  // 将 length 写入或初始化 curve.offset。然后将 curve 写入或初始化 segments[segmentCount++]。然后把 curve.length 累加到 length（长度）。然后将 curve.end 写入或初始化 cursor。
            } else {  // 前一个判断不成立时执行此分支。
                if(dot(direction[edge],direction[i])<0.9999f  // 当 dot(direction[edge],direction[i])<0.9999f 成立时进入此分支，否则继续后续流程。
                    &&!addRotation(nodes[i].p,heads[edge],heads[i]))return "CORNER_TURN_SPACE";  // 继续上一行的组合条件，本行提供 &&!addRotation(nodes[i].p,heads[edge],heads[i]))return "CORNER_TURN_SPACE";。
                cursor=nodes[i].p;  // 将 nodes[i].p 写入或初始化 cursor。
            }  // 结束此条件分支。
        }  // 结束此条件分支。
    }  // 结束此循环体。
    if(!addRotation(nodes[cfg.count-1].p,heads[cfg.count-2],cfg.endHeading))return "FINAL_TURN_SPACE";  // 若 !addRotation(nodes[cfg.count-1].p,heads[cfg.count-2],c…，立即返回 "FINAL_TURN_SPACE"。
    for(size_t i=0;i<segmentCount;++i){const char *error=validatePhase(i);if(error)return error;}  // 按 size_t i=0;i<segmentCount;++i 组织循环的初值、继续条件和步进。
    speedCount=static_cast<size_t>(ceilf(length/5))+1;  // 将 static_cast<size_t>(ceilf(length/5))+1 写入或初始化 speedCount（速度计数）。
    if(speedCount>MAX_SPEED_SAMPLES)return "PATH_TOO_LONG";  // 若 speedCount>MAX_SPEED_SAMPLES，立即返回 "PATH_TOO_LONG"。
    spacing=length/(speedCount-1);  // 将 length/(speedCount-1) 写入或初始化 spacing。
    for(size_t i=0;i<speedCount;++i){  // 按 size_t i=0;i<speedCount;++i 组织循环的初值、继续条件和步进。
        float s=i*spacing;speeds[i]=geometricCap(s);  // 将 i*spacing 写入或初始化 s。然后将 geometricCap(s) 写入或初始化 speeds[i]。
        for(size_t j=0;j<segmentCount;++j)if(segments[j].kind==ROTATE)  // 按 size_t j=0;j<segmentCount;++j)if(segments[j].kind==ROT… 组织循环的初值、继续条件和步进。
            speeds[i]=fminf(speeds[i],sqrtf(2*cfg.accel*fabsf(s-segments[j].offset)));  // 将 fminf(speeds[i],sqrtf(2*cfg.accel*fabsf(s-segments[j].… 写入或初始化 speeds[i]。
    }  // 结束此循环体。
    speeds[0]=0;speeds[speedCount-1]=0;  // 将 0 写入或初始化 speeds[0]。然后将 0 写入或初始化 speeds[speedCount-1]。
    for(size_t i=1;i<speedCount;++i)speeds[i]=fminf(speeds[i],sqrtf(speeds[i-1]*speeds[i-1]+2*cfg.accel*spacing));  // 按 size_t i=1;i<speedCount;++i)speeds[i]=fminf(speeds[i],… 组织循环的初值、继续条件和步进。
    for(size_t i=speedCount-1;i>0;--i)speeds[i-1]=fminf(speeds[i-1],sqrtf(speeds[i]*speeds[i]+2*cfg.accel*spacing));  // 按 size_t i=speedCount-1;i>0;--i)speeds[i-1]=fminf(speeds… 组织循环的初值、继续条件和步进。
    for(size_t i=1;i<speedCount;++i){float sum=speeds[i]+speeds[i-1];if(sum<0.001f)return "NO_SPEED";plannedSeconds+=2*spacing/sum;}  // 按 size_t i=1;i<speedCount;++i 组织循环的初值、继续条件和步进。
    for(size_t i=0;i<segmentCount;++i)if(segments[i].kind==ROTATE)  // 按 size_t i=0;i<segmentCount;++i)if(segments[i].kind==ROT… 组织循环的初值、继续条件和步进。
        plannedSeconds+=fabsf(turn(segments[i].h0,segments[i].h1))/cfg.maxOmega+1;  // 把 fabsf(turn(segments[i].h0,segments[i].h1))/cfg.maxOmeg… 累加到 plannedSeconds。
    headingDelta=turn(cfg.startHeading,cfg.endHeading);return nullptr;  // 将 turn(cfg.startHeading,cfg.endHeading) 写入或初始化 headingDelta（航向Delta）。然后结束当前函数并返回 nullptr。
}  // 结束此函数或控制块。
const char *Route::validatePhase(size_t index) const {return validate(segments[index],config.margin);}  // 定义函数或成员 Route::validatePhase（路线validate阶段），参数由此处的签名确定。
size_t Route::movingPhase(float s) const {  // 定义函数或成员 的入口 Route::movingPhase（路线moving阶段），参数由此处的签名确定。
    size_t last=0;  // 将 0 写入或初始化 last（最后）。
    for(size_t i=0;i<segmentCount;++i)if(segments[i].kind!=ROTATE){last=i;if(s<segments[i].offset+segments[i].length-0.0001f)return i;}  // 按 size_t i=0;i<segmentCount;++i)if(segments[i].kind!=ROT… 组织循环的初值、继续条件和步进。
    return last;  // 结束当前函数并返回 last。
}  // 结束此函数或控制块。
Pose Route::phaseAt(size_t index,float local) const {return sample(segments[index],local);}  // 定义函数或成员 Route::phaseAt（路线阶段At），参数由此处的签名确定。
float Route::phaseHeading(size_t index,float local) const {return continuous::phaseHeading(segments[index],local);}  // 定义函数或成员 Route::phaseHeading（路线阶段航向），参数由此处的签名确定。
float Route::phaseSlope(size_t index,float local) const {  // 定义函数或成员 的入口 Route::phaseSlope（路线阶段斜率），参数由此处的签名确定。
    const auto &v=segments[index];if(v.kind!=CUBIC)return 0;  // 将 segments[index] 写入或初始化 v。然后若 v.kind!=CUBIC，立即返回 0。
    float u=clamp(local/v.length,0,1);return turn(v.h0,v.h1)*30*u*u*(1-u)*(1-u)/v.length;  // 将 clamp(local/v.length,0,1) 写入或初始化 u。然后结束当前函数并返回 turn(v.h0,v.h1)*30*u*u*(1-u)*(1-u)/v.length。
}  // 结束此函数或控制块。
float Route::nextStop(size_t index) const {  // 定义函数或成员 的入口 Route::nextStop（路线下一停止），参数由此处的签名确定。
    for(size_t i=index;i<segmentCount;++i)if(segments[i].kind==ROTATE)return segments[i].offset;  // 按 size_t i=index;i<segmentCount;++i)if(segments[i].kind=… 组织循环的初值、继续条件和步进。
    return length;  // 结束当前函数并返回 length。
}  // 结束此函数或控制块。
Output Tracker::updateHybrid(const Route &route,float actual,float dt,uint32_t sampleMs) {  // 定义函数或成员 的入口 Tracker::updateHybrid，参数由此处的签名确定。
    Output out;const Config &c=route.config;  // 声明 out，类型和可用范围由所在声明决定。然后将 route.config 写入或初始化 c。
    if(!movingBlock&&(!isfinite(dt)||dt<=0||dt>0.120f)){out.failure="CONTROL_OVERRUN";return out;}  // 当 !movingBlock&&(!isfinite(dt)||dt<=0||dt>0.120f) 成立时进入此分支，否则继续后续流程。
    // s 是整条路径的累计沿程。
    // 平移段之间靠段末沿程换段，后面若是原地转向就把 s 对齐到边界，并把平移速度置零。
    while(phaseIndex<route.segmentCount&&route.segments[phaseIndex].kind!=ROTATE  // 当 phaseIndex<route.segmentCount&&route.segments[phaseInd… 成立时继续循环。
        &&s>=route.segments[phaseIndex].offset+route.segments[phaseIndex].length-0.5f) {  // 继续上一行的组合条件，本行提供 &&s>=route.segments[phaseIndex].offset+route.segments[phaseIndex].length-0.5f) {。
        if(phaseIndex+1<route.segmentCount&&route.segments[phaseIndex+1].kind==ROTATE){s=route.segments[phaseIndex+1].offset;speed=0;}  // 当 phaseIndex+1<route.segmentCount&&route.segments[phaseI… 成立时进入此分支，否则继续后续流程。
        ++phaseIndex;stableSeconds=turnSeconds=badSeconds=0;rateReady=false;  // 续写当前声明或运算，提供 ++phaseIndex;。然后将 turnSeconds=badSeconds=0 写入或初始化 stableSeconds（稳定Seconds）。然后将 false 写入或初始化 rateReady（rate就绪）。
        phaseStopped=false;quiet.reset();if(!movingBlock)hold.reset();  // 将 false 写入或初始化 phaseStopped（阶段已停止）。然后调用 quiet.reset。然后当 !movingBlock 成立时进入此分支，否则继续后续流程。
    }  // 结束此初始化列表或局部作用域。
    if(phaseIndex>=route.segmentCount){out.reference=c.endHeading;out.error=turn(actual,out.reference);out.finished=true;return out;}  // 当 phaseIndex>=route.segmentCount 成立时进入此分支，否则继续后续流程。
    if(movingBlock&&route.segments[phaseIndex].kind==ROTATE){out.finished=true;return out;}  // 当 movingBlock&&route.segments[phaseIndex].kind==ROTATE 成立时进入此分支，否则继续后续流程。
    const Segment &phase=route.segments[phaseIndex];bool rotating=phase.kind==ROTATE;  // 将 route.segments[phaseIndex] 写入或初始化 phase（阶段）。然后将 phase.kind==ROTATE 写入或初始化 rotating。
    float reference=rotating?phase.h1:route.phaseHeading(phaseIndex,s-phase.offset);  // 将 rotating?phase.h1:route.phaseHeading(phaseIndex,s-phas… 写入或初始化 reference。
    out.reference=reference;out.error=turn(actual,reference);  // 将 reference 写入或初始化 out.reference。然后将 turn(actual,reference) 写入或初始化 out.error。
    float targetOmega=0;  // 将 0 写入或初始化 targetOmega（目标Omega）。
    if(rotating) {  // 当 rotating 成立时进入此分支，否则继续后续流程。
        speed=0;turnSeconds+=dt;finishing=phaseIndex+1==route.segmentCount;  // 将 0 写入或初始化 speed（速度）。然后把 dt 累加到 turnSeconds。然后将 phaseIndex+1==route.segmentCount 写入或初始化 finishing。
        if(requireInertial) {  // 当 requireInertial 成立时进入此分支，否则继续后续流程。
            // 惯性静止判据只接收零速命令之后的样本，避免上一阶段缓存的姿态提前满足静止条件。
            bool zero=zeroCommanded&&static_cast<int32_t>(motionSample.receivedAtMs-zeroSince)>=0;  // 将 zeroCommanded&&static_cast<int32_t>(motionSample.recei… 写入或初始化 zero（零点）。
            if(!phaseStopped) {  // 当 !phaseStopped 成立时进入此分支，否则继续后续流程。
                stage="SETTLE_BEFORE_TURN";omega=0;  // 将 "SETTLE_BEFORE_TURN" 写入或初始化 stage（阶段）。然后将 0 写入或初始化 omega。
                out.reference=phase.h0;out.error=turn(actual,phase.h0);  // 将 phase.h0 写入或初始化 out.reference。然后将 turn(actual,phase.h0) 写入或初始化 out.error。
                if(quiet.update(motionSample,zero)) {  // 当 quiet.update(motionSample,zero) 成立时进入此分支，否则继续后续流程。
                    if(fabsf(turn(actual,phase.h1))<=1) {  // 当 fabsf(turn(actual,phase.h1))<=1 成立时进入此分支，否则继续后续流程。
                        ++phaseIndex;quiet.reset();hold.reset();turnSeconds=0;  // 续写当前声明或运算，提供 ++phaseIndex;。然后调用 quiet.reset。然后调用 hold.reset。然后将 0 写入或初始化 turnSeconds。
                        if(phaseIndex==route.segmentCount)out.finished=true;  // 当 phaseIndex==route.segmentCount 成立时进入此分支，否则继续后续流程。
                    } else {phaseStopped=true;quiet.reset();turnSeconds=0;}  // 前一个判断不成立时执行此分支。
                }  // 结束此条件分支。
                else if(turnSeconds>3)out.failure="SETTLE_TIMEOUT";  // 仅在先前条件均不成立时，当 turnSeconds>3 成立时进入此分支，否则继续后续流程。
                return out;  // 结束当前函数并返回 out。
            }  // 结束此条件分支。
            stage=finishing?"FINAL_HEADING":"TURN";  // 将 finishing?"FINAL_HEADING":"TURN" 写入或初始化 stage（阶段）。
            if(fabsf(out.error)<=1) {  // 当 fabsf(out.error)<=1 成立时进入此分支，否则继续后续流程。
                omega=0;stage="SETTLE_AFTER_TURN";  // 将 0 写入或初始化 omega。然后将 "SETTLE_AFTER_TURN" 写入或初始化 stage（阶段）。
                if(quiet.update(motionSample,zero)) {  // 当 quiet.update(motionSample,zero) 成立时进入此分支，否则继续后续流程。
                    ++phaseIndex;turnSeconds=stableSeconds=0;rateReady=false;phaseStopped=false;  // 续写当前声明或运算，提供 ++phaseIndex;。然后将 stableSeconds=0 写入或初始化 turnSeconds。然后将 false 写入或初始化 rateReady（rate就绪）。然后将 false 写入或初始化 phaseStopped（阶段已停止）。
                    quiet.reset();hold.reset();  // 调用 quiet.reset，传入 );hold.reset(。
                    if(phaseIndex==route.segmentCount)out.finished=true;  // 当 phaseIndex==route.segmentCount 成立时进入此分支，否则继续后续流程。
                    return out;  // 结束当前函数并返回 out。
                }  // 结束此条件分支。
                if(turnSeconds>fabsf(turn(phase.h0,phase.h1))/fminf(45,c.maxOmega)*2+3)out.failure="TURN_TIMEOUT";  // 当 turnSeconds>fabsf(turn(phase.h0,phase.h1))/fminf(45,c.… 成立时进入此分支，否则继续后续流程。
                return out;  // 结束当前函数并返回 out。
            }  // 结束此条件分支。
            quiet.reset();  // 调用 quiet.reset。
        }  // 结束此条件分支。
        // 同一个姿态样本可被多个控制周期重复读到。
        // 角速度和稳定时间只在样本时间变化时更新。
        if(sampleMs!=previousSample) {  // 当 sampleMs!=previousSample 成立时进入此分支，否则继续后续流程。
            float sampleDt=(sampleMs-previousSample)/1000.0f;  // 将 (sampleMs-previousSample)/1000.0f 写入或初始化 sampleDt（采样时间差）。
            float rate=sampleDt>0&&sampleDt<=0.2f?fabsf(turn(previousActual,actual))/sampleDt:1000;  // 将 sampleDt>0&&sampleDt<=0.2f?fabsf(turn(previousActual,a… 写入或初始化 rate。
            if(rateReady&&fabsf(out.error)<=1&&rate<=3)stableSeconds+=fminf(sampleDt,0.12f);else stableSeconds=0;  // 当 rateReady&&fabsf(out.error)<=1&&rate<=3 成立时进入此分支，否则继续后续流程。
            rateReady=true;previousSample=sampleMs;previousActual=actual;  // 将 true 写入或初始化 rateReady（rate就绪）。然后将 sampleMs 写入或初始化 previousSample（上一采样）。然后将 actual 写入或初始化 previousActual（上一实测）。
        }  // 结束此条件分支。
        if(!requireInertial&&stableSeconds>=0.12f){++phaseIndex;omega=0;stableSeconds=turnSeconds=0;rateReady=false;hold.reset();  // 当 !requireInertial&&stableSeconds>=0.12f 成立时进入此分支，否则继续后续流程。
            if(phaseIndex==route.segmentCount)out.finished=true;return out;}  // 当 phaseIndex==route.segmentCount 成立时进入此分支，否则继续后续流程。
        float deadline=fabsf(turn(phase.h0,phase.h1))/fminf(45,c.maxOmega)*2+3;  // 将 fabsf(turn(phase.h0,phase.h1))/fminf(45,c.maxOmega)*2+3 写入或初始化 deadline（截止时间）。
        if(turnSeconds>deadline){out.failure="TURN_TIMEOUT";return out;}  // 当 turnSeconds>deadline 成立时进入此分支，否则继续后续流程。
        targetOmega=clamp(c.kp*out.error,-fminf(45,c.maxOmega),fminf(45,c.maxOmega));  // 将 clamp(c.kp*out.error,-fminf(45,c.maxOmega),fminf(45,c.… 写入或初始化 targetOmega（目标Omega）。
    } else {  // 前一个判断不成立时执行此分支。
        stage="HEADING_HOLD";  // 将 "HEADING_HOLD" 写入或初始化 stage（阶段）。
        finishing=false;  // 将 false 写入或初始化 finishing。
        badSeconds=fabsf(out.error)>5?badSeconds+dt:0;  // 将 fabsf(out.error)>5?badSeconds+dt:0 写入或初始化 badSeconds。
        if(!movingBlock&&badSeconds>=0.2f){out.failure="HEADING_ERROR";return out;}  // 当 !movingBlock&&badSeconds>=0.2f 成立时进入此分支，否则继续后续流程。
        float remaining=fmaxf(0,route.nextStop(phaseIndex)-s);  // 将 fmaxf(0,route.nextStop(phaseIndex)-s) 写入或初始化 remaining（剩余）。
        // 目标速度同时受路径限速以及到下一停车点的剩余距离约束。
        // 式中的 dt 项把当前控制周期也纳入制动估算。
        float target=fminf(route.speedCap(s),sqrtf(2*c.accel*remaining+c.accel*c.accel*dt*dt)-c.accel*dt);  // 将 fminf(route.speedCap(s),sqrtf(2*c.accel*remaining+c.ac… 写入或初始化 target（目标）。
        if(!movingBlock)target*=clamp(1-(fabsf(out.error)-1)/4,0.2f,1);  // 当 !movingBlock 成立时进入此分支，否则继续后续流程。
        float acceleration=c.accel;  // 将 c.accel 写入或初始化 acceleration（加速度）。
        if(phase.kind==CUBIC) {  // 当 phase.kind==CUBIC 成立时进入此分支，否则继续后续流程。
            float slope=fabsf(turn(phase.h0,phase.h1))*1.875f/phase.length;  // 将 fabsf(turn(phase.h0,phase.h1))*1.875f/phase.length 写入或初始化 slope（斜率）。
            if(slope>0.000001f)acceleration=fminf(acceleration,(c.omegaAccel/3)/slope);  // 当 slope>0.000001f 成立时进入此分支，否则继续后续流程。
        }  // 结束此条件分支。
        speed=clamp(target,fmaxf(0,speed-acceleration*dt),speed+acceleration*dt);  // 将 clamp(target,fmaxf(0,speed-acceleration*dt),speed+acce… 写入或初始化 speed（速度）。
        float correction=hold.update(actual,out.error,sampleMs,c.kp,c.correction);  // 将 hold.update(actual,out.error,sampleMs,c.kp,c.correctio… 写入或初始化 correction（修正量）。
        // 航向斜率乘沿程速度提供前馈角速度，再叠加航向保持量。
        // 后面的 omega 计算另外限制每周期角速度变化。
        targetOmega=clamp(route.phaseSlope(phaseIndex,s-phase.offset)*speed+correction,-c.maxOmega,c.maxOmega);  // 将 clamp(route.phaseSlope(phaseIndex,s-phase.offset)*spee… 写入或初始化 targetOmega（目标Omega）。
    }  // 结束此条件分支。
    omega=clamp(targetOmega,omega-c.omegaAccel*dt,omega+c.omegaAccel*dt);  // 将 clamp(targetOmega,omega-c.omegaAccel*dt,omega+c.omegaA… 写入或初始化 omega。
    Pose desired=route.phaseAt(phaseIndex,s-phase.offset);  // 将 route.phaseAt(phaseIndex,s-phase.offset) 写入或初始化 desired（期望）。
    Vec velocity=desired.tangent*speed;  // 将 desired.tangent*speed 写入或初始化 velocity（速度）。
    if(movingBlock&&!rotating) {  // 当 movingBlock&&!rotating 成立时进入此分支，否则继续后续流程。
        // 编码器/H30里程计的横向闭环；不把工位视觉的绝对定位职责交给IMU。
        Vec normal{-desired.tangent.y,desired.tangent.x};  // 声明 normal（法向），类型和可用范围由所在声明决定。
        float lateral=dot(desired.p-estimated,normal);  // 将 dot(desired.p-estimated,normal) 写入或初始化 lateral（横向）。
        float reserve=fmaxf(40,speed*.25f);  // 将 fmaxf(40,speed*.25f) 写入或初始化 reserve。
        velocity=velocity+normal*clamp(lateral*2,-reserve,reserve);  // 将 velocity+normal*clamp(lateral*2,-reserve,reserve) 写入或初始化 velocity（速度）。
    }  // 结束此条件分支。
    Vec body=rotating?Vec{}:worldToBody(velocity,actual);  // 将 rotating?Vec{}:worldToBody(velocity,actual) 写入或初始化 body（车体）。
    out.wheels=wheels(body,omega,c);return out;  // 将 wheels(body,omega,c) 写入或初始化 out.wheels。然后结束当前函数并返回 out。
}  // 结束此函数或控制块。
} // namespace continuous
