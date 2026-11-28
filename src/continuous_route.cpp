// 根据节点和圆角构造连续路径并计算跟踪指令。
// 用沿程距离查找路径位置、切向和航向，用速度曲线协调弯角与停车点。

#include "continuous_route.h"  // 引入 continuous_route.h 中的声明和定义。

namespace continuous {  // 定义 continuous 的类型成员或作用域。
/**
 * @brief 向路径追加起点a到终点b的直线段，更新累计行程。
 */
void Route::line(Vec a,Vec b) {  // 定义函数或成员 的入口 Route::line（路线line），参数由此处的签名确定。
    float n=norm(b-a); if(n<0.001f)return;  // 将 norm(b-a) 写入或初始化 n。然后当 n<0.001f 成立时进入此分支，否则继续后续流程。
    Segment &v=segments[segmentCount++]; v=Segment{};  // 将 segments[segmentCount++]; v=Segment{} 写入或初始化 v。
    v.start=a;v.end=b;v.offset=length;v.length=n;length+=n;  // 将 a 写入或初始化 v.start。然后将 b 写入或初始化 v.end。然后将 length 写入或初始化 v.offset。然后将 n 写入或初始化 v.length。然后把 n 累加到 length（长度）。
}  // 结束此函数或控制块。
/**
 * @brief 根据节点及配置构建路径段和速度规划；成功返回nullptr，否则为几何或配置错误原因。
 */
const char *Route::build(const Node *nodes,const Config &cfg) {  // 定义函数或成员 的入口 Route::build（路线build），参数由此处的签名确定。
    config=cfg;segmentCount=0;speedCount=0;length=0;plannedSeconds=0;  // 将 cfg 写入或初始化 config（配置）。然后将 0 写入或初始化 segmentCount（segment计数）。然后将 0 写入或初始化 speedCount（速度计数）。然后将 0 写入或初始化 length（长度）。然后将 0 写入或初始化 plannedSeconds。
    if(!nodes||cfg.count<2||cfg.count>MAX_NODES)return "COUNT";  // 若 !nodes||cfg.count<2||cfg.count>MAX_NODES，立即返回 "COUNT"。
    const float values[]={cfg.startHeading,cfg.endHeading,cfg.maxRpm,cfg.accel,cfg.lateralAccel,  // 续列跨行签名或声明中的参数，类型与名称为 const float values[]={cfg.startHeading,cfg.endHeading,…。
        cfg.lateralPpm,cfg.forwardPpm,cfg.rotationPpd,cfg.kp,cfg.maxOmega,cfg.correction};  // 续写当前声明或运算，提供 cfg.lateralPpm,cfg.forwardPpm,cfg.rotationPpd,cfg.kp,c…。
    for(float v:values)if(!isfinite(v))return "RANGE";  // 按 float v:values)if(!isfinite(v))return "RANGE"; 组织循环的初值、继续条件和步进。
    if(cfg.startHeading<0||cfg.startHeading>=360||cfg.endHeading<0||cfg.endHeading>=360  // 当 cfg.startHeading<0||cfg.startHeading>=360||cfg.endHead… 成立时进入此分支，否则继续后续流程。
        ||cfg.maxRpm<1||cfg.maxRpm>5000||cfg.maxRpm!=floorf(cfg.maxRpm)||cfg.accel<1||cfg.accel>((cfg.policy==2||cfg.policy==3)?4000:2000)  // 继续上一行的组合条件，本行提供 ||cfg.maxRpm<1||cfg.maxRpm>5000||cfg.maxRpm!=floorf(cfg.maxRpm)||cfg.accel<1||cfg.accel>(…。
        ||cfg.lateralAccel<1||cfg.lateralAccel>(cfg.policy==3?4000:3000)||cfg.lateralPpm<0.01f||cfg.lateralPpm>1000  // 继续上一行的组合条件，本行提供 ||cfg.lateralAccel<1||cfg.lateralAccel>(cfg.policy==3?4000:3000)||cfg.lateralPpm<0.01f||c…。
        ||cfg.forwardPpm<0.01f||cfg.forwardPpm>1000||cfg.rotationPpd<=0||cfg.rotationPpd>10000  // 继续上一行的组合条件，本行提供 ||cfg.forwardPpm<0.01f||cfg.forwardPpm>1000||cfg.rotationPpd<=0||cfg.rotationPpd>10000。
        ||cfg.kp<0.1f||cfg.kp>10||cfg.maxOmega<5||cfg.maxOmega>(cfg.policy==3?240:90)  // 继续上一行的组合条件，本行提供 ||cfg.kp<0.1f||cfg.kp>10||cfg.maxOmega<5||cfg.maxOmega>(cfg.policy==3?240:90)。
        ||cfg.correction<1||cfg.correction>=cfg.maxOmega)return "RANGE";  // 继续上一行的组合条件，本行提供 ||cfg.correction<1||cfg.correction>=cfg.maxOmega)return "RANGE";。
    if(cfg.policy)return buildHybrid(nodes,cfg);  // 若 cfg.policy，立即返回 buildHybrid(nodes,cfg)。
    Vec direction[MAX_NODES];float distance[MAX_NODES]={},trim[MAX_NODES]={};  // 将 {},trim[MAX_NODES]={} 写入或初始化 distance[MAX_NODES]。
    for(size_t i=0;i<cfg.count;++i) {  // 按 size_t i=0;i<cfg.count;++i 组织循环的初值、继续条件和步进。
        if(!isfinite(nodes[i].p.x)||!isfinite(nodes[i].p.y)||!isfinite(nodes[i].radius)  // 当 !isfinite(nodes[i].p.x)||!isfinite(nodes[i].p.y)||!isf… 成立时进入此分支，否则继续后续流程。
            ||nodes[i].radius<0||nodes[i].radius>1000)return "NODE";  // 继续上一行的组合条件，本行提供 ||nodes[i].radius<0||nodes[i].radius>1000)return "NODE";。
        if(i+1<cfg.count) {  // 当 i+1<cfg.count 成立时进入此分支，否则继续后续流程。
            distance[i]=norm(nodes[i+1].p-nodes[i].p);  // 将 norm(nodes[i+1].p-nodes[i].p) 写入或初始化 distance[i]。
            if(distance[i]<20)return "SHORT_SEGMENT";  // 若 distance[i]<20，立即返回 "SHORT_SEGMENT"。
            direction[i]=(nodes[i+1].p-nodes[i].p)*(1/distance[i]);  // 将 (nodes[i+1].p-nodes[i].p)*(1/distance[i]) 写入或初始化 direction[i]。
        }  // 结束此条件分支。
    }  // 结束此循环体。
    if(nodes[0].radius!=0||nodes[cfg.count-1].radius!=0)return "END_RADIUS";  // 若 nodes[0].radius!=0||nodes[cfg.count-1].radius!=0，立即返回 "END_RADIUS"。
    float minRadius=100000;  // 将 100000 写入或初始化 minRadius（最小Radius）。
    for(size_t i=1;i+1<cfg.count;++i) {  // 按 size_t i=1;i+1<cfg.count;++i 组织循环的初值、继续条件和步进。
        float d=dot(direction[i-1],direction[i]);  // 将 dot(direction[i-1],direction[i]) 写入或初始化 d。
        if(d>0.9999f)continue;  // 若 d>0.9999f，直接进入下一轮循环。
        if(fabsf(d)>0.001f)return "TURN_NOT_90";  // 若 fabsf(d)>0.001f，立即返回 "TURN_NOT_90"。
        if(nodes[i].radius<20)return "CORNER_NOT_ALLOWED";  // 若 nodes[i].radius<20，立即返回 "CORNER_NOT_ALLOWED"。
        trim[i]=nodes[i].radius;minRadius=fminf(minRadius,trim[i]);  // 将 nodes[i].radius 写入或初始化 trim[i]。然后将 fminf(minRadius,trim[i]) 写入或初始化 minRadius（最小Radius）。
    }  // 结束此循环体。
    for(size_t i=0;i+1<cfg.count;++i)  // 按 size_t i=0;i+1<cfg.count;++i 组织循环的初值、继续条件和步进。
        if(trim[i]+trim[i+1]>distance[i]-20)return "RADIUS_OVERLAP";  // 若 trim[i]+trim[i+1]>distance[i]-20，立即返回 "RADIUS_OVERLAP"。
    Vec cursor=nodes[0].p;  // 将 nodes[0].p 写入或初始化 cursor。
    for(size_t i=1;i<cfg.count;++i) {  // 按 size_t i=1;i<cfg.count;++i 组织循环的初值、继续条件和步进。
        Vec entry=nodes[i].p-direction[i-1]*trim[i];line(cursor,entry);  // 将 nodes[i].p-direction[i-1]*trim[i] 写入或初始化 entry。然后调用 line，传入 cursor,entry。
        if(trim[i]>0) {  // 当 trim[i]>0 成立时进入此分支，否则继续后续流程。
            Segment &v=segments[segmentCount++];v=Segment{};  // 将 segments[segmentCount++];v=Segment{} 写入或初始化 v。
            v.start=entry;v.end=nodes[i].p+direction[i]*trim[i];  // 将 entry 写入或初始化 v.start。然后将 nodes[i].p+direction[i]*trim[i] 写入或初始化 v.end。
            v.center=entry+direction[i]*trim[i];v.radius=trim[i];  // 将 entry+direction[i]*trim[i] 写入或初始化 v.center。然后将 trim[i] 写入或初始化 v.radius。
            v.angle=atan2f(entry.y-v.center.y,entry.x-v.center.x);  // 将 atan2f(entry.y-v.center.y,entry.x-v.center.x) 写入或初始化 v.angle。
            v.sweep=cross(direction[i-1],direction[i])>0?PI_F/2:-PI_F/2;  // 将 cross(direction[i-1],direction[i])>0?PI_F/2:-PI_F/2 写入或初始化 v.sweep。
            v.length=trim[i]*PI_F/2;v.offset=length;length+=v.length;  // 将 trim[i]*PI_F/2 写入或初始化 v.length。然后将 length 写入或初始化 v.offset。然后把 v.length 累加到 length（长度）。
            cursor=v.end;  // 将 v.end 写入或初始化 cursor。
        } else cursor=nodes[i].p;  // 前一个判断不成立时执行此分支。
    }  // 结束此循环体。
    headingDelta=turn(cfg.startHeading,cfg.endHeading);  // 将 turn(cfg.startHeading,cfg.endHeading) 写入或初始化 headingDelta（航向Delta）。
    // 空间与姿态轮廓验算，最多 5mm / 1deg，包含端点。
    float ds=fminf(5,minRadius*PI_F/180);  // 将 fminf(5,minRadius*PI_F/180) 写入或初始化 ds。
    if(fabsf(headingDelta)>0.001f)ds=fminf(ds,length/(fabsf(headingDelta)*1.875f));  // 当 fabsf(headingDelta)>0.001f 成立时进入此分支，否则继续后续流程。
    if(ds<=0||length/ds>200000)return "PATH_TOO_LONG";  // 若 ds<=0||length/ds>200000，立即返回 "PATH_TOO_LONG"。
    size_t checks=static_cast<size_t>(ceilf(length/ds));  // 将 static_cast<size_t>(ceilf(length/ds)) 写入或初始化 checks。
    for(size_t i=0;i<=checks;++i)if(!inside(length*i/checks))return "BOUNDARY";  // 按 size_t i=0;i<=checks;++i)if(!inside(length*i/checks))r… 组织循环的初值、继续条件和步进。
    speedCount=static_cast<size_t>(ceilf(length/5))+1;  // 将 static_cast<size_t>(ceilf(length/5))+1 写入或初始化 speedCount（速度计数）。
    if(speedCount>MAX_SPEED_SAMPLES)return "PATH_TOO_LONG";  // 若 speedCount>MAX_SPEED_SAMPLES，立即返回 "PATH_TOO_LONG"。
    spacing=length/(speedCount-1);  // 将 length/(speedCount-1) 写入或初始化 spacing。
    for(size_t i=0;i<speedCount;++i)speeds[i]=geometricCap(i*spacing);  // 按 size_t i=0;i<speedCount;++i)speeds[i]=geometricCap(i*s… 组织循环的初值、继续条件和步进。
    speeds[0]=0;speeds[speedCount-1]=0;  // 将 0 写入或初始化 speeds[0]。然后将 0 写入或初始化 speeds[speedCount-1]。
    for(size_t i=1;i<speedCount;++i)  // 按 size_t i=1;i<speedCount;++i 组织循环的初值、继续条件和步进。
        speeds[i]=fminf(speeds[i],sqrtf(speeds[i-1]*speeds[i-1]+2*cfg.accel*spacing));  // 将 fminf(speeds[i],sqrtf(speeds[i-1]*speeds[i-1]+2*cfg.ac… 写入或初始化 speeds[i]。
    for(size_t i=speedCount-1;i>0;--i)  // 按 size_t i=speedCount-1;i>0;--i 组织循环的初值、继续条件和步进。
        speeds[i-1]=fminf(speeds[i-1],sqrtf(speeds[i]*speeds[i]+2*cfg.accel*spacing));  // 将 fminf(speeds[i-1],sqrtf(speeds[i]*speeds[i]+2*cfg.acce… 写入或初始化 speeds[i-1]。
    for(size_t i=1;i<speedCount;++i) {  // 按 size_t i=1;i<speedCount;++i 组织循环的初值、继续条件和步进。
        float sum=speeds[i-1]+speeds[i];if(sum<0.01f)return "NO_SPEED";  // 将 speeds[i-1]+speeds[i] 写入或初始化 sum。然后若 sum<0.01f，立即返回 "NO_SPEED"。
        plannedSeconds+=2*spacing/sum;  // 把 2*spacing/sum 累加到 plannedSeconds。
    }  // 结束此循环体。
    // 初始位置和当前导航位置是否一致，由运行时再次校验。
    return nullptr;  // 结束当前函数并返回 nullptr。
}  // 结束此函数或控制块。
/**
 * @brief 查询累计路径距离s(mm)处的位置、切向及曲率。
 */
// 沿程查询返回位置和切向等几何量。
// 速度和期望航向通过独立接口查询，跟踪时通常在同一个 s 上读取这些量。
Pose Route::at(float s) const {  // 定义函数或成员 的入口 Route::at（路线at），参数由此处的签名确定。
    if(config.policy){size_t i=movingPhase(s);return phaseAt(i,s-segments[i].offset);}  // 当 config.policy 成立时进入此分支，否则继续后续流程。
    s=clamp(s,0,length);size_t i=0;  // 将 clamp(s,0,length) 写入或初始化 s。然后将 0 写入或初始化 i。
    while(i+1<segmentCount&&s>=segments[i].offset+segments[i].length)++i;  // 当 i+1<segmentCount&&s>=segments[i].offset+segments[i].le… 成立时继续循环。
    const Segment &v=segments[i];float t=clamp((s-v.offset)/v.length,0,1);Pose p;  // 将 segments[i] 写入或初始化 v。然后将 clamp((s-v.offset)/v.length,0,1) 写入或初始化 t。然后声明 p，类型和可用范围由所在声明决定。
    if(v.radius==0){p.tangent=(v.end-v.start)*(1/v.length);p.p=v.start+(v.end-v.start)*t;}  // 当 v.radius==0 成立时进入此分支，否则继续后续流程。
    else {float a=v.angle+v.sweep*t,sign=v.sweep>0?1:-1;  // 前一个判断不成立时执行此分支。
        p.p=v.center+Vec(cosf(a),sinf(a))*v.radius;  // 将 v.center+Vec(cosf(a),sinf(a))*v.radius 写入或初始化 p.p。
        p.tangent=Vec(-sinf(a)*sign,cosf(a)*sign);p.curvature=sign/v.radius;}  // 将 Vec(-sinf(a)*sign,cosf(a)*sign);p.curvature=sign/v.rad… 写入或初始化 p.tangent。
    return p;  // 结束当前函数并返回 p。
}  // 结束此函数或控制块。
/**
 * @brief 查询累计路径距离s(mm)处的计划地图航向(deg)。
 */
float Route::heading(float s) const {  // 定义函数或成员 的入口 Route::heading（路线航向），参数由此处的签名确定。
    if(config.policy){size_t i=movingPhase(s);return phaseHeading(i,s-segments[i].offset);}  // 当 config.policy 成立时进入此分支，否则继续后续流程。
    float u=clamp(s/length,0,1);return wrap(config.startHeading+headingDelta*u*u*u*(10+u*(-15+6*u)));  // 将 clamp(s/length,0,1) 写入或初始化 u。然后结束当前函数并返回 wrap(config.startHeading+headingDelta*u*u*u*(10+u*(-15…。
}  // 结束此函数或控制块。
/**
 * @brief 查询路径航向相对行程的变化率(deg/mm)。
 */
float Route::headingSlope(float s) const {  // 定义函数或成员 的入口 Route::headingSlope（路线航向斜率），参数由此处的签名确定。
    if(config.policy){size_t i=movingPhase(s);return phaseSlope(i,s-segments[i].offset);}  // 当 config.policy 成立时进入此分支，否则继续后续流程。
    float u=clamp(s/length,0,1);return headingDelta*30*u*u*(1-u)*(1-u)/length;  // 将 clamp(s/length,0,1) 写入或初始化 u。然后结束当前函数并返回 headingDelta*30*u*u*(1-u)*(1-u)/length。
}  // 结束此函数或控制块。
/**
 * @brief 检查路径距离s处的车体四角是否位于旧模型的场地边界内。
 */
bool Route::inside(float s) const {  // 定义函数或成员 的入口 Route::inside（路线inside），参数由此处的签名确定。
    Pose p=at(s);  // 将 at(s) 写入或初始化 p。
    for(int x:{-1,1})for(int y:{-1,1}) {  // 按 int x: 组织循环的初值、继续条件和步进。
        Vec q=p.p+bodyToWorld(Vec(x*150.0f,y*150.0f),heading(s));  // 将 p.p+bodyToWorld(Vec(x*150.0f,y*150.0f),heading(s)) 写入或初始化 q。
        if(q.x<20||q.y<20||q.x>2380||q.y>2380)return false;  // 若 q.x<20||q.y<20||q.x>2380||q.y>2380，立即返回 false。
    }  // 结束此循环体。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。
/**
 * @brief 结合四轮最大转速、曲率及航向变化计算当前位置的几何速度上限(mm/s)。
 */
float Route::geometricCap(float s) const {  // 定义函数或成员 的入口 Route::geometricCap（路线geometricCap），参数由此处的签名确定。
    Pose p=at(s);Vec body=worldToBody(p.tangent,heading(s));  // 将 at(s) 写入或初始化 p。然后将 worldToBody(p.tangent,heading(s)) 写入或初始化 body（车体）。
    float px=body.x*config.lateralPpm,py=body.y*config.forwardPpm,pr=headingSlope(s)*config.rotationPpd;  // 将 body.x*config.lateralPpm,py=body.y*config.forwardPpm,p… 写入或初始化 px。
    const float rates[]={-px+py+pr,-px-py+pr,px+py+pr,px-py+pr};  // 声明 rates，类型和可用范围由所在声明决定。
    float peak=0;for(float v:rates)peak=fmaxf(peak,fabsf(v)*60/3200);  // 将 0 写入或初始化 peak。然后按 float v:rates)peak=fmaxf(peak,fabsf(v)*60/3200); 组织循环的初值、继续条件和步进。
    float cap=config.maxRpm*0.75f/fmaxf(peak,0.0001f); // 给航向纠偏预留轮速
    if(fabsf(p.curvature)>0)cap=fminf(cap,sqrtf(config.lateralAccel/fabsf(p.curvature)));  // 当 fabsf(p.curvature)>0 成立时进入此分支，否则继续后续流程。
    if(fabsf(headingSlope(s))>0.000001f)  // 当 fabsf(headingSlope(s))>0.000001f 成立时进入此分支，否则继续后续流程。
        cap=fminf(cap,(config.maxOmega-config.correction)/fabsf(headingSlope(s)));  // 将 fminf(cap,(config.maxOmega-config.correction)/fabsf(he… 写入或初始化 cap。
    if(config.policy) {  // 当 config.policy 成立时进入此分支，否则继续后续流程。
        const Segment &phase=segments[movingPhase(s)];  // 将 segments[movingPhase(s)] 写入或初始化 phase（阶段）。
        if(phase.kind==CUBIC) {  // 当 phase.kind==CUBIC 成立时进入此分支，否则继续后续流程。
            // 五次姿态插值的 |d²theta/ds²| 峰值为 10/sqrt(3)*|delta|/L²。
            // 角加速度预算：几何变化1/2、平移加速1/3，余量用于纠偏。
            float peak=fabsf(turn(phase.h0,phase.h1))*5.773503f/(phase.length*phase.length);  // 将 fabsf(turn(phase.h0,phase.h1))*5.773503f/(phase.length… 写入或初始化 peak。
            if(peak>0.000001f)cap=fminf(cap,sqrtf(config.omegaAccel*.5f/peak));  // 当 peak>0.000001f 成立时进入此分支，否则继续后续流程。
        }  // 结束此条件分支。
    }  // 结束此条件分支。
    return cap;  // 结束当前函数并返回 cap。
}  // 结束此函数或控制块。
/**
 * @brief 查询路径距离s(mm)处结合几何、加速度及停止约束后的速度上限(mm/s)。
 */
float Route::speedCap(float s) const {  // 定义函数或成员 的入口 Route::speedCap（路线速度Cap），参数由此处的签名确定。
    size_t next=static_cast<size_t>(s/spacing)+1;if(next>=speedCount)next=speedCount-1;  // 将 static_cast<size_t>(s/spacing)+1 写入或初始化 next（下一）。然后当 next>=speedCount 成立时进入此分支，否则继续后续流程。
    float remain=fmaxf(0,next*spacing-s);  // 将 fmaxf(0,next*spacing-s) 写入或初始化 remain。
    return fminf(geometricCap(s),sqrtf(speeds[next]*speeds[next]+2*config.accel*remain));  // 结束当前函数并返回 fminf(geometricCap(s),sqrtf(speeds[next]*speeds[next]+…。
}  // 结束此函数或控制块。
/**
 * @brief 将车体平移速度(mm/s)及角速度(deg/s)换算为四轮RPM，并按配置上限等比例缩放。
 */
// 把车体坐标的平移速度和旋转分量映射到四个驱动轮。
// 排查前后、横移和原地转向的轮向时从这里对照实际接线。
WheelCommand wheels(Vec body,float omega,const Config &cfg) {  // 定义函数或成员 的入口 wheels（四轮），参数由此处的签名确定。
    WheelCommand out;  // 声明 out，类型和可用范围由所在声明决定。
    float px=body.x*cfg.lateralPpm,py=body.y*cfg.forwardPpm,pr=omega*cfg.rotationPpd;  // 将 body.x*cfg.lateralPpm,py=body.y*cfg.forwardPpm,pr=omeg… 写入或初始化 px。
    float values[]={-px+py+pr,-px-py+pr,px+py+pr,px-py+pr};  // 声明 values（参数值），类型和可用范围由所在声明决定。
    float peak=0;for(float &v:values){v*=60/3200.0f;peak=fmaxf(peak,fabsf(v));}  // 将 0;for(float &v:values){v*=60/3200.0f;peak=fmaxf(peak,f… 写入或初始化 peak。
    float scale=peak>cfg.maxRpm?cfg.maxRpm/peak:1;  // 将 peak>cfg.maxRpm?cfg.maxRpm/peak:1 写入或初始化 scale。
    float p[4];for(int i=0;i<4;++i){out.rpm[i]=static_cast<int16_t>(lroundf(values[i]*scale));p[i]=out.rpm[i]*3200/60.0f;}  // 调用 for，传入 int i=0;i<4;++i){out.rpm[i]=static_cast<int16_t>(lroun…。
    out.body.x=(-p[0]-p[1]+p[2]+p[3])/(4*cfg.lateralPpm);  // 将 (-p[0]-p[1]+p[2]+p[3])/(4*cfg.lateralPpm) 写入或初始化 out.body.x。
    out.body.y=(p[0]-p[1]+p[2]-p[3])/(4*cfg.forwardPpm);  // 将 (p[0]-p[1]+p[2]-p[3])/(4*cfg.forwardPpm) 写入或初始化 out.body.y。
    out.omega=(p[0]+p[1]+p[2]+p[3])/(4*cfg.rotationPpd);  // 将 (p[0]+p[1]+p[2]+p[3])/(4*cfg.rotationPpd) 写入或初始化 out.omega。
    return out;  // 结束当前函数并返回 out。
}  // 结束此函数或控制块。
/**
 * @brief 初始化路径跟踪状态；actualHeading为开始时实测地图航向(deg)。
 */
void Tracker::begin(const Route &route,float actual) {  // 定义函数或成员 的入口 Tracker::begin，参数由此处的签名确定。
    *this=Tracker{};estimated=route.at(0).p;(void)actual;  // 将 Tracker{};estimated=route.at(0).p;(void)actual 写入或初始化 this。
}  // 结束此函数或控制块。
/**
 * @brief 按已下发轮速、实测航向及时间dt(s)更新估算位置，不代表外部定位。
 */
void Tracker::integrate(const Route &route,float actual,float dt) {  // 定义函数或成员 的入口 Tracker::integrate，参数由此处的签名确定。
    Vec world=bodyToWorld(applied.body,actual);estimated=estimated+world*dt;  // 将 bodyToWorld(applied.body,actual) 写入或初始化 world（地图）。然后将 estimated+world*dt 写入或初始化 estimated。
    if(route.config.policy) {  // 当 route.config.policy 成立时进入此分支，否则继续后续流程。
        if(phaseIndex<route.segmentCount&&route.segments[phaseIndex].kind!=ROTATE) {  // 当 phaseIndex<route.segmentCount&&route.segments[phaseInd… 成立时进入此分支，否则继续后续流程。
            Vec tangent=route.phaseAt(phaseIndex,s-route.segments[phaseIndex].offset).tangent;  // 将 route.phaseAt(phaseIndex,s-route.segments[phaseIndex].… 写入或初始化 tangent（切向）。
            s=fminf(route.nextStop(phaseIndex),s+fmaxf(0,dot(world,tangent))*dt);  // 将 fminf(route.nextStop(phaseIndex),s+fmaxf(0,dot(world,t… 写入或初始化 s。
        }  // 结束此条件分支。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    if(!finishing)s=fminf(route.length,s+fmaxf(0,dot(world,route.at(s).tangent))*dt);  // 当 !finishing 成立时进入此分支，否则继续后续流程。
}  // 结束此函数或控制块。
/**
 * @brief 计算一个控制周期的四轮命令、航向误差及完成结果；dt为秒，sampleMs为样本毫秒数。
 */
Output Tracker::update(const Route &route,float actual,float dt,uint32_t sampleMs) {  // 定义函数或成员 的入口 Tracker::update，参数由此处的签名确定。
    if(route.config.policy)return updateHybrid(route,actual,dt,sampleMs);  // 若 route.config.policy，立即返回 updateHybrid(route,actual,dt,sampleMs)。
    Output out;const Config &c=route.config;  // 声明 out，类型和可用范围由所在声明决定。然后将 route.config 写入或初始化 c。
    out.reference=route.heading(s);out.error=turn(actual,out.reference);  // 将 route.heading(s) 写入或初始化 out.reference。然后将 turn(actual,out.reference) 写入或初始化 out.error。
    if(!isfinite(dt)||dt<=0||dt>0.120f){out.failure="CONTROL_OVERRUN";return out;}  // 当 !isfinite(dt)||dt<=0||dt>0.120f 成立时进入此分支，否则继续后续流程。
    badSeconds=fabsf(out.error)>20?badSeconds+dt:0;  // 将 fabsf(out.error)>20?badSeconds+dt:0 写入或初始化 badSeconds。
    if(badSeconds>=0.2f){out.failure="HEADING_ERROR";return out;}  // 当 badSeconds>=0.2f 成立时进入此分支，否则继续后续流程。
    if(route.length-s<=0.5f)finishing=true;  // 当 route.length-s<=0.5f 成立时进入此分支，否则继续后续流程。
    float targetOmega=0;  // 将 0 写入或初始化 targetOmega（目标Omega）。
    if(finishing) {  // 当 finishing 成立时进入此分支，否则继续后续流程。
        finishSeconds+=dt;out.reference=c.endHeading;out.error=turn(actual,c.endHeading);speed=0;  // 把 dt 累加到 finishSeconds。然后将 c.endHeading 写入或初始化 out.reference。然后将 turn(actual,c.endHeading) 写入或初始化 out.error。然后将 0 写入或初始化 speed（速度）。
        if(sampleMs!=previousSample) {  // 当 sampleMs!=previousSample 成立时进入此分支，否则继续后续流程。
            stableSeconds=fabsf(out.error)<=5?stableSeconds+dt:0;previousSample=sampleMs;  // 将 fabsf(out.error)<=5?stableSeconds+dt:0 写入或初始化 stableSeconds（稳定Seconds）。然后将 sampleMs 写入或初始化 previousSample（上一采样）。
        }  // 结束此条件分支。
        if(stableSeconds>=0.2f){out.finished=true;return out;}  // 当 stableSeconds>=0.2f 成立时进入此分支，否则继续后续流程。
        if(finishSeconds>3){out.failure="FINAL_HEADING_TIMEOUT";return out;}  // 当 finishSeconds>3 成立时进入此分支，否则继续后续流程。
        targetOmega=clamp(c.kp*out.error,-fminf(20,c.maxOmega),fminf(20,c.maxOmega));  // 将 clamp(c.kp*out.error,-fminf(20,c.maxOmega),fminf(20,c.… 写入或初始化 targetOmega（目标Omega）。
    } else {  // 前一个判断不成立时执行此分支。
        float target=route.speedCap(s);  // 将 route.speedCap(s) 写入或初始化 target（目标）。
        target=fminf(target,sqrtf(2*c.accel*(route.length-s)+c.accel*c.accel*dt*dt)-c.accel*dt);  // 将 fminf(target,sqrtf(2*c.accel*(route.length-s)+c.accel*… 写入或初始化 target（目标）。
        target*=clamp(1-(fabsf(out.error)-5)/15,0.2f,1);  // 用 clamp(1-(fabsf(out.error)-5)/15,0.2f,1) 对 target（目标） 应用 *= 更新。
        speed=clamp(target,fmaxf(0,speed-c.accel*dt),speed+c.accel*dt);  // 将 clamp(target,fmaxf(0,speed-c.accel*dt),speed+c.accel*d… 写入或初始化 speed（速度）。
        float correction=clamp(c.kp*out.error,-c.correction,c.correction);  // 将 clamp(c.kp*out.error,-c.correction,c.correction) 写入或初始化 correction（修正量）。
        targetOmega=clamp(route.headingSlope(s)*speed+correction,-c.maxOmega,c.maxOmega);  // 将 clamp(route.headingSlope(s)*speed+correction,-c.maxOme… 写入或初始化 targetOmega（目标Omega）。
    }  // 结束此条件分支。
    omega=clamp(targetOmega,omega-90*dt,omega+90*dt);  // 将 clamp(targetOmega,omega-90*dt,omega+90*dt) 写入或初始化 omega。
    Vec body=finishing?Vec{}:worldToBody(route.at(s).tangent*speed,actual);  // 将 finishing?Vec{}:worldToBody(route.at(s).tangent*speed,… 写入或初始化 body（车体）。
    out.wheels=wheels(body,omega,c);return out;  // 将 wheels(body,omega,c) 写入或初始化 out.wheels。然后结束当前函数并返回 out。
}  // 结束此函数或控制块。
} // namespace continuous
