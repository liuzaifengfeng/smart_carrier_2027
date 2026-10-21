#include "continuous_trial.h"
#include "robot_runtime.h"
#include "h30_usb.h"
#include "field_geometry.h"
#include "node_position_hardware.h"
#include "motion_executor.h"

namespace {
using namespace continuous;
Route route;
Node nodes[MAX_NODES];
uint8_t nodeIds[MAX_NODES]={};
Config staged;
node_position::Settings formalSettings;
uint32_t receivedMask=0,referenceVersion=0,failedVersion=0;
float initialHeading=0;
bool configured=false,needsCalibration=false,consumed=false;
bool policyReceived=false;
bool initialized=false;
SemaphoreHandle_t configurationLock=nullptr;
struct ConfigurationGuard {
    bool held;
    ConfigurationGuard():held(configurationLock&&xSemaphoreTake(configurationLock,0)==pdTRUE){}
    ~ConfigurationGuard(){if(held)xSemaphoreGive(configurationLock);}
};
portMUX_TYPE trialMux=portMUX_INITIALIZER_UNLOCKED;
enum State {IDLE,RUNNING,STOPPING,DONE,FAILED};
struct Snapshot {
    bool formal=false,async=false;
    uint32_t id=0,heartbeatMs=0;
    State state=IDLE;
    float s=0,total=0,x=0,y=0,reference=0,heading=0,error=0,cycleMs=0;
    int16_t rpm[4]={};
    const char *reason="NONE";
} status;
bool stopRequested=false;
bool otaActive=false;
const char *names[]={"IDLE","RUNNING","STOPPING","DONE","FAILED"};
Snapshot snapshot() {portENTER_CRITICAL(&trialMux);auto v=status;portEXIT_CRITICAL(&trialMux);return v;}
bool stopped() {portENTER_CRITICAL(&trialMux);bool v=stopRequested;portEXIT_CRITICAL(&trialMux);return v;}
bool otaBusy(){portENTER_CRITICAL(&trialMux);bool v=otaActive;portEXIT_CRITICAL(&trialMux);return v;}
const char *headingSample(float &heading,uint32_t &sampleMs,H30Attitude *motion=nullptr);
void hardStop() {
    for(uint8_t i=1;i<=4;++i){Emm_V5_Stop_Now(i,true);vTaskDelay(pdMS_TO_TICKS(5));}
    Emm_V5_Synchronous_motion(0);Serial1.flush();
}
// 整组速度先装载，再同步生效；停车请求在同步提交之前再次检查。
const char *sendWheels(const WheelCommand &v,uint32_t cycleStart) {
    for(uint8_t i=1;i<=4;++i){
        int16_t rpm=v.rpm[i-1];
        Emm_V5_Vel_Control(i,rpm>=0?1:0,abs(rpm),0,true);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if(stopped())return "STOPPED";
    if(millis()-cycleStart>120)return "CONTROL_OVERRUN";
    float heading=0;uint32_t sample=0;
    const char *error=headingSample(heading,sample);if(error)return error;
    Emm_V5_Synchronous_motion(0);Serial1.flush();return nullptr;
}
void telemetry(const Snapshot &v) {
    Serial.printf("{EVT,NAV,%s,%lu,%.2f,%.2f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%.1f}\n",
        v.formal?"ROUTE_SAMPLE":"TRIAL_SAMPLE",static_cast<unsigned long>(v.id),v.s,v.total,v.reference,v.heading,v.error,
        v.rpm[0],v.rpm[1],v.rpm[2],v.rpm[3],v.cycleMs);
}
const char *headingSample(float &heading,uint32_t &sampleMs,H30Attitude *motion) {
    uint32_t version=0;
    H30Attitude frame;
    const char *error=H30UsbGetMotionSnapshot(frame,heading,version);
    sampleMs=frame.receivedAtMs;if(motion)*motion=frame;
    if(error)return error;
    if(version!=referenceVersion)return "REFERENCE_CHANGED";
    if(millis()-sampleMs>100)return "IMU_STALE";
    return nullptr;
}
struct Execution {
    Tracker tracker;
    Snapshot job;
    node_position::Result position;
    bool active=false,success=false;
    const char *failure=nullptr;
    float actual=0;
    uint32_t sampleMs=0,started=0,previousCycle=0,integratedAt=0,lastTelemetry=0,timeout=0,due=0;
} execution;
void finishExecution() {
    auto &e=execution;
    hardStop();
    if(e.job.formal) {
        e.tracker.estimated=e.position.estimated;e.tracker.s=e.position.s;e.actual=e.position.heading;
    } else e.tracker.integrate(route,e.actual,(millis()-e.integratedAt)/1000.0f);
    currentPose={e.tracker.estimated.x,e.tracker.estimated.y,wrap(e.actual)};
    if(!e.success){needsCalibration=true;failedVersion=referenceVersion;}
    portENTER_CRITICAL(&trialMux);
    status.x=e.tracker.estimated.x;status.y=e.tracker.estimated.y;status.heading=e.actual;
    status.s=e.tracker.s;status.reason=e.success?"ESTIMATED":e.failure;
    for(int i=0;i<4;++i)status.rpm[i]=0;
    portEXIT_CRITICAL(&trialMux);
    Serial.printf("{EVT,NAV,%s,%lu,%s,%s,%.2f,%.2f,%.3f}\n",
        e.job.formal?"ROUTE_RESULT":"TRIAL_RESULT",static_cast<unsigned long>(e.job.id),e.success?"DONE":"FAILED",e.success?"ESTIMATED":e.failure,
        e.tracker.estimated.x,e.tracker.estimated.y,e.actual);
    if(e.job.formal&&e.job.async)Serial.printf("{EVT,NAV,%s,%s}\n",e.success?"ROUTE_DONE":"ROUTE_FAILED",e.success?"ESTIMATED":e.failure);
    portENTER_CRITICAL(&trialMux);status.state=e.success?DONE:FAILED;portEXIT_CRITICAL(&trialMux);
    e.active=false;
}
bool adjacent(uint8_t a,uint8_t b) {
    int ar=a/5,br=b/5,ac=a%5,bc=b%5;if(ar%2)ac=4-ac;if(br%2)bc=4-bc;
    return abs(ar-br)+abs(ac-bc)==1;
}
}

bool ContinuousTrialInit() {
    if(!configurationLock)configurationLock=xSemaphoreCreateMutex();
    initialized=configurationLock!=nullptr;return initialized;
}
bool ContinuousTrialActive(){auto v=snapshot();return v.state==RUNNING||v.state==STOPPING;}
const char *ContinuousTrialBegin(const continuous::Config &config) {
    ConfigurationGuard guard;if(!guard.held)return "BUSY";
    if(ContinuousTrialActive())return "BUSY";
    if(otaBusy())return "OTA";
    if(!initialized)return "INIT";
    if(config.id==0||config.id>9999999||config.count<2||config.count>continuous::MAX_NODES)return "RANGE";
    if(configured&&config.id==staged.id)return "ID_REUSED";
    staged=config;staged.policy=0;receivedMask=0;configured=true;consumed=false;policyReceived=false;
    portENTER_CRITICAL(&trialMux);status=Snapshot{};status.id=config.id;portEXIT_CRITICAL(&trialMux);
    return nullptr;
}
const char *ContinuousTrialPolicy(uint32_t id,uint8_t policy,float outset,float margin,uint32_t geometry) {
    ConfigurationGuard guard;if(!guard.held)return "BUSY";
    if(ContinuousTrialActive())return "BUSY";
    if(!configured||id!=staged.id)return "ID";
    if(geometry!=field::signature())return "GEOMETRY_MISMATCH";
    if((policy!=1&&policy!=2)||!isfinite(outset)||outset<0||outset>100
        ||!isfinite(margin)||margin<10||margin>40)return "POLICY_RANGE";
    staged.policy=policy;staged.outset=outset;staged.margin=margin;policyReceived=true;return nullptr;
}
const char *ContinuousTrialNode(uint32_t id,uint8_t index,uint8_t node,float radius,float expectedX,float expectedY) {
    ConfigurationGuard guard;if(!guard.held)return "BUSY";
    if(ContinuousTrialActive())return "BUSY";
    if(!configured||id!=staged.id)return "ID";
    if(index>=staged.count||node>=25||!isfinite(radius)||radius<0||radius>1000)return "RANGE";
    float x=0,y=0;if(!ChassisGetNodePosition(node,x,y))return "NODE";
    if(!isfinite(expectedX)||!isfinite(expectedY)||fabsf(x-expectedX)>0.05f||fabsf(y-expectedY)>0.05f)return "MAP_MISMATCH";
    nodes[index].p=continuous::Vec(x,y);nodes[index].radius=radius;nodeIds[index]=node;
    receivedMask|=1u<<index;return nullptr;
}
const char *ContinuousTrialPrepare(uint32_t id) {
    ConfigurationGuard guard;if(!guard.held)return "BUSY";
    if(ContinuousTrialActive())return "BUSY";
    if(otaBusy())return "OTA";
    if(!configured||id!=staged.id)return "ID";
    if(!policyReceived)return "POLICY_REQUIRED";
    if(receivedMask!=((1u<<staged.count)-1))return "INCOMPLETE";
    if(consumed)return "ID_REUSED";
    if(taskMotionAborted)return "ABORTED";
    for(size_t i=1;i<staged.count;++i)if(!adjacent(nodeIds[i-1],nodeIds[i]))return "NOT_ADJACENT";
    // 不允许上传期间地图改变后静默执行不同于上位机预览的轨迹。
    for(size_t i=0;i<staged.count;++i) {
        float x=0,y=0;
        if(!ChassisGetNodePosition(nodeIds[i],x,y))return "NODE";
        if(fabsf(x-nodes[i].p.x)>0.05f||fabsf(y-nodes[i].p.y)>0.05f)return "MAP_CHANGED";
    }
    staged.rotationPpd=THETA_PULSE;
    const char *error=route.build(nodes,staged);if(error)return error;
    if(!isfinite(currentPose.x)||!isfinite(currentPose.y)
        ||hypotf(currentPose.x-nodes[0].p.x,currentPose.y-nodes[0].p.y)>25)return "START_POSITION";
    float actual=0;uint32_t sample=0,version=0;
    H30Attitude motion;
    error=H30UsbGetMotionSnapshot(motion,actual,version);sample=motion.receivedAtMs;if(error)return error;
    if(millis()-sample>100)return "IMU_STALE";
    if(needsCalibration&&version==failedVersion)return "RECALIBRATE";
    if(fabsf(continuous::turn(actual,staged.startHeading))>5)return "START_HEADING";
    referenceVersion=version;
    initialHeading=actual;
    setAlignmentEnabled(false); // 等视觉任务停止，再发布运行态。
    consumed=true;
    portENTER_CRITICAL(&trialMux);
    stopRequested=false;status.state=RUNNING;status.reason="NONE";
    status.s=0;status.x=nodes[0].p.x;status.y=nodes[0].p.y;status.heading=actual;
    status.reference=staged.startHeading;status.error=continuous::turn(actual,staged.startHeading);
    status.total=route.length;status.heartbeatMs=millis();
    portEXIT_CRITICAL(&trialMux);return nullptr;
}
void ContinuousTrialLaunch(){
    MotionRequest request;request.kind=MotionKind::TRIAL;
    if(!SubmitMotion(request)) {
        portENTER_CRITICAL(&trialMux);status.state=FAILED;status.reason="BUSY";portEXIT_CRITICAL(&trialMux);
    }
}
const char *ContinuousTrialStop(uint32_t id) {
    portENTER_CRITICAL(&trialMux);
    const char *error=id!=status.id?"ID":nullptr;
    if(!error&&(status.state==RUNNING||status.state==STOPPING)){stopRequested=true;status.state=STOPPING;}
    portEXIT_CRITICAL(&trialMux);return error;
}
void ContinuousTrialCancel(){auto v=snapshot();ContinuousTrialStop(v.id);}
void ContinuousTrialSetOta(bool active){
    portENTER_CRITICAL(&trialMux);otaActive=active;
    if(active&&(status.state==RUNNING||status.state==STOPPING)){stopRequested=true;status.state=STOPPING;}
    portEXIT_CRITICAL(&trialMux);
    MotionExecutorSetOta(active);
}
const char *ContinuousTrialStatus(uint32_t id) {
    portENTER_CRITICAL(&trialMux);
    if(id!=status.id){portEXIT_CRITICAL(&trialMux);return "ID";}
    status.heartbeatMs=millis();auto v=status;
    portEXIT_CRITICAL(&trialMux);
    Serial.printf("{RSP,NAV,TRIAL_STATUS,OK,%lu,%s,%s,%.2f,%.2f,%.2f,%.2f,%.3f}\n",
        static_cast<unsigned long>(v.id),names[v.state],v.reason,v.s,v.total,v.x,v.y,v.heading);
    return nullptr;
}

bool H30PrepareNodePath(const uint8_t *path,size_t count,float speedMmS,float finalHeading,bool async) {
    ConfigurationGuard guard;if(!guard.held)return false;
    if(!initialized||otaBusy()||taskMotionAborted)return false;
    // 预留共享规划/任务资源；上传和其他运动不能在构建过程中插入。
    portENTER_CRITICAL(&trialMux);
    if(status.state==RUNNING||status.state==STOPPING){portEXIT_CRITICAL(&trialMux);return false;}
    status=Snapshot{};status.formal=true;status.async=async;status.state=STOPPING;
    stopRequested=false;
    portEXIT_CRITICAL(&trialMux);
    configured=false;
    H30Attitude motion;float heading=0;uint32_t version=0;
    const char *error=H30UsbGetMotionSnapshot(motion,heading,version);
    if(!error&&millis()-motion.receivedAtMs>100)error="IMU_STALE";
    if(!error&&needsCalibration&&version==failedVersion)error="RECALIBRATE";
    if(error) {
        Serial.printf("[Route] H30 assist unavailable at start (%s); using planned heading\n",error);
        heading=currentPose.theta;version=0;error=nullptr;
    }
    Config cfg;cfg.count=count;cfg.policy=2;cfg.startHeading=heading; // 正式导航统一到节点停车转向，不使用漂移切弯。
    cfg.maxRpm=node_position::rpmFor(speedMmS,Y_PULSE);
    cfg.accel=NODE_PATH_ACCEL_MM_S2; // 与轮速一同冻结，运行中不跟随参数写入变化。
    cfg.rotationPpd=THETA_PULSE;
    // 与正常扫码MovePosition一致：车身X用X_PULSE，车身Y用Y_PULSE。
    cfg.forwardPpm=Y_PULSE;cfg.lateralPpm=X_PULSE;
    formalSettings.speedMmS=speedMmS;formalSettings.accelMmS2=cfg.accel;
    formalSettings.turnDegS=NODE_PATH_TURN_DEG_S;
    formalSettings.turnAccelDegS2=NODE_PATH_TURN_ACCEL_DEG_S2;
    formalSettings.headingDeadbandDeg=NODE_PATH_HEADING_DEADBAND_DEG;
    float end=heading;
    for(size_t i=0;i<count&&!error;++i) {
        float x=0,y=0;
        if(!ChassisGetNodePosition(path[i],x,y))error="NODE";
        else {nodes[i]=Node{};nodes[i].p={x,y};}
        if(i&&!adjacent(path[i-1],path[i]))error="NOT_ADJACENT";
        if(i&&!error) {
            Vec d=nodes[i].p-nodes[i-1].p;
            float forward=wrap(atan2f(d.y,d.x)*180/PI_F),back=wrap(forward+180);
            end=fabsf(turn(end,back))<fabsf(turn(end,forward))-.01f?back:forward;
        }
    }
    cfg.endHeading=isnan(finalHeading)?end:wrap(finalHeading);
    if(!isfinite(cfg.endHeading))error="RANGE";
    if(!error)error=route.build(nodes,cfg);
    if(error) {
        Serial.printf("[Route] H30 start rejected: %s\n",error);
        portENTER_CRITICAL(&trialMux);status.state=FAILED;status.reason=error;portEXIT_CRITICAL(&trialMux);
        return false;
    }
    configured=false;referenceVersion=version;initialHeading=heading;
    Serial.printf("[Route] Transition window: disabled; separate heading correction deadband: %.2f deg\n",formalSettings.headingDeadbandDeg);
    Serial.printf("[Route] Profile: %.1f mm/s, %.1f mm/s2, %.1f deg/s, %.1f deg/s2\n",
        formalSettings.speedMmS,formalSettings.accelMmS2,formalSettings.turnDegS,formalSettings.turnAccelDegS2);
    formalSettings.startOffset={};
    if(scanPosePending&&path[0]==SCAN_AREA_NODE) {
        formalSettings.startOffset=Vec(currentPose.x,currentPose.y)-nodes[0].p;
        scanPosePending=false;
        Serial.printf("[Route] Scan start offset=(%.1f, %.1f) mm\n",
                      formalSettings.startOffset.x,formalSettings.startOffset.y);
    } else {
        currentPose.x=nodes[0].p.x;currentPose.y=nodes[0].p.y;
    }
    setAlignmentEnabled(false);
    portENTER_CRITICAL(&trialMux);
    status.total=route.length;status.state=RUNNING;status.heartbeatMs=millis();
    status.x=currentPose.x;status.y=currentPose.y;status.heading=heading;
    portEXIT_CRITICAL(&trialMux);
    return true;
}
bool H30StartNodePath(const uint8_t *path,size_t count,float speedMmS,float finalHeading,bool async) {
    MotionRequest r;r.kind=MotionKind::NODE_PATH;r.pathCount=count;r.speed=speedMmS;r.theta=finalHeading;
    r.automatic=async;memcpy(r.path,path,count);return SubmitMotion(r)!=0;
}
bool H30RunNodePath(const uint8_t *path,size_t count,float speedMmS,float finalHeading) {
    MotionRequest r;r.kind=MotionKind::NODE_PATH;r.pathCount=count;r.speed=speedMmS;r.theta=finalHeading;
    memcpy(r.path,path,count);return WaitMotion(SubmitMotion(r));
}
void ContinuousMotionBegin() {
    execution=Execution{};auto &e=execution;e.job=snapshot();e.active=true;e.actual=initialHeading;
    hardStop();
    if(!e.job.formal)e.failure=headingSample(e.actual,e.sampleMs);
    e.tracker.begin(route,e.actual);e.tracker.requireInertial=true;e.tracker.commit(WheelCommand{},millis());
    e.tracker.estimated={currentPose.x,currentPose.y};
    if(!e.failure&&!e.job.formal&&fabsf(turn(e.actual,route.config.startHeading))>5)e.failure="START_HEADING";
    e.started=e.previousCycle=e.integratedAt=millis();e.due=e.started;
    e.timeout=static_cast<uint32_t>((route.plannedSeconds*2+5)*1000);
    if(e.job.formal&&!e.failure) {
        e.position.estimated=e.tracker.estimated;e.position.heading=e.actual;
        BeginPositionNodeRoute(route,formalSettings,referenceVersion,stopped,e.position);
    }
    if(e.failure)finishExecution();
}
void ContinuousMotionTick() {
    auto &e=execution;if(!e.active)return;
    if(stopped()||taskMotionAborted){e.failure=stopped()?"STOPPED":"ABORTED";finishExecution();return;}
    if(e.job.formal) {
        TickPositionNodeRoute();
        portENTER_CRITICAL(&trialMux);status.s=e.position.s;status.x=e.position.estimated.x;status.y=e.position.estimated.y;status.heading=e.position.heading;portEXIT_CRITICAL(&trialMux);
        if(!PositionNodeRunner().active()){e.failure=PositionNodeRunner().failure();e.success=!e.failure;finishExecution();}
        return;
    }
    const uint32_t now=millis();if(static_cast<int32_t>(now-e.due)<0)return;
    if(now-snapshot().heartbeatMs>1500)e.failure="LINK_TIMEOUT";
    else if(now-e.started>e.timeout)e.failure="TIMEOUT";
    H30Attitude motion;if(!e.failure)e.failure=headingSample(e.actual,e.sampleMs,&motion);
    if(e.failure){finishExecution();return;}
    e.tracker.observeMotion(motion);
    const float dt=now==e.started?.04f:(now-e.previousCycle)*.001f;e.previousCycle=now;
    if(dt>.120f||dt<=0){e.failure="CONTROL_OVERRUN";finishExecution();return;}
    e.tracker.integrate(route,e.actual,(now-e.integratedAt)*.001f);e.integratedAt=now;
    e.failure=field::check(e.tracker.estimated.x,e.tracker.estimated.y,e.actual,0);
    if(e.failure){finishExecution();return;}
    Output out=e.tracker.update(route,e.actual,dt,e.sampleMs);
    if(out.failure){e.failure=out.failure;finishExecution();return;}
    if(out.finished){e.success=true;finishExecution();return;}
    e.failure=sendWheels(out.wheels,now);if(e.failure){finishExecution();return;}
    const uint32_t synced=millis();e.tracker.integrate(route,e.actual,(synced-e.integratedAt)*.001f);e.integratedAt=synced;
    e.tracker.commit(out.wheels,synced);
    portENTER_CRITICAL(&trialMux);
    status.s=e.tracker.s;status.x=e.tracker.estimated.x;status.y=e.tracker.estimated.y;
    status.reference=out.reference;status.heading=e.actual;status.error=out.error;status.cycleMs=dt*1000;
    for(int i=0;i<4;++i)status.rpm[i]=out.wheels.rpm[i];
    portEXIT_CRITICAL(&trialMux);
    if(now-e.lastTelemetry>=200){telemetry(snapshot());
        Serial.printf("{EVT,NAV,TRIAL_PHASE,%lu,%s}\n",static_cast<unsigned long>(e.job.id),e.tracker.stage);e.lastTelemetry=now;}
    e.due=now+40;
}
void ContinuousMotionCancel(){if(execution.active){execution.failure="STOPPED";execution.success=false;finishExecution();}}
bool ContinuousMotionActive(){return execution.active;}
const char *ContinuousMotionFailure(){return execution.failure;}
uint32_t ContinuousMotionNextDelay(){
    if(execution.job.formal)return PositionNodeRunner().active()?PositionNodeRunner().nextDelay():0;
    const int32_t left=static_cast<int32_t>(execution.due-millis());return left>0?left:0;
}
