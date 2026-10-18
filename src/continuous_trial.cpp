#include "continuous_trial.h"
#include "robot_runtime.h"
#include "h30_usb.h"
#include "field_geometry.h"
#include "node_position_hardware.h"

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
TaskHandle_t worker=nullptr;
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
void task(void *) {
    for(;;) {
        ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
        const uint32_t jobId=route.config.id;
        const auto job=snapshot();
        bool locked=xAlignmentMotionMutex&&xSemaphoreTake(xAlignmentMotionMutex,pdMS_TO_TICKS(100))==pdTRUE;
        const char *failure=locked?nullptr:"LOCK";
        if(locked)hardStop(); // 起跑前明确停止，不把默认零结构体当成已下发零轮速。
        Tracker tracker;float actual=initialHeading;uint32_t sampleMs=0;
        if(!failure&&!job.formal)failure=headingSample(actual,sampleMs);
        tracker.begin(route,actual);
        tracker.requireInertial=true;
        tracker.commit(WheelCommand{},millis());
        if(locked)tracker.estimated={currentPose.x,currentPose.y};
        if(!failure&&!job.formal&&fabsf(turn(actual,route.config.startHeading))>5)failure="START_HEADING";
        if(job.formal&&!failure) {
            node_position::Result result;result.estimated=tracker.estimated;result.heading=actual;
            failure=RunPositionNodeRoute(route,formalSettings,referenceVersion,stopped,result);
            tracker.estimated=result.estimated;tracker.s=result.s;actual=result.heading;
        }
        uint32_t started=millis(),previousCycle=started,integratedAt=started,lastTelemetry=0;
        uint32_t timeout=static_cast<uint32_t>((route.plannedSeconds*2+5)*1000);
        TickType_t wake=xTaskGetTickCount();bool success=job.formal&&!failure;
        while(!job.formal&&!failure&&!success) {
            uint32_t now=millis();
            if(stopped()){failure="STOPPED";break;}
            if(taskMotionAborted){failure="ABORTED";break;}
            if(!job.formal&&now-snapshot().heartbeatMs>1500){failure="LINK_TIMEOUT";break;}
            if(now-started>timeout){failure="TIMEOUT";break;}
            H30Attitude motion;
            failure=headingSample(actual,sampleMs,&motion);if(failure)break;
            tracker.observeMotion(motion);
            float dt=now==started?0.04f:(now-previousCycle)/1000.0f;previousCycle=now;
            if(dt>0.120f||dt<=0){failure="CONTROL_OVERRUN";break;}
            tracker.integrate(route,actual,(now-integratedAt)/1000.0f);integratedAt=now;
            failure=field::check(tracker.estimated.x,tracker.estimated.y,actual,0);
            if(failure)break; // 仅对估算位置兜底，不能代替实际定位。
            Output out=tracker.update(route,actual,dt,sampleMs);
            if(out.failure){failure=out.failure;break;}
            if(out.finished){success=true;break;}
            failure=sendWheels(out.wheels,now);if(failure)break;
            // 上一组指令一直持续到新同步触发生效；将组间延时计入距离。
            uint32_t synced=millis();
            tracker.integrate(route,actual,(synced-integratedAt)/1000.0f);integratedAt=synced;
            tracker.commit(out.wheels,synced);
            portENTER_CRITICAL(&trialMux);
            status.s=tracker.s;status.x=tracker.estimated.x;status.y=tracker.estimated.y;
            status.reference=out.reference;status.heading=actual;status.error=out.error;status.cycleMs=dt*1000;
            for(int i=0;i<4;++i)status.rpm[i]=out.wheels.rpm[i];
            portEXIT_CRITICAL(&trialMux);
            if(now-lastTelemetry>=200){telemetry(snapshot());
                Serial.printf("{EVT,NAV,%s,%lu,%s}\n",job.formal?"ROUTE_PHASE":"TRIAL_PHASE",static_cast<unsigned long>(jobId),tracker.stage);
                lastTelemetry=now;}
            vTaskDelayUntil(&wake,pdMS_TO_TICKS(40));
        }
        if(locked) {
            hardStop();
            tracker.integrate(route,actual,(millis()-integratedAt)/1000.0f);
            currentPose={tracker.estimated.x,tracker.estimated.y,wrap(actual)};
            xSemaphoreGive(xAlignmentMotionMutex);
        }
        if(!success){needsCalibration=true;failedVersion=referenceVersion;}
        portENTER_CRITICAL(&trialMux);
        status.x=tracker.estimated.x;status.y=tracker.estimated.y;status.heading=actual;
        status.s=tracker.s;status.reason=success?"ESTIMATED":failure;
        for(int i=0;i<4;++i)status.rpm[i]=0;
        // 先保持运行态直到完成事件打印，避免新 BEGIN 覆盖旧任务资源。
        portEXIT_CRITICAL(&trialMux);
        Serial.printf("{EVT,NAV,%s,%lu,%s,%s,%.2f,%.2f,%.3f}\n",
            job.formal?"ROUTE_RESULT":"TRIAL_RESULT",static_cast<unsigned long>(jobId),success?"DONE":"FAILED",success?"ESTIMATED":failure,
            tracker.estimated.x,tracker.estimated.y,actual);
        if(job.formal&&job.async)Serial.printf("{EVT,NAV,%s,%s}\n",success?"ROUTE_DONE":"ROUTE_FAILED",success?"ESTIMATED":failure);
        portENTER_CRITICAL(&trialMux);status.state=success?DONE:FAILED;portEXIT_CRITICAL(&trialMux);
    }
}
bool adjacent(uint8_t a,uint8_t b) {
    int ar=a/5,br=b/5,ac=a%5,bc=b%5;if(ar%2)ac=4-ac;if(br%2)bc=4-bc;
    return abs(ar-br)+abs(ac-bc)==1;
}
}

bool ContinuousTrialInit() {
    if(worker)return true;
    if(!configurationLock)configurationLock=xSemaphoreCreateMutex();
    if(!configurationLock)return false;
    return xTaskCreate(task,"ContinuousTrial",8192,nullptr,6,&worker)==pdPASS;
}
bool ContinuousTrialActive(){auto v=snapshot();return v.state==RUNNING||v.state==STOPPING;}
const char *ContinuousTrialBegin(const continuous::Config &config) {
    ConfigurationGuard guard;if(!guard.held)return "BUSY";
    if(ContinuousTrialActive())return "BUSY";
    if(otaBusy())return "OTA";
    if(!worker)return "INIT";
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
void ContinuousTrialLaunch(){if(worker)xTaskNotifyGive(worker);}
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

bool H30StartNodePath(const uint8_t *path,size_t count,float speedMmS,float finalHeading,bool async) {
    ConfigurationGuard guard;if(!guard.held)return false;
    if(!worker||otaBusy()||!path||count<2||count>MAX_NODES||!isfinite(speedMmS)||speedMmS<10||speedMmS>1600||taskMotionAborted)return false;
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
    Config cfg;cfg.count=count;cfg.policy=2;cfg.startHeading=heading;
    cfg.maxRpm=node_position::rpmFor(speedMmS,Y_PULSE);
    cfg.accel=NODE_PATH_ACCEL_MM_S2; // 与轮速一同冻结，运行中不跟随参数写入变化。
    cfg.rotationPpd=THETA_PULSE;
    // 与正常扫码MovePosition一致：车身X用X_PULSE，车身Y用Y_PULSE。
    cfg.forwardPpm=Y_PULSE;cfg.lateralPpm=X_PULSE;
    formalSettings.speedMmS=speedMmS;formalSettings.accelMmS2=cfg.accel;
    formalSettings.turnDegS=NODE_PATH_TURN_DEG_S;
    formalSettings.turnAccelDegS2=NODE_PATH_TURN_ACCEL_DEG_S2;
    formalSettings.headingDeadbandDeg=NODE_PATH_HEADING_DEADBAND_DEG;
    if(!isfinite(formalSettings.turnDegS)||formalSettings.turnDegS<10||formalSettings.turnDegS>240
        ||!isfinite(formalSettings.turnAccelDegS2)||formalSettings.turnAccelDegS2<30||formalSettings.turnAccelDegS2>720
        ||!isfinite(formalSettings.headingDeadbandDeg)||formalSettings.headingDeadbandDeg<1||formalSettings.headingDeadbandDeg>8)error="RANGE";
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
    Serial.printf("[Route] Transition window: 100ms; separate heading correction deadband: %.2f deg\n",formalSettings.headingDeadbandDeg);
    Serial.printf("[Route] Profile: %.1f mm/s, %.1f mm/s2, %.1f deg/s, %.1f deg/s2\n",
        formalSettings.speedMmS,formalSettings.accelMmS2,formalSettings.turnDegS,formalSettings.turnAccelDegS2);
    Serial.printf("[Route] Profile: %.1f mm/s, %.1f mm/s2, %.1f deg/s, %.1f deg/s2\n",
        formalSettings.speedMmS,formalSettings.accelMmS2,formalSettings.turnDegS,formalSettings.turnAccelDegS2);
    // 沿用MoveNodePath首节点的显式位置假设，不重新标定/覆盖H30航向参考。
    currentPose.x=nodes[0].p.x;currentPose.y=nodes[0].p.y;
    setAlignmentEnabled(false);
    portENTER_CRITICAL(&trialMux);
    status.total=route.length;status.state=RUNNING;status.heartbeatMs=millis();
    status.x=currentPose.x;status.y=currentPose.y;status.heading=heading;
    portEXIT_CRITICAL(&trialMux);
    ContinuousTrialLaunch();return true;
}
bool H30RunNodePath(const uint8_t *path,size_t count,float speedMmS,float finalHeading) {
    if(!H30StartNodePath(path,count,speedMmS,finalHeading,false))return false;
    while(ContinuousTrialActive())vTaskDelay(pdMS_TO_TICKS(10));
    return snapshot().state==DONE;
}
