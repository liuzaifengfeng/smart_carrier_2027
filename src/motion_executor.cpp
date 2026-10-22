#include "motion_executor.h"
#include "motion_hardware.h"
#include "motion_job.h"
#include "arm_motion.h"
#include "robot_runtime.h"
#include "continuous_trial.h"
#include "node_position_hardware.h"
#include "motor_bus.h"
#include <float.h>

// 保留旧参数注册位置以兼容上位机编号；观察姿态现按路径阶段提前触发。
float MOTION_LOOK_LEAD_MM=1500,MOTION_LOOK2_LEAD_MM=1500;
uint32_t ARM_FEEDBACK_ENABLED=0,ARM_GRIP_HOLD_MS=0,ARM_RELEASE_HOLD_MS=0;
namespace {
TaskHandle_t worker=nullptr;
QueueHandle_t queue=nullptr;
portMUX_TYPE jobMux=portMUX_INITIALIZER_UNLOCKED;
uint32_t nextId=0;
motion::LaunchGate launchGate;
bool ota=false,stopPending=false;
MotionStatus status,history[8];
size_t historyIndex=0;
struct Job {MotionRequest request;uint32_t id=0,generation=0;RobotState state=STATE_WAIT_START;int progress=0;} job;
motion::Program program;
motion::Sequence arm;
motion::Join join;
MotionHardware hardware;
bool running=false;
bool armSpanOpen=false;
uint8_t stage=0,scanAttempt=0;
RobotPose scanStart;
int64_t scanBefore[4]={};
float scanDistance=0;
const char *activeSpan=nullptr;

MotionStatus snapshot(){portENTER_CRITICAL(&jobMux);auto s=status;portEXIT_CRITICAL(&jobMux);return s;}
bool cancelled(){portENTER_CRITICAL(&jobMux);bool c=ota||launchGate.cancelled(job.generation);portEXIT_CRITICAL(&jobMux);return c||taskMotionAborted;}
void span(const char *name,bool begin) {
    Serial.printf("{EVT,MOTION,SPAN,%lu,%lu,%s,%s}\n",static_cast<unsigned long>(job.id),
        static_cast<unsigned long>(millis()),name,begin?"BEGIN":"END");
}
void phase(const char *name) {
    if(activeSpan&&strcmp(activeSpan,name)==0)return;
    if(activeSpan)span(activeSpan,false);activeSpan=name;span(name,true);
    portENTER_CRITICAL(&jobMux);status.phase=name;portEXIT_CRITICAL(&jobMux);
}
void flag(const char *name) {
    Serial.printf("{EVT,MOTION,FLAG,%lu,%lu,%s,1}\n",static_cast<unsigned long>(job.id),
        static_cast<unsigned long>(millis()),name);
}
void stopHardware() {
    ContinuousMotionCancel();
    CancelPositionMove();
    MotorBusGuard transaction;
    for(uint8_t id=5;id<=6;++id)Emm_V5_Stop_Now(id,false);
    // 保留夹爪当前命令，不在未知持料状态下松手。
}
void completedEvent(const MotionStatus &result) {
    const auto &r=job.request;const bool success=result.state==MotionState::DONE;
    if(r.kind==MotionKind::LIDAR_POSE&&r.category[0])CompleteLidarPose(success,r.debug);
    if(r.startVision) {
        const auto mode=static_cast<VisionStartMode>(r.vision);
        if(success&&currentState==job.state&&!taskMotionAborted)requestVisionStart(mode);
        else Serial.printf("{EVT,VISION,START_FAILED,%s,ARM}\n",mode==VisionStartMode::WORK_AREA?"WORK_AREA":"WORK_AREA_LOADED");
        return;
    }
    if(r.kind==MotionKind::DISC_GRAB) {
        if(r.position>=1&&r.position<=3)Serial.printf("{EVT,VISION,DISC_POSITIONS,%s,%u,%u,%u}\n",success?"DONE":"FAILED",r.position,r.color,r.cargo);
        else Serial.printf("{EVT,VISION,%s,%s,%u,%u}\n",r.force?"FORCE_GRAB":"GRAB",success?"DONE":"FAILED",r.color,r.cargo);
        if(r.automatic&&currentState==job.state) {
            if(!success) {
                requestVisionStop(VisionStartMode::DISC_MATERIAL);
                if(xHomeTimer)xTimerStop(xHomeTimer,0);
                currentState=STATE_TRANSFER_FAILED;
                updateDisplay("DISPLAY","DEBUG","DISC GRAB ERR");
            } else {
                const int next=AdvanceDiscGrabProgress(job.progress,r.cargo,true);
                if(next==3)requestVisionStop(VisionStartMode::DISC_MATERIAL);
                roundProgress=next;
            }
        }
        return;
    }
    if(r.category[0]) {
        if(r.kind==MotionKind::TRANSFER)Serial.printf("{EVT,ARM,TRANSFER,%s,%u,%u}\n",success?"DONE":"FAILED",r.cargo,r.work);
        else if(r.kind==MotionKind::RETRIEVE)Serial.printf("{EVT,ARM,RETRIEVE,%s,%u,%u}\n",success?"DONE":"FAILED",r.work,r.cargo);
        else if(r.kind==MotionKind::LIDAR_POSE)Serial.println(success?"{EVT,VISION,LIDAR_POSE_DONE}":"{EVT,VISION,LIDAR_POSE_FAILED}");
        else if(r.kind==MotionKind::POSITION)Serial.println(success?"{EVT,CHASSIS,MOVE_POSITION,DONE,ESTIMATED}":"{EVT,CHASSIS,MOVE_POSITION,FAILED}");
        else Serial.printf("{EVT,%s,%s,%s}\n",r.category,r.command,
            success?(r.kind==MotionKind::ARM_RAW||r.kind==MotionKind::ARM_POSE||r.kind==MotionKind::SERVO_RAW?"ISSUED":"DONE"):"FAILED");
    }
}
void finish(MotionState state,const char *reason=nullptr) {
    if(activeSpan){span(activeSpan,false);activeSpan=nullptr;}
    if(armSpanOpen){span("ARM_PROGRAM",false);armSpanOpen=false;}
    if(job.request.kind==MotionKind::NAVIGATE) {
        portENTER_CRITICAL(&nodePathMux);nodePathState=state==MotionState::DONE?NodePathState::DONE:NodePathState::FAILED;portEXIT_CRITICAL(&nodePathMux);
    }
    const uint32_t now=millis();
    portENTER_CRITICAL(&jobMux);
    status.finishedMs=now;status.elapsedMs=now-status.startedMs;status.state=state;status.reason=reason;
    status.chassisDone=join.chassisDone;status.armStarted=join.armStarted;status.armReady=join.armReady;
    const auto result=status;
    portEXIT_CRITICAL(&jobMux);
    Serial.printf("{EVT,MOTION,RESULT,%lu,%lu,%s,%lu}\n",static_cast<unsigned long>(job.id),
        static_cast<unsigned long>(now),MotionStateName(state),static_cast<unsigned long>(result.elapsedMs));
    if(reason)Serial.printf("{EVT,MOTION,ERROR,%lu,%lu,%s}\n",static_cast<unsigned long>(job.id),static_cast<unsigned long>(now),reason);
    completedEvent(result);
    portENTER_CRITICAL(&jobMux);history[historyIndex++%8]=result;portEXIT_CRITICAL(&jobMux);
    running=false;
}
bool beginArm(const MotionRequest &request) {
    portENTER_CRITICAL(&jobMux);status.phase="ARM_PREPARE";portEXIT_CRITICAL(&jobMux);
    const char *error=BuildArmProgram(request,program);
    if(error){stopHardware();finish(MotionState::FAILED,error);return false;}
    arm.begin(program,hardware);join.armStarted=true;join.armReady=false;
    span("ARM_PROGRAM",true);armSpanOpen=true;flag("ARM_STARTED");return true;
}
bool tickArm() {
    arm.tick();if(arm.active())return false;
    if(arm.failure()){
        portENTER_CRITICAL(&jobMux);status.phase="ARM_FAILED";portEXIT_CRITICAL(&jobMux);
        stopHardware();finish(MotionState::FAILED,arm.failure());return false;
    }
    if(!join.armReady){join.armReady=true;span("ARM_PROGRAM",false);armSpanOpen=false;flag("ARM_READY");}
    return true;
}
float endpointHeading(const MotionRequest &r) {
    if(isfinite(r.theta))return r.theta;
    const uint8_t end=r.kind==MotionKind::NAVIGATE?r.endNode:r.path[r.pathCount-1];
    return end==DISC_AREA_NODE?DISC_AREA_HEADING:end==COARSE_AREA_NODE?COARSE_AREA_HEADING:end==TEMP_AREA_NODE?TEMP_AREA_HEADING:NAN;
}
bool beginPath() {
    const auto &r=job.request;
    if(!H30PrepareNodePath(r.path,r.pathCount,r.kind==MotionKind::NAVIGATE?NODE_PATH_SPEED_MM_S:r.speed,endpointHeading(r),r.automatic)) {
        Serial.println("{EVT,NAV,ROUTE_FAILED,START}");finish(MotionState::FAILED,"START");return false;
    }
    phase("NAVIGATION");Serial.println("{EVT,NAV,ROUTE_RUNNING}");
    ContinuousMotionBegin();stage=2;join.armReady=r.observation==ArmObservation::NONE;
    return true;
}
bool pollTaskCode() {
    if(taskReceived)return true;
    char code[SCANNER_BUF_LEN];
    if(waitScannerCode(code,sizeof(code),0)) {
        const auto task=parseTaskCode(code);
        portENTER_CRITICAL(&taskCodeMux);if(!taskReceived&&task.valid){currentTask=task;taskReceived=true;}portEXIT_CRITICAL(&taskCodeMux);
        Serial.printf("[SCANNER] task code %s\n",task.valid?"OK":"ERR");
    }
    return taskReceived;
}
void recordScanTravel(const RobotPose &start,float distance) {
    const continuous::Vec d=continuous::bodyToWorld({0,distance},start.theta);
    currentPose={start.x+d.x,start.y+d.y,start.theta};
}
void scanTick() {
    const float direction=job.request.zone==1?1:-1;
    if(stage==0) {
        MotionRequest r;r.kind=MotionKind::ARM_POSE;r.observation=ArmObservation::START;
        if(!beginArm(r))return;stage=1;return;
    }
    if(stage==1) {
        if(!tickArm()||!running)return;
        if(pollTaskCode()){finish(MotionState::DONE);return;}
        scanStart=currentPose;
        ScanMotionSettings cfg{direction*700,direction*400,Y_PULSE,static_cast<uint16_t>(SCAN_APPROACH_SPEED_RPM),static_cast<uint16_t>(SCAN_SEARCH_SPEED_RPM)};
        BeginContinuousScan(cfg,pollTaskCode);stage=2;phase("SCAN_APPROACH");return;
    }
    if(stage==2) {
        TickContinuousScan();if(ContinuousScanRunner().active())return;
        recordScanTravel(scanStart,ContinuousScanRunner().forwardTravel());
        if(ContinuousScanRunner().failure()){finish(MotionState::FAILED,ContinuousScanRunner().failure());return;}
        if(pollTaskCode()){finish(MotionState::DONE);return;}
        scanAttempt=0;stage=3;
    }
    if(stage==3) {
        if(pollTaskCode()){finish(MotionState::DONE);return;}
        if(scanAttempt==3){finish(MotionState::FAILED,"EXECUTION");return;}
        ++scanAttempt;scanDistance=direction*(scanAttempt%2?-400:400);scanStart=currentPose;
        if(!ReadChassisEncoderPositions(scanBefore)||!BeginPositionMove(0,scanDistance,0,SCAN_SEARCH_SPEED_RPM)){finish(MotionState::FAILED,"EXECUTION");return;}
        stage=4;phase("SCAN_SEARCH");return;
    }
    if(stage==4) {
        const bool received=pollTaskCode();if(received&&PositionMoveActive())CancelPositionMove();else TickPositionMove();
        if(PositionMoveActive())return;
        int64_t after[4];if(!ReadChassisEncoderPositions(after)){finish(MotionState::FAILED,"EXECUTION");return;}
        double units=0;for(int i=0;i<4;++i)units+=fabs(static_cast<double>(after[i]-scanBefore[i]))/4;
        recordScanTravel(scanStart,(scanDistance>=0?1:-1)*units*3200/65536/Y_PULSE);
        if(received)finish(MotionState::DONE);else stage=3;
    }
}
void navigationTick() {
    auto &r=job.request;
    if(stage==0&&r.kind==MotionKind::NAVIGATE) {
        portENTER_CRITICAL(&nodePathMux);nodePathState=NodePathState::WAITING;nodePathLen=0;portEXIT_CRITICAL(&nodePathMux);
        Serial.println("{EVT,NAV,ROUTE_WAITING}");Serial.printf("{EVT,NAV,ROUTE_REQUEST,%u,%u}\n",r.startNode,r.endNode);
        phase("PATH_WAIT");stage=1;return;
    }
    if(stage==1) {
        portENTER_CRITICAL(&nodePathMux);
        const auto state=nodePathState;
        if(state==NodePathState::RECEIVED){r.pathCount=nodePathLen;memcpy(r.path,nodePathBuffer,nodePathLen);nodePathState=NodePathState::RUNNING;}
        portEXIT_CRITICAL(&nodePathMux);
        if(state==NodePathState::FAILED){finish(MotionState::FAILED,"STOPPED");return;}
        if(state!=NodePathState::RECEIVED)return;
        if(r.path[0]!=r.startNode||r.path[r.pathCount-1]!=r.endNode){Serial.println("{EVT,NAV,ROUTE_FAILED,ENDPOINT}");finish(MotionState::FAILED,"ENDPOINT");return;}
        if(!beginPath())return;
    } else if(stage==0) {if(!beginPath())return;}
    if(!join.chassisDone) {
        ContinuousMotionTick();
        if(!ContinuousMotionActive()) {
            if(ContinuousMotionFailure()){
                portENTER_CRITICAL(&jobMux);status.phase="NAVIGATION";portEXIT_CRITICAL(&jobMux);
                finish(MotionState::FAILED,ContinuousMotionFailure());return;
            }
            join.chassisDone=true;flag("CHASSIS_DONE");
            if(activeSpan){span(activeSpan,false);activeSpan=nullptr;}
        }
    }
    auto &base=PositionNodeRunner();
    const bool visualPose=r.observation==ArmObservation::LOOK||r.observation==ArmObservation::LOOK2||r.observation==ArmObservation::LOOK4;
    if(r.observation!=ArmObservation::NONE&&join.shouldStartArm(join.chassisDone,visualPose&&base.observationApproach())) {
        MotionRequest pose;pose.kind=MotionKind::ARM_POSE;pose.observation=r.observation;pose.zone=r.zone;
        if(!beginArm(pose))return;
    }
    if(join.armStarted&&!join.armReady)tickArm();
    if(!running)return;
    portENTER_CRITICAL(&jobMux);
    status.chassisDone=join.chassisDone;status.armStarted=join.armStarted;status.armReady=join.armReady;
    status.lastTranslation=base.lastTranslation();status.remainingMm=base.remainingDistance();status.terminalTurn=base.hasTerminalTurn();status.progress=base.progress();
    portEXIT_CRITICAL(&jobMux);
    if(join.complete())finish(MotionState::DONE);
}
void task(void *) {
    for(;;) {
        if(!running) {
            portENTER_CRITICAL(&jobMux);bool stop=stopPending;stopPending=false;portEXIT_CRITICAL(&jobMux);
            if(stop)stopHardware();
            Job pending;
            const bool queued=xQueuePeek(queue,&pending,0)==pdTRUE;
            portENTER_CRITICAL(&jobMux);
            const bool released=queued&&launchGate.runnable(pending.id,pending.generation,ota,taskMotionAborted);
            portEXIT_CRITICAL(&jobMux);
            if(!released||xQueueReceive(queue,&job,0)!=pdTRUE) {
                TickVisualAlignment();
                ulTaskNotifyTake(pdTRUE,alignmentEnabled?pdMS_TO_TICKS(20):portMAX_DELAY);continue;
            }
            running=true;stage=0;join=motion::Join{};activeSpan=nullptr;armSpanOpen=false;
            hardware.configure(job.id,cancelled);
            setAlignmentEnabled(false);
            TickVisualAlignment(); // 旧视觉停车在新位置目标装载前完成，不能在同步触发后覆盖它。
            portENTER_CRITICAL(&jobMux);status.state=MotionState::RUNNING;status.startedMs=millis();portEXIT_CRITICAL(&jobMux);
        }
        if(cancelled()){
            portENTER_CRITICAL(&jobMux);stopPending=false;portEXIT_CRITICAL(&jobMux);
            stopHardware();finish(MotionState::CANCELLED,taskMotionAborted?"ABORTED":"STOPPED");continue;
        }
        const auto kind=job.request.kind;
        if(kind==MotionKind::NAVIGATE||kind==MotionKind::NODE_PATH)navigationTick();
        else if(kind==MotionKind::SCAN_TASK)scanTick();
        else if(kind==MotionKind::TRIAL) {
            if(stage==0){ContinuousMotionBegin();stage=1;phase("TRIAL");}
            ContinuousMotionTick();
            if(!ContinuousMotionActive())finish(ContinuousMotionFailure()?MotionState::FAILED:MotionState::DONE,ContinuousMotionFailure());
        } else {
            if(stage==0) {
                if(job.request.force&&kind==MotionKind::DISC_GRAB&&cargoPlatforms[job.request.cargo-1].material!=MATERIAL_NONE)
                    Serial.printf("{EVT,VISION,FORCE_GRAB,WARN,OCCUPIED,%u,%u}\n",cargoPlatforms[job.request.cargo-1].material,job.request.cargo);
                if(beginArm(job.request))stage=1;
            } else if(tickArm()&&running)finish(MotionState::DONE);
        }
        if(!running)continue;
        if(join.armStarted&&!join.armReady){portENTER_CRITICAL(&jobMux);status.phase=arm.phase();portEXIT_CRITICAL(&jobMux);}
        TickVisualAlignment();
        uint32_t delay=20;
        if((kind==MotionKind::NAVIGATE||kind==MotionKind::NODE_PATH)&&stage==2&&!join.chassisDone)delay=ContinuousMotionNextDelay();
        if(join.armStarted&&!join.armReady){const uint32_t next=arm.nextDelay();if(next<delay)delay=next;}
        if(kind==MotionKind::TRIAL)delay=ContinuousMotionNextDelay();
        // 通知可由新视觉帧、取消或控制请求提前唤醒；没有新增稳定窗口。
        ulTaskNotifyTake(pdTRUE,delay?pdMS_TO_TICKS(delay):1);
    }
}
}
bool MotionExecutorInit() {
    if(worker)return true;
    queue=xQueueCreate(1,sizeof(Job));if(!queue)return false;
    return xTaskCreate(task,"MotionExecutor",12288,nullptr,6,&worker)==pdPASS;
}
void WakeMotionExecutor(){if(worker)xTaskNotifyGive(worker);}
uint32_t CurrentMotionId(){return job.id;}
uint32_t SubmitMotion(const MotionRequest &request,bool launch) {
    Job queued;queued.request=request;queued.state=currentState;queued.progress=roundProgress;
    portENTER_CRITICAL(&jobMux);
    if(!worker||ota||taskMotionAborted||status.state==MotionState::RUNNING||status.state==MotionState::QUEUED){portEXIT_CRITICAL(&jobMux);return 0;}
    queued.id=++nextId;queued.generation=launchGate.generation;status=MotionStatus{};status.id=queued.id;
    status.submittedMs=millis();status.state=MotionState::QUEUED;status.phase="QUEUED";
    launchGate.prepare(queued.id,launch);
    portEXIT_CRITICAL(&jobMux);
    if(xQueueSend(queue,&queued,0)!=pdTRUE)return 0;
    WakeMotionExecutor();return queued.id;
}
void LaunchMotion(uint32_t id){portENTER_CRITICAL(&jobMux);if(status.id==id&&status.state==MotionState::QUEUED)launchGate.release(id);portEXIT_CRITICAL(&jobMux);WakeMotionExecutor();}
MotionStatus GetMotionStatus(uint32_t id) {
    portENTER_CRITICAL(&jobMux);MotionStatus value;
    if(status.id==id)value=status;else for(const auto &entry:history)if(entry.id==id){value=entry;break;}
    if(value.state==MotionState::RUNNING)value.elapsedMs=millis()-value.startedMs;
    portEXIT_CRITICAL(&jobMux);return value;
}
bool MotionExecutorActive(){const auto s=snapshot();return s.state==MotionState::QUEUED||s.state==MotionState::RUNNING;}
void CancelMotion(){
    portENTER_CRITICAL(&jobMux);launchGate.cancel();stopPending=true;portEXIT_CRITICAL(&jobMux);
    ContinuousTrialCancel();setAlignmentEnabled(false);
    if(discMaterialActive){firstDiscGrabReady=false;requestVisionStop(VisionStartMode::DISC_MATERIAL);}
    // 显式停止/OTA 结束当前业务，不能在两个作业之间恢复旧阶段。
    // 沿用已有故障返回状态，不创建故障锁或自动重试。
    if(enableRun) {
        const RobotState state=currentState;
        if(state==STATE_READ_TASK)currentState=STATE_SCAN_FAILED;
        else if(state==STATE_GRAB_ROUND1||state==STATE_GRAB_ROUND2||state==STATE_PLACE_COARSE1||
            state==STATE_PLACE_COARSE2||state==STATE_PLACE_TEMP1||state==STATE_STACK_TEMP2||state==STATE_RETURN_HOME)
            currentState=STATE_TRANSFER_FAILED;
    }
    WakeMotionExecutor();
}
void MotionExecutorSetOta(bool active){portENTER_CRITICAL(&jobMux);ota=active;portEXIT_CRITICAL(&jobMux);if(active)CancelMotion();}
bool WaitMotion(uint32_t id){if(!id)return false;for(;;){const auto s=GetMotionStatus(id);if(s.complete())return s.state==MotionState::DONE;vTaskDelay(pdMS_TO_TICKS(10));}}
