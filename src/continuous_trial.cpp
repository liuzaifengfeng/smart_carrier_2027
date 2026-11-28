// 连续导航试验及节点路径执行入口。
// 串口上传、路径准备、执行和结束分别维护状态；底盘轮询推进时可以同时检查停止请求和航向数据。

#include "continuous_trial.h"  // 引入 continuous_trial.h 中的声明和定义。
#include "robot_runtime.h"  // 引入 robot_runtime.h 中的声明和定义。
#include "h30_usb.h"  // 引入 h30_usb.h 中的声明和定义。
#include "field_geometry.h"  // 引入 field_geometry.h 中的声明和定义。
#include "node_position_hardware.h"  // 引入 node_position_hardware.h 中的声明和定义。
#include "motion_executor.h"  // 引入 motion_executor.h 中的声明和定义。

namespace {  // 开始匿名命名空间，本文件内部共享其中的定义。
using namespace continuous;  // 引入名称或类型别名 namespace continuous。
Route route;  // 声明 route（路线），类型和可用范围由所在声明决定。
Node nodes[MAX_NODES];  // 声明 nodes（节点序列），类型和可用范围由所在声明决定。
uint8_t nodeIds[MAX_NODES]={};  // 将 {} 写入或初始化 nodeIds[MAX_NODES]。
Config staged;  // 声明 staged，类型和可用范围由所在声明决定。
node_position::Settings formalSettings;  // 声明 formalSettings（formal设置），类型和可用范围由所在声明决定。
uint32_t receivedMask=0,referenceVersion=0,failedVersion=0;  // 将 0,referenceVersion=0,failedVersion=0 写入或初始化 receivedMask（已接收Mask）。
float initialHeading=0;  // 将 0 写入或初始化 initialHeading（initial航向）。
bool configured=false,needsCalibration=false,consumed=false;  // 将 false,needsCalibration=false,consumed=false 写入或初始化 configured。
bool policyReceived=false;  // 将 false 写入或初始化 policyReceived（policy已接收）。
bool initialized=false;  // 将 false 写入或初始化 initialized。
SemaphoreHandle_t configurationLock=nullptr;  // 将 nullptr 写入或初始化 configurationLock。
struct ConfigurationGuard {  // 定义 ConfigurationGuard 的类型成员或作用域。
    bool held;  // 声明 held，类型和可用范围由所在声明决定。
    ConfigurationGuard():held(configurationLock&&xSemaphoreTake(configurationLock,0)==pdTRUE){}  // 调用 ConfigurationGuard，传入 ):held(configurationLock&&xSemaphoreTake(configuration…。
    ~ConfigurationGuard(){if(held)xSemaphoreGive(configurationLock);}  // 定义函数或成员 ConfigurationGuard，参数由此处的签名确定。
};  // 结束此类型定义。
portMUX_TYPE trialMux=portMUX_INITIALIZER_UNLOCKED;  // 将 portMUX_INITIALIZER_UNLOCKED 写入或初始化 trialMux（trial临界区锁）。
enum State {IDLE,RUNNING,STOPPING,DONE,FAILED};  // 定义 State（状态） 的类型成员或作用域。
struct Snapshot {  // 定义 Snapshot 的类型成员或作用域。
    bool formal=false,async=false;  // 将 false,async=false 写入或初始化 formal。
    uint32_t id=0,heartbeatMs=0;  // 将 0,heartbeatMs=0 写入或初始化 id（编号）。
    State state=IDLE;  // 将 IDLE 写入或初始化 state（状态）。
    float s=0,total=0,x=0,y=0,reference=0,heading=0,error=0,cycleMs=0;  // 将 0,total=0,x=0,y=0,reference=0,heading=0,error=0,cycleM… 写入或初始化 s。
    int16_t rpm[4]={};  // 将 {} 写入或初始化 rpm[4]。
    const char *reason="NONE";  // 将 "NONE" 写入或初始化 reason。
} status;  // 续写当前声明或运算，提供 } status;。
bool stopRequested=false;  // 将 false 写入或初始化 stopRequested（停止Requested）。
bool otaActive=false;  // 将 false 写入或初始化 otaActive（固件更新活跃）。
const char *names[]={"IDLE","RUNNING","STOPPING","DONE","FAILED"};  // 声明 names，类型和可用范围由所在声明决定。
Snapshot snapshot() {portENTER_CRITICAL(&trialMux);auto v=status;portEXIT_CRITICAL(&trialMux);return v;}  // 将 status;portEXIT_CRITICAL(&trialMux);return v;} 写入或初始化 v。
bool stopped() {portENTER_CRITICAL(&trialMux);bool v=stopRequested;portEXIT_CRITICAL(&trialMux);return v;}  // 将 stopRequested;portEXIT_CRITICAL(&trialMux);return v;} 写入或初始化 v。
bool otaBusy(){portENTER_CRITICAL(&trialMux);bool v=otaActive;portEXIT_CRITICAL(&trialMux);return v;}  // 将 otaActive;portEXIT_CRITICAL(&trialMux);return v;} 写入或初始化 v。
const char *headingSample(float &heading,uint32_t &sampleMs,H30Attitude *motion=nullptr);  // 声明接口 headingSample（航向采样），参数由此处的签名确定。
void hardStop() {  // 定义函数或成员 的入口 hardStop（hard停止），参数由此处的签名确定。
    for(uint8_t i=1;i<=4;++i){Emm_V5_Stop_Now(i,true);vTaskDelay(pdMS_TO_TICKS(5));}  // 按 uint8_t i=1;i<=4;++i 组织循环的初值、继续条件和步进。
    Emm_V5_Synchronous_motion(0);Serial1.flush();  // 调用 Emm_V5_Synchronous_motion，传入 0。然后调用 Serial1.flush。
}  // 结束此函数或控制块。
// 整组速度先装载，再同步生效；停车请求在同步提交之前再次检查。
const char *sendWheels(const WheelCommand &v,uint32_t cycleStart) {  // 定义函数或成员 的入口 sendWheels（send四轮），参数由此处的签名确定。
    for(uint8_t i=1;i<=4;++i){  // 按 uint8_t i=1;i<=4;++i 组织循环的初值、继续条件和步进。
        int16_t rpm=v.rpm[i-1];  // 将 v.rpm[i-1] 写入或初始化 rpm（转速）。
        Emm_V5_Vel_Control(i,rpm>=0?1:0,abs(rpm),0,true);  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 i,rpm>=0?1:0,abs(rpm),0,true。
        vTaskDelay(pdMS_TO_TICKS(5));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    }  // 结束此循环体。
    if(stopped())return "STOPPED";  // 若 stopped()，立即返回 "STOPPED"。
    if(millis()-cycleStart>120)return "CONTROL_OVERRUN";  // 若 millis()-cycleStart>120，立即返回 "CONTROL_OVERRUN"。
    float heading=0;uint32_t sample=0;  // 将 0 写入或初始化 heading（航向）。然后将 0 写入或初始化 sample（采样）。
    const char *error=headingSample(heading,sample);if(error)return error;  // 将 headingSample(heading,sample) 写入或初始化 error（错误）。然后若 error，立即返回 error。
    Emm_V5_Synchronous_motion(0);Serial1.flush();return nullptr;  // 调用 Emm_V5_Synchronous_motion，传入 0。然后调用 Serial1.flush。然后结束当前函数并返回 nullptr。
}  // 结束此函数或控制块。
void telemetry(const Snapshot &v) {  // 定义函数或成员 的入口 telemetry，参数由此处的签名确定。
    Serial.printf("{EVT,NAV,%s,%lu,%.2f,%.2f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%.1f}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
        v.formal?"ROUTE_SAMPLE":"TRIAL_SAMPLE",static_cast<unsigned long>(v.id),v.s,v.total,v.reference,v.heading,v.error,  // 定义函数或成员 long>，参数由此处的签名确定。
        v.rpm[0],v.rpm[1],v.rpm[2],v.rpm[3],v.cycleMs);  // 续写当前声明或运算，提供 v.rpm[0],v.rpm[1],v.rpm[2],v.rpm[3],v.cycleMs);。
}  // 结束此函数或控制块。
const char *headingSample(float &heading,uint32_t &sampleMs,H30Attitude *motion) {  // 定义函数或成员 的入口 headingSample（航向采样），参数由此处的签名确定。
    uint32_t version=0;  // 将 0 写入或初始化 version（版本）。
    H30Attitude frame;  // 声明 frame（帧），类型和可用范围由所在声明决定。
    const char *error=H30UsbGetMotionSnapshot(frame,heading,version);  // 将 H30UsbGetMotionSnapshot(frame,heading,version) 写入或初始化 error（错误）。
    sampleMs=frame.receivedAtMs;if(motion)*motion=frame;  // 将 frame.receivedAtMs 写入或初始化 sampleMs（采样毫秒）。然后当 motion 成立时进入此分支，否则继续后续流程。
    if(error)return error;  // 若 error，立即返回 error。
    if(version!=referenceVersion)return "REFERENCE_CHANGED";  // 若 version!=referenceVersion，立即返回 "REFERENCE_CHANGED"。
    if(millis()-sampleMs>100)return "IMU_STALE";  // 若 millis()-sampleMs>100，立即返回 "IMU_STALE"。
    return nullptr;  // 结束当前函数并返回 nullptr。
}  // 结束此函数或控制块。
struct Execution {  // 定义 Execution 的类型成员或作用域。
    Tracker tracker;  // 声明 tracker，类型和可用范围由所在声明决定。
    Snapshot job;  // 声明 job，类型和可用范围由所在声明决定。
    node_position::Result position;  // 声明 position（位置），类型和可用范围由所在声明决定。
    bool active=false,success=false;  // 将 false,success=false 写入或初始化 active（活跃）。
    const char *failure=nullptr;  // 将 nullptr 写入或初始化 failure。
    float actual=0;  // 将 0 写入或初始化 actual（实测）。
    uint32_t sampleMs=0,started=0,previousCycle=0,integratedAt=0,lastTelemetry=0,timeout=0,due=0;  // 将 0,started=0,previousCycle=0,integratedAt=0,lastTelemet… 写入或初始化 sampleMs（采样毫秒）。
} execution;  // 续写当前声明或运算，提供 } execution;。
void finishExecution() {  // 定义函数或成员 的入口 finishExecution，参数由此处的签名确定。
    auto &e=execution;  // 将 execution 写入或初始化 e。
    hardStop();  // 调用 hardStop（hard停止），使用无参形式推进相应操作。
    if(e.job.formal) {  // 当 e.job.formal 成立时进入此分支，否则继续后续流程。
        e.tracker.estimated=e.position.estimated;e.tracker.s=e.position.s;e.actual=e.position.heading;  // 将 e.position.estimated 写入或初始化 e.tracker.estimated。然后将 e.position.s 写入或初始化 e.tracker.s。然后将 e.position.heading 写入或初始化 e.actual。
    } else e.tracker.integrate(route,e.actual,(millis()-e.integratedAt)/1000.0f);  // 前一个判断不成立时执行此分支。
    currentPose={e.tracker.estimated.x,e.tracker.estimated.y,wrap(e.actual)};  // 将 {e.tracker.estimated.x,e.tracker.estimated.y,wrap(e.ac… 写入或初始化 currentPose（当前Pose）。
    if(!e.success){needsCalibration=true;failedVersion=referenceVersion;}  // 当 !e.success 成立时进入此分支，否则继续后续流程。
    portENTER_CRITICAL(&trialMux);  // 进入临界区，保护共享状态的本段访问。
    status.x=e.tracker.estimated.x;status.y=e.tracker.estimated.y;status.heading=e.actual;  // 将 e.tracker.estimated.x 写入或初始化 status.x。然后将 e.tracker.estimated.y 写入或初始化 status.y。然后将 e.actual 写入或初始化 status.heading。
    status.s=e.tracker.s;status.reason=e.success?"ESTIMATED":e.failure;  // 将 e.tracker.s 写入或初始化 status.s。然后将 e.success?"ESTIMATED":e.failure 写入或初始化 status.reason。
    for(int i=0;i<4;++i)status.rpm[i]=0;  // 按 int i=0;i<4;++i)status.rpm[i]=0; 组织循环的初值、继续条件和步进。
    portEXIT_CRITICAL(&trialMux);  // 离开临界区，保护共享状态的本段访问。
    Serial.printf("{EVT,NAV,%s,%lu,%s,%s,%.2f,%.2f,%.3f}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
        e.job.formal?"ROUTE_RESULT":"TRIAL_RESULT",static_cast<unsigned long>(e.job.id),e.success?"DONE":"FAILED",e.success?"ESTIMATED":e.failure,  // 定义函数或成员 long>，参数由此处的签名确定。
        e.tracker.estimated.x,e.tracker.estimated.y,e.actual);  // 续写当前声明或运算，提供 e.tracker.estimated.x,e.tracker.estimated.y,e.actual);。
    if(e.job.formal&&e.job.async)Serial.printf("{EVT,NAV,%s,%s}\n",e.success?"ROUTE_DONE":"ROUTE_FAILED",e.success?"ESTIMATED":e.failure);  // 当 e.job.formal&&e.job.async 成立时进入此分支，否则继续后续流程。
    portENTER_CRITICAL(&trialMux);status.state=e.success?DONE:FAILED;portEXIT_CRITICAL(&trialMux);  // 进入临界区，保护共享状态的本段访问。然后将 e.success?DONE:FAILED 写入或初始化 status.state。然后离开临界区，保护共享状态的本段访问。
    e.active=false;  // 将 false 写入或初始化 e.active。
}  // 结束此函数或控制块。
bool adjacent(uint8_t a,uint8_t b) {  // 定义函数或成员 的入口 adjacent，参数由此处的签名确定。
    int ar=a/5,br=b/5,ac=a%5,bc=b%5;if(ar%2)ac=4-ac;if(br%2)bc=4-bc;  // 将 a/5,br=b/5,ac=a%5,bc=b%5 写入或初始化 ar。然后当 ar%2 成立时进入此分支，否则继续后续流程。然后当 br%2 成立时进入此分支，否则继续后续流程。
    return abs(ar-br)+abs(ac-bc)==1;  // 结束当前函数并返回 abs(ar-br)+abs(ac-bc)==1。
}  // 结束此函数或控制块。
}  // 结束命名空间。

/**
 * @brief 初始化连续路径试跑模块，准备其状态和同步资源。
 */
bool ContinuousTrialInit() {  // 定义函数或成员 的入口 ContinuousTrialInit，参数由此处的签名确定。
    if(!configurationLock)configurationLock=xSemaphoreCreateMutex();  // 当 !configurationLock 成立时进入此分支，否则继续后续流程。
    initialized=configurationLock!=nullptr;return initialized;  // 将 configurationLock!=nullptr 写入或初始化 initialized。然后结束当前函数并返回 initialized。
}  // 结束此函数或控制块。
/**
 * @brief 判断独立连续路径试跑是否正在运行或停止中。
 */
bool ContinuousTrialActive(){auto v=snapshot();return v.state==RUNNING||v.state==STOPPING;}  // 将 snapshot();return v.state==RUNNING||v.state==STOPPING;} 写入或初始化 v。
/**
 * @brief 建立指定编号的试跑配置，供后续写入节点及策略；成功返回 nullptr。
 */
const char *ContinuousTrialBegin(const continuous::Config &config) {  // 定义函数或成员 的入口 ContinuousTrialBegin，参数由此处的签名确定。
    ConfigurationGuard guard;if(!guard.held)return "BUSY";  // 声明 guard，类型和可用范围由所在声明决定。然后若 !guard.held，立即返回 "BUSY"。
    if(ContinuousTrialActive())return "BUSY";  // 若 ContinuousTrialActive()，立即返回 "BUSY"。
    if(otaBusy())return "OTA";  // 若 otaBusy()，立即返回 "OTA"。
    if(!initialized)return "INIT";  // 若 !initialized，立即返回 "INIT"。
    if(config.id==0||config.id>9999999||config.count<2||config.count>continuous::MAX_NODES)return "RANGE";  // 若 config.id==0||config.id>9999999||config.count<2||confi…，立即返回 "RANGE"。
    if(configured&&config.id==staged.id)return "ID_REUSED";  // 若 configured&&config.id==staged.id，立即返回 "ID_REUSED"。
    staged=config;staged.policy=0;receivedMask=0;configured=true;consumed=false;policyReceived=false;  // 将 config 写入或初始化 staged。然后将 0 写入或初始化 staged.policy。然后将 0 写入或初始化 receivedMask（已接收Mask）。然后将 true 写入或初始化 configured。然后将 false 写入或初始化 consumed。然后将 false 写入或初始化 policyReceived（policy已接收）。
    portENTER_CRITICAL(&trialMux);status=Snapshot{};status.id=config.id;portEXIT_CRITICAL(&trialMux);  // 将 Snapshot{};status.id=config.id;portEXIT_CRITICAL(&tria… 写入或初始化 status（状态）。
    return nullptr;  // 结束当前函数并返回 nullptr。
}  // 结束此函数或控制块。
/**
 * @brief 设置试跑路径策略、外扩和几何版本；成功返回 nullptr。
 */
const char *ContinuousTrialPolicy(uint32_t id,uint8_t policy,float outset,float margin,uint32_t geometry) {  // 定义函数或成员 的入口 ContinuousTrialPolicy，参数由此处的签名确定。
    ConfigurationGuard guard;if(!guard.held)return "BUSY";  // 声明 guard，类型和可用范围由所在声明决定。然后若 !guard.held，立即返回 "BUSY"。
    if(ContinuousTrialActive())return "BUSY";  // 若 ContinuousTrialActive()，立即返回 "BUSY"。
    if(!configured||id!=staged.id)return "ID";  // 若 !configured||id!=staged.id，立即返回 "ID"。
    if(geometry!=field::signature())return "GEOMETRY_MISMATCH";  // 若 geometry!=field::signature()，立即返回 "GEOMETRY_MISMATCH"。
    if((policy!=1&&policy!=2)||!isfinite(outset)||outset<0||outset>100  // 当 (policy!=1&&policy!=2)||!isfinite(outset)||outset<0||o… 成立时进入此分支，否则继续后续流程。
        ||!isfinite(margin)||margin<10||margin>40)return "POLICY_RANGE";  // 继续上一行的组合条件，本行提供 ||!isfinite(margin)||margin<10||margin>40)return "POLICY_RANGE";。
    staged.policy=policy;staged.outset=outset;staged.margin=margin;policyReceived=true;return nullptr;  // 将 policy 写入或初始化 staged.policy。然后将 outset 写入或初始化 staged.outset。然后将 margin 写入或初始化 staged.margin。然后将 true 写入或初始化 policyReceived（policy已接收）。然后结束当前函数并返回 nullptr。
}  // 结束此函数或控制块。
/**
 * @brief 写入一个试跑节点及转弯半径，并核对上位机期望坐标；成功返回 nullptr。
 */
const char *ContinuousTrialNode(uint32_t id,uint8_t index,uint8_t node,float radius,float expectedX,float expectedY) {  // 定义函数或成员 的入口 ContinuousTrialNode（ContinuousTrial节点），参数由此处的签名确定。
    ConfigurationGuard guard;if(!guard.held)return "BUSY";  // 声明 guard，类型和可用范围由所在声明决定。然后若 !guard.held，立即返回 "BUSY"。
    if(ContinuousTrialActive())return "BUSY";  // 若 ContinuousTrialActive()，立即返回 "BUSY"。
    if(!configured||id!=staged.id)return "ID";  // 若 !configured||id!=staged.id，立即返回 "ID"。
    if(index>=staged.count||node>=25||!isfinite(radius)||radius<0||radius>1000)return "RANGE";  // 若 index>=staged.count||node>=25||!isfinite(radius)||radi…，立即返回 "RANGE"。
    float x=0,y=0;if(!ChassisGetNodePosition(node,x,y))return "NODE";  // 将 0,y=0 写入或初始化 x（横向）。然后若 !ChassisGetNodePosition(node,x,y)，立即返回 "NODE"。
    if(!isfinite(expectedX)||!isfinite(expectedY)||fabsf(x-expectedX)>0.05f||fabsf(y-expectedY)>0.05f)return "MAP_MISMATCH";  // 若 !isfinite(expectedX)||!isfinite(expectedY)||fabsf(x-ex…，立即返回 "MAP_MISMATCH"。
    nodes[index].p=continuous::Vec(x,y);nodes[index].radius=radius;nodeIds[index]=node;  // 将 continuous::Vec(x,y) 写入或初始化 p。然后将 radius 写入或初始化 radius。然后将 node 写入或初始化 nodeIds[index]。
    receivedMask|=1u<<index;return nullptr;  // 用 1u<<index 对 receivedMask（已接收Mask） 应用 |= 更新。然后结束当前函数并返回 nullptr。
}  // 结束此函数或控制块。
/**
 * @brief 完成试跑准备并登记RUNNING；实际启动等待ACK后的Launch通知。
 * @return nullptr 准备成功，否则为错误原因。
 */
const char *ContinuousTrialPrepare(uint32_t id) {  // 定义函数或成员 的入口 ContinuousTrialPrepare，参数由此处的签名确定。
    ConfigurationGuard guard;if(!guard.held)return "BUSY";  // 声明 guard，类型和可用范围由所在声明决定。然后若 !guard.held，立即返回 "BUSY"。
    if(ContinuousTrialActive())return "BUSY";  // 若 ContinuousTrialActive()，立即返回 "BUSY"。
    if(otaBusy())return "OTA";  // 若 otaBusy()，立即返回 "OTA"。
    if(!configured||id!=staged.id)return "ID";  // 若 !configured||id!=staged.id，立即返回 "ID"。
    if(!policyReceived)return "POLICY_REQUIRED";  // 若 !policyReceived，立即返回 "POLICY_REQUIRED"。
    if(receivedMask!=((1u<<staged.count)-1))return "INCOMPLETE";  // 若 receivedMask!=((1u<<staged.count)-1)，立即返回 "INCOMPLETE"。
    if(consumed)return "ID_REUSED";  // 若 consumed，立即返回 "ID_REUSED"。
    if(taskMotionAborted)return "ABORTED";  // 若 taskMotionAborted，立即返回 "ABORTED"。
    for(size_t i=1;i<staged.count;++i)if(!adjacent(nodeIds[i-1],nodeIds[i]))return "NOT_ADJACENT";  // 按 size_t i=1;i<staged.count;++i)if(!adjacent(nodeIds[i-1… 组织循环的初值、继续条件和步进。
    // 不允许上传期间地图改变后静默执行不同于上位机预览的轨迹。
    for(size_t i=0;i<staged.count;++i) {  // 按 size_t i=0;i<staged.count;++i 组织循环的初值、继续条件和步进。
        float x=0,y=0;  // 将 0,y=0 写入或初始化 x（横向）。
        if(!ChassisGetNodePosition(nodeIds[i],x,y))return "NODE";  // 若 !ChassisGetNodePosition(nodeIds[i],x,y)，立即返回 "NODE"。
        if(fabsf(x-nodes[i].p.x)>0.05f||fabsf(y-nodes[i].p.y)>0.05f)return "MAP_CHANGED";  // 若 fabsf(x-nodes[i].p.x)>0.05f||fabsf(y-nodes[i].p.y)>0.0…，立即返回 "MAP_CHANGED"。
    }  // 结束此循环体。
    staged.rotationPpd=THETA_PULSE;  // 将 THETA_PULSE 写入或初始化 staged.rotationPpd。
    const char *error=route.build(nodes,staged);if(error)return error;  // 将 route.build(nodes,staged) 写入或初始化 error（错误）。然后若 error，立即返回 error。
    if(!isfinite(currentPose.x)||!isfinite(currentPose.y)  // 当 !isfinite(currentPose.x)||!isfinite(currentPose.y 成立时进入此分支，否则继续后续流程。
        ||hypotf(currentPose.x-nodes[0].p.x,currentPose.y-nodes[0].p.y)>25)return "START_POSITION";  // 继续上一行的组合条件，本行提供 ||hypotf(currentPose.x-nodes[0].p.x,currentPose.y-nodes[0].p.y)>25)return "START_POSITION…。
    float actual=0;uint32_t sample=0,version=0;  // 将 0 写入或初始化 actual（实测）。然后将 0,version=0 写入或初始化 sample（采样）。
    H30Attitude motion;  // 声明 motion，类型和可用范围由所在声明决定。
    error=H30UsbGetMotionSnapshot(motion,actual,version);sample=motion.receivedAtMs;if(error)return error;  // 将 H30UsbGetMotionSnapshot(motion,actual,version) 写入或初始化 error（错误）。然后将 motion.receivedAtMs 写入或初始化 sample（采样）。然后若 error，立即返回 error。
    if(millis()-sample>100)return "IMU_STALE";  // 若 millis()-sample>100，立即返回 "IMU_STALE"。
    if(needsCalibration&&version==failedVersion)return "RECALIBRATE";  // 若 needsCalibration&&version==failedVersion，立即返回 "RECALIBRATE"。
    if(fabsf(continuous::turn(actual,staged.startHeading))>5)return "START_HEADING";  // 若 fabsf(continuous::turn(actual,staged.startHeading))>5，立即返回 "START_HEADING"。
    referenceVersion=version;  // 将 version 写入或初始化 referenceVersion（reference版本）。
    initialHeading=actual;  // 将 actual 写入或初始化 initialHeading（initial航向）。
    setAlignmentEnabled(false); // 等视觉任务停止，再发布运行态。
    consumed=true;  // 将 true 写入或初始化 consumed。
    portENTER_CRITICAL(&trialMux);  // 进入临界区，保护共享状态的本段访问。
    stopRequested=false;status.state=RUNNING;status.reason="NONE";  // 将 false 写入或初始化 stopRequested（停止Requested）。然后将 RUNNING 写入或初始化 status.state。然后将 "NONE" 写入或初始化 status.reason。
    status.s=0;status.x=nodes[0].p.x;status.y=nodes[0].p.y;status.heading=actual;  // 将 0 写入或初始化 status.s。然后将 nodes[0].p.x 写入或初始化 status.x。然后将 nodes[0].p.y 写入或初始化 status.y。然后将 actual 写入或初始化 status.heading。
    status.reference=staged.startHeading;status.error=continuous::turn(actual,staged.startHeading);  // 将 staged.startHeading 写入或初始化 status.reference。然后将 continuous::turn(actual,staged.startHeading) 写入或初始化 status.error。
    status.total=route.length;status.heartbeatMs=millis();  // 将 route.length 写入或初始化 status.total。然后将 millis() 写入或初始化 status.heartbeatMs。
    portEXIT_CRITICAL(&trialMux);return nullptr;  // 离开临界区，保护共享状态的本段访问。然后结束当前函数并返回 nullptr。
}  // 结束此函数或控制块。
/**
 * @brief 提交已准备的连续试跑作业；在串口ACK发出后调用。
 */
void ContinuousTrialLaunch(){  // 定义函数或成员 的入口 ContinuousTrialLaunch，参数由此处的签名确定。
    MotionRequest request;request.kind=MotionKind::TRIAL;  // 声明 request（请求），类型和可用范围由所在声明决定。然后将 MotionKind::TRIAL 写入或初始化 request.kind。
    if(!SubmitMotion(request)) {  // 当 !SubmitMotion(request) 成立时进入此分支，否则继续后续流程。
        portENTER_CRITICAL(&trialMux);status.state=FAILED;status.reason="BUSY";portEXIT_CRITICAL(&trialMux);  // 进入临界区，保护共享状态的本段访问。然后将 FAILED 写入或初始化 status.state。然后将 "BUSY" 写入或初始化 status.reason。然后离开临界区，保护共享状态的本段访问。
    }  // 结束此条件分支。
}  // 结束此函数或控制块。
/**
 * @brief 请求停止指定编号试跑，不在调用任务中直接操作电机；成功返回 nullptr。
 */
const char *ContinuousTrialStop(uint32_t id) {  // 定义函数或成员 的入口 ContinuousTrialStop（ContinuousTrial停止），参数由此处的签名确定。
    portENTER_CRITICAL(&trialMux);  // 进入临界区，保护共享状态的本段访问。
    const char *error=id!=status.id?"ID":nullptr;  // 将 id!=status.id?"ID":nullptr 写入或初始化 error（错误）。
    if(!error&&(status.state==RUNNING||status.state==STOPPING)){stopRequested=true;status.state=STOPPING;}  // 当 !error&&(status.state==RUNNING||status.state==STOPPING) 成立时进入此分支，否则继续后续流程。
    portEXIT_CRITICAL(&trialMux);return error;  // 离开临界区，保护共享状态的本段访问。然后结束当前函数并返回 error。
}  // 结束此函数或控制块。
/**
 * @brief 向当前试跑发出取消请求，供OTA或其他外部停止源调用。
 */
void ContinuousTrialCancel(){auto v=snapshot();ContinuousTrialStop(v.id);}  // 将 snapshot();ContinuousTrialStop(v.id);} 写入或初始化 v。
/**
 * @brief 设置试跑OTA状态，进入OTA时请求停止现有试跑。
 */
void ContinuousTrialSetOta(bool active){  // 定义函数或成员 的入口 ContinuousTrialSetOta（ContinuousTrialSet固件更新），参数由此处的签名确定。
    portENTER_CRITICAL(&trialMux);otaActive=active;  // 进入临界区，保护共享状态的本段访问。然后将 active 写入或初始化 otaActive（固件更新活跃）。
    if(active&&(status.state==RUNNING||status.state==STOPPING)){stopRequested=true;status.state=STOPPING;}  // 当 active&&(status.state==RUNNING||status.state==STOPPING) 成立时进入此分支，否则继续后续流程。
    portEXIT_CRITICAL(&trialMux);  // 离开临界区，保护共享状态的本段访问。
    MotionExecutorSetOta(active);  // 调用 MotionExecutorSetOta（MotionExecutorSet固件更新），传入 active。
}  // 结束此函数或控制块。
/**
 * @brief 输出指定试跑状态并续约1500ms心跳；成功返回 nullptr。
 */
const char *ContinuousTrialStatus(uint32_t id) {  // 定义函数或成员 的入口 ContinuousTrialStatus（ContinuousTrial状态），参数由此处的签名确定。
    portENTER_CRITICAL(&trialMux);  // 进入临界区，保护共享状态的本段访问。
    if(id!=status.id){portEXIT_CRITICAL(&trialMux);return "ID";}  // 当 id!=status.id 成立时进入此分支，否则继续后续流程。
    status.heartbeatMs=millis();auto v=status;  // 将 millis() 写入或初始化 status.heartbeatMs。然后将 status 写入或初始化 v。
    portEXIT_CRITICAL(&trialMux);  // 离开临界区，保护共享状态的本段访问。
    Serial.printf("{RSP,NAV,TRIAL_STATUS,OK,%lu,%s,%s,%.2f,%.2f,%.2f,%.2f,%.3f}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
        static_cast<unsigned long>(v.id),names[v.state],v.reason,v.s,v.total,v.x,v.y,v.heading);  // 声明接口 long>，参数由此处的签名确定。
    return nullptr;  // 结束当前函数并返回 nullptr。
}  // 结束此函数或控制块。

/**
 * @brief 在运动任务内构建正式路径并冻结本次速度/加速度配置；不等待行程完成。
 */
bool H30PrepareNodePath(const uint8_t *path,size_t count,float speedMmS,float finalHeading,bool async,  // 定义函数或成员 H30PrepareNodePath（H30Prepare节点路径），参数由此处的签名确定。
                       float startYOffset,float *plannedArrivalHeading,bool preserveStartPosition) {  // 声明 plannedArrivalHeading（plannedArrival航向），类型和可用范围由所在声明决定。
    ConfigurationGuard guard;if(!guard.held)return false;  // 声明 guard，类型和可用范围由所在声明决定。然后若 !guard.held，立即返回 false。
    if(!initialized||otaBusy()||taskMotionAborted)return false;  // 若 !initialized||otaBusy()||taskMotionAborted，立即返回 false。
    // 预留共享规划/任务资源；上传和其他运动不能在构建过程中插入。
    portENTER_CRITICAL(&trialMux);  // 进入临界区，保护共享状态的本段访问。
    if(status.state==RUNNING||status.state==STOPPING){portEXIT_CRITICAL(&trialMux);return false;}  // 当 status.state==RUNNING||status.state==STOPPING 成立时进入此分支，否则继续后续流程。
    status=Snapshot{};status.formal=true;status.async=async;status.state=STOPPING;  // 将 Snapshot{};status.formal=true;status.async=async;statu… 写入或初始化 status（状态）。
    stopRequested=false;  // 将 false 写入或初始化 stopRequested（停止Requested）。
    portEXIT_CRITICAL(&trialMux);  // 离开临界区，保护共享状态的本段访问。
    configured=false;  // 将 false 写入或初始化 configured。
    H30Attitude motion;float heading=0;uint32_t version=0;  // 声明 motion，类型和可用范围由所在声明决定。然后将 0 写入或初始化 heading（航向）。然后将 0 写入或初始化 version（版本）。
    const char *error=H30UsbGetMotionSnapshot(motion,heading,version);  // 将 H30UsbGetMotionSnapshot(motion,heading,version) 写入或初始化 error（错误）。
    if(!error&&millis()-motion.receivedAtMs>100)error="IMU_STALE";  // 当 !error&&millis()-motion.receivedAtMs>100 成立时进入此分支，否则继续后续流程。
    if(!error&&needsCalibration&&version==failedVersion)error="RECALIBRATE";  // 当 !error&&needsCalibration&&version==failedVersion 成立时进入此分支，否则继续后续流程。
    if(error) {  // 当 error 成立时进入此分支，否则继续后续流程。
        Serial.printf("[Route] H30 assist unavailable at start (%s); using planned heading\n",error);  // 调用 Serial.printf，传入 "[Route] H30 assist unavailable at start (%s); using p…。
        heading=currentPose.theta;version=0;error=nullptr;  // 将 currentPose.theta 写入或初始化 heading（航向）。然后将 0 写入或初始化 version（版本）。然后将 nullptr 写入或初始化 error（错误）。
    }  // 结束此条件分支。
    Config cfg;cfg.count=count;cfg.policy=2;cfg.startHeading=heading; // 正式导航统一到节点停车转向，不使用漂移切弯。
    cfg.maxRpm=node_position::rpmFor(speedMmS,Y_PULSE);  // 将 node_position::rpmFor(speedMmS,Y_PULSE) 写入或初始化 cfg.maxRpm。
    const uint32_t outerAccelLevel=NODE_PATH_OUTER_ACCEL_LEVEL;  // 将 NODE_PATH_OUTER_ACCEL_LEVEL 写入或初始化 outerAccelLevel。
    const uint32_t middleAccelLevel=NODE_PATH_MIDDLE_ACCEL_LEVEL;  // 将 NODE_PATH_MIDDLE_ACCEL_LEVEL 写入或初始化 middleAccelLevel。
    formalSettings.accelMmS2=node_position::motionAccelerationFor(outerAccelLevel,Y_PULSE);  // 将 node_position::motionAccelerationFor(outerAccelLevel,Y… 写入或初始化 formalSettings.accelMmS2。
    // 共用规划器保留原速度表范围；正式直行使用上面的实际换算值。
    cfg.accel=fminf(formalSettings.accelMmS2,4000.0f);  // 将 fminf(formalSettings.accelMmS2,4000.0f) 写入或初始化 cfg.accel。
    cfg.rotationPpd=THETA_PULSE;  // 将 THETA_PULSE 写入或初始化 cfg.rotationPpd。
    // 与正常扫码MovePosition一致：车身X用X_PULSE，车身Y用Y_PULSE。
    cfg.forwardPpm=Y_PULSE;cfg.lateralPpm=X_PULSE;  // 将 Y_PULSE 写入或初始化 cfg.forwardPpm。然后将 X_PULSE 写入或初始化 cfg.lateralPpm。
    formalSettings.speedMmS=speedMmS;  // 将 speedMmS 写入或初始化 formalSettings.speedMmS。
    formalSettings.middleSpeedMmS=NODE_PATH_MIDDLE_SPEED_MM_S;  // 将 NODE_PATH_MIDDLE_SPEED_MM_S 写入或初始化 formalSettings.middleSpeedMmS。
    formalSettings.middleAccelMmS2=node_position::motionAccelerationFor(middleAccelLevel,Y_PULSE);  // 将 node_position::motionAccelerationFor(middleAccelLevel,… 写入或初始化 formalSettings.middleAccelMmS2。
    formalSettings.outerStraightVelocity=true;  // 将 true 写入或初始化 formalSettings.outerStraightVelocity。
    formalSettings.recoverSmallResidual=true;  // 将 true 写入或初始化 formalSettings.recoverSmallResidual。
    formalSettings.turnDegS=NODE_PATH_TURN_DEG_S;  // 将 NODE_PATH_TURN_DEG_S 写入或初始化 formalSettings.turnDegS。
    formalSettings.turnAccelDegS2=NODE_PATH_TURN_ACCEL_DEG_S2;  // 将 NODE_PATH_TURN_ACCEL_DEG_S2 写入或初始化 formalSettings.turnAccelDegS2。
    formalSettings.headingDeadbandDeg=NODE_PATH_HEADING_DEADBAND_DEG;  // 将 NODE_PATH_HEADING_DEADBAND_DEG 写入或初始化 formalSettings.headingDeadbandDeg。
    if(count>=2&&path[count-1]==SCAN_AREA_NODE) {  // 当 count>=2&&path[count-1]==SCAN_AREA_NODE 成立时进入此分支，否则继续后续流程。
        cfg.slowApproachEdge=count>2?count-3:0; // 驶入扫码点的最后两段限速。
        for(size_t i=0;i<cfg.slowApproachEdge;++i) {  // 按 size_t i=0;i<cfg.slowApproachEdge;++i 组织循环的初值、继续条件和步进。
            if(path[i]==SCAN_AREA_NODE||path[i]==3||path[i]==7) {  // 当 path[i]==SCAN_AREA_NODE||path[i]==3||path[i]==7 成立时进入此分支，否则继续后续流程。
                cfg.slowApproachEdge=i;  // 将 i 写入或初始化 cfg.slowApproachEdge。
                break;  // 退出当前循环或 switch 分支。
            }  // 结束此条件分支。
        }  // 结束此循环体。
    }  // 结束此条件分支。
    float end=heading;  // 将 heading 写入或初始化 end（终点）。
    for(size_t i=0;i<count&&!error;++i) {  // 按 size_t i=0;i<count&&!error;++i 组织循环的初值、继续条件和步进。
        float x=0,y=0;  // 将 0,y=0 写入或初始化 x（横向）。
        if(!ChassisGetNodePosition(path[i],x,y))error="NODE";  // 当 !ChassisGetNodePosition(path[i],x,y) 成立时进入此分支，否则继续后续流程。
        else {nodes[i]=Node{};nodes[i].p={x,y};  // 前一个判断不成立时执行此分支。
            nodes[i].middle=path[i]==7||path[i]==11||path[i]==12||path[i]==13||path[i]==17;}  // 将 path[i]==7||path[i]==11||path[i]==12||path[i]==13||pat… 写入或初始化 middle。
        if(i&&!adjacent(path[i-1],path[i]))error="NOT_ADJACENT";  // 当 i&&!adjacent(path[i-1],path[i]) 成立时进入此分支，否则继续后续流程。
        if(i&&!error) {  // 当 i&&!error 成立时进入此分支，否则继续后续流程。
            Vec d=nodes[i].p-nodes[i-1].p;  // 将 nodes[i].p-nodes[i-1].p 写入或初始化 d。
            float forward=wrap(atan2f(d.y,d.x)*180/PI_F),back=wrap(forward+180);  // 将 wrap(atan2f(d.y,d.x)*180/PI_F),back=wrap(forward+180) 写入或初始化 forward（前向）。
            end=fabsf(turn(end,back))<fabsf(turn(end,forward))-.01f?back:forward;  // 将 fabsf(turn(end,back))<fabsf(turn(end,forward))-.01f?ba… 写入或初始化 end（终点）。
        }  // 结束此条件分支。
    }  // 结束此循环体。
    cfg.endHeading=isnan(finalHeading)?end:wrap(finalHeading);  // 将 isnan(finalHeading)?end:wrap(finalHeading) 写入或初始化 cfg.endHeading。
    if(!isfinite(cfg.endHeading))error="RANGE";  // 当 !isfinite(cfg.endHeading) 成立时进入此分支，否则继续后续流程。
    if(!error)error=route.build(nodes,cfg);  // 当 !error 成立时进入此分支，否则继续后续流程。
    if(error) {  // 当 error 成立时进入此分支，否则继续后续流程。
        Serial.printf("[Route] H30 start rejected: %s\n",error);  // 调用 Serial.printf，传入 "[Route] H30 start rejected: %s\n",error。
        portENTER_CRITICAL(&trialMux);status.state=FAILED;status.reason=error;portEXIT_CRITICAL(&trialMux);  // 进入临界区，保护共享状态的本段访问。然后将 FAILED 写入或初始化 status.state。然后将 error 写入或初始化 status.reason。然后离开临界区，保护共享状态的本段访问。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。
    configured=false;referenceVersion=version;initialHeading=heading;  // 将 false 写入或初始化 configured。然后将 version 写入或初始化 referenceVersion（reference版本）。然后将 heading 写入或初始化 initialHeading（initial航向）。
    Serial.printf("[Route] Transition window: disabled; separate heading correction deadband: %.2f deg\n",formalSettings.headingDeadbandDeg);  // 调用 Serial.printf，传入 "[Route] Transition window: disabled; separate heading…。
    Serial.printf("[Route] Profile: %.1f mm/s, %.1f mm/s2 (EMM V5 %lu), %.1f deg/s, %.1f deg/s2\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
        formalSettings.speedMmS,formalSettings.accelMmS2,static_cast<unsigned long>(outerAccelLevel),formalSettings.turnDegS,formalSettings.turnAccelDegS2);  // 声明接口 long>，参数由此处的签名确定。
    Serial.printf("[Route] Middle profile (7/11/12/13/17): %.1f mm/s, %.1f mm/s2 (EMM V5 %lu)\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
        formalSettings.middleSpeedMmS,formalSettings.middleAccelMmS2,static_cast<unsigned long>(middleAccelLevel));  // 声明接口 long>，参数由此处的签名确定。
    formalSettings.startOffset={};  // 将 {} 写入或初始化 formalSettings.startOffset。
    formalSettings.alignStartToRoad=preserveStartPosition;  // 将 preserveStartPosition 写入或初始化 formalSettings.alignStartToRoad。
    formalSettings.startYOffset=startYOffset;  // 将 startYOffset 写入或初始化 formalSettings.startYOffset。
    if(plannedArrivalHeading)*plannedArrivalHeading=cfg.endHeading;  // 当 plannedArrivalHeading 成立时进入此分支，否则继续后续流程。
    const Vec start=nodes[0].p+Vec{0,startYOffset};  // 将 nodes[0].p+Vec{0,startYOffset} 写入或初始化 start（起点）。
    if(preserveStartPosition) {  // 当 preserveStartPosition 成立时进入此分支，否则继续后续流程。
        formalSettings.startOffset=Vec{currentPose.x,currentPose.y}-start;  // 将 Vec{currentPose.x,currentPose.y}-start 写入或初始化 formalSettings.startOffset。
        Serial.printf("[Route] Start offset=(%.1f, %.1f) mm\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
            formalSettings.startOffset.x,formalSettings.startOffset.y);  // 续写当前声明或运算，提供 formalSettings.startOffset.x,formalSettings.startOffse…。
    } else {  // 前一个判断不成立时执行此分支。
        currentPose.x=start.x;currentPose.y=start.y;  // 将 start.x 写入或初始化 currentPose.x。然后将 start.y 写入或初始化 currentPose.y。
    }  // 结束此条件分支。
    setAlignmentEnabled(false);  // 调用 setAlignmentEnabled（setAlignment已启用），传入 false。
    portENTER_CRITICAL(&trialMux);  // 进入临界区，保护共享状态的本段访问。
    status.total=route.length;status.state=RUNNING;status.heartbeatMs=millis();  // 将 route.length 写入或初始化 status.total。然后将 RUNNING 写入或初始化 status.state。然后将 millis() 写入或初始化 status.heartbeatMs。
    status.x=currentPose.x;status.y=currentPose.y;status.heading=heading;  // 将 currentPose.x 写入或初始化 status.x。然后将 currentPose.y 写入或初始化 status.y。然后将 heading 写入或初始化 status.heading。
    portEXIT_CRITICAL(&trialMux);  // 离开临界区，保护共享状态的本段访问。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。
/**
 * @brief 提交正式节点导航作业；async 控制异步返回或同步等待完成。
 * @param path 节点编号数组。
 * @param count 节点数。
 * @param speedMmS 路径平移速度(mm/s)。
 * @param finalHeading 终点地图航向(deg)。
 * @param async true 异步提交，false 等待作业结果。
 * @return true 提交或执行成功，具体含义由 async 决定。
 */
bool H30StartNodePath(const uint8_t *path,size_t count,float speedMmS,float finalHeading,bool async) {  // 定义函数或成员 的入口 H30StartNodePath（H30起点节点路径），参数由此处的签名确定。
    MotionRequest r;r.kind=MotionKind::NODE_PATH;r.pathCount=count;r.speed=speedMmS;r.theta=finalHeading;  // 声明 r，类型和可用范围由所在声明决定。然后将 MotionKind::NODE_PATH 写入或初始化 r.kind。然后将 count 写入或初始化 r.pathCount。然后将 speedMmS 写入或初始化 r.speed。然后将 finalHeading 写入或初始化 r.theta。
    r.automatic=async;memcpy(r.path,path,count);return SubmitMotion(r)!=0;  // 将 async 写入或初始化 r.automatic。然后调用 memcpy，传入 r.path,path,count。然后结束当前函数并返回 SubmitMotion(r)!=0。
}  // 结束此函数或控制块。
/**
 * @brief 同步执行正式节点导航，参数为节点数组、节点数、平移速度及终点航向。
 */
bool H30RunNodePath(const uint8_t *path,size_t count,float speedMmS,float finalHeading) {  // 定义函数或成员 的入口 H30RunNodePath（H30Run节点路径），参数由此处的签名确定。
    MotionRequest r;r.kind=MotionKind::NODE_PATH;r.pathCount=count;r.speed=speedMmS;r.theta=finalHeading;  // 声明 r，类型和可用范围由所在声明决定。然后将 MotionKind::NODE_PATH 写入或初始化 r.kind。然后将 count 写入或初始化 r.pathCount。然后将 speedMmS 写入或初始化 r.speed。然后将 finalHeading 写入或初始化 r.theta。
    memcpy(r.path,path,count);return WaitMotion(SubmitMotion(r));  // 调用 memcpy，传入 r.path,path,count。然后结束当前函数并返回 WaitMotion(SubmitMotion(r))。
}  // 结束此函数或控制块。
/**
 * @brief 启动已经准备好的节点导航或连续试跑执行过程，仅由运动任务调用。
 */
void ContinuousMotionBegin() {  // 定义函数或成员 的入口 ContinuousMotionBegin，参数由此处的签名确定。
    execution=Execution{};auto &e=execution;e.job=snapshot();e.active=true;e.actual=initialHeading;  // 将 Execution{};auto &e=execution;e.job=snapshot();e.activ… 写入或初始化 execution。
    hardStop();  // 调用 hardStop（hard停止），使用无参形式推进相应操作。
    if(!e.job.formal)e.failure=headingSample(e.actual,e.sampleMs);  // 当 !e.job.formal 成立时进入此分支，否则继续后续流程。
    e.tracker.begin(route,e.actual);e.tracker.requireInertial=true;e.tracker.commit(WheelCommand{},millis());  // 调用 e.tracker.begin，传入 route,e.actual);e.tracker.requireInertial=true;e.track…。
    e.tracker.estimated={currentPose.x,currentPose.y};  // 将 {currentPose.x,currentPose.y} 写入或初始化 e.tracker.estimated。
    if(!e.failure&&!e.job.formal&&fabsf(turn(e.actual,route.config.startHeading))>5)e.failure="START_HEADING";  // 当 !e.failure&&!e.job.formal&&fabsf(turn(e.actual,route.c… 成立时进入此分支，否则继续后续流程。
    e.started=e.previousCycle=e.integratedAt=millis();e.due=e.started;  // 将 e.previousCycle=e.integratedAt=millis() 写入或初始化 e.started。然后将 e.started 写入或初始化 e.due。
    e.timeout=static_cast<uint32_t>((route.plannedSeconds*2+5)*1000);  // 将 static_cast<uint32_t>((route.plannedSeconds*2+5)*1000) 写入或初始化 e.timeout。
    if(e.job.formal&&!e.failure) {  // 当 e.job.formal&&!e.failure 成立时进入此分支，否则继续后续流程。
        e.position.estimated=e.tracker.estimated;e.position.heading=e.actual;  // 将 e.tracker.estimated 写入或初始化 e.position.estimated。然后将 e.actual 写入或初始化 e.position.heading。
        BeginPositionNodeRoute(route,formalSettings,referenceVersion,stopped,e.position);  // 调用 BeginPositionNodeRoute（Begin位置节点路线），传入 route,formalSettings,referenceVersion,stopped,e.positi…。
    }  // 结束此条件分支。
    if(e.failure)finishExecution();  // 当 e.failure 成立时进入此分支，否则继续后续流程。
}  // 结束此函数或控制块。
/**
 * @brief 推进一次正式导航或试跑；实际节点运动由 node_position::Runner 执行。
 */
void ContinuousMotionTick() {  // 定义函数或成员 的入口 ContinuousMotionTick（ContinuousMotion轮询），参数由此处的签名确定。
    auto &e=execution;if(!e.active)return;  // 将 execution 写入或初始化 e。然后当 !e.active 成立时进入此分支，否则继续后续流程。
    if(stopped()||taskMotionAborted){e.failure=stopped()?"STOPPED":"ABORTED";finishExecution();return;}  // 当 stopped()||taskMotionAborted 成立时进入此分支，否则继续后续流程。
    if(e.job.formal) {  // 当 e.job.formal 成立时进入此分支，否则继续后续流程。
        TickPositionNodeRoute();  // 调用 TickPositionNodeRoute（轮询位置节点路线），使用无参形式推进相应操作。
        portENTER_CRITICAL(&trialMux);status.s=e.position.s;status.x=e.position.estimated.x;status.y=e.position.estimated.y;status.heading=e.position.heading;portEXIT_CRITICAL(&trialMux);  // 进入临界区，保护共享状态的本段访问。然后将 e.position.s 写入或初始化 status.s。然后将 e.position.estimated.x 写入或初始化 status.x。然后将 e.position.estimated.y 写入或初始化 status.y。然后将 e.position.heading 写入或初始化 status.heading。然后离开临界区，保护共享状态的本段访问。
        if(!PositionNodeRunner().active()){e.failure=PositionNodeRunner().failure();e.success=!e.failure;finishExecution();}  // 当 !PositionNodeRunner().active() 成立时进入此分支，否则继续后续流程。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    const uint32_t now=millis();if(static_cast<int32_t>(now-e.due)<0)return;  // 将 millis() 写入或初始化 now。然后当 static_cast<int32_t>(now-e.due)<0 成立时进入此分支，否则继续后续流程。
    if(now-snapshot().heartbeatMs>1500)e.failure="LINK_TIMEOUT";  // 当 now-snapshot().heartbeatMs>1500 成立时进入此分支，否则继续后续流程。
    else if(now-e.started>e.timeout)e.failure="TIMEOUT";  // 仅在先前条件均不成立时，当 now-e.started>e.timeout 成立时进入此分支，否则继续后续流程。
    H30Attitude motion;if(!e.failure)e.failure=headingSample(e.actual,e.sampleMs,&motion);  // 声明 motion，类型和可用范围由所在声明决定。然后当 !e.failure 成立时进入此分支，否则继续后续流程。
    if(e.failure){finishExecution();return;}  // 当 e.failure 成立时进入此分支，否则继续后续流程。
    e.tracker.observeMotion(motion);  // 调用 e.tracker.observeMotion，传入 motion。
    const float dt=now==e.started?.04f:(now-e.previousCycle)*.001f;e.previousCycle=now;  // 将 now==e.started?.04f:(now-e.previousCycle)*.001f 写入或初始化 dt（时间差）。然后将 now 写入或初始化 e.previousCycle。
    if(dt>.120f||dt<=0){e.failure="CONTROL_OVERRUN";finishExecution();return;}  // 当 dt>.120f||dt<=0 成立时进入此分支，否则继续后续流程。
    e.tracker.integrate(route,e.actual,(now-e.integratedAt)*.001f);e.integratedAt=now;  // 调用 e.tracker.integrate，传入 route,e.actual,(now-e.integratedAt)*.001f。然后将 now 写入或初始化 e.integratedAt。
    e.failure=field::check(e.tracker.estimated.x,e.tracker.estimated.y,e.actual,0);  // 将 field::check(e.tracker.estimated.x,e.tracker.estimated… 写入或初始化 e.failure。
    if(e.failure){finishExecution();return;}  // 当 e.failure 成立时进入此分支，否则继续后续流程。
    Output out=e.tracker.update(route,e.actual,dt,e.sampleMs);  // 将 e.tracker.update(route,e.actual,dt,e.sampleMs) 写入或初始化 out。
    if(out.failure){e.failure=out.failure;finishExecution();return;}  // 当 out.failure 成立时进入此分支，否则继续后续流程。
    if(out.finished){e.success=true;finishExecution();return;}  // 当 out.finished 成立时进入此分支，否则继续后续流程。
    e.failure=sendWheels(out.wheels,now);if(e.failure){finishExecution();return;}  // 将 sendWheels(out.wheels,now);if(e.failure){finishExecuti… 写入或初始化 e.failure。
    const uint32_t synced=millis();e.tracker.integrate(route,e.actual,(synced-e.integratedAt)*.001f);e.integratedAt=synced;  // 将 millis() 写入或初始化 synced。然后调用 e.tracker.integrate，传入 route,e.actual,(synced-e.integratedAt)*.001f。然后将 synced 写入或初始化 e.integratedAt。
    e.tracker.commit(out.wheels,synced);  // 调用 e.tracker.commit，传入 out.wheels,synced。
    portENTER_CRITICAL(&trialMux);  // 进入临界区，保护共享状态的本段访问。
    status.s=e.tracker.s;status.x=e.tracker.estimated.x;status.y=e.tracker.estimated.y;  // 将 e.tracker.s 写入或初始化 status.s。然后将 e.tracker.estimated.x 写入或初始化 status.x。然后将 e.tracker.estimated.y 写入或初始化 status.y。
    status.reference=out.reference;status.heading=e.actual;status.error=out.error;status.cycleMs=dt*1000;  // 将 out.reference 写入或初始化 status.reference。然后将 e.actual 写入或初始化 status.heading。然后将 out.error 写入或初始化 status.error。然后将 dt*1000 写入或初始化 status.cycleMs。
    for(int i=0;i<4;++i)status.rpm[i]=out.wheels.rpm[i];  // 按 int i=0;i<4;++i)status.rpm[i]=out.wheels.rpm[i]; 组织循环的初值、继续条件和步进。
    portEXIT_CRITICAL(&trialMux);  // 离开临界区，保护共享状态的本段访问。
    if(now-e.lastTelemetry>=200){telemetry(snapshot());  // 当 now-e.lastTelemetry>=200 成立时进入此分支，否则继续后续流程。
        Serial.printf("{EVT,NAV,TRIAL_PHASE,%lu,%s}\n",static_cast<unsigned long>(e.job.id),e.tracker.stage);e.lastTelemetry=now;}  // 将 now;} 写入或初始化 e.lastTelemetry。
    e.due=now+40;  // 将 now+40 写入或初始化 e.due。
}  // 结束此函数或控制块。
/**
 * @brief 取消当前路径执行并结束其运动状态。
 */
void ContinuousMotionCancel(){if(execution.active){execution.failure="STOPPED";execution.success=false;finishExecution();}}  // 将 "STOPPED";execution.success=false;finishExecution();}} 写入或初始化 execution.failure。
/**
 * @brief 判断正式导航或连续试跑执行过程是否仍在运行。
 */
bool ContinuousMotionActive(){return execution.active;}  // 定义 ContinuousMotionActive（ContinuousMotion活跃），调用时返回 execution.active。
/**
 * @brief 读取当前路径执行错误原因；nullptr 表示未记录错误。
 */
const char *ContinuousMotionFailure(){return execution.failure;}  // 定义函数或成员 ContinuousMotionFailure，参数由此处的签名确定。
/**
 * @brief 查询路径控制器下一次推进前所需等待毫秒数。
 */
uint32_t ContinuousMotionNextDelay(){  // 定义函数或成员 的入口 ContinuousMotionNextDelay（ContinuousMotion下一延时），参数由此处的签名确定。
    if(execution.job.formal)return PositionNodeRunner().active()?PositionNodeRunner().nextDelay():0;  // 若 execution.job.formal，立即返回 PositionNodeRunner().active()?PositionNodeRunner().nex…。
    const int32_t left=static_cast<int32_t>(execution.due-millis());return left>0?left:0;  // 将 static_cast<int32_t>(execution.due-millis()) 写入或初始化 left（左）。然后结束当前函数并返回 left>0?left:0。
}  // 结束此函数或控制块。
