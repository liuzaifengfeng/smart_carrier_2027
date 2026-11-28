// 运行期共享业务状态及视觉会话控制。
// 串口入口在文件末尾文本包含，沿用本文件内的状态和函数；视觉启停与反馈处理应结合阅读。

#include "turret_motor.h"  // 引入 turret_motor.h 中的声明和定义。
#include "robot_runtime.h"  // 引入 robot_runtime.h 中的声明和定义。
#include "alignment_start_policy.h"  // 引入 alignment_start_policy.h 中的声明和定义。
#include "h30_usb.h"  // 引入 h30_usb.h 中的声明和定义。
#include "motion_executor.h"  // 引入 motion_executor.h 中的声明和定义。
#include "led_pwm.h"  // 引入 led_pwm.h 中的声明和定义。
#include "battery_monitor.h"  // 引入 battery_monitor.h 中的声明和定义。

// ================= 基础配置 =================

uint32_t ALIGN_PID_MAX_SPEED_RPM = 50;       // 视觉PID对齐轮速上限，单位RPM；原20，提高到2.5倍。
float NODE_PATH_SPEED_MM_S = 2500.0f;        // 正式节点路径的最大平移速度，单位mm/s。
uint32_t NODE_PATH_OUTER_ACCEL_LEVEL = 200; // 正式节点路径外圈平移的 EMM V5 加减速档位。
float NODE_PATH_MIDDLE_SPEED_MM_S = 3000.0f; // 经过7、11、12、13、17节点的平移速度上限。
uint32_t NODE_PATH_MIDDLE_ACCEL_LEVEL = 175; // 中间道路平移的 EMM V5 加减速档位。
float NODE_PATH_TURN_DEG_S = 300.0f;         // 节点路径原地转向的最大角速度，单位°/s。
float NODE_PATH_TURN_ACCEL_DEG_S2 = 400.0f;  // 节点路径原地转向的角加减速度，单位°/s²，并换算为驱动器加速度档位。
float NODE_PATH_HEADING_DEADBAND_DEG = 1.0f; // 停稳后独立航向修正的允许误差，单位°；误差在此范围内不再转动。
uint32_t SCAN_APPROACH_SPEED_RPM = 1500;        // 启停区接近扫码区域时的位置模式最大轮速，单位RPM。
uint32_t SCAN_SEARCH_SPEED_RPM = 1000;          // 扫码区域往返搜索任务码时的位置模式最大轮速，单位RPM。

CRGB leds[NUM_LEDS];  // LED 像素数组(板载 WS2812B)

// 运动标定系数
float X_PULSE     = 13.3f;    // X向 每毫米脉冲
float Y_PULSE     = 13.6f;    // Y向 每毫米脉冲
float THETA_PULSE = 49.353889f;    // 旋转 每度脉冲
float HEIGHT_PULSE = 80.0f;  // 升降机械臂 每毫米脉冲
float LENGTH_PULSE = 30.2f;  // 伸缩机械臂 每毫米脉冲

// ================= 任务码 =================
// 2027赛制任务码格式: 四组三位数 "R1+ P1+ R2+ P2"
//   第一组: 第一批3个物料颜色顺序(红1黄2蓝3绿4黑5浅蓝6)
//   第二组: 第一批在粗加工区/暂存区的放置位置(1-3)
//   第三组: 第二批3个物料颜色顺序
//   第四组: 第二批在粗加工区的放置位置
// 例: "156+123+516+231"

TaskCode currentTask = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, false };  // 将 { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, false } 写入或初始化 currentTask（当前任务）。
portMUX_TYPE taskCodeMux = portMUX_INITIALIZER_UNLOCKED;  // 将 portMUX_INITIALIZER_UNLOCKED 写入或初始化 taskCodeMux（任务Code临界区锁）。

// ================= 机载电脑通信(串口)协议 =================
// 机器帧使用 {CMD/RSP/EVT,类别,动作,...}\n，详见 Document/serial_protocol.md。

// ================= 业务状态 =================
volatile RobotState currentState = STATE_WAIT_START;  // 将 STATE_WAIT_START 写入或初始化 currentState（当前状态）。

// 共享业务变量(由机载电脑指令/任务更新)
volatile bool nano_ready = false;       // 机载电脑就绪
volatile StartZone currentStartZone = START_ZONE_UNKNOWN; // 当前启停区,由机载电脑告知
volatile bool taskReceived = false;// 已拿到任务码
volatile int  roundProgress = 0;   // 当前轮次已抓/放物料数 0-3
volatile bool discMaterialActive = false;  // 将 false 写入或初始化 discMaterialActive（圆盘物料活跃）。
volatile bool firstDiscGrabReady = false; // 当前轮导航、定位、放料和观察恢复完成后才允许下一次圆盘抓取。
volatile bool discMessageResumeRequested = false;  // 将 false 写入或初始化 discMessageResumeRequested（圆盘MessageResumeRequested）。
uint8_t discMaterialColor = 0; // 最近一次 COLOR；每次启停或抓取后失效。
volatile bool enableRun = false;   // 一键启动触发

// 两个任务通过临界区交接路径，避免串口在执行中覆盖节点数组。
NodePathState nodePathState = NodePathState::IDLE;  // 将 NodePathState::IDLE 写入或初始化 nodePathState（节点路径状态）。
portMUX_TYPE nodePathMux = portMUX_INITIALIZER_UNLOCKED;  // 将 portMUX_INITIALIZER_UNLOCKED 写入或初始化 nodePathMux（节点路径临界区锁）。
uint8_t nodePathBuffer[MAX_NODE_PATH_LENGTH] = {0};  // 将 {0} 写入或初始化 nodePathBuffer[MAX_NODE_PATH_LENGTH]。
size_t nodePathLen = 0;  // 将 0 写入或初始化 nodePathLen（节点路径Len）。
uint32_t nodePathRequestedAt = 0;  // 将 0 写入或初始化 nodePathRequestedAt（节点路径RequestedAt）。

QueueHandle_t xAlignmentQueue = NULL;  // 将 NULL 写入或初始化 xAlignmentQueue（横向AlignmentQueue）。
SemaphoreHandle_t xAlignmentMotionMutex = NULL;  // 将 NULL 写入或初始化 xAlignmentMotionMutex（横向AlignmentMotionMutex）。
uint32_t alignmentFrameSequence = 0; // 受运动锁保护，用于发现最新帧队列覆盖造成的丢帧。
VisionStartMode alignmentOwner = VisionStartMode::NONE; // 受运动锁保护。
volatile bool alignmentEnabled = false;  // 将 false 写入或初始化 alignmentEnabled（alignment已启用）。
bool alignmentStopPending=false;  // 将 false 写入或初始化 alignmentStopPending（alignment停止Pending）。
AutoAlignmentState autoAlignmentState = AutoAlignmentState::IDLE;  // 将 AutoAlignmentState::IDLE 写入或初始化 autoAlignmentState（autoAlignment状态）。
static bool cornerXYInverted=false; // 与对齐模式共用运动锁。
uint32_t autoAlignmentStartedMs = 0;  // 将 0 写入或初始化 autoAlignmentStartedMs（autoAlignmentStarted毫秒）。
uint32_t alignmentGeneration = 0; // 受运动锁保护，丢弃跨启停周期的已出队帧。

namespace {  // 开始匿名命名空间，本文件内部共享其中的定义。
/** @brief 对齐或圆盘识别任一开启时补光10%，全部关闭时关灯。 */
void updateVisionLight() {  // 定义函数或成员 的入口 updateVisionLight（update视觉Light），参数由此处的签名确定。
    LedPwm_SetBrightness(alignmentEnabled||discMaterialActive?10:0);  // 调用 LedPwm_SetBrightness（Led占空比输出SetBrightness），传入 alignmentEnabled||discMaterialActive?10:0。
}  // 结束此函数或控制块。
const char *visionNames[]={"DISC","DISC_MATERIAL","WORK_AREA","WORK_AREA_LOADED","CORNER"};  // 声明 visionNames（视觉Names），类型和可用范围由所在声明决定。
bool visionSpanActive[5]={},visionFirstFrame[5]={};  // 将 {},visionFirstFrame[5]={} 写入或初始化 visionSpanActive[5]。
portMUX_TYPE visionTraceMux=portMUX_INITIALIZER_UNLOCKED;  // 将 portMUX_INITIALIZER_UNLOCKED 写入或初始化 visionTraceMux（视觉Trace临界区锁）。
void traceVision(VisionStartMode mode,bool start,bool reportMissingFrame=true) {  // 定义函数或成员 的入口 traceVision（trace视觉），参数由此处的签名确定。
    if(mode==VisionStartMode::NONE)mode=VisionStartMode::DISC;  // 当 mode==VisionStartMode::NONE 成立时进入此分支，否则继续后续流程。
    const uint8_t index=static_cast<uint8_t>(mode);  // 将 static_cast<uint8_t>(mode) 写入或初始化 index（下标）。
    portENTER_CRITICAL(&visionTraceMux);  // 进入临界区，保护共享状态的本段访问。
    const bool wasActive=visionSpanActive[index],wasFirst=visionFirstFrame[index];  // 将 visionSpanActive[index],wasFirst=visionFirstFrame[inde… 写入或初始化 wasActive（was活跃）。
    visionSpanActive[index]=start;visionFirstFrame[index]=start;  // 将 start 写入或初始化 visionSpanActive[index]。然后将 start 写入或初始化 visionFirstFrame[index]。
    portEXIT_CRITICAL(&visionTraceMux);  // 离开临界区，保护共享状态的本段访问。
    // 未收到首帧便结束会话时不伪造首帧耗时，统计脚本保留该未完成区间。
    if(wasFirst&&reportMissingFrame)Serial.printf("{EVT,MOTION,ERROR,0,%lu,VISION_NO_FRAME_%s}\n",static_cast<unsigned long>(millis()),visionNames[index]);  // 当 wasFirst&&reportMissingFrame 成立时进入此分支，否则继续后续流程。
    if(wasActive)Serial.printf("{EVT,MOTION,SPAN,0,%lu,VISION_WAIT_%s,END}\n",static_cast<unsigned long>(millis()),visionNames[index]);  // 当 wasActive 成立时进入此分支，否则继续后续流程。
    if(start) {  // 当 start 成立时进入此分支，否则继续后续流程。
        Serial.printf("{EVT,MOTION,SPAN,0,%lu,VISION_WAIT_%s,BEGIN}\n",static_cast<unsigned long>(millis()),visionNames[index]);  // 调用 Serial.printf，传入 "{EVT,MOTION,SPAN,0,%lu,VISION_WAIT_%s,BEGIN}\n",stati…。
        Serial.printf("{EVT,MOTION,SPAN,0,%lu,VISION_FIRST_FRAME_%s,BEGIN}\n",static_cast<unsigned long>(millis()),visionNames[index]);  // 调用 Serial.printf，传入 "{EVT,MOTION,SPAN,0,%lu,VISION_FIRST_FRAME_%s,BEGIN}\n…。
    }  // 结束此条件分支。
}  // 结束此初始化列表或局部作用域。
void traceVisionFirstFrame(VisionStartMode mode) {  // 定义函数或成员 的入口 traceVisionFirstFrame（trace视觉首个帧），参数由此处的签名确定。
    if(mode==VisionStartMode::NONE)mode=VisionStartMode::DISC;  // 当 mode==VisionStartMode::NONE 成立时进入此分支，否则继续后续流程。
    const uint8_t index=static_cast<uint8_t>(mode);  // 将 static_cast<uint8_t>(mode) 写入或初始化 index（下标）。
    portENTER_CRITICAL(&visionTraceMux);const bool pending=visionFirstFrame[index];visionFirstFrame[index]=false;portEXIT_CRITICAL(&visionTraceMux);  // 进入临界区，保护共享状态的本段访问。然后将 visionFirstFrame[index] 写入或初始化 pending。然后将 false 写入或初始化 visionFirstFrame[index]。然后离开临界区，保护共享状态的本段访问。
    if(pending)Serial.printf("{EVT,MOTION,SPAN,0,%lu,VISION_FIRST_FRAME_%s,END}\n",static_cast<unsigned long>(millis()),visionNames[index]);  // 当 pending 成立时进入此分支，否则继续后续流程。
}  // 结束此函数或控制块。
}  // 结束命名空间。

/**
 * @brief 读取自动对齐状态，供主流程决定是否进入取放料阶段。
 */
AutoAlignmentState getAutoAlignmentState() {  // 定义函数或成员 的入口 getAutoAlignmentState（getAutoAlignment状态），参数由此处的签名确定。
    if (!xAlignmentMotionMutex) return AutoAlignmentState::FAILED;  // 若 !xAlignmentMotionMutex，立即返回 AutoAlignmentState::FAILED。
    xSemaphoreTake(xAlignmentMotionMutex, portMAX_DELAY);  // 调用 xSemaphoreTake（横向SemaphoreTake），传入 xAlignmentMotionMutex, portMAX_DELAY。
    const auto state = autoAlignmentState;  // 将 autoAlignmentState 写入或初始化 state（状态）。
    xSemaphoreGive(xAlignmentMotionMutex);  // 调用 xSemaphoreGive（横向SemaphoreGive），传入 xAlignmentMotionMutex。
    return state;  // 结束当前函数并返回 state。
}  // 结束此函数或控制块。

// 参数提交与 PID 更新共用锁，避免一个控制周期读取到半套参数。
/**
 * @brief 开启或关闭底盘视觉对齐，并维护当前模式、完成判定和显示应答。
 * @param enable true 开启，false 关闭。
 * @param acknowledge 是否输出控制应答。
 * @param automatic 是否属于自动比赛对齐。
 * @param owner 拥有该次对齐的视觉模式。
 * @param onlyOwner 是否仅允许匹配拥有者关闭。
 * @param invertCornerXY 正式返家启停区2时反转角点视觉XY输入。
 */
void setAlignmentEnabled(bool enable, bool acknowledge, bool automatic, VisionStartMode owner, bool onlyOwner,  // 定义函数或成员 setAlignmentEnabled（setAlignment已启用），参数由此处的签名确定。
    bool invertCornerXY) {  // 续写当前声明或运算，提供 bool invertCornerXY) {。
    if (xAlignmentMotionMutex != NULL  // 当 xAlignmentMotionMutex != NULL 成立时进入此分支，否则继续后续流程。
            && xSemaphoreTake(xAlignmentMotionMutex, portMAX_DELAY) == pdTRUE) {  // 继续上一行的组合条件，本行提供 && xSemaphoreTake(xAlignmentMotionMutex, portMAX_DELAY) == pdTRUE) {。
        // 结束旧定位功能不能停止随后开启的另一种定位或手动连续对齐。
        if (onlyOwner && alignmentOwner != owner) {  // 当 onlyOwner && alignmentOwner != owner 成立时进入此分支，否则继续后续流程。
            xSemaphoreGive(xAlignmentMotionMutex);  // 调用 xSemaphoreGive（横向SemaphoreGive），传入 xAlignmentMotionMutex。
            return;  // 结束当前函数，将执行权交回调用者。
        }  // 结束此条件分支。
        if (enable && !automatic) {  // 当 enable && !automatic 成立时进入此分支，否则继续后续流程。
            // 在同一运动锁内判定，保护 WAITING/DONE 以及尚未被主任务处理的 FAILED。
            const auto action = DecideAlignmentStart(  // 将 DecideAlignmentStart( 写入或初始化 action。
                autoAlignmentState != AutoAlignmentState::IDLE,  // 为当前列表或跨行表达式提供这一项 autoAlignmentState != AutoAlignmentState::IDLE。
                autoAlignmentState == AutoAlignmentState::FAILED,  // 为当前列表或跨行表达式提供这一项 autoAlignmentState == AutoAlignmentState::FAILED。
                alignmentOwner, owner);  // 续写当前声明或运算，提供 alignmentOwner, owner);。
            if (action != AlignmentStartAction::RESET) {  // 当 action != AlignmentStartAction::RESET 成立时进入此分支，否则继续后续流程。
                xSemaphoreGive(xAlignmentMotionMutex);  // 调用 xSemaphoreGive（横向SemaphoreGive），传入 xAlignmentMotionMutex。
                if (acknowledge) Serial.println(action == AlignmentStartAction::KEEP  // 当 acknowledge 成立时进入此分支，否则继续后续流程。
                    ? "{RSP,VISION,ALIGN_START,OK}"  // 续写上一行表达式的条件取值分支。
                    : "{RSP,VISION,ALIGN_START,ERR,BUSY}");  // 续写上一行表达式的条件取值分支。
                // 不清空反馈，不重置 PID、代次、连续帧计数、完成状态或超时起点。
                return;  // 结束当前函数，将执行权交回调用者。
            }  // 结束此条件分支。
        }  // 结束此条件分支。
        if(alignmentEnabled)traceVision(alignmentOwner,false);  // 当 alignmentEnabled 成立时进入此分支，否则继续后续流程。
        alignmentStopPending=alignmentStopPending||alignmentEnabled||enable;  // 将 alignmentStopPending||alignmentEnabled||enable 写入或初始化 alignmentStopPending（alignment停止Pending）。
        alignmentEnabled = false;  // 将 false 写入或初始化 alignmentEnabled（alignment已启用）。
        alignmentOwner = enable ? owner : VisionStartMode::NONE;  // 将 enable ? owner : VisionStartMode::NONE 写入或初始化 alignmentOwner。
        cornerXYInverted=enable&&owner==VisionStartMode::CORNER&&invertCornerXY;  // 将 enable&&owner==VisionStartMode::CORNER&&invertCornerXY 写入或初始化 cornerXYInverted。
        ++alignmentGeneration;  // 续写当前声明或运算，提供 ++alignmentGeneration;。
        if (xAlignmentQueue != NULL) xQueueReset(xAlignmentQueue);  // 当 xAlignmentQueue != NULL 成立时进入此分支，否则继续后续流程。
        ResetDiscAlignmentPid();  // 调用 ResetDiscAlignmentPid（Reset圆盘AlignmentPID），使用无参形式推进相应操作。
        ResetAlignmentWait();  // 调用 ResetAlignmentWait，使用无参形式推进相应操作。
        // 手动停止仍取消业务等待；重复启动已在上方保护自动状态。
        const bool explicitStop=!enable&&acknowledge&&autoAlignmentState==AutoAlignmentState::WAITING;  // 将 !enable&&acknowledge&&autoAlignmentState==AutoAlignmen… 写入或初始化 explicitStop（explicit停止）。
        autoAlignmentState=explicitStop?AutoAlignmentState::FAILED  // 将 explicitStop?AutoAlignmentState::FAILED 写入或初始化 autoAlignmentState（autoAlignment状态）。
            :automatic?AutoAlignmentState::WAITING:AutoAlignmentState::IDLE;  // 续写当前声明或运算，提供 :automatic?AutoAlignmentState::WAITING:AutoAlignmentSt…。
        autoAlignmentStartedMs = millis();  // 将 millis() 写入或初始化 autoAlignmentStartedMs（autoAlignmentStarted毫秒）。
        alignmentEnabled = enable;  // 将 enable 写入或初始化 alignmentEnabled（alignment已启用）。
        updateVisionLight();  // 调用 updateVisionLight（update视觉Light），使用无参形式推进相应操作。
        if(enable)traceVision(alignmentOwner,true);  // 当 enable 成立时进入此分支，否则继续后续流程。
        xSemaphoreGive(xAlignmentMotionMutex);  // 调用 xSemaphoreGive（横向SemaphoreGive），传入 xAlignmentMotionMutex。
    }  // 结束此函数或控制块。

    WakeMotionExecutor();  // 调用 WakeMotionExecutor，使用无参形式推进相应操作。
    if (acknowledge) {  // 当 acknowledge 成立时进入此分支，否则继续后续流程。
        Serial.println(enable ? "{RSP,VISION,ALIGN_START,OK}" : "{RSP,VISION,ALIGN_STOP,OK}");  // 调用 Serial.println，传入 enable ? "{RSP,VISION,ALIGN_START,OK}" : "{RSP,VISION,…。
    }  // 结束此条件分支。
}  // 结束此函数或控制块。

// ================= 任务/队列句柄 =================
TaskHandle_t xTask_MainStateMachine_Handle = NULL;  // 将 NULL 写入或初始化 xTask_MainStateMachine_Handle（横向任务Main状态MachineHandle）。
QueueHandle_t xVisualTaskQueue = NULL;      // 机载电脑指令队列
TimerHandle_t xHomeTimer = NULL;            // 总超时兜底(回启停区)
SemaphoreHandle_t xLidarPoseMutex = NULL;   // 防止自动流程和串口重复执行雷达位姿

// 执行雷达扫描位姿并统一发送串口应答。
// Release 成功后缓存结果，防止机载电脑重试导致相对位移累加。
// Debug 手动调试可显式允许重做，不读写 Release 的完成记录。
namespace {  // 开始匿名命名空间，本文件内部共享其中的定义。
bool releasePoseCompleted=false,releasePoseInitialized=false;  // 将 false,releasePoseInitialized=false 写入或初始化 releasePoseCompleted。
uint32_t releasePoseId=0;  // 将 0 写入或初始化 releasePoseId（releasePose编号）。
}  // 结束命名空间。
/**
 * @brief 处理扫码准备作业结束，仅在正式模式成功时登记准备已完成。
 */
void CompleteLidarPose(bool success,bool debug){if(!debug&&success)releasePoseCompleted=true;}  // 将 true;} 写入或初始化 releasePoseCompleted。
/**
 * @brief 按起始区提交扫码准备姿态，正式模式复用已完成标志避免重复动作。
 * @param fromCommand true 表示串口显式请求。
 * @param allowRepeat 是否允许重复执行已完成的正式准备动作。
 * @return 非零作业编号；0 表示未提交或无需重复。
 */
uint32_t SubmitLidarPoseAction(bool fromCommand,bool allowRepeat,uint8_t startZone) {  // 定义函数或成员 的入口 SubmitLidarPoseAction，参数由此处的签名确定。
    if(!xLidarPoseMutex||xSemaphoreTake(xLidarPoseMutex,portMAX_DELAY)!=pdTRUE)return 0;  // 若 !xLidarPoseMutex||xSemaphoreTake(xLidarPoseMutex,portM…，立即返回 0。
    if(!allowRepeat&&releasePoseCompleted) {  // 当 !allowRepeat&&releasePoseCompleted 成立时进入此分支，否则继续后续流程。
        if(fromCommand){Serial.println("{RSP,VISION,LIDAR_POSE,OK}");Serial.println("{EVT,VISION,LIDAR_POSE_DONE}");}  // 当 fromCommand 成立时进入此分支，否则继续后续流程。
        xSemaphoreGive(xLidarPoseMutex);return releasePoseId;  // 调用 xSemaphoreGive（横向SemaphoreGive），传入 xLidarPoseMutex。然后结束当前函数并返回 releasePoseId。
    }  // 结束此条件分支。
    if(!allowRepeat&&releasePoseId&&!GetMotionStatus(releasePoseId).complete()) {  // 当 !allowRepeat&&releasePoseId&&!GetMotionStatus(releaseP… 成立时进入此分支，否则继续后续流程。
        if(fromCommand)Serial.println("{RSP,VISION,LIDAR_POSE,ERR,BUSY}");  // 当 fromCommand 成立时进入此分支，否则继续后续流程。
        xSemaphoreGive(xLidarPoseMutex);return fromCommand?0:releasePoseId;  // 调用 xSemaphoreGive（横向SemaphoreGive），传入 xLidarPoseMutex。然后结束当前函数并返回 fromCommand?0:releasePoseId。
    }  // 结束此条件分支。
    // 采样期间复用忙碌应答，保持底盘静止；退出前释放姿态互斥量。
    if(H30UsbHeadingAverageActive()||MotionExecutorActive()){if(fromCommand)Serial.println("{RSP,VISION,LIDAR_POSE,ERR,BUSY}");xSemaphoreGive(xLidarPoseMutex);return 0;}  // 当 H30UsbHeadingAverageActive()||MotionExecutorActive() 成立时进入此分支，否则继续后续流程。
    const uint8_t zone=startZone?startZone:static_cast<uint8_t>(currentStartZone);  // 将 startZone?startZone:static_cast<uint8_t>(currentStartZ… 写入或初始化 zone。
    if(!allowRepeat&&!releasePoseInitialized) {  // 当 !allowRepeat&&!releasePoseInitialized 成立时进入此分支，否则继续后续流程。
        switch(zone) {  // 按 zone 的值选择对应分支。
        case START_ZONE_1: currentPose={2250,150,StartZoneHeading(1)};break;  // 当前值匹配 START_ZONE_1 时从这里执行。
        case START_ZONE_2: currentPose={150,150,StartZoneHeading(2)};break;  // 当前值匹配 START_ZONE_2 时从这里执行。
        default: xSemaphoreGive(xLidarPoseMutex);return 0;  // 没有匹配的 case 时走此默认分支。然后结束当前函数并返回 0。
        }  // 结束此函数或控制块。
        H30UsbBindHeadingAverage(currentPose.theta);  // 调用 H30UsbBindHeadingAverage（H30UsbBind航向Average），传入 currentPose.theta。
        releasePoseInitialized=true;  // 将 true 写入或初始化 releasePoseInitialized。
    }  // 结束此条件分支。
    MotionRequest request;request.kind=MotionKind::LIDAR_POSE;request.zone=zone;request.debug=allowRepeat;  // 声明 request（请求），类型和可用范围由所在声明决定。然后将 MotionKind::LIDAR_POSE 写入或初始化 request.kind。然后将 zone 写入或初始化 request.zone。然后将 allowRepeat 写入或初始化 request.debug。
    strcpy(request.category,"VISION");strcpy(request.command,"LIDAR_POSE");  // 调用 strcpy，传入 request.category,"VISION"。然后调用 strcpy，传入 request.command,"LIDAR_POSE"。
    // ACK 先于通知工作任务：提交在串口应答之后，完成事件由执行器发出。
    if(fromCommand)Serial.println("{RSP,VISION,LIDAR_POSE,ACK}");  // 当 fromCommand 成立时进入此分支，否则继续后续流程。
    const uint32_t id=SubmitMotion(request);if(!allowRepeat)releasePoseId=id;  // 将 SubmitMotion(request) 写入或初始化 id（编号）。然后当 !allowRepeat 成立时进入此分支，否则继续后续流程。
    if(!fromCommand&&id)Serial.println("{EVT,VISION,LIDAR_POSE_RUNNING}");  // 当 !fromCommand&&id 成立时进入此分支，否则继续后续流程。
    xSemaphoreGive(xLidarPoseMutex);return id;  // 调用 xSemaphoreGive（横向SemaphoreGive），传入 xLidarPoseMutex。然后结束当前函数并返回 id。
}  // 结束此函数或控制块。
// ================= 视觉闭环对齐任务 =================
/**
 * @brief 处理最新视觉反馈并推进一次底盘对齐闭环；由统一运动任务调用。
 */
// 轮询视觉对齐的运行状态。
// 调试视觉反馈时要结合启动模式、帧是否更新、误差输入及完成状态，串口收帧入口在 serial_commands.inc。
void TickVisualAlignment() {  // 定义函数或成员 的入口 TickVisualAlignment（轮询VisualAlignment），参数由此处的签名确定。
    static bool alignedReported=false;  // 将 false 写入或初始化 alignedReported。
    static bool discHeadingUnavailable=false;  // 将 false 写入或初始化 discHeadingUnavailable（圆盘航向Unavailable）。
    static uint32_t lastFeedbackMs=0,lastLogMs=0,previousGeneration=0,previousFrameSequence=0;  // 将 0,lastLogMs=0,previousGeneration=0,previousFrameSequen… 写入或初始化 lastFeedbackMs（最后反馈毫秒）。
    static bool hasFeedback=false;  // 将 false 写入或初始化 hasFeedback（has反馈）。
    static float lastAngleError=NAN,lastVisualX=0,lastVisualY=0;  // 将 NAN,lastVisualX=0,lastVisualY=0 写入或初始化 lastAngleError（最后角度错误）。
    if(!xAlignmentMotionMutex)return;  // 当 !xAlignmentMotionMutex 成立时进入此分支，否则继续后续流程。
    xSemaphoreTake(xAlignmentMotionMutex,portMAX_DELAY);  // 调用 xSemaphoreTake（横向SemaphoreTake），传入 xAlignmentMotionMutex,portMAX_DELAY。
    const uint32_t generation=alignmentGeneration;  // 将 alignmentGeneration 写入或初始化 generation。
    if(alignmentStopPending){OmniMove(0,0,0,0);alignmentStopPending=false;}  // 当 alignmentStopPending 成立时进入此分支，否则继续后续流程。
    if(generation!=previousGeneration) {  // 当 generation!=previousGeneration 成立时进入此分支，否则继续后续流程。
        alignedReported=false;lastFeedbackMs=0;  // 将 false 写入或初始化 alignedReported。然后将 0 写入或初始化 lastFeedbackMs（最后反馈毫秒）。
        discHeadingUnavailable=false;  // 将 false 写入或初始化 discHeadingUnavailable（圆盘航向Unavailable）。
        hasFeedback=false;  // 将 false 写入或初始化 hasFeedback（has反馈）。
        previousGeneration=generation;previousFrameSequence=0;  // 将 generation 写入或初始化 previousGeneration（上一Generation）。然后将 0 写入或初始化 previousFrameSequence（上一帧Sequence）。
    }  // 结束此条件分支。
    const auto mode=alignmentOwner==VisionStartMode::NONE?VisionStartMode::DISC:alignmentOwner;  // 将 alignmentOwner==VisionStartMode::NONE?VisionStartMode:… 写入或初始化 mode（模式）。
    const auto reportResult=[mode](const char *result) {  // 将 [mode](const char *result) { 写入或初始化 reportResult（report结果）。
        const char *name=visionNames[static_cast<uint8_t>(mode)];  // 将 visionNames[static_cast<uint8_t>(mode)] 写入或初始化 name。
        if(!hasFeedback) {  // 当 !hasFeedback 成立时进入此分支，否则继续后续流程。
            Serial.printf("{EVT,VISION,ALIGN_RESULT,%s,%s,0,NA,NA,NA}\n",name,result);  // 调用 Serial.printf，传入 "{EVT,VISION,ALIGN_RESULT,%s,%s,0,NA,NA,NA}\n",name,re…。
            return;  // 结束当前函数，将执行权交回调用者。
        }  // 结束此条件分支。
        char angle[24]="NA";  // 将 "NA" 写入或初始化 angle[24]。
        if(!isnan(lastAngleError))snprintf(angle,sizeof(angle),"%.3f",lastAngleError);  // 当 !isnan(lastAngleError) 成立时进入此分支，否则继续后续流程。
        Serial.printf("{EVT,VISION,ALIGN_RESULT,%s,%s,1,%s,%.3f,%.3f}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
            name,result,angle,lastVisualX,lastVisualY);  // 续写当前声明或运算，提供 name,result,angle,lastVisualX,lastVisualY);。
    };  // 结束此初始化列表或局部作用域。
    constexpr uint32_t discTimeoutMs=1500;  // 将 1500 写入或初始化 discTimeoutMs（圆盘超时毫秒）。
    constexpr uint32_t workAreaTimeoutMs=2500;  // 将 2500 写入或初始化 workAreaTimeoutMs（工位Area超时毫秒）。
    const bool workArea=mode==VisionStartMode::WORK_AREA||mode==VisionStartMode::WORK_AREA_LOADED;  // 将 mode==VisionStartMode::WORK_AREA||mode==VisionStartMod… 写入或初始化 workArea（工位Area）。
    const uint32_t timeoutMs=mode==VisionStartMode::DISC?discTimeoutMs:  // 将 mode==VisionStartMode::DISC?discTimeoutMs: 写入或初始化 timeoutMs（超时毫秒）。
        workArea?workAreaTimeoutMs:AUTO_ALIGN_TIMEOUT_MS;  // 续写当前声明或运算，提供 workArea?workAreaTimeoutMs:AUTO_ALIGN_TIMEOUT_MS;。
    if(autoAlignmentState==AutoAlignmentState::WAITING&&millis()-autoAlignmentStartedMs>=timeoutMs) {  // 当 autoAlignmentState==AutoAlignmentState::WAITING&&milli… 成立时进入此分支，否则继续后续流程。
        OmniMove(0,0,0,0);ResetDiscAlignmentPid();ResetAlignmentWait();  // 调用 OmniMove，传入 0,0,0,0。然后调用 ResetDiscAlignmentPid（Reset圆盘AlignmentPID），使用无参形式推进相应操作。然后调用 ResetAlignmentWait，使用无参形式推进相应操作。
        traceVision(alignmentOwner,false,false);alignmentEnabled=false;  // 调用 traceVision（trace视觉），传入 alignmentOwner,false,false。然后将 false 写入或初始化 alignmentEnabled（alignment已启用）。
        autoAlignmentState=AutoAlignmentState::DONE;  // 将 AutoAlignmentState::DONE 写入或初始化 autoAlignmentState（autoAlignment状态）。
        updateVisionLight();  // 调用 updateVisionLight（update视觉Light），使用无参形式推进相应操作。
        reportResult("TIMEOUT_CONTINUE");  // 调用 reportResult（report结果），传入 "TIMEOUT_CONTINUE"。
    }  // 结束此条件分支。
    if(!alignmentEnabled){alignedReported=false;lastFeedbackMs=0;xSemaphoreGive(xAlignmentMotionMutex);return;}  // 当 !alignmentEnabled 成立时进入此分支，否则继续后续流程。
    VisualAlignmentFrame_t alignment;  // 声明 alignment，类型和可用范围由所在声明决定。
    // 串口/视觉接收只发布最新反馈；此处与导航在同一运动任务内下发底盘命令。
    if(xQueueReceive(xAlignmentQueue,&alignment,0)!=pdTRUE){xSemaphoreGive(xAlignmentMotionMutex);return;}  // 当 xQueueReceive(xAlignmentQueue,&alignment,0)!=pdTRUE 成立时进入此分支，否则继续后续流程。
    const uint32_t now=millis();  // 将 millis() 写入或初始化 now。
    const float dt=lastFeedbackMs?(now-lastFeedbackMs)*.001f:.05f;  // 将 lastFeedbackMs?(now-lastFeedbackMs)*.001f:.05f 写入或初始化 dt（时间差）。
    lastFeedbackMs=now;  // 将 now 写入或初始化 lastFeedbackMs（最后反馈毫秒）。
    if((previousFrameSequence&&alignment.sequence!=previousFrameSequence+1)||dt>=ALIGN_FEEDBACK_TIMEOUT_MS*.001f)ResetAlignmentWait();  // 当 (previousFrameSequence&&alignment.sequence!=previousFr… 成立时进入此分支，否则继续后续流程。
    previousFrameSequence=alignment.sequence;  // 将 alignment.sequence 写入或初始化 previousFrameSequence（上一帧Sequence）。
    traceVisionFirstFrame(mode);  // 调用 traceVisionFirstFrame（trace视觉首个帧），传入 mode。
    float angleError=alignment.angleDeg;  // 将 alignment.angleDeg 写入或初始化 angleError（角度错误）。
    const char *headingError=nullptr;  // 将 nullptr 写入或初始化 headingError（航向错误）。
    if(mode==VisionStartMode::DISC) {  // 当 mode==VisionStartMode::DISC 成立时进入此分支，否则继续后续流程。
        float heading=0;uint32_t ageMs=0;  // 将 0 写入或初始化 heading（航向）。然后将 0 写入或初始化 ageMs（样本年龄毫秒）。
        headingError=H30UsbGetHeading(heading,ageMs);  // 将 H30UsbGetHeading(heading,ageMs) 写入或初始化 headingError（航向错误）。
        // OmniMove正旋转使地图航向增加，现有角度PID输出在底盘映射时取负，故输入为实测-目标。
        angleError=headingError?0.0f:remainderf(heading-DISC_AREA_HEADING,360.0f);  // 将 headingError?0.0f:remainderf(heading-DISC_AREA_HEADING… 写入或初始化 angleError（角度错误）。
        if(headingError&&!discHeadingUnavailable)  // 当 headingError&&!discHeadingUnavailable 成立时进入此分支，否则继续后续流程。
            Serial.printf("{EVT,VISION,DISC_HEADING,ERR,%s}\n",headingError);  // 调用 Serial.printf，传入 "{EVT,VISION,DISC_HEADING,ERR,%s}\n",headingError。
        if(!headingError&&discHeadingUnavailable)Serial.println("{EVT,VISION,DISC_HEADING,OK}");  // 当 !headingError&&discHeadingUnavailable 成立时进入此分支，否则继续后续流程。
        discHeadingUnavailable=headingError!=nullptr;  // 将 headingError!=nullptr 写入或初始化 discHeadingUnavailable（圆盘航向Unavailable）。
    }  // 结束此条件分支。
    const float visualX=cornerXYInverted?-alignment.visualX:alignment.visualX;  // 将 cornerXYInverted?-alignment.visualX:alignment.visualX 写入或初始化 visualX（visual横向）。
    const float visualY=cornerXYInverted?-alignment.visualY:alignment.visualY;  // 将 cornerXYInverted?-alignment.visualY:alignment.visualY 写入或初始化 visualY（visual纵向）。
    bool aligned=false;  // 将 false 写入或初始化 aligned。
    if(headingError) {  // 当 headingError 成立时进入此分支，否则继续后续流程。
        // 不沿用旧角度或退回摄像头角度；零角差复位角度PID，X/Y仍使用当前视觉反馈。
        AlignToVisionContinuous(mode,0.0f,visualX,visualY,dt,ALIGN_PID_MAX_SPEED_RPM);  // 调用 AlignToVisionContinuous（AlignTo视觉Continuous），传入 mode,0.0f,visualX,visualY,dt,ALIGN_PID_MAX_SPEED_RPM。
        ResetAlignmentWait();  // 调用 ResetAlignmentWait，使用无参形式推进相应操作。
    } else {  // 前一个判断不成立时执行此分支。
        aligned=WaitForAlignment(mode,angleError,visualX,visualY,dt,ALIGN_PID_MAX_SPEED_RPM);  // 将 WaitForAlignment(mode,angleError,visualX,visualY,dt,AL… 写入或初始化 aligned。
    }  // 结束此条件分支。
    hasFeedback=true;  // 将 true 写入或初始化 hasFeedback（has反馈）。
    lastAngleError=headingError?NAN:angleError;  // 将 headingError?NAN:angleError 写入或初始化 lastAngleError（最后角度错误）。
    lastVisualX=alignment.visualX;  // 将 alignment.visualX 写入或初始化 lastVisualX（最后Visual横向）。
    lastVisualY=alignment.visualY;  // 将 alignment.visualY 写入或初始化 lastVisualY（最后Visual纵向）。
    if(aligned&&autoAlignmentState==AutoAlignmentState::WAITING) {  // 当 aligned&&autoAlignmentState==AutoAlignmentState::WAITI… 成立时进入此分支，否则继续后续流程。
        autoAlignmentState=AutoAlignmentState::DONE;  // 将 AutoAlignmentState::DONE 写入或初始化 autoAlignmentState（autoAlignment状态）。
        reportResult("ALIGNED");  // 调用 reportResult（report结果），传入 "ALIGNED"。
    }  // 结束此条件分支。
    if(aligned&&!alignedReported)Serial.println("{EVT,VISION,ALIGN_DONE}");  // 当 aligned&&!alignedReported 成立时进入此分支，否则继续后续流程。
    alignedReported=aligned;  // 将 aligned 写入或初始化 alignedReported。
    if(!aligned&&now-lastLogMs>=ALIGN_LOG_INTERVAL_MS) {  // 当 !aligned&&now-lastLogMs>=ALIGN_LOG_INTERVAL_MS 成立时进入此分支，否则继续后续流程。
        Serial.printf("[Align] running: angle=%.2f deg, x=%.1f, y=%.1f, dt=%.3f, source=%s\n",angleError,alignment.visualX,alignment.visualY,dt,  // 通过串口执行 printf，输出内容或参数由本行给出。
            mode==VisionStartMode::DISC?(headingError?"IMU_UNAVAILABLE":"IMU"):"VISION");  // 续写当前声明或运算，提供 mode==VisionStartMode::DISC?(headingError?"IMU_UNAVAIL…。
        lastLogMs=now;  // 将 now 写入或初始化 lastLogMs（最后Log毫秒）。
    }  // 结束此条件分支。
    // 沿用项目当前设置：300ms 断流停车分支未启用，不扩大故障判定范围。
    xSemaphoreGive(xAlignmentMotionMutex);  // 调用 xSemaphoreGive（横向SemaphoreGive），传入 xAlignmentMotionMutex。
}  // 结束此函数或控制块。
// 通过 Serial0 通知机载电脑显示。
// 参数顺序：1. 命令类型 DISPLAY；2. 显示类型 TASK_CODE/DEBUG；3. 正文。
/**
 * @brief 封装并发送上位机显示事件；返回是否发送成功。
 */
bool updateDisplay(const char *commandType, const char *displayType, const char *content) {  // 定义函数或成员 的入口 updateDisplay，参数由此处的签名确定。
    if (commandType == nullptr || strcmp(commandType, "DISPLAY") != 0  // 当 commandType == nullptr || strcmp(commandType, "DISPLAY… 成立时进入此分支，否则继续后续流程。
            || displayType == nullptr  // 继续上一行的组合条件，本行提供 || displayType == nullptr。
            || (strcmp(displayType, "TASK_CODE") != 0 && strcmp(displayType, "DEBUG") != 0)  // 继续上一行的组合条件，本行提供 || (strcmp(displayType, "TASK_CODE") != 0 && strcmp(displayType, "DEBUG") != 0)。
            || content == nullptr || *content == '\0') return false;  // 继续上一行的组合条件，本行提供 || content == nullptr || *content == '\0') return false;。

    // 正文是一个字段：拒绝分隔符和控制字符，避免正文伪造第二条串口指令。
    for (const unsigned char *cursor = reinterpret_cast<const unsigned char *>(content);  // 按 const unsigned char *cursor = reinterpret_cast<const u… 组织循环的初值、继续条件和步进。
            *cursor != '\0'; ++cursor) {  // 续写当前声明或运算，提供 *cursor != '\0'; ++cursor) {。
        if (*cursor == ',' || *cursor == '{' || *cursor == '}'  // 当 *cursor == ',' || *cursor == ' 成立时进入此分支，否则继续后续流程。
                || *cursor < 0x20 || *cursor == 0x7f) return false;  // 继续上一行的组合条件，本行提供 || *cursor < 0x20 || *cursor == 0x7f) return false;。
    }  // 结束此初始化列表或局部作用域。
    char frame[SERIAL_FRAME_BYTES];  // 声明 frame（帧），类型和可用范围由所在声明决定。
    const int length = snprintf(frame, sizeof(frame), "{EVT,%s,%s,%s}",  // 将 snprintf(frame, sizeof(frame), "{EVT,%s,%s,%s}", 写入或初始化 length（长度）。
                                commandType, displayType, content);  // 声明 displayType，类型和可用范围由所在声明决定。
    if (length < 0 || static_cast<size_t>(length) >= sizeof(frame)) return false;  // 若 length < 0 || static_cast<size_t>(length) >= sizeof(fr…，立即返回 false。

    // 状态机有 50 ms 轮询，重复正文只发一次，避免日志和上位机被刷屏。
    static char previousFrame[SERIAL_FRAME_BYTES] = {0};  // 将 {0} 写入或初始化 previousFrame[SERIAL_FRAME_BYTES]。
    if (strcmp(previousFrame, frame) == 0) return true;  // 若 strcmp(previousFrame, frame) == 0，立即返回 true。
    Serial.println(frame);  // 调用 Serial.println，传入 frame。
    memcpy(previousFrame, frame, static_cast<size_t>(length) + 1);  // 调用 memcpy，传入 previousFrame, frame, static_cast<size_t>(length) + 1。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

// 主控请求机载电脑切换视觉功能；这些是单次请求，不代表视觉已启动或已对齐。
// 顺序与 Document/serial_protocol.md 中的五种模式一致。
static void requestVisionFunction(VisionStartMode mode, bool start,bool invertCornerXY=false) {  // 定义函数或成员 的入口 requestVisionFunction（请求视觉Function），参数由此处的签名确定。
    if (mode == VisionStartMode::DISC_MATERIAL) {  // 当 mode == VisionStartMode::DISC_MATERIAL 成立时进入此分支，否则继续后续流程。
        discMessageResumeRequested = false;  // 将 false 写入或初始化 discMessageResumeRequested（圆盘MessageResumeRequested）。
        discMaterialActive = start;  // 将 start 写入或初始化 discMaterialActive（圆盘物料活跃）。
        updateVisionLight();  // 调用 updateVisionLight（update视觉Light），使用无参形式推进相应操作。
        traceVision(mode,start);  // 调用 traceVision（trace视觉），传入 mode,start。
        discMaterialColor = 0;  // 将 0 写入或初始化 discMaterialColor（圆盘物料颜色）。
    }  // 结束此条件分支。
    const char *name = nullptr;  // 将 nullptr 写入或初始化 name。
    switch (mode) {  // 按 mode 的值选择对应分支。
        case VisionStartMode::DISC:  // 当前值匹配 VisionStartMode::DISC 时从这里执行。
            name = "DISC";  // 将 "DISC" 写入或初始化 name。
            break;  // 退出当前循环或 switch 分支。
        case VisionStartMode::DISC_MATERIAL:  // 当前值匹配 VisionStartMode::DISC_MATERIAL 时从这里执行。
            name = "DISC_MATERIAL";  // 将 "DISC_MATERIAL" 写入或初始化 name。
            break;  // 退出当前循环或 switch 分支。
        case VisionStartMode::WORK_AREA:  // 当前值匹配 VisionStartMode::WORK_AREA 时从这里执行。
            name = "WORK_AREA";  // 将 "WORK_AREA" 写入或初始化 name。
            break;  // 退出当前循环或 switch 分支。
        case VisionStartMode::WORK_AREA_LOADED:  // 当前值匹配 VisionStartMode::WORK_AREA_LOADED 时从这里执行。
            name = "WORK_AREA_LOADED";  // 将 "WORK_AREA_LOADED" 写入或初始化 name。
            break;  // 退出当前循环或 switch 分支。
        case VisionStartMode::CORNER:  // 当前值匹配 VisionStartMode::CORNER 时从这里执行。
            name = "CORNER";  // 将 "CORNER" 写入或初始化 name。
            break;  // 退出当前循环或 switch 分支。
        case VisionStartMode::NONE:  // 当前值匹配 VisionStartMode::NONE 时从这里执行。
            return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此函数或控制块。
    // 自动流程和串口按钮共用入口：先开启底盘闭环，再请求相机反馈。
    if (VisionModeUsesAlignment(mode)) {  // 当 VisionModeUsesAlignment(mode) 成立时进入此分支，否则继续后续流程。
        setAlignmentEnabled(start, false, start, mode, !start,invertCornerXY);  // 调用 setAlignmentEnabled（setAlignment已启用），传入 start, false, start, mode, !start,invertCornerXY。
    }  // 结束此条件分支。
    if (name) Serial.printf("{EVT,VISION,%s,%s}\n", start ? "START_REQUEST" : "STOP_REQUEST", name);  // 当 name 成立时进入此分支，否则继续后续流程。
}  // 结束此初始化列表或局部作用域。

/**
 * @brief 发送开启指定视觉模式的请求，并按模式设置对齐或物料识别状态。
 */
void requestVisionStart(VisionStartMode mode,bool invertCornerXY) { requestVisionFunction(mode,true,invertCornerXY); }  // 定义函数或成员 requestVisionStart（请求视觉起点），参数由此处的签名确定。

// 定位功能同时结束其底盘闭环；事件不表示相机已经停止。
/**
 * @brief 发送停止指定视觉模式的请求，并结束对应本地控制状态。
 */
void requestVisionStop(VisionStartMode mode) { requestVisionFunction(mode, false); }  // 定义函数或成员 requestVisionStop（请求视觉停止），参数由此处的签名确定。

// 总超时可能发生在任意路段或搬运中，位置和持料状态未知，停机等待人工处理。
/**
 * @brief 比赛总计时器超时回调，执行既有停止、失败状态及主任务暂停处理。
 */
void vHomeTimerCallback(TimerHandle_t xTimer) {  // 定义函数或成员 的入口 vHomeTimerCallback（vHome定时器Callback），参数由此处的签名确定。
    if (currentState != STATE_DONE) {  // 当 currentState != STATE_DONE 成立时进入此分支，否则继续后续流程。
        taskMotionAborted = true;  // 将 true 写入或初始化 taskMotionAborted（任务MotionAborted）。
        CancelMotion();  // 调用 CancelMotion，使用无参形式推进相应操作。
        currentState = STATE_TIMEOUT_FAILED;  // 将 STATE_TIMEOUT_FAILED 写入或初始化 currentState（当前状态）。
        firstDiscGrabReady = false;  // 将 false 写入或初始化 firstDiscGrabReady（首个圆盘Grab就绪）。
        requestVisionStop(VisionStartMode::DISC_MATERIAL);  // 调用 requestVisionStop（请求视觉停止），传入 VisionStartMode::DISC_MATERIAL。
        requestVisionStop(VisionStartMode::DISC);  // 调用 requestVisionStop（请求视觉停止），传入 VisionStartMode::DISC。
        requestVisionStop(VisionStartMode::WORK_AREA);  // 调用 requestVisionStop（请求视觉停止），传入 VisionStartMode::WORK_AREA。
        requestVisionStop(VisionStartMode::WORK_AREA_LOADED);  // 调用 requestVisionStop（请求视觉停止），传入 VisionStartMode::WORK_AREA_LOADED。
        requestVisionStop(VisionStartMode::CORNER);  // 调用 requestVisionStop（请求视觉停止），传入 VisionStartMode::CORNER。
        setAlignmentEnabled(false);  // 调用 setAlignmentEnabled（setAlignment已启用），传入 false。
        portENTER_CRITICAL(&nodePathMux);  // 进入临界区，保护共享状态的本段访问。
        nodePathState = NodePathState::FAILED;  // 将 NodePathState::FAILED 写入或初始化 nodePathState（节点路径状态）。
        portEXIT_CRITICAL(&nodePathMux);  // 离开临界区，保护共享状态的本段访问。
        if (xTask_MainStateMachine_Handle != NULL)  // 当 xTask_MainStateMachine_Handle != NULL 成立时进入此分支，否则继续后续流程。
            vTaskSuspend(xTask_MainStateMachine_Handle);  // 调用 vTaskSuspend（v任务Suspend），传入 xTask_MainStateMachine_Handle。
        // 主任务在暂停前可能从失败的 MoveArm 返回，重新锁存总超时故障。
        currentState = STATE_TIMEOUT_FAILED;  // 将 STATE_TIMEOUT_FAILED 写入或初始化 currentState（当前状态）。
        // 停止六个步进电机，保留夹爪位置，避免未知持料时松手。
        for (uint8_t motor = 1; motor <= 6; ++motor) Emm_V5_Stop_Now(motor, false);  // 按 uint8_t motor = 1; motor <= 6; ++motor) Emm_V5_Stop_No… 组织循环的初值、继续条件和步进。
        updateDisplay("DISPLAY", "DEBUG", "TASK TIMEOUT");  // 调用 updateDisplay，传入 "DISPLAY", "DEBUG", "TASK TIMEOUT"。
        Serial.println("{EVT,NAV,ROUTE_FAILED,TIMEOUT}");  // 调用 Serial.println，传入 "{EVT,NAV,ROUTE_FAILED,TIMEOUT}"。
        Serial.println("[TIMER] Task timeout: stopped; manual recovery required");  // 调用 Serial.println，传入 "[TIMER] Task timeout: stopped; manual recovery requir…。
    }  // 结束此条件分支。
}  // 结束此函数或控制块。

// @brief 等待扫码消息队列 (供主状态机在各环节调用)
// @param out       输出缓冲区
// @param len       缓冲区长度
// @param timeoutMs 阻塞超时(ms), 0=非阻塞
// @return true 拿到一帧扫码字符串
/**
 * @brief 从扫码枪消息队列取出一帧字符串。
 * @param out 输出任务码缓冲区。
 * @param len 缓冲区容量，包含结束符。
 * @param timeoutMs 等待毫秒数，0 表示立即查询。
 * @return true 取得数据，false 没有数据或参数无效。
 */
bool waitScannerCode(char *out, uint32_t len, uint32_t timeoutMs) {  // 定义函数或成员 的入口 waitScannerCode（wait扫码器Code），参数由此处的签名确定。
    if (Scanner_WaitCode(out, len, timeoutMs)) {  // 当 Scanner_WaitCode(out, len, timeoutMs) 成立时进入此分支，否则继续后续流程。
        if (!IsSerialDebugMode()) Serial.printf("[SCANNER] recv: %s\n", out);  // 调用 Serial.printf，传入 "[SCANNER] recv: %s\n", out。
        return true;  // 结束当前函数并返回 true。
    }  // 结束此条件分支。
    return false;  // 结束当前函数并返回 false。
}  // 结束此函数或控制块。

// ================= 主状态机 =================
// 底盘、升降和伸缩回读理想值；转盘电机与夹爪舵机现场读取，不触发运动。
// Debug 和 Release 共用此接口；单位顺序为 mm/mm/度/mm/mm/度/度。
static void sendPoseQueryReply() {  // 定义函数或成员 的入口 sendPoseQueryReply，参数由此处的签名确定。
    const RobotPose robot = currentPose;  // 将 currentPose 写入或初始化 robot。
    ArmPose arm = currentArm;  // 将 currentArm 写入或初始化 arm（机械臂）。
    // 转盘读取张大头电机多圈位置，夹爪读取舵机单圈角度。
    // 读取失败不以命令目标/上次角度冒充实测值，也不改写运动目标 currentArm。
    if (!TurretMotorReadAngle(arm.turret_angle) || !isfinite(arm.turret_angle)) {  // 当 !TurretMotorReadAngle(arm.turret_angle) || !isfinite(a… 成立时进入此分支，否则继续后续流程。
        Serial.println("{RSP,POSE,GET,ERR,MOTOR_READ,7}");  // 调用 Serial.println，传入 "{RSP,POSE,GET,ERR,MOTOR_READ,7}"。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    if (!Servo_QueryAngle(1, arm.pawl_angle) || !isfinite(arm.pawl_angle)) {  // 当 !Servo_QueryAngle(1, arm.pawl_angle) || !isfinite(arm.… 成立时进入此分支，否则继续后续流程。
        Serial.println("{RSP,POSE,GET,ERR,SERVO_READ,1}");  // 调用 Serial.println，传入 "{RSP,POSE,GET,ERR,SERVO_READ,1}"。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    Serial.printf("{RSP,POSE,GET,OK,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                  robot.x, robot.y, robot.theta, arm.high, arm.length,  // 为当前列表或跨行表达式提供这一项 robot.x, robot.y, robot.theta, arm.high, arm.length。
                  arm.turret_angle, arm.pawl_angle);  // 续写当前声明或运算，提供 arm.turret_angle, arm.pawl_angle);。
}  // 结束此函数或控制块。

#include "serial_commands.inc"  // 在当前位置展开 serial_commands.inc，与本文件共用编译单元。

// 开机读取一个舵机的角度，并同步到主控维护的机械臂位姿。
/**
 * @brief 启动时读取夹爪实际角度，成功时写入 storedAngle；不发运动指令。
 */
void readStartupServoAngle(uint8_t servoId, float &storedAngle) {  // 定义函数或成员 的入口 readStartupServoAngle（readStartupServo角度），参数由此处的签名确定。
    Serial.printf("[Servo] 开机角度读取开始: ID=%u\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                  static_cast<unsigned>(servoId));  // 声明接口 unsigned>，参数由此处的签名确定。
    float measuredAngle = 0.0f;  // 将 0.0f 写入或初始化 measuredAngle（measured角度）。
    const bool readOk = Servo_QueryAngle(servoId, measuredAngle, 100);  // 将 Servo_QueryAngle(servoId, measuredAngle, 100) 写入或初始化 readOk。
    if (!readOk || !isfinite(measuredAngle)) {  // 当 !readOk || !isfinite(measuredAngle) 成立时进入此分支，否则继续后续流程。
        Serial.printf("[Servo] 开机角度读取失败: ID=%u, 保留原值=%.1f度, 未进行归一化\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                      static_cast<unsigned>(servoId), storedAngle);  // 声明接口 unsigned>，参数由此处的签名确定。
        return; // 读取失败时保留原值，不把失败当作测得 0 度。
    }  // 结束此条件分支。

    float normalizedAngle = measuredAngle;  // 将 measuredAngle 写入或初始化 normalizedAngle（normalized角度）。
    // 1. 0~360 度（含端点）保持不变。
    // 2. 超出范围时映射到 [0, 360)，例如 -55 -> 305、725 -> 5。
    // 3. 只归一化主控中的角度数值，不命令舵机旋转或修改舵机零点。
    const bool needsNormalization = normalizedAngle < 0.0f || normalizedAngle > 360.0f;  // 将 normalizedAngle < 0.0f || normalizedAngle > 360.0f 写入或初始化 needsNormalization。
    if (needsNormalization) {  // 当 needsNormalization 成立时进入此分支，否则继续后续流程。
        normalizedAngle = fmodf(normalizedAngle, 360.0f);  // 将 fmodf(normalizedAngle, 360.0f) 写入或初始化 normalizedAngle（normalized角度）。
        if (normalizedAngle < 0.0f) normalizedAngle += 360.0f;  // 当 normalizedAngle < 0.0f 成立时进入此分支，否则继续后续流程。
    }  // 结束此条件分支。
    storedAngle = normalizedAngle;  // 将 normalizedAngle 写入或初始化 storedAngle（stored角度）。
    Serial.printf("[Servo] 开机角度读取成功: ID=%u, 原始角度=%.1f度, %s, 保存角度=%.1f度（仅更新数值）\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                  static_cast<unsigned>(servoId), measuredAngle,  // 定义函数或成员 unsigned>，参数由此处的签名确定。
                  needsNormalization ? "超出0~360度，已归一化" : "在0~360度内，无需归一化",  // 为当前列表或跨行表达式提供这一项 needsNormalization ? "超出0~360度，已归一化" : "在0~360度内，无需归一化"。
                  storedAngle);  // 续写当前声明或运算，提供 storedAngle);。
}  // 结束此函数或控制块。
