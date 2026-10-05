/**********************************************************
*** 2027 工创大赛·智能搬运 主控程序框架
*** 角色：ESP32 主控（运动控制 + 任务编排 + 决策）
*** 外部协作：机载电脑负责视觉识别 + 下发指令
***          （颜色/位置识别、二维码读取、转盘物料定位）
***
*** 说明：本文件为"可移植基础框架"。
***       原超市赛的补货/提货/配送强业务逻辑已删除，
***       替换为 2027 智能搬运赛制的状态机骨架 + 伪代码。
***       标有 [TODO] 的部分，需按实际机械/定位方案实现。
**********************************************************/
//////////////////////////////////////////////////123
// // 硬件串口 1
// #define UART1_TX 17
// #define UART1_RX 18

// // 硬件串口 2
// #define UART2_TX 15
// #define UART2_RX 16

// // 软件串口 3 (仅 RX)
// #define SOFT_RX3 4


#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <FastLED.h>
#include "ota_service.h"
#include <ArduinoJson.h>

#include "Emm_V5.h"
#include "chassis.h"
#include "servo.h"
#include "scanner.h"
#include "material_transfer.h"
#include "runtime_parameters.h"
#include "serial_frame.h"
#include "led_pwm.h"

// ================= 基础配置 =================
#define LED_PIN 48
#define NUM_LEDS 1
#define OTA_HOSTNAME "smartcarrier_ESP32S3"
#define VERSION "0.1.4-framework"

uint32_t ALIGN_PID_MAX_SPEED_RPM = 20;  // 移动速度单位为转/分
constexpr uint32_t ALIGN_FEEDBACK_TIMEOUT_MS = 300;
constexpr uint32_t ALIGN_LOG_INTERVAL_MS = 500;
constexpr uint32_t AUTO_ALIGN_TIMEOUT_MS = 15000; // 包含等待首帧的时间。
constexpr uint8_t SCAN_AREA_NODE = 2;
constexpr uint8_t DISC_AREA_NODE = 14;
// 粗加工区中心约为 (210, 1100)，对应当前 5x5 地图的 10 号节点。
constexpr uint8_t COARSE_AREA_NODE = 10;

CRGB leds[NUM_LEDS];  // LED 像素数组(板载 WS2812B)

// 电机使能/同步常量(沿用 Emm_V5)
#define CHASSIS_MOTOR_1 1
#define CHASSIS_MOTOR_2 2
#define CHASSIS_MOTOR_3 3
#define CHASSIS_MOTOR_4 4
#define LIFT_MOTOR_5   5      // 升降电机
#define ARM_MOTOR_6   6      // 机械臂电机

// 运动标定系数
float X_PULSE     = 13.3f;    // X向 每毫米脉冲
float Y_PULSE     = 13.6f;    // Y向 每毫米脉冲
float THETA_PULSE = 51.8f;    // 旋转 每度脉冲
float HEIGHT_PULSE = 80.0f;  // 升降机械臂 每毫米脉冲
float LENGTH_PULSE = 30.2f;  // 伸缩机械臂 每毫米脉冲


// ================= 任务码 =================
// 2027赛制任务码格式: 四组三位数 "R1+ P1+ R2+ P2"
//   第一组: 第一批3个物料颜色顺序(红1黄2蓝3绿4黑5浅蓝6)
//   第二组: 第一批在粗加工区/暂存区的放置位置(1-3)
//   第三组: 第二批3个物料颜色顺序
//   第四组: 第二批在粗加工区的放置位置
// 例: "156+123+516+231"

struct TaskCode {
    int round1_colors[3]; // 第一批颜色顺序
    int round1_pos[3];    // 第一批放置位置
    int round2_colors[3]; // 第二批颜色顺序
    int round2_pos[3];    // 第二批放置位置
    bool valid;           // 是否解析成功
};

TaskCode currentTask = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, false };

/**
 * @brief 解析任务码字符串,如 "156+123+516+231"
 * @return 解析结果(含 valid 标志)
 */
TaskCode parseTaskCode(const char* code) {
    TaskCode tc = {}; // 解析失败时 valid 保持 false，避免未初始化数据误报成功。
    // 初始化...
    int a1,a2,a3,b1,b2,b3,c1,c2,c3,d1,d2,d3;
    if (sscanf(code, "%1d%1d%1d+%1d%1d%1d+%1d%1d%1d+%1d%1d%1d",
               &a1,&a2,&a3,&b1,&b2,&b3,&c1,&c2,&c3,&d1,&d2,&d3) == 12) {
        tc.round1_colors[0]=a1; tc.round1_colors[1]=a2; tc.round1_colors[2]=a3;
        tc.round1_pos[0]=b1;    tc.round1_pos[1]=b2;    tc.round1_pos[2]=b3;
        tc.round2_colors[0]=c1; tc.round2_colors[1]=c2; tc.round2_colors[2]=c3;
        tc.round2_pos[0]=d1;    tc.round2_pos[1]=d2;    tc.round2_pos[2]=d3;
        tc.valid = true;
    }
    return tc;
}

// ================= 机载电脑通信(串口)协议 =================
// 机器帧使用 {CMD/RSP/EVT,类别,动作,...}\n，详见 Document/serial_protocol.md。

// ================= 业务状态 =================
enum RobotState {
    STATE_WAIT_START,    // 待机,等一键启动
    STATE_READ_TASK,     // 读取任务码(二维码板 / 机载电脑)
    STATE_SCAN_FAILED,   // 扫码重试耗尽，保持停车，不进入抓取流程
    STATE_ROUTE_FAILED,  // 路径失败，不进入后续用户代码
    STATE_ALIGN_FAILED,  // 自动对齐失败，停车等待处理
    STATE_TRANSFER_FAILED, // 搬运动作失败，保持停车等待处理
    STATE_GRAB_ROUND1,   // 第一批: 转盘抓取 3 个物料
    STATE_PLACE_COARSE1, // 第一批: 放到粗加工区
    STATE_PLACE_TEMP1,   // 第一批: 放到暂存区
    STATE_GRAB_ROUND2,   // 第二批: 转盘抓取
    STATE_PLACE_COARSE2, // 第二批: 放到粗加工区
    STATE_STACK_TEMP2,   // 第二批: 码垛到暂存区(叠在第一批上)
    STATE_RETURN_HOME,   // 回启停区,上报完成统计
    STATE_DONE
};
RobotState currentState = STATE_WAIT_START;

enum StartZone {
    START_ZONE_UNKNOWN = 0,
    START_ZONE_1 = 1,
    START_ZONE_2 = 2
};

// 共享业务变量(由机载电脑指令/任务更新)
volatile bool nano_ready = false;       // 机载电脑就绪
volatile StartZone currentStartZone = START_ZONE_UNKNOWN; // 当前启停区,由机载电脑告知
volatile bool taskReceived = false;// 已拿到任务码
volatile int  roundProgress = 0;   // 当前轮次已抓/放物料数 0-3
volatile bool enableRun = false;   // 一键启动触发

// 两个任务通过临界区交接路径，避免串口在执行中覆盖节点数组。
enum class NodePathState { IDLE, WAITING, RECEIVING, RECEIVED, RUNNING, DONE, FAILED };
NodePathState nodePathState = NodePathState::IDLE;
portMUX_TYPE nodePathMux = portMUX_INITIALIZER_UNLOCKED;
uint8_t nodePathBuffer[MAX_NODE_PATH_LENGTH] = {0};
size_t nodePathLen = 0;

static bool executeNodePathWithStatus(const uint8_t *path, size_t count) {
    Serial.println("{EVT,NAV,ROUTE_RUNNING}");
    const bool success = MoveNodePath(path, count);
    // DONE 表示指令和预计等待已结束，不是电机/视觉实测到位。
    Serial.println(success ? "{EVT,NAV,ROUTE_DONE,ESTIMATED}" : "{EVT,NAV,ROUTE_FAILED,EXECUTION}");
    return success;
}

/**
 * @brief 请求路径，等待接收并同步执行，成功后才返回 true。
 * 1. 等待时只阻塞主状态机，串口任务仍能接收命令。
 * 2. 收到后检查首尾节点，再执行全部路段。
 * 3. 调用者必须检查返回值，失败时不能继续用户代码。
 * 等待仍受现有总任务超时保护，不再固定等待 10 秒。
 */
static bool requestAndMoveNodePath(uint8_t startNode, uint8_t endNode) {
    portENTER_CRITICAL(&nodePathMux);
    nodePathState = NodePathState::WAITING;
    nodePathLen = 0;
    portEXIT_CRITICAL(&nodePathMux);
    Serial.println("{EVT,NAV,ROUTE_WAITING}");
    Serial.printf("{EVT,NAV,ROUTE_REQUEST,%u,%u}\n", startNode, endNode);

    uint8_t path[MAX_NODE_PATH_LENGTH];
    size_t count = 0;
    for (;;) {
        portENTER_CRITICAL(&nodePathMux);
        const bool received = nodePathState == NodePathState::RECEIVED;
        if (received) {
            count = nodePathLen;
            memcpy(path, nodePathBuffer, count * sizeof(path[0]));
            nodePathState = NodePathState::RUNNING;
        }
        portEXIT_CRITICAL(&nodePathMux);
        if (received) break;
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    bool success = false;
    if (path[0] != startNode || path[count - 1] != endNode) {
        Serial.println("{EVT,NAV,ROUTE_FAILED,ENDPOINT}");
    } else {
        success = executeNodePathWithStatus(path, count);
    }
    portENTER_CRITICAL(&nodePathMux);
    nodePathState = success ? NodePathState::DONE : NodePathState::FAILED;
    portEXIT_CRITICAL(&nodePathMux);
    return success;
}

typedef struct {
    float angleDeg;
    float visualX;
    float visualY;
} VisualAlignmentFrame_t;

QueueHandle_t xAlignmentQueue = NULL;
SemaphoreHandle_t xAlignmentMotionMutex = NULL;
volatile bool alignmentEnabled = false;
enum class AutoAlignmentState { IDLE, WAITING, DONE, FAILED };
AutoAlignmentState autoAlignmentState = AutoAlignmentState::IDLE;
uint32_t autoAlignmentStartedMs = 0;
uint32_t alignmentGeneration = 0; // 受运动锁保护，丢弃跨启停周期的已出队帧。

static AutoAlignmentState getAutoAlignmentState() {
    if (!xAlignmentMotionMutex) return AutoAlignmentState::FAILED;
    xSemaphoreTake(xAlignmentMotionMutex, portMAX_DELAY);
    const auto state = autoAlignmentState;
    xSemaphoreGive(xAlignmentMotionMutex);
    return state;
}

// 参数提交与 PID 更新共用锁，避免一个控制周期读取到半套参数。
static void setAlignmentEnabled(bool enable, bool acknowledge = false, bool automatic = false) {
    if (xAlignmentMotionMutex != NULL
            && xSemaphoreTake(xAlignmentMotionMutex, portMAX_DELAY) == pdTRUE) {
        alignmentEnabled = false;
        ++alignmentGeneration;
        if (xAlignmentQueue != NULL) xQueueReset(xAlignmentQueue);
        OmniMove(0.0f, 0.0f, 0.0f, 0);
        ResetDiscAlignmentPid();
        // 手动启停会取消业务等待；不允许旧的 DONE 推进新流程。
        autoAlignmentState = automatic ? AutoAlignmentState::WAITING : AutoAlignmentState::IDLE;
        autoAlignmentStartedMs = millis();
        alignmentEnabled = enable;
        xSemaphoreGive(xAlignmentMotionMutex);
    }

    if (acknowledge) {
        Serial.println(enable ? "{RSP,VISION,ALIGN_START,OK}" : "{RSP,VISION,ALIGN_STOP,OK}");
    }
}

// ================= 任务/队列句柄 =================
TaskHandle_t xTask_MainStateMachine_Handle = NULL;
QueueHandle_t xVisualTaskQueue = NULL;      // 机载电脑指令队列
TimerHandle_t xHomeTimer = NULL;            // 总超时兜底(回启停区)
SemaphoreHandle_t xLidarPoseMutex = NULL;   // 防止自动流程和串口重复执行雷达位姿

// 队列元素: 机载电脑指令
typedef struct {
    char cmd[20];
    float param1, param2, param3;
} VisualCmd_t;

// ================= 函数声明 =================
void Task_MainStateMachine(void *pvParameters);// 主状态机任务
void Task_Serial_CMD(void *pvParameters);// 串口指令任务
void Task_VisualAlignment(void *pvParameters);// 视觉对齐任务
void vHomeTimerCallback(TimerHandle_t xTimer);// 总超时兜底(回启停区)

// 执行雷达扫描位姿并统一发送串口应答。
// ACK 表示命令已开始处理；OK 只会在全部动作执行完成后发送。
static bool runLidarPoseAction(bool fromCommand = false) {
    if (xLidarPoseMutex == NULL
            || xSemaphoreTake(xLidarPoseMutex, 0) != pdTRUE) {
        Serial.println(fromCommand ? "{RSP,VISION,LIDAR_POSE,ERR,BUSY}"
                                   : "{EVT,VISION,LIDAR_POSE_FAILED,BUSY}");
        return false;
    }
    Serial.println(fromCommand ? "{RSP,VISION,LIDAR_POSE,ACK}"
                               : "{EVT,VISION,LIDAR_POSE_RUNNING}");
    const bool ok = PrepareLidarScanPose(static_cast<uint8_t>(currentStartZone));
    Serial.println(ok ? "{EVT,VISION,LIDAR_POSE_DONE}" : "{EVT,VISION,LIDAR_POSE_FAILED}");
    xSemaphoreGive(xLidarPoseMutex);
    return ok;
}

// ================= 视觉闭环对齐任务 =================
void Task_VisualAlignment(void *pvParameters) {
    VisualAlignmentFrame_t alignment;
    bool alignedReported = false;
    bool feedbackActive = false;
    uint32_t lastFeedbackMs = 0;
    uint32_t lastLogMs = 0;
    uint32_t previousGeneration = 0;

    for (;;) {
        xSemaphoreTake(xAlignmentMotionMutex, portMAX_DELAY);
        const uint32_t generation = alignmentGeneration;
        if (generation != previousGeneration) {
            feedbackActive = false;
            alignedReported = false;
            lastFeedbackMs = 0;
            previousGeneration = generation;
        }
        if (autoAlignmentState == AutoAlignmentState::WAITING
                && millis() - autoAlignmentStartedMs >= AUTO_ALIGN_TIMEOUT_MS) {
            OmniMove(0.0f, 0.0f, 0.0f, 0);
            ResetDiscAlignmentPid();
            alignmentEnabled = false;
            autoAlignmentState = AutoAlignmentState::FAILED;
            Serial.println("{EVT,VISION,ALIGN_FAILED,TIMEOUT}");
        }
        xSemaphoreGive(xAlignmentMotionMutex);
        if (!alignmentEnabled) {
            feedbackActive = false;
            alignedReported = false;
            lastFeedbackMs = 0;
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (xQueueReceive(
                xAlignmentQueue, &alignment,
                pdMS_TO_TICKS(ALIGN_FEEDBACK_TIMEOUT_MS)) != pdTRUE) {
            if (!alignmentEnabled) {
                continue;
            }
            if (feedbackActive
                    && millis() - lastFeedbackMs >= ALIGN_FEEDBACK_TIMEOUT_MS) {
                // 视觉断流时立即撤销所有速度，禁止沿最后一次命令继续运动。
                if (xSemaphoreTake(
                        xAlignmentMotionMutex, portMAX_DELAY) == pdTRUE) {
                    if (generation != alignmentGeneration || !alignmentEnabled) {
                        xSemaphoreGive(xAlignmentMotionMutex);
                        continue;
                    }
                    OmniMove(0.0f, 0.0f, 0.0f, 0);
                    ResetDiscAlignmentPid();
                    if (autoAlignmentState == AutoAlignmentState::WAITING) {
                        autoAlignmentState = AutoAlignmentState::FAILED;
                        alignmentEnabled = false;
                    }
                    xSemaphoreGive(xAlignmentMotionMutex);
                }
                Serial.println("{EVT,VISION,ALIGN_FAILED,TIMEOUT}");
                feedbackActive = false;
                alignedReported = false;
                lastFeedbackMs = 0;
            }
            continue;
        }

        const uint32_t nowMs = millis();
        float dtSeconds = 0.05f;
        if (lastFeedbackMs != 0) {
            dtSeconds = static_cast<float>(nowMs - lastFeedbackMs) / 1000.0f;
        }
        lastFeedbackMs = nowMs;
        feedbackActive = true;

        bool aligned = false;
        if (xSemaphoreTake(xAlignmentMotionMutex, portMAX_DELAY) == pdTRUE) {
            if (alignmentEnabled && generation == alignmentGeneration) {
                aligned = AlignToDiscContinuous(
                    alignment.angleDeg, alignment.visualX, alignment.visualY,
                    dtSeconds, ALIGN_PID_MAX_SPEED_RPM
                );
                if (aligned && autoAlignmentState == AutoAlignmentState::WAITING) {
                    // 自动流程为单次对齐，锁内停车并锁存结果，后续反馈不能重启底盘。
                    alignmentEnabled = false;
                    autoAlignmentState = AutoAlignmentState::DONE;
                    Serial.println("{EVT,VISION,ALIGN_DONE}");
                    alignedReported = true;
                }
            }
            xSemaphoreGive(xAlignmentMotionMutex);
        }

        if (aligned) {
            // 已停车并完成 PID 复位；之后即使上位机停止发送也不报断流。
            feedbackActive = false;
            if (!alignedReported) {
                Serial.printf(
                    "[Align] OK: angle=%.2f deg, x=%.1f, y=%.1f\n",
                    alignment.angleDeg, alignment.visualX, alignment.visualY
                );
                Serial.println("{EVT,VISION,ALIGN_DONE}");
                alignedReported = true;
            }
            continue;
        }
        alignedReported = false;

        if (nowMs - lastLogMs >= ALIGN_LOG_INTERVAL_MS) {
            Serial.printf(
                "[Align PID] angle=%.2f deg, x=%.1f, y=%.1f, dt=%.3f s\n",
                alignment.angleDeg, alignment.visualX,
                alignment.visualY, dtSeconds
            );
            lastLogMs = nowMs;
        }
    }
}

// 通过 Serial0 通知机载电脑显示。
// 参数顺序：1. 命令类型 DISPLAY；2. 显示类型 TASK_CODE/DEBUG；3. 正文。
bool updateDisplay(const char *commandType, const char *displayType, const char *content) {
    if (commandType == nullptr || strcmp(commandType, "DISPLAY") != 0
            || displayType == nullptr
            || (strcmp(displayType, "TASK_CODE") != 0 && strcmp(displayType, "DEBUG") != 0)
            || content == nullptr || *content == '\0') return false;

    // 正文是一个字段：拒绝分隔符和控制字符，避免正文伪造第二条串口指令。
    for (const unsigned char *cursor = reinterpret_cast<const unsigned char *>(content);
            *cursor != '\0'; ++cursor) {
        if (*cursor == ',' || *cursor == '{' || *cursor == '}'
                || *cursor < 0x20 || *cursor == 0x7f) return false;
    }
    char frame[SERIAL_FRAME_BYTES];
    const int length = snprintf(frame, sizeof(frame), "{EVT,%s,%s,%s}",
                                commandType, displayType, content);
    if (length < 0 || static_cast<size_t>(length) >= sizeof(frame)) return false;

    // 状态机有 50 ms 轮询，重复正文只发一次，避免日志和上位机被刷屏。
    static char previousFrame[SERIAL_FRAME_BYTES] = {0};
    if (strcmp(previousFrame, frame) == 0) return true;
    Serial.println(frame);
    memcpy(previousFrame, frame, static_cast<size_t>(length) + 1);
    return true;
}

// 主控请求机载电脑切换视觉功能；这些是单次请求，不代表视觉已启动或已对齐。
// 顺序与 Document/serial_protocol.md 中的五种模式一致。
enum class VisionStartMode : uint8_t {
    DISC,
    DISC_MATERIAL,
    WORK_AREA,
    WORK_AREA_LOADED,
    CORNER
};

static void requestVisionFunction(VisionStartMode mode, bool start) {
    const char *name = nullptr;
    switch (mode) {
        case VisionStartMode::DISC:
            name = "DISC";
            break;
        case VisionStartMode::DISC_MATERIAL:
            name = "DISC_MATERIAL";
            break;
        case VisionStartMode::WORK_AREA:
            name = "WORK_AREA";
            break;
        case VisionStartMode::WORK_AREA_LOADED:
            name = "WORK_AREA_LOADED";
            break;
        case VisionStartMode::CORNER:
            name = "CORNER";
            break;
    }
    if (name) Serial.printf("{EVT,VISION,%s,%s}\n", start ? "START_REQUEST" : "STOP_REQUEST", name);
}

void requestVisionStart(VisionStartMode mode) { requestVisionFunction(mode, true); }

// 只请求机载电脑结束指定功能，不表示相机已经停止。
void requestVisionStop(VisionStartMode mode) { requestVisionFunction(mode, false); }

// 超时兜底: 任一环节卡死则放弃本轮, 回启停区
void vHomeTimerCallback(TimerHandle_t xTimer) {
    if (currentState != STATE_DONE) {
        Serial.println("[TIMER] Timeout! Abort round, return home");
        portENTER_CRITICAL(&nodePathMux);
        nodePathState = NodePathState::FAILED;
        portEXIT_CRITICAL(&nodePathMux);
        Serial.println("{EVT,NAV,ROUTE_FAILED,TIMEOUT}");
        if (xTask_MainStateMachine_Handle != NULL)
            vTaskSuspend(xTask_MainStateMachine_Handle);
        // [TODO] 收缩机械臂到安全姿态 + 回启停区
        currentState = STATE_RETURN_HOME;
    }
}


// @brief 等待扫码消息队列 (供主状态机在各环节调用)
// @param out       输出缓冲区
// @param len       缓冲区长度
// @param timeoutMs 阻塞超时(ms), 0=非阻塞
// @return true 拿到一帧扫码字符串
static bool waitScannerCode(char *out, uint32_t len, uint32_t timeoutMs) {
    if (Scanner_WaitCode(out, len, timeoutMs)) {
        Serial.printf("[SCANNER] recv: %s\n", out);
        return true;
    }
    return false;
}

// ================= 主状态机 =================
void Task_MainStateMachine(void *pvParameters) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    bool discRouteCompleted = false; // 每次创建主任务时，第一轮路径重新等待执行。
    bool discMaterialRequested = false;
    bool coarse1RouteCompleted = false;
    bool coarse1VisionRequested = false;

    while (1) {
        switch (currentState) {


        case STATE_WAIT_START: // 等待开始区域
            Serial.println("TASK start");
            InitArm_start();// 初始化机械臂
            updateDisplay("DISPLAY", "DEBUG", "WAIT start_zone");
            while (currentStartZone == START_ZONE_UNKNOWN) vTaskDelay(100 / portTICK_PERIOD_MS);
            switch (currentStartZone)
            {
            case START_ZONE_1:
                updateDisplay("DISPLAY", "DEBUG", "start_zone: 1");
                // 右侧启停区车头朝左，即世界坐标 -X 方向。
                currentPose = {2250, 150, 180};
                break;
            case START_ZONE_2:
                updateDisplay("DISPLAY", "DEBUG", "start_zone: 2");
                // 左侧启停区车头朝右，即世界坐标 +X 方向。
                currentPose = {150, 150, 0};
                break;

            default:
                updateDisplay("DISPLAY", "DEBUG", "ERR:start_zone: unknown");
                break;
            }
            // Release 开局自动执行一次。动作未标定或执行失败时返回 ERR，
            // 仍停留在开局等待阶段，便于通过串口修正后再次手动调用。
            runLidarPoseAction();
            updateDisplay("DISPLAY", "DEBUG", "WAIT START");
            while (!enableRun) vTaskDelay(100 / portTICK_PERIOD_MS);
            // 启动总超时兜底(如 300s 内未回启停区)
            if (xHomeTimer != NULL) xTimerStart(xHomeTimer, 0);
            currentState = STATE_READ_TASK;
            break;

        case STATE_READ_TASK: { // 读取任务码
            updateDisplay("DISPLAY", "DEBUG", "READ TASK");
            InitArm_start();
            // 1. 每段最多估算 1300 mm；首次尝试 + 最多 3 次反向重试，共 4 段。
            constexpr float SCAN_MAX_DISTANCE_MM = 1300.0f;
            constexpr float SCAN_SPEED_RPM = 30.0f;
            constexpr uint8_t SCAN_MAX_RETRIES = 3;
            // 与 chassis.cpp 的 16 细分配置一致：3200 脉冲/圈。
            // MovePose 的 speed 直接传给电机，实际单位是 RPM，不是 mm/s。
            const float scanSpeedMmPerSecond = SCAN_SPEED_RPM * 3200.0f / 60.0f / X_PULSE;
            if ((currentStartZone != START_ZONE_1 && currentStartZone != START_ZONE_2)
                    || !isfinite(scanSpeedMmPerSecond) || scanSpeedMmPerSecond <= 0.0f) {
                MovePose(0, 0, true);
                Serial.println("[SCANNER] ERR: invalid start zone or pulse calibration");
                updateDisplay("DISPLAY", "DEBUG", "TASK ERR");
                if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                currentState = STATE_SCAN_FAILED;
                break;
            }
            int scanDirection = (currentStartZone == START_ZONE_1) ? 0 : 1;
            uint8_t scanRetries = 0;
            taskReceived = false;
            currentTask = {};
            // 从下发指令前计时，按匀速保守估算，不额外补偿启动加速距离。
            uint32_t scanLegStartMs = millis();
            MovePose(scanDirection, SCAN_SPEED_RPM, false);
            while (!taskReceived) {
                // 2. 移动期间持续非阻塞读取；收到有效任务码立即退出并停车。
                char scanCode[SCANNER_BUF_LEN];
                if (waitScannerCode(scanCode, sizeof(scanCode), 0)) {
                    currentTask = parseTaskCode(scanCode);
                    taskReceived = currentTask.valid;
                    Serial.printf("[SCANNER] task code %s\n", currentTask.valid ? "OK" : "ERR");
                }
                if (taskReceived) break;

                const float estimatedDistanceMm = scanSpeedMmPerSecond
                    * static_cast<float>(millis() - scanLegStartMs) / 1000.0f;
                if (estimatedDistanceMm >= SCAN_MAX_DISTANCE_MM) {
                    MovePose(scanDirection, 0, true);
                    if (scanRetries >= SCAN_MAX_RETRIES) {
                        Serial.println("[SCANNER] ERR: retries exhausted (3/3)");
                        break;
                    }
                    // 3. 停稳后反向，在同一段扫码区域往返；每次重新估算行程。
                    vTaskDelay(pdMS_TO_TICKS(100));
                    ++scanRetries;
                    scanDirection = 1 - scanDirection; // 0=前进，1=后退
                    Serial.printf("[SCANNER] retry %u/%u, direction=%d\n",
                                  static_cast<unsigned>(scanRetries),
                                  static_cast<unsigned>(SCAN_MAX_RETRIES), scanDirection);
                    scanLegStartMs = millis();
                    MovePose(scanDirection, SCAN_SPEED_RPM, false);
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            MovePose(scanDirection, 0, true); // 成功或重试耗尽都停车。
            if (!taskReceived) {
                updateDisplay("DISPLAY", "DEBUG", "TASK ERR");
                if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                currentState = STATE_SCAN_FAILED;
                break;
            }
            // 按已解析的 4 组三位数重新组装，避免二维码尾部杂字符进入显示帧。
            char taskCodeText[16];
            snprintf(taskCodeText, sizeof(taskCodeText), "%d%d%d+%d%d%d+%d%d%d+%d%d%d",
                     currentTask.round1_colors[0], currentTask.round1_colors[1], currentTask.round1_colors[2],
                     currentTask.round1_pos[0], currentTask.round1_pos[1], currentTask.round1_pos[2],
                     currentTask.round2_colors[0], currentTask.round2_colors[1], currentTask.round2_colors[2],
                     currentTask.round2_pos[0], currentTask.round2_pos[1], currentTask.round2_pos[2]);
            updateDisplay("DISPLAY", "TASK_CODE", taskCodeText);
            updateDisplay("DISPLAY", "DEBUG", "TASK OK");
            xQueueReset(xVisualTaskQueue); // 清残留信号
            currentState = STATE_GRAB_ROUND1;
            break;
        }

        case STATE_SCAN_FAILED: // 扫码失败
            Serial.println("SCAN_FAILED");
        case STATE_ROUTE_FAILED: // 路径失败，同样保持故障状态
            Serial.println("ROUTE_FAILED");
        case STATE_ALIGN_FAILED:// 定位失败
            Serial.println("ALIGN_FAILED");
        case STATE_TRANSFER_FAILED:
            // 保持故障状态，避免下一轮状态机再次启动扫码或执行残留路径。
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;

        case STATE_GRAB_ROUND1: { // 抓取第一轮物料
            // 此状态会轮询抓取进度，路径成功后只执行一次，避免重复行驶。
            // 原料区为旋转电动转盘(6-10s/圈, 转向随机, 物料120°分布)
            // 机载电脑识别目标颜色物料 -> 发 "color:N" 引导抓取
            // [TODO] 转盘同步/跟随, 逐次抓取, 每次抓完放上载物台
            // 规则: 每次抓1个; 物料必须放到机器人上才能抓下一个
            //       不允许手爪夹持运送
            updateDisplay("DISPLAY", "DEBUG", discMaterialRequested ? "WAIT DISC MATERIAL" : "GRAB R1");
            //调取接口获取路径, 并移动到目标位置
            if (!discRouteCompleted) {
                if (!requestAndMoveNodePath(SCAN_AREA_NODE, DISC_AREA_NODE)) {
                    updateDisplay("DISPLAY", "DEBUG", "ROUTE ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);// 路径失败后, 停止定时器
                    currentState = STATE_ROUTE_FAILED;
                    break; // 失败时跳过下面的用户代码
                }
                discRouteCompleted = true;
                roundProgress = 0;
                // 先清除旧反馈并开启单次闭环，再通知机载电脑发送 DISC 数据。
                setAlignmentEnabled(true, false, true);
                // 路径执行完后请求圆盘定位视觉；上位机应切到圆盘定位流程。
                requestVisionStart(VisionStartMode::DISC);

            }
            // 走到这里时路径已执行成功，可继续轮询后续任务进度。
            //视觉对齐圆盘
            if (!discMaterialRequested) {
                const auto alignmentState = getAutoAlignmentState();
                if (alignmentState == AutoAlignmentState::WAITING) break;
                setAlignmentEnabled(false); // 清空残留反馈并保持停车。
                requestVisionStop(VisionStartMode::DISC);
                if (alignmentState != AutoAlignmentState::DONE) {
                    updateDisplay("DISPLAY", "DEBUG", "DISC ALIGN ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_ALIGN_FAILED;
                    break;
                }
                xQueueReset(xVisualTaskQueue); // 丢弃定位阶段残留的颜色结果。
                requestVisionStart(VisionStartMode::DISC_MATERIAL);
                discMaterialRequested = true;
                updateDisplay("DISPLAY", "DEBUG", "WAIT DISC MATERIAL");
                // TODO: 接入旋转圆盘的目标颜色/取料时机反馈后逐件 MoveDiscToCargo。
                // 不可直接 LoadRoundFromDisc：该函数不等待视觉，也不跟随转盘。
            }

            if (roundProgress >= 3 || true) {//先跳过抓取物料

                roundProgress = 0;
                currentState = STATE_PLACE_COARSE1;
            }

            vTaskDelay(pdMS_TO_TICKS(5000));
            break;
        }

        // 第一批物料放置到粗加工区：导航至粗加工区 → 视觉对齐 → 放置物料 → 转入暂存区放置
        case STATE_PLACE_COARSE1:
        {
            // 按 round1_pos 顺序放置到粗加工区对应圆环
            // 圆环评分: 1环15分 2环10分 3环7分 ... 越中心分越高
            // 从粗加工区取回3个, 按 round1_pos 放到暂存区
            // 在粗加工区取回：
            updateDisplay("DISPLAY", "DEBUG",
                          coarse1RouteCompleted ? "ALIGN COARSE1" : "GO COARSE1");

            if (!coarse1RouteCompleted) {
                // 离开圆盘前关闭物料识别，避免导航途中继续产生颜色结果。
                requestVisionStop(VisionStartMode::DISC_MATERIAL);
                xQueueReset(xVisualTaskQueue);
                if (!requestAndMoveNodePath(DISC_AREA_NODE, COARSE_AREA_NODE)) {
                    updateDisplay("DISPLAY", "DEBUG", "ROUTE ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_ROUTE_FAILED;
                    break;
                }
                coarse1RouteCompleted = true;
                roundProgress = 0;
                // 到达粗加工区后开启一次自动 PID 对齐；定位视觉只负责空工位。
                setAlignmentEnabled(true, false, true);
                requestVisionStart(VisionStartMode::WORK_AREA);
                coarse1VisionRequested = true;
                break;
            }

            if (coarse1VisionRequested) {
                const auto alignmentState = getAutoAlignmentState();
                if (alignmentState == AutoAlignmentState::WAITING) break;
                setAlignmentEnabled(false);
                requestVisionStop(VisionStartMode::WORK_AREA);
                coarse1VisionRequested = false;
                if (alignmentState != AutoAlignmentState::DONE) {
                    updateDisplay("DISPLAY", "DEBUG", "COARSE ALIGN ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_ALIGN_FAILED;
                    break;
                }
            }

            updateDisplay("DISPLAY", "DEBUG", "PLACE C1");
            // 批量接口会按任务码将载物台 1~3 全部放到第一层；返回 true
            // 只说明预设动作均已执行，不代表传感器确认物料实际放置成功。
            if (!PlaceTaskCargoToWorkArea(currentTask.round1_pos, 1)) {
                updateDisplay("DISPLAY", "DEBUG", "PLACE C1 ERR");
                if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                currentState = STATE_TRANSFER_FAILED;
                break;
            }
            roundProgress = 0;
            requestVisionStart(VisionStartMode::WORK_AREA_LOADED);
            currentState = STATE_PLACE_TEMP1;
            break;
        }

        case STATE_PLACE_TEMP1:

            // 到达暂存区后复用相同工位位姿：
            updateDisplay("DISPLAY", "DEBUG", "PLACE T1");
            if (roundProgress >= 3) { roundProgress = 0; currentState = STATE_GRAB_ROUND2; }
            break;

        case STATE_GRAB_ROUND2:
            // 同 round1, 抓第二批
            updateDisplay("DISPLAY", "DEBUG", "GRAB R2");
            if (roundProgress >= 3) { roundProgress = 0; currentState = STATE_PLACE_COARSE2; }
            break;

        case STATE_PLACE_COARSE2:
            // 第二批放粗加工区
            updateDisplay("DISPLAY", "DEBUG", "PLACE C2");
            if (roundProgress >= 3) { roundProgress = 0; currentState = STATE_STACK_TEMP2; }
            break;

        case STATE_STACK_TEMP2:
            // 第二批在暂存区码垛到第一批上方(颜色一致, 需平稳放置)
            updateDisplay("DISPLAY", "DEBUG", "STACK T2");
            if (roundProgress >= 3) { roundProgress = 0; currentState = STATE_RETURN_HOME; }
            break;

        case STATE_RETURN_HOME:
            updateDisplay("DISPLAY", "DEBUG", "GO HOME");
            // [TODO] 回到启停区
            // 回家路径执行结束后调用 requestVisionStart(VisionStartMode::CORNER)，
            // 等机载电脑确认角点定位结果后再进入 STATE_DONE；当前回家导航尚未实现。
            currentState = STATE_DONE;
            break;

        case STATE_DONE:
            updateDisplay("DISPLAY", "DEBUG", "DONE");
            if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
            Emm_V5_En_Control_all(false);
            vTaskDelay(100000000 / portTICK_PERIOD_MS); // 保持
            break;
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// 底盘、升降和伸缩回读理想值；两个舵机通过总线现场读取，不触发运动。
// Debug 和 Release 共用此接口；单位顺序为 mm/mm/度/mm/mm/度/度。
static void sendPoseQueryReply() {
    const RobotPose robot = currentPose;
    ArmPose arm = currentArm;
    // 与 MoveArm 一致：2 号舵盘读取真实多圈角度，1 号夹爪读取单圈角度。
    // 读取失败不以命令目标/上次角度冒充实测值，也不改写运动目标 currentArm。
    if (!Servo_QueryAngleMTurn(2, arm.turret_angle) || !isfinite(arm.turret_angle)) {
        Serial.println("{RSP,POSE,GET,ERR,SERVO_READ,2}");
        return;
    }
    if (!Servo_QueryAngle(1, arm.pawl_angle) || !isfinite(arm.pawl_angle)) {
        Serial.println("{RSP,POSE,GET,ERR,SERVO_READ,1}");
        return;
    }
    Serial.printf("{RSP,POSE,GET,OK,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f}\n",
                  robot.x, robot.y, robot.theta, arm.high, arm.length,
                  arm.turret_angle, arm.pawl_angle);
}

#include "serial_commands.inc"

// 开机读取一个舵机的角度，并同步到主控维护的机械臂位姿。
static void readStartupServoAngle(uint8_t servoId, float &storedAngle) {
    Serial.printf("[Servo] 开机角度读取开始: ID=%u\n",
                  static_cast<unsigned>(servoId));
    float measuredAngle = 0.0f;
    bool readOk = false;
    for (uint8_t attempt = 0; attempt < 3 && !readOk; ++attempt) {
        if (attempt != 0) vTaskDelay(pdMS_TO_TICKS(30));
        readOk = servoId == 2
            ? Servo_QueryAngleMTurn(servoId, measuredAngle, 100)
            : Servo_QueryAngle(servoId, measuredAngle, 100);
    }
    if (!readOk || !isfinite(measuredAngle)) {
        if (servoId == 2) DisableTurretMotionUntilRestart("INITIAL_MULTI_READ");
        Serial.printf("[Servo] 开机角度读取失败: ID=%u, 保留原值=%.1f度, 未进行归一化\n",
                      static_cast<unsigned>(servoId), storedAngle);
        return; // 读取失败时保留原值，不把失败当作测得 0 度。
    }

    if (servoId == 2) {
        storedAngle = measuredAngle;
        if (storedAngle < 0.0f || storedAngle > 180.0f) {
            // 用户约定：每次上电时转台物理上都停在 0~180 度。
            // 只在单圈实测也符合该区间、且与多圈角度的余数一致时重置圈数。
            float singleAngle = 0.0f;
            float wrappedAngle = fmodf(measuredAngle, 360.0f);
            if (wrappedAngle > 180.0f) wrappedAngle -= 360.0f;
            if (wrappedAngle < -180.0f) wrappedAngle += 360.0f;
            bool singleReadOk = false;
            for (uint8_t attempt = 0; attempt < 3 && !singleReadOk; ++attempt) {
                if (attempt != 0) vTaskDelay(pdMS_TO_TICKS(30));
                singleReadOk = Servo_QueryAngle(2, singleAngle, 100);
            }
            if (!singleReadOk || !isfinite(singleAngle)
                    || singleAngle < 0.0f || singleAngle > 180.0f
                    || fabsf(singleAngle - wrappedAngle) > 3.0f) {
                DisableTurretMotionUntilRestart("SINGLE_READ_OR_MISMATCH");
                Serial.printf("[Servo] ERR: 转台单圈校验失败：多圈=%.1f度，余角=%.1f度，单圈读取%s，单圈=%.1f度；未重置圈数\n",
                              measuredAngle, wrappedAngle, singleReadOk ? "成功" : "失败", singleAngle);
                return;
            }
            float resetAngle = 0.0f;
            if (!Servo_ResetTurnCount(2, resetAngle) || !isfinite(resetAngle)
                    || resetAngle < 0.0f || resetAngle > 180.0f
                    || fabsf(resetAngle - singleAngle) > 3.0f) {
                DisableTurretMotionUntilRestart("RESET_VERIFY");
                Serial.printf("[Servo] ERR: 转台圈数重置未验证成功；重置前=%.1f度，单圈=%.1f度，回读=%.1f度\n",
                              measuredAngle, singleAngle, resetAngle);
                return;
            }
            storedAngle = resetAngle;
            Serial.printf("[Servo] 转台圈数已重置：%.1f -> %.1f度；未命令转台旋转\n",
                          measuredAngle, storedAngle);
        } else {
            Serial.printf("[Servo] 转台实测多圈角度=%.1f度（线缆允许范围 -180~360 度），无需重置\n",
                          storedAngle);
        }
        return;
    }

    float normalizedAngle = measuredAngle;
    // 1. 0~360 度（含端点）保持不变。
    // 2. 超出范围时映射到 [0, 360)，例如 -55 -> 305、725 -> 5。
    // 3. 只归一化主控中的角度数值，不命令舵机旋转或修改舵机零点。
    const bool needsNormalization = normalizedAngle < 0.0f || normalizedAngle > 360.0f;
    if (needsNormalization) {
        normalizedAngle = fmodf(normalizedAngle, 360.0f);
        if (normalizedAngle < 0.0f) normalizedAngle += 360.0f;
        if (normalizedAngle == 0.0f) normalizedAngle = 0.0f; // 消除负零。
    }
    storedAngle = normalizedAngle;
    Serial.printf("[Servo] 开机角度读取成功: ID=%u, 原始角度=%.1f度, %s, 保存角度=%.1f度（仅更新数值）\n",
                  static_cast<unsigned>(servoId), measuredAngle,
                  needsNormalization ? "超出0~360度，已归一化" : "在0~360度内，无需归一化",
                  storedAngle);
}

// ================= setup  =================
void setup() {
    const bool ledPwmReady = LedPwm_Init();
    Serial.begin(115200);
    if (!ledPwmReady) Serial.println("[LED] PWM initialization failed");
    Serial.printf("version: %s\n", VERSION);
    Servo_Init();    // 总线舵机初始化 (默认 Serial2: RX=16, TX=15, 115200bps)
    Emm_V5_Init();   // 电机初始化
    Scanner_Init();  // 扫码模块初始化 (软串口 RX=IO4, 9600bps)
    // [TODO] 若有传感器/定位硬件, 在此初始化
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(10);

    currentPose = {0, 0, 0};   // [TODO] 初始位姿按实际
    currentArm = {0, 0, 0, 0}; // 升降/伸缩位置仍待标定，舵机角度由下方实读更新。
    // 与 MoveArm 的实际 ID 对应：2 号=转台，1 号=夹爪。
    // 在串口任务启动前读取，避免多个任务同时访问舵机总线。
    // 给上电中的舵机留出启动时间；总线已初始化但舵机不一定立即应答。
    vTaskDelay(pdMS_TO_TICKS(100));
    readStartupServoAngle(2, currentArm.turret_angle);
    readStartupServoAngle(1, currentArm.pawl_angle);

    // 注意: 原 initLidar() 已移除, 雷达/定位方案待定
     init_ota_service("null", "1234567899", OTA_HOSTNAME);//调试使用，正式比赛时注释掉
    //init_ota_service("longggg", "asdfghjkl", OTA_HOSTNAME);//调试使用，正式比赛时注释掉
    leds[0] = CRGB::Red; FastLED.show();

    // 机载电脑指令队列(深度10)
    xVisualTaskQueue = xQueueCreate(10, sizeof(VisualCmd_t));
    xLidarPoseMutex = xSemaphoreCreateMutex();
    // 视觉 PID 对齐只保留 20 Hz 连续反馈中的最新一帧。
    xAlignmentQueue = xQueueCreate(1, sizeof(VisualAlignmentFrame_t));
    xAlignmentMotionMutex = xSemaphoreCreateMutex();
    if (xAlignmentQueue != NULL && xAlignmentMotionMutex != NULL) {
        xTaskCreate(
            Task_VisualAlignment, "Task_VisualAlignment",
            8192, NULL, 7, NULL
        );
    } else {
        Serial.println("[Align] ERR: failed to create alignment queue or mutex");
    }

    vTaskDelay(pdMS_TO_TICKS(3000));
    // 上电固定进入 Debug；串口发送 {CMD,SYS,RELEASE} 切换到正式模式。
    {
        Serial.println("Debug mode");
        leds[0] = CRGB::Yellow; FastLED.show();
        // Debug 模式默认直接启用视觉闭环对齐，仍可通过
        // {CMD,VISION,ALIGN_STOP}/{CMD,VISION,ALIGN_START} 可停止或重启。
        setAlignmentEnabled(true);
        xTaskCreate(Task_Serial_CMD, "Task_Serial_CMD", 16384, NULL, 5, NULL);
    }
}

void loop() {
    vTaskDelay(10000 / portTICK_PERIOD_MS);
}
