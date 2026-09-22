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

// ================= 基础配置 =================
#define MODE_key 0             // 开机按键(长按进调试模式)
#define LED_PIN 48
#define NUM_LEDS 1
#define OTA_HOSTNAME "smartcarrier_ESP32S3"
#define VERSION "0.1.4-framework"

uint32_t ALIGN_PID_MAX_SPEED_RPM = 20;  // 移动速度单位为转/分
constexpr uint32_t ALIGN_FEEDBACK_TIMEOUT_MS = 300;
constexpr uint32_t ALIGN_LOG_INTERVAL_MS = 500;

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
    TaskCode tc;
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
// 所有机器可读帧统一使用 {魔术字:参数}\n，详见 Document/serial_protocol.md。

// ================= 业务状态 =================
enum RobotState {
    STATE_WAIT_START,    // 待机,等一键启动
    STATE_READ_TASK,     // 读取任务码(二维码板 / 机载电脑)
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

volatile bool moveNodePathFlag = false;
uint8_t nodePathBuffer[16] = {0};
size_t nodePathLen = 0;

/**
 * @brief 解析节点路径帧，例如 "{way:2-3-6}"。
 *
 * 仅接受 1~9 号节点、短横线分隔和完整花括号，避免 atoi 将非法内容
 * 静默转换成 0。节点是否相邻由 MoveNodePath 在运动前统一检查。
 */
bool parseNodePathCommand(const char *command, uint8_t *path, size_t capacity, size_t &pathLength) {
    pathLength = 0;
    constexpr char WAY_PREFIX[] = "{way:";
    if (command == nullptr || path == nullptr || capacity < 2
            || strncmp(command, WAY_PREFIX, sizeof(WAY_PREFIX) - 1) != 0) {
        return false;
    }

    const char *cursor = command + sizeof(WAY_PREFIX) - 1;
    for (;;) {
        if (*cursor < '1' || *cursor > '9' || pathLength >= capacity) {
            pathLength = 0;
            return false;
        }
        path[pathLength++] = static_cast<uint8_t>(*cursor - '0');
        ++cursor;

        if (*cursor == '}') {
            if (cursor[1] == '\0' && pathLength >= 2) {
                return true;
            }
            pathLength = 0;
            return false;
        }
        if (*cursor != '-') {
            pathLength = 0;
            return false;
        }
        ++cursor;
    }
}

/**
 * @brief 解析视觉对齐帧："{ALIGN:angle,x,y}"。
 *
 * 三个字段依次为水平校正角（deg）和 2 号圆盘 X/Y 视觉误差。
 * 必须恰好包含三个完整的有限浮点数，不接受缺字段或尾随字符。
 */
int parseVisualAlignmentFrame(const char *frame, float &angleDeg, float &x, float &y) {
    if (frame == nullptr) {
        return 0;
    }

    constexpr char ALIGN_PREFIX[] = "{ALIGN:";
    if (strncmp(frame, ALIGN_PREFIX, sizeof(ALIGN_PREFIX) - 1) != 0) {
        return 0;
    }
    const char *cursor = frame + sizeof(ALIGN_PREFIX) - 1;

    char *end = nullptr;
    angleDeg = strtof(cursor, &end);
    if (end == cursor || *end != ',') return 0;
    cursor = end + 1;

    x = strtof(cursor, &end);
    if (end == cursor || *end != ',') return 0;
    cursor = end + 1;

    y = strtof(cursor, &end);
    if (end == cursor || *end != '}' || end[1] != '\0') return 0;

    return isfinite(x) && isfinite(y) && isfinite(angleDeg) ? 1 : 0;
}

typedef struct {
    float angleDeg;
    float visualX;
    float visualY;
} VisualAlignmentFrame_t;

QueueHandle_t xAlignmentQueue = NULL;
SemaphoreHandle_t xAlignmentMotionMutex = NULL;
volatile bool alignmentEnabled = false;

// 参数提交与 PID 更新共用锁，避免一个控制周期读取到半套参数。
static bool handleParameterFrame(const char *frame, bool debugMode) {
    if (strncmp(frame, "{CFG:", 5) != 0) return false;
    // 完整目录的串口输出耗时较长；对齐期间拒绝读取，避免阻塞视觉反馈接收。
    if (alignmentEnabled) {
        const char *comma = strchr(frame, ',');
        const unsigned long id = comma ? strtoul(comma + 1, nullptr, 10) : 0;
        Serial.printf("{CFG:ERR,%lu,BUSY_OR_MODE}\n", id);
        return true;
    }
    if (xAlignmentMotionMutex != NULL &&
            xSemaphoreTake(xAlignmentMotionMutex, portMAX_DELAY) == pdTRUE) {
        HandleRuntimeParameters(frame, debugMode && !alignmentEnabled);
        xSemaphoreGive(xAlignmentMotionMutex);
    } else {
        Serial.println("{CFG:ERR,0,LOCK}");
    }
    return true;
}

static bool handleAlignmentControlFrame(const char *frame) {
    bool enable = false;
    if (strcmp(frame, "{ALIGN:START}") == 0) {
        enable = true;
    } else if (strcmp(frame, "{ALIGN:STOP}") != 0) {
        return false;
    }

    alignmentEnabled = false;
    if (xAlignmentQueue != NULL) {
        xQueueReset(xAlignmentQueue);
    }
    if (xAlignmentMotionMutex != NULL
            && xSemaphoreTake(xAlignmentMotionMutex, portMAX_DELAY) == pdTRUE) {
        OmniMove(0.0f, 0.0f, 0.0f, 0);
        ResetDiscAlignmentPid();
        xSemaphoreGive(xAlignmentMotionMutex);
    }

    alignmentEnabled = enable;
    Serial.println(enable ? "{ALIGN:STARTED}" : "{ALIGN:STOPPED}");
    return true;
}

static bool handleVisualAlignmentFrame(const char *frame) {// 处理视觉对齐帧
    float visualX = 0.0f;
    float visualY = 0.0f;
    float angleDeg = 0.0f;
    if (!parseVisualAlignmentFrame(frame, angleDeg, visualX, visualY)) {
        return false;
    }

    VisualAlignmentFrame_t alignment = {angleDeg, visualX, visualY};
    if (alignmentEnabled && xAlignmentQueue != NULL) {
        // 队列长度为 1；视觉以 20 Hz 连续发送时始终只保留最新一帧。
        xQueueOverwrite(xAlignmentQueue, &alignment);
    }
    return true;
}

/** @brief 严格解析启停区帧 "{StartZone:1}" / "{StartZone:2}"。 */
static bool parseStartZoneFrame(const char *frame, StartZone &zone) {
    constexpr char START_ZONE_PREFIX[] = "{StartZone:";
    if (frame == nullptr
            || strncmp(frame, START_ZONE_PREFIX,
                       sizeof(START_ZONE_PREFIX) - 1) != 0) {
        return false;
    }

    const char *value = frame + sizeof(START_ZONE_PREFIX) - 1;
    if (value[1] != '}' || value[2] != '\0') {
        return false;
    }
    if (value[0] == '1') {
        zone = START_ZONE_1;
        return true;
    }
    if (value[0] == '2') {
        zone = START_ZONE_2;
        return true;
    }
    return false;
}

// ================= 任务/队列句柄 =================
TaskHandle_t xTask_MainStateMachine_Handle = NULL;
QueueHandle_t xVisualTaskQueue = NULL;      // 机载电脑指令队列
TimerHandle_t xHomeTimer = NULL;            // 总超时兜底(回启停区)

// 队列元素: 机载电脑指令
typedef struct {
    char cmd[20];
    float param1, param2, param3;
} VisualCmd_t;

// ================= 函数声明 =================
void Task_MainStateMachine(void *pvParameters);// 主状态机任务
void Task_Serial_CMD(void *pvParameters);// 串口指令任务
void Task_Debug_CMD(void *pvParameters);// 调试指令任务
void Task_VisualAlignment(void *pvParameters);// 视觉对齐任务
void vHomeTimerCallback(TimerHandle_t xTimer);// 总超时兜底(回启停区)

// ================= 视觉闭环对齐任务 =================
void Task_VisualAlignment(void *pvParameters) {
    VisualAlignmentFrame_t alignment;
    bool alignedReported = false;
    bool feedbackActive = false;
    uint32_t lastFeedbackMs = 0;
    uint32_t lastLogMs = 0;

    for (;;) {
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
                    OmniMove(0.0f, 0.0f, 0.0f, 0);
                    ResetDiscAlignmentPid();
                    xSemaphoreGive(xAlignmentMotionMutex);
                }
                Serial.println("{ALIGN:ERR,TIMEOUT}");
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
            if (alignmentEnabled) {
                aligned = AlignToDiscContinuous(
                    alignment.angleDeg, alignment.visualX, alignment.visualY,
                    dtSeconds, ALIGN_PID_MAX_SPEED_RPM
                );
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
                Serial.println("{ALIGN:OK}");
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

// 任务码显示装置 [TODO: 按硬件接入]
void updateDisplay(const char* text) {
    // 例: 驱动 LED点阵 / OLED / TFT 显示任务码与完成统计
    // 硬性要求: 字高>=12mm, 醒目位置, 亮光显示, 不被遮挡
    // 本函数目前仅串口打印, 待接真实显示硬件
    Serial.print("[DISPLAY] ");
    Serial.println(text);
}

// 超时兜底: 任一环节卡死则放弃本轮, 回启停区
void vHomeTimerCallback(TimerHandle_t xTimer) {
    if (currentState != STATE_DONE) {
        Serial.println("[TIMER] Timeout! Abort round, return home");
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

    while (1) {
        switch (currentState) {

        case STATE_WAIT_START:
            InitArm();// 初始化机械臂
            updateDisplay("WAIT start_zone");
            while (currentStartZone == START_ZONE_UNKNOWN) vTaskDelay(100 / portTICK_PERIOD_MS);
            switch (currentStartZone)
            {
            case START_ZONE_1:
                updateDisplay("start_zone: 1");
                currentPose = {2250, 150, 0};
                break;
            case START_ZONE_2:
                updateDisplay("start_zone: 2");
                currentPose = {150, 150, 0};
                break;
            
            default:
                updateDisplay("ERR:start_zone: unknown");
                break;
            }
            updateDisplay("WAIT START");
            while (!enableRun) vTaskDelay(100 / portTICK_PERIOD_MS);
            // 启动总超时兜底(如 300s 内未回启停区)
            if (xHomeTimer != NULL) xTimerStart(xHomeTimer, 0);
            currentState = STATE_READ_TASK;
            break;

        case STATE_READ_TASK:
            updateDisplay("READ TASK");
            // 方案: 机器人走到二维码板,扫码枪读二维码/条码, 从扫码消息队列取任务码

            if (currentStartZone == START_ZONE_1)
            {
                // 向左移动到二维码板位姿
                MovePose(2, 100, false);
            }
            else if (currentStartZone == START_ZONE_2)
            {
                // 向右移动到二维码板位姿
                MovePose(3, 100, false);
            }
            else {
                Serial.println("ERR:start_zone: unknown");
            }
            
            while (!taskReceived) {
                // 非阻塞检查扫码消息队列(伪函数)
                char scanCode[SCANNER_BUF_LEN];
                if (waitScannerCode(scanCode, sizeof(scanCode), 0)) {
                    currentTask = parseTaskCode(scanCode);
                    taskReceived = currentTask.valid;
                    Serial.printf("[SCANNER] task code %s\n", currentTask.valid ? "OK" : "ERR");
                }
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            MovePose(0, 100, true);//成功扫码后停下
            updateDisplay(currentTask.valid ? "TASK OK" : "TASK ERR");
            xQueueReset(xVisualTaskQueue); // 清残留信号
            currentState = STATE_GRAB_ROUND1;
            break;

        case STATE_GRAB_ROUND1:
            // 原料区为旋转电动转盘(6-10s/圈, 转向随机, 物料120°分布)
            // 机载电脑识别目标颜色物料 -> 发 "color:N" 引导抓取
            // [TODO] 转盘同步/跟随, 逐次抓取, 每次抓完放上载物台
            // 规则: 每次抓1个; 物料必须放到机器人上才能抓下一个
            //       不允许手爪夹持运送
            updateDisplay("GRAB R1");// 第一批
            //调取接口获取路径, 并移动到目标位置
            Serial.println("{way:2-6?}");
            // 到达圆盘并完成视觉对准后调用：
            // LoadRoundFromDisc(currentTask.round1_colors);

            vTaskDelay(10000 / portTICK_PERIOD_MS);//暂时阻塞10s

            if (roundProgress >= 3) { roundProgress = 0; currentState = STATE_PLACE_COARSE1; }//抓取3个物料后, 放置到粗加工区对应圆环
            break;

        case STATE_PLACE_COARSE1:
            // 按 round1_pos 顺序放置到粗加工区对应圆环
            // 圆环评分: 1环15分 2环10分 3环7分 ... 越中心分越高
            // 到达粗加工区并完成停车定位后调用：
            // PlaceRoundToWorkArea(currentTask.round1_pos);
            updateDisplay("PLACE C1");
            if (roundProgress >= 3) { roundProgress = 0; currentState = STATE_PLACE_TEMP1; }
            break;

        case STATE_PLACE_TEMP1:
            // 从粗加工区取回3个, 按 round1_pos 放到暂存区
            // 在粗加工区取回：
            // RetrieveRoundToCargo(currentTask.round1_colors, currentTask.round1_pos);
            // 到达暂存区后复用相同工位位姿：
            // PlaceRoundToWorkArea(currentTask.round1_pos);
            updateDisplay("PLACE T1");
            if (roundProgress >= 3) { roundProgress = 0; currentState = STATE_GRAB_ROUND2; }
            break;

        case STATE_GRAB_ROUND2:
            // 同 round1, 抓第二批
            // LoadRoundFromDisc(currentTask.round2_colors);
            updateDisplay("GRAB R2");
            if (roundProgress >= 3) { roundProgress = 0; currentState = STATE_PLACE_COARSE2; }
            break;

        case STATE_PLACE_COARSE2:
            // 第二批放粗加工区
            // PlaceRoundToWorkArea(currentTask.round2_pos);
            updateDisplay("PLACE C2");
            if (roundProgress >= 3) { roundProgress = 0; currentState = STATE_STACK_TEMP2; }
            break;

        case STATE_STACK_TEMP2:
            // 第二批在暂存区码垛到第一批上方(颜色一致, 需平稳放置)
            // 在粗加工区取回第二批后，到暂存区码放第二层：
            // RetrieveRoundToCargo(currentTask.round2_colors, currentTask.round2_pos);
            // StackRoundToWorkArea(currentTask.round2_pos);
            updateDisplay("STACK T2");
            if (roundProgress >= 3) { roundProgress = 0; currentState = STATE_RETURN_HOME; }
            break;

        case STATE_RETURN_HOME:
            updateDisplay("GO HOME");
            // [TODO] 回到启停区, 停转盘...
            currentState = STATE_DONE;
            break;

        case STATE_DONE:
            updateDisplay("DONE");
            if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
            Emm_V5_En_Control_all(false);
            vTaskDelay(100000000 / portTICK_PERIOD_MS); // 保持
            break;
        }

        if (moveNodePathFlag) {
            moveNodePathFlag = false;
            MoveNodePath(nodePathBuffer, nodePathLen);
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// ================= 机载电脑串口指令任务 =================
void Task_Serial_CMD(void *pvParameters) {
    char rxBuffer[64];
    int rxIdx = 0;

    for (;;) {
        while (Serial.available() > 0) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                rxBuffer[rxIdx] = '\0';
                if (rxIdx > 0) {
                    if (handleParameterFrame(rxBuffer, false)) {
                        // 正式运行模式只允许读取参数。
                    }
                    else if (handleAlignmentControlFrame(rxBuffer)) {
                        // 对齐任务已显式启动或停止。
                    }
                    else if (handleVisualAlignmentFrame(rxBuffer)) {
                        // 已接收视觉对齐帧，例如 {ALIGN:1,10,-26}
                    }
                    else if (strcmp(rxBuffer, "{ready}") == 0) { // 机载电脑就绪
                        nano_ready = true;
                        Serial.println("{ready:OK}");
                    }
                    else if (strcmp(rxBuffer, "{start}") == 0) { // 启动机器人
                        enableRun = true;
                        Serial.println("{start:OK}");
                    }
                    else {
                        StartZone parsedZone = START_ZONE_UNKNOWN;
                        if (parseStartZoneFrame(rxBuffer, parsedZone)) {
                            currentStartZone = parsedZone;
                            Serial.printf(
                                "{StartZone:OK,%u}\n",
                                static_cast<unsigned>(parsedZone)
                            );
                        }
                        else if (strncmp(rxBuffer, "{color:", 7) == 0
                                && rxBuffer[strlen(rxBuffer) - 1] == '}') {
                            char *end = nullptr;
                            const float color = strtof(rxBuffer + 7, &end);
                            if (end == rxBuffer + 7 || *end != '}' || end[1] != '\0'
                                    || !isfinite(color)) {
                                Serial.println("{color:ERR}");
                                rxIdx = 0;
                                continue;
                            }
                            VisualCmd_t vc = {"COLOR", color, 0, 0};
                            if (xVisualTaskQueue) xQueueSend(xVisualTaskQueue, &vc, 0);
                            Serial.println("{color:OK}");
                        }
                        else if (strcmp(rxBuffer, "{ok}") == 0) {// 视觉确认到位
                            VisualCmd_t vc = {"OK", 0, 0, 0};
                            if (xVisualTaskQueue) xQueueSend(xVisualTaskQueue, &vc, 0);
                            Serial.println("{ok:ACK}");
                        }
                        else if (strncmp(rxBuffer, "{way:", 5) == 0) {
                            size_t count = 0;
                            if (parseNodePathCommand(
                                    rxBuffer, nodePathBuffer,
                                    sizeof(nodePathBuffer) / sizeof(nodePathBuffer[0]), count)) {
                                nodePathLen = count;
                                moveNodePathFlag = true;
                                Serial.printf("{way:OK,%u}\n", (unsigned)count);
                            } else {
                                Serial.println("{way:ERR}");
                            }
                        }
                        else if (rxBuffer[0] == '{') {
                            Serial.println("{ERR:UNKNOWN_FRAME}");
                        }
                    }
                    rxIdx = 0;
                }
            } else if (rxIdx < 63) {
                rxBuffer[rxIdx++] = c;
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// ================= 调试模式串口命令 =================
void Task_Debug_CMD(void *pvParameters) {
    char buffer[100];
    int bufferIndex = 0;
    for (;;) {
        while (Serial.available() > 0) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                if (bufferIndex > 0) {
                    buffer[bufferIndex] = '\0';
                    if (handleParameterFrame(buffer, true)) {
                        bufferIndex = 0;
                        continue;
                    }
                    if (handleAlignmentControlFrame(buffer)) {
                        bufferIndex = 0;
                        continue;
                    }
                    if (handleVisualAlignmentFrame(buffer)) {
                        bufferIndex = 0;
                        continue;
                    }
                    StartZone debugZone = START_ZONE_UNKNOWN;
                    if (parseStartZoneFrame(buffer, debugZone)) {
                        currentStartZone = debugZone;
                        if (debugZone == START_ZONE_1) {
                            currentPose = {2250, 150, 180};
                        } else {
                            currentPose = {150, 150, 0};
                        }
                        Serial.printf(
                            "{StartZone:OK,%u}\n",
                            static_cast<unsigned>(debugZone)
                        );
                        bufferIndex = 0;
                        continue;
                    }
                    if (strncmp(buffer, "{way:", 5) == 0) {
                        uint8_t debugPath[16] = {0};
                        size_t debugPathLength = 0;
                        if (parseNodePathCommand(
                                buffer, debugPath,
                                sizeof(debugPath) / sizeof(debugPath[0]), debugPathLength)) {
                            Serial.printf("{way:OK,%u}\n", (unsigned)debugPathLength);
                            MoveNodePath(debugPath, debugPathLength);
                        } else {
                            Serial.println("{way:ERR}");
                        }
                        bufferIndex = 0;
                        continue;
                    }
                    const size_t frameLength = strlen(buffer);
                    if (frameLength < 2 || buffer[0] != '{'
                            || buffer[frameLength - 1] != '}') {
                        Serial.println("{ERR:FRAME}");
                        bufferIndex = 0;
                        continue;
                    }
                    buffer[frameLength - 1] = '\0';
                    char *debugCommand = buffer + 1;
                    for (char *cursor = debugCommand; *cursor != '\0'; ++cursor) {
                        if (*cursor == ':' || *cursor == ',') *cursor = ' ';
                    }
                    char cmd[20]; float p1=0,p2=0,p3=0;
                    if (sscanf(debugCommand, "%19s %f %f %f", cmd, &p1, &p2, &p3) >= 1) {
                        // [TODO] 按需接入: GOTOpose / movepose / SERVO / height / enable 等
                        if (strcmp(cmd, "GOTOpose") == 0) 
                            {  Serial.printf("{GOTOpose:ACK,%.0f,%.0f,%.0f}\n", p1, p2, p3);  GotoPose(p1,p2,p3,true);}
                        else if (strcmp(cmd, "SetPose") == 0)
                            {
                                if (isfinite(p1) && isfinite(p2) && isfinite(p3)
                                        && p1 >= 0 && p1 <= 2400
                                        && p2 >= 0 && p2 <= 2400) {
                                    currentPose = {p1, p2, p3};
                                    Serial.printf("{SetPose:ACK,%.1f,%.1f,%.1f}\n", p1, p2, p3);
                                } else {
                                    Serial.println("{SetPose:ERR,RANGE}");
                                }
                            }
                        else if (strcmp(cmd, "Movepose") == 0) 
                            {  Serial.printf("{Movepose:ACK,%.0f,%.0f,%.0f}\n", p1, p2, p3); MovePose(p1,p2,p3);}
                        else if (strcmp(cmd, "MoveArm_1") == 0) 
                            {  Serial.printf("{MoveArm_1:ACK,%.0f,%.0f,%.0f}\n", p1, p2, p3); MoveArm(p1,p2,-1,-1,p3);}
                        else if (strcmp(cmd, "MoveArm_2") == 0) 
                            {  Serial.printf("{MoveArm_2:ACK,%.0f,%.0f,%.0f}\n", p1, p2, p3); MoveArm(-1,-1,p1,p2,p3);}
                        else if (strcmp(cmd, "SERVO") == 0) 
                            {  Serial.printf("{SERVO:ACK,%.0f,%.0f}\n", p1, p2);  Servo_SetAngle((uint8_t)p1, p2, 0, 0);}
                        else if (strcmp(cmd, "En_C") == 0) 
                            {  Serial.printf("{En_C:ACK,%.0f}\n", p1);  Emm_V5_En_Control_all(p1);}
                        else if (strcmp(cmd, "MaterialDemo") == 0)
                            {
                                Serial.println("{MaterialDemo:ACK}");
                                const bool ok = DemoCargoToRoughArea();
                                Serial.println(ok ? "{MaterialDemo:OK}" : "{MaterialDemo:ERR}");
                            }
                        else if (strcmp(cmd, "help") == 0)
                            {    Serial.println("Cmds: {ALIGN:START} {ALIGN:STOP} {ALIGN:a,x,y} {way:1-2-5} {StartZone:1} {GOTOpose:x,y,theta} {SetPose:x,y,theta} {Movepose:dir,speed,stop} {MoveArm_1:h,l,speed} {MoveArm_2:turret,pawl,speed} {SERVO:id,angle} {En_C:enable} {help}");}
                        else  {    Serial.println("{ERR:UNKNOWN_FRAME}");}
                    }
                    bufferIndex = 0;
                }
            } else if (bufferIndex < 99) {
                buffer[bufferIndex++] = c;
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// ================= setup  =================
void setup() {
    Serial.begin(115200);
    Serial.printf("version: %s\n", VERSION);
    Servo_Init();    // 总线舵机初始化 (默认 Serial2: RX=16, TX=15, 115200bps)
    Emm_V5_Init();   // 电机初始化
    Scanner_Init();  // 扫码模块初始化 (软串口 RX=IO4, 9600bps)
    // [TODO] 若有传感器/定位硬件, 在此初始化
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(10);

    currentPose = {0, 0, 0};   // [TODO] 初始位姿按实际
    currentArm = {0, 0};        // [TODO] 初始臂位姿按实际

    // 注意: 原 initLidar() 已移除, 雷达/定位方案待定
     init_ota_service("null", "1234567899", OTA_HOSTNAME);//调试使用，正式比赛时注释掉
    //init_ota_service("longggg", "asdfghjkl", OTA_HOSTNAME);//调试使用，正式比赛时注释掉
    leds[0] = CRGB::Red; FastLED.show();

    // 一键启动: 物理按键(长按)进入 Release 运行模式
    pinMode(MODE_key, INPUT_PULLUP);
    bool bootKeyPressed = false;
    unsigned long startTime = millis();
    while (millis() - startTime < 5000) {
        if (digitalRead(MODE_key) == LOW) { bootKeyPressed = true; break; }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // 机载电脑指令队列(深度10)
    xVisualTaskQueue = xQueueCreate(10, sizeof(VisualCmd_t));
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

    if (bootKeyPressed) {
        Serial.println("Debug mode");
        leds[0] = CRGB::Yellow; FastLED.show();
        // Debug 模式默认直接启用视觉闭环对齐，仍可通过
        // {ALIGN:STOP}/{ALIGN:START} 在运行时停止或重新启动。
        handleAlignmentControlFrame("{ALIGN:START}");
        xTaskCreate(Task_Debug_CMD, "Task_Debug_CMD", 16384, NULL, 5, NULL);
    } else {
        Serial.println("Release mode");
        leds[0] = CRGB::Green; FastLED.show();
        xTaskCreate(Task_MainStateMachine, "Task_MainStateMachine", 16384, NULL, 8, &xTask_MainStateMachine_Handle);
        xTaskCreate(Task_Serial_CMD, "Task_Serial_CMD", 16384, NULL, 8, NULL);
        // 总超时兜底(如 300s), 未完成则回启停区
        xHomeTimer = xTimerCreate("HomeTimer", pdMS_TO_TICKS(300000), pdFALSE, NULL, vHomeTimerCallback);
    }
}

void loop() {
    vTaskDelay(10000 / portTICK_PERIOD_MS);
}
