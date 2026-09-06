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
#include <FastLED.h>
#include "ota_service.h"
#include <ArduinoJson.h>

#include "Emm_V5.h"
#include "chassis.h"
#include "servo.h"
#include "scanner.h"

// ================= 基础配置 =================
#define MODE_key 0             // 开机按键(长按进调试模式)
#define LED_PIN 48
#define NUM_LEDS 1
#define OTA_HOSTNAME "smartcarrier"
#define VERSION "0.1.0-framework"

CRGB leds[NUM_LEDS];  // LED 像素数组(板载 WS2812B)

// 电机使能/同步常量(沿用 Emm_V5)
#define CHASSIS_MOTOR_1 1
#define CHASSIS_MOTOR_2 2
#define CHASSIS_MOTOR_3 3
#define CHASSIS_MOTOR_4 4
#define LIFT_MOTOR_5   5      // 升降电机
#define ARM_MOTOR_6   6      // 机械臂电机

// 运动标定系数(新底盘需重新标定) [TODO]
float X_PULSE     = 10.5f;    // X向 每毫米脉冲
float Y_PULSE     = 11.4f;    // Y向 每毫米脉冲
float THETA_PULSE = 83.4f;    // 旋转 每度脉冲
float HEIGHT_PULSE = 32.26f;  // 升降 每毫米脉冲

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
// 机载电脑 -> ESP32 (通过 Serial0)
//   "ready"                  : 机载电脑就绪
//   "task:<任务码>"          : 下发搬运任务码
//   "color:<编号>"           : 识别到指定颜色物料,请求抓取
//   "target:<x,y,theta>"     : 下发目标坐标(视觉定位引导)
//   "ok"                     : 视觉确认到位
// ESP32 -> 机载电脑
//   "[TASK:<任务码>]"        : 确认收到任务
//   "[GRAB_OK]" / "[PLACE_OK]" : 抓取/放置完成反馈

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

// 共享业务变量(由机载电脑指令/任务更新)
volatile bool ready = false;       // 机载电脑就绪
volatile bool taskReceived = false;// 已拿到任务码
volatile int  roundProgress = 0;   // 当前轮次已抓/放物料数 0-3
volatile bool enableRun = false;   // 一键启动触发

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
void Task_MainStateMachine(void *pvParameters);
void Task_Serial_CMD(void *pvParameters);
void Task_Debug_CMD(void *pvParameters);
void vHomeTimerCallback(TimerHandle_t xTimer);

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

// ================= 主状态机 =================

// 伪函数: 等待扫码消息队列 (供主状态机在各环节调用)
// 当前为占位实现, 后续在此补充: 解析任务码 / 匹配物料 / 触发抓取等业务逻辑
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

void Task_MainStateMachine(void *pvParameters) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (1) {
        switch (currentState) {

        case STATE_WAIT_START:
            // 等一键启动信号(物理按键 / 机载电脑 "ready" 后人工按键)
            updateDisplay("WAIT START");
            while (!enableRun) vTaskDelay(100 / portTICK_PERIOD_MS);
            // 启动总超时兜底(如 300s 内未回启停区)
            if (xHomeTimer != NULL) xTimerStart(xHomeTimer, 0);
            currentState = STATE_READ_TASK;
            break;

        case STATE_READ_TASK:
            updateDisplay("READ TASK");
            // 方案A: 机器人走到二维码板, 机载电脑读码后发 "task:<码>"
            // 方案B: 机载电脑直接下发任务码
            // 方案C: 扫码枪读二维码/条码, 从扫码消息队列取任务码
            // [TODO] 移动到二维码板位姿
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
            updateDisplay("GRAB R1");
            if (roundProgress >= 3) { roundProgress = 0; currentState = STATE_PLACE_COARSE1; }
            break;

        case STATE_PLACE_COARSE1:
            // 按 round1_pos 顺序放置到粗加工区对应圆环
            // 圆环评分: 1环15分 2环10分 3环7分 ... 越中心分越高
            // [TODO] 移动到粗加工区 + 精确放置 + 视觉确认
            updateDisplay("PLACE C1");
            if (roundProgress >= 3) { roundProgress = 0; currentState = STATE_PLACE_TEMP1; }
            break;

        case STATE_PLACE_TEMP1:
            // 从粗加工区取回3个, 按 round1_pos 放到暂存区
            // [TODO] 取回 + 放置暂存区
            updateDisplay("PLACE T1");
            if (roundProgress >= 3) { roundProgress = 0; currentState = STATE_GRAB_ROUND2; }
            break;

        case STATE_GRAB_ROUND2:
            // 同 round1, 抓第二批
            updateDisplay("GRAB R2");
            if (roundProgress >= 3) { roundProgress = 0; currentState = STATE_PLACE_COARSE2; }
            break;

        case STATE_PLACE_COARSE2:
            // 第二批放粗加工区
            updateDisplay("PLACE C2");
            if (roundProgress >= 3) { roundProgress = 0; currentState = STATE_STACK_TEMP2; }
            break;

        case STATE_STACK_TEMP2:
            // 第二批在暂存区码垛到第一批上方(颜色一致, 需平稳放置)
            // [TODO] 精确高度控制 + 码垛
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
                    if (strstr(rxBuffer, "ready")) {
                        ready = true;
                        Serial.println("ready->");
                    }
                    else if (strncmp(rxBuffer, "task:", 5) == 0) {
                        currentTask = parseTaskCode(rxBuffer + 5);
                        taskReceived = currentTask.valid;
                        Serial.printf("task-> %s\n", currentTask.valid ? "OK" : "ERR");
                    }
                    else if (strncmp(rxBuffer, "color:", 6) == 0) {
                        // 视觉识别到目标颜色, 请求主控抓取
                        VisualCmd_t vc = {"COLOR", atof(rxBuffer+6), 0, 0};
                        if (xVisualTaskQueue) xQueueSend(xVisualTaskQueue, &vc, 0);
                        Serial.println("color->");
                    }
                    else if (strncmp(rxBuffer, "target:", 7) == 0) {
                        // 视觉下发目标坐标 x,y,theta
                        VisualCmd_t vc = {"TARGET", 0, 0, 0};
                        sscanf(rxBuffer+7, "%f,%f,%f", &vc.param1, &vc.param2, &vc.param3);
                        if (xVisualTaskQueue) xQueueSend(xVisualTaskQueue, &vc, 0);
                    }
                    else if (strcmp(rxBuffer, "ok") == 0) {
                        // 视觉确认到位
                        VisualCmd_t vc = {"OK", 0, 0, 0};
                        if (xVisualTaskQueue) xQueueSend(xVisualTaskQueue, &vc, 0);
                        Serial.println("ok->");
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
        if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                if (bufferIndex > 0) {
                    buffer[bufferIndex] = '\0';
                    char cmd[20]; float p1=0,p2=0,p3=0;
                    if (sscanf(buffer, "%s %f %f %f", cmd, &p1, &p2, &p3) >= 1) {
                        // [TODO] 按需接入: GOTOpose / movepose / SERVO / height / enable 等
                        if (strcmp(cmd, "GOTOpose") == 0)     GotoPose(p1,p2,p3,false,false);
                        else if (strcmp(cmd, "GOTORpose") == 0) GotoPose(p1,p2,p3,true,false);
                        else if (strcmp(cmd, "movepose") == 0)  movepose(p1,p2,p3);
                        else if (strcmp(cmd, "SERVO") == 0)     Servo_SetAngle((uint8_t)p1, p2, 500);
                        else if (strcmp(cmd, "En_C") == 0)      Emm_V5_En_Control_all(p1);
                        else if (strcmp(cmd, "help") == 0)
                            Serial.println("Cmds: GOTOpose GOTORpose movepose SERVO<id,angle> En_C");
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

// ================= setup / loop =================
void setup() {
    Serial.begin(115200);
    Servo_Init();    // 总线舵机初始化 (默认 Serial2: RX=16, TX=15, 115200bps)
    Emm_V5_Init();   // 电机初始化
    Scanner_Init();  // 扫码模块初始化 (软串口 RX=IO4, 9600bps)
    // [TODO] 若有传感器/定位硬件, 在此初始化
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(10);

    currentPose = {0, 0, 0};   // [TODO] 初始位姿按实际
    // 注意: 原 initLidar() 已移除, 雷达/定位方案待定
    init_ota_service("null", "1234567899", OTA_HOSTNAME);//调试使用，正式比赛时注释掉
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

    if (bootKeyPressed) {
        Serial.println("Debug mode");
        leds[0] = CRGB::Yellow; FastLED.show();
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
