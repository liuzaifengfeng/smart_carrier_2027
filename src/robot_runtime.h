#pragma once

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
#include "disc_material_decision.h"
#include "task_code.h"

#define LED_PIN 48
#define NUM_LEDS 1
#define OTA_HOSTNAME "smartcarrier_ESP32S3"
#define VERSION "0.1.4-framework"
#define CHASSIS_MOTOR_1 1
#define CHASSIS_MOTOR_2 2
#define CHASSIS_MOTOR_3 3
#define CHASSIS_MOTOR_4 4
#define LIFT_MOTOR_5   5      // 升降电机
#define ARM_MOTOR_6   6      // 机械臂电机
constexpr uint32_t ALIGN_FEEDBACK_TIMEOUT_MS = 300;
constexpr uint32_t ALIGN_LOG_INTERVAL_MS = 500;
constexpr uint32_t AUTO_ALIGN_TIMEOUT_MS = 15000; // 包含等待首帧的时间。
constexpr uint8_t SCAN_AREA_NODE = 2;
constexpr uint8_t DISC_AREA_NODE = 14;
// 粗加工区中心约为 (210, 1100)，对应当前 5x5 地图的 10 号节点。
constexpr uint8_t COARSE_AREA_NODE = 10;
// 当前地图暂存区中心 (1200,2180) 对应节点 22，地图调整时同步核对。
constexpr uint8_t TEMP_AREA_NODE = 22;
// 启停区附近的地图节点；最终归位由 CORNER 视觉对齐。
constexpr uint8_t HOME_ZONE1_NODE = 4;
constexpr uint8_t HOME_ZONE2_NODE = 0;
constexpr float DISC_AREA_HEADING = 90.0f;
constexpr float COARSE_AREA_HEADING = 270.0f;
constexpr float TEMP_AREA_HEADING = 180.0f;
enum RobotState {
    STATE_WAIT_START,    // 待机,等一键启动
    STATE_READ_TASK,     // 读取任务码(二维码板 / 机载电脑)
    STATE_SCAN_FAILED,   // 扫码重试耗尽，保持停车，不进入抓取流程
    STATE_ROUTE_FAILED,  // 路径失败，不进入后续用户代码
    STATE_ALIGN_FAILED,  // 自动对齐失败，停车等待处理
    STATE_TIMEOUT_FAILED, // 总任务超时，停止并等待人工处理
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

enum StartZone {
    START_ZONE_UNKNOWN = 0,
    START_ZONE_1 = 1,
    START_ZONE_2 = 2
};

enum class NodePathState { IDLE, WAITING, RECEIVING, RECEIVED, RUNNING, DONE, FAILED };

enum class AutoAlignmentState { IDLE, WAITING, DONE, FAILED };

typedef struct {
    float angleDeg;
    float visualX;
    float visualY;
    uint32_t sequence;
} VisualAlignmentFrame_t;

typedef struct {
    char cmd[20];
    float param1, param2, param3;
} VisualCmd_t;

extern CRGB leds[NUM_LEDS];
extern float X_PULSE;
extern TaskCode currentTask;
extern portMUX_TYPE taskCodeMux;
extern volatile RobotState currentState;
extern volatile StartZone currentStartZone;
extern volatile bool taskReceived;
extern volatile int  roundProgress;
extern volatile bool firstDiscGrabReady;
extern volatile bool discMessageResumeRequested; // look2 完成后，处理固定旧缓存至帧边界再恢复颜色处理。
extern volatile bool enableRun;
extern QueueHandle_t xAlignmentQueue;
extern SemaphoreHandle_t xAlignmentMotionMutex;
extern QueueHandle_t xVisualTaskQueue;
extern TimerHandle_t xHomeTimer;
extern SemaphoreHandle_t xLidarPoseMutex;

bool requestAndMoveNodePath(uint8_t startNode, uint8_t endNode, float finalHeading = NAN);
AutoAlignmentState getAutoAlignmentState();
void setAlignmentEnabled(bool enable, bool acknowledge = false, bool automatic = false,
                         VisionStartMode owner = VisionStartMode::NONE, bool onlyOwner = false);
bool runLidarPoseAction(bool fromCommand = false);
bool updateDisplay(const char *commandType, const char *displayType, const char *content);
void requestVisionStart(VisionStartMode mode);
void requestVisionStop(VisionStartMode mode);
bool waitScannerCode(char *out, uint32_t len, uint32_t timeoutMs);
void readStartupServoAngle(uint8_t servoId, float &storedAngle);
void Task_MainStateMachine(void *pvParameters);
void Task_Serial_CMD(void *pvParameters);
void Task_VisualAlignment(void *pvParameters);
void vHomeTimerCallback(TimerHandle_t xTimer);
