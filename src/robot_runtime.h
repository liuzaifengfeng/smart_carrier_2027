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
constexpr uint8_t BOOT0_PIN = 0; // 板载BOOT按键，低电平按下。
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
extern float NODE_PATH_SPEED_MM_S; // 正式节点路径平移速度，mm/s；不是电机RPM。
extern float NODE_PATH_ACCEL_MM_S2; // 正式节点导航平移加速度，mm/s²；启动整条路径时冻结。
extern float NODE_PATH_TURN_DEG_S;
extern float NODE_PATH_TURN_ACCEL_DEG_S2;
extern float NODE_PATH_HEADING_DEADBAND_DEG;
extern uint32_t SCAN_APPROACH_SPEED_RPM;
extern uint32_t SCAN_SEARCH_SPEED_RPM;
constexpr uint8_t SCAN_AREA_NODE = 2;
extern bool scanPosePending;
constexpr uint8_t DISC_AREA_NODE = 14;
// 粗加工区中心约为 (210, 1100)，对应当前 5x5 地图的 10 号节点。
constexpr uint8_t COARSE_AREA_NODE = 10;
// 当前地图暂存区中心 (1200,2180) 对应节点 22，地图调整时同步核对。
constexpr uint8_t TEMP_AREA_NODE = 22;
// 启停区附近的地图节点；最终归位由 CORNER 视觉对齐。
constexpr uint8_t HOME_ZONE1_NODE = 4;
constexpr uint8_t HOME_ZONE2_NODE = 0;
// 两个启停区实车车头均朝左；区2向右扫码是后退，不改变车头朝向。
constexpr float START_ZONE_HEADING = 180.0f;
constexpr float DISC_AREA_HEADING = 90.0f;
constexpr float COARSE_AREA_HEADING = 270.0f;
constexpr float TEMP_AREA_HEADING = 180.0f;
#include "robot_state.h"



enum class NodePathState { IDLE, WAITING, RECEIVING, RECEIVED, RUNNING, DONE, FAILED };



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
extern volatile bool discMessageResumeRequested; // look3 实测到位后，排空此前旧帧再开放圆盘抓取。
extern volatile bool enableRun;
extern volatile bool alignmentEnabled;
extern volatile bool discMaterialActive;
extern uint8_t discMaterialColor;
extern QueueHandle_t xAlignmentQueue;
extern NodePathState nodePathState;
extern portMUX_TYPE nodePathMux;
extern uint8_t nodePathBuffer[MAX_NODE_PATH_LENGTH];
extern size_t nodePathLen;
extern SemaphoreHandle_t xAlignmentMotionMutex;
extern QueueHandle_t xVisualTaskQueue;
extern TimerHandle_t xHomeTimer;
extern SemaphoreHandle_t xLidarPoseMutex;

bool requestAndMoveNodePath(uint8_t startNode, uint8_t endNode, float finalHeading = NAN);
AutoAlignmentState getAutoAlignmentState();
void setAlignmentEnabled(bool enable, bool acknowledge = false, bool automatic = false, VisionStartMode owner = VisionStartMode::NONE, bool onlyOwner = false);
bool runLidarPoseAction(bool fromCommand = false, bool allowRepeat = false);
uint32_t SubmitLidarPoseAction(bool fromCommand=false,bool allowRepeat=false);
void CompleteLidarPose(bool success,bool debug);
bool updateDisplay(const char *commandType, const char *displayType, const char *content);
void requestVisionStart(VisionStartMode mode);
void requestVisionStop(VisionStartMode mode);
bool waitScannerCode(char *out, uint32_t len, uint32_t timeoutMs);
void readStartupServoAngle(uint8_t servoId, float &storedAngle);
void Task_MainStateMachine(void *pvParameters);
void Task_Serial_CMD(void *pvParameters);
void PollDebugBootButton(); // loop采样，Release由空闲的串口任务统一执行。
void Task_VisualAlignment(void *pvParameters);
void vHomeTimerCallback(TimerHandle_t xTimer);
