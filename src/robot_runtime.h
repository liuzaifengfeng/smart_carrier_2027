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

/**
 * @brief 请求起终点节点路径并同步等待导航作业；正式主状态机改用 SubmitMotion。
 * @return true 路径作业完成，false 请求或执行失败。
 */
bool requestAndMoveNodePath(uint8_t startNode, uint8_t endNode, float finalHeading = NAN);
/**
 * @brief 读取自动对齐状态，供主流程决定是否进入取放料阶段。
 */
AutoAlignmentState getAutoAlignmentState();
/**
 * @brief 开启或关闭底盘视觉对齐，并维护当前模式、完成判定和显示应答。
 * @param enable true 开启，false 关闭。
 * @param acknowledge 是否输出控制应答。
 * @param automatic 是否属于自动比赛对齐。
 * @param owner 拥有该次对齐的视觉模式。
 * @param onlyOwner 是否仅允许匹配拥有者关闭。
 */
void setAlignmentEnabled(bool enable, bool acknowledge = false, bool automatic = false, VisionStartMode owner = VisionStartMode::NONE, bool onlyOwner = false);
/**
 * @brief 同步兼容入口：提交起始区扫码姿态并等待结果。
 * @return true 作业成功，false 未提交或执行失败。
 */
bool runLidarPoseAction(bool fromCommand = false, bool allowRepeat = false);
/**
 * @brief 按起始区提交扫码准备姿态，正式模式复用已完成标志避免重复动作。
 * @param fromCommand true 表示串口显式请求。
 * @param allowRepeat 是否允许重复执行已完成的正式准备动作。
 * @return 非零作业编号；0 表示未提交或无需重复。
 */
uint32_t SubmitLidarPoseAction(bool fromCommand=false,bool allowRepeat=false);
/**
 * @brief 处理扫码准备作业结束，仅在正式模式成功时登记准备已完成。
 */
void CompleteLidarPose(bool success,bool debug);
/**
 * @brief 封装并发送上位机显示事件；返回是否发送成功。
 */
bool updateDisplay(const char *commandType, const char *displayType, const char *content);
/**
 * @brief 发送开启指定视觉模式的请求，并按模式设置对齐或物料识别状态。
 */
void requestVisionStart(VisionStartMode mode);
/**
 * @brief 发送停止指定视觉模式的请求，并结束对应本地控制状态。
 */
void requestVisionStop(VisionStartMode mode);
/**
 * @brief 从扫码枪消息队列取出一帧字符串。
 * @param out 输出任务码缓冲区。
 * @param len 缓冲区容量，包含结束符。
 * @param timeoutMs 等待毫秒数，0 表示立即查询。
 * @return true 取得数据，false 没有数据或参数无效。
 */
bool waitScannerCode(char *out, uint32_t len, uint32_t timeoutMs);
/**
 * @brief 启动时读取转台或夹爪实际角度，成功时写入 storedAngle；不发运动指令。
 */
void readStartupServoAngle(uint8_t servoId, float &storedAngle);
/**
 * @brief 比赛主任务，每10ms调用 CompetitionFlow::tick，处理状态编排及主阶段日志。
 */
void Task_MainStateMachine(void *pvParameters);
/**
 * @brief 接收串口帧并分发查询、参数、调试与运动请求；运动交给统一执行器。
 */
void Task_Serial_CMD(void *pvParameters);
/**
 * @brief 采样板载BOOT按钮并记录Release请求，实际切换由空闲串口任务处理。
 */
void PollDebugBootButton(); // loop采样，Release由空闲的串口任务统一执行。
/**
 * @brief 接收视觉反馈并发布最新数据；实际闭环发令由运动任务推进。
 */
void Task_VisualAlignment(void *pvParameters);
/**
 * @brief 比赛总计时器超时回调，执行既有停止、失败状态及主任务暂停处理。
 */
void vHomeTimerCallback(TimerHandle_t xTimer);
