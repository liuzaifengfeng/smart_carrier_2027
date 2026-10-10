#include "robot_runtime.h"
#include "alignment_start_policy.h"

// ================= 基础配置 =================

uint32_t ALIGN_PID_MAX_SPEED_RPM = 20;  // 移动速度单位为转/分

CRGB leds[NUM_LEDS];  // LED 像素数组(板载 WS2812B)

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

TaskCode currentTask = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, false };
portMUX_TYPE taskCodeMux = portMUX_INITIALIZER_UNLOCKED;

// ================= 机载电脑通信(串口)协议 =================
// 机器帧使用 {CMD/RSP/EVT,类别,动作,...}\n，详见 Document/serial_protocol.md。

// ================= 业务状态 =================
volatile RobotState currentState = STATE_WAIT_START;

// 共享业务变量(由机载电脑指令/任务更新)
volatile bool nano_ready = false;       // 机载电脑就绪
volatile StartZone currentStartZone = START_ZONE_UNKNOWN; // 当前启停区,由机载电脑告知
volatile bool taskReceived = false;// 已拿到任务码
volatile int  roundProgress = 0;   // 当前轮次已抓/放物料数 0-3
volatile bool discMaterialActive = false;
volatile bool firstDiscGrabReady = false; // 当前轮导航、定位和观察恢复完成后才允许业务抓取。
volatile bool discMessageResumeRequested = false;
uint8_t discMaterialColor = 0; // 最近一次 COLOR；每次启停或抓取后失效。
volatile bool enableRun = false;   // 一键启动触发

// 两个任务通过临界区交接路径，避免串口在执行中覆盖节点数组。
NodePathState nodePathState = NodePathState::IDLE;
portMUX_TYPE nodePathMux = portMUX_INITIALIZER_UNLOCKED;
uint8_t nodePathBuffer[MAX_NODE_PATH_LENGTH] = {0};
size_t nodePathLen = 0;

static bool executeNodePathWithStatus(const uint8_t *path, size_t count, float finalHeading = NAN) {
    Serial.println("{EVT,NAV,ROUTE_RUNNING}");
    if (!isfinite(finalHeading) && path != nullptr && count > 0) {
        switch (path[count - 1]) {
            case DISC_AREA_NODE: finalHeading = DISC_AREA_HEADING; break;
            case COARSE_AREA_NODE: finalHeading = COARSE_AREA_HEADING; break;
            case TEMP_AREA_NODE: finalHeading = TEMP_AREA_HEADING; break;
            default: break;
        }
    }
    const bool success = MoveNodePath( path, count, NODE_PATH_SPEED_RPM, 50, finalHeading);
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
bool requestAndMoveNodePath(uint8_t startNode, uint8_t endNode, float finalHeading) {
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
        success = executeNodePathWithStatus(path, count, finalHeading);
    }
    portENTER_CRITICAL(&nodePathMux);
    nodePathState = success ? NodePathState::DONE : NodePathState::FAILED;
    portEXIT_CRITICAL(&nodePathMux);
    return success;
}

QueueHandle_t xAlignmentQueue = NULL;
SemaphoreHandle_t xAlignmentMotionMutex = NULL;
uint32_t alignmentFrameSequence = 0; // 受运动锁保护，用于发现最新帧队列覆盖造成的丢帧。
VisionStartMode alignmentOwner = VisionStartMode::NONE; // 受运动锁保护。
volatile bool alignmentEnabled = false;
AutoAlignmentState autoAlignmentState = AutoAlignmentState::IDLE;
uint32_t autoAlignmentStartedMs = 0;
uint32_t alignmentGeneration = 0; // 受运动锁保护，丢弃跨启停周期的已出队帧。

AutoAlignmentState getAutoAlignmentState() {
    if (!xAlignmentMotionMutex) return AutoAlignmentState::FAILED;
    xSemaphoreTake(xAlignmentMotionMutex, portMAX_DELAY);
    const auto state = autoAlignmentState;
    xSemaphoreGive(xAlignmentMotionMutex);
    return state;
}

// 参数提交与 PID 更新共用锁，避免一个控制周期读取到半套参数。
void setAlignmentEnabled(bool enable, bool acknowledge, bool automatic,
                                VisionStartMode owner,
                                bool onlyOwner) {
    if (xAlignmentMotionMutex != NULL
            && xSemaphoreTake(xAlignmentMotionMutex, portMAX_DELAY) == pdTRUE) {
        // 结束旧定位功能不能停止随后开启的另一种定位或手动连续对齐。
        if (onlyOwner && alignmentOwner != owner) {
            xSemaphoreGive(xAlignmentMotionMutex);
            return;
        }
        if (enable && !automatic) {
            // 在同一运动锁内判定，保护 WAITING/DONE 以及尚未被主任务处理的 FAILED。
            const auto action = DecideAlignmentStart(
                autoAlignmentState != AutoAlignmentState::IDLE,
                autoAlignmentState == AutoAlignmentState::FAILED,
                alignmentOwner, owner);
            if (action != AlignmentStartAction::RESET) {
                xSemaphoreGive(xAlignmentMotionMutex);
                if (acknowledge) Serial.println(action == AlignmentStartAction::KEEP
                    ? "{RSP,VISION,ALIGN_START,OK}"
                    : "{RSP,VISION,ALIGN_START,ERR,BUSY}");
                // 不清空反馈，不重置 PID、代次、连续帧计数、完成状态或超时起点。
                return;
            }
        }
        alignmentEnabled = false;
        alignmentOwner = enable ? owner : VisionStartMode::NONE;
        ++alignmentGeneration;
        if (xAlignmentQueue != NULL) xQueueReset(xAlignmentQueue);
        OmniMove(0.0f, 0.0f, 0.0f, 0);
        ResetDiscAlignmentPid();
        ResetAlignmentWait();
        // 手动停止仍取消业务等待；重复启动已在上方保护自动状态。
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

// 执行雷达扫描位姿并统一发送串口应答。
// ACK 表示命令已开始处理；OK 只会在全部动作执行完成后发送。
bool runLidarPoseAction(bool fromCommand) {
    if (xLidarPoseMutex == NULL || xSemaphoreTake(xLidarPoseMutex, 0) != pdTRUE) {
        Serial.println(fromCommand ? "{RSP,VISION,LIDAR_POSE,ERR,BUSY}" : "{EVT,VISION,LIDAR_POSE_FAILED,BUSY}");
        return false;
    }
    Serial.println(fromCommand ? "{RSP,VISION,LIDAR_POSE,ACK}" : "{EVT,VISION,LIDAR_POSE_RUNNING}");
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
    uint32_t previousFrameSequence = 0;

    for (;;) {
        xSemaphoreTake(xAlignmentMotionMutex, portMAX_DELAY);
        const uint32_t generation = alignmentGeneration;
        if (generation != previousGeneration) {
            feedbackActive = false;
            alignedReported = false;
            lastFeedbackMs = 0;
            previousGeneration = generation;
            previousFrameSequence = 0;
        }
        if (autoAlignmentState == AutoAlignmentState::WAITING
                && millis() - autoAlignmentStartedMs >= AUTO_ALIGN_TIMEOUT_MS) {
            OmniMove(0.0f, 0.0f, 0.0f, 0);
            ResetDiscAlignmentPid();
            ResetAlignmentWait();
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
            /* 临时禁用 300 ms 断流停车/失败分支，保留 15 秒首次对齐总超时。
             * 无新反馈时会保持最后一次轮速；恢复保护时取消本段注释。
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
                    ResetAlignmentWait();
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
            */
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
                // 丢帧或长时间未处理反馈时，不把前后帧累计为连续 5 帧。
                if ((previousFrameSequence != 0
                        && alignment.sequence != previousFrameSequence + 1)
                        || dtSeconds >= ALIGN_FEEDBACK_TIMEOUT_MS / 1000.0f) {
                    ResetAlignmentWait();
                }
                previousFrameSequence = alignment.sequence;
                const auto mode = alignmentOwner == VisionStartMode::NONE
                    ? VisionStartMode::DISC : alignmentOwner;
                aligned = WaitForAlignment(mode,
                    alignment.angleDeg, alignment.visualX, alignment.visualY,
                    dtSeconds, ALIGN_PID_MAX_SPEED_RPM
                );
                if (aligned && autoAlignmentState == AutoAlignmentState::WAITING) {
                    // 锁存连续 5 帧达标结果供业务流程读取；连续闭环保持开启，直到显式停止。
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
static void requestVisionFunction(VisionStartMode mode, bool start) {
    if (mode == VisionStartMode::DISC_MATERIAL) {
        discMessageResumeRequested = false;
        discMaterialActive = start;
        discMaterialColor = 0;
    }
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
        case VisionStartMode::NONE:
            return;
    }
    // 自动流程和串口按钮共用入口：先开启底盘闭环，再请求相机反馈。
    if (VisionModeUsesAlignment(mode)) {
        setAlignmentEnabled(start, false, start, mode, !start);
    }
    if (name) Serial.printf("{EVT,VISION,%s,%s}\n", start ? "START_REQUEST" : "STOP_REQUEST", name);
}

void requestVisionStart(VisionStartMode mode) { requestVisionFunction(mode, true); }

// 定位功能同时结束其底盘闭环；事件不表示相机已经停止。
void requestVisionStop(VisionStartMode mode) { requestVisionFunction(mode, false); }

// 总超时可能发生在任意路段或搬运中，位置和持料状态未知，停机等待人工处理。
void vHomeTimerCallback(TimerHandle_t xTimer) {
    if (currentState != STATE_DONE) {
        taskMotionAborted = true;
        currentState = STATE_TIMEOUT_FAILED;
        firstDiscGrabReady = false;
        requestVisionStop(VisionStartMode::DISC_MATERIAL);
        requestVisionStop(VisionStartMode::DISC);
        requestVisionStop(VisionStartMode::WORK_AREA);
        requestVisionStop(VisionStartMode::WORK_AREA_LOADED);
        requestVisionStop(VisionStartMode::CORNER);
        setAlignmentEnabled(false);
        portENTER_CRITICAL(&nodePathMux);
        nodePathState = NodePathState::FAILED;
        portEXIT_CRITICAL(&nodePathMux);
        if (xTask_MainStateMachine_Handle != NULL)
            vTaskSuspend(xTask_MainStateMachine_Handle);
        // 主任务在暂停前可能从失败的 MoveArm 返回，重新锁存总超时故障。
        currentState = STATE_TIMEOUT_FAILED;
        // 停止六个步进电机，保留夹爪位置，避免未知持料时松手。
        for (uint8_t motor = 1; motor <= 6; ++motor) Emm_V5_Stop_Now(motor, false);
        updateDisplay("DISPLAY", "DEBUG", "TASK TIMEOUT");
        Serial.println("{EVT,NAV,ROUTE_FAILED,TIMEOUT}");
        Serial.println("[TIMER] Task timeout: stopped; manual recovery required");
    }
}

// @brief 等待扫码消息队列 (供主状态机在各环节调用)
// @param out       输出缓冲区
// @param len       缓冲区长度
// @param timeoutMs 阻塞超时(ms), 0=非阻塞
// @return true 拿到一帧扫码字符串
bool waitScannerCode(char *out, uint32_t len, uint32_t timeoutMs) {
    if (Scanner_WaitCode(out, len, timeoutMs)) {
        Serial.printf("[SCANNER] recv: %s\n", out);
        return true;
    }
    return false;
}

// ================= 主状态机 =================
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
void readStartupServoAngle(uint8_t servoId, float &storedAngle) {
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
