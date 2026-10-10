// 由 robot_runtime.cpp 在业务状态与动作函数之后包含。仅此任务读取 Serial0。
#include <float.h>
#include "debug_release_button.h"
static bool serialDebugMode = true;
static DebugReleaseButton bootReleaseButton;
static portMUX_TYPE bootReleaseMux = portMUX_INITIALIZER_UNLOCKED;

// loop独立采样，串口任务执行阻塞动作时也能识别并忽略按键。
void PollDebugBootButton() {
    const bool pressed = digitalRead(BOOT0_PIN) == LOW;
    const uint32_t nowMs = millis();
    portENTER_CRITICAL(&bootReleaseMux);
    bootReleaseButton.update(pressed, nowMs);
    portEXIT_CRITICAL(&bootReleaseMux);
}

static void serialBootSetIdle(bool idle) {
    portENTER_CRITICAL(&bootReleaseMux);
    bootReleaseButton.setIdle(serialDebugMode && idle);
    portEXIT_CRITICAL(&bootReleaseMux);
}
static void serialReply(const SerialFrame &frame, const char *status) {
    Serial.printf("{RSP,%s,%s,%s}\n", frame.fields[1], frame.fields[2], status);
}

static void serialCfgError(const SerialFrame &frame, const char *reason) {
    uint32_t id = 0;
    if (frame.count > 3) serialUInt(frame.fields[3], id, UINT32_MAX);
    Serial.printf("{RSP,CFG,%s,ERR,%lu,%s}\n", frame.fields[2],
                  static_cast<unsigned long>(id), reason);
}

static bool serialKnownAction(const SerialFrame &frame) {
    static const char *const names[][2] = {
        {"SYS", "HELLO"}, {"SYS", "RELEASE"}, {"SYS", "READY"},
        {"SYS", "START"}, {"SYS", "HELP"},
        {"TASK", "SET"},
        {"LED", "SET"}, {"LED", "GET"}, {"LED", "FREQ"},
        {"NAV", "START_ZONE"}, {"NAV", "ROUTE"},
        {"VISION", "ALIGN_START"}, {"VISION", "ALIGN_STOP"},
        {"VISION", "START_REQUEST"},
        {"VISION", "STOP_REQUEST"},
        {"VISION", "ALIGN_DATA"}, {"VISION", "COLOR"},
        {"VISION", "GRAB"},
        {"VISION", "FORCE_GRAB"},
        {"VISION", "CONFIRM"}, {"VISION", "LIDAR_POSE"},
        {"POSE", "GET"}, {"POSE", "SET"}, {"POSE", "GOTO_REL"},
        {"CHASSIS", "MOVE"}, {"CHASSIS", "MOVE_POSITION"},
        {"ARM", "MOVE1"}, {"ARM", "MOVE2"}, {"ARM", "SERVO"},
        {"ARM", "ENABLE"}, {"ARM", "DEMO1"}, {"ARM", "DEMO2"}, {"ARM", "DEMO3"},
        {"ARM", "DEMO4"},
        {"ARM", "INIT_START"}, {"ARM", "INIT_LOOK"}, {"ARM", "INIT_LOOK2"},
        {"CFG", "GET"}, {"CFG", "BEGIN"}, {"CFG", "SET"}, {"CFG", "COMMIT"},
    };
    for (const auto &name : names) {
        if (strcmp(frame.fields[1], name[0]) == 0
                && strcmp(frame.fields[2], name[1]) == 0) return true;
    }
    return false;
}

static bool serialFloats(const SerialFrame &frame, float *values, const float *minimum,
                         const float *maximum, size_t count) {
    if (frame.count != count + 3) return false;
    for (size_t i = 0; i < count; ++i) {
        if (!serialFloat(frame.fields[i + 3], values[i], minimum[i], maximum[i])) return false;
    }
    return true;
}

static void serialRoute(const SerialFrame &frame) {
    const size_t count = frame.count - 3;
    if (count < 2 || count > MAX_NODE_PATH_LENGTH) {
        serialError(frame, count < 2 ? "FORMAT" : "RANGE");
        return;
    }
    uint8_t path[MAX_NODE_PATH_LENGTH] = {0};
    for (size_t i = 0; i < count; ++i) {
        uint32_t node = 0;
        if (!serialUInt(frame.fields[i + 3], node, FIELD_NODE_COUNT - 1)) {
            serialError(frame, "RANGE");
            return;
        }
        path[i] = static_cast<uint8_t>(node);
    }
    if (serialDebugMode) {
        Serial.printf("{RSP,NAV,ROUTE,ACK,%u}\n", static_cast<unsigned>(count));
        executeNodePathWithStatus(path, count);
        return;
    }
    portENTER_CRITICAL(&nodePathMux);
    const bool accepting = nodePathState == NodePathState::WAITING;
    if (accepting) {
        memcpy(nodePathBuffer, path, count);
        nodePathLen = count;
        nodePathState = NodePathState::RECEIVING;
    }
    portEXIT_CRITICAL(&nodePathMux);
    if (!accepting) {
        serialError(frame, "NOT_WAITING");
        return;
    }
    Serial.printf("{RSP,NAV,ROUTE,ACK,%u}\n", static_cast<unsigned>(count));
    // 应答先于主状态机执行事件，避免上位机把 ACK 当成完成。
    portENTER_CRITICAL(&nodePathMux);
    if (nodePathState == NodePathState::RECEIVING) nodePathState = NodePathState::RECEIVED;
    portEXIT_CRITICAL(&nodePathMux);
}

static void serialRelease(const SerialFrame &frame) {
    if (!serialDebugMode) { serialError(frame, "MODE"); return; }
    if (discMaterialActive) requestVisionStop(VisionStartMode::DISC_MATERIAL);
    setAlignmentEnabled(false);
    xHomeTimer = xTimerCreate("HomeTimer", pdMS_TO_TICKS(3000000), pdFALSE,// 3000s 后强制返回起停区
                             NULL, vHomeTimerCallback);
    if (!xHomeTimer) { serialError(frame, "TIMER"); return; }
    currentState = STATE_WAIT_START;
    taskMotionAborted = false;
    firstDiscGrabReady = false;
    enableRun = false;
    if (xTaskCreate(Task_MainStateMachine, "Task_MainStateMachine", 16384,
                    NULL, 8, &xTask_MainStateMachine_Handle) != pdPASS) {
        xTimerDelete(xHomeTimer, portMAX_DELAY);
        xHomeTimer = NULL;
        serialError(frame, "TASK");
        return;
    }
    serialDebugMode = false;
    leds[0] = CRGB::Green;
    FastLED.show();
    Serial.println("Release mode");
    serialReply(frame, "OK");
}

static void serialBootReleaseAtIdle(bool idle) {
    portENTER_CRITICAL(&bootReleaseMux);
    bootReleaseButton.setIdle(serialDebugMode && idle);
    const bool requested = bootReleaseButton.takeRequest();
    portEXIT_CRITICAL(&bootReleaseMux);
    if (!requested) return;
    // 与串口命令同一个任务、同一个入口，避免并发创建定时器/主状态机。
    char command[] = "{CMD,SYS,RELEASE}";
    SerialFrame frame;
    if (parseSerialFrame(command, frame)) serialRelease(frame);
}

// 普通 COLOR/GRAB 共用完成路径，避免接令或失败也推进任务。
static void serialExecuteTaskGrab(uint8_t color, uint8_t cargo) {
    const int progress = roundProgress;
    const RobotState grabState = currentState;
    const bool automaticGrab = !serialDebugMode
        && (grabState == STATE_GRAB_ROUND1 || grabState == STATE_GRAB_ROUND2);
    if (automaticGrab) {
        // 相机保持识别，只屏蔽主控的颜色处理，避免每件之间反复切换视觉模式。
        firstDiscGrabReady = false;
        discMessageResumeRequested = false;
    }
    discMaterialColor = 0;
    const bool completed = GrabDiscMaterial(color, cargo);
    const int next = AdvanceDiscGrabProgress(progress, cargo, completed);
    Serial.printf("{EVT,VISION,GRAB,%s,%u,%u}\n", completed ? "DONE" : "FAILED", color, cargo);
    if (automaticGrab && !completed) {
        // 可能已执行部分动作，禁止下一帧颜色自动重试同一次抓取。
        requestVisionStop(VisionStartMode::DISC_MATERIAL);
        updateDisplay("DISPLAY", "DEBUG", "DISC GRAB ERR");
        if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
        if (currentState == grabState) currentState = STATE_TRANSFER_FAILED;
        return;
    }
    // 动作执行期间若已超时切换主状态，不发布旧流程的进度。
    if (automaticGrab && currentState != grabState) return;
    if (next != progress) {
        // 先关闭识别，再发布第三件完成，避免主流程已导航时仍有颜色入队。
        if (next == 3) requestVisionStop(VisionStartMode::DISC_MATERIAL);
        roundProgress = next;
    }
}

static void serialDiscColorDecision(uint8_t color) {
    const RobotState state = currentState;
    const int progress = roundProgress;
    const int *colors = state == STATE_GRAB_ROUND2 ? currentTask.round2_colors
        : state == STATE_GRAB_ROUND1 || serialDebugMode ? currentTask.round1_colors : nullptr;
    const uint8_t cargo = progress >= 0 && progress < 3 ? progress + 1 : 0;
    const auto decision = EvaluateDiscColor(colors, progress, color, currentTask.valid,
        alignmentEnabled, serialDebugMode || firstDiscGrabReady,
        cargo != 0 && cargoPlatforms[cargo - 1].material != MATERIAL_NONE);
    if (decision == DiscColorDecision::GRAB) {
        Serial.printf("{EVT,VISION,COLOR,GRAB,%u,%u}\n", color, cargo);
        serialExecuteTaskGrab(color, cargo);
        return;
    }
    const char *reason = "";
    switch (decision) {
        case DiscColorDecision::COLOR_MISMATCH: reason = "COLOR_MISMATCH"; break;
        case DiscColorDecision::NO_TASK: reason = "NO_TASK"; break;
        case DiscColorDecision::NOT_WAITING: reason = "NOT_WAITING"; break;
        case DiscColorDecision::ROUND_COMPLETE: reason = "ROUND_COMPLETE"; break;
        case DiscColorDecision::ALIGN_ACTIVE: reason = "ALIGN_ACTIVE"; break;
        case DiscColorDecision::NOT_READY: reason = "NOT_READY"; break;
        case DiscColorDecision::OCCUPIED: reason = "OCCUPIED"; break;
        case DiscColorDecision::GRAB: return;
    }
    Serial.printf("{EVT,VISION,COLOR,%s,%u,%u,%s}\n",
        decision == DiscColorDecision::COLOR_MISMATCH ? "SKIP" : "BLOCKED", color, cargo, reason);
    if (colors != nullptr && progress >= 0 && progress < 3 && currentTask.valid) {
        Serial.printf("[COLOR] received=%u, expected=%d, cargo=%u, reason=%s\n",
                      color, colors[progress], cargo, reason);
    }
}

// 显式 GRAB 与 COLOR 共用任务进度和完成事件。
static void serialDiscGrab(const SerialFrame &frame) {
    uint32_t color = discMaterialColor;
    uint32_t cargo = 0;
    const bool explicitColor = frame.count == 5;
    if ((explicitColor && (!serialUInt(frame.fields[3], color, 6) || color == 0))
            || !serialUInt(frame.fields[explicitColor ? 4 : 3], cargo, 3) || cargo == 0) {
        serialError(frame, "RANGE"); return;
    }
    if (!discMaterialActive) { serialError(frame, "NOT_ACTIVE"); return; }
    if (alignmentEnabled) { serialError(frame, "ALIGN_ACTIVE"); return; }
    if (color == 0) { serialError(frame, "NO_COLOR"); return; }
    const RobotState grabState = serialDebugMode ? STATE_GRAB_ROUND1 : currentState;
    {
        if ((grabState != STATE_GRAB_ROUND1 && grabState != STATE_GRAB_ROUND2)
                || (!serialDebugMode && !firstDiscGrabReady)
                || roundProgress < 0 || roundProgress >= 3) {
            serialError(frame, "NOT_WAITING"); return;
        }
        const int *colors = grabState == STATE_GRAB_ROUND1
            ? currentTask.round1_colors : currentTask.round2_colors;
        if (cargo != static_cast<uint32_t>(roundProgress + 1)) {
            serialError(frame, "ORDER"); return;
        }
        if (!ShouldGrabTaskColor(colors, roundProgress, static_cast<uint8_t>(color), currentTask.valid)) {
            serialError(frame, "COLOR_MISMATCH"); return;
        }
    }
    if (cargoPlatforms[cargo - 1].material != MATERIAL_NONE) {
        serialError(frame, "OCCUPIED"); return;
    }
    discMaterialColor = 0; // 消费颜色，下一次抓取必须有新的颜色反馈或显式颜色参数。
    Serial.printf("{RSP,VISION,GRAB,ACK,%lu,%lu}\n", color, cargo);
    serialExecuteTaskGrab(static_cast<uint8_t>(color), static_cast<uint8_t>(cargo));
}

static void serialDispatch(const SerialFrame &frame) {
    // 每个分支必须先完整匹配动作及参数数量，再读取参数或调用运动函数。
    // 1. 通用命令与共享业务状态。
    if (frame.is("SYS", "HELLO", 0)) {
        Serial.println("{RSP,SYS,HELLO,OK,2}");
    } else if (frame.is("TASK", "SET", 1)) {
        const TaskCode task = parseTaskCode(frame.fields[3]);
        if (!task.valid) { serialError(frame, "RANGE"); return; }
        // Release 只允许开局预置或扫码等待期间输入，防止覆盖正在执行的任务。
        portENTER_CRITICAL(&taskCodeMux);
        const bool accepting = serialDebugMode || currentState == STATE_WAIT_START
            || (currentState == STATE_READ_TASK && !taskReceived);
        if (accepting) {
            currentTask = task;
            taskReceived = true;
            roundProgress = 0;
            discMaterialColor = 0;
        }
        portEXIT_CRITICAL(&taskCodeMux);
        if (!accepting) { serialError(frame, "BUSY"); return; }
        Serial.printf("{RSP,TASK,SET,OK,%s}\n", frame.fields[3]);
        updateDisplay("DISPLAY", "TASK_CODE", frame.fields[3]);
    } else if (frame.is("SYS", "RELEASE", 0)) {
        serialRelease(frame);
    } else if (frame.is("SYS", "READY", 0)) {
        if (serialDebugMode) { serialError(frame, "MODE"); return; }
        nano_ready = true;
        serialReply(frame, "OK");
    } else if (frame.is("SYS", "START", 0)) {
        if (serialDebugMode) { serialError(frame, "MODE"); return; }
        enableRun = true;
        serialReply(frame, "OK");
    } else if (frame.is("SYS", "HELP", 0)) {
        Serial.println("[Help] See Document/serial_protocol.md, protocol v2");
        serialReply(frame, "OK");
    } else if (frame.is("POSE", "GET", 0)) {
        sendPoseQueryReply();
    } else if (frame.is("LED", "SET", 1) || frame.is("LED", "GET", 0)
               || frame.is("LED", "FREQ", 1)) {
        uint32_t percent = 0;
        uint32_t hz = 0;
        const bool frequency = strcmp(frame.fields[2], "FREQ") == 0;
        const bool setting = strcmp(frame.fields[2], "SET") == 0;
        if (setting && !serialUInt(frame.fields[3], percent, 100)) {
            serialError(frame, "RANGE"); return;
        }
        if (frequency && (!serialUInt(frame.fields[3], hz, LED_PWM_MAX_FREQUENCY_HZ)
                          || hz < LED_PWM_MIN_FREQUENCY_HZ)) {
            serialError(frame, "RANGE"); return;
        }
        if (!LedPwm_IsReady() || (setting && !LedPwm_SetBrightness(percent))
                || (frequency && !LedPwm_SetFrequency(hz))) {
            serialError(frame, "PWM"); return;
        }
        Serial.printf("{RSP,LED,%s,OK,%u,%lu}\n", frame.fields[2],
                      static_cast<unsigned>(LedPwm_GetBrightness()),
                      static_cast<unsigned long>(LedPwm_GetFrequency()));
    } else if (frame.is("NAV", "START_ZONE", 1)) {
        uint32_t zone = 0;
        if (!serialUInt(frame.fields[3], zone, 2) || zone < 1) {
            serialError(frame, "RANGE"); return;
        }
        currentStartZone = static_cast<StartZone>(zone);
        if (serialDebugMode) currentPose = zone == 1 ? RobotPose{2250, 150, START_ZONE_HEADING}
                                                      : RobotPose{150, 150, START_ZONE_HEADING};
        Serial.printf("{RSP,NAV,START_ZONE,OK,%u}\n", static_cast<unsigned>(zone));
    } else if (frame.starts("NAV", "ROUTE")) {
        serialRoute(frame);
    } else if (frame.is("VISION", "START_REQUEST", 1)
               || frame.is("VISION", "STOP_REQUEST", 1)) {
        VisionStartMode mode;
        const char *name = frame.fields[3];
        if (strcmp(name, "DISC") == 0) mode = VisionStartMode::DISC;
        else if (strcmp(name, "DISC_MATERIAL") == 0) mode = VisionStartMode::DISC_MATERIAL;
        else if (strcmp(name, "WORK_AREA") == 0) mode = VisionStartMode::WORK_AREA;
        else if (strcmp(name, "WORK_AREA_LOADED") == 0) mode = VisionStartMode::WORK_AREA_LOADED;
        else if (strcmp(name, "CORNER") == 0) mode = VisionStartMode::CORNER;
        else { serialError(frame, "RANGE"); return; }
        // 两种运行模式都允许请求切换；ACK 仅表示接令，事件才表示主控已发出请求。
        Serial.printf("{RSP,VISION,%s,ACK,%s}\n", frame.fields[2], name);
        if (strcmp(frame.fields[2], "START_REQUEST") == 0) {
            if (mode == VisionStartMode::WORK_AREA || mode == VisionStartMode::WORK_AREA_LOADED) {
                const RobotState startingState = currentState;
                setAlignmentEnabled(false);
                const bool looked = InitArm_look();
                if (!looked || taskMotionAborted || currentState != startingState) {
                    Serial.printf("{EVT,VISION,START_FAILED,%s,ARM}\n", name);
                    return;
                }
            }
            requestVisionStart(mode);
        } else requestVisionStop(mode);
    } else if (frame.is("VISION", "ALIGN_START", 0)
               || frame.is("VISION", "ALIGN_START", 1)) {
        VisionStartMode mode = VisionStartMode::NONE; // 自动期间沿用当前模式；否则旧命令使用 DISC。
        if (frame.count == 4) {
            const char *name = frame.fields[3];
            if (strcmp(name, "DISC") == 0) mode = VisionStartMode::DISC;
            else if (strcmp(name, "DISC_MATERIAL") == 0) mode = VisionStartMode::DISC_MATERIAL;
            else if (strcmp(name, "WORK_AREA") == 0) mode = VisionStartMode::WORK_AREA;
            else if (strcmp(name, "WORK_AREA_LOADED") == 0) mode = VisionStartMode::WORK_AREA_LOADED;
            else if (strcmp(name, "CORNER") == 0) mode = VisionStartMode::CORNER;
            else { serialError(frame, "RANGE"); return; }
        }
        setAlignmentEnabled(true, true, false, mode);
    } else if (frame.is("VISION", "ALIGN_STOP", 0)) {
        setAlignmentEnabled(false, true);
    } else if (frame.is("VISION", "ALIGN_DATA", 3)) {
        const float low[] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
        const float high[] = {FLT_MAX, FLT_MAX, FLT_MAX};
        float value[3];
        if (!serialFloats(frame, value, low, high, 3)) {
            xSemaphoreTake(xAlignmentMotionMutex, portMAX_DELAY);
            OmniMove(0.0f, 0.0f, 0.0f, 0);
            ResetDiscAlignmentPid();
            ResetAlignmentWait();
            if (xAlignmentQueue) xQueueReset(xAlignmentQueue);
            ++alignmentGeneration; // 已出队的旧帧也不能延续连续计数。
            xSemaphoreGive(xAlignmentMotionMutex);
            serialError(frame, "RANGE"); return;
        }
        xSemaphoreTake(xAlignmentMotionMutex, portMAX_DELAY);
        VisualAlignmentFrame_t alignment = {value[0], value[1], value[2], ++alignmentFrameSequence};
        if (alignmentEnabled && xAlignmentQueue) {
            xQueueOverwrite(xAlignmentQueue, &alignment);
        } else {
            // 断流/超时/手动停止后不允许反馈自行重启运动；限频提示被丢弃的原因。
            static uint32_t lastIgnoredMs = 0;
            const uint32_t now = millis();
            if (now - lastIgnoredMs >= 1000) {
                Serial.println("{EVT,VISION,ALIGN_IGNORED,DISABLED}");
                lastIgnoredMs = now;
            }
        }
        xSemaphoreGive(xAlignmentMotionMutex);
    } else if (frame.is("VISION", "COLOR", 1)) {
        uint32_t color = 0;
        if (!serialUInt(frame.fields[3], color, 6) || color == 0) {
            serialError(frame, "RANGE"); return;
        }
        if (!discMaterialActive) { serialError(frame, "NOT_ACTIVE"); return; }
        // 抓取、放料及恢复观察姿态期间丢弃颜色，不更新缓存，也不触发抓取。
        // 仍按协议确认收帧；恢复处理后只使用新收到的颜色。
        if (!serialDebugMode
                && (currentState == STATE_GRAB_ROUND1 || currentState == STATE_GRAB_ROUND2)
                && !firstDiscGrabReady) {
            serialReply(frame, "OK");
            return;
        }
        discMaterialColor = static_cast<uint8_t>(color);
        serialReply(frame, "OK");
        serialDiscColorDecision(static_cast<uint8_t>(color));
    } else if (frame.is("VISION", "GRAB", 1) || frame.is("VISION", "GRAB", 2)) {
        serialDiscGrab(frame);
    } else if (frame.is("VISION", "FORCE_GRAB", 2)) {
        uint32_t color = 0, cargo = 0;
        if (!serialUInt(frame.fields[3], color, 6) || color == 0
                || !serialUInt(frame.fields[4], cargo, 3) || cargo == 0) {
            serialError(frame, "RANGE"); return;
        }
        Serial.printf("{RSP,VISION,FORCE_GRAB,ACK,%lu,%lu}\n", color, cargo);
        // 手动强制入口不依赖相机和任务状态；先停闭环，避免与抓取中的底盘动作冲突。
        setAlignmentEnabled(false);
        GrabDiscMaterial(static_cast<uint8_t>(color), static_cast<uint8_t>(cargo), true);
        // 强制入口不推进任务；REQUESTED 仅表示已调用并返回。
        Serial.printf("{EVT,VISION,FORCE_GRAB,REQUESTED,%lu,%lu}\n", color, cargo);
    } else if (frame.is("VISION", "CONFIRM", 0)) {
        if (serialDebugMode) { serialError(frame, "MODE"); return; }
        VisualCmd_t command = {"OK", 0, 0, 0};
        if (xVisualTaskQueue) xQueueSend(xVisualTaskQueue, &command, 0);
        serialReply(frame, "ACK");
    } else if (frame.is("VISION", "LIDAR_POSE", 0)) {
        runLidarPoseAction(true);
    } else if (frame.starts("CFG", "GET") || frame.starts("CFG", "BEGIN")
               || frame.starts("CFG", "SET") || frame.starts("CFG", "COMMIT")) {
        if (alignmentEnabled) { serialCfgError(frame, "BUSY_OR_MODE"); return; }
        if (xAlignmentMotionMutex && xSemaphoreTake(xAlignmentMotionMutex, portMAX_DELAY) == pdTRUE) {
            const char *arguments[SERIAL_MAX_FIELDS];
            for (size_t i = 2; i < frame.count; ++i) arguments[i - 2] = frame.fields[i];
            HandleRuntimeParameters(arguments, frame.count - 2, serialDebugMode);
            xSemaphoreGive(xAlignmentMotionMutex);
        } else serialCfgError(frame, "LOCK");
    } else if (frame.is("POSE", "SET", 3) || frame.is("POSE", "GOTO_REL", 3)) {
        // 2. Debug 运动命令：先校验全部数值，再回复 ACK 和执行。
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }
        float values[3];
        const bool set = strcmp(frame.fields[2], "SET") == 0;
        const float minimum[] = {set ? 0.0f : -3394.113f, set ? 0.0f : -3394.113f,
                                 set ? -FLT_MAX : -360.0f};
        const float maximum[] = {set ? 2400.0f : 3394.113f, set ? 2400.0f : 3394.113f,
                                 set ? FLT_MAX : 360.0f};
        if (!serialFloats(frame, values, minimum, maximum, 3)) {
            serialError(frame, "RANGE"); return;
        }
        if (set) currentPose = {values[0], values[1], values[2]};
        Serial.printf("{RSP,POSE,%s,ACK,%.3f,%.3f,%.3f}\n", frame.fields[2],
                      values[0], values[1], values[2]);
        if (!set) GotoPose(values[0], values[1], values[2], true);
    } else if (frame.is("CHASSIS", "MOVE_POSITION", 4)) {
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }
        const float minimum[] = {-3394.113f, -3394.113f, -360.0f, 1.0f};
        const float maximum[] = {3394.113f, 3394.113f, 360.0f, 5000.0f};
        float values[4];
        if (!serialFloats(frame, values, minimum, maximum, 4)) {
            serialError(frame, "RANGE"); return;
        }
        // 关闭上电默认视觉对齐，再独占底盘运动锁，避免 PID 命令覆盖位置命令。
        setAlignmentEnabled(false);
        if (!xAlignmentMotionMutex
                || xSemaphoreTake(xAlignmentMotionMutex, portMAX_DELAY) != pdTRUE) {
            serialError(frame, "LOCK"); return;
        }
        Serial.printf("{RSP,CHASSIS,MOVE_POSITION,ACK,%.3f,%.3f,%.3f,%.3f}\n",
                      values[0], values[1], values[2], values[3]);
        const bool ok = MovePosition(values[0], values[1], values[2], values[3]);
        xSemaphoreGive(xAlignmentMotionMutex);
        Serial.println(ok ? "{EVT,CHASSIS,MOVE_POSITION,DONE,ESTIMATED}"
                          : "{EVT,CHASSIS,MOVE_POSITION,FAILED}");
    } else if (frame.is("CHASSIS", "MOVE", 3)) {
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }
        uint32_t direction = 0, stop = 0;
        float speed = 0;
        if (!serialUInt(frame.fields[3], direction, 5)
                || !serialFloat(frame.fields[4], speed, 0, 1000)
                || !serialUInt(frame.fields[5], stop, 1)) {
            serialError(frame, "RANGE"); return;
        }
        Serial.printf("{RSP,CHASSIS,MOVE,ACK,%u,%.3f,%u}\n",
                      static_cast<unsigned>(direction), speed, static_cast<unsigned>(stop));
        MovePose(direction, speed, stop != 0);
    } else if (frame.is("ARM", "INIT_START", 0) || frame.is("ARM", "INIT_LOOK", 0)
            || frame.is("ARM", "INIT_LOOK2", 0)) {
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }
        serialReply(frame, "ACK");
        const bool ok = strcmp(frame.fields[2], "INIT_START") == 0
            ? InitArm_start()
            : (strcmp(frame.fields[2], "INIT_LOOK") == 0 ? InitArm_look() : InitArm_look2());
        Serial.printf("{EVT,ARM,%s,%s}\n", frame.fields[2], ok ? "ISSUED" : "FAILED");
    } else if (frame.is("ARM", "MOVE1", 3) || frame.is("ARM", "MOVE2", 3)) {
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }
        const bool first = strcmp(frame.fields[2], "MOVE1") == 0;
        const float minimum[] = {first ? -1.0f : TURRET_CABLE_MIN_DEG, first ? -1.0f : -360.0f, 1.0f};
        const float maximum[] = {first ? ARM_HEIGHT_LIMIT_MM : 360.0f, first ? 170.0f : 360.0f, 1000.0f};
        float values[3];
        if (!serialFloats(frame, values, minimum, maximum, 3)
                || (first && ((values[0] < 0 && values[0] != -1)
                    || (values[1] < 0 && values[1] != -1)))) {
            serialError(frame, "RANGE"); return;
        }
        Serial.printf("{RSP,ARM,%s,ACK,%.3f,%.3f,%.3f}\n", frame.fields[2],
                      values[0], values[1], values[2]);
        const bool ok = first
            ? MoveArm(values[0], values[1], -1, -1, values[2])
            : MoveArm(-1, -1, values[0], values[1], values[2]);
        Serial.printf("{EVT,ARM,%s,%s}\n", frame.fields[2], ok ? "ISSUED" : "FAILED");
    } else if (frame.is("ARM", "SERVO", 2)) {
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }
        uint32_t id = 0;
        float angle = 0;
        if (!serialUInt(frame.fields[3], id, 254)
                || id == SERVO_ID_BROADCAST
                || !serialFloat(frame.fields[4], angle, -135, 135)) {
            serialError(frame, "RANGE"); return;
        }
        Serial.printf("{RSP,ARM,SERVO,ACK,%u,%.3f}\n", static_cast<unsigned>(id), angle);
        if (id == 2) {
            const bool ok = MoveArm(-1, -1, angle, -1, 150);
            Serial.printf("{EVT,ARM,SERVO,%s}\n", ok ? "ISSUED" : "FAILED");
        } else {
            Servo_SetAngle(static_cast<uint8_t>(id), angle, 0, 0);
        }
    } else if (frame.is("ARM", "ENABLE", 1)) {
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }
        uint32_t enabled = 0;
        if (!serialUInt(frame.fields[3], enabled, 1)) { serialError(frame, "RANGE"); return; }
        Serial.printf("{RSP,ARM,ENABLE,ACK,%u}\n", static_cast<unsigned>(enabled));
        Emm_V5_En_Control_all(enabled);
    } else if (frame.is("ARM", "DEMO1", 0) || frame.is("ARM", "DEMO2", 0)
               || frame.is("ARM", "DEMO3", 0) || frame.is("ARM", "DEMO4", 0)) {
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }
        serialReply(frame, "ACK");
        const bool ok = strcmp(frame.fields[2], "DEMO1") == 0 ? DemoCargoToRoughArea()
            : strcmp(frame.fields[2], "DEMO2") == 0 ? DemoStackCargoToWorkArea()
            : strcmp(frame.fields[2], "DEMO3") == 0 ? DemoStackCargoToWorkArea3()
            : DemoWorkAreaToCargo();
        Serial.printf("{EVT,ARM,%s,%s}\n", frame.fields[2], ok ? "DONE" : "FAILED");
    } else if (frame.count >= 3) {
        serialError(frame, serialKnownAction(frame) ? "FORMAT" : "UNKNOWN_ACTION");
    }
}

void Task_Serial_CMD(void *pvParameters) {
    // 1. 只有本任务读取 Serial0；Debug/Release 共用缓冲区和收帧规则。
    char line[SERIAL_FRAME_BYTES];
    size_t length = 0;
    bool overflow = false;
    bool invalid = false;
    bool discResumeDraining = false;
    size_t discOldBytesRemaining = 0;
    // 固定旧数据的边界，持续新来帧不能无限延长屏蔽时间。
    // 旧数据仍正常分发（COLOR 在未 ready 时丢弃），非颜色指令不受影响。
    const auto resumeDiscAtFrameBoundary = [&]() {
        if (discMessageResumeRequested && !discResumeDraining) {
            discOldBytesRemaining = static_cast<size_t>(Serial.available());
            discResumeDraining = true;
            discMessageResumeRequested = false;
        }
        if (!discResumeDraining || discOldBytesRemaining != 0
                || length != 0 || overflow || invalid) return;
        discResumeDraining = false;
        discMaterialColor = 0;
        const int progress = roundProgress;
        if (!serialDebugMode
                && (currentState == STATE_GRAB_ROUND1 || currentState == STATE_GRAB_ROUND2)
                && discMaterialActive && progress >= 0 && progress < 3) {
            firstDiscGrabReady = true;
            Serial.printf("[DISC] Color handling resumed: expected=%d, cargo=%d\n",
                          (currentState == STATE_GRAB_ROUND2 ? currentTask.round2_colors
                                                               : currentTask.round1_colors)[progress], progress + 1);
        }
    };
    for (;;) {
        resumeDiscAtFrameBoundary();
        while (Serial.available() > 0) {
            // 收帧/分发期间不是空闲；同时取消尚未处理的按键请求。
            serialBootSetIdle(false);
            // 请求可能在读取持续数据期间到达；先记录旧缓存再读下一字节。
            resumeDiscAtFrameBoundary();
            const char c = static_cast<char>(Serial.read());
            if (discResumeDraining && discOldBytesRemaining > 0) --discOldBytesRemaining;
            if (c == '\n' || c == '\r') {
                // 2. 完整行到达后才解析。过长或含 NUL 的整行直接拒绝。
                if (overflow) Serial.println("{RSP,SYS,FRAME,ERR,TOO_LONG}");
                else if (invalid) Serial.println("{RSP,SYS,FRAME,ERR,FORMAT}");
                else if (length > 0) {
                    line[length] = '\0';
                    SerialFrame frame;
                    // 3. 分发层只看到已经通过基础结构检查的 CMD 帧。
                    if (parseSerialFrame(line, frame)) serialDispatch(frame);
                    else Serial.println("{RSP,SYS,FRAME,ERR,FORMAT}");
                }
                length = 0;
                overflow = false;
                invalid = false;
            } else if (c == '\0') {
                invalid = true;
            } else if (!overflow) {
                if (length < sizeof(line) - 1) line[length++] = c;
                else overflow = true; // 丢弃整行剩余字节，不执行截断后的命令。
            }
        }
        // 无新数据时也可以恢复；持续数据时由上面的帧边界检查恢复。
        resumeDiscAtFrameBoundary();
        serialBootReleaseAtIdle(length == 0 && !overflow && !invalid && Serial.available() == 0);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
