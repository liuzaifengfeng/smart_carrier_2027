// 由 main.cpp 在业务状态与动作函数之后包含。仅此任务读取 Serial0。
#include <float.h>
static bool serialDebugMode = true;

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
        {"NAV", "START_ZONE"}, {"NAV", "ROUTE"},
        {"VISION", "ALIGN_START"}, {"VISION", "ALIGN_STOP"},
        {"VISION", "START_REQUEST"},
        {"VISION", "ALIGN_DATA"}, {"VISION", "COLOR"},
        {"VISION", "CONFIRM"}, {"VISION", "LIDAR_POSE"},
        {"POSE", "GET"}, {"POSE", "SET"}, {"POSE", "GOTO_REL"},
        {"CHASSIS", "MOVE"},
        {"ARM", "MOVE1"}, {"ARM", "MOVE2"}, {"ARM", "SERVO"},
        {"ARM", "ENABLE"}, {"ARM", "DEMO1"}, {"ARM", "DEMO2"}, {"ARM", "DEMO3"},
        {"ARM", "DEMO4"},
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
    setAlignmentEnabled(false);
    xHomeTimer = xTimerCreate("HomeTimer", pdMS_TO_TICKS(300000), pdFALSE,
                             NULL, vHomeTimerCallback);
    if (!xHomeTimer) { serialError(frame, "TIMER"); return; }
    currentState = STATE_WAIT_START;
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

static void serialDispatch(const SerialFrame &frame) {
    // 每个分支必须先完整匹配动作及参数数量，再读取参数或调用运动函数。
    // 1. 通用命令与共享业务状态。
    if (frame.is("SYS", "HELLO", 0)) {
        Serial.println("{RSP,SYS,HELLO,OK,2}");
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
    } else if (frame.is("NAV", "START_ZONE", 1)) {
        uint32_t zone = 0;
        if (!serialUInt(frame.fields[3], zone, 2) || zone < 1) {
            serialError(frame, "RANGE"); return;
        }
        currentStartZone = static_cast<StartZone>(zone);
        if (serialDebugMode) currentPose = zone == 1 ? RobotPose{2250, 150, 180}
                                                      : RobotPose{150, 150, 0};
        Serial.printf("{RSP,NAV,START_ZONE,OK,%u}\n", static_cast<unsigned>(zone));
    } else if (frame.starts("NAV", "ROUTE")) {
        serialRoute(frame);
    } else if (frame.is("VISION", "START_REQUEST", 1)) {
        VisionStartMode mode;
        const char *name = frame.fields[3];
        if (strcmp(name, "DISC") == 0) mode = VisionStartMode::DISC;
        else if (strcmp(name, "DISC_MATERIAL") == 0) mode = VisionStartMode::DISC_MATERIAL;
        else if (strcmp(name, "WORK_AREA") == 0) mode = VisionStartMode::WORK_AREA;
        else if (strcmp(name, "WORK_AREA_LOADED") == 0) mode = VisionStartMode::WORK_AREA_LOADED;
        else if (strcmp(name, "CORNER") == 0) mode = VisionStartMode::CORNER;
        else { serialError(frame, "RANGE"); return; }
        // 两种运行模式都允许请求切换；ACK 仅表示接令，事件才表示主控已发出请求。
        Serial.printf("{RSP,VISION,START_REQUEST,ACK,%s}\n", name);
        requestVisionStart(mode);
    } else if (frame.is("VISION", "ALIGN_START", 0)) {
        setAlignmentEnabled(true, true);
    } else if (frame.is("VISION", "ALIGN_STOP", 0)) {
        setAlignmentEnabled(false, true);
    } else if (frame.is("VISION", "ALIGN_DATA", 3)) {
        const float low[] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
        const float high[] = {FLT_MAX, FLT_MAX, FLT_MAX};
        float value[3];
        if (!serialFloats(frame, value, low, high, 3)) {
            serialError(frame, "RANGE"); return;
        }
        VisualAlignmentFrame_t alignment = {value[0], value[1], value[2]};
        if (alignmentEnabled && xAlignmentQueue) xQueueOverwrite(xAlignmentQueue, &alignment);
    } else if (frame.is("VISION", "COLOR", 1)) {
        if (serialDebugMode) { serialError(frame, "MODE"); return; }
        float color = 0;
        if (!serialFloat(frame.fields[3], color, -FLT_MAX, FLT_MAX)) {
            serialError(frame, "RANGE"); return;
        }
        VisualCmd_t command = {"COLOR", color, 0, 0};
        if (xVisualTaskQueue) xQueueSend(xVisualTaskQueue, &command, 0);
        serialReply(frame, "OK");
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
    } else if (frame.is("CHASSIS", "MOVE", 3)) {
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }
        uint32_t direction = 0, stop = 0;
        float speed = 0;
        if (!serialUInt(frame.fields[3], direction, 3)
                || !serialFloat(frame.fields[4], speed, 0, 1000)
                || !serialUInt(frame.fields[5], stop, 1)) {
            serialError(frame, "RANGE"); return;
        }
        Serial.printf("{RSP,CHASSIS,MOVE,ACK,%u,%.3f,%u}\n",
                      static_cast<unsigned>(direction), speed, static_cast<unsigned>(stop));
        MovePose(direction, speed, stop != 0);
    } else if (frame.is("ARM", "MOVE1", 3) || frame.is("ARM", "MOVE2", 3)) {
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }
        const bool first = strcmp(frame.fields[2], "MOVE1") == 0;
        const float minimum[] = {first ? -1.0f : -360.0f, first ? -1.0f : -360.0f, 1.0f};
        const float maximum[] = {first ? 160.0f : 360.0f, first ? 170.0f : 360.0f, 1000.0f};
        float values[3];
        if (!serialFloats(frame, values, minimum, maximum, 3)
                || (first && ((values[0] < 0 && values[0] != -1)
                    || (values[1] < 0 && values[1] != -1)))) {
            serialError(frame, "RANGE"); return;
        }
        Serial.printf("{RSP,ARM,%s,ACK,%.3f,%.3f,%.3f}\n", frame.fields[2],
                      values[0], values[1], values[2]);
        if (first) MoveArm(values[0], values[1], -1, -1, values[2]);
        else MoveArm(-1, -1, values[0], values[1], values[2]);
    } else if (frame.is("ARM", "SERVO", 2)) {
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }
        uint32_t id = 0;
        float angle = 0;
        if (!serialUInt(frame.fields[3], id, 254)
                || !serialFloat(frame.fields[4], angle, -135, 135)) {
            serialError(frame, "RANGE"); return;
        }
        Serial.printf("{RSP,ARM,SERVO,ACK,%u,%.3f}\n", static_cast<unsigned>(id), angle);
        Servo_SetAngle(static_cast<uint8_t>(id), angle, 0, 0);
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
    for (;;) {
        while (Serial.available() > 0) {
            const char c = static_cast<char>(Serial.read());
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
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
