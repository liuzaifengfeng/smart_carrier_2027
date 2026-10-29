// 由 robot_runtime.cpp 在业务状态与动作函数之后包含。仅此任务读取 Serial0。
#include <float.h>
#include "debug_release_button.h"
#include "h30_usb.h"
#include "continuous_trial.h"
#include "disc_position_grab.h"
#include "motion_executor.h"
#include "node_position_hardware.h"
#include "motor_bus.h"
static bool serialDebugMode = true;
/**
 * @brief 处理连续路径试跑配置、准备、启动、状态和停止指令；返回是否已匹配处理。
 */
static bool serialTrial(const SerialFrame &frame);
static DebugReleaseButton bootReleaseButton;
static portMUX_TYPE bootReleaseMux = portMUX_INITIALIZER_UNLOCKED;

// loop独立采样，串口任务执行阻塞动作时也能识别并忽略按键。
/**
 * @brief 采样板载BOOT按钮并记录Release请求，实际切换由空闲串口任务处理。
 */
void PollDebugBootButton() {
    const bool pressed = digitalRead(BOOT0_PIN) == LOW;
    const uint32_t nowMs = millis();
    portENTER_CRITICAL(&bootReleaseMux);
    bootReleaseButton.update(pressed, nowMs);
    portEXIT_CRITICAL(&bootReleaseMux);
}

/**
 * @brief 更新按钮Release可用状态；只有调试空闲且试跑、运动执行器均空闲时允许触发。
 */
static void serialBootSetIdle(bool idle) {
    portENTER_CRITICAL(&bootReleaseMux);
    bootReleaseButton.setIdle(serialDebugMode && idle && !ContinuousTrialActive() && !MotionExecutorActive());
    portEXIT_CRITICAL(&bootReleaseMux);
}
/**
 * @brief 按原请求类别和动作发送指定状态的RSP串口应答。
 */
static void serialReply(const SerialFrame &frame, const char *status) {
    Serial.printf("{RSP,%s,%s,%s}\n", frame.fields[1], frame.fields[2], status);
}

/**
 * @brief 复制串口应答上下文并暂存运动作业，等待ACK后放行；返回作业编号，0表示未提交。
 */
static uint32_t serialPrepareMotion(const SerialFrame &frame,MotionRequest &request) {
    snprintf(request.category,sizeof(request.category),"%s",frame.fields[1]);
    snprintf(request.command,sizeof(request.command),"%s",frame.fields[2]);
    const uint32_t id=SubmitMotion(request,false);
    if(!id)serialError(frame,"BUSY");
    return id;
}
/**
 * @brief 准备运动作业，先发ACK，再通过LaunchMotion允许工作任务执行。
 */
static void serialMotion(const SerialFrame &frame,MotionRequest &request) {
    const uint32_t id=serialPrepareMotion(frame,request);if(!id)return;
    serialReply(frame,"ACK");LaunchMotion(id);
}

/**
 * @brief 发送参数配置请求的错误应答，reason为错误原因。
 */
static void serialCfgError(const SerialFrame &frame, const char *reason) {
    uint32_t id = 0;
    if (frame.count > 3) serialUInt(frame.fields[3], id, UINT32_MAX);
    Serial.printf("{RSP,CFG,%s,ERR,%lu,%s}\n", frame.fields[2],
                  static_cast<unsigned long>(id), reason);
}

/**
 * @brief 检查帧中的类别和动作名是否属于已支持的串口指令。
 */
static bool serialKnownAction(const SerialFrame &frame) {
    static const char *const names[][2] = {
        {"SYS", "HELLO"}, {"SYS", "RELEASE"}, {"SYS", "READY"},
        {"SYS", "START"}, {"SYS", "HELP"},
        {"MOTION", "STATUS"},
        {"EMM", "AXIS_TRIAL"},
        {"TASK", "SET"},
        {"LED", "SET"}, {"LED", "GET"}, {"LED", "FREQ"},
        {"NAV", "START_ZONE"}, {"NAV", "ROUTE"}, {"NAV", "STOP"},
        {"NAV", "TRIAL_BEGIN"}, {"NAV", "TRIAL_POLICY"}, {"NAV", "TRIAL_NODE"}, {"NAV", "TRIAL_START"},
        {"NAV", "TRIAL_STATUS"}, {"NAV", "TRIAL_STOP"},
        {"VISION", "ALIGN_START"}, {"VISION", "ALIGN_STOP"},
        {"VISION", "START_REQUEST"},
        {"VISION", "STOP_REQUEST"},
        {"VISION", "ALIGN_DATA"}, {"VISION", "COLOR"},
        {"VISION", "GRAB"},
        {"VISION", "DISC_POSITIONS"},
        {"VISION", "FORCE_GRAB"},
        {"VISION", "CONFIRM"}, {"VISION", "LIDAR_POSE"},
        {"POSE", "GET"}, {"POSE", "SET"}, {"POSE", "GOTO_REL"},
        {"IMU", "GET"},
        {"IMU", "HEADING"},
        {"CHASSIS", "MOVE"}, {"CHASSIS", "MOVE_POSITION"},
        {"ARM", "MOVE1"}, {"ARM", "MOVE2"}, {"ARM", "SERVO"}, {"ARM", "SERVO_ZERO"},
        {"ARM", "SERVO_PID_GET"}, {"ARM", "SERVO_PID_SET"},
        {"ARM", "ENABLE"}, {"ARM", "DEMO1"}, {"ARM", "DEMO2"}, {"ARM", "DEMO3"},
        {"ARM", "DEMO4"}, {"ARM", "TRANSFER"}, {"ARM", "RETRIEVE"},
        {"ARM", "INIT_START"}, {"ARM", "INIT_LOOK"}, {"ARM", "INIT_LOOK2"},
        {"CFG", "GET"}, {"CFG", "BEGIN"}, {"CFG", "SET"}, {"CFG", "COMMIT"},
    };
    for (const auto &name : names) {
        if (strcmp(frame.fields[1], name[0]) == 0
                && strcmp(frame.fields[2], name[1]) == 0) return true;
    }
    return false;
}

/**
 * @brief 按各字段范围解析一组浮点参数；失败时输出相应协议错误并返回false。
 */
static bool serialFloats(const SerialFrame &frame, float *values, const float *minimum, const float *maximum, size_t count) {
    if (frame.count != count + 3) return false;
    for (size_t i = 0; i < count; ++i) {
        if (!serialFloat(frame.fields[i + 3], values[i], minimum[i], maximum[i])) return false;
    }
    return true;
}

/**
 * @brief 处理节点路径串口请求，区分自动路径回传与调试路径执行。
 */
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
    if(serialDebugMode) {
        MotionRequest r;r.kind=MotionKind::NODE_PATH;r.pathCount=count;r.theta=NAN;r.speed=NODE_PATH_SPEED_MM_S;r.automatic=true;
        memcpy(r.path,path,count);const uint32_t id=SubmitMotion(r,false);
        if(!id){serialError(frame,"BUSY");return;}
        Serial.printf("{RSP,NAV,ROUTE,ACK,%u}\n",static_cast<unsigned>(count));LaunchMotion(id);return;
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

/**
 * @brief 处理进入正式运行模式，重置比赛准备状态并启动或恢复主状态机任务。
 */
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

/**
 * @brief 在串口空闲时消费板载按钮的Release请求，复用正式模式切换入口。
 */
static void serialBootReleaseAtIdle(bool idle) {
    portENTER_CRITICAL(&bootReleaseMux);
    bootReleaseButton.setIdle(serialDebugMode && idle && !ContinuousTrialActive() && !MotionExecutorActive());
    const bool requested = bootReleaseButton.takeRequest();
    portEXIT_CRITICAL(&bootReleaseMux);
    if (!requested) return;
    // 与串口命令同一个任务、同一个入口，避免并发创建定时器/主状态机。
    char command[] = "{CMD,SYS,RELEASE}";
    SerialFrame frame;
    if (parseSerialFrame(command, frame)) serialRelease(frame);
}

// 串口只提交；ACK、选择事件先于 Launch，实际完成及进度由运动任务发布。
/**
 * @brief 构造并提交圆盘抓放请求，附颜色、载物台、位置及视觉应答上下文；返回作业编号。
 */
static uint32_t serialQueueDiscGrab(uint8_t color,uint8_t cargo,uint8_t position=0,uint8_t ack=0,size_t count=0,bool force=false) {
    MotionRequest r;r.kind=MotionKind::DISC_GRAB;r.color=color;r.cargo=cargo;r.position=position;r.force=force;
    r.automatic=!force&&!serialDebugMode&&(currentState==STATE_GRAB_ROUND1||currentState==STATE_GRAB_ROUND2);
    const uint32_t id=SubmitMotion(r,false);if(!id)return 0;
    if(r.automatic) {
        firstDiscGrabReady=false;discMessageResumeRequested=false;
        traceVision(VisionStartMode::DISC_MATERIAL,false);
        if(position)requestVisionStop(VisionStartMode::DISC_MATERIAL);
    }
    discMaterialColor=0;
    if(ack==1)Serial.printf("{RSP,VISION,GRAB,ACK,%u,%u}\n",color,cargo);
    else if(ack==2) {
        Serial.printf("{RSP,VISION,DISC_POSITIONS,ACK,%u}\n",static_cast<unsigned>(count));
        Serial.printf("{EVT,VISION,DISC_POSITIONS,SELECTED,%u,%u,%u}\n",position,color,cargo);
    } else if(ack==3) {
        Serial.printf("{RSP,VISION,FORCE_GRAB,ACK,%u,%u}\n",color,cargo);
        Serial.printf("{EVT,VISION,FORCE_GRAB,REQUESTED,%u,%u}\n",color,cargo);
    }
    LaunchMotion(id);return id;
}
/**
 * @brief 按任务颜色和目标载物台提交自动圆盘抓放作业。
 */
static void serialExecuteTaskGrab(uint8_t color,uint8_t cargo) {
    serialQueueDiscGrab(color,cargo);
}

/**
 * @brief 按当前任务进度评估识别颜色，满足条件后触发抓取，否则发送拒绝原因。
 */
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
/**
 * @brief 处理串口单件抓取、强制抓取等圆盘指令，并提交对应运动作业。
 */
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
    if(!serialQueueDiscGrab(color,cargo,0,1))serialError(frame,"BUSY");
}

// 视觉保证本帧内全部位置此刻可抓；只选一件，不缓存未选中的候选。
/**
 * @brief 解析一帧圆盘候选位置和颜色，选出下一件任务目标并提交抓取。
 */
static void serialDiscPositions(const SerialFrame &frame) {
    if (frame.count < 5 || frame.count > 9 || (frame.count - 3) % 2 != 0) {
        serialError(frame, "FORMAT"); return;
    }
    DiscPositionCandidate candidates[3] = {};
    const size_t count = (frame.count - 3) / 2;
    for (size_t i = 0; i < count; ++i) {
        uint32_t position = 0, color = 0;
        if (!serialUInt(frame.fields[3 + 2 * i], position, 3) || position == 0
                || !serialUInt(frame.fields[4 + 2 * i], color, 6) || color == 0) {
            serialError(frame, "RANGE"); return;
        }
        for (size_t j = 0; j < i; ++j) {
            if (candidates[j].position == position) {
                serialError(frame, "DUPLICATE_POSITION"); return;
            }
        }
        candidates[i] = {static_cast<uint8_t>(position), static_cast<uint8_t>(color)};
    }
    const RobotState state = currentState;
    // 自动抓放期间已经结束了旧识别会话。优先返回 NOT_READY，明确表示
    // 这是一帧跨越抓取边界的旧位置数据，而不是普通的“功能未开启”。
    if (!serialDebugMode
            && (state == STATE_GRAB_ROUND1 || state == STATE_GRAB_ROUND2)
            && !firstDiscGrabReady) {
        serialError(frame, "NOT_READY"); return;
    }
    if (!discMaterialActive) { serialError(frame, "NOT_ACTIVE"); return; }
    const int progress = roundProgress;
    const int *colors = state == STATE_GRAB_ROUND2 ? currentTask.round2_colors
        : state == STATE_GRAB_ROUND1 || serialDebugMode ? currentTask.round1_colors : nullptr;
    if (!currentTask.valid) { serialError(frame, "NO_TASK"); return; }
    if (!colors) { serialError(frame, "NOT_WAITING"); return; }
    if (progress < 0 || progress >= 3) { serialError(frame, "ROUND_COMPLETE"); return; }
    if (alignmentEnabled) { serialError(frame, "ALIGN_ACTIVE"); return; }
    const uint8_t cargo = static_cast<uint8_t>(progress + 1);
    if (cargoPlatforms[cargo - 1].material != MATERIAL_NONE) {
        serialError(frame, "OCCUPIED"); return;
    }
    traceVisionFirstFrame(VisionStartMode::DISC_MATERIAL);
    const int selected = SelectDiscPosition(candidates, count, colors, progress, currentTask.valid);
    if(selected<0) {
        Serial.printf("{RSP,VISION,DISC_POSITIONS,ACK,%u}\n",static_cast<unsigned>(count));
        Serial.printf("{EVT,VISION,DISC_POSITIONS,SKIP,%d,%u,COLOR_MISMATCH}\n",colors[progress],cargo);return;
    }
    const auto candidate=candidates[selected];
    if(!serialQueueDiscGrab(candidate.color,cargo,candidate.position,2,count))serialError(frame,"BUSY");
}

/**
 * @brief 处理载物台到工位的单件放料指令，解析编号后提交TRANSFER作业。
 */
static void serialTransfer(const SerialFrame &frame) {
    if (!serialDebugMode) { serialError(frame, "MODE"); return; }
    uint32_t cargoCode = 0, workAreaCode = 0;
    if (!serialUInt(frame.fields[3], cargoCode, MATERIAL_STATION_COUNT) || cargoCode == 0
            || !serialUInt(frame.fields[4], workAreaCode, MATERIAL_STATION_COUNT) || workAreaCode == 0) {
        serialError(frame, "RANGE"); return;
    }
    MotionRequest r;r.kind=MotionKind::TRANSFER;r.cargo=cargoCode;r.work=workAreaCode;r.debug=true;
    serialMotion(frame,r);
}
/**
 * @brief 处理工位到载物台的单件取回指令，解析编号后提交RETRIEVE作业。
 */
static void serialRetrieve(const SerialFrame &frame) {
    if(!serialDebugMode){serialError(frame,"MODE");return;}
    uint32_t work=0,cargo=0;
    if(!serialUInt(frame.fields[3],work,MATERIAL_STATION_COUNT)||!work
        ||!serialUInt(frame.fields[4],cargo,MATERIAL_STATION_COUNT)||!cargo){serialError(frame,"RANGE");return;}
    if(cargoPlatforms[cargo-1].material!=MATERIAL_NONE){serialError(frame,"OCCUPIED");return;}
    MotionRequest r;r.kind=MotionKind::RETRIEVE;r.cargo=cargo;r.work=work;r.color=MATERIAL_RED;
    serialMotion(frame,r);
}

/**
 * @brief 串口业务总分发入口：按类别和动作处理系统、视觉、底盘、机械臂及参数请求。
 */
static void serialDispatch(const SerialFrame &frame) {
    const bool motionStop=frame.is("CHASSIS","MOVE",3)&&strcmp(frame.fields[5],"1")==0;
    if(motionStop) {
        uint32_t direction=0,stop=0;float speed=0;
        if(!serialUInt(frame.fields[3],direction,5)||!serialFloat(frame.fields[4],speed,0,1000)
            ||!serialUInt(frame.fields[5],stop,1)){serialError(frame,"RANGE");return;}
        CancelMotion();Serial.printf("{RSP,CHASSIS,MOVE,ACK,%u,%.3f,1}\n",static_cast<unsigned>(direction),speed);return;
    }
    if(MotionExecutorActive()&&!frame.is("IMU","GET",0)&&!frame.is("IMU","HEADING",0)
        &&!frame.is("POSE","GET",0)&&!frame.starts("CFG","GET")&&!frame.is("MOTION","STATUS",1)
        &&!frame.is("NAV","TRIAL_STATUS",1)&&!frame.is("NAV","TRIAL_STOP",1)&&!frame.is("NAV","STOP",0)
        &&!frame.starts("NAV","ROUTE")&&!frame.is("NAV","START_ZONE",1)
        &&!frame.is("SYS","HELLO",0)&&!frame.is("SYS","HELP",0)&&!frame.is("SYS","START",0)&&!frame.is("SYS","READY",0)
        &&!frame.is("TASK","SET",1)&&!frame.is("VISION","ALIGN_DATA",3)&&!frame.is("VISION","LIDAR_POSE",0)
        &&!frame.is("VISION","COLOR",1)&&!frame.starts("VISION","DISC_POSITIONS")&&!frame.starts("VISION","GRAB")
        &&!frame.is("VISION","STOP_REQUEST",1)&&!frame.is("VISION","ALIGN_STOP",0)) {
        serialError(frame,"BUSY");return;
    }
    if(serialTrial(frame))return;
    // 每个分支必须先完整匹配动作及参数数量，再读取参数或调用运动函数。
    // 1. 通用命令与共享业务状态。
    if (frame.is("SYS", "HELLO", 0)) {
        Serial.println("{RSP,SYS,HELLO,OK,2}");
    } else if(frame.is("EMM","TURRET_TRIAL",5)) {
        if(!serialDebugMode){serialError(frame,"MODE");return;}
        float angle=0,speed=0;uint32_t ramp=0,period=0,duration=0;
        if(!serialFloat(frame.fields[3],angle,TURRET_CABLE_MIN_DEG,TURRET_CABLE_MAX_DEG)
            ||!serialFloat(frame.fields[4],speed,1,65535.0f*6/TURRET_MOTOR_GEAR_RATIO)
            ||!serialUInt(frame.fields[5],ramp,UINT16_MAX)
            ||!serialUInt(frame.fields[6],period,UINT32_MAX)||!period
            ||!serialUInt(frame.fields[7],duration,UINT32_MAX)||!duration){serialError(frame,"RANGE");return;}
        MotionRequest r;r.kind=MotionKind::TURRET_TRIAL;r.turret=angle;r.speed=speed;
        r.turretTrialRampMs=ramp;r.turretTrialPeriodMs=period;r.turretTrialDurationMs=duration;
        serialMotion(frame,r);
    } else if(frame.is("EMM","AXIS_TRIAL",6)) {
        if(!serialDebugMode){serialError(frame,"MODE");return;}
        uint32_t id=0,rpm=0,ramp=0,period=0,duration=0;float position=0;
        if(!serialUInt(frame.fields[3],id,6)||(id!=5&&id!=6)
            ||!serialFloat(frame.fields[4],position,0,id==5?ARM_HEIGHT_LIMIT_MM:170.0f)
            ||!serialUInt(frame.fields[5],rpm,UINT16_MAX)||!rpm
            ||!serialUInt(frame.fields[6],ramp,UINT16_MAX)
            ||!serialUInt(frame.fields[7],period,UINT32_MAX)||!period
            ||!serialUInt(frame.fields[8],duration,UINT32_MAX)||!duration){serialError(frame,"RANGE");return;}
        MotionRequest r;r.kind=MotionKind::AXIS_TRIAL;r.axisTrialMotor=id;
        if(id==5)r.high=position;else r.length=position;
        r.axisTrialRpm=rpm;r.axisTrialRampMs=ramp;r.axisTrialPeriodMs=period;r.axisTrialDurationMs=duration;
        serialMotion(frame,r);
    } else if(frame.is("EMM","DRIVE_GET",1)||frame.is("EMM","DRIVE_SET",9)) {
        if(!serialDebugMode){serialError(frame,"MODE");return;}
        uint32_t id=0,save=0;
        if(!serialUInt(frame.fields[3],id,7)||!id){serialError(frame,"RANGE");return;}
        uint16_t values[7]={};const char *error=nullptr;
        if(frame.is("EMM","DRIVE_SET",9)) {
            for(uint8_t i=0;i<7;++i){uint32_t value=0;
                if(!serialUInt(frame.fields[4+i],value,UINT16_MAX)){serialError(frame,"RANGE");return;}
                values[i]=value;
            }
            if(!serialUInt(frame.fields[11],save,1)){serialError(frame,"RANGE");return;}
            error=SetMotorDriveParamsVerified(id,values,save);
        } else error=ReadMotorDriveParams(id,values);
        if(error&&strcmp(error,"DRIVE_VERIFY_MISMATCH")!=0){serialError(frame,error);return;}
        Serial.printf("{RSP,EMM,%s,%s,%u,%u,%u,%u,%u,%u,%u,%u}\n",frame.fields[2],error?"READBACK_MISMATCH":"READBACK",unsigned(id),
            values[0],values[1],values[2],values[3],values[4],values[5],values[6]);
    } else if(frame.is("EMM","PID_GET",1)||frame.is("EMM","PID_SET",5)
            ||frame.is("EMM","STATE_GET",1)||frame.is("EMM","MODE_SET",3)
            ||frame.is("EMM","ENABLE",2)||frame.is("EMM","CLEAR_STALL",1)) {
        if(!serialDebugMode){serialError(frame,"MODE");return;}
        uint32_t id=0;
        if(!serialUInt(frame.fields[3],id,7)||!id){serialError(frame,"RANGE");return;}
        MotorBusGuard transaction;
        const char *error=nullptr;
        if(frame.is("EMM","PID_GET",1)||frame.is("EMM","PID_SET",5)) {
            uint32_t gains[3]={},save=0;
            if(frame.is("EMM","PID_SET",5)) {
                for(uint8_t i=0;i<3;++i)if(!serialUInt(frame.fields[4+i],gains[i],UINT32_MAX)){serialError(frame,"RANGE");return;}
                if(!serialUInt(frame.fields[7],save,1)){serialError(frame,"RANGE");return;}
                error=SetMotorPositionPidVerified(id,gains,save);
                if(error){
                    Serial.printf("[EMM] id=%u PID_SET stage=WRITE_VERIFY error=%s\n",unsigned(id),error);
                    serialError(frame,error);return;
                }
            }
            else error=ReadMotorPositionPid(id,gains);
            if(error){
                Serial.printf("[EMM] id=%u %s stage=READBACK error=%s\n",unsigned(id),frame.fields[2],error);
                serialError(frame,error);return;
            }
            Serial.printf("{RSP,EMM,%s,READBACK,%u,%lu,%lu,%lu}\n",frame.fields[2],unsigned(id),
                (unsigned long)gains[0],(unsigned long)gains[1],(unsigned long)gains[2]);
        } else if(frame.is("EMM","STATE_GET",1)) {
            uint16_t encoder=0;int64_t position=0;int16_t rpm=0;uint8_t flags=0;
            error=ReadMotorEncoderSnapshot(id,encoder,position,rpm,flags);
            if(error){serialError(frame,error);return;}
            Serial.printf("{RSP,EMM,STATE_GET,READBACK,%u,%u,%lld,%d,%u}\n",unsigned(id),encoder,(long long)position,rpm,flags);
        } else {
            uint32_t value=0,save=0;uint8_t function=0x0E;
            if(frame.is("EMM","MODE_SET",3)) {
                if(!serialUInt(frame.fields[4],value,3)||!serialUInt(frame.fields[5],save,1)){serialError(frame,"RANGE");return;}
                function=0x46;
            } else if(frame.is("EMM","ENABLE",2)) {
                if(!serialUInt(frame.fields[4],value,1)){serialError(frame,"RANGE");return;}
                function=0xF3;
            }
            error=ConfigureEmmMotor(id,function,value,save);
            if(error){serialError(frame,error);return;}
            Serial.printf("{RSP,EMM,%s,ACK,%u}\n",frame.fields[2],unsigned(id));
        }
    } else if(frame.is("MOTION","STATUS",1)) {
        uint32_t id=0;if(!serialUInt(frame.fields[3],id,UINT32_MAX)){serialError(frame,"RANGE");return;}
        const auto result=GetMotionStatus(id);
        if(!id||!result.id){serialError(frame,"ID");return;}
        Serial.printf("{RSP,MOTION,STATUS,OK,%lu,%s,%s,%lu}\n",static_cast<unsigned long>(id),
            MotionStateName(result.state),result.phase,static_cast<unsigned long>(result.elapsedMs));
    } else if(frame.is("NAV","STOP",0)) {
        CancelMotion();
        Serial.println("{RSP,NAV,STOP,ACK}");
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
    } else if (frame.is("IMU", "GET", 0)) {
        H30Attitude sample;
        uint32_t age = 0;
        const char *error = H30UsbGetAttitude(sample, age);
        if (error) { serialError(frame, error); return; }
        Serial.printf("{RSP,IMU,GET,OK,%.6f,%.6f,%.6f,%lu}\n",
                      sample.pitch, sample.roll, sample.yaw, static_cast<unsigned long>(age));
    } else if (frame.is("IMU", "HEADING", 0)) {
        float heading = 0;
        uint32_t age = 0;
        const char *error = H30UsbGetHeading(heading, age);
        if (error) { serialError(frame, error); return; }
        Serial.printf("{RSP,IMU,HEADING,OK,%.6f,%lu}\n",
                      heading, static_cast<unsigned long>(age));
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
        if (serialDebugMode) {
            currentPose = zone == 1 ? RobotPose{2250, 150, START_ZONE_HEADING}
                                   : RobotPose{150, 150, START_ZONE_HEADING};
            // 只更新地图初始位姿，不覆盖初始化后的3秒平均航向参考。
        }
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
        const bool start=strcmp(frame.fields[2],"START_REQUEST")==0;
        if(start&&(mode==VisionStartMode::WORK_AREA||mode==VisionStartMode::WORK_AREA_LOADED)) {
            MotionRequest r;r.kind=MotionKind::ARM_POSE;r.observation=ArmObservation::LOOK;r.startVision=true;r.vision=static_cast<uint8_t>(mode);
            const uint32_t id=serialPrepareMotion(frame,r);if(!id)return;
            Serial.printf("{RSP,VISION,START_REQUEST,ACK,%s}\n",name);LaunchMotion(id);
        } else {
            Serial.printf("{RSP,VISION,%s,ACK,%s}\n",frame.fields[2],name);
            if(start)requestVisionStart(mode);else requestVisionStop(mode);
        }
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
            alignmentStopPending=true;
            ResetDiscAlignmentPid();
            ResetAlignmentWait();
            if (xAlignmentQueue) xQueueReset(xAlignmentQueue);
            ++alignmentGeneration; // 已出队的旧帧也不能延续连续计数。
            xSemaphoreGive(xAlignmentMotionMutex);
            WakeMotionExecutor();serialError(frame, "RANGE"); return;
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
        xSemaphoreGive(xAlignmentMotionMutex);WakeMotionExecutor();
    } else if (frame.starts("VISION", "DISC_POSITIONS")) {
        serialDiscPositions(frame);
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
        traceVisionFirstFrame(VisionStartMode::DISC_MATERIAL);
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
        if(!serialQueueDiscGrab(color,cargo,0,3,0,true))serialError(frame,"BUSY");
    } else if (frame.is("VISION", "CONFIRM", 0)) {
        if (serialDebugMode) { serialError(frame, "MODE"); return; }
        VisualCmd_t command = {"OK", 0, 0, 0};
        if (xVisualTaskQueue) xQueueSend(xVisualTaskQueue, &command, 0);
        serialReply(frame, "ACK");
    } else if (frame.is("VISION", "LIDAR_POSE", 0)) {
        SubmitLidarPoseAction(true,serialDebugMode);
    } else if (frame.starts("CFG", "GET") || frame.starts("CFG", "BEGIN")
               || frame.starts("CFG", "SET") || frame.starts("CFG", "COMMIT")) {
        const bool reading = frame.starts("CFG", "GET");
        if (!reading && alignmentEnabled) { serialCfgError(frame, "BUSY_OR_MODE"); return; }
        const char *arguments[SERIAL_MAX_FIELDS];
        for (size_t i = 2; i < frame.count; ++i) arguments[i - 2] = frame.fields[i];
        if (reading) {
            // 参数写入也只由此串口任务执行。只读流式回传无需持有运动锁，
            // 否则几秒的目录回读会阻塞视觉闭环；不启停视觉、不重置PID。
            HandleRuntimeParameters(arguments, frame.count - 2, false);
            return;
        }
        if (xAlignmentMotionMutex && xSemaphoreTake(xAlignmentMotionMutex, portMAX_DELAY) == pdTRUE) {
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
        if (set) {
            currentPose = {values[0], values[1], values[2]};
            CalibrateH30MapHeading(currentPose.theta);
        }
        uint32_t motionId=0;
        if(!set){MotionRequest r;r.kind=MotionKind::GOTO;r.x=values[0];r.y=values[1];r.theta=values[2];motionId=serialPrepareMotion(frame,r);if(!motionId)return;}
        Serial.printf("{RSP,POSE,%s,ACK,%.3f,%.3f,%.3f}\n", frame.fields[2],
                      values[0], values[1], values[2]);
        if(!set)LaunchMotion(motionId);
    } else if (frame.is("CHASSIS", "MOVE_POSITION", 4)) {
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }
        const float minimum[] = {-3394.113f, -3394.113f, -360.0f, 1.0f};
        const float maximum[] = {3394.113f, 3394.113f, 360.0f, 5000.0f};
        float values[4];
        if (!serialFloats(frame, values, minimum, maximum, 4)) {
            serialError(frame, "RANGE"); return;
        }
        MotionRequest r;r.kind=MotionKind::POSITION;r.x=values[0];r.y=values[1];r.theta=values[2];r.speed=values[3];
        const uint32_t id=serialPrepareMotion(frame,r);if(!id)return;
        Serial.printf("{RSP,CHASSIS,MOVE_POSITION,ACK,%.3f,%.3f,%.3f,%.3f}\n",values[0],values[1],values[2],values[3]);LaunchMotion(id);
    } else if (frame.is("CHASSIS", "MOVE", 3)) {
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }
        uint32_t direction = 0, stop = 0;
        float speed = 0;
        if (!serialUInt(frame.fields[3], direction, 5)
                || !serialFloat(frame.fields[4], speed, 0, 1000)
                || !serialUInt(frame.fields[5], stop, 1)) {
            serialError(frame, "RANGE"); return;
        }
        MotionRequest r;r.kind=MotionKind::VELOCITY;r.position=direction;r.speed=speed;r.force=stop;
        const uint32_t id=serialPrepareMotion(frame,r);if(!id)return;
        Serial.printf("{RSP,CHASSIS,MOVE,ACK,%u,%.3f,%u}\n",
                      static_cast<unsigned>(direction), speed, static_cast<unsigned>(stop));
        LaunchMotion(id);
    } else if (frame.is("ARM", "INIT_START", 0) || frame.is("ARM", "INIT_LOOK", 0)
            || frame.is("ARM", "INIT_LOOK2", 0)) {
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }
        MotionRequest r;r.kind=MotionKind::ARM_POSE;
        r.observation=strcmp(frame.fields[2],"INIT_START")==0?ArmObservation::START:
            strcmp(frame.fields[2],"INIT_LOOK")==0?ArmObservation::LOOK:ArmObservation::LOOK2;
        serialMotion(frame,r);
    } else if (frame.is("ARM", "MOVE1", 3) || frame.is("ARM", "MOVE2", 3)) {
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }
        const bool first = strcmp(frame.fields[2], "MOVE1") == 0;
        const float minimum[] = {first ? -1.0f : TURRET_CABLE_MIN_DEG, first ? -1.0f : -360.0f, 1.0f};
        const float maximum[] = {first ? ARM_HEIGHT_LIMIT_MM : 360.0f, first ? 170.0f : 360.0f, FLT_MAX};
        float values[3];
        if (!serialFloats(frame, values, minimum, maximum, 3)
                || (first && ((values[0] < 0 && values[0] != -1)
                    || (values[1] < 0 && values[1] != -1)))) {
            serialError(frame, "RANGE"); return;
        }
        MotionRequest r;r.kind=MotionKind::ARM_RAW;r.speed=values[2];
        if(first){r.high=values[0];r.length=values[1];}else{r.turret=values[0];r.pawl=values[1];}
        const uint32_t id=serialPrepareMotion(frame,r);if(!id)return;
        Serial.printf("{RSP,ARM,%s,ACK,%.3f,%.3f,%.3f}\n", frame.fields[2],
                      values[0], values[1], values[2]);
        LaunchMotion(id);
    } else if (frame.is("ARM", "SERVO_PID_GET", 1)||frame.is("ARM", "SERVO_PID_SET", 8)) {
        if(!serialDebugMode){serialError(frame,"MODE");return;}
        uint32_t id=0;
        if(!serialUInt(frame.fields[3],id,253)){serialError(frame,"RANGE");return;}
        // 旧协议中的逻辑轴2仍代表转盘；该轴已不使用华馨京舵机PID。
        if(id==2){serialError(frame,"UNSUPPORTED");return;}
        uint16_t values[7];
        if(strcmp(frame.fields[2],"SERVO_PID_SET")==0) {
            for(uint8_t i=0;i<7;++i){uint32_t value=0;
                if(!serialUInt(frame.fields[4+i],value,UINT16_MAX)){serialError(frame,"RANGE");return;}
                values[i]=value;
            }
            if(!Servo_WritePid(id,values)){serialError(frame,"SERVO_WRITE");return;}
        }
        if(!Servo_ReadPid(id,values)){serialError(frame,"SERVO_READ");return;}
        Serial.printf("{RSP,ARM,%s,READBACK,%u,%u,%u,%u,%u,%u,%u,%u}\n",frame.fields[2],
            static_cast<unsigned>(id),values[0],values[1],values[2],values[3],values[4],values[5],values[6]);
    } else if (frame.is("ARM", "SERVO_ZERO", 1)) {
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }
        uint32_t id=0;
        if(!serialUInt(frame.fields[3],id,253)){serialError(frame,"RANGE");return;}
        if(id==5||id==6) {
            MotorBusGuard transaction;
            if(ZeroMotorEncoderPosition(id)){serialError(frame,"MOTOR_WRITE");return;}
            int64_t position=0;
            if(ReadMotorEncoderPosition(id,position)){serialError(frame,"MOTOR_READ");return;}
            // 两个直线轴的正向位置指令均为CCW，对应负编码器坐标。
            const float measured=static_cast<double>(-position)*3200.0/65536.0/(id==5?HEIGHT_PULSE:LENGTH_PULSE);
            if(id==5)currentArm.high=measured;
            else currentArm.length=measured;
            Serial.printf("{RSP,ARM,SERVO_ZERO,READBACK,%u,%.3f,%.3f}\n",static_cast<unsigned>(id),measured,measured);
            return;
        }
        if(id==2) {
            float angle=0;
            if(!TurretMotorSetZero(angle)){serialError(frame,"MOTOR_READ");return;}
            currentArm.turret_angle=angle;
            Serial.printf("{RSP,ARM,SERVO_ZERO,READBACK,2,%.3f,%.3f}\n",angle,angle);
            return;
        }
        if(!Servo_SetOrigin(id)){serialError(frame,"SERVO_WRITE");return;}
        float single=0,multi=0;
        if(!Servo_QueryAngle(id,single,100)||!Servo_QueryAngleMTurn(id,multi,100)){
            serialError(frame,"SERVO_READ");return;
        }
        if(id==1)currentArm.pawl_angle=multi;
        Serial.printf("{RSP,ARM,SERVO_ZERO,READBACK,%u,%.3f,%.3f}\n",static_cast<unsigned>(id),single,multi);
    } else if (frame.is("ARM", "SERVO", 2)) {
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }
        uint32_t id = 0;
        float angle = 0;
        if (!serialUInt(frame.fields[3], id, 254)
                || id == SERVO_ID_BROADCAST
                || !serialFloat(frame.fields[4], angle,
                                id == 2 ? TURRET_CABLE_MIN_DEG : -135.0f,
                                id == 2 ? TURRET_CABLE_MAX_DEG : 135.0f)) {
            serialError(frame, "RANGE"); return;
        }
        MotionRequest r;r.kind=id==2?MotionKind::ARM_RAW:MotionKind::SERVO_RAW;r.cargo=id;r.turret=angle;r.speed=150;
        const uint32_t motionId=serialPrepareMotion(frame,r);if(!motionId)return;
        Serial.printf("{RSP,ARM,SERVO,ACK,%u,%.3f}\n",static_cast<unsigned>(id),angle);LaunchMotion(motionId);
    } else if (frame.is("ARM", "ENABLE", 1)) {
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }
        uint32_t enabled = 0;
        if (!serialUInt(frame.fields[3], enabled, 1)) { serialError(frame, "RANGE"); return; }
        MotionRequest r;r.kind=MotionKind::ENABLE;r.force=enabled;
        const uint32_t id=serialPrepareMotion(frame,r);if(!id)return;
        Serial.printf("{RSP,ARM,ENABLE,ACK,%u}\n",static_cast<unsigned>(enabled));LaunchMotion(id);
    } else if (frame.is("ARM", "TRANSFER", 2)) {
        serialTransfer(frame);
    } else if (frame.is("ARM", "RETRIEVE", 2)) {
        serialRetrieve(frame);
    } else if (frame.is("ARM", "DEMO1", 0) || frame.is("ARM", "DEMO2", 0)
               || frame.is("ARM", "DEMO3", 0) || frame.is("ARM", "DEMO4", 0)) {
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }
        MotionRequest r;r.kind=strcmp(frame.fields[2],"DEMO4")==0?MotionKind::DEMO_RETRIEVE:MotionKind::DEMO_PLACE;
        r.layer=strcmp(frame.fields[2],"DEMO3")==0?3:strcmp(frame.fields[2],"DEMO2")==0?2:1;serialMotion(frame,r);
    } else if (frame.count >= 3) {
        serialError(frame, serialKnownAction(frame) ? "FORMAT" : "UNKNOWN_ACTION");
    }
}

/**
 * @brief 处理连续路径试跑配置、准备、启动、状态和停止指令；返回是否已匹配处理。
 */
static bool serialTrial(const SerialFrame &frame) {
    if(strcmp(frame.fields[1],"NAV")!=0||strncmp(frame.fields[2],"TRIAL_",6)!=0)return false;
    uint32_t id=0;
    if(!serialDebugMode){serialError(frame,"MODE");return true;}
    if(frame.count<4||!serialUInt(frame.fields[3],id,9999999)||id==0){serialError(frame,"RANGE");return true;}
    const char *error=nullptr;
    if(frame.is("NAV","TRIAL_BEGIN",12)) {
        uint32_t count=0;
        continuous::Config cfg;cfg.id=id;
        if(!serialUInt(frame.fields[4],count,25)||count<2){serialError(frame,"RANGE");return true;}
        cfg.count=count;
        float *dest[]={&cfg.startHeading,&cfg.endHeading,&cfg.maxRpm,&cfg.accel,&cfg.lateralAccel,
            &cfg.lateralPpm,&cfg.forwardPpm,&cfg.kp,&cfg.maxOmega,&cfg.correction};
        const float low[]={0,0,1,1,1,0.01f,0.01f,0.1f,5,1};
        const float high[]={359.999f,359.999f,5000,2000,3000,1000,1000,10,90,89};
        for(size_t i=0;i<10;++i)if(!serialFloat(frame.fields[i+5],*dest[i],low[i],high[i])) {
            serialError(frame,"RANGE");return true;
        }
        if(cfg.correction>=cfg.maxOmega){serialError(frame,"RANGE");return true;}
        error=ContinuousTrialBegin(cfg);
    } else if(frame.is("NAV","TRIAL_POLICY",5)) {
        uint32_t policy=0,geometry=0;float outset=0,margin=0;
        if(!serialUInt(frame.fields[4],policy,2)||policy<1
            ||!serialFloat(frame.fields[5],outset,0,100)||!serialFloat(frame.fields[6],margin,10,40)
            ||!serialUInt(frame.fields[7],geometry,UINT32_MAX)) {serialError(frame,"RANGE");return true;}
        error=ContinuousTrialPolicy(id,policy,outset,margin,geometry);
    } else if(frame.is("NAV","TRIAL_NODE",6)) {
        uint32_t index=0,node=0;float radius=0,x=0,y=0;
        if(!serialUInt(frame.fields[4],index,24)||!serialUInt(frame.fields[5],node,24)
            ||!serialFloat(frame.fields[6],radius,0,1000)
            ||!serialFloat(frame.fields[7],x,0,2400)||!serialFloat(frame.fields[8],y,0,2400)) {
            serialError(frame,"RANGE");return true;
        }
        error=ContinuousTrialNode(id,index,node,radius,x,y);
    } else if(frame.is("NAV","TRIAL_START",1))error=ContinuousTrialPrepare(id);
    else if(frame.is("NAV","TRIAL_STOP",1))error=ContinuousTrialStop(id);
    else if(frame.is("NAV","TRIAL_STATUS",1)) {
        error=ContinuousTrialStatus(id);if(!error)return true;
    } else {serialError(frame,"FORMAT");return true;}
    if(error){
        Serial.printf("{RSP,NAV,%s,ERR,%lu,%s}\n",frame.fields[2],static_cast<unsigned long>(id),error);
    } else {
        if(frame.is("NAV","TRIAL_NODE",6))
            Serial.printf("{RSP,NAV,TRIAL_NODE,ACK,%lu,%s}\n",static_cast<unsigned long>(id),frame.fields[4]);
        else Serial.printf("{RSP,NAV,%s,ACK,%lu}\n",frame.fields[2],static_cast<unsigned long>(id));
        if(frame.is("NAV","TRIAL_START",1))ContinuousTrialLaunch();
    }
    return true;
}

/**
 * @brief 接收串口帧并分发查询、参数、调试与运动请求；运动交给统一执行器。
 */
void Task_Serial_CMD(void *pvParameters) {
    // 1. 只有本任务读取 Serial0；Debug/Release 共用缓冲区和收帧规则。
    char line[SERIAL_FRAME_BYTES];
    size_t length = 0;
    bool overflow = false;
    bool invalid = false;
    bool discResumeDraining = false;
    size_t discOldBytesRemaining = 0;
    // 固定旧数据的边界，持续新来帧不能无限延长屏蔽时间。
    // 旧数据仍正常分发：COLOR静默丢弃，DISC_POSITIONS回复NOT_READY且不动作；其他指令不受影响。
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
            Serial.printf("[DISC] Material handling resumed: expected=%d, cargo=%d\n",
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
