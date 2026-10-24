/**********************************************************
*** 2027 工创大赛·智能搬运 主控程序框架
*** 角色：ESP32 主控（运动控制 + 任务编排 + 决策）
*** 外部协作：机载电脑负责视觉识别 + 下发指令
***          （颜色/位置识别、转盘物料定位）
***
***       标有 [TODO] 的部分，需按实际机械/定位方案实现。
**********************************************************/

#include "robot_runtime.h"
#include "h30_usb.h"
#include "continuous_trial.h"
#include "motion_executor.h"
#include "competition_flow.h"

// 作业编号 0 保留给主流程时间线；这里只记录观察到的变化，不改变运动或故障处理。
/**
 * @brief 将对外RobotState转换成PHASE日志使用的固定名称。
 */
static const char *mainPhaseName(RobotState state) {
    switch (state) {
    case STATE_WAIT_START: return "WAIT_START";
    case STATE_READ_TASK: return "READ_TASK";
    case STATE_SCAN_FAILED: return "SCAN_FAILED";
    case STATE_ROUTE_FAILED: return "ROUTE_FAILED";
    case STATE_ALIGN_FAILED: return "ALIGN_FAILED";
    case STATE_TIMEOUT_FAILED: return "TIMEOUT_FAILED";
    case STATE_TRANSFER_FAILED: return "TRANSFER_FAILED";
    case STATE_GRAB_ROUND1: return "GRAB_ROUND1";
    case STATE_PLACE_COARSE1: return "PLACE_COARSE1";
    case STATE_PLACE_TEMP1: return "PLACE_TEMP1";
    case STATE_GRAB_ROUND2: return "GRAB_ROUND2";
    case STATE_PLACE_COARSE2: return "PLACE_COARSE2";
    case STATE_STACK_TEMP2: return "STACK_TEMP2";
    case STATE_RETURN_HOME: return "RETURN_HOME";
    case STATE_DONE: return "DONE";
    }
    return "UNKNOWN";
}

/**
 * @brief 仅在主状态或抓取进度变化时输出时间线事件，不改变运行状态。
 */
static void logMainMotionState(RobotState state, int progress) {
    static int previousState = -1;
    static int previousProgress = -1;
    if (previousState != static_cast<int>(state)) {
        Serial.printf("{EVT,MOTION,PHASE,0,%lu,%s}\n",
                      static_cast<unsigned long>(millis()), mainPhaseName(state));
        previousState = static_cast<int>(state);
    }
    if (previousProgress != progress) {
        Serial.printf("{EVT,MOTION,FLAG,0,%lu,ROUND_PROGRESS,%d}\n",
                      static_cast<unsigned long>(millis()), progress);
        previousProgress = progress;
    }
}

namespace {
class RuntimeCompetition : public CompetitionIO {
    uint32_t startedAt=0;
public:
    /**
     * @brief 读取主流程使用的设备运行毫秒数，供阶段间隔计时。
     */
    uint32_t now()override{return millis();}
    /**
     * @brief 读取或更新对外 RobotState；读接口返回当前状态，写接口供阶段切换使用。
     */
    RobotState state()override{return currentState;}
    /**
     * @brief 读取或更新对外 RobotState；读接口返回当前状态，写接口供阶段切换使用。
     */
    void state(RobotState value)override{currentState=value;}
    /**
     * @brief 查询整场任务是否已取消，取消后主流程不再提交动作。
     */
    bool aborted()override{return taskMotionAborted;}
    /**
     * @brief 读取当前选择的起始区号；主流程在启动准备时锁定返家区号。
     */
    uint8_t zone()override{return static_cast<uint8_t>(currentStartZone);}
    void beginHeadingCalibration()override{H30UsbBeginHeadingAverage(START_ZONE_HEADING);}
    bool headingCalibrationDone()override {
        const int result=H30UsbPollHeadingAverage();
        if(result==1)LedPwm_BlinkCalibrationComplete();
        return result!=0;
    }
    /**
     * @brief 查询正式比赛启动许可，许可到达后开始计时并进入扫码。
     */
    bool permission()override{return enableRun;}
    /**
     * @brief 读取当前轮已经完成抓取并登记到载物台的物料件数。
     */
    int progress()override{return roundProgress;}
    /**
     * @brief 将当前轮圆盘抓取进度清零，供进入下一轮或阶段时使用。
     */
    void clearProgress()override{roundProgress=0;}
    /**
     * @brief 将指定轮次的三件颜色和工位编号填入运动请求。
     * @param request 接收本轮任务数据的动作请求。
     * @param second true 第二轮，false 第一轮。
     */
    void task(MotionRequest &request,bool second)override {
        portENTER_CRITICAL(&taskCodeMux);
        for(int i=0;i<3;++i){request.colors[i]=second?currentTask.round2_colors[i]:currentTask.round1_colors[i];request.positions[i]=second?currentTask.round2_pos[i]:currentTask.round1_pos[i];}
        portEXIT_CRITICAL(&taskCodeMux);
    }
    /**
     * @brief 提交一个非阻塞运动请求；返回作业编号，后续用 status 查询完成结果。
     */
    uint32_t submit(const MotionRequest &request)override{return SubmitMotion(request);}
    /**
     * @brief 提交起始区扫码准备姿态作业；返回作业编号，不在主流程等待物理动作。
     */
    uint32_t lidar()override{return SubmitLidarPoseAction();}
    /**
     * @brief 读取指定作业的执行快照；只有终态 DONE 才表示该作业按当前完成策略结束。
     */
    MotionStatus status(uint32_t id)override{return GetMotionStatus(id);}
    /**
     * @brief 请求开启指定视觉功能；工位对齐与单件物料识别使用不同模式。
     */
    void visionStart(VisionStartMode mode)override{requestVisionStart(mode);}
    /**
     * @brief 读取当前自动视觉对齐的 WAITING、DONE 或 FAILED 状态。
     */
    AutoAlignmentState visionStatus()override{return getAutoAlignmentState();}
    /**
     * @brief 请求停止指定视觉功能，结束该阶段的视觉控制。
     */
    void visionStop(VisionStartMode mode)override{requestVisionStop(mode);}
    /**
     * @brief 开启圆盘单件物料识别；识别结果触发独立的抓取作业。
     */
    void materialStart()override {
        firstDiscGrabReady=false;xQueueReset(xVisualTaskQueue);
        requestVisionStart(VisionStartMode::DISC_MATERIAL);
        discMessageResumeRequested=true;display("WAIT DISC MATERIAL");
    }
    /**
     * @brief 停止圆盘单件物料识别并清除对应就绪标志。
     */
    void materialStop()override{firstDiscGrabReady=false;requestVisionStop(VisionStartMode::DISC_MATERIAL);}
    /**
     * @brief 发布已解析的任务码显示，并设置扫码结束后的位姿准备标志。
     */
    void taskReady()override {
        scanPosePending=true;xQueueReset(xVisualTaskQueue);
        char text[16];snprintf(text,sizeof(text),"%d%d%d+%d%d%d+%d%d%d+%d%d%d",
            currentTask.round1_colors[0],currentTask.round1_colors[1],currentTask.round1_colors[2],
            currentTask.round1_pos[0],currentTask.round1_pos[1],currentTask.round1_pos[2],
            currentTask.round2_colors[0],currentTask.round2_colors[1],currentTask.round2_colors[2],
            currentTask.round2_pos[0],currentTask.round2_pos[1],currentTask.round2_pos[2]);
        updateDisplay("DISPLAY","TASK_CODE",text);display("TASK OK");
    }
    /**
     * @brief 记录正式开始时间、输出 RUN_START，并启动比赛总计时器。
     */
    void runStart()override {
        enableRun=true; // 自动START与串口START共用运行许可，保持取消和状态查询语义一致。
        startedAt=millis();Serial.printf("{EVT,MOTION,RUN_START,0,%lu}\n",static_cast<unsigned long>(startedAt));
        if(xHomeTimer)xTimerStart(xHomeTimer,0);
    }
    /**
     * @brief 输出正式完成结果、停止比赛计时，并提交电机失能作业。
     */
    void runDone()override {
        if(xHomeTimer)xTimerStop(xHomeTimer,0);
        const uint32_t finish=millis();
        Serial.printf("{EVT,MOTION,RUN_RESULT,0,%lu,DONE,%lu}\n",static_cast<unsigned long>(finish),static_cast<unsigned long>(finish-startedAt));
        MotionRequest request;request.kind=MotionKind::ENABLE;request.force=false;SubmitMotion(request);display("DONE");
    }
    /**
     * @brief 转入指定业务失败状态，取消运动并更新显示；沿用现有失败处理。
     */
    void fail(RobotState reason)override {
        if(taskMotionAborted)return;
        if(xHomeTimer)xTimerStop(xHomeTimer,0);
        currentState=reason;CancelMotion();
        display(reason==STATE_ROUTE_FAILED?"ROUTE ERR":reason==STATE_ALIGN_FAILED?"ALIGN ERR":reason==STATE_SCAN_FAILED?"TASK ERR":"TRANSFER ERR");
    }
    /**
     * @brief 向上位机发布当前业务阶段的显示文本。
     */
    void display(const char *text)override{updateDisplay("DISPLAY","DEBUG",text);}
};
}
/**
 * @brief 比赛主任务，每10ms调用 CompetitionFlow::tick，处理状态编排及主阶段日志。
 */
void Task_MainStateMachine(void *) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    RuntimeCompetition runtime;CompetitionMap map;
    map.scan=SCAN_AREA_NODE;map.disc=DISC_AREA_NODE;map.coarse=COARSE_AREA_NODE;map.temp=TEMP_AREA_NODE;
    map.home1=HOME_ZONE1_NODE;map.home2=HOME_ZONE2_NODE;map.homeHeading=START_ZONE_HEADING;
    CompetitionFlow flow(runtime,map);
    for(;;) {
        logMainMotionState(currentState,roundProgress);
        flow.tick();
        logMainMotionState(currentState,roundProgress);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ================= setup  =================
/**
 * @brief 初始化参数、串口、运动硬件及任务，建立比赛运行环境。
 */
void setup() {
    pinMode(BOOT0_PIN, INPUT_PULLUP);
    const bool ledPwmReady = LedPwm_Init();
    Serial.begin(115200);
    if (!ApplyCompiledParameterDefaults()) {
        Serial.println("[CFG] ERR: compiled defaults not applied; continuing startup");
    }
    if (!ledPwmReady) Serial.println("[LED] PWM initialization failed");
    Serial.printf("version: %s\n", VERSION);
    Servo_Init();    // 总线舵机初始化 (默认 Serial2: RX=16, TX=15, 115200bps)
    Emm_V5_Init();   // 电机初始化
    Scanner_Init();  // 扫码模块初始化 (软串口 RX=IO4, 9600bps)
    if (!H30UsbBegin()) Serial.println("[H30] USB Host initialization failed");
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(10);

    currentPose = {0, 0, 0};   // [TODO] 初始位姿按实际
    currentArm = {0, 0, 0, 0}; // 升降/伸缩目标坐标对应驱动器回零后的原点，舵机角度由下方实读更新。
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

    xVisualTaskQueue = xQueueCreate(10, sizeof(VisualCmd_t)); // 机载电脑指令队列(深度10)
    xLidarPoseMutex = xSemaphoreCreateMutex();
    xAlignmentQueue = xQueueCreate(1, sizeof(VisualAlignmentFrame_t)); // 视觉 PID 对齐只保留 20 Hz 连续反馈中的最新一帧。
    xAlignmentMotionMutex = xSemaphoreCreateMutex();
    if(!ContinuousTrialInit())Serial.println("[Trial] initialization failed");
    if(!MotionExecutorInit())Serial.println("[Motion] task initialization failed");

    vTaskDelay(pdMS_TO_TICKS(500));
    // 上电固定进入 Debug；串口SYS RELEASE或空闲时按BOOT0切换到正式模式。
    {
        Serial.println("Debug mode");
        leds[0] = CRGB::Yellow; FastLED.show();
        // Debug 模式默认直接启用视觉闭环对齐，仍可通过
        // {CMD,VISION,ALIGN_STOP}/{CMD,VISION,ALIGN_START} 可停止或重启。
        setAlignmentEnabled(true);
        xTaskCreate(Task_Serial_CMD, "Task_Serial_CMD", 16384, NULL, 5, NULL);
    }
}

/**
 * @brief 处理后台维护、灯光与按键采样；比赛业务由独立主状态机任务推进。
 */
void loop() {
    PollDebugBootButton();
    // 不依赖主状态机：总超时暂停主任务后仍持续闪灯。
    const RobotState state = currentState;
    LedPwm_UpdateFaultBlink(state == STATE_SCAN_FAILED || state == STATE_ROUTE_FAILED
        || state == STATE_ALIGN_FAILED || state == STATE_TIMEOUT_FAILED
        || state == STATE_TRANSFER_FAILED);
    vTaskDelay(pdMS_TO_TICKS(20));
}
