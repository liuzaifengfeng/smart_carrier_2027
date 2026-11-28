// 固件装配入口。
// setup 初始化总线、硬件与任务，比赛适配层把流程请求交给运行动作接口，loop 维护需要周期刷新的面板与提示状态。

#include "turret_motor.h"  // 引入 turret_motor.h 中的声明和定义。
#include "battery_monitor.h"  // 引入 battery_monitor.h 中的声明和定义。
/**********************************************************
*** 2027 工创大赛·智能搬运 主控程序框架
*** 角色：ESP32 主控（运动控制 + 任务编排 + 决策）
*** 外部协作：机载电脑负责视觉识别 + 下发指令
***          （颜色/位置识别、转盘物料定位）
***
***       标有 [TODO] 的部分，需按实际机械/定位方案实现。    
**********************************************************/

#include "robot_runtime.h"  // 引入 robot_runtime.h 中的声明和定义。
#include "h30_usb.h"  // 引入 h30_usb.h 中的声明和定义。
#include "continuous_trial.h"  // 引入 continuous_trial.h 中的声明和定义。
#include "motion_executor.h"   // 引入 motion_executor.h 中的声明和定义。
#include "competition_flow.h"  // 引入 competition_flow.h 中的声明和定义。
#include "start_panel.h"  // 引入 start_panel.h 中的声明和定义。

// 作业编号 0 保留给主流程时间线；这里只记录观察到的变化，不改变运动或故障处理。
/**
 * @brief 将对外RobotState转换成PHASE日志使用的固定名称。
 */
static const char *mainPhaseName(RobotState state) {  // 定义函数或成员 的入口 mainPhaseName（main阶段Name），参数由此处的签名确定。
    switch (state) {  // 按 state 的值选择对应分支。
    case STATE_WAIT_START: return "WAIT_START";  // 当前值匹配 STATE_WAIT_START 时从这里执行。
    case STATE_READ_TASK: return "READ_TASK";  // 当前值匹配 STATE_READ_TASK 时从这里执行。
    case STATE_SCAN_FAILED: return "SCAN_FAILED";  // 当前值匹配 STATE_SCAN_FAILED 时从这里执行。
    case STATE_ROUTE_FAILED: return "ROUTE_FAILED";  // 当前值匹配 STATE_ROUTE_FAILED 时从这里执行。
    case STATE_ALIGN_FAILED: return "ALIGN_FAILED";  // 当前值匹配 STATE_ALIGN_FAILED 时从这里执行。
    case STATE_TIMEOUT_FAILED: return "TIMEOUT_FAILED";  // 当前值匹配 STATE_TIMEOUT_FAILED 时从这里执行。
    case STATE_TRANSFER_FAILED: return "TRANSFER_FAILED";  // 当前值匹配 STATE_TRANSFER_FAILED 时从这里执行。
    case STATE_GRAB_ROUND1: return "GRAB_ROUND1";  // 当前值匹配 STATE_GRAB_ROUND1 时从这里执行。
    case STATE_PLACE_COARSE1: return "PLACE_COARSE1";  // 当前值匹配 STATE_PLACE_COARSE1 时从这里执行。
    case STATE_PLACE_TEMP1: return "PLACE_TEMP1";  // 当前值匹配 STATE_PLACE_TEMP1 时从这里执行。
    case STATE_GRAB_ROUND2: return "GRAB_ROUND2";  // 当前值匹配 STATE_GRAB_ROUND2 时从这里执行。
    case STATE_PLACE_COARSE2: return "PLACE_COARSE2";  // 当前值匹配 STATE_PLACE_COARSE2 时从这里执行。
    case STATE_STACK_TEMP2: return "STACK_TEMP2";  // 当前值匹配 STATE_STACK_TEMP2 时从这里执行。
    case STATE_RETURN_HOME: return "RETURN_HOME";  // 当前值匹配 STATE_RETURN_HOME 时从这里执行。
    case STATE_DONE: return "DONE";  // 当前值匹配 STATE_DONE 时从这里执行。
    }  // 结束此函数或控制块。
    return "UNKNOWN";  // 结束当前函数并返回 "UNKNOWN"。
}  // 结束此函数或控制块。

/**
 * @brief 仅在主状态或抓取进度变化时输出时间线事件，不改变运行状态。
 */
static void logMainMotionState(RobotState state, int progress) {  // 定义函数或成员 的入口 logMainMotionState（logMainMotion状态），参数由此处的签名确定。
    static int previousState = -1;  // 将 -1 写入或初始化 previousState（上一状态）。
    static int previousProgress = -1;  // 将 -1 写入或初始化 previousProgress（上一Progress）。
    if (previousState != static_cast<int>(state)) {  // 当 previousState != static_cast<int>(state) 成立时进入此分支，否则继续后续流程。
        Serial.printf("{EVT,MOTION,PHASE,0,%lu,%s}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                      static_cast<unsigned long>(millis()), mainPhaseName(state));  // 声明接口 long>，参数由此处的签名确定。
        previousState = static_cast<int>(state);  // 将 static_cast<int>(state) 写入或初始化 previousState（上一状态）。
    }  // 结束此条件分支。
    if (previousProgress != progress) {  // 当 previousProgress != progress 成立时进入此分支，否则继续后续流程。
        Serial.printf("{EVT,MOTION,FLAG,0,%lu,ROUND_PROGRESS,%d}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                      static_cast<unsigned long>(millis()), progress);  // 声明接口 long>，参数由此处的签名确定。
        previousProgress = progress;  // 将 progress 写入或初始化 previousProgress（上一Progress）。
    }  // 结束此条件分支。
}  // 结束此函数或控制块。

namespace {  // 开始匿名命名空间，本文件内部共享其中的定义。
class RuntimeCompetition : public CompetitionIO {  // 定义 RuntimeCompetition 的类型成员或作用域。
    uint32_t startedAt=0;  // 将 0 写入或初始化 startedAt。
    const bool fromStartPanel;  // 声明 fromStartPanel（from起点Panel），类型和可用范围由所在声明决定。
public:  // 此后的成员采用 public 访问权限，直到下一个权限标签。
    explicit RuntimeCompetition(bool fromStartPanel):fromStartPanel(fromStartPanel){}  // 定义函数或成员 RuntimeCompetition，参数由此处的签名确定。
    /**
     * @brief 读取主流程使用的设备运行毫秒数，供阶段间隔计时。
     */
    uint32_t now()override{return millis();}  // 定义函数或成员 now，参数由此处的签名确定。
    /**
     * @brief 读取或更新对外 RobotState；读接口返回当前状态，写接口供阶段切换使用。
     */
    RobotState state()override{return currentState;}  // 定义函数或成员 state（状态），参数由此处的签名确定。
    /**
     * @brief 读取或更新对外 RobotState；读接口返回当前状态，写接口供阶段切换使用。
     */
    void state(RobotState value)override{currentState=value;}  // 将 value;} 写入或初始化 currentState（当前状态）。
    /**
     * @brief 查询整场任务是否已取消，取消后主流程不再提交动作。
     */
    bool aborted()override{return taskMotionAborted;}  // 定义函数或成员 aborted，参数由此处的签名确定。
    /**
     * @brief 读取当前选择的起始区号；主流程在启动准备时锁定返家区号。
     */
    uint8_t zone()override{  // 定义函数或成员 的入口 zone，参数由此处的签名确定。
        const auto selected=SelectReleaseStartZone(fromStartPanel);  // 将 SelectReleaseStartZone(fromStartPanel) 写入或初始化 selected。
        if(selected!=START_ZONE_UNKNOWN&&ReadPanelStartRequest()!=START_ZONE_UNKNOWN) {  // 当 selected!=START_ZONE_UNKNOWN&&ReadPanelStartRequest()!… 成立时进入此分支，否则继续后续流程。
            currentStartZone=selected;  // 将 selected 写入或初始化 currentStartZone（当前起点Zone）。
            Serial.printf("[StartPanel] zone=%u\n",static_cast<unsigned>(selected));  // 调用 Serial.printf，传入 "[StartPanel] zone=%u\n",static_cast<unsigned>(selecte…。
        }  // 结束此条件分支。
        return static_cast<uint8_t>(selected);  // 结束当前函数并返回 static_cast<uint8_t>(selected)。
    }  // 结束此函数或控制块。
    void beginHeadingCalibration()override{H30UsbBeginHeadingAverage();}  // 定义函数或成员 beginHeadingCalibration（begin航向Calibration），参数由此处的签名确定。
    bool headingCalibrationDone()override {  // 定义函数或成员 的入口 headingCalibrationDone（航向Calibration完成），参数由此处的签名确定。
        const int result=H30UsbPollHeadingAverage();  // 将 H30UsbPollHeadingAverage() 写入或初始化 result（结果）。
        if(result!=0)LedPwm_PlayCue(3);  // 当 result!=0 成立时进入此分支，否则继续后续流程。
        return result!=0;  // 结束当前函数并返回 result!=0。
    }  // 结束此函数或控制块。
    /**
     * @brief 查询正式比赛启动许可，许可到达后开始计时并进入扫码。
     */
    bool permission()override{return ReadPanelStartRequest()!=START_ZONE_UNKNOWN||enableRun;}  // 定义函数或成员 permission，参数由此处的签名确定。
    /**
     * @brief 读取当前轮已经完成抓取并登记到载物台的物料件数。
     */
    int progress()override{return roundProgress;}  // 定义函数或成员 progress，参数由此处的签名确定。
    /**
     * @brief 将当前轮圆盘抓取进度清零，供进入下一轮或阶段时使用。
     */
    void clearProgress()override{roundProgress=0;}  // 将 0;} 写入或初始化 roundProgress（轮次Progress）。
    /**
     * @brief 将指定轮次的三件颜色和工位编号填入运动请求。
     * @param request 接收本轮任务数据的动作请求。
     * @param second true 第二轮，false 第一轮。
     */
    void task(MotionRequest &request,bool second)override {  // 定义函数或成员 的入口 task（任务），参数由此处的签名确定。
        portENTER_CRITICAL(&taskCodeMux);  // 进入临界区，保护共享状态的本段访问。
        for(int i=0;i<3;++i){request.colors[i]=second?currentTask.round2_colors[i]:currentTask.round1_colors[i];request.positions[i]=second?currentTask.round2_pos[i]:currentTask.round1_pos[i];}  // 按 int i=0;i<3;++i 组织循环的初值、继续条件和步进。
        portEXIT_CRITICAL(&taskCodeMux);  // 离开临界区，保护共享状态的本段访问。
    }  // 结束此函数或控制块。
    /**
     * @brief 提交一个非阻塞运动请求；返回作业编号，后续用 status 查询完成结果。
     */
    uint32_t submit(const MotionRequest &request)override{return SubmitMotion(request);}  // 定义函数或成员 submit，参数由此处的签名确定。
    uint32_t submit(const MotionRequest &request,MotionSubmitError &error)override {  // 定义函数或成员 的入口 submit，参数由此处的签名确定。
        const uint32_t id=SubmitMotion(request,true,&error);  // 将 SubmitMotion(request,true,&error) 写入或初始化 id（编号）。
        if(!id&&error!=MotionSubmitError::BUSY)  // 当 !id&&error!=MotionSubmitError::BUSY 成立时进入此分支，否则继续后续流程。
            Serial.printf("{EVT,MOTION,SUBMIT_FAILED,%s}\n",error==MotionSubmitError::QUEUE?"QUEUE":"UNAVAILABLE");  // 调用 Serial.printf，传入 "{EVT,MOTION,SUBMIT_FAILED,%s}\n",error==MotionSubmitE…。
        return id;  // 结束当前函数并返回 id。
    }  // 结束此函数或控制块。
    bool motionBusy()override{return MotionExecutorActive();}  // 定义函数或成员 motionBusy，参数由此处的签名确定。
    /**
     * @brief 提交起始区扫码准备姿态作业；返回作业编号，不在主流程等待物理动作。
     */
    uint32_t lidar(uint8_t startZone)override{return SubmitLidarPoseAction(false,false,startZone);}  // 定义函数或成员 lidar，参数由此处的签名确定。
    /**
     * @brief 读取指定作业的执行快照；只有终态 DONE 才表示该作业按当前完成策略结束。
     */
    MotionStatus status(uint32_t id)override{return GetMotionStatus(id);}  // 定义函数或成员 status（状态），参数由此处的签名确定。
    /**
     * @brief 请求开启指定视觉功能；工位对齐与单件物料识别使用不同模式。
     */
    void visionStart(VisionStartMode mode,bool invertCornerXY)override{requestVisionStart(mode,invertCornerXY);}  // 定义函数或成员 visionStart（视觉起点），参数由此处的签名确定。
    /**
     * @brief 读取当前自动视觉对齐的 WAITING、DONE 或 FAILED 状态。
     */
    AutoAlignmentState visionStatus()override{return getAutoAlignmentState();}  // 定义函数或成员 visionStatus（视觉状态），参数由此处的签名确定。
    /**
     * @brief 请求停止指定视觉功能，结束该阶段的视觉控制。
     */
    void visionStop(VisionStartMode mode)override{requestVisionStop(mode);}  // 定义函数或成员 visionStop（视觉停止），参数由此处的签名确定。
    /**
     * @brief 开启圆盘单件物料识别；识别结果触发独立的抓取作业。
     */
    void materialStart()override {  // 定义函数或成员 的入口 materialStart（物料起点），参数由此处的签名确定。
        firstDiscGrabReady=false;xQueueReset(xVisualTaskQueue);  // 将 false 写入或初始化 firstDiscGrabReady（首个圆盘Grab就绪）。然后调用 xQueueReset（横向QueueReset），传入 xVisualTaskQueue。
        requestVisionStart(VisionStartMode::DISC_MATERIAL);  // 调用 requestVisionStart（请求视觉起点），传入 VisionStartMode::DISC_MATERIAL。
        discMessageResumeRequested=true;display("WAIT DISC MATERIAL");  // 将 true 写入或初始化 discMessageResumeRequested（圆盘MessageResumeRequested）。然后调用 display，传入 "WAIT DISC MATERIAL"。
    }  // 结束此函数或控制块。
    /**
     * @brief 停止圆盘单件物料识别并清除对应就绪标志。
     */
    void materialStop()override{firstDiscGrabReady=false;requestVisionStop(VisionStartMode::DISC_MATERIAL);}  // 将 false;requestVisionStop(VisionStartMode::DISC_MATERIAL… 写入或初始化 firstDiscGrabReady（首个圆盘Grab就绪）。
    /**
     * @brief 发布已解析的任务码显示。
     */
    void taskReady()override {  // 定义函数或成员 的入口 taskReady（任务就绪），参数由此处的签名确定。
        xQueueReset(xVisualTaskQueue);  // 调用 xQueueReset（横向QueueReset），传入 xVisualTaskQueue。
        char text[16];snprintf(text,sizeof(text),"%d%d%d+%d%d%d+%d%d%d+%d%d%d",  // 声明 text，类型和可用范围由所在声明决定。然后调用 snprintf，传入 text,sizeof(text),"%d%d%d+%d%d%d+%d%d%d+%d%d%d",。
            currentTask.round1_colors[0],currentTask.round1_colors[1],currentTask.round1_colors[2],  // 为当前列表或跨行表达式提供这一项 currentTask.round1_colors[0],currentTask.round1_colors…。
            currentTask.round1_pos[0],currentTask.round1_pos[1],currentTask.round1_pos[2],  // 为当前列表或跨行表达式提供这一项 currentTask.round1_pos[0],currentTask.round1_pos[1],cu…。
            currentTask.round2_colors[0],currentTask.round2_colors[1],currentTask.round2_colors[2],  // 为当前列表或跨行表达式提供这一项 currentTask.round2_colors[0],currentTask.round2_colors…。
            currentTask.round2_pos[0],currentTask.round2_pos[1],currentTask.round2_pos[2]);  // 续写当前声明或运算，提供 currentTask.round2_pos[0],currentTask.round2_pos[1],cu…。
        updateDisplay("DISPLAY","TASK_CODE",text);display("TASK OK");  // 调用 updateDisplay，传入 "DISPLAY","TASK_CODE",text。然后调用 display，传入 "TASK OK"。
    }  // 结束此函数或控制块。
    /**
     * @brief 记录正式开始时间、输出 RUN_START，并启动比赛总计时器。
     */
    void runStart()override {  // 定义函数或成员 的入口 runStart（run起点），参数由此处的签名确定。
        enableRun=true; // 自动START与串口START共用运行许可，保持取消和状态查询语义一致。
        startedAt=millis();Serial.printf("{EVT,MOTION,RUN_START,0,%lu}\n",static_cast<unsigned long>(startedAt));  // 将 millis();Serial.printf("{EVT,MOTION,RUN_START,0,%lu}\n… 写入或初始化 startedAt。
        if(xHomeTimer)xTimerStart(xHomeTimer,0);  // 当 xHomeTimer 成立时进入此分支，否则继续后续流程。
    }  // 结束此函数或控制块。
    /**
     * @brief 输出正式完成结果、停止比赛计时，并提交电机失能作业。
     */
    void runDone()override {  // 定义函数或成员 的入口 runDone（run完成），参数由此处的签名确定。
        if(xHomeTimer)xTimerStop(xHomeTimer,0);  // 当 xHomeTimer 成立时进入此分支，否则继续后续流程。
        const uint32_t finish=millis();  // 将 millis() 写入或初始化 finish。
        Serial.printf("{EVT,MOTION,RUN_RESULT,0,%lu,DONE,%lu}\n",static_cast<unsigned long>(finish),static_cast<unsigned long>(finish-startedAt));  // 调用 Serial.printf，传入 "{EVT,MOTION,RUN_RESULT,0,%lu,DONE,%lu}\n",static_cast…。
        LedPwm_PlayCompletionCue();  // 调用 LedPwm_PlayCompletionCue（Led占空比输出PlayCompletionCue），使用无参形式推进相应操作。
        MotionRequest request;request.kind=MotionKind::ENABLE;request.force=false;SubmitMotion(request);display("DONE");  // 声明 request（请求），类型和可用范围由所在声明决定。然后将 MotionKind::ENABLE 写入或初始化 request.kind。然后将 false 写入或初始化 request.force。然后调用 SubmitMotion，传入 request。然后调用 display，传入 "DONE"。
    }  // 结束此函数或控制块。
    /**
     * @brief 转入指定业务失败状态，取消运动并更新显示；沿用现有失败处理。
     */
    void fail(RobotState reason)override {  // 定义函数或成员 的入口 fail，参数由此处的签名确定。
        if(taskMotionAborted)return;  // 当 taskMotionAborted 成立时进入此分支，否则继续后续流程。
        if(xHomeTimer)xTimerStop(xHomeTimer,0);  // 当 xHomeTimer 成立时进入此分支，否则继续后续流程。
        currentState=reason;CancelMotion();  // 将 reason 写入或初始化 currentState（当前状态）。然后调用 CancelMotion，使用无参形式推进相应操作。
        display(reason==STATE_ROUTE_FAILED?"ROUTE ERR":reason==STATE_ALIGN_FAILED?"ALIGN ERR":reason==STATE_SCAN_FAILED?"SCAN ROUTE ERR":"TRANSFER ERR");  // 调用 display，传入 reason==STATE_ROUTE_FAILED?"ROUTE ERR":reason==STATE_A…。
    }  // 结束此函数或控制块。
    /**
     * @brief 向上位机发布当前业务阶段的显示文本。
     */
    void display(const char *text)override{updateDisplay("DISPLAY","DEBUG",text);}  // 定义函数或成员 display，参数由此处的签名确定。
};  // 结束此类型定义。
}  // 结束命名空间。
/**
 * @brief 比赛主任务，每10ms调用 CompetitionFlow::tick，处理状态编排及主阶段日志。
 */
void Task_MainStateMachine(void *startPanel) {  // 定义函数或成员 的入口 Task_MainStateMachine（任务Main状态Machine），参数由此处的签名确定。
    vTaskDelay(pdMS_TO_TICKS(1000));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    RuntimeCompetition runtime(startPanel!=nullptr);CompetitionMap map;  // 声明接口 runtime，参数由此处的签名确定。然后声明 map，类型和可用范围由所在声明决定。
    map.scan=SCAN_AREA_NODE;map.disc=DISC_AREA_NODE;map.coarse=COARSE_AREA_NODE;map.temp=TEMP_AREA_NODE;  // 将 SCAN_AREA_NODE 写入或初始化 map.scan。然后将 DISC_AREA_NODE 写入或初始化 map.disc。然后将 COARSE_AREA_NODE 写入或初始化 map.coarse。然后将 TEMP_AREA_NODE 写入或初始化 map.temp。
    map.home1=HOME_ZONE1_NODE;map.home2=HOME_ZONE2_NODE;map.homeHeading=HOME_ZONE_HEADING;  // 将 HOME_ZONE1_NODE 写入或初始化 map.home1。然后将 HOME_ZONE2_NODE 写入或初始化 map.home2。然后将 HOME_ZONE_HEADING 写入或初始化 map.homeHeading。
    CompetitionFlow flow(runtime,map);  // 声明接口 flow，参数由此处的签名确定。
    for(;;) {  // 按 ;; 组织循环的初值、继续条件和步进。
        logMainMotionState(currentState,roundProgress);  // 调用 logMainMotionState（logMainMotion状态），传入 currentState,roundProgress。
        flow.tick();  // 调用 flow.tick。
        logMainMotionState(currentState,roundProgress);  // 调用 logMainMotionState（logMainMotion状态），传入 currentState,roundProgress。
        vTaskDelay(pdMS_TO_TICKS(10));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    }  // 结束此循环体。
}  // 结束此函数或控制块。

// ================= setup  =================
/**
 * @brief 初始化参数、串口、运动硬件及任务，建立比赛运行环境。
 */
void setup() {  // 定义函数或成员 的入口 setup，参数由此处的签名确定。
    pinMode(BOOT0_PIN, INPUT_PULLUP);  // 调用 pinMode（pin模式），传入 BOOT0_PIN, INPUT_PULLUP。
    pinMode(START_ZONE_SWITCH_PIN,INPUT);  // 调用 pinMode（pin模式），传入 START_ZONE_SWITCH_PIN,INPUT。
    pinMode(START_BUTTON_PIN,INPUT);  // 调用 pinMode（pin模式），传入 START_BUTTON_PIN,INPUT。
    ledcSetup(BUTTON_LED_PWM_CHANNEL,BUTTON_LED_PWM_HZ,BUTTON_LED_PWM_BITS);  // 调用 ledcSetup，传入 BUTTON_LED_PWM_CHANNEL,BUTTON_LED_PWM_HZ,BUTTON_LED_PW…。
    ledcWrite(BUTTON_LED_PWM_CHANNEL,0);  // 调用 ledcWrite，传入 BUTTON_LED_PWM_CHANNEL,0。
    ledcAttachPin(BUTTON_LED_PIN,BUTTON_LED_PWM_CHANNEL);  // 调用 ledcAttachPin，传入 BUTTON_LED_PIN,BUTTON_LED_PWM_CHANNEL。
    BatteryMonitorBegin();  // 调用 BatteryMonitorBegin，使用无参形式推进相应操作。
    const bool ledPwmReady = LedPwm_Init();  // 将 LedPwm_Init() 写入或初始化 ledPwmReady（led占空比输出就绪）。
    Serial.begin(115200);  // 调用 Serial.begin，传入 115200。
    if (!ApplyCompiledParameterDefaults()) {  // 当 !ApplyCompiledParameterDefaults() 成立时进入此分支，否则继续后续流程。
        Serial.println("[CFG] ERR: compiled defaults not applied; continuing startup");  // 调用 Serial.println，传入 "[CFG] ERR: compiled defaults not applied; continuing …。
    }  // 结束此条件分支。
    if (!ledPwmReady) Serial.println("[LED] PWM initialization failed");  // 当 !ledPwmReady 成立时进入此分支，否则继续后续流程。
    Serial.printf("version: %s\n", VERSION);  // 调用 Serial.printf，传入 "version: %s\n", VERSION。
    Servo_Init();    // 总线舵机初始化
    Emm_V5_Init();   // 电机初始化
    Scanner_Init();  // 扫码模块初始化 (软串口 RX=IO4, 9600bps)
    if (!H30UsbBegin()) Serial.println("[H30] USB Host initialization failed");  // 当 !H30UsbBegin() 成立时进入此分支，否则继续后续流程。
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);  // 声明接口 GRB>，参数由此处的签名确定。
    FastLED.setBrightness(10);  // 调用 FastLED.setBrightness，传入 10。

    currentPose = {0, 0, 0};   // [TODO] 初始位姿按实际
    currentArm = {0, 0, 0, 0}; // 升降/伸缩目标坐标对应驱动器回零后的原点，舵机角度由下方实读更新。
    // 转盘为Serial1的7号张大头电机；夹爪为Serial2的1号舵机。
    // 在串口任务启动前读取，避免多个任务同时访问舵机总线。
    // 给上电中的舵机留出启动时间；总线已初始化但舵机不一定立即应答。
    vTaskDelay(pdMS_TO_TICKS(100));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    Servo_MoveCargoTray(0);  // 调用 Servo_MoveCargoTray（ServoMove货位料盘），传入 0。
    if (!TurretMotorReadAngle(currentArm.turret_angle))  // 当 !TurretMotorReadAngle(currentArm.turret_angle) 成立时进入此分支，否则继续后续流程。
        Serial.println("[Turret] startup angle unavailable; continuing startup");  // 调用 Serial.println，传入 "[Turret] startup angle unavailable; continuing startu…。
    else Serial.printf("[Turret] startup mechanical angle=%.3f deg (saved zero)\n",currentArm.turret_angle);  // 前一个判断不成立时执行此分支。
    readStartupServoAngle(1, currentArm.pawl_angle);  // 调用 readStartupServoAngle（readStartupServo角度），传入 1, currentArm.pawl_angle。

    // 注意: 原 initLidar() 已移除, 雷达/定位方案待定
     init_ota_service("null", "1234567899", OTA_HOSTNAME);//调试使用，正式比赛时注释掉
    //init_ota_service("longggg", "asdfghjkl", OTA_HOSTNAME);//调试使用，正式比赛时注释掉
    leds[0] = CRGB::Red; FastLED.show();  // 将 CRGB::Red 写入或初始化 leds[0]。然后调用 FastLED.show。

    xVisualTaskQueue = xQueueCreate(10, sizeof(VisualCmd_t)); // 机载电脑指令队列(深度10)
    xLidarPoseMutex = xSemaphoreCreateMutex();  // 将 xSemaphoreCreateMutex() 写入或初始化 xLidarPoseMutex（横向LidarPoseMutex）。
    xAlignmentQueue = xQueueCreate(1, sizeof(VisualAlignmentFrame_t)); // 视觉 PID 对齐只保留 20 Hz 连续反馈中的最新一帧。
    xAlignmentMotionMutex = xSemaphoreCreateMutex();  // 将 xSemaphoreCreateMutex() 写入或初始化 xAlignmentMotionMutex（横向AlignmentMotionMutex）。
    if(!ContinuousTrialInit())Serial.println("[Trial] initialization failed");  // 当 !ContinuousTrialInit() 成立时进入此分支，否则继续后续流程。
    if(!MotionExecutorInit())Serial.println("[Motion] task initialization failed");  // 当 !MotionExecutorInit() 成立时进入此分支，否则继续后续流程。

    vTaskDelay(pdMS_TO_TICKS(500));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    // 上电固定进入 Debug；串口SYS RELEASE或空闲时按BOOT0切换到正式模式。
    {  // 为当前列表或跨行表达式提供这一项 {。
        Serial.println("Debug mode");  // 调用 Serial.println，传入 "Debug mode"。
        leds[0] = CRGB::Yellow; FastLED.show();  // 将 CRGB::Yellow 写入或初始化 leds[0]。然后调用 FastLED.show。
        // Debug 模式默认直接启用视觉闭环对齐，仍可通过
        // {CMD,VISION,ALIGN_STOP}/{CMD,VISION,ALIGN_START} 可停止或重启。
        setAlignmentEnabled(true);  // 调用 setAlignmentEnabled（setAlignment已启用），传入 true。
        xTaskCreate(Task_Serial_CMD, "Task_Serial_CMD", 16384, NULL, 5, NULL);  // 调用 xTaskCreate（横向任务Create），传入 Task_Serial_CMD, "Task_Serial_CMD", 16384, NULL, 5, NU…。
    }  // 结束此函数或控制块。
}  // 结束此函数或控制块。

/**
 * @brief 处理后台维护、灯光与按键采样；比赛业务由独立主状态机任务推进。
 */
void loop() {  // 定义函数或成员 的入口 loop，参数由此处的签名确定。
    BatteryMonitorUpdate();  // 调用 BatteryMonitorUpdate，使用无参形式推进相应操作。
    PollDebugBootButton();  // 调用 PollDebugBootButton，使用无参形式推进相应操作。
    // 不依赖主状态机：总超时暂停主任务后仍持续闪灯。
    const RobotState state = currentState;  // 将 currentState 写入或初始化 state（状态）。
    LedPwm_UpdateFaultBlink(state == STATE_SCAN_FAILED || state == STATE_ROUTE_FAILED  // 调用 LedPwm_UpdateFaultBlink（Led占空比输出UpdateFaultBlink），传入 state == STATE_SCAN_FAILED || state == STATE_ROUTE_FAI…。
        || state == STATE_ALIGN_FAILED || state == STATE_TIMEOUT_FAILED  // 继续上一行的组合条件，本行提供 || state == STATE_ALIGN_FAILED || state == STATE_TIMEOUT_FAILED。
        || state == STATE_TRANSFER_FAILED);  // 继续上一行的组合条件，本行提供 || state == STATE_TRANSFER_FAILED);。
    vTaskDelay(pdMS_TO_TICKS(20));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
}  // 结束此函数或控制块。
