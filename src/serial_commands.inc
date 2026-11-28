// 串口命令分发的文本包含文件。
// 任务负责读 Serial0，分发层按命令组和动作处理，异步运动用编号和后续事件表示实际结果。

// 由 robot_runtime.cpp 在业务状态与动作函数之后包含。仅此任务读取 Serial0。
#include <float.h>  // 引入 float.h 中的声明和定义。
#include "debug_release_button.h"  // 引入 debug_release_button.h 中的声明和定义。
#include "start_panel.h"  // 引入 start_panel.h 中的声明和定义。
#include "h30_usb.h"  // 引入 h30_usb.h 中的声明和定义。
#include "continuous_trial.h"  // 引入 continuous_trial.h 中的声明和定义。
#include "disc_position_grab.h"  // 引入 disc_position_grab.h 中的声明和定义。
#include "motion_executor.h"  // 引入 motion_executor.h 中的声明和定义。
#include "node_position_hardware.h"  // 引入 node_position_hardware.h 中的声明和定义。
#include "motor_bus.h"  // 引入 motor_bus.h 中的声明和定义。
#include "ring_correction.h"  // 引入 ring_correction.h 中的声明和定义。
static bool serialDebugMode = true;  // 将 true 写入或初始化 serialDebugMode（serialDebug模式）。
/**
 * @brief 处理连续路径试跑配置、准备、启动、状态和停止指令；返回是否已匹配处理。
 */
static bool serialTrial(const SerialFrame &frame);  // 声明接口 serialTrial，参数由此处的签名确定。
static DebugReleaseButton bootReleaseButton;  // 声明 bootReleaseButton，类型和可用范围由所在声明决定。
static DebugReleaseButton panelStartButton;  // 声明 panelStartButton（panel起点Button），类型和可用范围由所在声明决定。
static portMUX_TYPE bootReleaseMux = portMUX_INITIALIZER_UNLOCKED;  // 将 portMUX_INITIALIZER_UNLOCKED 写入或初始化 bootReleaseMux（bootRelease临界区锁）。
static StartZone panelZone=START_ZONE_UNKNOWN;  // 将 START_ZONE_UNKNOWN 写入或初始化 panelZone。
static StartZone panelStartZone=START_ZONE_UNKNOWN;  // 将 START_ZONE_UNKNOWN 写入或初始化 panelStartZone（panel起点Zone）。
static bool debugReleaseReady=false;  // 将 false 写入或初始化 debugReleaseReady（debugRelease就绪）。
static bool dialReleasePending=false;  // 将 false 写入或初始化 dialReleasePending。
static bool releaseFromDial=false;  // 将 false 写入或初始化 releaseFromDial。

// loop独立采样，串口任务执行阻塞动作时也能识别并忽略按键。
/**
 * @brief 采样启动面板与板载 BOOT，实际 Release 切换由串口任务处理。
 */
void PollDebugBootButton() {  // 定义函数或成员 的入口 PollDebugBootButton，参数由此处的签名确定。
    const bool pressed = digitalRead(BOOT0_PIN) == LOW;  // 将 digitalRead(BOOT0_PIN) == LOW 写入或初始化 pressed。
    const uint32_t nowMs = millis();  // 将 millis() 写入或初始化 nowMs（now毫秒）。
    const bool buttonHigh=digitalRead(START_BUTTON_PIN)==HIGH;  // 将 digitalRead(START_BUTTON_PIN)==HIGH 写入或初始化 buttonHigh。
    const bool zoneHigh=digitalRead(START_ZONE_SWITCH_PIN)==HIGH;  // 将 digitalRead(START_ZONE_SWITCH_PIN)==HIGH 写入或初始化 zoneHigh。
    const auto zone=zoneHigh?START_ZONE_2:START_ZONE_1;  // 将 zoneHigh?START_ZONE_2:START_ZONE_1 写入或初始化 zone。
    portENTER_CRITICAL(&bootReleaseMux);  // 进入临界区，保护共享状态的本段访问。
    bootReleaseButton.update(pressed, nowMs);  // 调用 bootReleaseButton.update，传入 pressed, nowMs。
    if(debugReleaseReady&&panelZone!=START_ZONE_UNKNOWN&&panelZone!=zone)  // 当 debugReleaseReady&&panelZone!=START_ZONE_UNKNOWN&&pane… 成立时进入此分支，否则继续后续流程。
        dialReleasePending=true;  // 将 true 写入或初始化 dialReleasePending。
    panelZone=zone;  // 将 zone 写入或初始化 panelZone。
    const bool wasPanelPressed=panelStartButton.stablePressed;  // 将 panelStartButton.stablePressed 写入或初始化 wasPanelPressed。
    panelStartButton.update(buttonHigh,nowMs);  // 调用 panelStartButton.update，传入 buttonHigh,nowMs。
    const bool panelPressed=!wasPanelPressed&&panelStartButton.stablePressed;  // 将 !wasPanelPressed&&panelStartButton.stablePressed 写入或初始化 panelPressed。
    const bool startRequested=panelStartButton.takeRequest();  // 将 panelStartButton.takeRequest() 写入或初始化 startRequested（起点Requested）。
    if(startRequested)panelStartZone=zone;  // 当 startRequested 成立时进入此分支，否则继续后续流程。
    const bool debugReady=debugReleaseReady;  // 将 debugReleaseReady 写入或初始化 debugReady（debug就绪）。
    const bool waitingForStart=!serialDebugMode&&panelStartZone==START_ZONE_UNKNOWN;  // 将 !serialDebugMode&&panelStartZone==START_ZONE_UNKNOWN 写入或初始化 waitingForStart（waitingFor起点）。
    portEXIT_CRITICAL(&bootReleaseMux);  // 离开临界区，保护共享状态的本段访问。

    if(panelPressed) {  // 当 panelPressed 成立时进入此分支，否则继续后续流程。
        Serial.printf("[StartPanel] button=PRESS start_level=%u zone_level=%u zone=%u start=%s\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
            static_cast<unsigned>(buttonHigh),static_cast<unsigned>(zoneHigh),static_cast<unsigned>(zone),  // 定义函数或成员 unsigned>，参数由此处的签名确定。
            startRequested?"REQUESTED":"IGNORED");  // 续写当前声明或运算，提供 startRequested?"REQUESTED":"IGNORED");。
    }  // 结束此条件分支。

    const RobotState state=currentState;  // 将 currentState 写入或初始化 state（状态）。
    auto light=StartPanelLight::OFF;  // 将 StartPanelLight::OFF 写入或初始化 light。
    if(StartPanelTaskActive(state))light=StartPanelLight::BREATH;  // 当 StartPanelTaskActive(state) 成立时进入此分支，否则继续后续流程。
    else if(waitingForStart&&state==STATE_WAIT_START)  // 仅在先前条件均不成立时，当 waitingForStart&&state==STATE_WAIT_START 成立时进入此分支，否则继续后续流程。
        light=zoneHigh?StartPanelLight::DOUBLE_FLASH:StartPanelLight::SINGLE_FLASH;  // 将 zoneHigh?StartPanelLight::DOUBLE_FLASH:StartPanelLight… 写入或初始化 light。
    else if(debugReady)light=StartPanelLight::STEADY;  // 仅在先前条件均不成立时，当 debugReady 成立时进入此分支，否则继续后续流程。

    static auto lastLight=StartPanelLight::OFF;  // 将 StartPanelLight::OFF 写入或初始化 lastLight（最后Light）。
    static uint32_t lightSince=0;  // 将 0 写入或初始化 lightSince。
    if(light!=lastLight) {  // 当 light!=lastLight 成立时进入此分支，否则继续后续流程。
        lastLight=light;  // 将 light 写入或初始化 lastLight（最后Light）。
        lightSince=nowMs;  // 将 nowMs 写入或初始化 lightSince。
    }  // 结束此条件分支。
    ledcWrite(BUTTON_LED_PWM_CHANNEL,StartPanelLightDuty(light,nowMs-lightSince));  // 调用 ledcWrite，传入 BUTTON_LED_PWM_CHANNEL,StartPanelLightDuty(light,nowMs…。
}  // 结束此函数或控制块。

StartZone ReadPanelStartRequest() {  // 定义函数或成员 的入口 ReadPanelStartRequest（ReadPanel起点请求），参数由此处的签名确定。
    portENTER_CRITICAL(&bootReleaseMux);  // 进入临界区，保护共享状态的本段访问。
    const auto zone=panelStartZone;  // 将 panelStartZone 写入或初始化 zone。
    portEXIT_CRITICAL(&bootReleaseMux);  // 离开临界区，保护共享状态的本段访问。
    return zone;  // 结束当前函数并返回 zone。
}  // 结束此函数或控制块。

StartZone SelectReleaseStartZone(bool waitForPanel) {  // 定义函数或成员 的入口 SelectReleaseStartZone（SelectRelease起点Zone），参数由此处的签名确定。
    portENTER_CRITICAL(&bootReleaseMux);  // 进入临界区，保护共享状态的本段访问。
    const auto zone=panelStartZone!=START_ZONE_UNKNOWN?panelStartZone:  // 将 panelStartZone!=START_ZONE_UNKNOWN?panelStartZone: 写入或初始化 zone。
        waitForPanel?START_ZONE_UNKNOWN:currentStartZone;  // 续写当前声明或运算，提供 waitForPanel?START_ZONE_UNKNOWN:currentStartZone;。
    panelStartButton.setIdle(zone==START_ZONE_UNKNOWN);  // 调用 panelStartButton.setIdle，传入 zone==START_ZONE_UNKNOWN。
    portEXIT_CRITICAL(&bootReleaseMux);  // 离开临界区，保护共享状态的本段访问。
    return zone;  // 结束当前函数并返回 zone。
}  // 结束此函数或控制块。

/**
 * @brief 更新按钮Release可用状态；只有调试空闲且试跑、运动执行器均空闲时允许触发。
 */
static void serialBootSetIdle(bool idle) {  // 定义函数或成员 的入口 serialBootSetIdle，参数由此处的签名确定。
    portENTER_CRITICAL(&bootReleaseMux);  // 进入临界区，保护共享状态的本段访问。
    const bool ready=serialDebugMode&&idle&&!ContinuousTrialActive()&&!MotionExecutorActive();  // 将 serialDebugMode&&idle&&!ContinuousTrialActive()&&!Moti… 写入或初始化 ready（就绪）。
    bootReleaseButton.setIdle(ready);  // 调用 bootReleaseButton.setIdle，传入 ready。
    debugReleaseReady=ready;  // 将 ready 写入或初始化 debugReleaseReady（debugRelease就绪）。
    portEXIT_CRITICAL(&bootReleaseMux);  // 离开临界区，保护共享状态的本段访问。
}  // 结束此函数或控制块。
/**
 * @brief 按原请求类别和动作发送指定状态的RSP串口应答。
 */
static void serialReply(const SerialFrame &frame, const char *status) {  // 定义函数或成员 的入口 serialReply，参数由此处的签名确定。
    Serial.printf("{RSP,%s,%s,%s}\n", frame.fields[1], frame.fields[2], status);  // 调用 Serial.printf，传入 "{RSP,%s,%s,%s}\n", frame.fields[1], frame.fields[2], …。
}  // 结束此函数或控制块。

/**
 * @brief 复制串口应答上下文并暂存运动作业，等待ACK后放行；返回作业编号，0表示未提交。
 */
// 串口请求先尝试进入队列，提交返回的编号用于回复及后续的 Launch。
// 遇到运动忙或队列错误时可从这里追踪回复。
static uint32_t serialPrepareMotion(const SerialFrame &frame,MotionRequest &request) {  // 定义函数或成员 的入口 serialPrepareMotion，参数由此处的签名确定。
    snprintf(request.category,sizeof(request.category),"%s",frame.fields[1]);  // 调用 snprintf，传入 request.category,sizeof(request.category),"%s",frame.f…。
    snprintf(request.command,sizeof(request.command),"%s",frame.fields[2]);  // 调用 snprintf，传入 request.command,sizeof(request.command),"%s",frame.fie…。
    const uint32_t id=SubmitMotion(request,false);  // 将 SubmitMotion(request,false) 写入或初始化 id（编号）。
    if(!id)serialError(frame,"BUSY");  // 当 !id 成立时进入此分支，否则继续后续流程。
    return id;  // 结束当前函数并返回 id。
}  // 结束此函数或控制块。
/**
 * @brief 准备运动作业，先发ACK，再通过LaunchMotion允许工作任务执行。
 */
static void serialMotion(const SerialFrame &frame,MotionRequest &request) {  // 定义函数或成员 的入口 serialMotion，参数由此处的签名确定。
    const uint32_t id=serialPrepareMotion(frame,request);if(!id)return;  // 将 serialPrepareMotion(frame,request) 写入或初始化 id（编号）。然后当 !id 成立时进入此分支，否则继续后续流程。
    serialReply(frame,"ACK");LaunchMotion(id);  // 调用 serialReply，传入 frame,"ACK"。然后调用 LaunchMotion，传入 id。
}  // 结束此函数或控制块。

/**
 * @brief 发送参数配置请求的错误应答，reason为错误原因。
 */
static void serialCfgError(const SerialFrame &frame, const char *reason) {  // 定义函数或成员 的入口 serialCfgError（serial配置错误），参数由此处的签名确定。
    uint32_t id = 0;  // 将 0 写入或初始化 id（编号）。
    if (frame.count > 3) serialUInt(frame.fields[3], id, UINT32_MAX);  // 当 frame.count > 3 成立时进入此分支，否则继续后续流程。
    Serial.printf("{RSP,CFG,%s,ERR,%lu,%s}\n", frame.fields[2],  // 通过串口执行 printf，输出内容或参数由本行给出。
                  static_cast<unsigned long>(id), reason);  // 声明接口 long>，参数由此处的签名确定。
}  // 结束此函数或控制块。

/**
 * @brief 检查帧中的类别和动作名是否属于已支持的串口指令。
 */
static bool serialKnownAction(const SerialFrame &frame) {  // 定义函数或成员 的入口 serialKnownAction，参数由此处的签名确定。
    static const char *const names[][2] = {  // 声明 names，类型和可用范围由所在声明决定。
        {"SYS", "HELLO"}, {"SYS", "RELEASE"}, {"SYS", "READY"},  // 为当前列表或跨行表达式提供这一项 {"SYS", "HELLO"}, {"SYS", "RELEASE"}, {"SYS", "READY"}。
        {"SYS", "START"}, {"SYS", "HELP"}, {"SYS", "REBOOT"}, {"SYS", "BEEP"}, {"SYS", "PANEL_GET"},  // 为当前列表或跨行表达式提供这一项 {"SYS", "START"}, {"SYS", "HELP"}, {"SYS", "REBOOT"}, …。
        {"MOTION", "STATUS"}, {"MOTION", "RESUME_MATERIAL"},  // 为当前列表或跨行表达式提供这一项 {"MOTION", "STATUS"}, {"MOTION", "RESUME_MATERIAL"}。
        {"EMM", "AXIS_TRIAL"},  // 为当前列表或跨行表达式提供这一项 {"EMM", "AXIS_TRIAL"}。
        {"TRAY", "ROTATE"}, {"TRAY", "ZERO"},  // 为当前列表或跨行表达式提供这一项 {"TRAY", "ROTATE"}, {"TRAY", "ZERO"}。
        {"TASK", "SET"},  // 为当前列表或跨行表达式提供这一项 {"TASK", "SET"}。
        {"LED", "SET"}, {"LED", "GET"}, {"LED", "FREQ"},  // 为当前列表或跨行表达式提供这一项 {"LED", "SET"}, {"LED", "GET"}, {"LED", "FREQ"}。
        {"NAV", "START_ZONE"}, {"NAV", "ROUTE"}, {"NAV", "STOP"},  // 为当前列表或跨行表达式提供这一项 {"NAV", "START_ZONE"}, {"NAV", "ROUTE"}, {"NAV", "STOP…。
        {"NAV", "TRIAL_BEGIN"}, {"NAV", "TRIAL_POLICY"}, {"NAV", "TRIAL_NODE"}, {"NAV", "TRIAL_START"},  // 为当前列表或跨行表达式提供这一项 {"NAV", "TRIAL_BEGIN"}, {"NAV", "TRIAL_POLICY"}, {"NAV…。
        {"NAV", "TRIAL_STATUS"}, {"NAV", "TRIAL_STOP"},  // 为当前列表或跨行表达式提供这一项 {"NAV", "TRIAL_STATUS"}, {"NAV", "TRIAL_STOP"}。
        {"VISION", "ALIGN_START"}, {"VISION", "ALIGN_STOP"},  // 为当前列表或跨行表达式提供这一项 {"VISION", "ALIGN_START"}, {"VISION", "ALIGN_STOP"}。
        {"VISION", "START_REQUEST"},  // 为当前列表或跨行表达式提供这一项 {"VISION", "START_REQUEST"}。
        {"VISION", "STOP_REQUEST"},  // 为当前列表或跨行表达式提供这一项 {"VISION", "STOP_REQUEST"}。
        {"VISION", "RING_OFFSET"}, {"VISION", "RING_AXIS_PID_GET"}, {"VISION", "RING_AXIS_PID_SET"},  // 为当前列表或跨行表达式提供这一项 {"VISION", "RING_OFFSET"}, {"VISION", "RING_AXIS_PID_G…。
        {"VISION", "ALIGN_DATA"}, {"VISION", "COLOR"},  // 为当前列表或跨行表达式提供这一项 {"VISION", "ALIGN_DATA"}, {"VISION", "COLOR"}。
        {"VISION", "GRAB"},  // 为当前列表或跨行表达式提供这一项 {"VISION", "GRAB"}。
        {"VISION", "DISC_POSITIONS"},  // 为当前列表或跨行表达式提供这一项 {"VISION", "DISC_POSITIONS"}。
        {"VISION", "FORCE_GRAB"},  // 为当前列表或跨行表达式提供这一项 {"VISION", "FORCE_GRAB"}。
        {"VISION", "CONFIRM"}, {"VISION", "LIDAR_POSE"},  // 为当前列表或跨行表达式提供这一项 {"VISION", "CONFIRM"}, {"VISION", "LIDAR_POSE"}。
        {"POSE", "GET"}, {"POSE", "SET"}, {"POSE", "GOTO_REL"},  // 为当前列表或跨行表达式提供这一项 {"POSE", "GET"}, {"POSE", "SET"}, {"POSE", "GOTO_REL"}。
        {"IMU", "GET"},  // 为当前列表或跨行表达式提供这一项 {"IMU", "GET"}。
        {"IMU", "HEADING"},  // 为当前列表或跨行表达式提供这一项 {"IMU", "HEADING"}。
        {"CHASSIS", "MOVE"}, {"CHASSIS", "MOVE_POSITION"},  // 为当前列表或跨行表达式提供这一项 {"CHASSIS", "MOVE"}, {"CHASSIS", "MOVE_POSITION"}。
        {"ARM", "MOVE1"}, {"ARM", "MOVE2"}, {"ARM", "SERVO"}, {"ARM", "SERVO_ZERO"},  // 为当前列表或跨行表达式提供这一项 {"ARM", "MOVE1"}, {"ARM", "MOVE2"}, {"ARM", "SERVO"}, …。
        {"ARM", "SERVO_PID_GET"}, {"ARM", "SERVO_PID_SET"},  // 为当前列表或跨行表达式提供这一项 {"ARM", "SERVO_PID_GET"}, {"ARM", "SERVO_PID_SET"}。
        {"ARM", "ENABLE"}, {"ARM", "DEMO1"}, {"ARM", "DEMO2"}, {"ARM", "DEMO3"},  // 为当前列表或跨行表达式提供这一项 {"ARM", "ENABLE"}, {"ARM", "DEMO1"}, {"ARM", "DEMO2"},…。
        {"ARM", "DEMO4"}, {"ARM", "TRANSFER"}, {"ARM", "RETRIEVE"},  // 为当前列表或跨行表达式提供这一项 {"ARM", "DEMO4"}, {"ARM", "TRANSFER"}, {"ARM", "RETRIE…。
        {"ARM", "INIT_START"}, {"ARM", "INIT_LOOK"}, {"ARM", "INIT_LOOK2"},  // 为当前列表或跨行表达式提供这一项 {"ARM", "INIT_START"}, {"ARM", "INIT_LOOK"}, {"ARM", "…。
        {"ARM", "RING_PREPARE"}, {"ARM", "RING_DEMO"},  // 为当前列表或跨行表达式提供这一项 {"ARM", "RING_PREPARE"}, {"ARM", "RING_DEMO"}。
        {"CFG", "GET"}, {"CFG", "BEGIN"}, {"CFG", "SET"}, {"CFG", "COMMIT"},  // 为当前列表或跨行表达式提供这一项 {"CFG", "GET"}, {"CFG", "BEGIN"}, {"CFG", "SET"}, {"CF…。
    };  // 结束此初始化列表或局部作用域。
    for (const auto &name : names) {  // 按 const auto &name : names 组织循环的初值、继续条件和步进。
        if (strcmp(frame.fields[1], name[0]) == 0  // 当 strcmp(frame.fields[1], name[0]) == 0 成立时进入此分支，否则继续后续流程。
                && strcmp(frame.fields[2], name[1]) == 0) return true;  // 继续上一行的组合条件，本行提供 && strcmp(frame.fields[2], name[1]) == 0) return true;。
    }  // 结束此循环体。
    return false;  // 结束当前函数并返回 false。
}  // 结束此函数或控制块。

/**
 * @brief 按各字段范围解析一组浮点参数；失败时输出相应协议错误并返回false。
 */
static bool serialFloats(const SerialFrame &frame, float *values, const float *minimum, const float *maximum, size_t count) {  // 定义函数或成员 的入口 serialFloats，参数由此处的签名确定。
    if (frame.count != count + 3) return false;  // 若 frame.count != count + 3，立即返回 false。
    for (size_t i = 0; i < count; ++i) {  // 按 size_t i = 0; i < count; ++i 组织循环的初值、继续条件和步进。
        if (!serialFloat(frame.fields[i + 3], values[i], minimum[i], maximum[i])) return false;  // 若 !serialFloat(frame.fields[i + 3], values[i], minimum[i…，立即返回 false。
    }  // 结束此循环体。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

/**
 * @brief 处理节点路径串口请求，区分自动路径回传与调试路径执行。
 */
static void serialRoute(const SerialFrame &frame) {  // 定义函数或成员 的入口 serialRoute（serial路线），参数由此处的签名确定。
    const size_t count = frame.count - 3;  // 将 frame.count - 3 写入或初始化 count（计数）。
    if (count < 2 || count > MAX_NODE_PATH_LENGTH) {  // 当 count < 2 || count > MAX_NODE_PATH_LENGTH 成立时进入此分支，否则继续后续流程。
        serialError(frame, count < 2 ? "FORMAT" : "RANGE");  // 调用 serialError（serial错误），传入 frame, count < 2 ? "FORMAT" : "RANGE"。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    uint8_t path[MAX_NODE_PATH_LENGTH] = {0};  // 将 {0} 写入或初始化 path[MAX_NODE_PATH_LENGTH]。
    for (size_t i = 0; i < count; ++i) {  // 按 size_t i = 0; i < count; ++i 组织循环的初值、继续条件和步进。
        uint32_t node = 0;  // 将 0 写入或初始化 node（节点）。
        if (!serialUInt(frame.fields[i + 3], node, FIELD_NODE_COUNT - 1)) {  // 当 !serialUInt(frame.fields[i + 3], node, FIELD_NODE_COUN… 成立时进入此分支，否则继续后续流程。
            serialError(frame, "RANGE");  // 调用 serialError（serial错误），传入 frame, "RANGE"。
            return;  // 结束当前函数，将执行权交回调用者。
        }  // 结束此条件分支。
        path[i] = static_cast<uint8_t>(node);  // 将 static_cast<uint8_t>(node) 写入或初始化 path[i]。
    }  // 结束此循环体。
    if(serialDebugMode) {  // 当 serialDebugMode 成立时进入此分支，否则继续后续流程。
        MotionRequest r;r.kind=MotionKind::NODE_PATH;r.pathCount=count;r.theta=NAN;r.speed=NODE_PATH_SPEED_MM_S;r.automatic=true;  // 声明 r，类型和可用范围由所在声明决定。然后将 MotionKind::NODE_PATH 写入或初始化 r.kind。然后将 count 写入或初始化 r.pathCount。然后将 NAN 写入或初始化 r.theta。然后将 NODE_PATH_SPEED_MM_S 写入或初始化 r.speed。然后将 true 写入或初始化 r.automatic。
        memcpy(r.path,path,count);const uint32_t id=SubmitMotion(r,false);  // 调用 memcpy，传入 r.path,path,count。然后将 SubmitMotion(r,false) 写入或初始化 id（编号）。
        if(!id){serialError(frame,"BUSY");return;}  // 当 !id 成立时进入此分支，否则继续后续流程。
        Serial.printf("{RSP,NAV,ROUTE,ACK,%u}\n",static_cast<unsigned>(count));LaunchMotion(id);return;  // 通过串口执行 printf，输出内容或参数由本行给出。
    }  // 结束此条件分支。

    portENTER_CRITICAL(&nodePathMux);  // 进入临界区，保护共享状态的本段访问。
    const bool accepting = nodePathState == NodePathState::WAITING  // 将 nodePathState == NodePathState::WAITING 写入或初始化 accepting。
        && millis() - nodePathRequestedAt < ROUTE_REPLY_TIMEOUT_MS;  // 继续上一行的组合条件，本行提供 && millis() - nodePathRequestedAt < ROUTE_REPLY_TIMEOUT_MS;。
    if (accepting) {  // 当 accepting 成立时进入此分支，否则继续后续流程。
        memcpy(nodePathBuffer, path, count);  // 调用 memcpy，传入 nodePathBuffer, path, count。
        nodePathLen = count;  // 将 count 写入或初始化 nodePathLen（节点路径Len）。
        nodePathState = NodePathState::RECEIVING;  // 将 NodePathState::RECEIVING 写入或初始化 nodePathState（节点路径状态）。
    }  // 结束此条件分支。
    portEXIT_CRITICAL(&nodePathMux);  // 离开临界区，保护共享状态的本段访问。
    if (!accepting) {  // 当 !accepting 成立时进入此分支，否则继续后续流程。
        serialError(frame, "NOT_WAITING");  // 调用 serialError（serial错误），传入 frame, "NOT_WAITING"。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    Serial.printf("{RSP,NAV,ROUTE,ACK,%u}\n", static_cast<unsigned>(count));  // 调用 Serial.printf，传入 "{RSP,NAV,ROUTE,ACK,%u}\n", static_cast<unsigned>(coun…。
    // 应答先于主状态机执行事件，避免上位机把 ACK 当成完成。
    portENTER_CRITICAL(&nodePathMux);  // 进入临界区，保护共享状态的本段访问。
    if (nodePathState == NodePathState::RECEIVING) nodePathState = NodePathState::RECEIVED;  // 当 nodePathState == NodePathState::RECEIVING 成立时进入此分支，否则继续后续流程。
    portEXIT_CRITICAL(&nodePathMux);  // 离开临界区，保护共享状态的本段访问。
}  // 结束此函数或控制块。

/**
 * @brief 处理进入正式运行模式，重置比赛准备状态并启动或恢复主状态机任务。
 */
static void serialRelease(const SerialFrame &frame,bool fromDial=false) {  // 定义函数或成员 的入口 serialRelease，参数由此处的签名确定。
    if (!serialDebugMode) { serialError(frame, "MODE"); return; }  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
    if (discMaterialActive) requestVisionStop(VisionStartMode::DISC_MATERIAL);  // 当 discMaterialActive 成立时进入此分支，否则继续后续流程。
    setAlignmentEnabled(false);  // 调用 setAlignmentEnabled（setAlignment已启用），传入 false。
    xHomeTimer = xTimerCreate("HomeTimer", pdMS_TO_TICKS(3000000), pdFALSE,// 3000s 后强制返回起停区
                             NULL, vHomeTimerCallback);  // 续写当前声明或运算，提供 NULL, vHomeTimerCallback);。
    if (!xHomeTimer) { serialError(frame, "TIMER"); return; }  // 当 !xHomeTimer 成立时进入此分支，否则继续后续流程。
    currentState = STATE_WAIT_START;  // 将 STATE_WAIT_START 写入或初始化 currentState（当前状态）。
    taskMotionAborted = false;  // 将 false 写入或初始化 taskMotionAborted（任务MotionAborted）。
    firstDiscGrabReady = false;  // 将 false 写入或初始化 firstDiscGrabReady（首个圆盘Grab就绪）。
    enableRun = false;  // 将 false 写入或初始化 enableRun。
    if (xTaskCreate(Task_MainStateMachine, "Task_MainStateMachine", 16384,  // 当 xTaskCreate(Task_MainStateMachine, "Task_MainStateMach… 成立时进入此分支，否则继续后续流程。
                    fromDial?&dialReleasePending:nullptr, 8, &xTask_MainStateMachine_Handle) != pdPASS) {  // 续写当前声明或运算，提供 fromDial?&dialReleasePending:nullptr, 8, &xTask_MainSt…。
        xTimerDelete(xHomeTimer, portMAX_DELAY);  // 调用 xTimerDelete（横向定时器Delete），传入 xHomeTimer, portMAX_DELAY。
        xHomeTimer = NULL;  // 将 NULL 写入或初始化 xHomeTimer（横向Home定时器）。
        serialError(frame, "TASK");  // 调用 serialError（serial错误），传入 frame, "TASK"。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此函数或控制块。
    portENTER_CRITICAL(&bootReleaseMux);  // 进入临界区，保护共享状态的本段访问。
    serialDebugMode = false;  // 将 false 写入或初始化 serialDebugMode（serialDebug模式）。
    portEXIT_CRITICAL(&bootReleaseMux);  // 离开临界区，保护共享状态的本段访问。
    serialBootSetIdle(false);  // 调用 serialBootSetIdle，传入 false。
    releaseFromDial=fromDial;  // 将 fromDial 写入或初始化 releaseFromDial。
    LedPwm_PlayCue(2);  // 调用 LedPwm_PlayCue（Led占空比输出PlayCue），传入 2。
    leds[0] = CRGB::Green;  // 将 CRGB::Green 写入或初始化 leds[0]。
    FastLED.show();  // 调用 FastLED.show。
    Serial.println("Release mode");  // 调用 Serial.println，传入 "Release mode"。
    serialReply(frame, "OK");  // 调用 serialReply，传入 frame, "OK"。
}  // 结束此初始化列表或局部作用域。

/**
 * @brief 在串口空闲时处理板载按钮或拨杆触发的 Release。
 */
static void serialBootReleaseAtIdle(bool idle) {  // 定义函数或成员 的入口 serialBootReleaseAtIdle，参数由此处的签名确定。
    serialBootSetIdle(idle);  // 调用 serialBootSetIdle，传入 idle。
    portENTER_CRITICAL(&bootReleaseMux);  // 进入临界区，保护共享状态的本段访问。
    const bool requested = bootReleaseButton.takeRequest();  // 将 bootReleaseButton.takeRequest() 写入或初始化 requested。
    const bool fromDial=debugReleaseReady&&dialReleasePending;  // 将 debugReleaseReady&&dialReleasePending 写入或初始化 fromDial。
    if(fromDial||!serialDebugMode)dialReleasePending=false;  // 当 fromDial||!serialDebugMode 成立时进入此分支，否则继续后续流程。
    portEXIT_CRITICAL(&bootReleaseMux);  // 离开临界区，保护共享状态的本段访问。
    if (!requested&&!fromDial) return;  // 当 !requested&&!fromDial 成立时进入此分支，否则继续后续流程。
    // 与串口命令同一个任务、同一个入口，避免并发创建定时器/主状态机。
    char command[] = "{CMD,SYS,RELEASE}";  // 声明 command，类型和可用范围由所在声明决定。
    SerialFrame frame;  // 声明 frame（帧），类型和可用范围由所在声明决定。
    if (fromDial)Serial.println("[StartPanel] dial=CHANGED release=REQUESTED");  // 当 fromDial 成立时进入此分支，否则继续后续流程。
    if (parseSerialFrame(command, frame)) serialRelease(frame,fromDial);  // 当 parseSerialFrame(command, frame) 成立时进入此分支，否则继续后续流程。
}  // 结束此函数或控制块。

// 串口只提交；ACK、选择事件先于 Launch，实际完成及进度由运动任务发布。
/**
 * @brief 构造并提交圆盘抓放请求，附颜色、载物台、位置及视觉应答上下文；返回作业编号。
 */
static uint32_t serialQueueDiscGrab(uint8_t color,uint8_t cargo,uint8_t position=0,uint8_t ack=0,size_t count=0,bool force=false) {  // 定义函数或成员 的入口 serialQueueDiscGrab（serialQueue圆盘Grab），参数由此处的签名确定。
    MotionRequest r;r.kind=MotionKind::DISC_GRAB;r.color=color;r.cargo=cargo;r.position=position;r.force=force;  // 声明 r，类型和可用范围由所在声明决定。然后将 MotionKind::DISC_GRAB 写入或初始化 r.kind。然后将 color 写入或初始化 r.color。然后将 cargo 写入或初始化 r.cargo。然后将 position 写入或初始化 r.position。然后将 force 写入或初始化 r.force。
    r.automatic=!force&&!serialDebugMode&&(currentState==STATE_GRAB_ROUND1||currentState==STATE_GRAB_ROUND2);  // 将 !force&&!serialDebugMode&&(currentState==STATE_GRAB_RO… 写入或初始化 r.automatic。
    const uint32_t id=SubmitMotion(r,false);if(!id)return 0;  // 将 SubmitMotion(r,false) 写入或初始化 id（编号）。然后若 !id，立即返回 0。
    if(r.automatic) {  // 当 r.automatic 成立时进入此分支，否则继续后续流程。
        firstDiscGrabReady=false;discMessageResumeRequested=false;  // 将 false 写入或初始化 firstDiscGrabReady（首个圆盘Grab就绪）。然后将 false 写入或初始化 discMessageResumeRequested（圆盘MessageResumeRequested）。
        traceVision(VisionStartMode::DISC_MATERIAL,false);  // 调用 traceVision（trace视觉），传入 VisionStartMode::DISC_MATERIAL,false。
        if(position)requestVisionStop(VisionStartMode::DISC_MATERIAL);  // 当 position 成立时进入此分支，否则继续后续流程。
    }  // 结束此条件分支。
    discMaterialColor=0;  // 将 0 写入或初始化 discMaterialColor（圆盘物料颜色）。
    if(ack==1)Serial.printf("{RSP,VISION,GRAB,ACK,%u,%u}\n",color,cargo);  // 当 ack==1 成立时进入此分支，否则继续后续流程。
    else if(ack==2) {  // 仅在先前条件均不成立时，当 ack==2 成立时进入此分支，否则继续后续流程。
        Serial.printf("{RSP,VISION,DISC_POSITIONS,ACK,%u}\n",static_cast<unsigned>(count));  // 调用 Serial.printf，传入 "{RSP,VISION,DISC_POSITIONS,ACK,%u}\n",static_cast<uns…。
        Serial.printf("{EVT,VISION,DISC_POSITIONS,SELECTED,%u,%u,%u}\n",position,color,cargo);  // 调用 Serial.printf，传入 "{EVT,VISION,DISC_POSITIONS,SELECTED,%u,%u,%u}\n",posi…。
    } else if(ack==3) {  // 仅在先前条件均不成立时，当 ack==3 成立时进入此分支，否则继续后续流程。
        Serial.printf("{RSP,VISION,FORCE_GRAB,ACK,%u,%u}\n",color,cargo);  // 调用 Serial.printf，传入 "{RSP,VISION,FORCE_GRAB,ACK,%u,%u}\n",color,cargo。
        Serial.printf("{EVT,VISION,FORCE_GRAB,REQUESTED,%u,%u}\n",color,cargo);  // 调用 Serial.printf，传入 "{EVT,VISION,FORCE_GRAB,REQUESTED,%u,%u}\n",color,cargo。
    }  // 结束此条件分支。
    LaunchMotion(id);return id;  // 调用 LaunchMotion，传入 id。然后结束当前函数并返回 id。
}  // 结束此初始化列表或局部作用域。
/**
 * @brief 按任务颜色和目标载物台提交自动圆盘抓放作业。
 */
static void serialExecuteTaskGrab(uint8_t color,uint8_t cargo) {  // 定义函数或成员 的入口 serialExecuteTaskGrab（serialExecute任务Grab），参数由此处的签名确定。
    serialQueueDiscGrab(color,cargo);  // 调用 serialQueueDiscGrab（serialQueue圆盘Grab），传入 color,cargo。
}  // 结束此函数或控制块。

/**
 * @brief 按当前任务进度评估识别颜色，满足条件后触发抓取，否则发送拒绝原因。
 */
static void serialDiscColorDecision(uint8_t color) {  // 定义函数或成员 的入口 serialDiscColorDecision（serial圆盘颜色Decision），参数由此处的签名确定。
    const RobotState state = currentState;  // 将 currentState 写入或初始化 state（状态）。
    const int progress = roundProgress;  // 将 roundProgress 写入或初始化 progress。
    const int *colors = state == STATE_GRAB_ROUND2 ? currentTask.round2_colors  // 将 state == STATE_GRAB_ROUND2 ? currentTask.round2_colors 写入或初始化 colors。
        : state == STATE_GRAB_ROUND1 || serialDebugMode ? currentTask.round1_colors : nullptr;  // 续写上一行表达式的条件取值分支。
    const uint8_t cargo = progress >= 0 && progress < 3 ? CargoForGrabIndex(progress) : 0;  // 将 progress >= 0 && progress < 3 ? CargoForGrabIndex(prog… 写入或初始化 cargo（货位）。
    const auto decision = EvaluateDiscColor(colors, progress, color, currentTask.valid,  // 将 EvaluateDiscColor(colors, progress, color, currentTask… 写入或初始化 decision。
        alignmentEnabled, serialDebugMode || firstDiscGrabReady,  // 为当前列表或跨行表达式提供这一项 alignmentEnabled, serialDebugMode || firstDiscGrabReady。
        cargo != 0 && cargoPlatforms[cargo - 1].material != MATERIAL_NONE);  // 续写当前声明或运算，提供 cargo != 0 && cargoPlatforms[cargo - 1].material != MA…。
    if (decision == DiscColorDecision::GRAB) {  // 当 decision == DiscColorDecision::GRAB 成立时进入此分支，否则继续后续流程。
        Serial.printf("{EVT,VISION,COLOR,GRAB,%u,%u}\n", color, cargo);  // 调用 Serial.printf，传入 "{EVT,VISION,COLOR,GRAB,%u,%u}\n", color, cargo。
        serialExecuteTaskGrab(color, cargo);  // 调用 serialExecuteTaskGrab（serialExecute任务Grab），传入 color, cargo。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    const char *reason = "";  // 将 "" 写入或初始化 reason。
    switch (decision) {  // 按 decision 的值选择对应分支。
        case DiscColorDecision::COLOR_MISMATCH: reason = "COLOR_MISMATCH"; break;  // 当前值匹配 DiscColorDecision::COLOR_MISMATCH 时从这里执行。
        case DiscColorDecision::NO_TASK: reason = "NO_TASK"; break;  // 当前值匹配 DiscColorDecision::NO_TASK 时从这里执行。
        case DiscColorDecision::NOT_WAITING: reason = "NOT_WAITING"; break;  // 当前值匹配 DiscColorDecision::NOT_WAITING 时从这里执行。
        case DiscColorDecision::ROUND_COMPLETE: reason = "ROUND_COMPLETE"; break;  // 当前值匹配 DiscColorDecision::ROUND_COMPLETE 时从这里执行。
        case DiscColorDecision::ALIGN_ACTIVE: reason = "ALIGN_ACTIVE"; break;  // 当前值匹配 DiscColorDecision::ALIGN_ACTIVE 时从这里执行。
        case DiscColorDecision::NOT_READY: reason = "NOT_READY"; break;  // 当前值匹配 DiscColorDecision::NOT_READY 时从这里执行。
        case DiscColorDecision::OCCUPIED: reason = "OCCUPIED"; break;  // 当前值匹配 DiscColorDecision::OCCUPIED 时从这里执行。
        case DiscColorDecision::GRAB: return;  // 当前值匹配 DiscColorDecision::GRAB 时从这里执行。
    }  // 结束此函数或控制块。
    Serial.printf("{EVT,VISION,COLOR,%s,%u,%u,%s}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
        decision == DiscColorDecision::COLOR_MISMATCH ? "SKIP" : "BLOCKED", color, cargo, reason);  // 续写当前声明或运算，提供 decision == DiscColorDecision::COLOR_MISMATCH ? "SKIP"…。
    if (colors != nullptr && progress >= 0 && progress < 3 && currentTask.valid) {  // 当 colors != nullptr && progress >= 0 && progress < 3 && … 成立时进入此分支，否则继续后续流程。
        Serial.printf("[COLOR] received=%u, expected=%d, cargo=%u, reason=%s\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                      color, colors[progress], cargo, reason);  // 声明 colors，类型和可用范围由所在声明决定。
    }  // 结束此条件分支。
}  // 结束此函数或控制块。

// 显式 GRAB 与 COLOR 共用任务进度和完成事件。
/**
 * @brief 处理串口单件抓取、强制抓取等圆盘指令，并提交对应运动作业。
 */
static void serialDiscGrab(const SerialFrame &frame) {  // 定义函数或成员 的入口 serialDiscGrab（serial圆盘Grab），参数由此处的签名确定。
    uint32_t color = discMaterialColor;  // 将 discMaterialColor 写入或初始化 color（颜色）。
    uint32_t cargo = 0;  // 将 0 写入或初始化 cargo（货位）。
    const bool explicitColor = frame.count == 5;  // 将 frame.count == 5 写入或初始化 explicitColor（explicit颜色）。
    if ((explicitColor && (!serialUInt(frame.fields[3], color, 6) || color == 0))  // 当 (explicitColor && (!serialUInt(frame.fields[3], color,… 成立时进入此分支，否则继续后续流程。
            || !serialUInt(frame.fields[explicitColor ? 4 : 3], cargo, 3) || cargo == 0) {  // 继续上一行的组合条件，本行提供 || !serialUInt(frame.fields[explicitColor ? 4 : 3], cargo, 3) || cargo == 0) {。
        serialError(frame, "RANGE"); return;  // 调用 serialError（serial错误），传入 frame, "RANGE"。然后结束当前函数，将执行权交回调用者。
    }  // 结束此函数或控制块。
    if (!discMaterialActive) { serialError(frame, "NOT_ACTIVE"); return; }  // 当 !discMaterialActive 成立时进入此分支，否则继续后续流程。
    if (alignmentEnabled) { serialError(frame, "ALIGN_ACTIVE"); return; }  // 当 alignmentEnabled 成立时进入此分支，否则继续后续流程。
    if (color == 0) { serialError(frame, "NO_COLOR"); return; }  // 当 color == 0 成立时进入此分支，否则继续后续流程。
    const RobotState grabState = serialDebugMode ? STATE_GRAB_ROUND1 : currentState;  // 将 serialDebugMode ? STATE_GRAB_ROUND1 : currentState 写入或初始化 grabState（grab状态）。
    {  // 为当前列表或跨行表达式提供这一项 {。
        if ((grabState != STATE_GRAB_ROUND1 && grabState != STATE_GRAB_ROUND2)  // 当 (grabState != STATE_GRAB_ROUND1 && grabState != STATE_… 成立时进入此分支，否则继续后续流程。
                || (!serialDebugMode && !firstDiscGrabReady)  // 继续上一行的组合条件，本行提供 || (!serialDebugMode && !firstDiscGrabReady)。
                || roundProgress < 0 || roundProgress >= 3) {  // 继续上一行的组合条件，本行提供 || roundProgress < 0 || roundProgress >= 3) {。
            serialError(frame, "NOT_WAITING"); return;  // 调用 serialError（serial错误），传入 frame, "NOT_WAITING"。然后结束当前函数，将执行权交回调用者。
        }  // 结束此初始化列表或局部作用域。
        const int *colors = grabState == STATE_GRAB_ROUND1  // 将 grabState == STATE_GRAB_ROUND1 写入或初始化 colors。
            ? currentTask.round1_colors : currentTask.round2_colors;  // 续写上一行表达式的条件取值分支。
        if (cargo != CargoForGrabIndex(roundProgress)) {  // 当 cargo != CargoForGrabIndex(roundProgress) 成立时进入此分支，否则继续后续流程。
            serialError(frame, "ORDER"); return;  // 调用 serialError（serial错误），传入 frame, "ORDER"。然后结束当前函数，将执行权交回调用者。
        }  // 结束此条件分支。
        if (!ShouldGrabTaskColor(colors, roundProgress, static_cast<uint8_t>(color), currentTask.valid)) {  // 当 !ShouldGrabTaskColor(colors, roundProgress, static_cas… 成立时进入此分支，否则继续后续流程。
            serialError(frame, "COLOR_MISMATCH"); return;  // 调用 serialError（serial错误），传入 frame, "COLOR_MISMATCH"。然后结束当前函数，将执行权交回调用者。
        }  // 结束此条件分支。
    }  // 结束此初始化列表或局部作用域。
    if (cargoPlatforms[cargo - 1].material != MATERIAL_NONE) {  // 当 cargoPlatforms[cargo - 1].material != MATERIAL_NONE 成立时进入此分支，否则继续后续流程。
        serialError(frame, "OCCUPIED"); return;  // 调用 serialError（serial错误），传入 frame, "OCCUPIED"。然后结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    discMaterialColor = 0; // 消费颜色，下一次抓取必须有新的颜色反馈或显式颜色参数。
    if(!serialQueueDiscGrab(color,cargo,0,1))serialError(frame,"BUSY");  // 当 !serialQueueDiscGrab(color,cargo,0,1) 成立时进入此分支，否则继续后续流程。
}  // 结束此函数或控制块。

// 视觉保证本帧内全部位置此刻可抓；只选一件，不缓存未选中的候选。
/**
 * @brief 解析一帧圆盘候选位置和颜色，选出下一件任务目标并提交抓取。
 */
static void serialDiscPositions(const SerialFrame &frame) {  // 定义函数或成员 的入口 serialDiscPositions（serial圆盘Positions），参数由此处的签名确定。
    if (frame.count < 5 || frame.count > 9 || (frame.count - 3) % 2 != 0) {  // 当 frame.count < 5 || frame.count > 9 || (frame.count - 3… 成立时进入此分支，否则继续后续流程。
        serialError(frame, "FORMAT"); return;  // 调用 serialError（serial错误），传入 frame, "FORMAT"。然后结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    DiscPositionCandidate candidates[3] = {};  // 将 {} 写入或初始化 candidates[3]。
    const size_t count = (frame.count - 3) / 2;  // 将 (frame.count - 3) / 2 写入或初始化 count（计数）。
    for (size_t i = 0; i < count; ++i) {  // 按 size_t i = 0; i < count; ++i 组织循环的初值、继续条件和步进。
        uint32_t position = 0, color = 0;  // 将 0, color = 0 写入或初始化 position（位置）。
        if (!serialUInt(frame.fields[3 + 2 * i], position, 3) || position == 0  // 当 !serialUInt(frame.fields[3 + 2 * i], position, 3) || p… 成立时进入此分支，否则继续后续流程。
                || !serialUInt(frame.fields[4 + 2 * i], color, 6) || color == 0) {  // 继续上一行的组合条件，本行提供 || !serialUInt(frame.fields[4 + 2 * i], color, 6) || color == 0) {。
            serialError(frame, "RANGE"); return;  // 调用 serialError（serial错误），传入 frame, "RANGE"。然后结束当前函数，将执行权交回调用者。
        }  // 结束此函数或控制块。
        for (size_t j = 0; j < i; ++j) {  // 按 size_t j = 0; j < i; ++j 组织循环的初值、继续条件和步进。
            if (candidates[j].position == position) {  // 当 candidates[j].position == position 成立时进入此分支，否则继续后续流程。
                serialError(frame, "DUPLICATE_POSITION"); return;  // 调用 serialError（serial错误），传入 frame, "DUPLICATE_POSITION"。然后结束当前函数，将执行权交回调用者。
            }  // 结束此条件分支。
        }  // 结束此循环体。
        candidates[i] = {static_cast<uint8_t>(position), static_cast<uint8_t>(color)};  // 将 {static_cast<uint8_t>(position), static_cast<uint8_t>(… 写入或初始化 candidates[i]。
    }  // 结束此循环体。
    const RobotState state = currentState;  // 将 currentState 写入或初始化 state（状态）。
    // 自动抓放期间已经结束了旧识别会话。优先返回 NOT_READY，明确表示
    // 这是一帧跨越抓取边界的旧位置数据，而不是普通的“功能未开启”。
    if (!serialDebugMode  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
            && (state == STATE_GRAB_ROUND1 || state == STATE_GRAB_ROUND2)  // 继续上一行的组合条件，本行提供 && (state == STATE_GRAB_ROUND1 || state == STATE_GRAB_ROUND2)。
            && !firstDiscGrabReady) {  // 继续上一行的组合条件，本行提供 && !firstDiscGrabReady) {。
        serialError(frame, "NOT_READY"); return;  // 调用 serialError（serial错误），传入 frame, "NOT_READY"。然后结束当前函数，将执行权交回调用者。
    }  // 结束此函数或控制块。
    if (!discMaterialActive) { serialError(frame, "NOT_ACTIVE"); return; }  // 当 !discMaterialActive 成立时进入此分支，否则继续后续流程。
    const int progress = roundProgress;  // 将 roundProgress 写入或初始化 progress。
    const int *colors = state == STATE_GRAB_ROUND2 ? currentTask.round2_colors  // 将 state == STATE_GRAB_ROUND2 ? currentTask.round2_colors 写入或初始化 colors。
        : state == STATE_GRAB_ROUND1 || serialDebugMode ? currentTask.round1_colors : nullptr;  // 续写上一行表达式的条件取值分支。
    if (!currentTask.valid) { serialError(frame, "NO_TASK"); return; }  // 当 !currentTask.valid 成立时进入此分支，否则继续后续流程。
    if (!colors) { serialError(frame, "NOT_WAITING"); return; }  // 当 !colors 成立时进入此分支，否则继续后续流程。
    if (progress < 0 || progress >= 3) { serialError(frame, "ROUND_COMPLETE"); return; }  // 当 progress < 0 || progress >= 3 成立时进入此分支，否则继续后续流程。
    if (alignmentEnabled) { serialError(frame, "ALIGN_ACTIVE"); return; }  // 当 alignmentEnabled 成立时进入此分支，否则继续后续流程。
    const uint8_t cargo = CargoForGrabIndex(progress);  // 将 CargoForGrabIndex(progress) 写入或初始化 cargo（货位）。
    if (cargoPlatforms[cargo - 1].material != MATERIAL_NONE) {  // 当 cargoPlatforms[cargo - 1].material != MATERIAL_NONE 成立时进入此分支，否则继续后续流程。
        serialError(frame, "OCCUPIED"); return;  // 调用 serialError（serial错误），传入 frame, "OCCUPIED"。然后结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    traceVisionFirstFrame(VisionStartMode::DISC_MATERIAL);  // 调用 traceVisionFirstFrame（trace视觉首个帧），传入 VisionStartMode::DISC_MATERIAL。
    const int selected = SelectDiscPosition(candidates, count, colors, progress, currentTask.valid);  // 将 SelectDiscPosition(candidates, count, colors, progress… 写入或初始化 selected。
    if(selected<0) {  // 当 selected<0 成立时进入此分支，否则继续后续流程。
        Serial.printf("{RSP,VISION,DISC_POSITIONS,ACK,%u}\n",static_cast<unsigned>(count));  // 调用 Serial.printf，传入 "{RSP,VISION,DISC_POSITIONS,ACK,%u}\n",static_cast<uns…。
        Serial.printf("{EVT,VISION,DISC_POSITIONS,SKIP,%d,%u,COLOR_MISMATCH}\n",colors[progress],cargo);return;  // 通过串口执行 printf，输出内容或参数由本行给出。
    }  // 结束此条件分支。
    const auto candidate=candidates[selected];  // 将 candidates[selected] 写入或初始化 candidate。
    if(!serialQueueDiscGrab(candidate.color,cargo,candidate.position,2,count))serialError(frame,"BUSY");  // 当 !serialQueueDiscGrab(candidate.color,cargo,candidate.p… 成立时进入此分支，否则继续后续流程。
}  // 结束此函数或控制块。

/**
 * @brief 处理载物台到工位的单件放料指令，解析编号后提交TRANSFER作业。
 */
static void serialTransfer(const SerialFrame &frame) {  // 定义函数或成员 的入口 serialTransfer，参数由此处的签名确定。
    if (!serialDebugMode) { serialError(frame, "MODE"); return; }  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
    uint32_t cargoCode = 0, workAreaCode = 0;  // 将 0, workAreaCode = 0 写入或初始化 cargoCode（货位Code）。
    if (!serialUInt(frame.fields[3], cargoCode, MATERIAL_STATION_COUNT) || cargoCode == 0  // 当 !serialUInt(frame.fields[3], cargoCode, MATERIAL_STATI… 成立时进入此分支，否则继续后续流程。
            || !serialUInt(frame.fields[4], workAreaCode, MATERIAL_STATION_COUNT) || workAreaCode == 0) {  // 继续上一行的组合条件，本行提供 || !serialUInt(frame.fields[4], workAreaCode, MATERIAL_STATION_COUNT) || workAreaCode == …。
        serialError(frame, "RANGE"); return;  // 调用 serialError（serial错误），传入 frame, "RANGE"。然后结束当前函数，将执行权交回调用者。
    }  // 结束此函数或控制块。
    MotionRequest r;r.kind=MotionKind::TRANSFER;r.cargo=cargoCode;r.work=workAreaCode;r.debug=true;  // 声明 r，类型和可用范围由所在声明决定。然后将 MotionKind::TRANSFER 写入或初始化 r.kind。然后将 cargoCode 写入或初始化 r.cargo。然后将 workAreaCode 写入或初始化 r.work。然后将 true 写入或初始化 r.debug。
    serialMotion(frame,r);  // 调用 serialMotion，传入 frame,r。
}  // 结束此函数或控制块。
/**
 * @brief 处理工位到载物台的单件取回指令，解析编号后提交RETRIEVE作业。
 */
static void serialRetrieve(const SerialFrame &frame) {  // 定义函数或成员 的入口 serialRetrieve，参数由此处的签名确定。
    if(!serialDebugMode){serialError(frame,"MODE");return;}  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
    uint32_t work=0,cargo=0;  // 将 0,cargo=0 写入或初始化 work（工位）。
    if(!serialUInt(frame.fields[3],work,MATERIAL_STATION_COUNT)||!work  // 当 !serialUInt(frame.fields[3],work,MATERIAL_STATION_COUN… 成立时进入此分支，否则继续后续流程。
        ||!serialUInt(frame.fields[4],cargo,MATERIAL_STATION_COUNT)||!cargo){serialError(frame,"RANGE");return;}  // 继续上一行的组合条件，本行提供 ||!serialUInt(frame.fields[4],cargo,MATERIAL_STATION_COUNT)||!cargo){serialError(frame,"R…。
    if(cargoPlatforms[cargo-1].material!=MATERIAL_NONE){serialError(frame,"OCCUPIED");return;}  // 当 cargoPlatforms[cargo-1].material!=MATERIAL_NONE 成立时进入此分支，否则继续后续流程。
    MotionRequest r;r.kind=MotionKind::RETRIEVE;r.cargo=cargo;r.work=work;r.color=MATERIAL_RED;  // 声明 r，类型和可用范围由所在声明决定。然后将 MotionKind::RETRIEVE 写入或初始化 r.kind。然后将 cargo 写入或初始化 r.cargo。然后将 work 写入或初始化 r.work。然后将 MATERIAL_RED 写入或初始化 r.color。
    serialMotion(frame,r);  // 调用 serialMotion，传入 frame,r。
}  // 结束此函数或控制块。

/**
 * @brief 串口业务总分发入口：按类别和动作处理系统、视觉、底盘、机械臂及参数请求。
 */
// 这里是一帧命令的业务入口。
// 共用前置判断决定命令是否能进入分支，分支再解析自己的字段并选择同步回复或异步提交。
static void serialDispatch(const SerialFrame &frame) {  // 定义函数或成员 的入口 serialDispatch，参数由此处的签名确定。
    if (frame.is("SYS", "PANEL_GET", 0)) {  // 当 frame.is("SYS", "PANEL_GET", 0) 成立时进入此分支，否则继续后续流程。
        const bool buttonHigh=digitalRead(START_BUTTON_PIN)==HIGH;  // 将 digitalRead(START_BUTTON_PIN)==HIGH 写入或初始化 buttonHigh。
        const bool zoneHigh=digitalRead(START_ZONE_SWITCH_PIN)==HIGH;  // 将 digitalRead(START_ZONE_SWITCH_PIN)==HIGH 写入或初始化 zoneHigh。
        const auto zone=zoneHigh?START_ZONE_2:START_ZONE_1;  // 将 zoneHigh?START_ZONE_2:START_ZONE_1 写入或初始化 zone。
        Serial.printf("{RSP,SYS,PANEL_GET,OK,%u,%u,%u}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
            static_cast<unsigned>(buttonHigh),static_cast<unsigned>(zoneHigh),static_cast<unsigned>(zone));  // 声明接口 unsigned>，参数由此处的签名确定。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    if (frame.is("SYS", "BEEP", 0)) {  // 当 frame.is("SYS", "BEEP", 0) 成立时进入此分支，否则继续后续流程。
        LedPwm_PlayCue(3);  // 调用 LedPwm_PlayCue（Led占空比输出PlayCue），传入 3。
        serialReply(frame, "OK");  // 调用 serialReply，传入 frame, "OK"。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    if (frame.is("SYS", "REBOOT", 0)) {  // 当 frame.is("SYS", "REBOOT", 0) 成立时进入此分支，否则继续后续流程。
        serialReply(frame, "ACK");  // 调用 serialReply，传入 frame, "ACK"。
        Serial.flush();  // 调用 Serial.flush。
        ESP.restart();  // 调用 ESP.restart。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    if (frame.starts("BAT", "GET") || frame.starts("BAT", "SET")) {  // 当 frame.starts("BAT", "GET") || frame.starts("BAT", "SET… 成立时进入此分支，否则继续后续流程。
        if (frame.is("BAT", "SET", 1)) {  // 当 frame.is("BAT", "SET", 1) 成立时进入此分支，否则继续后续流程。
            uint32_t alarm = 0;  // 将 0 写入或初始化 alarm。
            if (!serialUInt(frame.fields[3], alarm, UINT32_MAX)) { serialError(frame, "RANGE"); return; }  // 当 !serialUInt(frame.fields[3], alarm, UINT32_MAX) 成立时进入此分支，否则继续后续流程。
            if (!BatterySetAlarmMv(alarm)) { serialError(frame, "STORAGE"); return; }  // 当 !BatterySetAlarmMv(alarm) 成立时进入此分支，否则继续后续流程。
        } else if (!frame.is("BAT", "GET", 0)) {  // 仅在先前条件均不成立时，当 !frame.is("BAT", "GET", 0) 成立时进入此分支，否则继续后续流程。
            serialError(frame, "FORMAT");  // 调用 serialError（serial错误），传入 frame, "FORMAT"。
            return;  // 结束当前函数，将执行权交回调用者。
        }  // 结束此条件分支。
        const auto voltage = BatteryVoltageMv();  // 将 BatteryVoltageMv() 写入或初始化 voltage。
        const auto alarm = BatteryAlarmMv();  // 将 BatteryAlarmMv() 写入或初始化 alarm。
        Serial.printf("{RSP,BAT,%s,OK,%lu,%lu,%u}\n", frame.fields[2],  // 通过串口执行 printf，输出内容或参数由本行给出。
            static_cast<unsigned long>(voltage), static_cast<unsigned long>(alarm), voltage < alarm ? 1u : 0u);  // 声明接口 long>，参数由此处的签名确定。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    const bool motionStop=frame.is("CHASSIS","MOVE",3)&&strcmp(frame.fields[5],"1")==0;  // 将 frame.is("CHASSIS","MOVE",3)&&strcmp(frame.fields[5],"… 写入或初始化 motionStop（motion停止）。
    if(motionStop) {  // 当 motionStop 成立时进入此分支，否则继续后续流程。
        uint32_t direction=0,stop=0;float speed=0;  // 将 0,stop=0 写入或初始化 direction（方向）。然后将 0 写入或初始化 speed（速度）。
        if(!serialUInt(frame.fields[3],direction,5)||!serialFloat(frame.fields[4],speed,0,1000)  // 当 !serialUInt(frame.fields[3],direction,5)||!serialFloat… 成立时进入此分支，否则继续后续流程。
            ||!serialUInt(frame.fields[5],stop,1)){serialError(frame,"RANGE");return;}  // 继续上一行的组合条件，本行提供 ||!serialUInt(frame.fields[5],stop,1)){serialError(frame,"RANGE");return;}。
        CancelMotion();Serial.printf("{RSP,CHASSIS,MOVE,ACK,%u,%.3f,1}\n",static_cast<unsigned>(direction),speed);return;  // 调用 CancelMotion，传入 );Serial.printf("{RSP,CHASSIS,MOVE,ACK,%u,%.3f,1}\n",s…。
    }  // 结束此条件分支。
    if(MotionExecutorActive()&&!frame.is("IMU","GET",0)&&!frame.is("IMU","HEADING",0)  // 当 MotionExecutorActive()&&!frame.is("IMU","GET",0)&&!fra… 成立时进入此分支，否则继续后续流程。
        &&!frame.is("POSE","GET",0)&&!frame.starts("CFG","GET")&&!frame.is("MOTION","STATUS",1)  // 继续上一行的组合条件，本行提供 &&!frame.is("POSE","GET",0)&&!frame.starts("CFG","GET")&&!frame.is("MOTION","STATUS",1)。
        &&!frame.is("MOTION","RESUME_MATERIAL",2)  // 继续上一行的组合条件，本行提供 &&!frame.is("MOTION","RESUME_MATERIAL",2)。
        &&!frame.is("NAV","TRIAL_STATUS",1)&&!frame.is("NAV","TRIAL_STOP",1)&&!frame.is("NAV","STOP",0)  // 继续上一行的组合条件，本行提供 &&!frame.is("NAV","TRIAL_STATUS",1)&&!frame.is("NAV","TRIAL_STOP",1)&&!frame.is("NAV","ST…。
        &&!frame.starts("NAV","ROUTE")&&!frame.is("NAV","START_ZONE",1)  // 继续上一行的组合条件，本行提供 &&!frame.starts("NAV","ROUTE")&&!frame.is("NAV","START_ZONE",1)。
        &&!frame.is("SYS","HELLO",0)&&!frame.is("SYS","HELP",0)&&!frame.is("SYS","START",0)&&!frame.is("SYS","READY",0)  // 继续上一行的组合条件，本行提供 &&!frame.is("SYS","HELLO",0)&&!frame.is("SYS","HELP",0)&&!frame.is("SYS","START",0)&&!fra…。
        &&!frame.is("TASK","SET",1)&&!frame.is("VISION","ALIGN_DATA",3)&&!frame.is("VISION","LIDAR_POSE",0)  // 继续上一行的组合条件，本行提供 &&!frame.is("TASK","SET",1)&&!frame.is("VISION","ALIGN_DATA",3)&&!frame.is("VISION","LIDA…。
        &&!frame.starts("VISION","RING_OFFSET")&&!frame.is("VISION","RING_AXIS_PID_GET",0)  // 继续上一行的组合条件，本行提供 &&!frame.starts("VISION","RING_OFFSET")&&!frame.is("VISION","RING_AXIS_PID_GET",0)。
        &&!frame.is("VISION","COLOR",1)&&!frame.starts("VISION","DISC_POSITIONS")&&!frame.starts("VISION","GRAB")  // 继续上一行的组合条件，本行提供 &&!frame.is("VISION","COLOR",1)&&!frame.starts("VISION","DISC_POSITIONS")&&!frame.starts(…。
        &&!frame.is("VISION","STOP_REQUEST",1)&&!frame.is("VISION","ALIGN_STOP",0)) {  // 继续上一行的组合条件，本行提供 &&!frame.is("VISION","STOP_REQUEST",1)&&!frame.is("VISION","ALIGN_STOP",0)) {。
        serialError(frame,"BUSY");return;  // 调用 serialError（serial错误），传入 frame,"BUSY"。然后结束当前函数，将执行权交回调用者。
    }  // 结束此函数或控制块。
    if(serialTrial(frame))return;  // 当 serialTrial(frame) 成立时进入此分支，否则继续后续流程。
    // 每个分支必须先完整匹配动作及参数数量，再读取参数或调用运动函数。
    // 1. 通用命令与共享业务状态。
    if (frame.is("SYS", "HELLO", 0)) {  // 当 frame.is("SYS", "HELLO", 0) 成立时进入此分支，否则继续后续流程。
        Serial.println("{RSP,SYS,HELLO,OK,2}");  // 调用 Serial.println，传入 "{RSP,SYS,HELLO,OK,2}"。
    } else if(frame.is("EMM","TURRET_TRIAL",5)) {  // 仅在先前条件均不成立时，当 frame.is("EMM","TURRET_TRIAL",5) 成立时进入此分支，否则继续后续流程。
        if(!serialDebugMode){serialError(frame,"MODE");return;}  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
        float angle=0,speed=0;uint32_t ramp=0,period=0,duration=0;  // 将 0,speed=0 写入或初始化 angle（角度）。然后将 0,period=0,duration=0 写入或初始化 ramp。
        if(!serialFloat(frame.fields[3],angle,TURRET_CABLE_MIN_DEG,TURRET_CABLE_MAX_DEG)  // 当 !serialFloat(frame.fields[3],angle,TURRET_CABLE_MIN_DE… 成立时进入此分支，否则继续后续流程。
            ||!serialFloat(frame.fields[4],speed,1,65535.0f*6/TURRET_MOTOR_GEAR_RATIO)  // 继续上一行的组合条件，本行提供 ||!serialFloat(frame.fields[4],speed,1,65535.0f*6/TURRET_MOTOR_GEAR_RATIO)。
            ||!serialUInt(frame.fields[5],ramp,UINT16_MAX)  // 继续上一行的组合条件，本行提供 ||!serialUInt(frame.fields[5],ramp,UINT16_MAX)。
            ||!serialUInt(frame.fields[6],period,UINT32_MAX)||!period  // 继续上一行的组合条件，本行提供 ||!serialUInt(frame.fields[6],period,UINT32_MAX)||!period。
            ||!serialUInt(frame.fields[7],duration,UINT32_MAX)||!duration){serialError(frame,"RANGE");return;}  // 继续上一行的组合条件，本行提供 ||!serialUInt(frame.fields[7],duration,UINT32_MAX)||!duration){serialError(frame,"RANGE")…。
        MotionRequest r;r.kind=MotionKind::TURRET_TRIAL;r.turret=angle;r.speed=speed;  // 声明 r，类型和可用范围由所在声明决定。然后将 MotionKind::TURRET_TRIAL 写入或初始化 r.kind。然后将 angle 写入或初始化 r.turret。然后将 speed 写入或初始化 r.speed。
        r.turretTrialRampMs=ramp;r.turretTrialPeriodMs=period;r.turretTrialDurationMs=duration;  // 将 ramp 写入或初始化 r.turretTrialRampMs。然后将 period 写入或初始化 r.turretTrialPeriodMs。然后将 duration 写入或初始化 r.turretTrialDurationMs。
        serialMotion(frame,r);  // 调用 serialMotion，传入 frame,r。
    } else if(frame.is("EMM","AXIS_TRIAL",6)) {  // 仅在先前条件均不成立时，当 frame.is("EMM","AXIS_TRIAL",6) 成立时进入此分支，否则继续后续流程。
        if(!serialDebugMode){serialError(frame,"MODE");return;}  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
        uint32_t id=0,rpm=0,ramp=0,period=0,duration=0;float position=0;  // 将 0,rpm=0,ramp=0,period=0,duration=0 写入或初始化 id（编号）。然后将 0 写入或初始化 position（位置）。
        if(!serialUInt(frame.fields[3],id,6)||(id!=5&&id!=6)  // 当 !serialUInt(frame.fields[3],id,6)||(id!=5&&id!=6 成立时进入此分支，否则继续后续流程。
            ||!serialFloat(frame.fields[4],position,0,id==5?ARM_HEIGHT_LIMIT_MM:170.0f)  // 继续上一行的组合条件，本行提供 ||!serialFloat(frame.fields[4],position,0,id==5?ARM_HEIGHT_LIMIT_MM:170.0f)。
            ||!serialUInt(frame.fields[5],rpm,UINT16_MAX)||!rpm  // 继续上一行的组合条件，本行提供 ||!serialUInt(frame.fields[5],rpm,UINT16_MAX)||!rpm。
            ||!serialUInt(frame.fields[6],ramp,UINT16_MAX)  // 继续上一行的组合条件，本行提供 ||!serialUInt(frame.fields[6],ramp,UINT16_MAX)。
            ||!serialUInt(frame.fields[7],period,UINT32_MAX)||!period  // 继续上一行的组合条件，本行提供 ||!serialUInt(frame.fields[7],period,UINT32_MAX)||!period。
            ||!serialUInt(frame.fields[8],duration,UINT32_MAX)||!duration){serialError(frame,"RANGE");return;}  // 继续上一行的组合条件，本行提供 ||!serialUInt(frame.fields[8],duration,UINT32_MAX)||!duration){serialError(frame,"RANGE")…。
        MotionRequest r;r.kind=MotionKind::AXIS_TRIAL;r.axisTrialMotor=id;  // 声明 r，类型和可用范围由所在声明决定。然后将 MotionKind::AXIS_TRIAL 写入或初始化 r.kind。然后将 id 写入或初始化 r.axisTrialMotor。
        if(id==5)r.high=position;else r.length=position;  // 当 id==5 成立时进入此分支，否则继续后续流程。
        r.axisTrialRpm=rpm;r.axisTrialRampMs=ramp;r.axisTrialPeriodMs=period;r.axisTrialDurationMs=duration;  // 将 rpm 写入或初始化 r.axisTrialRpm。然后将 ramp 写入或初始化 r.axisTrialRampMs。然后将 period 写入或初始化 r.axisTrialPeriodMs。然后将 duration 写入或初始化 r.axisTrialDurationMs。
        serialMotion(frame,r);  // 调用 serialMotion，传入 frame,r。
    } else if(frame.is("EMM","DRIVE_GET",1)||frame.is("EMM","DRIVE_SET",9)) {  // 仅在先前条件均不成立时，当 frame.is("EMM","DRIVE_GET",1)||frame.is("EMM","DRIVE_S… 成立时进入此分支，否则继续后续流程。
        if(!serialDebugMode){serialError(frame,"MODE");return;}  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
        uint32_t id=0,save=0;  // 将 0,save=0 写入或初始化 id（编号）。
        if(!serialUInt(frame.fields[3],id,7)||!id){serialError(frame,"RANGE");return;}  // 当 !serialUInt(frame.fields[3],id,7)||!id 成立时进入此分支，否则继续后续流程。
        uint16_t values[7]={};const char *error=nullptr;  // 将 {};const char *error=nullptr 写入或初始化 values[7]。
        if(frame.is("EMM","DRIVE_SET",9)) {  // 当 frame.is("EMM","DRIVE_SET",9) 成立时进入此分支，否则继续后续流程。
            for(uint8_t i=0;i<7;++i){uint32_t value=0;  // 按 uint8_t i=0;i<7;++i 组织循环的初值、继续条件和步进。
                if(!serialUInt(frame.fields[4+i],value,UINT16_MAX)){serialError(frame,"RANGE");return;}  // 当 !serialUInt(frame.fields[4+i],value,UINT16_MAX) 成立时进入此分支，否则继续后续流程。
                values[i]=value;  // 将 value 写入或初始化 values[i]。
            }  // 结束此循环体。
            if(!serialUInt(frame.fields[11],save,1)){serialError(frame,"RANGE");return;}  // 当 !serialUInt(frame.fields[11],save,1) 成立时进入此分支，否则继续后续流程。
            error=SetMotorDriveParamsVerified(id,values,save);  // 将 SetMotorDriveParamsVerified(id,values,save) 写入或初始化 error（错误）。
        } else error=ReadMotorDriveParams(id,values);  // 前一个判断不成立时执行此分支。
        if(error&&strcmp(error,"DRIVE_VERIFY_MISMATCH")!=0){serialError(frame,error);return;}  // 当 error&&strcmp(error,"DRIVE_VERIFY_MISMATCH")!=0 成立时进入此分支，否则继续后续流程。
        Serial.printf("{RSP,EMM,%s,%s,%u,%u,%u,%u,%u,%u,%u,%u}\n",frame.fields[2],error?"READBACK_MISMATCH":"READBACK",unsigned(id),  // 通过串口执行 printf，输出内容或参数由本行给出。
            values[0],values[1],values[2],values[3],values[4],values[5],values[6]);  // 续写当前声明或运算，提供 values[0],values[1],values[2],values[3],values[4],valu…。
    } else if(frame.is("EMM","PID_GET",1)||frame.is("EMM","PID_SET",5)  // 仅在先前条件均不成立时，当 frame.is("EMM","PID_GET",1)||frame.is("EMM","PID_SET",5 成立时进入此分支，否则继续后续流程。
            ||frame.is("EMM","STATE_GET",1)||frame.is("EMM","MODE_SET",3)  // 继续上一行的组合条件，本行提供 ||frame.is("EMM","STATE_GET",1)||frame.is("EMM","MODE_SET",3)。
            ||frame.is("EMM","ENABLE",2)||frame.is("EMM","CLEAR_STALL",1)) {  // 继续上一行的组合条件，本行提供 ||frame.is("EMM","ENABLE",2)||frame.is("EMM","CLEAR_STALL",1)) {。
        if(!serialDebugMode){serialError(frame,"MODE");return;}  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
        uint32_t id=0;  // 将 0 写入或初始化 id（编号）。
        if(!serialUInt(frame.fields[3],id,7)||!id){serialError(frame,"RANGE");return;}  // 当 !serialUInt(frame.fields[3],id,7)||!id 成立时进入此分支，否则继续后续流程。
        MotorBusGuard transaction;  // 声明 transaction，类型和可用范围由所在声明决定。
        const char *error=nullptr;  // 将 nullptr 写入或初始化 error（错误）。
        if(frame.is("EMM","PID_GET",1)||frame.is("EMM","PID_SET",5)) {  // 当 frame.is("EMM","PID_GET",1)||frame.is("EMM","PID_SET",… 成立时进入此分支，否则继续后续流程。
            uint32_t gains[3]={},save=0;  // 将 {},save=0 写入或初始化 gains[3]。
            if(frame.is("EMM","PID_SET",5)) {  // 当 frame.is("EMM","PID_SET",5) 成立时进入此分支，否则继续后续流程。
                for(uint8_t i=0;i<3;++i)if(!serialUInt(frame.fields[4+i],gains[i],UINT32_MAX)){serialError(frame,"RANGE");return;}  // 按 uint8_t i=0;i<3;++i)if(!serialUInt(frame.fields[4+i],g… 组织循环的初值、继续条件和步进。
                if(!serialUInt(frame.fields[7],save,1)){serialError(frame,"RANGE");return;}  // 当 !serialUInt(frame.fields[7],save,1) 成立时进入此分支，否则继续后续流程。
                error=SetMotorPositionPidVerified(id,gains,save);  // 将 SetMotorPositionPidVerified(id,gains,save) 写入或初始化 error（错误）。
                if(error){  // 当 error 成立时进入此分支，否则继续后续流程。
                    Serial.printf("[EMM] id=%u PID_SET stage=WRITE_VERIFY error=%s\n",unsigned(id),error);  // 调用 Serial.printf，传入 "[EMM] id=%u PID_SET stage=WRITE_VERIFY error=%s\n",un…。
                    serialError(frame,error);return;  // 调用 serialError（serial错误），传入 frame,error。然后结束当前函数，将执行权交回调用者。
                }  // 结束此条件分支。
            }  // 结束此条件分支。
            else error=ReadMotorPositionPid(id,gains);  // 前一个判断不成立时执行此分支。
            if(error){  // 当 error 成立时进入此分支，否则继续后续流程。
                Serial.printf("[EMM] id=%u %s stage=READBACK error=%s\n",unsigned(id),frame.fields[2],error);  // 调用 Serial.printf，传入 "[EMM] id=%u %s stage=READBACK error=%s\n",unsigned(id…。
                serialError(frame,error);return;  // 调用 serialError（serial错误），传入 frame,error。然后结束当前函数，将执行权交回调用者。
            }  // 结束此条件分支。
            Serial.printf("{RSP,EMM,%s,READBACK,%u,%lu,%lu,%lu}\n",frame.fields[2],unsigned(id),  // 通过串口执行 printf，输出内容或参数由本行给出。
                (unsigned long)gains[0],(unsigned long)gains[1],(unsigned long)gains[2]);  // 为当前列表或跨行表达式提供这一项 (unsigned long)gains[0],(unsigned long)gains[1],(unsig…。
        } else if(frame.is("EMM","STATE_GET",1)) {  // 仅在先前条件均不成立时，当 frame.is("EMM","STATE_GET",1) 成立时进入此分支，否则继续后续流程。
            uint16_t encoder=0;int64_t position=0;int16_t rpm=0;uint8_t flags=0;  // 将 0 写入或初始化 encoder（编码器）。然后将 0 写入或初始化 position（位置）。然后将 0 写入或初始化 rpm（转速）。然后将 0 写入或初始化 flags（标志位）。
            error=ReadMotorEncoderSnapshot(id,encoder,position,rpm,flags);  // 将 ReadMotorEncoderSnapshot(id,encoder,position,rpm,flags) 写入或初始化 error（错误）。
            if(error){serialError(frame,error);return;}  // 当 error 成立时进入此分支，否则继续后续流程。
            Serial.printf("{RSP,EMM,STATE_GET,READBACK,%u,%u,%lld,%d,%u}\n",unsigned(id),encoder,(long long)position,rpm,flags);  // 调用 Serial.printf，传入 "{RSP,EMM,STATE_GET,READBACK,%u,%u,%lld,%d,%u}\n",unsi…。
        } else {  // 前一个判断不成立时执行此分支。
            uint32_t value=0,save=0;uint8_t function=0x0E;  // 将 0,save=0 写入或初始化 value（测值）。然后将 0x0E 写入或初始化 function。
            if(frame.is("EMM","MODE_SET",3)) {  // 当 frame.is("EMM","MODE_SET",3) 成立时进入此分支，否则继续后续流程。
                if(!serialUInt(frame.fields[4],value,3)||!serialUInt(frame.fields[5],save,1)){serialError(frame,"RANGE");return;}  // 当 !serialUInt(frame.fields[4],value,3)||!serialUInt(fram… 成立时进入此分支，否则继续后续流程。
                function=0x46;  // 将 0x46 写入或初始化 function。
            } else if(frame.is("EMM","ENABLE",2)) {  // 仅在先前条件均不成立时，当 frame.is("EMM","ENABLE",2) 成立时进入此分支，否则继续后续流程。
                if(!serialUInt(frame.fields[4],value,1)){serialError(frame,"RANGE");return;}  // 当 !serialUInt(frame.fields[4],value,1) 成立时进入此分支，否则继续后续流程。
                function=0xF3;  // 将 0xF3 写入或初始化 function。
            }  // 结束此条件分支。
            error=ConfigureEmmMotor(id,function,value,save);  // 将 ConfigureEmmMotor(id,function,value,save) 写入或初始化 error（错误）。
            if(error){serialError(frame,error);return;}  // 当 error 成立时进入此分支，否则继续后续流程。
            Serial.printf("{RSP,EMM,%s,ACK,%u}\n",frame.fields[2],unsigned(id));  // 调用 Serial.printf，传入 "{RSP,EMM,%s,ACK,%u}\n",frame.fields[2],unsigned(id)。
        }  // 结束此条件分支。
    } else if(frame.is("MOTION","STATUS",1)) {  // 仅在先前条件均不成立时，当 frame.is("MOTION","STATUS",1) 成立时进入此分支，否则继续后续流程。
        uint32_t id=0;if(!serialUInt(frame.fields[3],id,UINT32_MAX)){serialError(frame,"RANGE");return;}  // 将 0;if(!serialUInt(frame.fields[3],id,UINT32_MAX)){seria… 写入或初始化 id（编号）。
        const auto result=GetMotionStatus(id);  // 将 GetMotionStatus(id) 写入或初始化 result（结果）。
        if(!id||!result.id){serialError(frame,"ID");return;}  // 当 !id||!result.id 成立时进入此分支，否则继续后续流程。
        Serial.printf("{RSP,MOTION,STATUS,OK,%lu,%s,%s,%lu}\n",static_cast<unsigned long>(id),  // 通过串口执行 printf，输出内容或参数由本行给出。
            MotionStateName(result.state),result.phase,static_cast<unsigned long>(result.elapsedMs));  // 调用 MotionStateName（Motion状态Name），传入 result.state),result.phase,static_cast<unsigned long>(…。
    // 接续与状态查询共用运动作业编号，但接续还要求令牌。
    // 令牌由当前等待阶段给出，用于区分同一作业内不同接续窗口。
    } else if(frame.is("MOTION","RESUME_MATERIAL",2)) {  // 仅在先前条件均不成立时，当 frame.is("MOTION","RESUME_MATERIAL",2) 成立时进入此分支，否则继续后续流程。
        uint32_t id=0,token=0;  // 将 0,token=0 写入或初始化 id（编号）。
        if(!serialUInt(frame.fields[3],id,UINT32_MAX)||!id||!serialUInt(frame.fields[4],token,UINT32_MAX)) {  // 当 !serialUInt(frame.fields[3],id,UINT32_MAX)||!id||!seri… 成立时进入此分支，否则继续后续流程。
            serialError(frame,"RANGE");return;  // 调用 serialError（serial错误），传入 frame,"RANGE"。然后结束当前函数，将执行权交回调用者。
        }  // 结束此条件分支。
        if(!ResumeMaterialMotion(id,token)){serialError(frame,"NOT_WAITING");return;}  // 当 !ResumeMaterialMotion(id,token) 成立时进入此分支，否则继续后续流程。
        Serial.printf("{RSP,MOTION,RESUME_MATERIAL,ACK,%lu,%lu}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
            static_cast<unsigned long>(id),static_cast<unsigned long>(token));  // 声明接口 long>，参数由此处的签名确定。
    } else if(frame.is("NAV","STOP",0)) {  // 仅在先前条件均不成立时，当 frame.is("NAV","STOP",0) 成立时进入此分支，否则继续后续流程。
        CancelMotion();  // 调用 CancelMotion，使用无参形式推进相应操作。
        Serial.println("{RSP,NAV,STOP,ACK}");  // 调用 Serial.println，传入 "{RSP,NAV,STOP,ACK}"。
    } else if (frame.is("TASK", "SET", 1)) {  // 仅在先前条件均不成立时，当 frame.is("TASK", "SET", 1) 成立时进入此分支，否则继续后续流程。
        const TaskCode task = parseTaskCode(frame.fields[3]);  // 将 parseTaskCode(frame.fields[3]) 写入或初始化 task（任务）。
        if (!task.valid) { serialError(frame, "RANGE"); return; }  // 当 !task.valid 成立时进入此分支，否则继续后续流程。
        // Release 只允许开局预置或扫码等待期间输入，防止覆盖正在执行的任务。
        portENTER_CRITICAL(&taskCodeMux);  // 进入临界区，保护共享状态的本段访问。
        const bool accepting = serialDebugMode || currentState == STATE_WAIT_START  // 将 serialDebugMode || currentState == STATE_WAIT_START 写入或初始化 accepting。
            || (currentState == STATE_READ_TASK && !taskReceived);  // 继续上一行的组合条件，本行提供 || (currentState == STATE_READ_TASK && !taskReceived);。
        if (accepting) {  // 当 accepting 成立时进入此分支，否则继续后续流程。
            currentTask = task;  // 将 task 写入或初始化 currentTask（当前任务）。
            taskReceived = true;  // 将 true 写入或初始化 taskReceived（任务已接收）。
            roundProgress = 0;  // 将 0 写入或初始化 roundProgress（轮次Progress）。
            discMaterialColor = 0;  // 将 0 写入或初始化 discMaterialColor（圆盘物料颜色）。
        }  // 结束此条件分支。
        portEXIT_CRITICAL(&taskCodeMux);  // 离开临界区，保护共享状态的本段访问。
        if (!accepting) { serialError(frame, "BUSY"); return; }  // 当 !accepting 成立时进入此分支，否则继续后续流程。
        Serial.printf("{RSP,TASK,SET,OK,%s}\n", frame.fields[3]);  // 调用 Serial.printf，传入 "{RSP,TASK,SET,OK,%s}\n", frame.fields[3]。
        updateDisplay("DISPLAY", "TASK_CODE", frame.fields[3]);  // 调用 updateDisplay，传入 "DISPLAY", "TASK_CODE", frame.fields[3]。
    } else if (frame.is("SYS", "RELEASE", 0)) {  // 仅在先前条件均不成立时，当 frame.is("SYS", "RELEASE", 0) 成立时进入此分支，否则继续后续流程。
        serialRelease(frame);  // 调用 serialRelease，传入 frame。
    } else if (frame.is("SYS", "READY", 0)) {  // 仅在先前条件均不成立时，当 frame.is("SYS", "READY", 0) 成立时进入此分支，否则继续后续流程。
        if (serialDebugMode) { serialError(frame, "MODE"); return; }  // 当 serialDebugMode 成立时进入此分支，否则继续后续流程。
        nano_ready = true;  // 将 true 写入或初始化 nano_ready（nano就绪）。
        serialReply(frame, "OK");  // 调用 serialReply，传入 frame, "OK"。
    } else if (frame.is("SYS", "START", 0)) {  // 仅在先前条件均不成立时，当 frame.is("SYS", "START", 0) 成立时进入此分支，否则继续后续流程。
        if (serialDebugMode) { serialError(frame, "MODE"); return; }  // 当 serialDebugMode 成立时进入此分支，否则继续后续流程。
        enableRun = true;  // 将 true 写入或初始化 enableRun。
        serialReply(frame, "OK");  // 调用 serialReply，传入 frame, "OK"。
    } else if (frame.is("SYS", "HELP", 0)) {  // 仅在先前条件均不成立时，当 frame.is("SYS", "HELP", 0) 成立时进入此分支，否则继续后续流程。
        Serial.println("[Help] See Document/serial_protocol.md, protocol v2");  // 调用 Serial.println，传入 "[Help] See Document/serial_protocol.md, protocol v2"。
        serialReply(frame, "OK");  // 调用 serialReply，传入 frame, "OK"。
    } else if (frame.is("POSE", "GET", 0)) {  // 仅在先前条件均不成立时，当 frame.is("POSE", "GET", 0) 成立时进入此分支，否则继续后续流程。
        sendPoseQueryReply();  // 调用 sendPoseQueryReply，使用无参形式推进相应操作。
    } else if (frame.is("IMU", "GET", 0)) {  // 仅在先前条件均不成立时，当 frame.is("IMU", "GET", 0) 成立时进入此分支，否则继续后续流程。
        H30Attitude sample;  // 声明 sample（采样），类型和可用范围由所在声明决定。
        uint32_t age = 0;  // 将 0 写入或初始化 age（样本年龄）。
        const char *error = H30UsbGetAttitude(sample, age);  // 将 H30UsbGetAttitude(sample, age) 写入或初始化 error（错误）。
        if (error) { serialError(frame, error); return; }  // 当 error 成立时进入此分支，否则继续后续流程。
        Serial.printf("{RSP,IMU,GET,OK,%.6f,%.6f,%.6f,%lu}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                      sample.pitch, sample.roll, sample.yaw, static_cast<unsigned long>(age));  // 声明接口 long>，参数由此处的签名确定。
    } else if (frame.is("IMU", "HEADING", 0)) {  // 仅在先前条件均不成立时，当 frame.is("IMU", "HEADING", 0) 成立时进入此分支，否则继续后续流程。
        float heading = 0;  // 将 0 写入或初始化 heading（航向）。
        uint32_t age = 0;  // 将 0 写入或初始化 age（样本年龄）。
        const char *error = H30UsbGetHeading(heading, age);  // 将 H30UsbGetHeading(heading, age) 写入或初始化 error（错误）。
        if (error) { serialError(frame, error); return; }  // 当 error 成立时进入此分支，否则继续后续流程。
        Serial.printf("{RSP,IMU,HEADING,OK,%.6f,%lu}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                      heading, static_cast<unsigned long>(age));  // 声明接口 long>，参数由此处的签名确定。
    } else if (frame.is("LED", "SET", 1) || frame.is("LED", "GET", 0)  // 仅在先前条件均不成立时，当 frame.is("LED", "SET", 1) || frame.is("LED", "GET", 0 成立时进入此分支，否则继续后续流程。
               || frame.is("LED", "FREQ", 1)) {  // 继续上一行的组合条件，本行提供 || frame.is("LED", "FREQ", 1)) {。
        uint32_t percent = 0;  // 将 0 写入或初始化 percent。
        uint32_t hz = 0;  // 将 0 写入或初始化 hz。
        const bool frequency = strcmp(frame.fields[2], "FREQ") == 0;  // 将 strcmp(frame.fields[2], "FREQ") == 0 写入或初始化 frequency。
        const bool setting = strcmp(frame.fields[2], "SET") == 0;  // 将 strcmp(frame.fields[2], "SET") == 0 写入或初始化 setting。
        if (setting && !serialUInt(frame.fields[3], percent, 100)) {  // 当 setting && !serialUInt(frame.fields[3], percent, 100) 成立时进入此分支，否则继续后续流程。
            serialError(frame, "RANGE"); return;  // 调用 serialError（serial错误），传入 frame, "RANGE"。然后结束当前函数，将执行权交回调用者。
        }  // 结束此条件分支。
        if (frequency && (!serialUInt(frame.fields[3], hz, LED_PWM_MAX_FREQUENCY_HZ)  // 当 frequency && (!serialUInt(frame.fields[3], hz, LED_PWM… 成立时进入此分支，否则继续后续流程。
                          || hz < LED_PWM_MIN_FREQUENCY_HZ)) {  // 继续上一行的组合条件，本行提供 || hz < LED_PWM_MIN_FREQUENCY_HZ)) {。
            serialError(frame, "RANGE"); return;  // 调用 serialError（serial错误），传入 frame, "RANGE"。然后结束当前函数，将执行权交回调用者。
        }  // 结束此函数或控制块。
        if (!LedPwm_IsReady() || (setting && !LedPwm_SetBrightness(percent))  // 当 !LedPwm_IsReady() || (setting && !LedPwm_SetBrightness… 成立时进入此分支，否则继续后续流程。
                || (frequency && !LedPwm_SetFrequency(hz))) {  // 继续上一行的组合条件，本行提供 || (frequency && !LedPwm_SetFrequency(hz))) {。
            serialError(frame, "PWM"); return;  // 调用 serialError（serial错误），传入 frame, "PWM"。然后结束当前函数，将执行权交回调用者。
        }  // 结束此函数或控制块。
        Serial.printf("{RSP,LED,%s,OK,%u,%lu}\n", frame.fields[2],  // 通过串口执行 printf，输出内容或参数由本行给出。
                      static_cast<unsigned>(LedPwm_GetBrightness()),  // 定义函数或成员 unsigned>，参数由此处的签名确定。
                      static_cast<unsigned long>(LedPwm_GetFrequency()));  // 声明接口 long>，参数由此处的签名确定。
    } else if (frame.is("NAV", "START_ZONE", 1)) {  // 仅在先前条件均不成立时，当 frame.is("NAV", "START_ZONE", 1) 成立时进入此分支，否则继续后续流程。
        uint32_t zone = 0;  // 将 0 写入或初始化 zone。
        if (!serialUInt(frame.fields[3], zone, 2) || zone < 1) {  // 当 !serialUInt(frame.fields[3], zone, 2) || zone < 1 成立时进入此分支，否则继续后续流程。
            serialError(frame, "RANGE"); return;  // 调用 serialError（serial错误），传入 frame, "RANGE"。然后结束当前函数，将执行权交回调用者。
        }  // 结束此条件分支。
        currentStartZone = static_cast<StartZone>(zone);  // 将 static_cast<StartZone>(zone) 写入或初始化 currentStartZone（当前起点Zone）。
        if (serialDebugMode) {  // 当 serialDebugMode 成立时进入此分支，否则继续后续流程。
            currentPose = {zone == 1 ? 2250.0f : 150.0f, 150, StartZoneHeading(zone)};  // 将 {zone == 1 ? 2250.0f : 150.0f, 150, StartZoneHeading(z… 写入或初始化 currentPose（当前Pose）。
            // 只更新地图初始位姿，不覆盖初始化后的3秒平均航向参考。
        }  // 结束此条件分支。
        Serial.printf("{RSP,NAV,START_ZONE,OK,%u}\n", static_cast<unsigned>(zone));  // 调用 Serial.printf，传入 "{RSP,NAV,START_ZONE,OK,%u}\n", static_cast<unsigned>(…。
    } else if (frame.starts("NAV", "ROUTE")) {  // 仅在先前条件均不成立时，当 frame.starts("NAV", "ROUTE") 成立时进入此分支，否则继续后续流程。
        serialRoute(frame);  // 调用 serialRoute（serial路线），传入 frame。
    } else if (frame.is("VISION", "RING_OFFSET", 3)) {  // 仅在先前条件均不成立时，当 frame.is("VISION", "RING_OFFSET", 3) 成立时进入此分支，否则继续后续流程。
        uint32_t token=0;  // 将 0 写入或初始化 token。
        ring::Frame sample;  // 声明 sample（采样），类型和可用范围由所在声明决定。
        if(!serialUInt(frame.fields[3],token,UINT32_MAX)  // 当 !serialUInt(frame.fields[3],token,UINT32_MAX 成立时进入此分支，否则继续后续流程。
            ||!serialFloat(frame.fields[4],sample.x,-FLT_MAX,FLT_MAX)  // 继续上一行的组合条件，本行提供 ||!serialFloat(frame.fields[4],sample.x,-FLT_MAX,FLT_MAX)。
            ||!serialFloat(frame.fields[5],sample.y,-FLT_MAX,FLT_MAX)) {  // 继续上一行的组合条件，本行提供 ||!serialFloat(frame.fields[5],sample.y,-FLT_MAX,FLT_MAX)) {。
            serialError(frame,"RANGE");return;  // 调用 serialError（serial错误），传入 frame,"RANGE"。然后结束当前函数，将执行权交回调用者。
        }  // 结束此函数或控制块。
        sample.timeMs=millis();  // 将 millis() 写入或初始化 sample.timeMs。
        const bool accepted=ring::input().submit(token,sample);  // 将 ring::input().submit(token,sample) 写入或初始化 accepted。
        Serial.printf("{EVT,VISION,RING_OFFSET,%s,%s,%s,%s}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
            frame.fields[3],frame.fields[4],frame.fields[5],accepted?"ACCEPTED":"IGNORED");  // 续写当前声明或运算，提供 frame.fields[3],frame.fields[4],frame.fields[5],accept…。
        if(accepted)WakeMotionExecutor();  // 当 accepted 成立时进入此分支，否则继续后续流程。
    } else if (frame.is("VISION", "RING_AXIS_PID_GET", 0) || frame.is("VISION", "RING_AXIS_PID_SET", 6)) {  // 仅在先前条件均不成立时，当 frame.is("VISION", "RING_AXIS_PID_GET", 0) || frame.is… 成立时进入此分支，否则继续后续流程。
        if(frame.count>3) {  // 当 frame.count>3 成立时进入此分支，否则继续后续流程。
            if(!serialDebugMode){serialError(frame,"MODE");return;}  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
            if(MotionExecutorActive()){serialError(frame,"BUSY");return;}  // 当 MotionExecutorActive() 成立时进入此分支，否则继续后续流程。
            const float low[6]={};  // 将 {} 写入或初始化 low[6]。
            const float high[6]={FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX};  // 将 {FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX} 写入或初始化 high[6]。
            float gain[6];  // 声明 gain（增益），类型和可用范围由所在声明决定。
            if(!serialFloats(frame,gain,low,high,6)) {  // 当 !serialFloats(frame,gain,low,high,6) 成立时进入此分支，否则继续后续流程。
                serialError(frame,"RANGE");return;  // 调用 serialError（serial错误），传入 frame,"RANGE"。然后结束当前函数，将执行权交回调用者。
            }  // 结束此条件分支。
            ring::Settings cfg;  // 声明 cfg（配置），类型和可用范围由所在声明决定。
            cfg.turret={gain[0],gain[1],gain[2]};  // 将 {gain[0],gain[1],gain[2]} 写入或初始化 cfg.turret。
            cfg.extension={gain[3],gain[4],gain[5]};  // 将 {gain[3],gain[4],gain[5]} 写入或初始化 cfg.extension。
            ring::input().settings(cfg);  // 调用 ring::input（圆环输入），传入 ).settings(cfg。
        }  // 结束此条件分支。
        const auto cfg=ring::input().settings();  // 将 ring::input().settings() 写入或初始化 cfg（配置）。
        Serial.printf("{RSP,VISION,%s,OK,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
            frame.fields[2],cfg.turret.kp,cfg.turret.ki,cfg.turret.kd,  // 为当前列表或跨行表达式提供这一项 frame.fields[2],cfg.turret.kp,cfg.turret.ki,cfg.turret…。
            cfg.extension.kp,cfg.extension.ki,cfg.extension.kd);  // 续写当前声明或运算，提供 cfg.extension.kp,cfg.extension.ki,cfg.extension.kd);。
    } else if ((frame.starts("VISION","START_REQUEST")||frame.starts("VISION","STOP_REQUEST"))  // 仅在先前条件均不成立时，当 (frame.starts("VISION","START_REQUEST")||frame.starts(… 成立时进入此分支，否则继续后续流程。
               && frame.count>=4 && strcmp(frame.fields[3],"RING_CORRECTION")==0) {  // 继续上一行的组合条件，本行提供 && frame.count>=4 && strcmp(frame.fields[3],"RING_CORRECTION")==0) {。
        if(!serialDebugMode){serialError(frame,"MODE");return;}  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
        if(MotionExecutorActive()){serialError(frame,"BUSY");return;}  // 当 MotionExecutorActive() 成立时进入此分支，否则继续后续流程。
        if(frame.is("VISION","START_REQUEST",2)) {  // 当 frame.is("VISION","START_REQUEST",2) 成立时进入此分支，否则继续后续流程。
            uint32_t work=0;  // 将 0 写入或初始化 work（工位）。
            if(!serialUInt(frame.fields[4],work,3)||work==0){serialError(frame,"RANGE");return;}  // 当 !serialUInt(frame.fields[4],work,3)||work==0 成立时进入此分支，否则继续后续流程。
            Serial.printf("{RSP,VISION,START_REQUEST,ACK,RING_CORRECTION,%lu}\n",static_cast<unsigned long>(work));  // 调用 Serial.printf，传入 "{RSP,VISION,START_REQUEST,ACK,RING_CORRECTION,%lu}\n"…。
            Serial.printf("{EVT,VISION,RING_TARGET,%lu}\n",static_cast<unsigned long>(work));  // 调用 Serial.printf，传入 "{EVT,VISION,RING_TARGET,%lu}\n",static_cast<unsigned …。
            Serial.println("{EVT,VISION,START_REQUEST,RING_CORRECTION}");  // 调用 Serial.println，传入 "{EVT,VISION,START_REQUEST,RING_CORRECTION}"。
        } else if(frame.is("VISION","STOP_REQUEST",1)) {  // 仅在先前条件均不成立时，当 frame.is("VISION","STOP_REQUEST",1) 成立时进入此分支，否则继续后续流程。
            Serial.println("{RSP,VISION,STOP_REQUEST,ACK,RING_CORRECTION}");  // 调用 Serial.println，传入 "{RSP,VISION,STOP_REQUEST,ACK,RING_CORRECTION}"。
            Serial.println("{EVT,VISION,STOP_REQUEST,RING_CORRECTION}");  // 调用 Serial.println，传入 "{EVT,VISION,STOP_REQUEST,RING_CORRECTION}"。
        } else serialError(frame,"FORMAT");  // 前一个判断不成立时执行此分支。
    } else if (frame.is("VISION", "START_REQUEST", 1)  // 仅在先前条件均不成立时，当 frame.is("VISION", "START_REQUEST", 1 成立时进入此分支，否则继续后续流程。
               || frame.is("VISION", "STOP_REQUEST", 1)) {  // 继续上一行的组合条件，本行提供 || frame.is("VISION", "STOP_REQUEST", 1)) {。
        VisionStartMode mode;  // 声明 mode（模式），类型和可用范围由所在声明决定。
        const char *name = frame.fields[3];  // 将 frame.fields[3] 写入或初始化 name。
        if (strcmp(name, "DISC") == 0) mode = VisionStartMode::DISC;  // 当 strcmp(name, "DISC") == 0 成立时进入此分支，否则继续后续流程。
        else if (strcmp(name, "DISC_MATERIAL") == 0) mode = VisionStartMode::DISC_MATERIAL;  // 仅在先前条件均不成立时，当 strcmp(name, "DISC_MATERIAL") == 0 成立时进入此分支，否则继续后续流程。
        else if (strcmp(name, "WORK_AREA") == 0) mode = VisionStartMode::WORK_AREA;  // 仅在先前条件均不成立时，当 strcmp(name, "WORK_AREA") == 0 成立时进入此分支，否则继续后续流程。
        else if (strcmp(name, "WORK_AREA_LOADED") == 0) mode = VisionStartMode::WORK_AREA_LOADED;  // 仅在先前条件均不成立时，当 strcmp(name, "WORK_AREA_LOADED") == 0 成立时进入此分支，否则继续后续流程。
        else if (strcmp(name, "CORNER") == 0) mode = VisionStartMode::CORNER;  // 仅在先前条件均不成立时，当 strcmp(name, "CORNER") == 0 成立时进入此分支，否则继续后续流程。
        else { serialError(frame, "RANGE"); return; }  // 前一个判断不成立时执行此分支。
        const bool start=strcmp(frame.fields[2],"START_REQUEST")==0;  // 将 strcmp(frame.fields[2],"START_REQUEST")==0 写入或初始化 start（起点）。
        if(start&&(mode==VisionStartMode::WORK_AREA||mode==VisionStartMode::WORK_AREA_LOADED)) {  // 当 start&&(mode==VisionStartMode::WORK_AREA||mode==Vision… 成立时进入此分支，否则继续后续流程。
            MotionRequest r;r.kind=MotionKind::ARM_POSE;r.observation=ArmObservation::LOOK;r.startVision=true;r.vision=static_cast<uint8_t>(mode);  // 声明 r，类型和可用范围由所在声明决定。然后将 MotionKind::ARM_POSE 写入或初始化 r.kind。然后将 ArmObservation::LOOK 写入或初始化 r.observation。然后将 true 写入或初始化 r.startVision。然后将 static_cast<uint8_t>(mode) 写入或初始化 r.vision。
            const uint32_t id=serialPrepareMotion(frame,r);if(!id)return;  // 将 serialPrepareMotion(frame,r) 写入或初始化 id（编号）。然后当 !id 成立时进入此分支，否则继续后续流程。
            Serial.printf("{RSP,VISION,START_REQUEST,ACK,%s}\n",name);LaunchMotion(id);  // 调用 Serial.printf，传入 "{RSP,VISION,START_REQUEST,ACK,%s}\n",name);LaunchMoti…。
        } else {  // 前一个判断不成立时执行此分支。
            Serial.printf("{RSP,VISION,%s,ACK,%s}\n",frame.fields[2],name);  // 调用 Serial.printf，传入 "{RSP,VISION,%s,ACK,%s}\n",frame.fields[2],name。
            if(start)requestVisionStart(mode);else requestVisionStop(mode);  // 当 start 成立时进入此分支，否则继续后续流程。
        }  // 结束此条件分支。
    } else if (frame.is("VISION", "ALIGN_START", 0)  // 仅在先前条件均不成立时，当 frame.is("VISION", "ALIGN_START", 0 成立时进入此分支，否则继续后续流程。
               || frame.is("VISION", "ALIGN_START", 1)) {  // 继续上一行的组合条件，本行提供 || frame.is("VISION", "ALIGN_START", 1)) {。
        VisionStartMode mode = VisionStartMode::NONE; // 自动期间沿用当前模式；否则旧命令使用 DISC。
        if (frame.count == 4) {  // 当 frame.count == 4 成立时进入此分支，否则继续后续流程。
            const char *name = frame.fields[3];  // 将 frame.fields[3] 写入或初始化 name。
            if (strcmp(name, "DISC") == 0) mode = VisionStartMode::DISC;  // 当 strcmp(name, "DISC") == 0 成立时进入此分支，否则继续后续流程。
            else if (strcmp(name, "DISC_MATERIAL") == 0) mode = VisionStartMode::DISC_MATERIAL;  // 仅在先前条件均不成立时，当 strcmp(name, "DISC_MATERIAL") == 0 成立时进入此分支，否则继续后续流程。
            else if (strcmp(name, "WORK_AREA") == 0) mode = VisionStartMode::WORK_AREA;  // 仅在先前条件均不成立时，当 strcmp(name, "WORK_AREA") == 0 成立时进入此分支，否则继续后续流程。
            else if (strcmp(name, "WORK_AREA_LOADED") == 0) mode = VisionStartMode::WORK_AREA_LOADED;  // 仅在先前条件均不成立时，当 strcmp(name, "WORK_AREA_LOADED") == 0 成立时进入此分支，否则继续后续流程。
            else if (strcmp(name, "CORNER") == 0) mode = VisionStartMode::CORNER;  // 仅在先前条件均不成立时，当 strcmp(name, "CORNER") == 0 成立时进入此分支，否则继续后续流程。
            else { serialError(frame, "RANGE"); return; }  // 前一个判断不成立时执行此分支。
        }  // 结束此条件分支。
        setAlignmentEnabled(true, true, false, mode);  // 调用 setAlignmentEnabled（setAlignment已启用），传入 true, true, false, mode。
    } else if (frame.is("VISION", "ALIGN_STOP", 0)) {  // 仅在先前条件均不成立时，当 frame.is("VISION", "ALIGN_STOP", 0) 成立时进入此分支，否则继续后续流程。
        setAlignmentEnabled(false, true);  // 调用 setAlignmentEnabled（setAlignment已启用），传入 false, true。
    // 对齐误差帧属于已经开启的会话。
    // 接收端根据当前模式分发误差；是否继续闭环和何时结束由运行时对齐状态决定。
    } else if (frame.is("VISION", "ALIGN_DATA", 3)) {  // 仅在先前条件均不成立时，当 frame.is("VISION", "ALIGN_DATA", 3) 成立时进入此分支，否则继续后续流程。
        const float low[] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};  // 声明 low，类型和可用范围由所在声明决定。
        const float high[] = {FLT_MAX, FLT_MAX, FLT_MAX};  // 声明 high，类型和可用范围由所在声明决定。
        float value[3];  // 声明 value（测值），类型和可用范围由所在声明决定。
        if (!serialFloats(frame, value, low, high, 3)) {  // 当 !serialFloats(frame, value, low, high, 3) 成立时进入此分支，否则继续后续流程。
            xSemaphoreTake(xAlignmentMotionMutex, portMAX_DELAY);  // 调用 xSemaphoreTake（横向SemaphoreTake），传入 xAlignmentMotionMutex, portMAX_DELAY。
            alignmentStopPending=true;  // 将 true 写入或初始化 alignmentStopPending（alignment停止Pending）。
            ResetDiscAlignmentPid();  // 调用 ResetDiscAlignmentPid（Reset圆盘AlignmentPID），使用无参形式推进相应操作。
            ResetAlignmentWait();  // 调用 ResetAlignmentWait，使用无参形式推进相应操作。
            if (xAlignmentQueue) xQueueReset(xAlignmentQueue);  // 当 xAlignmentQueue 成立时进入此分支，否则继续后续流程。
            ++alignmentGeneration; // 已出队的旧帧也不能延续连续计数。
            xSemaphoreGive(xAlignmentMotionMutex);  // 调用 xSemaphoreGive（横向SemaphoreGive），传入 xAlignmentMotionMutex。
            WakeMotionExecutor();serialError(frame, "RANGE"); return;  // 调用 WakeMotionExecutor，使用无参形式推进相应操作。然后调用 serialError（serial错误），传入 frame, "RANGE"。然后结束当前函数，将执行权交回调用者。
        }  // 结束此条件分支。
        xSemaphoreTake(xAlignmentMotionMutex, portMAX_DELAY);  // 调用 xSemaphoreTake（横向SemaphoreTake），传入 xAlignmentMotionMutex, portMAX_DELAY。
        VisualAlignmentFrame_t alignment = {value[0], value[1], value[2], ++alignmentFrameSequence};  // 将 {value[0], value[1], value[2], ++alignmentFrameSequenc… 写入或初始化 alignment。
        if (alignmentEnabled && xAlignmentQueue) {  // 当 alignmentEnabled && xAlignmentQueue 成立时进入此分支，否则继续后续流程。
            xQueueOverwrite(xAlignmentQueue, &alignment);  // 调用 xQueueOverwrite（横向QueueOverwrite），传入 xAlignmentQueue, &alignment。
        } else {  // 前一个判断不成立时执行此分支。
            // 断流/超时/手动停止后不允许反馈自行重启运动；限频提示被丢弃的原因。
            static uint32_t lastIgnoredMs = 0;  // 将 0 写入或初始化 lastIgnoredMs（最后Ignored毫秒）。
            const uint32_t now = millis();  // 将 millis() 写入或初始化 now。
            if (now - lastIgnoredMs >= 1000) {  // 当 now - lastIgnoredMs >= 1000 成立时进入此分支，否则继续后续流程。
                Serial.println("{EVT,VISION,ALIGN_IGNORED,DISABLED}");  // 调用 Serial.println，传入 "{EVT,VISION,ALIGN_IGNORED,DISABLED}"。
                lastIgnoredMs = now;  // 将 now 写入或初始化 lastIgnoredMs（最后Ignored毫秒）。
            }  // 结束此条件分支。
        }  // 结束此条件分支。
        xSemaphoreGive(xAlignmentMotionMutex);WakeMotionExecutor();  // 调用 xSemaphoreGive（横向SemaphoreGive），传入 xAlignmentMotionMutex。然后调用 WakeMotionExecutor，使用无参形式推进相应操作。
    } else if (frame.starts("VISION", "DISC_POSITIONS")) {  // 仅在先前条件均不成立时，当 frame.starts("VISION", "DISC_POSITIONS") 成立时进入此分支，否则继续后续流程。
        serialDiscPositions(frame);  // 调用 serialDiscPositions（serial圆盘Positions），传入 frame。
    } else if (frame.is("VISION", "COLOR", 1)) {  // 仅在先前条件均不成立时，当 frame.is("VISION", "COLOR", 1) 成立时进入此分支，否则继续后续流程。
        uint32_t color = 0;  // 将 0 写入或初始化 color（颜色）。
        if (!serialUInt(frame.fields[3], color, 6) || color == 0) {  // 当 !serialUInt(frame.fields[3], color, 6) || color == 0 成立时进入此分支，否则继续后续流程。
            serialError(frame, "RANGE"); return;  // 调用 serialError（serial错误），传入 frame, "RANGE"。然后结束当前函数，将执行权交回调用者。
        }  // 结束此条件分支。
        if (!discMaterialActive) { serialError(frame, "NOT_ACTIVE"); return; }  // 当 !discMaterialActive 成立时进入此分支，否则继续后续流程。
        // 抓取、放料及恢复观察姿态期间丢弃颜色，不更新缓存，也不触发抓取。
        // 仍按协议确认收帧；恢复处理后只使用新收到的颜色。
        if (!serialDebugMode  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
                && (currentState == STATE_GRAB_ROUND1 || currentState == STATE_GRAB_ROUND2)  // 继续上一行的组合条件，本行提供 && (currentState == STATE_GRAB_ROUND1 || currentState == STATE_GRAB_ROUND2)。
                && !firstDiscGrabReady) {  // 继续上一行的组合条件，本行提供 && !firstDiscGrabReady) {。
            serialReply(frame, "OK");  // 调用 serialReply，传入 frame, "OK"。
            return;  // 结束当前函数，将执行权交回调用者。
        }  // 结束此函数或控制块。
        traceVisionFirstFrame(VisionStartMode::DISC_MATERIAL);  // 调用 traceVisionFirstFrame（trace视觉首个帧），传入 VisionStartMode::DISC_MATERIAL。
        discMaterialColor = static_cast<uint8_t>(color);  // 将 static_cast<uint8_t>(color) 写入或初始化 discMaterialColor（圆盘物料颜色）。
        serialReply(frame, "OK");  // 调用 serialReply，传入 frame, "OK"。
        serialDiscColorDecision(static_cast<uint8_t>(color));  // 调用 serialDiscColorDecision（serial圆盘颜色Decision），传入 static_cast<uint8_t>(color)。
    } else if (frame.is("VISION", "GRAB", 1) || frame.is("VISION", "GRAB", 2)) {  // 仅在先前条件均不成立时，当 frame.is("VISION", "GRAB", 1) || frame.is("VISION", "G… 成立时进入此分支，否则继续后续流程。
        serialDiscGrab(frame);  // 调用 serialDiscGrab（serial圆盘Grab），传入 frame。
    } else if (frame.is("VISION", "FORCE_GRAB", 2)) {  // 仅在先前条件均不成立时，当 frame.is("VISION", "FORCE_GRAB", 2) 成立时进入此分支，否则继续后续流程。
        uint32_t color = 0, cargo = 0;  // 将 0, cargo = 0 写入或初始化 color（颜色）。
        if (!serialUInt(frame.fields[3], color, 6) || color == 0  // 当 !serialUInt(frame.fields[3], color, 6) || color == 0 成立时进入此分支，否则继续后续流程。
                || !serialUInt(frame.fields[4], cargo, 3) || cargo == 0) {  // 继续上一行的组合条件，本行提供 || !serialUInt(frame.fields[4], cargo, 3) || cargo == 0) {。
            serialError(frame, "RANGE"); return;  // 调用 serialError（serial错误），传入 frame, "RANGE"。然后结束当前函数，将执行权交回调用者。
        }  // 结束此函数或控制块。
        if(!serialQueueDiscGrab(color,cargo,0,3,0,true))serialError(frame,"BUSY");  // 当 !serialQueueDiscGrab(color,cargo,0,3,0,true) 成立时进入此分支，否则继续后续流程。
    } else if (frame.is("VISION", "CONFIRM", 0)) {  // 仅在先前条件均不成立时，当 frame.is("VISION", "CONFIRM", 0) 成立时进入此分支，否则继续后续流程。
        if (serialDebugMode) { serialError(frame, "MODE"); return; }  // 当 serialDebugMode 成立时进入此分支，否则继续后续流程。
        VisualCmd_t command = {"OK", 0, 0, 0};  // 将 {"OK", 0, 0, 0} 写入或初始化 command。
        if (xVisualTaskQueue) xQueueSend(xVisualTaskQueue, &command, 0);  // 当 xVisualTaskQueue 成立时进入此分支，否则继续后续流程。
        serialReply(frame, "ACK");  // 调用 serialReply，传入 frame, "ACK"。
    } else if (frame.is("VISION", "LIDAR_POSE", 0)) {  // 仅在先前条件均不成立时，当 frame.is("VISION", "LIDAR_POSE", 0) 成立时进入此分支，否则继续后续流程。
        const auto zone=ReadPanelStartRequest();  // 将 ReadPanelStartRequest() 写入或初始化 zone。
        if(releaseFromDial&&zone==START_ZONE_UNKNOWN){serialError(frame,"NOT_READY");return;}  // 当 releaseFromDial&&zone==START_ZONE_UNKNOWN 成立时进入此分支，否则继续后续流程。
        SubmitLidarPoseAction(true,serialDebugMode,static_cast<uint8_t>(zone));  // 调用 SubmitLidarPoseAction，传入 true,serialDebugMode,static_cast<uint8_t>(zone)。
    } else if (frame.starts("CFG", "GET") || frame.starts("CFG", "BEGIN")  // 仅在先前条件均不成立时，当 frame.starts("CFG", "GET") || frame.starts("CFG", "BEG… 成立时进入此分支，否则继续后续流程。
               || frame.starts("CFG", "SET") || frame.starts("CFG", "COMMIT")) {  // 继续上一行的组合条件，本行提供 || frame.starts("CFG", "SET") || frame.starts("CFG", "COMMIT")) {。
        const bool reading = frame.starts("CFG", "GET");  // 将 frame.starts("CFG", "GET") 写入或初始化 reading。
        if (!reading && alignmentEnabled) { serialCfgError(frame, "BUSY_OR_MODE"); return; }  // 当 !reading && alignmentEnabled 成立时进入此分支，否则继续后续流程。
        const char *arguments[SERIAL_MAX_FIELDS];  // 声明 arguments，类型和可用范围由所在声明决定。
        for (size_t i = 2; i < frame.count; ++i) arguments[i - 2] = frame.fields[i];  // 按 size_t i = 2; i < frame.count; ++i) arguments[i - 2] =… 组织循环的初值、继续条件和步进。
        if (reading) {  // 当 reading 成立时进入此分支，否则继续后续流程。
            // 参数写入也只由此串口任务执行。只读流式回传无需持有运动锁，
            // 否则几秒的目录回读会阻塞视觉闭环；不启停视觉、不重置PID。
            HandleRuntimeParameters(arguments, frame.count - 2, false);  // 调用 HandleRuntimeParameters，传入 arguments, frame.count - 2, false。
            return;  // 结束当前函数，将执行权交回调用者。
        }  // 结束此条件分支。
        if (xAlignmentMotionMutex && xSemaphoreTake(xAlignmentMotionMutex, portMAX_DELAY) == pdTRUE) {  // 当 xAlignmentMotionMutex && xSemaphoreTake(xAlignmentMoti… 成立时进入此分支，否则继续后续流程。
            HandleRuntimeParameters(arguments, frame.count - 2, serialDebugMode);  // 调用 HandleRuntimeParameters，传入 arguments, frame.count - 2, serialDebugMode。
            xSemaphoreGive(xAlignmentMotionMutex);  // 调用 xSemaphoreGive（横向SemaphoreGive），传入 xAlignmentMotionMutex。
        } else serialCfgError(frame, "LOCK");  // 前一个判断不成立时执行此分支。
    } else if (frame.is("POSE", "SET", 3) || frame.is("POSE", "GOTO_REL", 3)) {  // 仅在先前条件均不成立时，当 frame.is("POSE", "SET", 3) || frame.is("POSE", "GOTO_R… 成立时进入此分支，否则继续后续流程。
        // 2. Debug 运动命令：先校验全部数值，再回复 ACK 和执行。
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
        float values[3];  // 声明 values（参数值），类型和可用范围由所在声明决定。
        const bool set = strcmp(frame.fields[2], "SET") == 0;  // 将 strcmp(frame.fields[2], "SET") == 0 写入或初始化 set。
        const float minimum[] = {set ? 0.0f : -3394.113f, set ? 0.0f : -3394.113f,  // 续列跨行签名或声明中的参数，类型与名称为 const float minimum[] = {set ? 0.0f : -3394.113f, set …。
                                 set ? -FLT_MAX : -360.0f};  // 续写当前声明或运算，提供 set ? -FLT_MAX : -360.0f};。
        const float maximum[] = {set ? 2400.0f : 3394.113f, set ? 2400.0f : 3394.113f,  // 续列跨行签名或声明中的参数，类型与名称为 const float maximum[] = {set ? 2400.0f : 3394.113f, se…。
                                 set ? FLT_MAX : 360.0f};  // 续写当前声明或运算，提供 set ? FLT_MAX : 360.0f};。
        if (!serialFloats(frame, values, minimum, maximum, 3)) {  // 当 !serialFloats(frame, values, minimum, maximum, 3) 成立时进入此分支，否则继续后续流程。
            serialError(frame, "RANGE"); return;  // 调用 serialError（serial错误），传入 frame, "RANGE"。然后结束当前函数，将执行权交回调用者。
        }  // 结束此条件分支。
        if (set) {  // 当 set 成立时进入此分支，否则继续后续流程。
            currentPose = {values[0], values[1], values[2]};  // 将 {values[0], values[1], values[2]} 写入或初始化 currentPose（当前Pose）。
            CalibrateH30MapHeading(currentPose.theta);  // 调用 CalibrateH30MapHeading（CalibrateH30Map航向），传入 currentPose.theta。
        }  // 结束此条件分支。
        uint32_t motionId=0;  // 将 0 写入或初始化 motionId（motion编号）。
        if(!set){MotionRequest r;r.kind=MotionKind::GOTO;r.x=values[0];r.y=values[1];r.theta=values[2];motionId=serialPrepareMotion(frame,r);if(!motionId)return;}  // 当 !set 成立时进入此分支，否则继续后续流程。
        Serial.printf("{RSP,POSE,%s,ACK,%.3f,%.3f,%.3f}\n", frame.fields[2],  // 通过串口执行 printf，输出内容或参数由本行给出。
                      values[0], values[1], values[2]);  // 续写当前声明或运算，提供 values[0], values[1], values[2]);。
        if(!set)LaunchMotion(motionId);  // 当 !set 成立时进入此分支，否则继续后续流程。
    } else if (frame.is("CHASSIS", "MOVE_POSITION", 4)) {  // 仅在先前条件均不成立时，当 frame.is("CHASSIS", "MOVE_POSITION", 4) 成立时进入此分支，否则继续后续流程。
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
        const float minimum[] = {-3394.113f, -3394.113f, -360.0f, 1.0f};  // 声明 minimum，类型和可用范围由所在声明决定。
        const float maximum[] = {3394.113f, 3394.113f, 360.0f, 5000.0f};  // 声明 maximum，类型和可用范围由所在声明决定。
        float values[4];  // 声明 values（参数值），类型和可用范围由所在声明决定。
        if (!serialFloats(frame, values, minimum, maximum, 4)) {  // 当 !serialFloats(frame, values, minimum, maximum, 4) 成立时进入此分支，否则继续后续流程。
            serialError(frame, "RANGE"); return;  // 调用 serialError（serial错误），传入 frame, "RANGE"。然后结束当前函数，将执行权交回调用者。
        }  // 结束此条件分支。
        MotionRequest r;r.kind=MotionKind::POSITION;r.x=values[0];r.y=values[1];r.theta=values[2];r.speed=values[3];  // 声明 r，类型和可用范围由所在声明决定。然后将 MotionKind::POSITION 写入或初始化 r.kind。然后将 values[0] 写入或初始化 r.x。然后将 values[1] 写入或初始化 r.y。然后将 values[2] 写入或初始化 r.theta。然后将 values[3] 写入或初始化 r.speed。
        const uint32_t id=serialPrepareMotion(frame,r);if(!id)return;  // 将 serialPrepareMotion(frame,r) 写入或初始化 id（编号）。然后当 !id 成立时进入此分支，否则继续后续流程。
        Serial.printf("{RSP,CHASSIS,MOVE_POSITION,ACK,%.3f,%.3f,%.3f,%.3f}\n",values[0],values[1],values[2],values[3]);LaunchMotion(id);  // 调用 Serial.printf，传入 "{RSP,CHASSIS,MOVE_POSITION,ACK,%.3f,%.3f,%.3f,%.3f}\n…。
    } else if (frame.is("CHASSIS", "MOVE", 3)) {  // 仅在先前条件均不成立时，当 frame.is("CHASSIS", "MOVE", 3) 成立时进入此分支，否则继续后续流程。
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
        uint32_t direction = 0, stop = 0;  // 将 0, stop = 0 写入或初始化 direction（方向）。
        float speed = 0;  // 将 0 写入或初始化 speed（速度）。
        if (!serialUInt(frame.fields[3], direction, 5)  // 当 !serialUInt(frame.fields[3], direction, 5 成立时进入此分支，否则继续后续流程。
                || !serialFloat(frame.fields[4], speed, 0, 1000)  // 继续上一行的组合条件，本行提供 || !serialFloat(frame.fields[4], speed, 0, 1000)。
                || !serialUInt(frame.fields[5], stop, 1)) {  // 继续上一行的组合条件，本行提供 || !serialUInt(frame.fields[5], stop, 1)) {。
            serialError(frame, "RANGE"); return;  // 调用 serialError（serial错误），传入 frame, "RANGE"。然后结束当前函数，将执行权交回调用者。
        }  // 结束此函数或控制块。
        MotionRequest r;r.kind=MotionKind::VELOCITY;r.position=direction;r.speed=speed;r.force=stop;  // 声明 r，类型和可用范围由所在声明决定。然后将 MotionKind::VELOCITY 写入或初始化 r.kind。然后将 direction 写入或初始化 r.position。然后将 speed 写入或初始化 r.speed。然后将 stop 写入或初始化 r.force。
        const uint32_t id=serialPrepareMotion(frame,r);if(!id)return;  // 将 serialPrepareMotion(frame,r) 写入或初始化 id（编号）。然后当 !id 成立时进入此分支，否则继续后续流程。
        Serial.printf("{RSP,CHASSIS,MOVE,ACK,%u,%.3f,%u}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                      static_cast<unsigned>(direction), speed, static_cast<unsigned>(stop));  // 声明接口 unsigned>，参数由此处的签名确定。
        LaunchMotion(id);  // 调用 LaunchMotion，传入 id。
    } else if (frame.is("ARM", "INIT_START", 0) || frame.is("ARM", "INIT_LOOK", 0)  // 仅在先前条件均不成立时，当 frame.is("ARM", "INIT_START", 0) || frame.is("ARM", "I… 成立时进入此分支，否则继续后续流程。
            || frame.is("ARM", "INIT_LOOK2", 0)) {  // 继续上一行的组合条件，本行提供 || frame.is("ARM", "INIT_LOOK2", 0)) {。
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
        MotionRequest r;r.kind=MotionKind::ARM_POSE;  // 声明 r，类型和可用范围由所在声明决定。然后将 MotionKind::ARM_POSE 写入或初始化 r.kind。
        r.observation=strcmp(frame.fields[2],"INIT_START")==0?ArmObservation::START:  // 将 strcmp(frame.fields[2],"INIT_START")==0?ArmObservation… 写入或初始化 r.observation。
            strcmp(frame.fields[2],"INIT_LOOK")==0?ArmObservation::LOOK:ArmObservation::LOOK2;  // 调用 strcmp，传入 frame.fields[2],"INIT_LOOK")==0?ArmObservation::LOOK:A…。
        serialMotion(frame,r);  // 调用 serialMotion，传入 frame,r。
    } else if (frame.is("ARM", "MOVE1", 3) || frame.is("ARM", "MOVE2", 3)) {  // 仅在先前条件均不成立时，当 frame.is("ARM", "MOVE1", 3) || frame.is("ARM", "MOVE2"… 成立时进入此分支，否则继续后续流程。
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
        const bool first = strcmp(frame.fields[2], "MOVE1") == 0;  // 将 strcmp(frame.fields[2], "MOVE1") == 0 写入或初始化 first（首个）。
        const float minimum[] = {first ? -1.0f : TURRET_CABLE_MIN_DEG, first ? -1.0f : -360.0f, 1.0f};  // 声明 minimum，类型和可用范围由所在声明决定。
        const float maximum[] = {first ? ARM_HEIGHT_LIMIT_MM : 360.0f, first ? 170.0f : ARM_CLAW_MAX_DEG, FLT_MAX};  // 声明 maximum，类型和可用范围由所在声明决定。
        float values[3];  // 声明 values（参数值），类型和可用范围由所在声明决定。
        if (!serialFloats(frame, values, minimum, maximum, 3)  // 当 !serialFloats(frame, values, minimum, maximum, 3 成立时进入此分支，否则继续后续流程。
                || (first && ((values[0] < 0 && values[0] != -1)  // 继续上一行的组合条件，本行提供 || (first && ((values[0] < 0 && values[0] != -1)。
                    || (values[1] < 0 && values[1] != -1)))) {  // 继续上一行的组合条件，本行提供 || (values[1] < 0 && values[1] != -1)))) {。
            serialError(frame, "RANGE"); return;  // 调用 serialError（serial错误），传入 frame, "RANGE"。然后结束当前函数，将执行权交回调用者。
        }  // 结束此初始化列表或局部作用域。
        MotionRequest r;r.kind=MotionKind::ARM_RAW;r.speed=values[2];  // 声明 r，类型和可用范围由所在声明决定。然后将 MotionKind::ARM_RAW 写入或初始化 r.kind。然后将 values[2] 写入或初始化 r.speed。
        if(first){r.high=values[0];r.length=values[1];}else{r.turret=values[0];r.pawl=values[1]=LimitArmClawAngle(values[1]);}  // 当 first 成立时进入此分支，否则继续后续流程。
        const uint32_t id=serialPrepareMotion(frame,r);if(!id)return;  // 将 serialPrepareMotion(frame,r) 写入或初始化 id（编号）。然后当 !id 成立时进入此分支，否则继续后续流程。
        Serial.printf("{RSP,ARM,%s,ACK,%.3f,%.3f,%.3f}\n", frame.fields[2],  // 通过串口执行 printf，输出内容或参数由本行给出。
                      values[0], values[1], values[2]);  // 续写当前声明或运算，提供 values[0], values[1], values[2]);。
        LaunchMotion(id);  // 调用 LaunchMotion，传入 id。
    } else if (frame.is("ARM", "SERVO_PID_GET", 1)||frame.is("ARM", "SERVO_PID_SET", 8)) {  // 仅在先前条件均不成立时，当 frame.is("ARM", "SERVO_PID_GET", 1)||frame.is("ARM", "… 成立时进入此分支，否则继续后续流程。
        if(!serialDebugMode){serialError(frame,"MODE");return;}  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
        uint32_t id=0;  // 将 0 写入或初始化 id（编号）。
        if(!serialUInt(frame.fields[3],id,253)){serialError(frame,"RANGE");return;}  // 当 !serialUInt(frame.fields[3],id,253) 成立时进入此分支，否则继续后续流程。
        uint16_t values[7];  // 声明 values（参数值），类型和可用范围由所在声明决定。
        if(strcmp(frame.fields[2],"SERVO_PID_SET")==0) {  // 当 strcmp(frame.fields[2],"SERVO_PID_SET")==0 成立时进入此分支，否则继续后续流程。
            for(uint8_t i=0;i<7;++i){uint32_t value=0;  // 按 uint8_t i=0;i<7;++i 组织循环的初值、继续条件和步进。
                if(!serialUInt(frame.fields[4+i],value,UINT16_MAX)){serialError(frame,"RANGE");return;}  // 当 !serialUInt(frame.fields[4+i],value,UINT16_MAX) 成立时进入此分支，否则继续后续流程。
                values[i]=value;  // 将 value 写入或初始化 values[i]。
            }  // 结束此循环体。
            if(!Servo_WritePid(id,values)){serialError(frame,"SERVO_WRITE");return;}  // 当 !Servo_WritePid(id,values) 成立时进入此分支，否则继续后续流程。
        }  // 结束此条件分支。
        if(!Servo_ReadPid(id,values)){serialError(frame,"SERVO_READ");return;}  // 当 !Servo_ReadPid(id,values) 成立时进入此分支，否则继续后续流程。
        Serial.printf("{RSP,ARM,%s,READBACK,%u,%u,%u,%u,%u,%u,%u,%u}\n",frame.fields[2],  // 通过串口执行 printf，输出内容或参数由本行给出。
            static_cast<unsigned>(id),values[0],values[1],values[2],values[3],values[4],values[5],values[6]);  // 声明接口 unsigned>，参数由此处的签名确定。
    // 此处分流机械臂转台与载物料盘的置零请求。
    // 相似的命令名称对应不同设备，后续读取和保存的参考值也各有入口。
    } else if (frame.is("ARM", "SERVO_ZERO", 1) || frame.is("TRAY", "ZERO", 0)) {  // 仅在先前条件均不成立时，当 frame.is("ARM", "SERVO_ZERO", 1) || frame.is("TRAY", "… 成立时进入此分支，否则继续后续流程。
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
        const bool tray=frame.is("TRAY","ZERO",0);  // 将 frame.is("TRAY","ZERO",0) 写入或初始化 tray（料盘）。
        uint32_t id=SERVO_ID_CARGO_TRAY;  // 将 SERVO_ID_CARGO_TRAY 写入或初始化 id（编号）。
        if(!tray&&!serialUInt(frame.fields[3],id,253)){serialError(frame,"RANGE");return;}  // 当 !tray&&!serialUInt(frame.fields[3],id,253) 成立时进入此分支，否则继续后续流程。
        if(!tray&&(id==5||id==6)) {  // 当 !tray&&(id==5||id==6) 成立时进入此分支，否则继续后续流程。
            MotorBusGuard transaction;  // 声明 transaction，类型和可用范围由所在声明决定。
            if(ZeroMotorEncoderPosition(id)){serialError(frame,"MOTOR_WRITE");return;}  // 当 ZeroMotorEncoderPosition(id) 成立时进入此分支，否则继续后续流程。
            int64_t position=0;  // 将 0 写入或初始化 position（位置）。
            if(ReadMotorEncoderPosition(id,position)){serialError(frame,"MOTOR_READ");return;}  // 当 ReadMotorEncoderPosition(id,position) 成立时进入此分支，否则继续后续流程。
            // 两个直线轴的正向位置指令均为CCW，对应负编码器坐标。
            const float measured=static_cast<double>(-position)*3200.0/65536.0/(id==5?HEIGHT_PULSE:LENGTH_PULSE);  // 将 static_cast<double>(-position)*3200.0/65536.0/(id==5?H… 写入或初始化 measured。
            if(id==5)currentArm.high=measured;  // 当 id==5 成立时进入此分支，否则继续后续流程。
            else currentArm.length=measured;  // 前一个判断不成立时执行此分支。
            Serial.printf("{RSP,ARM,SERVO_ZERO,READBACK,%u,%.3f,%.3f}\n",static_cast<unsigned>(id),measured,measured);  // 调用 Serial.printf，传入 "{RSP,ARM,SERVO_ZERO,READBACK,%u,%.3f,%.3f}\n",static_…。
            return;  // 结束当前函数，将执行权交回调用者。
        }  // 结束此条件分支。
        if(!tray&&id==2) { // ARM 旧逻辑轴 2 对应 EMM 7，和总线舵机地址无关。
            float angle=0;  // 将 0 写入或初始化 angle（角度）。
            if(!TurretMotorSetZero(angle)){serialError(frame,"MOTOR_READ");return;}  // 当 !TurretMotorSetZero(angle) 成立时进入此分支，否则继续后续流程。
            currentArm.turret_angle=angle;  // 将 angle 写入或初始化 currentArm.turret_angle。
            Serial.printf("{RSP,ARM,SERVO_ZERO,READBACK,2,%.3f,%.3f}\n",angle,angle);  // 调用 Serial.printf，传入 "{RSP,ARM,SERVO_ZERO,READBACK,2,%.3f,%.3f}\n",angle,an…。
            return;  // 结束当前函数，将执行权交回调用者。
        }  // 结束此条件分支。
        if(!Servo_SetOrigin(id)){serialError(frame,"SERVO_WRITE");return;}  // 当 !Servo_SetOrigin(id) 成立时进入此分支，否则继续后续流程。
        float single=0,multi=0;  // 将 0,multi=0 写入或初始化 single。
        if(!Servo_QueryAngle(id,single,100)||!Servo_QueryAngleMTurn(id,multi,100)){  // 当 !Servo_QueryAngle(id,single,100)||!Servo_QueryAngleMTu… 成立时进入此分支，否则继续后续流程。
            serialError(frame,"SERVO_READ");return;  // 调用 serialError（serial错误），传入 frame,"SERVO_READ"。然后结束当前函数，将执行权交回调用者。
        }  // 结束此条件分支。
        if(id==SERVO_ID_CLAW)currentArm.pawl_angle=multi;  // 当 id==SERVO_ID_CLAW 成立时进入此分支，否则继续后续流程。
        Serial.printf("{RSP,%s,%s,READBACK,%u,%.3f,%.3f}\n",frame.fields[1],frame.fields[2],static_cast<unsigned>(id),single,multi);  // 调用 Serial.printf，传入 "{RSP,%s,%s,READBACK,%u,%.3f,%.3f}\n",frame.fields[1],…。
    } else if (frame.is("ARM", "SERVO", 2)) {  // 仅在先前条件均不成立时，当 frame.is("ARM", "SERVO", 2) 成立时进入此分支，否则继续后续流程。
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
        uint32_t id = 0;  // 将 0 写入或初始化 id（编号）。
        float angle = 0;  // 将 0 写入或初始化 angle（角度）。
        if (!serialUInt(frame.fields[3], id, 254)  // 当 !serialUInt(frame.fields[3], id, 254 成立时进入此分支，否则继续后续流程。
                || id == SERVO_ID_BROADCAST  // 继续上一行的组合条件，本行提供 || id == SERVO_ID_BROADCAST。
                || !serialFloat(frame.fields[4], angle,  // 继续上一行的组合条件，本行提供 || !serialFloat(frame.fields[4], angle,。
                                id == 2 ? TURRET_CABLE_MIN_DEG : -135.0f,  // 为当前列表或跨行表达式提供这一项 id == 2 ? TURRET_CABLE_MIN_DEG : -135.0f。
                                id == 2 ? TURRET_CABLE_MAX_DEG : id == SERVO_ID_CLAW ? ARM_CLAW_MAX_DEG : 135.0f)) {  // 续写当前声明或运算，提供 id == 2 ? TURRET_CABLE_MAX_DEG : id == SERVO_ID_CLAW ?…。
            serialError(frame, "RANGE"); return;  // 调用 serialError（serial错误），传入 frame, "RANGE"。然后结束当前函数，将执行权交回调用者。
        }  // 结束此初始化列表或局部作用域。
        // ARM SERVO 保留旧逻辑轴 2；料盘旋转通过 TRAY ROTATE。
        MotionRequest r;r.kind=id==2?MotionKind::ARM_RAW:MotionKind::SERVO_RAW;r.cargo=id;r.turret=angle;r.speed=150;  // 声明 r，类型和可用范围由所在声明决定。然后将 id==2?MotionKind::ARM_RAW:MotionKind::SERVO_RAW 写入或初始化 r.kind。然后将 id 写入或初始化 r.cargo。然后将 angle 写入或初始化 r.turret。然后将 150 写入或初始化 r.speed。
        const uint32_t motionId=serialPrepareMotion(frame,r);if(!motionId)return;  // 将 serialPrepareMotion(frame,r) 写入或初始化 motionId（motion编号）。然后当 !motionId 成立时进入此分支，否则继续后续流程。
        Serial.printf("{RSP,ARM,SERVO,ACK,%u,%.3f}\n",static_cast<unsigned>(id),angle);LaunchMotion(motionId);  // 调用 Serial.printf，传入 "{RSP,ARM,SERVO,ACK,%u,%.3f}\n",static_cast<unsigned>(…。
    } else if (frame.is("TRAY", "ROTATE", 3)) {  // 仅在先前条件均不成立时，当 frame.is("TRAY", "ROTATE", 3) 成立时进入此分支，否则继续后续流程。
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
        MotionRequest r;r.kind=MotionKind::TRAY_ROTATE;  // 声明 r，类型和可用范围由所在声明决定。然后将 MotionKind::TRAY_ROTATE 写入或初始化 r.kind。
        uint32_t power=0;  // 将 0 写入或初始化 power（功率）。
        if(!serialFloat(frame.fields[3],r.turret,-368640,368640)  // 当 !serialFloat(frame.fields[3],r.turret,-368640,368640 成立时进入此分支，否则继续后续流程。
            ||!serialUInt(frame.fields[4],r.trayIntervalMs,UINT32_MAX)  // 继续上一行的组合条件，本行提供 ||!serialUInt(frame.fields[4],r.trayIntervalMs,UINT32_MAX)。
            ||!serialUInt(frame.fields[5],power,UINT16_MAX)) {  // 继续上一行的组合条件，本行提供 ||!serialUInt(frame.fields[5],power,UINT16_MAX)) {。
            serialError(frame,"RANGE");return;  // 调用 serialError（serial错误），传入 frame,"RANGE"。然后结束当前函数，将执行权交回调用者。
        }  // 结束此函数或控制块。
        r.trayPowerMw=static_cast<uint16_t>(power);  // 将 static_cast<uint16_t>(power) 写入或初始化 r.trayPowerMw。
        serialMotion(frame,r);  // 调用 serialMotion，传入 frame,r。
    } else if (frame.is("ARM", "ENABLE", 1)) {  // 仅在先前条件均不成立时，当 frame.is("ARM", "ENABLE", 1) 成立时进入此分支，否则继续后续流程。
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
        uint32_t enabled = 0;  // 将 0 写入或初始化 enabled（已启用）。
        if (!serialUInt(frame.fields[3], enabled, 1)) { serialError(frame, "RANGE"); return; }  // 当 !serialUInt(frame.fields[3], enabled, 1) 成立时进入此分支，否则继续后续流程。
        MotionRequest r;r.kind=MotionKind::ENABLE;r.force=enabled;  // 声明 r，类型和可用范围由所在声明决定。然后将 MotionKind::ENABLE 写入或初始化 r.kind。然后将 enabled 写入或初始化 r.force。
        const uint32_t id=serialPrepareMotion(frame,r);if(!id)return;  // 将 serialPrepareMotion(frame,r) 写入或初始化 id（编号）。然后当 !id 成立时进入此分支，否则继续后续流程。
        Serial.printf("{RSP,ARM,ENABLE,ACK,%u}\n",static_cast<unsigned>(enabled));LaunchMotion(id);  // 调用 Serial.printf，传入 "{RSP,ARM,ENABLE,ACK,%u}\n",static_cast<unsigned>(enab…。
    } else if (frame.is("ARM", "TRANSFER", 2)) {  // 仅在先前条件均不成立时，当 frame.is("ARM", "TRANSFER", 2) 成立时进入此分支，否则继续后续流程。
        serialTransfer(frame);  // 调用 serialTransfer，传入 frame。
    } else if (frame.is("ARM", "RETRIEVE", 2)) {  // 仅在先前条件均不成立时，当 frame.is("ARM", "RETRIEVE", 2) 成立时进入此分支，否则继续后续流程。
        serialRetrieve(frame);  // 调用 serialRetrieve，传入 frame。
    } else if (frame.is("ARM", "RING_PREPARE", 1) || frame.is("ARM", "RING_DEMO", 0)) {  // 仅在先前条件均不成立时，当 frame.is("ARM", "RING_PREPARE", 1) || frame.is("ARM", … 成立时进入此分支，否则继续后续流程。
        if(!serialDebugMode){serialError(frame,"MODE");return;}  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
        MotionRequest r;  // 声明 r，类型和可用范围由所在声明决定。
        if(frame.count==4) {  // 当 frame.count==4 成立时进入此分支，否则继续后续流程。
            uint32_t work=0;  // 将 0 写入或初始化 work（工位）。
            if(!serialUInt(frame.fields[3],work,3)||work==0){serialError(frame,"RANGE");return;}  // 当 !serialUInt(frame.fields[3],work,3)||work==0 成立时进入此分支，否则继续后续流程。
            r.kind=MotionKind::RING_PREPARE;  // 将 MotionKind::RING_PREPARE 写入或初始化 r.kind。
            r.work=work;  // 将 work 写入或初始化 r.work。
        } else {  // 前一个判断不成立时执行此分支。
            r.kind=MotionKind::DEMO_PLACE;  // 将 MotionKind::DEMO_PLACE 写入或初始化 r.kind。
            r.ringCorrection=true;  // 将 true 写入或初始化 r.ringCorrection。
        }  // 结束此条件分支。
        r.debug=true;  // 将 true 写入或初始化 r.debug。
        serialMotion(frame,r);  // 调用 serialMotion，传入 frame,r。
    } else if (frame.is("ARM", "DEMO1", 0) || frame.is("ARM", "DEMO2", 0)  // 仅在先前条件均不成立时，当 frame.is("ARM", "DEMO1", 0) || frame.is("ARM", "DEMO2"… 成立时进入此分支，否则继续后续流程。
               || frame.is("ARM", "DEMO3", 0) || frame.is("ARM", "DEMO4", 0)) {  // 继续上一行的组合条件，本行提供 || frame.is("ARM", "DEMO3", 0) || frame.is("ARM", "DEMO4", 0)) {。
        if (!serialDebugMode) { serialError(frame, "MODE"); return; }  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
        MotionRequest r;r.kind=strcmp(frame.fields[2],"DEMO4")==0?MotionKind::DEMO_RETRIEVE:MotionKind::DEMO_PLACE;  // 声明 r，类型和可用范围由所在声明决定。然后将 strcmp(frame.fields[2],"DEMO4")==0?MotionKind::DEMO_RE… 写入或初始化 r.kind。
        r.layer=strcmp(frame.fields[2],"DEMO3")==0?3:strcmp(frame.fields[2],"DEMO2")==0?2:1;serialMotion(frame,r);  // 将 strcmp(frame.fields[2],"DEMO3")==0?3:strcmp(frame.fiel… 写入或初始化 r.layer。然后调用 serialMotion，传入 frame,r。
    } else if (frame.count >= 3) {  // 仅在先前条件均不成立时，当 frame.count >= 3 成立时进入此分支，否则继续后续流程。
        serialError(frame, serialKnownAction(frame) ? "FORMAT" : "UNKNOWN_ACTION");  // 调用 serialError（serial错误），传入 frame, serialKnownAction(frame) ? "FORMAT" : "UNKNOWN_…。
    }  // 结束此条件分支。
}  // 结束此函数或控制块。

/**
 * @brief 处理连续路径试跑配置、准备、启动、状态和停止指令；返回是否已匹配处理。
 */
// 试验命令处理上传、准备、启动和状态查询。
// 新增试验参数时，应对应检查解析字段、连续试验配置及状态回传。
static bool serialTrial(const SerialFrame &frame) {  // 定义函数或成员 的入口 serialTrial，参数由此处的签名确定。
    if(strcmp(frame.fields[1],"NAV")!=0||strncmp(frame.fields[2],"TRIAL_",6)!=0)return false;  // 若 strcmp(frame.fields[1],"NAV")!=0||strncmp(frame.fields…，立即返回 false。
    uint32_t id=0;  // 将 0 写入或初始化 id（编号）。
    if(!serialDebugMode){serialError(frame,"MODE");return true;}  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
    if(frame.count<4||!serialUInt(frame.fields[3],id,9999999)||id==0){serialError(frame,"RANGE");return true;}  // 当 frame.count<4||!serialUInt(frame.fields[3],id,9999999)… 成立时进入此分支，否则继续后续流程。
    const char *error=nullptr;  // 将 nullptr 写入或初始化 error（错误）。
    if(frame.is("NAV","TRIAL_BEGIN",12)) {  // 当 frame.is("NAV","TRIAL_BEGIN",12) 成立时进入此分支，否则继续后续流程。
        uint32_t count=0;  // 将 0 写入或初始化 count（计数）。
        continuous::Config cfg;cfg.id=id;  // 声明 cfg（配置），类型和可用范围由所在声明决定。然后将 id 写入或初始化 cfg.id。
        if(!serialUInt(frame.fields[4],count,25)||count<2){serialError(frame,"RANGE");return true;}  // 当 !serialUInt(frame.fields[4],count,25)||count<2 成立时进入此分支，否则继续后续流程。
        cfg.count=count;  // 将 count 写入或初始化 cfg.count。
        float *dest[]={&cfg.startHeading,&cfg.endHeading,&cfg.maxRpm,&cfg.accel,&cfg.lateralAccel,  // 续列跨行签名或声明中的参数，类型与名称为 float *dest[]={&cfg.startHeading,&cfg.endHeading,&cfg.…。
            &cfg.lateralPpm,&cfg.forwardPpm,&cfg.kp,&cfg.maxOmega,&cfg.correction};  // 续写当前声明或运算，提供 &cfg.lateralPpm,&cfg.forwardPpm,&cfg.kp,&cfg.maxOmega,…。
        const float low[]={0,0,1,1,1,0.01f,0.01f,0.1f,5,1};  // 声明 low，类型和可用范围由所在声明决定。
        const float high[]={359.999f,359.999f,5000,2000,3000,1000,1000,10,90,89};  // 声明 high，类型和可用范围由所在声明决定。
        for(size_t i=0;i<10;++i)if(!serialFloat(frame.fields[i+5],*dest[i],low[i],high[i])) {  // 按 size_t i=0;i<10;++i)if(!serialFloat(frame.fields[i+5],… 组织循环的初值、继续条件和步进。
            serialError(frame,"RANGE");return true;  // 调用 serialError（serial错误），传入 frame,"RANGE"。然后结束当前函数并返回 true。
        }  // 结束此循环体。
        if(cfg.correction>=cfg.maxOmega){serialError(frame,"RANGE");return true;}  // 当 cfg.correction>=cfg.maxOmega 成立时进入此分支，否则继续后续流程。
        error=ContinuousTrialBegin(cfg);  // 将 ContinuousTrialBegin(cfg) 写入或初始化 error（错误）。
    } else if(frame.is("NAV","TRIAL_POLICY",5)) {  // 仅在先前条件均不成立时，当 frame.is("NAV","TRIAL_POLICY",5) 成立时进入此分支，否则继续后续流程。
        uint32_t policy=0,geometry=0;float outset=0,margin=0;  // 将 0,geometry=0 写入或初始化 policy。然后将 0,margin=0 写入或初始化 outset。
        if(!serialUInt(frame.fields[4],policy,2)||policy<1  // 当 !serialUInt(frame.fields[4],policy,2)||policy<1 成立时进入此分支，否则继续后续流程。
            ||!serialFloat(frame.fields[5],outset,0,100)||!serialFloat(frame.fields[6],margin,10,40)  // 继续上一行的组合条件，本行提供 ||!serialFloat(frame.fields[5],outset,0,100)||!serialFloat(frame.fields[6],margin,10,40)。
            ||!serialUInt(frame.fields[7],geometry,UINT32_MAX)) {serialError(frame,"RANGE");return true;}  // 继续上一行的组合条件，本行提供 ||!serialUInt(frame.fields[7],geometry,UINT32_MAX)) {serialError(frame,"RANGE");return tr…。
        error=ContinuousTrialPolicy(id,policy,outset,margin,geometry);  // 将 ContinuousTrialPolicy(id,policy,outset,margin,geometry) 写入或初始化 error（错误）。
    } else if(frame.is("NAV","TRIAL_NODE",6)) {  // 仅在先前条件均不成立时，当 frame.is("NAV","TRIAL_NODE",6) 成立时进入此分支，否则继续后续流程。
        uint32_t index=0,node=0;float radius=0,x=0,y=0;  // 将 0,node=0 写入或初始化 index（下标）。然后将 0,x=0,y=0 写入或初始化 radius。
        if(!serialUInt(frame.fields[4],index,24)||!serialUInt(frame.fields[5],node,24)  // 当 !serialUInt(frame.fields[4],index,24)||!serialUInt(fra… 成立时进入此分支，否则继续后续流程。
            ||!serialFloat(frame.fields[6],radius,0,1000)  // 继续上一行的组合条件，本行提供 ||!serialFloat(frame.fields[6],radius,0,1000)。
            ||!serialFloat(frame.fields[7],x,0,2400)||!serialFloat(frame.fields[8],y,0,2400)) {  // 继续上一行的组合条件，本行提供 ||!serialFloat(frame.fields[7],x,0,2400)||!serialFloat(frame.fields[8],y,0,2400)) {。
            serialError(frame,"RANGE");return true;  // 调用 serialError（serial错误），传入 frame,"RANGE"。然后结束当前函数并返回 true。
        }  // 结束此函数或控制块。
        error=ContinuousTrialNode(id,index,node,radius,x,y);  // 将 ContinuousTrialNode(id,index,node,radius,x,y) 写入或初始化 error（错误）。
    } else if(frame.is("NAV","TRIAL_START",1))error=ContinuousTrialPrepare(id);  // 仅在先前条件均不成立时，当 frame.is("NAV","TRIAL_START",1) 成立时进入此分支，否则继续后续流程。
    else if(frame.is("NAV","TRIAL_STOP",1))error=ContinuousTrialStop(id);  // 仅在先前条件均不成立时，当 frame.is("NAV","TRIAL_STOP",1) 成立时进入此分支，否则继续后续流程。
    else if(frame.is("NAV","TRIAL_STATUS",1)) {  // 仅在先前条件均不成立时，当 frame.is("NAV","TRIAL_STATUS",1) 成立时进入此分支，否则继续后续流程。
        error=ContinuousTrialStatus(id);if(!error)return true;  // 将 ContinuousTrialStatus(id) 写入或初始化 error（错误）。然后若 !error，立即返回 true。
    } else {serialError(frame,"FORMAT");return true;}  // 前一个判断不成立时执行此分支。
    if(error){  // 当 error 成立时进入此分支，否则继续后续流程。
        Serial.printf("{RSP,NAV,%s,ERR,%lu,%s}\n",frame.fields[2],static_cast<unsigned long>(id),error);  // 调用 Serial.printf，传入 "{RSP,NAV,%s,ERR,%lu,%s}\n",frame.fields[2],static_cas…。
    } else {  // 前一个判断不成立时执行此分支。
        if(frame.is("NAV","TRIAL_NODE",6))  // 当 frame.is("NAV","TRIAL_NODE",6) 成立时进入此分支，否则继续后续流程。
            Serial.printf("{RSP,NAV,TRIAL_NODE,ACK,%lu,%s}\n",static_cast<unsigned long>(id),frame.fields[4]);  // 调用 Serial.printf，传入 "{RSP,NAV,TRIAL_NODE,ACK,%lu,%s}\n",static_cast<unsign…。
        else Serial.printf("{RSP,NAV,%s,ACK,%lu}\n",frame.fields[2],static_cast<unsigned long>(id));  // 前一个判断不成立时执行此分支。
        if(frame.is("NAV","TRIAL_START",1))ContinuousTrialLaunch();  // 当 frame.is("NAV","TRIAL_START",1) 成立时进入此分支，否则继续后续流程。
    }  // 结束此条件分支。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

/**
 * @brief 接收串口帧并分发查询、参数、调试与运动请求；运动交给统一执行器。
 */
void Task_Serial_CMD(void *pvParameters) {  // 定义函数或成员 的入口 Task_Serial_CMD（任务Serial命令），参数由此处的签名确定。
    // 1. 只有本任务读取 Serial0；Debug/Release 共用缓冲区和收帧规则。
    char line[SERIAL_FRAME_BYTES];  // 声明 line，类型和可用范围由所在声明决定。
    size_t length = 0;  // 将 0 写入或初始化 length（长度）。
    bool overflow = false;  // 将 false 写入或初始化 overflow。
    bool invalid = false;  // 将 false 写入或初始化 invalid。
    bool discResumeDraining = false;  // 将 false 写入或初始化 discResumeDraining（圆盘ResumeDraining）。
    size_t discOldBytesRemaining = 0;  // 将 0 写入或初始化 discOldBytesRemaining（圆盘Old字节剩余）。
    // 固定旧数据的边界，持续新来帧不能无限延长屏蔽时间。
    // 旧数据仍正常分发：COLOR静默丢弃，DISC_POSITIONS回复NOT_READY且不动作；其他指令不受影响。
    const auto resumeDiscAtFrameBoundary = [&]() {  // 将 [&]() { 写入或初始化 resumeDiscAtFrameBoundary（resume圆盘At帧Boundary）。
        if (discMessageResumeRequested && !discResumeDraining) {  // 当 discMessageResumeRequested && !discResumeDraining 成立时进入此分支，否则继续后续流程。
            discOldBytesRemaining = static_cast<size_t>(Serial.available());  // 将 static_cast<size_t>(Serial.available()) 写入或初始化 discOldBytesRemaining（圆盘Old字节剩余）。
            discResumeDraining = true;  // 将 true 写入或初始化 discResumeDraining（圆盘ResumeDraining）。
            discMessageResumeRequested = false;  // 将 false 写入或初始化 discMessageResumeRequested（圆盘MessageResumeRequested）。
        }  // 结束此条件分支。
        if (!discResumeDraining || discOldBytesRemaining != 0  // 当 !discResumeDraining || discOldBytesRemaining != 0 成立时进入此分支，否则继续后续流程。
                || length != 0 || overflow || invalid) return;  // 继续上一行的组合条件，本行提供 || length != 0 || overflow || invalid) return;。
        discResumeDraining = false;  // 将 false 写入或初始化 discResumeDraining（圆盘ResumeDraining）。
        discMaterialColor = 0;  // 将 0 写入或初始化 discMaterialColor（圆盘物料颜色）。
        const int progress = roundProgress;  // 将 roundProgress 写入或初始化 progress。
        if (!serialDebugMode  // 当 !serialDebugMode 成立时进入此分支，否则继续后续流程。
                && (currentState == STATE_GRAB_ROUND1 || currentState == STATE_GRAB_ROUND2)  // 继续上一行的组合条件，本行提供 && (currentState == STATE_GRAB_ROUND1 || currentState == STATE_GRAB_ROUND2)。
                && discMaterialActive && progress >= 0 && progress < 3) {  // 继续上一行的组合条件，本行提供 && discMaterialActive && progress >= 0 && progress < 3) {。
            firstDiscGrabReady = true;  // 将 true 写入或初始化 firstDiscGrabReady（首个圆盘Grab就绪）。
            Serial.printf("[DISC] Material handling resumed: expected=%d, cargo=%d\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                          (currentState == STATE_GRAB_ROUND2 ? currentTask.round2_colors  // 为当前列表或跨行表达式提供这一项 (currentState == STATE_GRAB_ROUND2 ? currentTask.round…。
                                                               : currentTask.round1_colors)[progress], CargoForGrabIndex(progress));  // 调用 CargoForGrabIndex（货位ForGrab下标），传入 progress)。
        }  // 结束此初始化列表或局部作用域。
    };  // 结束此初始化列表或局部作用域。
    for (;;) {  // 按 ;; 组织循环的初值、继续条件和步进。
        resumeDiscAtFrameBoundary();  // 调用 resumeDiscAtFrameBoundary（resume圆盘At帧Boundary），使用无参形式推进相应操作。
        while (Serial.available() > 0) {  // 当 Serial.available() > 0 成立时继续循环。
            // 收帧/分发期间不是空闲；同时取消尚未处理的按键请求。
            serialBootSetIdle(false);  // 调用 serialBootSetIdle，传入 false。
            // 请求可能在读取持续数据期间到达；先记录旧缓存再读下一字节。
            resumeDiscAtFrameBoundary();  // 调用 resumeDiscAtFrameBoundary（resume圆盘At帧Boundary），使用无参形式推进相应操作。
            const char c = static_cast<char>(Serial.read());  // 将 static_cast<char>(Serial.read()) 写入或初始化 c。
            if (discResumeDraining && discOldBytesRemaining > 0) --discOldBytesRemaining;  // 当 discResumeDraining && discOldBytesRemaining > 0 成立时进入此分支，否则继续后续流程。
            if (c == '\n' || c == '\r') {  // 当 c == '\n' || c == '\r' 成立时进入此分支，否则继续后续流程。
                // 2. 完整行到达后才解析。过长或含 NUL 的整行直接拒绝。
                if (overflow) Serial.println("{RSP,SYS,FRAME,ERR,TOO_LONG}");  // 当 overflow 成立时进入此分支，否则继续后续流程。
                else if (invalid) Serial.println("{RSP,SYS,FRAME,ERR,FORMAT}");  // 仅在先前条件均不成立时，当 invalid 成立时进入此分支，否则继续后续流程。
                else if (length > 0) {  // 仅在先前条件均不成立时，当 length > 0 成立时进入此分支，否则继续后续流程。
                    line[length] = '\0';  // 将 '\0' 写入或初始化 line[length]。
                    SerialFrame frame;  // 声明 frame（帧），类型和可用范围由所在声明决定。
                    // 3. 分发层只看到已经通过基础结构检查的 CMD 帧。
                    if (parseSerialFrame(line, frame)) serialDispatch(frame);  // 当 parseSerialFrame(line, frame) 成立时进入此分支，否则继续后续流程。
                    else Serial.println("{RSP,SYS,FRAME,ERR,FORMAT}");  // 前一个判断不成立时执行此分支。
                }  // 结束此条件分支。
                length = 0;  // 将 0 写入或初始化 length（长度）。
                overflow = false;  // 将 false 写入或初始化 overflow。
                invalid = false;  // 将 false 写入或初始化 invalid。
            } else if (c == '\0') {  // 仅在先前条件均不成立时，当 c == '\0' 成立时进入此分支，否则继续后续流程。
                invalid = true;  // 将 true 写入或初始化 invalid。
            } else if (!overflow) {  // 仅在先前条件均不成立时，当 !overflow 成立时进入此分支，否则继续后续流程。
                if (length < sizeof(line) - 1) line[length++] = c;  // 当 length < sizeof(line) - 1 成立时进入此分支，否则继续后续流程。
                else overflow = true; // 丢弃整行剩余字节，不执行截断后的命令。
            }  // 结束此条件分支。
        }  // 结束此循环体。
        // 无新数据时也可以恢复；持续数据时由上面的帧边界检查恢复。
        resumeDiscAtFrameBoundary();  // 调用 resumeDiscAtFrameBoundary（resume圆盘At帧Boundary），使用无参形式推进相应操作。
        serialBootReleaseAtIdle(length == 0 && !overflow && !invalid && Serial.available() == 0);  // 调用 serialBootReleaseAtIdle，传入 length == 0 && !overflow && !invalid && Serial.availab…。
        vTaskDelay(pdMS_TO_TICKS(10));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    }  // 结束此循环体。
}  // 结束此函数或控制块。
