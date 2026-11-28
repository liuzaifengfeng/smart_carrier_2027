// OTA 独立任务负责网络接入和更新回调。
// 开始和结束事件会通知运动执行层，维护任务自身由 FreeRTOS 调度。

#include <WiFi.h>  // 引入 WiFi.h 中的声明和定义。
#include <ArduinoOTA.h>  // 引入 ArduinoOTA.h 中的声明和定义。
#include "ota_service.h"  // 引入 ota_service.h 中的声明和定义。
#include "Emm_V5.h"  // 引入 Emm_V5.h 中的声明和定义。
#include "chassis.h"  // 引入 chassis.h 中的声明和定义。

#include "continuous_trial.h"  // 引入 continuous_trial.h 中的声明和定义。
#include "motion_executor.h"  // 引入 motion_executor.h 中的声明和定义。

static TaskHandle_t otaTask = nullptr;  // 将 nullptr 写入或初始化 otaTask（固件更新任务）。
static constexpr uint32_t wifiDisconnected = 1;  // 将 1 写入或初始化 wifiDisconnected。
static constexpr uint32_t wifiGotIp = 2;  // 将 2 写入或初始化 wifiGotIp。

// 声明外部需要暂停的业务任务句柄
extern TaskHandle_t xTask_MainStateMachine_Handle;  // 声明外部定义 TaskHandle_t xTask_MainStateMachine_Handle，供本编译单元引用。

static void ota_task(void *pvParameters) {  // 定义函数或成员 的入口 ota_task（固件更新任务），参数由此处的签名确定。
    const char* hostname = (const char*)pvParameters;  // 将 (const char*)pvParameters 写入或初始化 hostname。

    ArduinoOTA.setHostname(hostname);  // 调用 ArduinoOTA.setHostname，传入 hostname。
    static bool uploadStarted = false;  // 将 false 写入或初始化 uploadStarted。

    // OTA 开始：挂起业务任务，防止总线竞争和中断干扰
    ArduinoOTA.onStart([]() {  // 调用 ArduinoOTA.onStart（Arduino固件更新on起点），传入 []() {。
        uploadStarted = true;  // 将 true 写入或初始化 uploadStarted。
        ContinuousTrialSetOta(true);  // 调用 ContinuousTrialSetOta（ContinuousTrialSet固件更新），传入 true。
        // 必须先确认四轮已由试跑任务停车；超时则重启前仍尝试直接停车。
        uint32_t stopAt=millis();  // 将 millis() 写入或初始化 stopAt（停止At）。
        while((MotionExecutorActive()||ContinuousTrialActive())&&millis()-stopAt<500)vTaskDelay(pdMS_TO_TICKS(10));  // 当 (MotionExecutorActive()||ContinuousTrialActive())&&mil… 成立时继续循环。
        if(MotionExecutorActive()||ContinuousTrialActive()){Emm_V5_Stop_Now(0,false);Serial1.flush();}  // 当 MotionExecutorActive()||ContinuousTrialActive() 成立时进入此分支，否则继续后续流程。
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";  // 将 (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "fil… 写入或初始化 type。
        Serial.println("Start updating " + type);  // 调用 Serial.println，传入 "Start updating " + type。

        // 挂起主状态机任务，独占 CPU 和总线
        if (xTask_MainStateMachine_Handle != NULL) {  // 当 xTask_MainStateMachine_Handle != NULL 成立时进入此分支，否则继续后续流程。
            vTaskSuspend(xTask_MainStateMachine_Handle);  // 调用 vTaskSuspend（v任务Suspend），传入 xTask_MainStateMachine_Handle。
        }  // 结束此条件分支。
    });  // 结束此函数或控制块。

    ArduinoOTA.onEnd([]() {   // 调用 ArduinoOTA.onEnd（Arduino固件更新on终点），传入 []() {。
        Serial.println("\nEnd");   // 调用 Serial.println，传入 "\nEnd"。
    });  // 结束此函数或控制块。

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {  // 调用 ArduinoOTA.onProgress（Arduino固件更新onProgress），传入 [](unsigned int progress, unsigned int total) {。
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));  // 调用 Serial.printf，传入 "Progress: %u%%\r", (progress / (total / 100))。
    });  // 结束此函数或控制块。

    // 出错时恢复业务
    ArduinoOTA.onError([](ota_error_t error) {  // 调用 ArduinoOTA.onError（Arduino固件更新on错误），传入 [](ota_error_t error) {。
        Serial.printf("[OTA] Error[%u]\n", error);  // 调用 Serial.printf，传入 "[OTA] Error[%u]\n", error。
        if (uploadStarted) {  // 当 uploadStarted 成立时进入此分支，否则继续后续流程。
            uploadStarted = false;  // 将 false 写入或初始化 uploadStarted。
            ContinuousTrialSetOta(false);  // 调用 ContinuousTrialSetOta（ContinuousTrialSet固件更新），传入 false。
            if (xTask_MainStateMachine_Handle != nullptr && !taskMotionAborted) {  // 当 xTask_MainStateMachine_Handle != nullptr && !taskMotio… 成立时进入此分支，否则继续后续流程。
                vTaskResume(xTask_MainStateMachine_Handle);  // 调用 vTaskResume（v任务Resume），传入 xTask_MainStateMachine_Handle。
            }  // 结束此条件分支。
        }  // 结束此条件分支。
    });  // 结束此函数或控制块。

    bool listening = false;  // 将 false 写入或初始化 listening。
    uint32_t wifiEvents = 0;  // 将 0 写入或初始化 wifiEvents。
    for (;;) {  // 按 ;; 组织循环的初值、继续条件和步进。
        const bool connected = WiFi.status() == WL_CONNECTED;  // 将 WiFi.status() == WL_CONNECTED 写入或初始化 connected。
        // 短暂断线后即使已经取回 IP，也要重建旧监听。上传返回后才处理事件。
        if (listening && (!connected || wifiEvents)) {  // 当 listening && (!connected || wifiEvents) 成立时进入此分支，否则继续后续流程。
            ArduinoOTA.end();  // 调用 ArduinoOTA.end。
            listening = false;  // 将 false 写入或初始化 listening。
        }  // 结束此条件分支。
        if (connected) {  // 当 connected 成立时进入此分支，否则继续后续流程。
            if (!listening) {  // 当 !listening 成立时进入此分支，否则继续后续流程。
                ArduinoOTA.begin();  // 调用 ArduinoOTA.begin。
                listening = true;  // 将 true 写入或初始化 listening。
                Serial.printf("[OTA] %s.local, IP: %s\n", hostname, WiFi.localIP().toString().c_str());  // 调用 Serial.printf，传入 "[OTA] %s.local, IP: %s\n", hostname, WiFi.localIP().t…。
            }  // 结束此条件分支。
            ArduinoOTA.handle();  // 调用 ArduinoOTA.handle。
        }  // 结束此条件分支。
        wifiEvents = 0;  // 将 0 写入或初始化 wifiEvents。
        if (xTaskNotifyWait(0, UINT32_MAX, &wifiEvents, pdMS_TO_TICKS(connected ? 2 : 10000)) == pdFALSE  // 当 xTaskNotifyWait(0, UINT32_MAX, &wifiEvents, pdMS_TO_TI… 成立时进入此分支，否则继续后续流程。
            && !connected && WiFi.status() != WL_CONNECTED) {  // 继续上一行的组合条件，本行提供 && !connected && WiFi.status() != WL_CONNECTED) {。
            WiFi.reconnect();  // 调用 WiFi.reconnect。
        }  // 结束此函数或控制块。
    }  // 结束此循环体。
}  // 结束此函数或控制块。

/**
 * @brief 初始化WiFi、mDNS及OTA后台任务；ssid、password为网络凭据，hostname为设备名称。
 */
void init_ota_service(const char* ssid, const char* password, const char* hostname) {  // 定义函数或成员 的入口 init_ota_service（init固件更新service），参数由此处的签名确定。
    WiFi.mode(WIFI_STA);  // 调用 WiFi.mode，传入 WIFI_STA。
    if (xTaskCreatePinnedToCore(ota_task, "OTA_Task", 16384, (void*)hostname, 10, &otaTask, 0) != pdPASS) {  // 当 xTaskCreatePinnedToCore(ota_task, "OTA_Task", 16384, (… 成立时进入此分支，否则继续后续流程。
        Serial.println("[OTA] task creation failed");  // 调用 Serial.println，传入 "[OTA] task creation failed"。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    WiFi.onEvent([](arduino_event_id_t event) {  // 调用 WiFi.onEvent，传入 [](arduino_event_id_t event) {。
        if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {  // 当 event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED 成立时进入此分支，否则继续后续流程。
            xTaskNotify(otaTask, wifiDisconnected, eSetBits);  // 调用 xTaskNotify（横向任务Notify），传入 otaTask, wifiDisconnected, eSetBits。
        } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {  // 仅在先前条件均不成立时，当 event == ARDUINO_EVENT_WIFI_STA_GOT_IP 成立时进入此分支，否则继续后续流程。
            xTaskNotify(otaTask, wifiGotIp, eSetBits);  // 调用 xTaskNotify（横向任务Notify），传入 otaTask, wifiGotIp, eSetBits。
        }  // 结束此条件分支。
    });  // 结束此函数或控制块。
    WiFi.begin(ssid, password);  // 调用 WiFi.begin，传入 ssid, password。
    vTaskDelay(pdMS_TO_TICKS(1000));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    Serial.println("WiFi connection started in background");  // 调用 Serial.println，传入 "WiFi connection started in background"。
}  // 结束此函数或控制块。
