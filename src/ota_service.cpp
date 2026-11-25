#include <WiFi.h>
#include <ArduinoOTA.h>
#include "ota_service.h"
#include "Emm_V5.h"
#include "chassis.h"

#include "continuous_trial.h"
#include "motion_executor.h"

static TaskHandle_t otaTask = nullptr;
static constexpr uint32_t wifiDisconnected = 1;
static constexpr uint32_t wifiGotIp = 2;

// 声明外部需要暂停的业务任务句柄
extern TaskHandle_t xTask_MainStateMachine_Handle;

static void ota_task(void *pvParameters) {
    const char* hostname = (const char*)pvParameters;

    ArduinoOTA.setHostname(hostname);
    static bool uploadStarted = false;

    // OTA 开始：挂起业务任务，防止总线竞争和中断干扰
    ArduinoOTA.onStart([]() {
        uploadStarted = true;
        ContinuousTrialSetOta(true);
        // 必须先确认四轮已由试跑任务停车；超时则重启前仍尝试直接停车。
        uint32_t stopAt=millis();
        while((MotionExecutorActive()||ContinuousTrialActive())&&millis()-stopAt<500)vTaskDelay(pdMS_TO_TICKS(10));
        if(MotionExecutorActive()||ContinuousTrialActive()){Emm_V5_Stop_Now(0,false);Serial1.flush();}
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        Serial.println("Start updating " + type);

        // 挂起主状态机任务，独占 CPU 和总线
        if (xTask_MainStateMachine_Handle != NULL) {
            vTaskSuspend(xTask_MainStateMachine_Handle);
        }
    });

    ArduinoOTA.onEnd([]() { 
        Serial.println("\nEnd"); 
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });

    // 出错时恢复业务
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[OTA] Error[%u]\n", error);
        if (uploadStarted) {
            uploadStarted = false;
            ContinuousTrialSetOta(false);
            if (xTask_MainStateMachine_Handle != nullptr && !taskMotionAborted) {
                vTaskResume(xTask_MainStateMachine_Handle);
            }
        }
    });

    bool listening = false;
    uint32_t wifiEvents = 0;
    for (;;) {
        const bool connected = WiFi.status() == WL_CONNECTED;
        // 短暂断线后即使已经取回 IP，也要重建旧监听。上传返回后才处理事件。
        if (listening && (!connected || wifiEvents)) {
            ArduinoOTA.end();
            listening = false;
        }
        if (connected) {
            if (!listening) {
                ArduinoOTA.begin();
                listening = true;
                Serial.printf("[OTA] %s.local, IP: %s\n", hostname, WiFi.localIP().toString().c_str());
            }
            ArduinoOTA.handle();
        }
        wifiEvents = 0;
        if (xTaskNotifyWait(0, UINT32_MAX, &wifiEvents, pdMS_TO_TICKS(connected ? 2 : 10000)) == pdFALSE
            && !connected && WiFi.status() != WL_CONNECTED) {
            WiFi.reconnect();
        }
    }
}

/**
 * @brief 初始化WiFi、mDNS及OTA后台任务；ssid、password为网络凭据，hostname为设备名称。
 */
void init_ota_service(const char* ssid, const char* password, const char* hostname) {
    WiFi.mode(WIFI_STA);
    if (xTaskCreatePinnedToCore(ota_task, "OTA_Task", 16384, (void*)hostname, 10, &otaTask, 0) != pdPASS) {
        Serial.println("[OTA] task creation failed");
        return;
    }
    WiFi.onEvent([](arduino_event_id_t event) {
        if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
            xTaskNotify(otaTask, wifiDisconnected, eSetBits);
        } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
            xTaskNotify(otaTask, wifiGotIp, eSetBits);
        }
    });
    WiFi.begin(ssid, password);
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.println("WiFi connection started in background");
}
