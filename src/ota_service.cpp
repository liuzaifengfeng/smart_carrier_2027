#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include "ota_service.h"

static bool otaStarted = false;
static bool wifiConnected = false;
static const char* wifi_ssid = nullptr;
static const char* wifi_password = nullptr;
static const char* wifi_hostname = nullptr;

static TaskHandle_t xOtaTaskHandle = NULL;

// 声明外部需要暂停的业务任务句柄
extern TaskHandle_t xTask_MainStateMachine_Handle;

static void ota_task(void *pvParameters) {
    const char* hostname = (const char*)pvParameters;

    MDNS.begin(hostname);
    Serial.printf("mDNS responder started: %s.local\n", hostname);

    ArduinoOTA.setHostname(hostname);

    // OTA 开始：挂起业务任务，防止总线竞争和中断干扰
    ArduinoOTA.onStart([]() {
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
        Serial.printf("Error[%u]: ", error);
        if (xTask_MainStateMachine_Handle != NULL) {
            vTaskResume(xTask_MainStateMachine_Handle);
        }
    });

    ArduinoOTA.begin();
    Serial.println("OTA service started");

    for (;;) {
        ArduinoOTA.handle();
        vTaskDelay(pdMS_TO_TICKS(2)); // 缩短轮询延时，加快吞吐
    }
}

static void wifi_task(void *pvParameters) {
    (void)pvParameters;

    WiFi.mode(WIFI_STA);
    for (;;) {
        if (WiFi.status() == WL_CONNECTED) {
            if (!wifiConnected) {
                wifiConnected = true;
                Serial.printf("WiFi Connected. IP: %s\n", WiFi.localIP().toString().c_str());

                if (!otaStarted && wifi_hostname != nullptr) {
                    otaStarted = true;
                    // 优先级提到 10（高于主状态机的 8），并绑定到 Core 0 与网络栈同核
                    xTaskCreatePinnedToCore(
                        ota_task, 
                        "OTA_Task", 
                        16384, 
                        (void*)wifi_hostname, 
                        10, 
                        &xOtaTaskHandle, 
                        0
                    );
                }
            }
        } else {
            if (wifiConnected) {
                wifiConnected = false;
                Serial.println("WiFi Disconnected");

                if (xOtaTaskHandle != NULL) {
                    vTaskDelete(xOtaTaskHandle);
                    xOtaTaskHandle = NULL;
                }
                otaStarted = false;
            }

            if (wifi_ssid != nullptr && wifi_password != nullptr) {
                if (WiFi.status() != WL_CONNECTED) {
                    Serial.println("try connecting to WiFi... " + String(wifi_ssid) + " " + String(wifi_password));
                    WiFi.begin(wifi_ssid, wifi_password);
                }
                vTaskDelay(pdMS_TO_TICKS(10000));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

void init_ota_service(const char* ssid, const char* password, const char* hostname) {
    wifi_ssid = ssid;
    wifi_password = password;
    wifi_hostname = hostname;

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.println("WiFi connection started in background");

    xTaskCreate(wifi_task, "WiFi_Task", 8192, NULL, 3, NULL);
}