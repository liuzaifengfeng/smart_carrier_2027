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

static void ota_task(void *pvParameters) {
    const char* hostname = (const char*)pvParameters;

    MDNS.begin(hostname);
    Serial.printf("mDNS responder started: %s.local\n", hostname);

    ArduinoOTA.setHostname(hostname);

    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
    });

    ArduinoOTA.begin();
    Serial.println("OTA service started");

    for (;;) {
        ArduinoOTA.handle();
        vTaskDelay(pdMS_TO_TICKS(5));
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
                    xTaskCreate(ota_task, "OTA_Task", 16384, (void*)wifi_hostname, 6, &xOtaTaskHandle);
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
            vTaskDelay(pdMS_TO_TICKS(10000));//未连接，等待10秒后重试连接
            }
        }
        vTaskDelay(pdMS_TO_TICKS(30000));//已连接，等待每30秒检查一次WiFi连接状态
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