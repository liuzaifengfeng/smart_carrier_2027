#ifndef OTA_SERVICE_H
#define OTA_SERVICE_H

// 初始化 OTA 服务（包含 WiFi 连接、mDNS 和 OTA 任务启动）
void init_ota_service(const char* ssid, const char* password, const char* hostname);

#endif