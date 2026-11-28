// OTA 启动入口。
// 调用者提供网络和主机名，具体连接及维护循环由实现中的任务负责。

#ifndef OTA_SERVICE_H  // 宏 OTA_SERVICE_H 未定义时纳入此分支。
#define OTA_SERVICE_H  // 为后续文本展开定义宏 OTA_SERVICE_H。

// 初始化 OTA 服务（包含 WiFi 连接、mDNS 和 OTA 任务启动）
/**
 * @brief 初始化WiFi、mDNS及OTA后台任务；ssid、password为网络凭据，hostname为设备名称。
 */
void init_ota_service(const char* ssid, const char* password, const char* hostname);  // 声明接口 init_ota_service（init固件更新service），参数由此处的签名确定。

#endif  // 结束本组条件编译。