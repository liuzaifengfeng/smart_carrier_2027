#include "scanner.h"
#include <SoftwareSerial.h>

// 软串口实例 (仅接收, 不发送)
static SoftwareSerial *s_scannerSerial = nullptr;

// 扫码消息队列
QueueHandle_t xScannerQueue = NULL;

// 监听任务句柄
static TaskHandle_t s_scannerTaskHandle = NULL;

/**
 * @brief 扫码监听任务
 *        循环读取软串口字节, 按换行('\n')/回车('\r')切分一帧字符串,
 *        打包后发送到扫码消息队列。
 */
static void Task_ScannerListen(void *pvParameters) {
    char rxBuf[SCANNER_BUF_LEN];
    int rxIdx = 0;

    for (;;) {
        if (s_scannerSerial != nullptr) {
            while (s_scannerSerial->available() > 0) {
                char c = s_scannerSerial->read();

                if (c == '\n' || c == '\r') {
                    // 一帧结束
                    rxBuf[rxIdx] = '\0';
                    if (rxIdx > 0) {
                        ScannerMsg_t msg;
                        memset(msg.data, 0, sizeof(msg.data));
                        strncpy(msg.data, rxBuf, SCANNER_BUF_LEN - 1);
                        if (xScannerQueue != NULL) {
                            // 队列满则丢弃最旧一帧, 保证最新数据可用
                            xQueueSend(xScannerQueue, &msg, 0);
                        }
                    }
                    rxIdx = 0;
                } else if (rxIdx < SCANNER_BUF_LEN - 1) {
                    rxBuf[rxIdx++] = c;
                } else {
                    // 超长, 丢弃并重置, 防止越界
                    rxIdx = 0;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief 初始化扫码模块
 */
void Scanner_Init(int rxPin, uint32_t baudrate) {
    // 配置软串口 (仅接收: TX 传 -1)
    if (s_scannerSerial != nullptr) {
        delete s_scannerSerial;
        s_scannerSerial = nullptr;
    }
    s_scannerSerial = new SoftwareSerial(rxPin, -1);
    s_scannerSerial->begin(baudrate);

    // 创建扫码消息队列 (若已存在则先删除重建)
    if (xScannerQueue != NULL) {
        vQueueDelete(xScannerQueue);
        xScannerQueue = NULL;
    }
    xScannerQueue = xQueueCreate(SCANNER_QUEUE_LEN, sizeof(ScannerMsg_t));

    // 启动监听任务
    if (s_scannerTaskHandle != NULL) {
        vTaskDelete(s_scannerTaskHandle);
        s_scannerTaskHandle = NULL;
    }
    xTaskCreate(Task_ScannerListen, "Task_Scanner", 4096, NULL, 6, &s_scannerTaskHandle);

    Serial.printf("[Scanner] SoftSerial RX=%d @ %lu bps, queue len=%d ready\n",
                  rxPin, (unsigned long)baudrate, SCANNER_QUEUE_LEN);
}

/**
 * @brief 等待一帧扫码数据 (伪函数)
 */
bool Scanner_WaitCode(char *out, uint32_t len, TickType_t timeoutMs) {
    if (out == nullptr || len == 0 || xScannerQueue == NULL) {
        return false;
    }

    ScannerMsg_t msg;
    if (xQueueReceive(xScannerQueue, &msg, timeoutMs) == pdPASS) {
        strncpy(out, msg.data, len - 1);
        out[len - 1] = '\0';
        return true;
    }
    return false;
}
