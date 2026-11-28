// 扫码串口的接收任务。
// 监听端组装任务码并交给等待端，业务任务通过队列接口取得扫码结果。

#include "scanner.h"  // 引入 scanner.h 中的声明和定义。
#include <SoftwareSerial.h>  // 引入 SoftwareSerial.h 中的声明和定义。

// 软串口实例 (仅接收, 不发送)
static SoftwareSerial *s_scannerSerial = nullptr;  // 将 nullptr 写入或初始化 s_scannerSerial（s扫码器Serial）。

// 扫码消息队列
QueueHandle_t xScannerQueue = NULL;  // 将 NULL 写入或初始化 xScannerQueue（横向扫码器Queue）。

// 监听任务句柄
static TaskHandle_t s_scannerTaskHandle = NULL;  // 将 NULL 写入或初始化 s_scannerTaskHandle（s扫码器任务Handle）。

/**
 * @brief 扫码监听任务
 *        循环读取软串口字节, 按换行('\n')/回车('\r')切分一帧字符串,
 *        打包后发送到扫码消息队列。
 */
static void Task_ScannerListen(void *pvParameters) {  // 定义函数或成员 的入口 Task_ScannerListen（任务扫码器Listen），参数由此处的签名确定。
    char rxBuf[SCANNER_BUF_LEN];  // 声明 rxBuf（接收Buf），类型和可用范围由所在声明决定。
    int rxIdx = 0;  // 将 0 写入或初始化 rxIdx（接收下标）。

    for (;;) {  // 按 ;; 组织循环的初值、继续条件和步进。
        if (s_scannerSerial != nullptr) {  // 当 s_scannerSerial != nullptr 成立时进入此分支，否则继续后续流程。
            while (s_scannerSerial->available() > 0) {  // 当 s_scannerSerial->available() > 0 成立时继续循环。
                char c = s_scannerSerial->read();  // 将 s_scannerSerial->read() 写入或初始化 c。

                if (c == '\n' || c == '\r') {  // 当 c == '\n' || c == '\r' 成立时进入此分支，否则继续后续流程。
                    // 一帧结束
                    rxBuf[rxIdx] = '\0';  // 将 '\0' 写入或初始化 rxBuf[rxIdx]。
                    if (rxIdx > 0) {  // 当 rxIdx > 0 成立时进入此分支，否则继续后续流程。
                        ScannerMsg_t msg;  // 声明 msg，类型和可用范围由所在声明决定。
                        memset(msg.data, 0, sizeof(msg.data));  // 调用 memset，传入 msg.data, 0, sizeof(msg.data)。
                        strncpy(msg.data, rxBuf, SCANNER_BUF_LEN - 1);  // 调用 strncpy，传入 msg.data, rxBuf, SCANNER_BUF_LEN - 1。
                        if (xScannerQueue != NULL) {  // 当 xScannerQueue != NULL 成立时进入此分支，否则继续后续流程。
                            // 以零等待投递。队列满时本次发送失败，已排队的旧帧继续等待消费。
                            xQueueSend(xScannerQueue, &msg, 0);  // 调用 xQueueSend（横向QueueSend），传入 xScannerQueue, &msg, 0。
                        }  // 结束此条件分支。
                    }  // 结束此条件分支。
                    rxIdx = 0;  // 将 0 写入或初始化 rxIdx（接收下标）。
                } else if (rxIdx < SCANNER_BUF_LEN - 1) {  // 仅在先前条件均不成立时，当 rxIdx < SCANNER_BUF_LEN - 1 成立时进入此分支，否则继续后续流程。
                    rxBuf[rxIdx++] = c;  // 将 c 写入或初始化 rxBuf[rxIdx++]。
                } else {  // 前一个判断不成立时执行此分支。
                    // 超长, 丢弃并重置, 防止越界
                    rxIdx = 0;  // 将 0 写入或初始化 rxIdx（接收下标）。
                }  // 结束此条件分支。
            }  // 结束此循环体。
        }  // 结束此条件分支。
        vTaskDelay(pdMS_TO_TICKS(10));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    }  // 结束此循环体。
}  // 结束此函数或控制块。

/**
 * @brief 初始化扫码模块
 */
void Scanner_Init(int rxPin, uint32_t baudrate) {  // 定义函数或成员 的入口 Scanner_Init（扫码器Init），参数由此处的签名确定。
    // 配置软串口 (仅接收: TX 传 -1)
    if (s_scannerSerial != nullptr) {  // 当 s_scannerSerial != nullptr 成立时进入此分支，否则继续后续流程。
        delete s_scannerSerial;  // 声明 s_scannerSerial（s扫码器Serial），类型和可用范围由所在声明决定。
        s_scannerSerial = nullptr;  // 将 nullptr 写入或初始化 s_scannerSerial（s扫码器Serial）。
    }  // 结束此条件分支。
    s_scannerSerial = new SoftwareSerial(rxPin, -1);  // 将 new SoftwareSerial(rxPin, -1) 写入或初始化 s_scannerSerial（s扫码器Serial）。
    s_scannerSerial->begin(baudrate);  // 调用 s_scannerSerial->begin，传入 baudrate。

    // 创建扫码消息队列 (若已存在则先删除重建)
    if (xScannerQueue != NULL) {  // 当 xScannerQueue != NULL 成立时进入此分支，否则继续后续流程。
        vQueueDelete(xScannerQueue);  // 调用 vQueueDelete，传入 xScannerQueue。
        xScannerQueue = NULL;  // 将 NULL 写入或初始化 xScannerQueue（横向扫码器Queue）。
    }  // 结束此条件分支。
    xScannerQueue = xQueueCreate(SCANNER_QUEUE_LEN, sizeof(ScannerMsg_t));  // 将 xQueueCreate(SCANNER_QUEUE_LEN, sizeof(ScannerMsg_t)) 写入或初始化 xScannerQueue（横向扫码器Queue）。

    // 启动监听任务
    if (s_scannerTaskHandle != NULL) {  // 当 s_scannerTaskHandle != NULL 成立时进入此分支，否则继续后续流程。
        vTaskDelete(s_scannerTaskHandle);  // 调用 vTaskDelete（v任务Delete），传入 s_scannerTaskHandle。
        s_scannerTaskHandle = NULL;  // 将 NULL 写入或初始化 s_scannerTaskHandle（s扫码器任务Handle）。
    }  // 结束此条件分支。
    xTaskCreate(Task_ScannerListen, "Task_Scanner", 4096, NULL, 6, &s_scannerTaskHandle);  // 调用 xTaskCreate（横向任务Create），传入 Task_ScannerListen, "Task_Scanner", 4096, NULL, 6, &s_…。

    Serial.printf("[Scanner] SoftSerial RX=%d @ %lu bps, queue len=%d ready\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                  rxPin, (unsigned long)baudrate, SCANNER_QUEUE_LEN);  // 续写当前声明或运算，提供 rxPin, (unsigned long)baudrate, SCANNER_QUEUE_LEN);。
}  // 结束此函数或控制块。

/**
 * @brief 从扫码队列读取一帧字符串，timeoutMs 实际为 FreeRTOS Tick 数；0 立即查询。
 */
bool Scanner_WaitCode(char *out, uint32_t len, TickType_t timeoutMs) {  // 定义函数或成员 的入口 Scanner_WaitCode（扫码器WaitCode），参数由此处的签名确定。
    if (out == nullptr || len == 0 || xScannerQueue == NULL) {  // 当 out == nullptr || len == 0 || xScannerQueue == NULL 成立时进入此分支，否则继续后续流程。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。

    ScannerMsg_t msg;  // 声明 msg，类型和可用范围由所在声明决定。
    if (xQueueReceive(xScannerQueue, &msg, timeoutMs) == pdPASS) {  // 当 xQueueReceive(xScannerQueue, &msg, timeoutMs) == pdPASS 成立时进入此分支，否则继续后续流程。
        strncpy(out, msg.data, len - 1);  // 调用 strncpy，传入 out, msg.data, len - 1。
        out[len - 1] = '\0';  // 将 '\0' 写入或初始化 out[len - 1]。
        return true;  // 结束当前函数并返回 true。
    }  // 结束此条件分支。
    return false;  // 结束当前函数并返回 false。
}  // 结束此函数或控制块。
