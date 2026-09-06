#ifndef SCANNER_H
#define SCANNER_H

#include <Arduino.h>
#include <FreeRTOS.h>
#include <queue.h>

/*******************************************************************************
 * 扫码模块 (条码/二维码扫码枪) 驱动
 *
 * 硬件说明:
 *  - 通信方式: 软串口 (SoftwareSerial), 仅接收 (RX)
 *  - 默认引脚: IO4 (GPIO4)
 *  - 波特率:   9600 (ASCII 明文, 每帧以换行/回车结尾)
 *
 * 工作流程:
 *  - Scanner_Init() 配置软串口 + 创建消息队列 + 启动监听任务
 *  - 监听任务按行解析收到的字符串, 打包成 ScannerMsg_t 发送到消息队列
 *  - 主状态机通过 Scanner_WaitCode() 阻塞等待(或非阻塞)取出一帧扫码数据
 ******************************************************************************/

// 扫码枪软串口配置 (可在调用 Scanner_Init 时覆盖)
#ifndef SCANNER_RX_PIN
#define SCANNER_RX_PIN 4          // 软串口 RX = IO4
#endif

#ifndef SCANNER_BAUDRATE
#define SCANNER_BAUDRATE 9600     // 扫码枪默认波特率
#endif

// 单帧扫码字符串最大长度 (含结尾 '\0')
#define SCANNER_BUF_LEN 64

// 消息队列深度
#define SCANNER_QUEUE_LEN 8

// 消息队列句柄 (主状态机通过它 / Scanner_WaitCode 读取)
extern QueueHandle_t xScannerQueue;

// 队列元素: 一帧扫码字符串 (ASCII, 不含换行回车)
typedef struct {
    char data[SCANNER_BUF_LEN];
} ScannerMsg_t;

/**
 * @brief 扫码模块初始化
 *        - 配置软串口 (默认 RX=IO4, 9600bps)
 *        - 创建扫码消息队列
 *        - 启动监听任务 Task_ScannerListen
 * @param rxPin   软串口接收引脚 (默认 IO4)
 * @param baudrate 波特率 (默认 9600)
 */
void Scanner_Init(int rxPin = SCANNER_RX_PIN, uint32_t baudrate = SCANNER_BAUDRATE);

/**
 * @brief 等待一帧扫码数据 (伪函数, 供主状态机调用)
 * @param out       输出缓冲区
 * @param len       输出缓冲区长度 (字节)
 * @param timeoutMs 阻塞超时时间(ms); 0 表示非阻塞, portMAX_DELAY 表示永久阻塞
 * @return true 拿到一帧数据, false 超时/无数据
 */
bool Scanner_WaitCode(char *out, uint32_t len, TickType_t timeoutMs);

#endif // SCANNER_H
