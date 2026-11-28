// 扫码模块的串口初始化和等待接口。
// 等待端提供输出缓冲区和超时，收到的内容再由任务码解析层解释。

#ifndef SCANNER_H  // 宏 SCANNER_H 未定义时纳入此分支。
#define SCANNER_H  // 为后续文本展开定义宏 SCANNER_H。

#include <Arduino.h>  // 引入 Arduino.h 中的声明和定义。
#include <FreeRTOS.h>  // 引入 FreeRTOS.h 中的声明和定义。
#include <queue.h>  // 引入 queue.h 中的声明和定义。

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
 *  - 运动任务通过 pollTaskCode() 非阻塞取出扫码数据，主状态机只提交扫码作业
 ******************************************************************************/

// 扫码枪软串口配置 (可在调用 Scanner_Init 时覆盖)
#ifndef SCANNER_RX_PIN  // 宏 SCANNER_RX_PIN 未定义时纳入此分支。
#define SCANNER_RX_PIN 4          // 软串口 RX = IO4
#endif  // 结束本组条件编译。

#ifndef SCANNER_BAUDRATE  // 宏 SCANNER_BAUDRATE 未定义时纳入此分支。
#define SCANNER_BAUDRATE 9600     // 扫码枪默认波特率
#endif  // 结束本组条件编译。

// 单帧扫码字符串最大长度 (含结尾 '\0')
#define SCANNER_BUF_LEN 64  // 为后续文本展开定义宏 SCANNER_BUF_LEN 64。

// 消息队列深度
#define SCANNER_QUEUE_LEN 8  // 为后续文本展开定义宏 SCANNER_QUEUE_LEN 8。

// 消息队列句柄 (主状态机通过它 / Scanner_WaitCode 读取)
extern QueueHandle_t xScannerQueue;  // 声明外部定义 QueueHandle_t xScannerQueue，供本编译单元引用。

// 队列元素: 一帧扫码字符串 (ASCII, 不含换行回车)
typedef struct {  // 声明 struct，类型和可用范围由所在声明决定。
    char data[SCANNER_BUF_LEN];  // 声明 data（数据），类型和可用范围由所在声明决定。
} ScannerMsg_t;  // 续写当前声明或运算，提供 } ScannerMsg_t;。

/**
 * @brief 扫码模块初始化
 *        - 配置软串口 (默认 RX=IO4, 9600bps)
 *        - 创建扫码消息队列
 *        - 启动监听任务 Task_ScannerListen
 * @param rxPin   软串口接收引脚 (默认 IO4)
 * @param baudrate 波特率 (默认 9600)
 */
void Scanner_Init(int rxPin = SCANNER_RX_PIN, uint32_t baudrate = SCANNER_BAUDRATE);  // 声明接口 Scanner_Init（扫码器Init），参数由此处的签名确定。

/**
 * @brief 从扫码枪消息队列读取一帧字符串；由 waitScannerCode 包装为毫秒接口。
 * @param out       输出缓冲区
 * @param len       输出缓冲区长度 (字节)
 * @param timeoutMs 实际为 FreeRTOS Tick 数；0 立即查询，portMAX_DELAY 永久等待。
 * @return true 拿到一帧数据, false 超时/无数据
 */
bool Scanner_WaitCode(char *out, uint32_t len, TickType_t timeoutMs);  // 声明接口 Scanner_WaitCode（扫码器WaitCode），参数由此处的签名确定。

#endif // SCANNER_H
