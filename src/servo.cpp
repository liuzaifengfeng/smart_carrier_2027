// 总线舵机协议实现。
// 收发封包共用串口，具体命令分别编码普通角度、多圈角度、阻尼或参数操作。

#include "servo.h"  // 引入 servo.h 中的声明和定义。
#include <freertos/FreeRTOS.h>  // 引入 freertos/FreeRTOS.h 中的声明和定义。
#include <freertos/semphr.h>  // 引入 freertos/semphr.h 中的声明和定义。

// 查询期间保护“发送请求 + 接收回复”，防止其他任务的控制指令清空回复。
// sendPacket 也取同一个递归锁，因此查询内部再次调用发送函数不会死锁。
static SemaphoreHandle_t s_servoMutex = nullptr;  // 将 nullptr 写入或初始化 s_servoMutex。
class ServoBusGuard {  // 定义 ServoBusGuard（Servo总线Guard） 的类型成员或作用域。
public:  // 此后的成员采用 public 访问权限，直到下一个权限标签。
    ServoBusGuard() : locked(s_servoMutex != nullptr  // 调用 ServoBusGuard（Servo总线Guard），传入 ) : locked(s_servoMutex != nullptr。
        && xSemaphoreTakeRecursive(s_servoMutex, portMAX_DELAY) == pdTRUE) {}  // 继续上一行的组合条件，本行提供 && xSemaphoreTakeRecursive(s_servoMutex, portMAX_DELAY) == pdTRUE) {}。
    ~ServoBusGuard() { if (locked) xSemaphoreGiveRecursive(s_servoMutex); }  // 定义函数或成员 ServoBusGuard（Servo总线Guard），参数由此处的签名确定。
    bool locked;  // 声明 locked，类型和可用范围由所在声明决定。
};  // 结束此类型定义。

// 内部串口指针，默认指向 Serial2
static HardwareSerial *s_servoSerial = &Serial2;  // 将 &Serial2 写入或初始化 s_servoSerial。

/**
 * @brief 发送完整的数据帧给总线舵机
 * 数据包协议结构:
 * Byte 0: Header Low Byte  (0x12)
 * Byte 1: Header High Byte (0x4C)
 * Byte 2: CmdId
 * Byte 3: Content Size
 * Byte 4 .. 4+ContentSize-1: Content
 * Byte Last: Checksum = (0x12 + 0x4C + CmdId + ContentSize + Sum(Content)) % 256
 */
// 收发共用同一个协议帧格式。
// 定位角度不一致时，先核对各个命令装入 payload 的单位和字节顺序，再检查此层的帧头和校验。
static void sendPacket(uint8_t cmdId, const uint8_t *content, uint8_t contentSize, uint16_t header = FSUS_PACK_REQ_HEADER) {  // 定义函数或成员 的入口 sendPacket，参数由此处的签名确定。
    ServoBusGuard guard;  // 声明 guard，类型和可用范围由所在声明决定。
    if (!guard.locked) return;  // 当 !guard.locked 成立时进入此分支，否则继续后续流程。

    uint8_t headerLow = header & 0xFF;  // 将 header & 0xFF 写入或初始化 headerLow。
    uint8_t headerHigh = (header >> 8) & 0xFF;  // 将 (header >> 8) & 0xFF 写入或初始化 headerHigh。

    uint16_t sum = headerLow + headerHigh + cmdId + contentSize;  // 将 headerLow + headerHigh + cmdId + contentSize 写入或初始化 sum。
    for (uint8_t i = 0; i < contentSize; i++) {  // 按 uint8_t i = 0; i < contentSize; i++ 组织循环的初值、继续条件和步进。
        sum += content[i];  // 把 content[i] 累加到 sum。
    }  // 结束此循环体。
    uint8_t checksum = sum % 256;  // 将 sum % 256 写入或初始化 checksum。

    // 清空接收缓冲区未处理的历史残留
    while (s_servoSerial->available() > 0) {  // 当 s_servoSerial->available() > 0 成立时继续循环。
        s_servoSerial->read();  // 调用 s_servoSerial->read。
    }  // 结束此循环体。

    s_servoSerial->write(headerLow);  // 调用 s_servoSerial->write，传入 headerLow。
    s_servoSerial->write(headerHigh);  // 调用 s_servoSerial->write，传入 headerHigh。
    s_servoSerial->write(cmdId);  // 调用 s_servoSerial->write，传入 cmdId。
    s_servoSerial->write(contentSize);  // 调用 s_servoSerial->write，传入 contentSize。
    if (contentSize > 0) {  // 当 contentSize > 0 成立时进入此分支，否则继续后续流程。
        s_servoSerial->write(content, contentSize);  // 调用 s_servoSerial->write，传入 content, contentSize。
    }  // 结束此条件分支。
    s_servoSerial->write(checksum);  // 调用 s_servoSerial->write，传入 checksum。
}  // 结束此初始化列表或局部作用域。

/**
 * @brief 接收应答帧 (针对带有返回的指令)
 * 应答帧头: 0x05, 0x1C (0x1C05)
 */
static uint8_t receivePacket(uint8_t expectedCmdId, uint8_t *contentBuf, uint8_t *contentSize, uint32_t timeoutMs) {  // 定义函数或成员 的入口 receivePacket，参数由此处的签名确定。
    uint32_t startMs = millis();  // 将 millis() 写入或初始化 startMs（起点毫秒）。
    uint8_t step = 0;  // 将 0 写入或初始化 step。
    uint8_t cmdId = 0;  // 将 0 写入或初始化 cmdId（命令编号）。
    uint8_t length = 0;  // 将 0 写入或初始化 length（长度）。
    uint8_t count = 0;  // 将 0 写入或初始化 count（计数）。
    uint8_t checksum = 0;  // 将 0 写入或初始化 checksum。
    uint16_t sum = 0;  // 将 0 写入或初始化 sum。

    while (millis() - startMs < timeoutMs) {  // 当 millis() - startMs < timeoutMs 成立时继续循环。
        if (s_servoSerial->available() > 0) {  // 当 s_servoSerial->available() > 0 成立时进入此分支，否则继续后续流程。
            uint8_t b = s_servoSerial->read();  // 将 s_servoSerial->read() 写入或初始化 b。

            switch (step) {  // 按 step 的值选择对应分支。
                case 0: // 匹配帧头低字节 0x05
                    if (b == (FSUS_PACK_RES_HEADER & 0xFF)) {  // 当 b == (FSUS_PACK_RES_HEADER & 0xFF) 成立时进入此分支，否则继续后续流程。
                        sum = b;  // 将 b 写入或初始化 sum。
                        step = 1;  // 将 1 写入或初始化 step。
                    }  // 结束此条件分支。
                    break;  // 退出当前循环或 switch 分支。
                case 1: // 匹配帧头高字节 0x1C
                    if (b == ((FSUS_PACK_RES_HEADER >> 8) & 0xFF)) {  // 当 b == ((FSUS_PACK_RES_HEADER >> 8) & 0xFF) 成立时进入此分支，否则继续后续流程。
                        sum += b;  // 把 b 累加到 sum。
                        step = 2;  // 将 2 写入或初始化 step。
                    } else {  // 前一个判断不成立时执行此分支。
                        step = 0; // 重置
                    }  // 结束此条件分支。
                    break;  // 退出当前循环或 switch 分支。
                case 2: // CmdId
                    cmdId = b;  // 将 b 写入或初始化 cmdId（命令编号）。
                    sum += b;  // 把 b 累加到 sum。
                    step = 3;  // 将 3 写入或初始化 step。
                    break;  // 退出当前循环或 switch 分支。
                case 3: // Length
                    length = b;  // 将 b 写入或初始化 length（长度）。
                    sum += b;  // 把 b 累加到 sum。
                    count = 0;  // 将 0 写入或初始化 count（计数）。
                    if (length > 0) {  // 当 length > 0 成立时进入此分支，否则继续后续流程。
                        step = 4;  // 将 4 写入或初始化 step。
                    } else {  // 前一个判断不成立时执行此分支。
                        step = 5;  // 将 5 写入或初始化 step。
                    }  // 结束此条件分支。
                    break;  // 退出当前循环或 switch 分支。
                case 4: // Content
                    if (count < 32) {  // 当 count < 32 成立时进入此分支，否则继续后续流程。
                        contentBuf[count] = b;  // 将 b 写入或初始化 contentBuf[count]。
                    }  // 结束此条件分支。
                    sum += b;  // 把 b 累加到 sum。
                    count++;  // 将 count（计数） 自增一。
                    if (count >= length) {  // 当 count >= length 成立时进入此分支，否则继续后续流程。
                        step = 5;  // 将 5 写入或初始化 step。
                    }  // 结束此条件分支。
                    break;  // 退出当前循环或 switch 分支。
                case 5: // Checksum
                    checksum = b;  // 将 b 写入或初始化 checksum。
                    if ((sum % 256) == checksum) {  // 当 (sum % 256) == checksum 成立时进入此分支，否则继续后续流程。
                        if (cmdId == expectedCmdId) {  // 当 cmdId == expectedCmdId 成立时进入此分支，否则继续后续流程。
                            *contentSize = length;  // 将 length 写入或初始化 contentSize（content大小）。
                            return FSUS_STATUS_SUCCESS;  // 结束当前函数并返回 FSUS_STATUS_SUCCESS。
                        }  // 结束此条件分支。
                    }  // 结束此条件分支。
                    // 上一条控制命令的应答可能晚于本次查询到达；跳过它继续等匹配帧。
                    // 校验错误也继续扫描，避免一个坏帧使后面的有效应答被丢弃。
                    step = 0;  // 将 0 写入或初始化 step。
                    break;  // 退出当前循环或 switch 分支。
            }  // 结束此函数或控制块。
        } else {  // 前一个判断不成立时执行此分支。
            delayMicroseconds(50);  // 调用 delayMicroseconds（延时Microseconds），传入 50。
        }  // 结束此条件分支。
    }  // 结束此循环体。

    return FSUS_STATUS_TIMEOUT;  // 结束当前函数并返回 FSUS_STATUS_TIMEOUT。
}  // 结束此函数或控制块。

// 初始化总线串口配置
void Servo_Init(HardwareSerial *serialPort, uint32_t baudrate, int rxPin, int txPin) {  // 定义函数或成员 的入口 Servo_Init，参数由此处的签名确定。
    if (s_servoMutex == nullptr) s_servoMutex = xSemaphoreCreateRecursiveMutex();  // 当 s_servoMutex == nullptr 成立时进入此分支，否则继续后续流程。
    if (s_servoMutex == nullptr) {  // 当 s_servoMutex == nullptr 成立时进入此分支，否则继续后续流程。
        Serial.println("[Servo] ERR: bus mutex allocation failed");  // 调用 Serial.println，传入 "[Servo] ERR: bus mutex allocation failed"。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    s_servoSerial = serialPort;  // 将 serialPort 写入或初始化 s_servoSerial。
    // 配置 ESP32 硬件串口
    s_servoSerial->begin(baudrate, SERIAL_8N1, rxPin, txPin);  // 调用 s_servoSerial->begin，传入 baudrate, SERIAL_8N1, rxPin, txPin。
    Serial.printf("[Servo] Bus Servo UART initialized on TX:%d RX:%d at %d bps\n", txPin, rxPin, baudrate);  // 调用 Serial.printf，传入 "[Servo] Bus Servo UART initialized on TX:%d RX:%d at …。
}  // 结束此函数或控制块。

// 设置舵机单圈绝对角度
void Servo_SetAngle(uint8_t servoId, float angle, uint16_t interval, uint16_t power) {  // 定义函数或成员 的入口 Servo_SetAngle（ServoSet角度），参数由此处的签名确定。
    int16_t angleInt = (int16_t)(angle * 10.0f); // 0.1°精度

    uint8_t content[7];  // 声明 content，类型和可用范围由所在声明决定。
    content[0] = servoId;  // 将 servoId 写入或初始化 content[0]。
    content[1] = angleInt & 0xFF;  // 将 angleInt & 0xFF 写入或初始化 content[1]。
    content[2] = (angleInt >> 8) & 0xFF;  // 将 (angleInt >> 8) & 0xFF 写入或初始化 content[2]。
    content[3] = interval & 0xFF;  // 将 interval & 0xFF 写入或初始化 content[3]。
    content[4] = (interval >> 8) & 0xFF;  // 将 (interval >> 8) & 0xFF 写入或初始化 content[4]。
    content[5] = power & 0xFF;  // 将 power & 0xFF 写入或初始化 content[5]。
    content[6] = (power >> 8) & 0xFF;  // 将 (power >> 8) & 0xFF 写入或初始化 content[6]。

    sendPacket(FSUS_CMD_SET_ANGLE, content, 7);  // 调用 sendPacket，传入 FSUS_CMD_SET_ANGLE, content, 7。
}  // 结束此函数或控制块。

// 多圈绝对角度控制 (HA8-U25H-M 专用，支持 ±1024 圈)
void Servo_SetAngleMTurn(uint8_t servoId, float angle, uint32_t interval, uint16_t power) {  // 定义函数或成员 的入口 Servo_SetAngleMTurn（ServoSet角度MTurn），参数由此处的签名确定。
    int32_t angleInt = (int32_t)(angle * 10.0f);  // 将 (int32_t)(angle * 10.0f) 写入或初始化 angleInt（角度Int）。

    uint8_t content[11];  // 声明 content，类型和可用范围由所在声明决定。
    content[0] = servoId;  // 将 servoId 写入或初始化 content[0]。
    content[1] = angleInt & 0xFF;  // 将 angleInt & 0xFF 写入或初始化 content[1]。
    content[2] = (angleInt >> 8) & 0xFF;  // 将 (angleInt >> 8) & 0xFF 写入或初始化 content[2]。
    content[3] = (angleInt >> 16) & 0xFF;  // 将 (angleInt >> 16) & 0xFF 写入或初始化 content[3]。
    content[4] = (angleInt >> 24) & 0xFF;  // 将 (angleInt >> 24) & 0xFF 写入或初始化 content[4]。
    content[5] = interval & 0xFF;  // 将 interval & 0xFF 写入或初始化 content[5]。
    content[6] = (interval >> 8) & 0xFF;  // 将 (interval >> 8) & 0xFF 写入或初始化 content[6]。
    content[7] = (interval >> 16) & 0xFF;  // 将 (interval >> 16) & 0xFF 写入或初始化 content[7]。
    content[8] = (interval >> 24) & 0xFF;  // 将 (interval >> 24) & 0xFF 写入或初始化 content[8]。
    content[9] = power & 0xFF;  // 将 power & 0xFF 写入或初始化 content[9]。
    content[10] = (power >> 8) & 0xFF;  // 将 (power >> 8) & 0xFF 写入或初始化 content[10]。

    sendPacket(FSUS_CMD_SET_ANGLE_MTURN, content, 11);  // 调用 sendPacket，传入 FSUS_CMD_SET_ANGLE_MTURN, content, 11。
}  // 结束此函数或控制块。

// 0x0F 按厂商 SDK 的 13 字节负载，速度为 16 位，单位 0.1°/s。
void Servo_MoveCargoTray(float angle) {  // 定义函数或成员 的入口 Servo_MoveCargoTray（ServoMove货位料盘），参数由此处的签名确定。
    constexpr uint16_t speed = 3000; // 300°/s。
    constexpr uint16_t rampMs = 30;  // 将 30 写入或初始化 rampMs（ramp毫秒）。
    const uint32_t position = static_cast<uint32_t>(static_cast<int32_t>(angle * 10.0f));  // 将 static_cast<uint32_t>(static_cast<int32_t>(angle * 10.… 写入或初始化 position（位置）。
    const uint8_t content[] = {  // 声明 content，类型和可用范围由所在声明决定。
        SERVO_ID_CARGO_TRAY,  // 为当前列表或跨行表达式提供这一项 SERVO_ID_CARGO_TRAY。
        static_cast<uint8_t>(position), static_cast<uint8_t>(position >> 8),  // 定义函数或成员 uint8_t>，参数由此处的签名确定。
        static_cast<uint8_t>(position >> 16), static_cast<uint8_t>(position >> 24),  // 定义函数或成员 uint8_t>，参数由此处的签名确定。
        static_cast<uint8_t>(speed), static_cast<uint8_t>(speed >> 8),  // 定义函数或成员 uint8_t>，参数由此处的签名确定。
        static_cast<uint8_t>(rampMs), static_cast<uint8_t>(rampMs >> 8),  // 定义函数或成员 uint8_t>，参数由此处的签名确定。
        static_cast<uint8_t>(rampMs), static_cast<uint8_t>(rampMs >> 8),  // 定义函数或成员 uint8_t>，参数由此处的签名确定。
        0, 0, // 使用舵机功率保护阈值。
    };  // 结束此初始化列表或局部作用域。
    sendPacket(FSUS_CMD_SET_ANGLE_MTURN_VELOCITY, content, sizeof(content));  // 调用 sendPacket，传入 FSUS_CMD_SET_ANGLE_MTURN_VELOCITY, content, sizeof(con…。
}  // 结束此函数或控制块。

// 官方 UART/RS-485 协议第11节：高级多圈角度控制（基于时间）。
// https://fashionstar.com.cn/wiki/protocols/servo/uart-rs485/
void Servo_SetAngleMTurnByInterval(uint8_t servoId, float angle, uint32_t interval, uint16_t t_acc, uint16_t t_dec, uint16_t power) {  // 定义函数或成员 的入口 Servo_SetAngleMTurnByInterval（ServoSet角度MTurnByInterval），参数由此处的签名确定。
    const int32_t angleInt = static_cast<int32_t>(angle * 10.0f);  // 将 static_cast<int32_t>(angle * 10.0f) 写入或初始化 angleInt（角度Int）。
    // 先转为无符号位模式，负角度也按32位补码、小端顺序发送。
    const uint32_t angleBits = static_cast<uint32_t>(angleInt);  // 将 static_cast<uint32_t>(angleInt) 写入或初始化 angleBits（角度Bits）。

    uint8_t content[15];  // 声明 content，类型和可用范围由所在声明决定。
    content[0] = servoId;  // 将 servoId 写入或初始化 content[0]。
    content[1] = angleBits & 0xFF;  // 将 angleBits & 0xFF 写入或初始化 content[1]。
    content[2] = (angleBits >> 8) & 0xFF;  // 将 (angleBits >> 8) & 0xFF 写入或初始化 content[2]。
    content[3] = (angleBits >> 16) & 0xFF;  // 将 (angleBits >> 16) & 0xFF 写入或初始化 content[3]。
    content[4] = (angleBits >> 24) & 0xFF;  // 将 (angleBits >> 24) & 0xFF 写入或初始化 content[4]。
    content[5] = interval & 0xFF;  // 将 interval & 0xFF 写入或初始化 content[5]。
    content[6] = (interval >> 8) & 0xFF;  // 将 (interval >> 8) & 0xFF 写入或初始化 content[6]。
    content[7] = (interval >> 16) & 0xFF;  // 将 (interval >> 16) & 0xFF 写入或初始化 content[7]。
    content[8] = (interval >> 24) & 0xFF;  // 将 (interval >> 24) & 0xFF 写入或初始化 content[8]。
    content[9] = t_acc & 0xFF;  // 将 t_acc & 0xFF 写入或初始化 content[9]。
    content[10] = (t_acc >> 8) & 0xFF;  // 将 (t_acc >> 8) & 0xFF 写入或初始化 content[10]。
    content[11] = t_dec & 0xFF;  // 将 t_dec & 0xFF 写入或初始化 content[11]。
    content[12] = (t_dec >> 8) & 0xFF;  // 将 (t_dec >> 8) & 0xFF 写入或初始化 content[12]。
    content[13] = power & 0xFF;  // 将 power & 0xFF 写入或初始化 content[13]。
    content[14] = (power >> 8) & 0xFF;  // 将 (power >> 8) & 0xFF 写入或初始化 content[14]。

    // sendPacket 沿用总线锁，并补齐帧头、指令、长度及累加校验。
    sendPacket(FSUS_CMD_SET_ANGLE_MTURN_INTERVAL, content, sizeof(content));  // 调用 sendPacket，传入 FSUS_CMD_SET_ANGLE_MTURN_INTERVAL, content, sizeof(con…。
}  // 结束此函数或控制块。

// 梯形加减速角度控制
void Servo_SetAngleByInterval(uint8_t servoId, float angle, uint16_t interval, uint16_t t_acc, uint16_t t_dec, uint16_t power) {  // 定义函数或成员 的入口 Servo_SetAngleByInterval（ServoSet角度ByInterval），参数由此处的签名确定。
    int16_t angleInt = (int16_t)(angle * 10.0f);  // 将 (int16_t)(angle * 10.0f) 写入或初始化 angleInt（角度Int）。

    uint8_t content[11];  // 声明 content，类型和可用范围由所在声明决定。
    content[0] = servoId;  // 将 servoId 写入或初始化 content[0]。
    content[1] = angleInt & 0xFF;  // 将 angleInt & 0xFF 写入或初始化 content[1]。
    content[2] = (angleInt >> 8) & 0xFF;  // 将 (angleInt >> 8) & 0xFF 写入或初始化 content[2]。
    content[3] = interval & 0xFF;  // 将 interval & 0xFF 写入或初始化 content[3]。
    content[4] = (interval >> 8) & 0xFF;  // 将 (interval >> 8) & 0xFF 写入或初始化 content[4]。
    content[5] = t_acc & 0xFF;  // 将 t_acc & 0xFF 写入或初始化 content[5]。
    content[6] = (t_acc >> 8) & 0xFF;  // 将 (t_acc >> 8) & 0xFF 写入或初始化 content[6]。
    content[7] = t_dec & 0xFF;  // 将 t_dec & 0xFF 写入或初始化 content[7]。
    content[8] = (t_dec >> 8) & 0xFF;  // 将 (t_dec >> 8) & 0xFF 写入或初始化 content[8]。
    content[9] = power & 0xFF;  // 将 power & 0xFF 写入或初始化 content[9]。
    content[10] = (power >> 8) & 0xFF;  // 将 (power >> 8) & 0xFF 写入或初始化 content[10]。

    sendPacket(FSUS_CMD_SET_ANGLE_INTERVAL, content, 11);  // 调用 sendPacket，传入 FSUS_CMD_SET_ANGLE_INTERVAL, content, 11。
}  // 结束此函数或控制块。

// 查询舵机当前角度
bool Servo_QueryAngle(uint8_t servoId, float &currentAngle, uint32_t timeoutMs) {  // 定义函数或成员 的入口 Servo_QueryAngle（ServoQuery角度），参数由此处的签名确定。
    ServoBusGuard guard;  // 声明 guard，类型和可用范围由所在声明决定。
    if (!guard.locked) return false;  // 若 !guard.locked，立即返回 false。
    uint8_t content[1] = { servoId };  // 将 { servoId } 写入或初始化 content[1]。
    sendPacket(FSUS_CMD_QUERY_ANGLE, content, 1);  // 调用 sendPacket，传入 FSUS_CMD_QUERY_ANGLE, content, 1。

    uint8_t rxBuf[32]; // receivePacket 最多写入 32 字节。
    uint8_t rxLen = 0;  // 将 0 写入或初始化 rxLen（接收Len）。
    uint8_t res = receivePacket(FSUS_CMD_QUERY_ANGLE, rxBuf, &rxLen, timeoutMs);  // 将 receivePacket(FSUS_CMD_QUERY_ANGLE, rxBuf, &rxLen, tim… 写入或初始化 res。
    if (res == FSUS_STATUS_SUCCESS && rxLen == 3 && rxBuf[0] == servoId) {  // 当 res == FSUS_STATUS_SUCCESS && rxLen == 3 && rxBuf[0] =… 成立时进入此分支，否则继续后续流程。
        int16_t angleRaw = (int16_t)(rxBuf[1] | (rxBuf[2] << 8));  // 将 (int16_t)(rxBuf[1] | (rxBuf[2] << 8)) 写入或初始化 angleRaw（角度Raw）。
        currentAngle = (float)angleRaw / 10.0f;  // 将 (float)angleRaw / 10.0f 写入或初始化 currentAngle（当前角度）。
        return true;  // 结束当前函数并返回 true。
    }  // 结束此条件分支。
    return false;  // 结束当前函数并返回 false。
}  // 结束此函数或控制块。

bool Servo_QueryAngleMTurn(uint8_t servoId, float &currentAngle, uint32_t timeoutMs) {  // 定义函数或成员 的入口 Servo_QueryAngleMTurn（ServoQuery角度MTurn），参数由此处的签名确定。
    ServoBusGuard guard;  // 声明 guard，类型和可用范围由所在声明决定。
    if (!guard.locked) return false;  // 若 !guard.locked，立即返回 false。
    uint8_t content[1] = { servoId };  // 将 { servoId } 写入或初始化 content[1]。
    sendPacket(FSUS_CMD_QUERY_ANGLE_MTURN, content, 1);  // 调用 sendPacket，传入 FSUS_CMD_QUERY_ANGLE_MTURN, content, 1。

    uint8_t rxBuf[32];  // 声明 rxBuf（接收Buf），类型和可用范围由所在声明决定。
    uint8_t rxLen = 0;  // 将 0 写入或初始化 rxLen（接收Len）。
    const uint8_t res = receivePacket(FSUS_CMD_QUERY_ANGLE_MTURN, rxBuf, &rxLen, timeoutMs);  // 将 receivePacket(FSUS_CMD_QUERY_ANGLE_MTURN, rxBuf, &rxLe… 写入或初始化 res。
    // 应答为：ID(1) + 有符号多圈角度(4, 0.1度) + 圈数(2)。
    if (res == FSUS_STATUS_SUCCESS && rxLen == 7 && rxBuf[0] == servoId) {  // 当 res == FSUS_STATUS_SUCCESS && rxLen == 7 && rxBuf[0] =… 成立时进入此分支，否则继续后续流程。
        const uint32_t rawBits = static_cast<uint32_t>(rxBuf[1])  // 将 static_cast<uint32_t>(rxBuf[1]) 写入或初始化 rawBits。
            | (static_cast<uint32_t>(rxBuf[2]) << 8)  // 调用 uint32_t>，传入 rxBuf[2]) << 8。
            | (static_cast<uint32_t>(rxBuf[3]) << 16)  // 调用 uint32_t>，传入 rxBuf[3]) << 16。
            | (static_cast<uint32_t>(rxBuf[4]) << 24);  // 调用 uint32_t>，传入 rxBuf[4]) << 24。
        const int32_t rawAngle = static_cast<int32_t>(rawBits);  // 将 static_cast<int32_t>(rawBits) 写入或初始化 rawAngle（raw角度）。
        currentAngle = static_cast<float>(rawAngle) / 10.0f;  // 将 static_cast<float>(rawAngle) / 10.0f 写入或初始化 currentAngle（当前角度）。
        return true;  // 结束当前函数并返回 true。
    }  // 结束此条件分支。
    Serial.printf("[Servo] multi-turn query ID=%u failed: status=%u length=%u\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                  static_cast<unsigned>(servoId), static_cast<unsigned>(res),  // 定义函数或成员 unsigned>，参数由此处的签名确定。
                  static_cast<unsigned>(rxLen));  // 声明接口 unsigned>，参数由此处的签名确定。
    return false;  // 结束当前函数并返回 false。
}  // 结束此函数或控制块。

/** @brief 厂家进阶参数协议：13 4D帧头，C5读取ID及25字节参数块。 */
static bool readPidBlock(uint8_t servoId, uint8_t block[32]) {  // 定义函数或成员 的入口 readPidBlock，参数由此处的签名确定。
    sendPacket(0xC5, &servoId, 1, 0x4D13);  // 调用 sendPacket，传入 0xC5, &servoId, 1, 0x4D13。
    uint8_t length=0;  // 将 0 写入或初始化 length（长度）。
    return receivePacket(0xC5,block,&length,100)==FSUS_STATUS_SUCCESS  // 结束当前函数并返回 receivePacket(0xC5,block,&length,100)==FSUS_STATUS_SUC…。
        &&length==26&&block[0]==servoId;  // 继续上一行的组合条件，本行提供 &&length==26&&block[0]==servoId;。
}  // 结束此函数或控制块。

// 读取 PID 块后按协议顺序提取字段。
// 配合写入函数阅读时要注意 PID 数据只占整块的一部分。
bool Servo_ReadPid(uint8_t servoId, uint16_t values[7]) {  // 定义函数或成员 的入口 Servo_ReadPid（ServoReadPID），参数由此处的签名确定。
    ServoBusGuard guard;if(!guard.locked)return false;  // 声明 guard，类型和可用范围由所在声明决定。然后若 !guard.locked，立即返回 false。
    uint8_t block[32];if(!readPidBlock(servoId,block))return false;  // 声明 block，类型和可用范围由所在声明决定。然后若 !readPidBlock(servoId,block)，立即返回 false。
    for(uint8_t i=0;i<7;++i)values[i]=block[1+2*i]|(uint16_t(block[2+2*i])<<8);  // 按 uint8_t i=0;i<7;++i)values[i]=block[1+2*i]|(uint16_t(b… 组织循环的初值、继续条件和步进。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

bool Servo_WritePid(uint8_t servoId, const uint16_t values[7]) {  // 定义函数或成员 的入口 Servo_WritePid（ServoWritePID），参数由此处的签名确定。
    ServoBusGuard guard;if(!guard.locked)return false;  // 声明 guard，类型和可用范围由所在声明决定。然后若 !guard.locked，立即返回 false。
    uint8_t block[32];if(!readPidBlock(servoId,block))return false;  // 声明 block，类型和可用范围由所在声明决定。然后若 !readPidBlock(servoId,block)，立即返回 false。
    for(uint8_t i=0;i<7;++i){block[1+2*i]=values[i]&0xFF;block[2+2*i]=values[i]>>8;}  // 按 uint8_t i=0;i<7;++i 组织循环的初值、继续条件和步进。
    // 沿用厂家调试工具的解锁顺序，未公开的11字节参数保持原值。
    const uint8_t release[]={servoId,0x10,0,0};sendPacket(FSUS_CMD_STOP,release,4);  // 调用 sendPacket，传入 FSUS_CMD_STOP,release,4。
    s_servoSerial->flush();vTaskDelay(pdMS_TO_TICKS(60));  // 调用 s_servoSerial->flush，传入 );vTaskDelay(pdMS_TO_TICKS(60)。
    sendPacket(0xC4,block,26,0x4D13);s_servoSerial->flush();  // 调用 sendPacket，传入 0xC4,block,26,0x4D13。然后调用 s_servoSerial->flush。
    // 参数写入后留300ms，再恢复锁力并由串口命令回读，避免紧接着查询无应答。
    vTaskDelay(pdMS_TO_TICKS(300));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    const uint8_t hold[]={servoId,0x11,0,0};sendPacket(FSUS_CMD_STOP,hold,4);  // 调用 sendPacket，传入 FSUS_CMD_STOP,hold,4。
    s_servoSerial->flush();  // 调用 s_servoSerial->flush。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

bool Servo_ResetTurnCount(uint8_t servoId, float &newAngle) {  // 定义函数或成员 的入口 Servo_ResetTurnCount（ServoResetTurn计数），参数由此处的签名确定。
    ServoBusGuard guard;  // 声明 guard，类型和可用范围由所在声明决定。
    if (!guard.locked || servoId == SERVO_ID_BROADCAST) return false;  // 若 !guard.locked || servoId == SERVO_ID_BROADCAST，立即返回 false。

    // 厂商要求重置圈数前先释放锁力。停止帧内容：ID、模式、功率低/高字节。
    const uint8_t releaseContent[4] = {servoId, 0x10, 0, 0};  // 将 {servoId, 0x10, 0, 0} 写入或初始化 releaseContent[4]。
    const uint8_t holdContent[4] = {servoId, 0x11, 0, 0};  // 将 {servoId, 0x11, 0, 0} 写入或初始化 holdContent[4]。
    const uint8_t resetContent[1] = {servoId};  // 将 {servoId} 写入或初始化 resetContent[1]。
    sendPacket(FSUS_CMD_STOP, releaseContent, sizeof(releaseContent));  // 调用 sendPacket，传入 FSUS_CMD_STOP, releaseContent, sizeof(releaseContent)。
    vTaskDelay(pdMS_TO_TICKS(10));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    sendPacket(FSUS_CMD_RESET_TURNS, resetContent, sizeof(resetContent));  // 调用 sendPacket，传入 FSUS_CMD_RESET_TURNS, resetContent, sizeof(resetConten…。
    vTaskDelay(pdMS_TO_TICKS(10));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    // 不论后续回读是否成功，都先恢复当前位置锁力。
    sendPacket(FSUS_CMD_STOP, holdContent, sizeof(holdContent));  // 调用 sendPacket，传入 FSUS_CMD_STOP, holdContent, sizeof(holdContent)。
    vTaskDelay(pdMS_TO_TICKS(10));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    return Servo_QueryAngleMTurn(servoId, newAngle, 100);  // 结束当前函数并返回 Servo_QueryAngleMTurn(servoId, newAngle, 100)。
}  // 结束此函数或控制块。

// 检测舵机通讯状态 (Ping)
bool Servo_Ping(uint8_t servoId, uint32_t timeoutMs) {  // 定义函数或成员 的入口 Servo_Ping，参数由此处的签名确定。
    ServoBusGuard guard;  // 声明 guard，类型和可用范围由所在声明决定。
    if (!guard.locked) return false;  // 若 !guard.locked，立即返回 false。
    uint8_t content[1] = { servoId };  // 将 { servoId } 写入或初始化 content[1]。
    sendPacket(FSUS_CMD_PING, content, 1);  // 调用 sendPacket，传入 FSUS_CMD_PING, content, 1。

    uint8_t rxBuf[4];  // 声明 rxBuf（接收Buf），类型和可用范围由所在声明决定。
    uint8_t rxLen = 0;  // 将 0 写入或初始化 rxLen（接收Len）。
    uint8_t res = receivePacket(FSUS_CMD_PING, rxBuf, &rxLen, timeoutMs);  // 将 receivePacket(FSUS_CMD_PING, rxBuf, &rxLen, timeoutMs) 写入或初始化 res。
    return (res == FSUS_STATUS_SUCCESS);  // 结束当前函数并返回 (res == FSUS_STATUS_SUCCESS)。
}  // 结束此函数或控制块。

// 设置阻尼模式 / 卸力
void Servo_SetDamping(uint8_t servoId, uint16_t power) {  // 定义函数或成员 的入口 Servo_SetDamping，参数由此处的签名确定。
    uint8_t content[3];  // 声明 content，类型和可用范围由所在声明决定。
    content[0] = servoId;  // 将 servoId 写入或初始化 content[0]。
    content[1] = power & 0xFF;  // 将 power & 0xFF 写入或初始化 content[1]。
    content[2] = (power >> 8) & 0xFF;  // 将 (power >> 8) & 0xFF 写入或初始化 content[2]。

    sendPacket(FSUS_CMD_SET_DAMPING, content, 3);  // 调用 sendPacket，传入 FSUS_CMD_SET_DAMPING, content, 3。
}  // 结束此函数或控制块。

// 轮式连续旋转模式
void Servo_SetWheelMode(uint8_t servoId, float speed) {  // 定义函数或成员 的入口 Servo_SetWheelMode（ServoSet轮模式），参数由此处的签名确定。
    int16_t speedInt = (int16_t)(speed * 10.0f);  // 将 (int16_t)(speed * 10.0f) 写入或初始化 speedInt（速度Int）。
    uint8_t content[3];  // 声明 content，类型和可用范围由所在声明决定。
    content[0] = servoId;  // 将 servoId 写入或初始化 content[0]。
    content[1] = speedInt & 0xFF;  // 将 speedInt & 0xFF 写入或初始化 content[1]。
    content[2] = (speedInt >> 8) & 0xFF;  // 将 (speedInt >> 8) & 0xFF 写入或初始化 content[2]。

    sendPacket(FSUS_CMD_WHEEL_MODE, content, 3);  // 调用 sendPacket，传入 FSUS_CMD_WHEEL_MODE, content, 3。
}  // 结束此函数或控制块。

// 立即停止
void Servo_Stop(uint8_t servoId) {  // 定义函数或成员 的入口 Servo_Stop（Servo停止），参数由此处的签名确定。
    float curAngle = 0;  // 将 0 写入或初始化 curAngle（cur角度）。
    if (Servo_QueryAngle(servoId, curAngle, 20)) {  // 当 Servo_QueryAngle(servoId, curAngle, 20) 成立时进入此分支，否则继续后续流程。
        Servo_SetAngle(servoId, curAngle, 0);  // 调用 Servo_SetAngle（ServoSet角度），传入 servoId, curAngle, 0。
    } else {  // 前一个判断不成立时执行此分支。
        Servo_SetDamping(servoId, 500);  // 调用 Servo_SetDamping，传入 servoId, 500。
    }  // 结束此条件分支。
}  // 结束此函数或控制块。

void Servo_HoldPosition(uint8_t servoId) {  // 定义函数或成员 的入口 Servo_HoldPosition（Servo保持位置），参数由此处的签名确定。
    const uint8_t hold[]={servoId,0x11,0,0};  // 声明 hold（保持），类型和可用范围由所在声明决定。
    sendPacket(FSUS_CMD_STOP,hold,sizeof(hold));  // 调用 sendPacket，传入 FSUS_CMD_STOP,hold,sizeof(hold)。
}  // 结束此函数或控制块。

// 厂商设置原点指令：ID、reset=0；与重置多圈圈数(0x11)不同。
bool Servo_SetOrigin(uint8_t servoId) {  // 定义函数或成员 的入口 Servo_SetOrigin（ServoSet原点），参数由此处的签名确定。
    ServoBusGuard guard;  // 声明 guard，类型和可用范围由所在声明决定。
    if(!guard.locked)return false;  // 若 !guard.locked，立即返回 false。
    // 厂商SDK要求失锁后才能设置原点；结束后从当前位置恢复锁力。
    const uint8_t release[4]={servoId,0x10,0,0};  // 将 {servoId,0x10,0,0} 写入或初始化 release[4]。
    const uint8_t content[2]={servoId,0};  // 将 {servoId,0} 写入或初始化 content[2]。
    const uint8_t hold[4]={servoId,0x11,0,0};  // 将 {servoId,0x11,0,0} 写入或初始化 hold[4]。
    sendPacket(FSUS_CMD_STOP,release,sizeof(release));  // 调用 sendPacket，传入 FSUS_CMD_STOP,release,sizeof(release)。
    s_servoSerial->flush();vTaskDelay(pdMS_TO_TICKS(10));  // 调用 s_servoSerial->flush，传入 );vTaskDelay(pdMS_TO_TICKS(10)。
    sendPacket(FSUS_CMD_SET_ORIGIN,content,sizeof(content));  // 调用 sendPacket，传入 FSUS_CMD_SET_ORIGIN,content,sizeof(content)。
    s_servoSerial->flush();vTaskDelay(pdMS_TO_TICKS(10));  // 调用 s_servoSerial->flush，传入 );vTaskDelay(pdMS_TO_TICKS(10)。
    sendPacket(FSUS_CMD_STOP,hold,sizeof(hold));  // 调用 sendPacket，传入 FSUS_CMD_STOP,hold,sizeof(hold)。
    s_servoSerial->flush();vTaskDelay(pdMS_TO_TICKS(10));  // 调用 s_servoSerial->flush，传入 );vTaskDelay(pdMS_TO_TICKS(10)。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。
