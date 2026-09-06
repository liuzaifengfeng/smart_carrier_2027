#include "servo.h"

// 内部串口指针，默认指向 Serial2
static HardwareSerial *s_servoSerial = &Serial2;

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
static void sendPacket(uint8_t cmdId, const uint8_t *content, uint8_t contentSize) {
    if (!s_servoSerial) return;

    uint8_t headerLow = FSUS_PACK_REQ_HEADER & 0xFF;         // 0x12
    uint8_t headerHigh = (FSUS_PACK_REQ_HEADER >> 8) & 0xFF; // 0x4C

    uint16_t sum = headerLow + headerHigh + cmdId + contentSize;
    for (uint8_t i = 0; i < contentSize; i++) {
        sum += content[i];
    }
    uint8_t checksum = sum % 256;

    // 清空接收缓冲区未处理的历史残留
    while (s_servoSerial->available() > 0) {
        s_servoSerial->read();
    }

    s_servoSerial->write(headerLow);
    s_servoSerial->write(headerHigh);
    s_servoSerial->write(cmdId);
    s_servoSerial->write(contentSize);
    if (contentSize > 0 && content != nullptr) {
        s_servoSerial->write(content, contentSize);
    }
    s_servoSerial->write(checksum);
}

/**
 * @brief 接收应答帧 (针对带有返回的指令)
 * 应答帧头: 0x05, 0x1C (0x1C05)
 */
static uint8_t receivePacket(uint8_t expectedCmdId, uint8_t *contentBuf, uint8_t *contentSize, uint32_t timeoutMs) {
    if (!s_servoSerial) return FSUS_STATUS_TIMEOUT;

    uint32_t startMs = millis();
    uint8_t step = 0;
    uint8_t cmdId = 0;
    uint8_t length = 0;
    uint8_t count = 0;
    uint8_t checksum = 0;
    uint16_t sum = 0;

    while (millis() - startMs < timeoutMs) {
        if (s_servoSerial->available() > 0) {
            uint8_t b = s_servoSerial->read();

            switch (step) {
                case 0: // 匹配帧头低字节 0x05
                    if (b == (FSUS_PACK_RES_HEADER & 0xFF)) {
                        sum = b;
                        step = 1;
                    }
                    break;
                case 1: // 匹配帧头高字节 0x1C
                    if (b == ((FSUS_PACK_RES_HEADER >> 8) & 0xFF)) {
                        sum += b;
                        step = 2;
                    } else {
                        step = 0; // 重置
                    }
                    break;
                case 2: // CmdId
                    cmdId = b;
                    sum += b;
                    step = 3;
                    break;
                case 3: // Length
                    length = b;
                    sum += b;
                    count = 0;
                    if (length > 0) {
                        step = 4;
                    } else {
                        step = 5;
                    }
                    break;
                case 4: // Content
                    if (count < 32 && contentBuf != nullptr) {
                        contentBuf[count] = b;
                    }
                    sum += b;
                    count++;
                    if (count >= length) {
                        step = 5;
                    }
                    break;
                case 5: // Checksum
                    checksum = b;
                    if ((sum % 256) == checksum) {
                        if (contentSize != nullptr) {
                            *contentSize = length;
                        }
                        if (cmdId == expectedCmdId) {
                            return FSUS_STATUS_SUCCESS;
                        } else {
                            return FSUS_STATUS_ID_MISMATCH;
                        }
                    } else {
                        return FSUS_STATUS_CHECKSUM_ERR;
                    }
            }
        } else {
            delayMicroseconds(50);
        }
    }

    return FSUS_STATUS_TIMEOUT;
}

// 初始化总线串口配置
void Servo_Init(HardwareSerial *serialPort, uint32_t baudrate, int rxPin, int txPin) {
    if (serialPort != nullptr) {
        s_servoSerial = serialPort;
    }
    // 配置 ESP32 硬件串口
    s_servoSerial->begin(baudrate, SERIAL_8N1, rxPin, txPin);
    Serial.printf("[Servo] Bus Servo UART initialized on TX:%d RX:%d at %d bps\n", txPin, rxPin, baudrate);
}

// 设置舵机单圈绝对角度
void Servo_SetAngle(uint8_t servoId, float angle, uint16_t interval, uint16_t power) {
    // 约束范围: -135.0° ~ 135.0°
    if (angle < -135.0f) angle = -135.0f;
    if (angle > 135.0f) angle = 135.0f;

    int16_t angleInt = (int16_t)(angle * 10.0f); // 0.1°精度

    uint8_t content[7];
    content[0] = servoId;
    content[1] = angleInt & 0xFF;
    content[2] = (angleInt >> 8) & 0xFF;
    content[3] = interval & 0xFF;
    content[4] = (interval >> 8) & 0xFF;
    content[5] = power & 0xFF;
    content[6] = (power >> 8) & 0xFF;

    sendPacket(FSUS_CMD_SET_ANGLE, content, 7);
}

// 多圈绝对角度控制 (HA8-U25H-M 专用，支持 ±1024 圈)
void Servo_SetAngleMTurn(uint8_t servoId, float angle, uint32_t interval, uint16_t power) {
    int32_t angleInt = (int32_t)(angle * 10.0f);

    uint8_t content[11];
    content[0] = servoId;
    content[1] = angleInt & 0xFF;
    content[2] = (angleInt >> 8) & 0xFF;
    content[3] = (angleInt >> 16) & 0xFF;
    content[4] = (angleInt >> 24) & 0xFF;
    content[5] = interval & 0xFF;
    content[6] = (interval >> 8) & 0xFF;
    content[7] = (interval >> 16) & 0xFF;
    content[8] = (interval >> 24) & 0xFF;
    content[9] = power & 0xFF;
    content[10] = (power >> 8) & 0xFF;

    sendPacket(FSUS_CMD_SET_ANGLE_MTURN, content, 11);
}

// 梯形加减速角度控制
void Servo_SetAngleByInterval(uint8_t servoId, float angle, uint16_t interval, uint16_t t_acc, uint16_t t_dec, uint16_t power) {
    if (angle < -135.0f) angle = -135.0f;
    if (angle > 135.0f) angle = 135.0f;

    int16_t angleInt = (int16_t)(angle * 10.0f);

    uint8_t content[11];
    content[0] = servoId;
    content[1] = angleInt & 0xFF;
    content[2] = (angleInt >> 8) & 0xFF;
    content[3] = interval & 0xFF;
    content[4] = (interval >> 8) & 0xFF;
    content[5] = t_acc & 0xFF;
    content[6] = (t_acc >> 8) & 0xFF;
    content[7] = t_dec & 0xFF;
    content[8] = (t_dec >> 8) & 0xFF;
    content[9] = power & 0xFF;
    content[10] = (power >> 8) & 0xFF;

    sendPacket(FSUS_CMD_SET_ANGLE_INTERVAL, content, 11);
}

// 查询舵机当前角度
bool Servo_QueryAngle(uint8_t servoId, float &currentAngle, uint32_t timeoutMs) {
    uint8_t content[1] = { servoId };
    sendPacket(FSUS_CMD_QUERY_ANGLE, content, 1);

    uint8_t rxBuf[8];
    uint8_t rxLen = 0;
    uint8_t res = receivePacket(FSUS_CMD_QUERY_ANGLE, rxBuf, &rxLen, timeoutMs);
    if (res == FSUS_STATUS_SUCCESS && rxLen >= 3) {
        int16_t angleRaw = (int16_t)(rxBuf[1] | (rxBuf[2] << 8));
        currentAngle = (float)angleRaw / 10.0f;
        return true;
    }
    return false;
}

// 检测舵机通讯状态 (Ping)
bool Servo_Ping(uint8_t servoId, uint32_t timeoutMs) {
    uint8_t content[1] = { servoId };
    sendPacket(FSUS_CMD_PING, content, 1);

    uint8_t rxBuf[4];
    uint8_t rxLen = 0;
    uint8_t res = receivePacket(FSUS_CMD_PING, rxBuf, &rxLen, timeoutMs);
    return (res == FSUS_STATUS_SUCCESS);
}

// 设置阻尼模式 / 卸力
void Servo_SetDamping(uint8_t servoId, uint16_t power) {
    uint8_t content[3];
    content[0] = servoId;
    content[1] = power & 0xFF;
    content[2] = (power >> 8) & 0xFF;

    sendPacket(FSUS_CMD_SET_DAMPING, content, 3);
}

// 轮式连续旋转模式
void Servo_SetWheelMode(uint8_t servoId, float speed) {
    int16_t speedInt = (int16_t)(speed * 10.0f);
    uint8_t content[3];
    content[0] = servoId;
    content[1] = speedInt & 0xFF;
    content[2] = (speedInt >> 8) & 0xFF;

    sendPacket(FSUS_CMD_WHEEL_MODE, content, 3);
}

// 立即停止
void Servo_Stop(uint8_t servoId) {
    float curAngle = 0;
    if (Servo_QueryAngle(servoId, curAngle, 20)) {
        Servo_SetAngle(servoId, curAngle, 0);
    } else {
        Servo_SetDamping(servoId, 500);
    }
}
