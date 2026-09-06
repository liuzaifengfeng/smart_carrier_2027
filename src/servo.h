#ifndef FASHIONSTAR_SERVO_H
#define FASHIONSTAR_SERVO_H

#include <Arduino.h>

/*******************************************************************************
 * Fashion Star (华馨京) UART 总线舵机驱动模块 (以 HA8-U25H-M 为主)
 * 
 * 硬件说明:
 *  - 型号: HA8-U25H-M (12-bit 磁编码高压总线舵机)
 *  - 通信方式: UART / TTL 半双工异步串行通信 (波特率默认 115200)
 *  - 默认引脚 (ESP32-S3): UART2 TX=15, RX=16
 *  - 接线说明: ESP32-S3 串口连接总线舵机转接板 (或半双工驱动电路)
 ******************************************************************************/

// 默认串口引脚定义 (ESP32-S3 可在 Servo_Init 中自定义引脚)
#ifndef SERVO_UART_TX_PIN
#define SERVO_UART_TX_PIN 15
#endif

#ifndef SERVO_UART_RX_PIN
#define SERVO_UART_RX_PIN 16
#endif

#define SERVO_UART_BAUDRATE 115200

// 常用机械结构舵机 ID 宏定义 (可按实际硬件分配调整)
#define SERVO_ID_ARM_BIG    1    // 大臂舵机
#define SERVO_ID_ARM_FORE   2    // 夹臂舵机
#define SERVO_ID_CLAW       3    // 夹爪舵机
#define SERVO_ID_CYLINDER   5    // 料筒/料仓舵机
#define SERVO_ID_BROADCAST  0xFE // 广播 ID (0xFE / 0xFF)

// 协议常量定义
#define FSUS_PACK_REQ_HEADER    0x4C12  // 请求帧头 (小端发送为 0x12, 0x4C)
#define FSUS_PACK_RES_HEADER    0x1C05  // 应答帧头 (小端接收为 0x05, 0x1C)

// 指令码定义
#define FSUS_CMD_PING           0x01    // 通讯检测 / PING
#define FSUS_CMD_RESET_USER_DATA 0x02   // 恢复出厂设置
#define FSUS_CMD_READ_DATA      0x03    // 读数据
#define FSUS_CMD_WRITE_DATA     0x04    // 写数据
#define FSUS_CMD_WHEEL_MODE     0x07    // 轮式连续旋转模式
#define FSUS_CMD_SET_ANGLE      0x08    // 单圈角度控制 (指定时间)
#define FSUS_CMD_SET_DAMPING    0x09    // 阻尼模式 / 掉电卸力
#define FSUS_CMD_QUERY_ANGLE    0x0A    // 读取舵机实时角度
#define FSUS_CMD_SET_ANGLE_INTERVAL 0x11// 梯形加减速角度控制
#define FSUS_CMD_SET_ANGLE_MTURN    0x0D// 多圈绝对角度控制

// 状态返回值
#define FSUS_STATUS_SUCCESS     0
#define FSUS_STATUS_TIMEOUT     1
#define FSUS_STATUS_CHECKSUM_ERR 2
#define FSUS_STATUS_ID_MISMATCH 3

// ================= 函数接口声明 =================

/**
 * @brief 总线舵机模块初始化
 * @param serialPort 使用的 HardwareSerial (如 &Serial2)
 * @param baudrate 通信波特率 (默认 115200)
 * @param rxPin RX 引脚号 (默认 16)
 * @param txPin TX 引脚号 (默认 15)
 */
void Servo_Init(HardwareSerial *serialPort = &Serial2, uint32_t baudrate = SERVO_UART_BAUDRATE, int rxPin = SERVO_UART_RX_PIN, int txPin = SERVO_UART_TX_PIN);

/**
 * @brief 设置舵机单圈绝对角度 (最常用接口)
 * @param servoId 舵机 ID (0 ~ 254, 0xFE 为广播)
 * @param angle 目标角度 (-135.0° ~ 135.0°，精度 0.1°)
 * @param interval 运动总耗时 (单位: ms, 0 表示尽快到达)
 * @param power 功率限制 (0~1000 mW，0 为舵机默认最大功率)
 */
void Servo_SetAngle(uint8_t servoId, float angle, uint16_t interval = 0, uint16_t power = 0);

/**
 * @brief 多圈绝对角度控制 (HA8-U25H-M 磁编码专用，支持 ±1024 圈)
 * @param servoId 舵机 ID
 * @param angle 目标多圈角度 (-368640.0° ~ 368640.0°)
 * @param interval 运行时间 (ms)
 * @param power 功率限制 (mW)
 */
void Servo_SetAngleMTurn(uint8_t servoId, float angle, uint32_t interval = 0, uint16_t power = 0);

/**
 * @brief 梯形加减速角度控制 (平滑平稳运动)
 * @param servoId 舵机 ID
 * @param angle 目标角度 (-135.0° ~ 135.0°)
 * @param interval 运行总时间 (ms, 须满足 interval > t_acc + t_dec)
 * @param t_acc 加速时间 (ms, 最小 > 20ms)
 * @param t_dec 减速时间 (ms, 最小 > 20ms)
 * @param power 功率限制 (mW)
 */
void Servo_SetAngleByInterval(uint8_t servoId, float angle, uint16_t interval, uint16_t t_acc, uint16_t t_dec, uint16_t power = 0);

/**
 * @brief 读取舵机当前实时角度 (阻塞式读取)
 * @param servoId 舵机 ID
 * @param currentAngle 输出角度引用 (-135.0° ~ 135.0°)
 * @param timeoutMs 超时时间 (默认 50ms)
 * @return true 读取成功, false 超时或校验失败
 */
bool Servo_QueryAngle(uint8_t servoId, float &currentAngle, uint32_t timeoutMs = 50);

/**
 * @brief 检测舵机是否在线 (Ping)
 * @param servoId 舵机 ID
 * @param timeoutMs 超时时间 (默认 50ms)
 * @return true 在线, false 离线
 */
bool Servo_Ping(uint8_t servoId, uint32_t timeoutMs = 50);

/**
 * @brief 进入阻尼模式 / 卸力 / 掉电
 * @param servoId 舵机 ID
 * @param power 维持阻尼阻力 (0 为完全自由，数值越大阻尼感越强，最大 1000)
 */
void Servo_SetDamping(uint8_t servoId, uint16_t power = 0);

/**
 * @brief 轮式连续旋转模式 (可作电机用)
 * @param servoId 舵机 ID
 * @param speed 旋转速度 (单位: °/s, 正负代表方向, 0 为停止)
 */
void Servo_SetWheelMode(uint8_t servoId, float speed);

/**
 * @brief 停止舵机运动 (保持当前角度)
 * @param servoId 舵机 ID
 */
void Servo_Stop(uint8_t servoId);

#endif // FASHIONSTAR_SERVO_H
