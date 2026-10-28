#pragma once
#include <Arduino.h>

// 与底盘共用 Serial1（115200，TX17/RX18）。驱动器需设置为16细分。
constexpr uint8_t TURRET_MOTOR_ID = 7;
constexpr float TURRET_MOTOR_PULSES_PER_REV = 3200.0f;
constexpr float TURRET_MOTOR_GEAR_RATIO = 1.0f; // 电机圈数 / 转盘圈数
constexpr float TURRET_MOTOR_SPEED_DEG_S = 500.0f;
constexpr uint8_t TURRET_MOTOR_ACCELERATION = 126; // 相对acc=100，物理角加速度提高20%：156/130=1.2。
constexpr uint8_t TURRET_MOTOR_POSITIVE_DIR = 1; // 转盘正角度为CCW，与原舵机方向一致
constexpr uint32_t TURRET_QUERY_GAP_MS = 5; // 连续总线通信时，查询前留出的验证性帧间隔

// 用已保存机械零位恢复绝对坐标；电机每次上电须位于机械0~180度。
struct TurretMotorState {float angle=0;int16_t rpm=0;uint8_t flags=0;uint16_t encoder=0;int64_t position=0;};
bool TurretMotorReadState(TurretMotorState &state);
bool TurretMotorReadAngle(float &angle);
bool TurretMotorMove(float angle, uint16_t rampMs);
void TurretMotorPoll();
bool TurretMotorTracking();
void TurretMotorCancelTrace();
// 清零当前电机坐标，并将单圈编码器零位保存到主控NVS。
bool TurretMotorSetZero(float &angle);

// 固定观察窗口仅用于调参记录，不以到位标志提前结束。
bool TurretMotorTrial(float angle,float speedDegS,uint16_t rampMs,uint32_t periodMs,uint32_t durationMs);
uint32_t TurretMotorNextDelay();
