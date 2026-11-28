// 机械臂转盘反馈、移动和试验接口。
// 转盘角度和驱动器脉冲位置是不同表示，量化接口负责两者转换。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include <Arduino.h>  // 引入 Arduino.h 中的声明和定义。

// 与底盘共用 Serial1（115200，TX17/RX18）。驱动器需设置为16细分。
constexpr uint8_t TURRET_MOTOR_ID = 7;  // 将 7 写入或初始化 TURRET_MOTOR_ID（转台MOTOR编号）。
constexpr float TURRET_MOTOR_PULSES_PER_REV = 3200.0f;  // 将 3200.0f 写入或初始化 TURRET_MOTOR_PULSES_PER_REV（转台MOTOR脉冲数PERREV）。
constexpr float TURRET_MOTOR_GEAR_RATIO = 1.0f; // 电机圈数 / 转盘圈数
constexpr float TURRET_MOTOR_SPEED_DEG_S = 500.0f;  // 将 500.0f 写入或初始化 TURRET_MOTOR_SPEED_DEG_S（转台MOTOR速度角度S）。
constexpr uint16_t TURRET_MOTOR_POSITION_RPM =  // 续写当前声明或运算，提供 constexpr uint16_t TURRET_MOTOR_POSITION_RPM =。
    static_cast<uint16_t>(TURRET_MOTOR_SPEED_DEG_S * TURRET_MOTOR_GEAR_RATIO / 6.0f + 0.5f);  // 声明接口 uint16_t>，参数由此处的签名确定。
constexpr uint8_t TURRET_ACCELERATION_SLOW = 135;  // 将 135 写入或初始化 TURRET_ACCELERATION_SLOW（转台加速度SLOW）。
constexpr uint8_t TURRET_ACCELERATION_FAST = 160;  // 将 160 写入或初始化 TURRET_ACCELERATION_FAST（转台加速度FAST）。
constexpr uint8_t TURRET_ACCELERATION_CARGO_PLACE = 160; // 持料转向旋转料盘。
constexpr uint8_t TURRET_MOTOR_POSITIVE_DIR = 1; // 转盘正角度为CCW，与原舵机方向一致
constexpr uint32_t TURRET_QUERY_GAP_MS = 5; // 连续总线通信时，查询前留出的验证性帧间隔

// 用已保存机械零位恢复绝对坐标；电机每次上电须位于机械0~180度。
struct TurretMotorState {float angle=0;int16_t rpm=0;uint8_t flags=0;uint16_t encoder=0;int64_t position=0;};  // 定义 TurretMotorState（转台Motor状态） 的类型成员或作用域。
bool TurretMotorReadState(TurretMotorState &state);  // 声明接口 TurretMotorReadState（转台MotorRead状态），参数由此处的签名确定。
bool TurretMotorReadAngle(float &angle);  // 声明接口 TurretMotorReadAngle（转台MotorRead角度），参数由此处的签名确定。
bool TurretMotorMove(float angle, uint8_t acceleration, uint16_t rpm=TURRET_MOTOR_POSITION_RPM);  // 声明接口 TurretMotorMove（转台MotorMove），参数由此处的签名确定。
struct TurretMotorPosition { uint8_t direction; uint32_t pulses; };  // 定义 TurretMotorPosition（转台Motor位置） 的类型成员或作用域。
TurretMotorPosition TurretMotorQuantizePosition(float angle);  // 声明接口 TurretMotorQuantizePosition（转台MotorQuantize位置），参数由此处的签名确定。
bool TurretMotorMoveQuantized(float angle,const TurretMotorPosition &position,uint8_t acceleration,  // 定义函数或成员 TurretMotorMoveQuantized（转台MotorMoveQuantized），参数由此处的签名确定。
                             uint16_t rpm,uint32_t &commandId);  // 声明 rpm（转速），类型和可用范围由所在声明决定。
bool TurretMotorCommandCancelled(uint32_t commandId);  // 声明接口 TurretMotorCommandCancelled（转台MotorCommand已取消），参数由此处的签名确定。
bool TurretMotorCommandActive(uint32_t commandId);  // 声明接口 TurretMotorCommandActive（转台MotorCommand活跃），参数由此处的签名确定。
void TurretMotorPoll();  // 声明接口 TurretMotorPoll（转台MotorPoll），参数由此处的签名确定。
bool TurretMotorTracking();  // 声明接口 TurretMotorTracking（转台MotorTracking），参数由此处的签名确定。
void TurretMotorCancelTrace();  // 声明接口 TurretMotorCancelTrace（转台MotorCancelTrace），参数由此处的签名确定。
// 清零当前电机坐标，并将单圈编码器零位保存到主控NVS。
bool TurretMotorSetZero(float &angle);  // 声明接口 TurretMotorSetZero（转台MotorSet零点），参数由此处的签名确定。

// 固定观察窗口仅用于调参记录，不以到位标志提前结束。
bool TurretMotorTrial(float angle,float speedDegS,uint16_t rampMs,uint32_t periodMs,uint32_t durationMs);  // 声明接口 TurretMotorTrial（转台MotorTrial），参数由此处的签名确定。
uint32_t TurretMotorNextDelay();  // 声明接口 TurretMotorNextDelay（转台Motor下一延时），参数由此处的签名确定。
