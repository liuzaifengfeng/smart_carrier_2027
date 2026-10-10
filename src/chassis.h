#ifndef CHASSIS_H
#define CHASSIS_H

#include <Arduino.h>
#include <initializer_list>
#include "vision_alignment.h"

constexpr float ARM_HEIGHT_LIMIT_MM = 175.0f; // Maximum arm height (mm).
// 总任务超时后锁止 MoveArm 的后续发令，上电/重新进入 Release 才清除。
extern volatile bool taskMotionAborted;
// 2 号转台舵机的真实多圈角度边界，由线缆可运动范围决定。
constexpr float TURRET_CABLE_MIN_DEG = -180.0f;
constexpr float TURRET_CABLE_MAX_DEG = 360.0f;

// 机器人位姿结构体(世界坐标, 单位 mm / 度)
// 注: 定位(如何获得真实位姿)方案待团队定, 本模块只维护运动原语与理想位姿
struct RobotPose {
    float x;        // X (mm)
    float y;        // Y (mm)
    float theta;    // 航向角 (0-360, 度)
};
//机械臂位姿结构体(世界坐标, 单位 mm / 度)
struct ArmPose {
    float high;     // 高度 (mm)
    float length;      // 长度 (mm)
    float turret_angle;      // 转台角度 (度)
    float pawl_angle;      // 夹爪角度 (度)
};

extern RobotPose currentPose;   // 主控维护的"理想位姿"
extern ArmPose currentArm;     // 升降/伸缩为最近下发的绝对目标，非驱动器实测位置。

// 运动标定系数
extern float X_PULSE;      // X向 每毫米脉冲
extern float Y_PULSE;      // Y向 每毫米脉冲
extern float THETA_PULSE;  // 旋转 每度脉冲
extern float HEIGHT_PULSE; // 升降机械臂 每毫米脉冲
extern float LENGTH_PULSE; // 伸缩机械臂 每毫米脉冲


// ================= 运动原语(麦克纳姆轮, 通用) =================

// @brief 速度模式移动. direction 0=前进 1=后退 2=左移 3=右移 4=左转 5=右转, speed=RPM, stop=true停止
void MovePose(int direction, float speed, bool stop);

// @brief 位置模式移动到位.
// @param x 目标 X 位移 (mm)
// @param y 目标 Y 位移 (mm)
// @param theta 目标航向 (0-360, 度)
// @param isRelative 1相对移动 / 0绝对移动(需定位)
void GotoPose(float x, float y, float theta, bool isRelative);

/**
 * @brief 相对位置移动：四轮位置命令同步启动，平移与旋转同时完成。
 * @param x 起始车身坐标系 X 位移 (mm)，正方向沿用 GotoPose
 * @param y 起始车身坐标系 Y 位移 (mm)，正方向沿用 GotoPose
 * @param theta 本次转角 (deg)，正值左转；可为负值
 * @param speed 最大轮速 (RPM)，1~5000，驱动器按整数 RPM 执行
 * 阻塞当前任务至估算时间结束。混合运动沿恒定车身速度的圆弧到达终点；
 * 全圈旋转同时平移等无法用单段圆弧表示的请求返回 false。
 * 调用期间须独占底盘，不能同时使用 MovePose/OmniMove/视觉对齐。
 * @return true 命令已下发并等待结束（开环估计），false 参数无效或超范围。
 */
// shouldStop 可在等待期间轮询外部事件；返回 true 时立即停车并返回 false，
// 中断时不将理想位姿更新为目标点。省略回调时保持原有行为。
bool MovePosition(float x, float y, float theta, float speed,
                  bool (*shouldStop)() = nullptr);

/**
 * @brief 麦克纳姆轮全向速度控制。
 *
 * 三个速度参数是车身坐标系中的归一化权重，范围 -1~1；函数进行四轮
 * 速度混合，并在组合量超过 1 时等比例归一化，保持运动方向不变。
 *
 * @param xVelocity        车身 X 方向速度权重，正值与 GotoPose X 正方向一致
 * @param yVelocity        车身 Y 方向速度权重，正值与 GotoPose Y 正方向一致
 * @param rotationVelocity 旋转速度权重，正值与 GotoPose theta 正方向一致
 * @param speedRpm         最大轮速，范围 0~5000 RPM；0 表示停车
 */
void OmniMove(float xVelocity, float yVelocity, float rotationVelocity,
              uint16_t speedRpm);

/**
 * @brief 使用一帧视觉误差更新连续 PID 对齐速度。
 *
 * 摄像头与车身约呈 90°，因此视觉 Y 映射到底盘 X，视觉 X 映射到底盘 Y；
 * 三个 PID 输出会在同一周期合成为全向运动速度。
 *
 * @param angleErrorDeg 视觉角度偏差 (deg)
 * @param visualXError  视觉 X 偏差
 * @param visualYError  视觉 Y 偏差
 * @param dtSeconds     与上一视觉帧的时间间隔 (s)
 * @param speedRpm      PID 满输出时的最大轮速 (RPM)
 * @return true 三个误差均进入允许范围；false 仍在对齐
 */
bool AlignToDiscContinuous(float angleErrorDeg, float visualXError,
                           float visualYError, float dtSeconds,
                           uint16_t speedRpm = 80);

// 清除连续对齐 PID 的积分/微分历史；不会自行发送停车命令。
void ResetDiscAlignmentPid();

// 通用单帧 PID：按模式使用独立 X/Y/角度死区，true 只代表这一帧进入死区。
bool AlignToVisionContinuous(VisionStartMode mode, float angleErrorDeg,
                             float visualXError, float visualYError,
                             float dtSeconds, uint16_t speedRpm = 80);

// 非阻塞等待：仅在新帧到达时调用，同时更新 PID；连续 5 帧进入死区返回 true。
// 调用方应在启停、断流、反馈丢帧或无效反馈时调用 ResetAlignmentWait。
bool WaitForAlignment(VisionStartMode mode, float angleErrorDeg,
                      float visualXError, float visualYError,
                      float dtSeconds, uint16_t speedRpm = 80);
void ResetAlignmentWait();

// @brief 机械臂移动到位.
// 升降/伸缩使用 EMM V5 绝对位置模式，以开机当前位置清零为原点；-1 跳过该轴。
// 返回值表示所请求的控制命令是否全部成功下发，不代表机构已物理到位。
bool MoveArm(float high, float length, float turret_angle, float pawl_angle, float speed);

// 开机圈数校验失败后锁住转台动作，直到修正姿态并重新上电。
void DisableTurretMotionUntilRestart(const char* reason);

// @brief 机械臂初始化归零位
bool InitArm_start();
bool InitArm_look();
bool InitArm_look2();
bool InitArm_look3();
bool InitArm_look4();

/**
 * @brief 按 0~24 号场地节点路径移动，只使用原地转向和车身 Y 轴前后直行。
 *
 * 路径首项表示小车当前所在节点；相邻输入节点必须在 5x5 蛇形节点图中上下或左右相邻。
 * 连续同向且共线的多段路径会自动合并，例如 1-2-3 合并为 1-3。
 * 每段选择前进/后退中转角较小的朝向，等角时优先前进；不做车身横向平移。
 * 反向路段先停车再换方向；后退时理想航向保持车头朝向，不改成行进方向。
 * 指定 finalHeading 时，最后一段优先最小化终点转角，平行时前进/后退直接到达。
 * 横竖路径配合 0/90/180/270 度终点朝向：平行时不补转，垂直时到达后补转 90 度。
 * 本函数会按脉冲数、目标转速和加速度档位估算完成时间，并物理阻塞调用任务。
 *
 * @param path         节点序号数组，例如 {0, 1, 8, 7}
 * @param pathLength   数组中的节点数量，可变长度且至少为 2
 * @param speedRpm     电机目标转速，范围 1~5000 RPM
 * @param acceleration EMM V5 加速度档位，0 表示直接启动
 * @param finalHeading 终点车头的场地角度，默认 NaN 不约束；有限角度归一化到 0~360 度
 * @return true 路径有效且全部运动指令已执行；false 参数或路径无效
 */
constexpr uint8_t FIELD_GRID_SIZE = 5;
constexpr uint8_t FIELD_NODE_COUNT = 25;
constexpr size_t MAX_NODE_PATH_LENGTH = 25;

bool MoveNodePath(const uint8_t *path, size_t pathLength,
                  uint16_t speedRpm = 80, uint8_t acceleration = 50,
                  float finalHeading = NAN);

// 可直接写 MoveNodePath({0, 1, 8, 7})，节点数量由初始化列表自动传入。
inline bool MoveNodePath(std::initializer_list<uint8_t> path,
                         uint16_t speedRpm = 80,
                         uint8_t acceleration = 50, float finalHeading = NAN) {
    return MoveNodePath(path.begin(), path.size(), speedRpm, acceleration, finalHeading);
}



#endif // CHASSIS_H
