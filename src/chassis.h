#ifndef CHASSIS_H
#define CHASSIS_H

#include <Arduino.h>
#include <initializer_list>

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
extern ArmPose currentArm;     // 主控维护的"理想臂位姿"

// 运动标定系数
extern float X_PULSE;      // X向 每毫米脉冲
extern float Y_PULSE;      // Y向 每毫米脉冲
extern float THETA_PULSE;  // 旋转 每度脉冲
extern float HEIGHT_PULSE; // 升降机械臂 每毫米脉冲
extern float LENGTH_PULSE; // 伸缩机械臂 每毫米脉冲


// ================= 运动原语(麦克纳姆轮, 通用) =================

// @brief 速度模式移动. direction 0=前进 1=后退 2=左移 3=右移, speed=mm/s, stop=true停止
void MovePose(int direction, float speed, bool stop);

// @brief 位置模式移动到位.
//        isRelative=true 相对移动 / false 绝对移动(需定位)
void GotoPose(float x, float y, float theta, bool isRelative);

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

// @brief 机械臂移动到位.
void MoveArm(float high, float length, float turret_angle, float pawl_angle, float speed);

// @brief 机械臂初始化归零位
void InitArm();

/**
 * @brief 按 1~9 号场地节点路径移动，只使用原地转向和向前直行。
 *
 * 路径首项表示小车当前所在节点；相邻输入节点必须在 3x3 节点图中上下或左右相邻。
 * 连续同向且共线的多段路径会自动合并，例如 1-2-3 合并为 1-3。
 * 本函数会按脉冲数、目标转速和加速度档位估算完成时间，并物理阻塞调用任务。
 *
 * @param path         节点序号数组，例如 {1, 2, 5, 4}
 * @param pathLength   数组中的节点数量，可变长度且至少为 2
 * @param speedRpm     电机目标转速，范围 1~5000 RPM
 * @param acceleration EMM V5 加速度档位，0 表示直接启动
 * @return true 路径有效且全部运动指令已执行；false 参数或路径无效
 */
bool MoveNodePath(const uint8_t *path, size_t pathLength,
                  uint16_t speedRpm = 80, uint8_t acceleration = 50);

// 可直接写 MoveNodePath({1, 2, 5, 4})，节点数量由初始化列表自动传入。
inline bool MoveNodePath(std::initializer_list<uint8_t> path,
                         uint16_t speedRpm = 80,
                         uint8_t acceleration = 50) {
    return MoveNodePath(path.begin(), path.size(), speedRpm, acceleration);
}



#endif // CHASSIS_H
