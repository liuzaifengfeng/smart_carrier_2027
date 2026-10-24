#ifndef CHASSIS_H
#define CHASSIS_H

#include <Arduino.h>
#include <initializer_list>
#include "vision_alignment.h"

constexpr float ARM_HEIGHT_LIMIT_MM = 175.0f; // Maximum arm height (mm).
// 总任务超时后锁止 MoveArm 的后续发令，上电/重新进入 Release 才清除。
extern volatile bool taskMotionAborted;
// 2 号转台舵机的真实多圈角度边界，由线缆可运动范围决定。
constexpr float TURRET_CABLE_MIN_DEG = -150.0f;
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
// 只读访问已生效的地图配置，供独立试跑规划使用。
/**
 * @brief 按节点编号读取当前地图X/Y坐标(mm)，成功时写入x和y并返回true。
 */
bool ChassisGetNodePosition(uint8_t node,float &x,float &y);
extern ArmPose currentArm;     // 升降/伸缩为最近下发的绝对目标，非驱动器实测位置。

// 运动标定系数
extern float X_PULSE;      // X向 每毫米脉冲
extern float Y_PULSE;      // Y向 每毫米脉冲
extern float THETA_PULSE;  // 旋转 每度脉冲
extern float HEIGHT_PULSE; // 升降机械臂 每毫米脉冲
extern float LENGTH_PULSE; // 伸缩机械臂 每毫米脉冲


// ================= 运动原语(麦克纳姆轮, 通用) =================

// @brief 速度模式移动. direction 0=前进 1=后退 2=左移 3=右移 4=左转 5=右转, speed=RPM, stop=true停止
/**
 * @brief 按方向发送底盘速度命令；direction为前后左右及原地转向，speed单位RPM，stop=true停车。
 */
void MovePose(int direction, float speed, bool stop);

// @brief 位置模式移动到位.
// @param x 目标 X 位移 (mm)
// @param y 目标 Y 位移 (mm)
// @param theta 目标航向 (0-360, 度)
// @param isRelative 1相对移动 / 0绝对移动(需定位)
/**
 * @brief 使用旧位姿接口执行相对或绝对目标移动；x/y单位mm，theta单位deg。
 */
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
 * 契约：位移、转角、速度和标定系数有效，由输入入口负责校验。
 * @return true 命令已下发并等待结束（开环估计），false 被取消或请求无法用单段圆弧表示。
 */
bool MovePosition(float x, float y, float theta, float speed, bool (*shouldStop)() = nullptr);
/**
 * @brief 初始化四轮同步相对位置动作，不等待估算行程结束；由TickPositionMove推进。
 * @return true 准备成功，false 请求无法执行。
 */
bool BeginPositionMove(float x,float y,float theta,float speed);
/**
 * @brief 检查相对位置动作的估算时间并推进结束处理，不代表实测到位。
 */
void TickPositionMove();
/**
 * @brief 判断相对位置动作是否仍处于等待结束的运行状态。
 */
bool PositionMoveActive();
/**
 * @brief 停止当前相对位置移动并清除运行状态。
 */
void CancelPositionMove();
// 扫码直行：用四轮编码器记录完整行程或提前停车后的实际行程。
/**
 * @brief 同步执行扫码往返直行，用编码器记录完整或提前停车后的行程；distance单位mm，speed单位RPM。
 */
bool MoveScanPosition(float distance,float speed,bool (*shouldStop)() = nullptr);
/**
 * @brief 同步执行接近后降速扫码，距离单位mm，速度单位RPM；收到任务码回调为true时停车。
 */
bool MoveScanApproach(float approach,float search,uint16_t approachRpm,uint16_t searchRpm, bool (*received)());

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
void OmniMove(float xVelocity, float yVelocity, float rotationVelocity, uint16_t speedRpm);

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
bool AlignToDiscContinuous(float angleErrorDeg, float visualXError, float visualYError, float dtSeconds, uint16_t speedRpm = 80);

// 清除连续对齐 PID 的积分/微分历史；不会自行发送停车命令。
void ResetDiscAlignmentPid();

// 通用单帧 PID：按模式使用独立 X/Y/角度死区，true 只代表这一帧进入死区。
bool AlignToVisionContinuous(VisionStartMode mode, float angleErrorDeg, float visualXError, float visualYError, float dtSeconds, uint16_t speedRpm = 80);

// 非阻塞等待：仅在新帧到达时调用，同时更新 PID；连续 2 帧进入死区返回 true。
// 调用方应在启停、断流、反馈丢帧或无效反馈时调用 ResetAlignmentWait。
bool WaitForAlignment(VisionStartMode mode, float angleErrorDeg, float visualXError, float visualYError, float dtSeconds, uint16_t speedRpm = 80);
void ResetAlignmentWait();

// @brief 机械臂移动到位.
// 升降/伸缩使用 EMM V5 绝对位置模式，以开机当前位置清零为原点；-1 跳过该轴。
// speed 为有限且≥1的公共速度指令，无人为上限；升降/伸缩RPM仅按16位协议饱和；夹爪基于200≈90°/s的原标定计算后，将耗时缩短为1/5。
// 返回值表示所请求的控制命令是否全部成功下发，不代表机构已物理到位。
// 契约：目标在机械行程内，速度为正；RPM 在写入协议时饱和。
// 串口和参数入口负责校验；-1 跳过对应轴。
// slowLower仅将升降轴转速及实际加速度减半，伸缩、转台和夹爪保持原参数。
// turretRampMs 为2号转台加速、减速各自时间(ms)，由动作阶段传入；不改变夹爪参数。
bool MoveArm(float high, float length, float turret_angle, float pawl_angle, float speed, uint16_t turretRampMs=600, bool slowLower=false);

// 读取2号转台实际多圈角度并检查线缆范围；失败时不改写 angle，也不更新目标缓存。
/**
 * @brief 读取2号转台实际多圈角度(deg)，同时沿用现有线缆范围检查；成功写入angle并返回true。
 */
bool ReadArmTurretAngle(float &angle);

// 执行已规划的精确多圈转台角度，不重新选择等效角度；转台 -1 表示实际 -1 度。
// 其他三轴仍以 -1 跳过；速度、计时、保护和返回值含义与 MoveArm 相同。
/**
 * @brief 按已规划的多圈绝对转台目标发令，不再选择最近等效角；其他轴的-1仍表示跳过。
 * @param turretRampMs 转台加速、减速各自时长(ms)，第一层放料全程500，收料、第二层叠放及圆盘全程20。
 * @return true 指令下发成功，不表示实测到位。
 */
bool MoveArmPlanned(float high, float length, float turret_angle, float pawl_angle, float speed, uint16_t turretRampMs=600, bool slowLower=false);

// @brief 机械臂初始化归零位
/**
 * @brief 同步提交并等待START收臂姿态，供旧调试入口调用。
 */
bool InitArm_start();
/**
 * @brief 同步提交并等待工位视觉前LOOK姿态，供旧调试入口调用。
 */
bool InitArm_look();
/**
 * @brief 同步提交并等待圆盘整体对齐前LOOK2姿态。
 */
bool InitArm_look2();
/**
 * @brief 同步提交并等待圆盘单件识别前LOOK3姿态及转台实测就绪。
 */
bool InitArm_look3();
/**
 * @brief 同步提交并等待返家角点视觉前LOOK4姿态。
 */
bool InitArm_look4();

/**
 * @brief 按0~24节点导航：全部路段位置模式，到节点停车转向，不漂移切弯。
 *
 * 路径首项表示小车当前所在节点；相邻输入节点必须在 5x5 蛇形节点图中上下或左右相邻。
 * 连续同向且共线的多段路径会自动合并，例如 1-2-3 合并为 1-3。
 * 每段选择前进/后退中转角较小的朝向，外侧与中间十字逻辑一致。
 * 反向路段先停车再换方向；后退时理想航向保持车头朝向，不改成行进方向。
 * 仅下发一次有限位置目标及驱动加速度，不使用外圈连续曲线控制。
 * 沿用实际位置容差及近零转速接续，不额外添加稳定窗口。
 * 路径起步清零一次，后续累计绝对目标，避免每段重复清零、停车和等待。
 * 运动中H30差动轮速纠偏；独立校正默认±3度死区，小残差保留低速恢复。
 * 首节点XY仍为调用方明确给出的假设，编码器不能消除轮胎打滑造成的地图位置误差。
 * H30缺项/失联/参考变化时退回编码器控制，不中断任务；真实运动异常仍返回失败。
 *
 * @param path         节点序号数组，例如 {0, 1, 8, 7}
 * @param pathLength   数组中的节点数量，2~25
 * @param speedMmS     平移速度上限，范围10~5000mm/s；转向速度/加速度可在参数页独立调整
 * @param acceleration 保留旧接口参数；新节点控制器不使用此位置模式档位
 * @param finalHeading 终点车头的场地角度，默认 NaN 不约束；有限角度归一化到 0~360 度
 * @return true 路径有效且全部运动指令已执行；false 参数或路径无效
 */
constexpr uint8_t FIELD_GRID_SIZE = 5;
constexpr uint8_t FIELD_NODE_COUNT = 25;
constexpr size_t MAX_NODE_PATH_LENGTH = 25;

bool MoveNodePath(const uint8_t *path, size_t pathLength, float speedMmS = 1200, uint8_t acceleration = 0, float finalHeading = NAN);

// 可直接写 MoveNodePath({0, 1, 8, 7})，节点数量由初始化列表自动传入。
inline bool MoveNodePath(std::initializer_list<uint8_t> path, float speedMmS = 1200, uint8_t acceleration = 0, float finalHeading = NAN) {
    return MoveNodePath(path.begin(), path.size(), speedMmS, acceleration, finalHeading);
}



#endif // CHASSIS_H
