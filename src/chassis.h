#ifndef CHASSIS_H
#define CHASSIS_H

#include <Arduino.h>

// 机器人位姿结构体(世界坐标, 单位 mm / 度)
// 注: 定位(如何获得真实位姿)方案待团队定, 本模块只维护运动原语与理想位姿
struct RobotPose {
    float x;        // X (mm)
    float y;        // Y (mm)
    float theta;    // 航向角 (0-360, 度)
};
extern RobotPose currentPose;   // 主控维护的"理想位姿"

// 运动标定系数(新底盘需重新标定) [TODO]
extern float X_PULSE;      // X向 每毫米脉冲
extern float Y_PULSE;      // Y向 每毫米脉冲
extern float THETA_PULSE;  // 旋转 每度脉冲
extern float HEIGHT_PULSE; // 升降 每毫米脉冲

// ================= 运动原语(麦克纳姆轮, 通用) =================
void movepose(bool forward, float speed, bool stop);
// @brief 速度模式直线移动. forward=前进/后退, speed=mm/s, stop=true停止

void GotoPose(float x, float y, float theta, bool isRelative, bool isAdjust);
// @brief 位置模式移动到位.
//        isRelative=true 相对移动 / false 绝对移动(需定位)
//        isAdjust=是否仅微调(不更新理想位姿)

#endif // CHASSIS_H
