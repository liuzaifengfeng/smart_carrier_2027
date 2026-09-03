#include "chassis.h"
#include "Emm_V5.h"

// 理想位姿(由主控维护; 真实位姿的获取方案待定)
RobotPose currentPose = {0, 0, 0};

/**
 * @brief 速度模式直线移动(麦克纳姆轮, 通用)
 * @param forward true=前进 false=后退
 * @param speed   速度 (mm/s)
 * @param stop    true=停止 false=开始移动
 */
void movepose(bool forward, float speed, bool stop) {
    static bool isMoving = false;

    if (stop) {
        if (isMoving) {
            Emm_V5_Stop_Now(0, 0);
            vTaskDelay(pdMS_TO_TICKS(5));
            // [TODO] 停止后按实际反馈更新位姿(如编码器/里程计/视觉)
            isMoving = false;
            Serial.println("move stopped");
        }
        return;
    }

    if (forward) { // 前进
        Emm_V5_Vel_Control(1, 0, speed, 50, 1);
        Emm_V5_Vel_Control(2, 0, speed, 50, 1);
        Emm_V5_Vel_Control(3, 1, speed, 50, 1);
        Emm_V5_Vel_Control(4, 1, speed, 50, 1);
    } else {       // 后退
        Emm_V5_Vel_Control(1, 1, speed, 50, 0);
        Emm_V5_Vel_Control(2, 1, speed, 50, 1);
        Emm_V5_Vel_Control(3, 0, speed, 50, 1);
        Emm_V5_Vel_Control(4, 0, speed, 50, 1);
    }
    Emm_V5_Synchronous_motion(0);
    isMoving = true;
}

/**
 * @brief 位置模式移动到位(麦克纳姆轮运动学, 通用)
 * @param isRelative true=相对坐标 false=绝对坐标
 * @param isAdjust   微调标志(是否更新理想位姿)
 * 注意: 4 轮同向=平移, 4 轮同转向=原地旋转. 具体轮序(direction)
 *       需按实际电机接线确认. [TODO]
 */
void GotoPose(float x, float y, float theta, bool isRelative, bool isAdjust) {
    int speed = 80;   // 移动速度

    if (isRelative) {
        // 平移 X (同向差速)
        if (x != 0) {
            uint8_t dir = (x > 0) ? 0 : 1;
            uint32_t pulses = (uint32_t)(fabsf(x) * X_PULSE);
            Emm_V5_Pos_Control(1, dir,          speed, 50, pulses, 0, 1);
            Emm_V5_Pos_Control(2, dir==0?1:0,    speed, 50, pulses, 0, 1);
            Emm_V5_Pos_Control(3, dir,          speed, 50, pulses, 0, 1);
            Emm_V5_Pos_Control(4, dir==0?1:0,    speed, 50, pulses, 0, 1);
            Emm_V5_Synchronous_motion(0);
        }
        // 平移 Y
        if (y != 0) {
            uint8_t dir = (y > 0) ? 0 : 1;
            uint32_t pulses = (uint32_t)(fabsf(y) * Y_PULSE);
            Emm_V5_Pos_Control(1, dir, speed, 50, pulses, 0, 1);
            Emm_V5_Pos_Control(2, dir, speed, 50, pulses, 0, 1);
            Emm_V5_Pos_Control(3, dir==0?1:0, speed, 50, pulses, 0, 1);
            Emm_V5_Pos_Control(4, dir==0?1:0, speed, 50, pulses, 0, 1);
            Emm_V5_Synchronous_motion(0);
        }
        // 原地旋转
        if (theta != 0) {
            uint8_t dir = (theta > 0) ? 0 : 1;
            uint32_t pulses = (uint32_t)(fabsf(theta) * THETA_PULSE);
            Emm_V5_Pos_Control(1, dir, speed, 50, pulses, 0, 1);
            Emm_V5_Pos_Control(2, dir, speed, 50, pulses, 0, 1);
            Emm_V5_Pos_Control(3, dir, speed, 50, pulses, 0, 1);
            Emm_V5_Pos_Control(4, dir, speed, 50, pulses, 0, 1);
            Emm_V5_Synchronous_motion(0);
        }
    } else {
        // 绝对坐标: 需先获取当前位姿, 算出位移增量再调用相对移动.
        // [TODO] 依赖新定位方案, 由负责定位的成员实现
        // 例:
        //   RobotPose p = currentPose;   // 或定位融合结果
        //   GotoPose(x - p.x, y - p.y, theta - p.theta, true, false);
        Serial.println("[TODO] GotoPose absolute needs localization");
    }
}
