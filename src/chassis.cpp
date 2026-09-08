#include "chassis.h"
#include "Emm_V5.h"
#include "servo.h"

// 理想位姿(由主控维护; 真实位姿的获取方案待定)
//车体位姿——X坐标、Y坐标、Theta角度
RobotPose currentPose = {0, 0, 0};//X,Y,Theta
//机械臂位姿——大臂高度、小臂伸出长度、转台角度、夹爪角度
ArmPose currentArm = {0, 0, 0, 0};//high,length,turret_angle,pawl_angle

/**
 * @brief 机械臂移动到指定位姿
 * @param high 大臂高度 (mm)
 * @param length 小臂伸出长度 (mm)
 * @param turret_angle 转台角度 (°)
 * @param pawl_angle 夹爪角度 (°)
 * @param speed 速度 (mm/s)
 * @note -1 表示不操作该轴
 */
void MoveArm(float high, float length, float turret_angle, float pawl_angle, float speed) {

        if (currentArm.high - high != 0 && high != -1) {
            if( high < 0 || high > 200){//行程保护
                Serial.println("high out of range");
            } else {
                uint8_t dir = (currentArm.high - high > 0) ? 0 : 1;
                uint32_t pulses = (uint32_t)(fabsf(currentArm.high - high) * HEIGHT_PULSE);
                Emm_V5_Pos_Control(5, dir, speed, 50, pulses, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
                currentArm.high = high;
            }
        }

        if (currentArm.length - length != 0 && length != -1) {
            if( length < 0 || length > 170){//行程保护
                Serial.println("length out of range");
            } else {
                uint8_t dir = (currentArm.length - length > 0) ? 0 : 1;
                uint32_t pulses = (uint32_t)(fabsf(currentArm.length - length) * LENGTH_PULSE);
                Emm_V5_Pos_Control(6, dir, speed, 50, pulses, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
                currentArm.length = length;
            }
        }
        
        if(currentArm.turret_angle - turret_angle != 0 && turret_angle != -1) {
            if( turret_angle < -360 || turret_angle > 360){//行程保护
                Serial.println("turret_angle out of range");
            } else {
                Servo_SetAngleMTurn(1, turret_angle, speed, 0);
                currentArm.turret_angle = turret_angle;
            }
        }

        if(currentArm.pawl_angle - pawl_angle != 0 && pawl_angle != -1) {
            if( pawl_angle < -360 || pawl_angle > 360){//行程保护
                Serial.println("pawl_angle out of range");
            } else {
                Servo_SetAngleMTurn(2, pawl_angle, speed, 0);
                currentArm.pawl_angle = pawl_angle;
            }
        }

}


/**
 * @brief 速度模式直线移动(麦克纳姆轮, 通用)
 * @param forward true=前进 false=后退
 * @param speed   速度 (mm/s)
 * @param stop    true=停止 false=开始移动
 */
void MovePose(bool forward, float speed, bool stop) {
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

    vTaskDelay(pdMS_TO_TICKS(5));

    if (forward) { // 前进
        Emm_V5_Vel_Control(1, 1, speed, 50, 0);
        vTaskDelay(pdMS_TO_TICKS(5));
        Emm_V5_Vel_Control(2, 0, speed, 50, 0);
        vTaskDelay(pdMS_TO_TICKS(5));
        Emm_V5_Vel_Control(3, 1, speed, 50, 0);
        vTaskDelay(pdMS_TO_TICKS(5));
        Emm_V5_Vel_Control(4, 0, speed, 50, 0);
        vTaskDelay(pdMS_TO_TICKS(5));
    } else {       // 后退
        Emm_V5_Vel_Control(1, 0, speed, 50, 0);
        vTaskDelay(pdMS_TO_TICKS(5));
        Emm_V5_Vel_Control(2, 1, speed, 50, 0);
        vTaskDelay(pdMS_TO_TICKS(5));
        Emm_V5_Vel_Control(3, 0, speed, 50, 0);
        vTaskDelay(pdMS_TO_TICKS(5));
        Emm_V5_Vel_Control(4, 1, speed, 50, 0);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    isMoving = true;
}

/**
 * @brief 位置模式移动到位(麦克纳姆轮运动学, 通用)
 * @param isRelative true=相对坐标 false=绝对坐标
 * 注意: 4 轮同向=平移, 4 轮同转向=原地旋转. 具体轮序(direction)
 *       需按实际电机接线确认. [TODO]
 */
void GotoPose(float x, float y, float theta, bool isRelative) {
    int speed = 80;   // 移动速度

    if (isRelative) {
        // 平移 X (同向差速)
        if (x != 0) {
            uint8_t dir = (x > 0) ? 0 : 1;
            uint32_t pulses = (uint32_t)(fabsf(x) * X_PULSE);
            Emm_V5_Pos_Control(1, dir, speed, 50, pulses, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(5));
            Emm_V5_Pos_Control(2, dir, speed, 50, pulses, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(5));
            Emm_V5_Pos_Control(3, !dir, speed, 50, pulses, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(5));
            Emm_V5_Pos_Control(4, !dir, speed, 50, pulses, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        // 平移 Y
        if (y != 0) {
            uint8_t dir = (y > 0) ? 0 : 1;
            uint32_t pulses = (uint32_t)(fabsf(y) * Y_PULSE);
            Emm_V5_Pos_Control(1, !dir, speed, 50, pulses, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(5));
            Emm_V5_Pos_Control(2,  dir, speed, 50, pulses, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(5));
            Emm_V5_Pos_Control(3, !dir, speed, 50, pulses, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(5));
            Emm_V5_Pos_Control(4,  dir, speed, 50, pulses, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        // 原地旋转
        if (theta != 0) {
            uint8_t dir = (theta < 0) ? 0 : 1;
            uint32_t pulses = (uint32_t)(fabsf(theta) * THETA_PULSE);
            Emm_V5_Pos_Control(1, dir, speed, 50, pulses, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(5));
            Emm_V5_Pos_Control(2, dir, speed, 50, pulses, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(5));
            Emm_V5_Pos_Control(3, dir, speed, 50, pulses, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(5));
            Emm_V5_Pos_Control(4, dir, speed, 50, pulses, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
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