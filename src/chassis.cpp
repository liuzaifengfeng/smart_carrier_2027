#include "chassis.h"
#include "Emm_V5.h"
#include "servo.h"

// 理想位姿(由主控维护; 真实位姿的获取方案待定)
//车体位姿——X坐标、Y坐标、Theta角度
RobotPose currentPose = {0, 0, 0};//X,Y,Theta
//机械臂位姿——大臂高度、小臂伸出长度、转台角度、夹爪角度
ArmPose currentArm = {0, 0, 0, 0};//high,length,turret_angle,pawl_angle

namespace {

// ================= 节点路径移动的内部数据与参数 =================

// 地图节点的场地绝对坐标，单位为毫米。
struct NodePosition {
    float x;  // X 坐标
    float y;  // Y 坐标
};

// 数组下标比节点序号小 1：
// 第一行是节点 1、2、3，第二行是节点 4、5、6，第三行是节点 7、8、9。
// 节点顺序与 Python 上位机地图完全一致。
constexpr NodePosition NODE_POSITIONS[9] = {
    {400.0f,  400.0f}, {1200.0f,  400.0f}, {2000.0f,  400.0f},
    {400.0f, 1200.0f}, {1200.0f, 1200.0f}, {2000.0f, 1200.0f},
    {400.0f, 2000.0f}, {1200.0f, 2000.0f}, {2000.0f, 2000.0f},
};

constexpr uint32_t MOTOR_PULSES_PER_REVOLUTION = 3200; // 16 细分时，电机转一圈的脉冲数
constexpr uint32_t MOTOR_COMMAND_GAP_MS = 5;           // 连续发送两条电机命令的间隔
constexpr uint32_t ROUTE_SETTLE_TIME_MS = 250;         // 每次运动结束后的停车稳定时间
constexpr float ROUTE_ANGLE_EPSILON_DEG = 0.01f;       // 小于该角度时不再执行转向

// 根据节点序号读取坐标。序号只能是 1~9。
bool getNodePosition(uint8_t node, NodePosition &position) {
    if (node < 1 || node > 9) {
        return false;
    }
    position = NODE_POSITIONS[node - 1];
    return true;
}

// 判断两个节点在 3×3 节点图中是否上下或左右相邻。
// 例如 1 与 2 相邻、2 与 5 相邻，但 1 与 5 不相邻。
bool areAdjacentNodes(uint8_t first, uint8_t second) {
    if (first < 1 || first > 9 || second < 1 || second > 9) {
        return false;
    }
    int firstIndex = first - 1;
    int secondIndex = second - 1;
    int columnDifference = abs(firstIndex % 3 - secondIndex % 3);
    int rowDifference = abs(firstIndex / 3 - secondIndex / 3);
    return columnDifference + rowDifference == 1;
}

// 判断 start -> middle -> end 是否是方向不变的一条直线。
// 叉积为 0 代表三点共线，点积大于 0 代表没有掉头。
// 例如 1-2-3 返回 true，而 1-2-1 返回 false。
bool continuesStraight(const NodePosition &start, const NodePosition &middle,
                       const NodePosition &end) {
    float firstX = middle.x - start.x;
    float firstY = middle.y - start.y;
    float secondX = end.x - middle.x;
    float secondY = end.y - middle.y;
    float cross = firstX * secondY - firstY * secondX;
    float dot = firstX * secondX + firstY * secondY;
    return fabsf(cross) < 0.001f && dot > 0.0f;
}

// 把任意航向角换算到 0~360 度范围。
float normalizeHeading(float heading) {
    heading = fmodf(heading, 360.0f);
    if (heading < 0.0f) {
        heading += 360.0f;
    }
    return heading;
}

// 计算当前航向转到目标航向所需的最短转角，结果范围为 -180~180 度。
float shortestTurn(float currentHeading, float targetHeading) {
    float turn = fmodf(targetHeading - currentHeading + 540.0f, 360.0f);
    return turn - 180.0f;
}

/**
 * EMM V5 说明书给出的加速规律：
 * 每增加 1 RPM 用时 (256 - acceleration) * 50 us。
 * 位置模式按对称加减速估算；短行程使用三角速度曲线。
 */
uint32_t estimateMotionTimeMs(uint32_t pulses, uint16_t speedRpm,
                              uint8_t acceleration) {
    if (pulses == 0 || speedRpm == 0) {
        return 0;
    }

    float revolutions =
        static_cast<float>(pulses) / MOTOR_PULSES_PER_REVOLUTION;
    float maximumSpeed = static_cast<float>(speedRpm) / 60.0f;
    float durationSeconds = 0.0f;

    if (acceleration == 0) {
        // 加速度档位为 0：驱动器直接达到目标速度，不计算加减速过程。
        durationSeconds = revolutions / maximumSpeed;
    } else {
        // 驱动器每增加 1 RPM 所需的秒数。
        float secondsPerRpm =
            static_cast<float>(256 - acceleration) * 50.0e-6f;
        // 将 RPM/s 换算成 圈/s²，方便与脉冲圈数统一计算。
        float accelerationRpmPerSecond = 1.0f / secondsPerRpm;
        float accelerationRevPerSecondSquared =
            accelerationRpmPerSecond / 60.0f;
        // 从 0 加速到目标速度、再减速到 0 所需的总距离。
        float accelerationAndDecelerationDistance =
            maximumSpeed * maximumSpeed / accelerationRevPerSecondSquared;

        if (revolutions >= accelerationAndDecelerationDistance) {
            // 距离足够长：加速 -> 匀速 -> 减速，使用梯形速度曲线。
            float rampTime =
                2.0f * maximumSpeed / accelerationRevPerSecondSquared;
            float cruiseTime =
                (revolutions - accelerationAndDecelerationDistance)
                / maximumSpeed;
            durationSeconds = rampTime + cruiseTime;
        } else {
            // 距离较短：还未达到设定速度就要减速，使用三角速度曲线。
            float peakSpeed =
                sqrtf(revolutions * accelerationRevPerSecondSquared);
            durationSeconds =
                2.0f * peakSpeed / accelerationRevPerSecondSquared;
        }
    }

    // 向上取整，并额外增加停车稳定时间，避免下一段过早开始。
    return static_cast<uint32_t>(ceilf(durationSeconds * 1000.0f))
           + ROUTE_SETTLE_TIME_MS;
}

// 按估算出的真实运动时间阻塞当前 FreeRTOS 任务。
// vTaskDelay 只阻塞调用本函数的任务，不会卡住串口等其他 FreeRTOS 任务。
void waitForPhysicalMotion(uint32_t pulses, uint16_t speedRpm,
                           uint8_t acceleration) {
    uint32_t delayMs = estimateMotionTimeMs(pulses, speedRpm, acceleration);
    if (delayMs > 0) {
        Serial.printf("[Route] Waiting for motion to complete: %lu ms\n",
                      static_cast<unsigned long>(delayMs));
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}

// 下发四轮同步原地旋转命令，然后等待车辆完全停稳。
void commandSynchronizedRotation(float angle, uint16_t speedRpm,
                                 uint8_t acceleration) {
    // 这里沿用现有 GotoPose 的旋转方向约定：负角度用方向 0，正角度用方向 1。
    uint8_t direction = (angle < 0.0f) ? 0 : 1;
    // 角度乘以旋转标定系数，得到每个电机需要运行的脉冲数。
    uint32_t pulses =
        static_cast<uint32_t>(lroundf(fabsf(angle) * THETA_PULSE));

    // 同步标志设为 true：四条命令只装载参数，暂时不开始运动。
    for (uint8_t motor = 1; motor <= 4; ++motor) {
        Emm_V5_Pos_Control(
            motor, direction, speedRpm, acceleration, pulses, false, true
        );
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
    }
    // 广播同步触发命令，让四个轮子同时开始旋转。
    Emm_V5_Synchronous_motion(0);
    waitForPhysicalMotion(pulses, speedRpm, acceleration);
}

// 下发四轮同步向前直行命令，然后等待车辆完全停稳。
void commandSynchronizedForward(float distance, uint16_t speedRpm,
                                uint8_t acceleration) {
    // 只使用车头向前方向，因此统一使用 X_PULSE 作为前进距离标定系数。
    uint32_t pulses =
        static_cast<uint32_t>(lroundf(distance * X_PULSE));
    // 与 MovePose(0, ...) 使用相同的“车头向前”电机方向。
    constexpr uint8_t FORWARD_DIRECTIONS[4] = {1, 0, 1, 0};

    // 先把相同的距离、速度和加速度装载到四个电机。
    for (uint8_t motor = 1; motor <= 4; ++motor) {
        Emm_V5_Pos_Control(
            motor, FORWARD_DIRECTIONS[motor - 1], speedRpm, acceleration,
            pulses, false, true
        );
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
    }
    // 四轮同步启动，避免逐个启动造成车身偏转。
    Emm_V5_Synchronous_motion(0);
    waitForPhysicalMotion(pulses, speedRpm, acceleration);
}

// 执行一段已经简化好的节点移动：先原地转向，再向前直行。
bool executeNodeSegment(uint8_t startNode, uint8_t endNode,
                        uint16_t speedRpm, uint8_t acceleration) {
    NodePosition start;
    NodePosition end;
    if (!getNodePosition(startNode, start) || !getNodePosition(endNode, end)) {
        return false;
    }

    float deltaX = end.x - start.x;       // 目标相对起点的 X 距离
    float deltaY = end.y - start.y;       // 目标相对起点的 Y 距离
    float distance = hypotf(deltaX, deltaY); // 本段直线距离
    // atan2 根据 X/Y 差值求出目标方向：+X 为 0 度，+Y 为 90 度。
    float targetHeading =
        normalizeHeading(atan2f(deltaY, deltaX) * 180.0f / PI);
    // 选择不超过 180 度的最短转向。
    float turn = shortestTurn(currentPose.theta, targetHeading);

    Serial.printf(
        "[Route] Node %u -> %u, distance %.0f mm, target heading %.0f deg\n",
        startNode, endNode, distance, targetHeading
    );

    // 第一步：车辆原地转到目标方向，并等待转向完全结束。
    if (fabsf(turn) > ROUTE_ANGLE_EPSILON_DEG) {
        Serial.printf("[Route] Rotating in place %.1f deg\n", turn);
        commandSynchronizedRotation(turn, speedRpm, acceleration);
    }
    currentPose.theta = targetHeading;

    // 第二步：车头沿目标方向向前移动，不使用麦克纳姆轮横向平移。
    Serial.printf("[Route] Moving forward %.0f mm\n", distance);
    commandSynchronizedForward(distance, speedRpm, acceleration);
    // 当前没有外部定位反馈，因此运动结束后更新的是“理想位姿”。
    currentPose.x = end.x;
    currentPose.y = end.y;
    return true;
}

} // namespace

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

    int acc = 50;

        if (currentArm.high - high != 0 && high != -1) {
            if( high < 0 || high > 200){//行程保护
                Serial.println("high out of range");
            } else {
                uint8_t dir = (currentArm.high - high > 0) ? 0 : 1;
                uint32_t pulses = (uint32_t)(fabsf(currentArm.high - high) * HEIGHT_PULSE);
                Emm_V5_Pos_Control(5, dir, speed, acc, pulses, 0, 0);
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
                Emm_V5_Pos_Control(6, dir, speed, acc, pulses, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
                currentArm.length = length;
            }
        }
        
        if(turret_angle != -1) {
            if( turret_angle < -360 || turret_angle > 360){//行程保护
                Serial.println("turret_angle out of range");
            } else {
                Servo_SetAngleMTurn(2, turret_angle, speed, 3000);
                currentArm.turret_angle = turret_angle;
            }
        }

        if(pawl_angle != -1) {
            if( pawl_angle < -360 || pawl_angle > 360){//行程保护
                Serial.println("pawl_angle out of range");
            } else {
                Servo_SetAngleMTurn(1, pawl_angle, speed, 3000);
                currentArm.pawl_angle = pawl_angle;
            }
        }

}


/**
 * @brief 速度模式直线移动(麦克纳姆轮, 通用)
 * @param direction 0=前进 1=后退 2=左移 3=右移
 * @param speed   速度 (mm/s)
 * @param stop    true=停止 false=开始移动
 */
void MovePose(int direction, float speed, bool stop) {
    static bool isMoving = false;

    if (stop) {
        if (isMoving) {
            //Emm_V5_Stop_Now(0, 0);
            Emm_V5_Vel_Control(1, 0, 0, 230, 1);
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
            Emm_V5_Vel_Control(2, 0, 0, 230, 1);
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
            Emm_V5_Vel_Control(3, 0, 0, 230, 1);
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
            Emm_V5_Vel_Control(4, 0, 0, 230, 1);
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
            Emm_V5_Synchronous_motion(0);
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
            // [TODO] 停止后按实际反馈更新位姿(如编码器/里程计/视觉)
            isMoving = false;
            Serial.println("move stopped");
        }
        return;
    }

    if (direction < 0 || direction > 3) {
        Serial.println("MovePose direction error, should be 0=fwd, 1=back, 2=left, 3=right");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));

    if (direction == 0) { // 前进
        Emm_V5_Vel_Control(1, 1, speed, 50, 1);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
        Emm_V5_Vel_Control(2, 0, speed, 50, 1);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
        Emm_V5_Vel_Control(3, 1, speed, 50, 1);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
        Emm_V5_Vel_Control(4, 0, speed, 50, 1);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
        Emm_V5_Synchronous_motion(0);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
    } else if (direction == 1) {       // 后退
        Emm_V5_Vel_Control(1, 0, speed, 50, 1);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
        Emm_V5_Vel_Control(2, 1, speed, 50, 1);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
        Emm_V5_Vel_Control(3, 0, speed, 50, 1);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
        Emm_V5_Vel_Control(4, 1, speed, 50, 1);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
        Emm_V5_Synchronous_motion(0);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
    } else if (direction == 2) { // 左
        Emm_V5_Vel_Control(1, 1, speed, 50, 1);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
        Emm_V5_Vel_Control(2, 1, speed, 50, 1);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
        Emm_V5_Vel_Control(3, 0, speed, 50, 1);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
        Emm_V5_Vel_Control(4, 0, speed, 50, 1);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
        Emm_V5_Synchronous_motion(0);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
    } else if (direction == 3) { // 右
        Emm_V5_Vel_Control(1, 0, speed, 50, 1);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
        Emm_V5_Vel_Control(2, 0, speed, 50, 1);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
        Emm_V5_Vel_Control(3, 1, speed, 50, 1);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
        Emm_V5_Vel_Control(4, 1, speed, 50, 1);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
        Emm_V5_Synchronous_motion(0);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
    }
    vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
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
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
            Emm_V5_Pos_Control(2, dir, speed, 50, pulses, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
            Emm_V5_Pos_Control(3, !dir, speed, 50, pulses, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
            Emm_V5_Pos_Control(4, !dir, speed, 50, pulses, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        // 平移 Y
        if (y != 0) {
            uint8_t dir = (y > 0) ? 0 : 1;
            uint32_t pulses = (uint32_t)(fabsf(y) * Y_PULSE);
            Emm_V5_Pos_Control(1, !dir, speed, 50, pulses, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
            Emm_V5_Pos_Control(2,  dir, speed, 50, pulses, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
            Emm_V5_Pos_Control(3, !dir, speed, 50, pulses, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
            Emm_V5_Pos_Control(4,  dir, speed, 50, pulses, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        // 原地旋转
        if (theta != 0) {
            uint8_t dir = (theta < 0) ? 0 : 1;
            uint32_t pulses = (uint32_t)(fabsf(theta) * THETA_PULSE);
            Emm_V5_Pos_Control(1, dir, speed, 50, pulses, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
            Emm_V5_Pos_Control(2, dir, speed, 50, pulses, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
            Emm_V5_Pos_Control(3, dir, speed, 50, pulses, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
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

/**
 * @brief 视觉圆盘对齐：先原地旋转 A，再在旋转后的车体系中平移。
 */
bool AlignToDisc(float targetX, float targetY, float angleDeg,
                 float cameraOffset) {
    constexpr float MAX_TRANSLATION_MM = 2000.0f;// 最大平移距离
    constexpr float MAX_ALIGNMENT_ANGLE_DEG = 180.0f;// 最大对齐角度
    constexpr float MOVEMENT_EPSILON = 0.01f;// 对齐角度阈值
    constexpr uint16_t ALIGN_SPEED_RPM = 80;// 对齐速度
    constexpr uint8_t ALIGN_ACCELERATION = 50;// 对齐加速度

    if (!isfinite(targetX) || !isfinite(targetY) || !isfinite(angleDeg)
            || !isfinite(cameraOffset) || cameraOffset < 0.0f
            || fabsf(targetX) > MAX_TRANSLATION_MM
            || fabsf(targetY) > MAX_TRANSLATION_MM
            || fabsf(angleDeg) > MAX_ALIGNMENT_ANGLE_DEG
            || X_PULSE <= 0.0f || Y_PULSE <= 0.0f || THETA_PULSE <= 0.0f) {
        Serial.println("[Align] ERR: invalid coordinate, angle, offset, or calibration");
        return false;
    }

    const float angleRad = angleDeg * PI / 180.0f;
    const float cosAngle = cosf(angleRad);
    const float sinAngle = sinf(angleRad);
    const float moveX = targetX * cosAngle + targetY * sinAngle
                        - cameraOffset * (1.0f - cosAngle);
    const float moveY = -targetX * sinAngle + targetY * cosAngle
                        + cameraOffset * sinAngle;

    if (fabsf(moveX) > MAX_TRANSLATION_MM
            || fabsf(moveY) > MAX_TRANSLATION_MM) {
        Serial.println("[Align] ERR: transformed translation exceeds safety limit");
        return false;
    }

    Serial.printf(
        "[Align] input x=%.1f mm, y=%.1f mm, angle=%.2f deg, L=%.1f mm\n",
        targetX, targetY, angleDeg, cameraOffset
    );

    // 必须等旋转结束后再平移，否则这里的坐标变换所基于的车体系尚未建立。
    if (fabsf(angleDeg) > MOVEMENT_EPSILON) {
        GotoPose(0.0f, 0.0f, angleDeg, true);
        const uint32_t rotationPulses = static_cast<uint32_t>(
            lroundf(fabsf(angleDeg) * THETA_PULSE)
        );
        waitForPhysicalMotion(
            rotationPulses, ALIGN_SPEED_RPM, ALIGN_ACCELERATION
        );
    }

    Serial.printf("[Align] translated command x=%.1f mm, y=%.1f mm\n", moveX, moveY);

    // X/Y 均交给 GotoPose，由其使用 X_PULSE/Y_PULSE 完成毫米到脉冲的换算。
    // 两轴分开调用并等待，防止后一轴命令覆盖尚未完成的前一轴运动。
    if (fabsf(moveX) > MOVEMENT_EPSILON) {
        const uint32_t pulses = static_cast<uint32_t>(
            lroundf(fabsf(moveX) * X_PULSE)
        );
        GotoPose(moveX, 0.0f, 0.0f, true);
        waitForPhysicalMotion(pulses, ALIGN_SPEED_RPM, ALIGN_ACCELERATION);
    }

    if (fabsf(moveY) > MOVEMENT_EPSILON) {
        const uint32_t pulses = static_cast<uint32_t>(
            lroundf(fabsf(moveY) * Y_PULSE)
        );
        GotoPose(0.0f, moveY, 0.0f, true);
        waitForPhysicalMotion(pulses, ALIGN_SPEED_RPM, ALIGN_ACCELERATION);
    }

    currentPose.theta = normalizeHeading(currentPose.theta + angleDeg);
    Serial.println("[Align] OK");
    return true;
}

/**
 * @brief 按节点序号路径移动，只原地转向和向前直行。
 */
bool MoveNodePath(const uint8_t *path, size_t pathLength,
                  uint16_t speedRpm, uint8_t acceleration) {
    // 第 1 步：检查调用参数。
    if (path == nullptr || pathLength < 2) {
        Serial.println("[Route] Error: path requires at least two nodes");
        return false;
    }
    if (speedRpm == 0 || speedRpm > 5000) {
        Serial.println("[Route] Error: speed must be in 1~5000 RPM range");
        return false;
    }
    if (X_PULSE <= 0.0f || THETA_PULSE <= 0.0f) {
        Serial.println("[Route] Error: forward and rotation pulse calibration coefficients must be > 0");
        return false;
    }

    // 第 2 步：开始运动前完整检查节点，避免走到一半才发现路径无效。
    for (size_t index = 0; index < pathLength; ++index) {
        NodePosition ignored;
        if (!getNodePosition(path[index], ignored)) {
            Serial.printf("[Route] Error: node %u does not exist\n", path[index]);
            return false;
        }
        if (index > 0 && !areAdjacentNodes(path[index - 1], path[index])) {
            Serial.printf(
                "[Route] Error: node %u and node %u are not adjacent\n",
                path[index - 1], path[index]
            );
            return false;
        }
    }

    // 第 3 步：把路径首项当作车辆当前节点。
    // 这里只设置理想 X/Y 坐标，不会让车辆从当前位置移动到首节点。
    NodePosition firstPosition;
    getNodePosition(path[0], firstPosition);
    currentPose.x = firstPosition.x;
    currentPose.y = firstPosition.y;
    currentPose.theta = normalizeHeading(currentPose.theta);

    uint8_t segmentStart = path[0];
    uint8_t segmentEnd = path[1];

    // 第 4 步：逐个查看后续节点。
    // 如果三个节点同向共线，就不断延长当前直线段，不在中间节点停车。
    for (size_t index = 2; index < pathLength; ++index) {
        NodePosition start;
        NodePosition middle;
        NodePosition end;
        getNodePosition(segmentStart, start);
        getNodePosition(segmentEnd, middle);
        getNodePosition(path[index], end);

        if (continuesStraight(start, middle, end)) {
            Serial.printf(
                "[Route] Node %u collinear, merged to node %u -> %u\n",
                segmentEnd, segmentStart, path[index]
            );
            segmentEnd = path[index];
            continue;
        }

        // 方向发生变化：先执行已经确定的直线段，停稳后再处理下一段。
        if (!executeNodeSegment(
                segmentStart, segmentEnd, speedRpm, acceleration)) {
            return false;
        }
        segmentStart = segmentEnd;
        segmentEnd = path[index];
    }

    // 第 5 步：执行循环结束后剩下的最后一段。
    if (!executeNodeSegment(
            segmentStart, segmentEnd, speedRpm, acceleration)) {
        return false;
    }

    Serial.println("[Route] All moves completed");
    return true;
}

/**
 * @brief 机械臂初始化归零位
 */
void InitArm() {
    MoveArm(200,-1,-1,0,150);
    vTaskDelay(pdMS_TO_TICKS(3000));
    Servo_SetAngleMTurn(2, -55, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    MoveArm(150,40,-1,0,150);
    vTaskDelay(pdMS_TO_TICKS(100));
}
