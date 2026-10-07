#include "chassis.h"
#include "Emm_V5.h"
#include "servo.h"
#include "runtime_parameters.h"
#include "node_route_heading.h"
volatile bool taskMotionAborted = false;
#include <float.h>

// 理想位姿(由主控维护; 真实位姿的获取方案待定)
//车体位姿——X坐标、Y坐标、Theta角度
RobotPose currentPose = {0, 0, 0};//X,Y,Theta
//机械臂位姿——大臂高度、小臂伸出长度、转台角度、夹爪角度
ArmPose currentArm = {0, 0, 0, 0};//high,length,turret_angle,pawl_angle
static bool s_turretStartupReferenceFault = false;
static const char* s_turretStartupReferenceFaultReason = "UNKNOWN";
void DisableTurretMotionUntilRestart(const char* reason) {
    s_turretStartupReferenceFault = true;
    s_turretStartupReferenceFaultReason = reason;
}

namespace {

// ================= 节点路径移动的内部数据与参数 =================

// 地图节点的场地绝对坐标，单位为毫米。
struct NodePosition {
    float x;  // X 坐标
    float y;  // Y 坐标
};

// 5×5 蛇形编号，下标就是节点编号 0~24。
// 从底向上、从左到右：0~4、9~5、10~14、19~15、20~24。
// 沿用项目 X 向右、Y 向上；中心间距 480 mm。
NodePosition NODE_POSITIONS[FIELD_NODE_COUNT] = {
    {380.0f, 240.0f}, {720.0f, 240.0f}, {1200.0f, 240.0f}, {1680.0f, 240.0f}, {2160.0f, 240.0f},
    {2160.0f, 720.0f}, {1680.0f, 720.0f}, {1200.0f, 720.0f}, {720.0f, 720.0f}, {380.0f, 720.0f},
    {380.0f, 1200.0f}, {720.0f, 1200.0f}, {1200.0f, 1200.0f}, {1680.0f, 1200.0f}, {2160.0f, 1200.0f},
    {2160.0f, 1680.0f}, {1680.0f, 1680.0f}, {1200.0f, 1680.0f}, {720.0f, 1680.0f}, {380.0f, 1680.0f},
    {380.0f, 2020.0f}, {720.0f, 2020.0f}, {1200.0f, 2020.0f}, {1680.0f, 2020.0f}, {2160.0f, 2020.0f},
};

constexpr uint32_t MOTOR_PULSES_PER_REVOLUTION = 3200; // 16 细分时，电机转一圈的脉冲数
constexpr uint32_t MOTOR_COMMAND_GAP_MS = 5;           // 连续发送两条电机命令的间隔
constexpr double MOTOR_MOTION_TIME_MARGIN = 1.30;      // 运动估算增加 30% 等待余量
uint32_t ROUTE_SETTLE_TIME_MS = 250;         // 每次运动结束后的停车稳定时间
constexpr float ROUTE_ANGLE_EPSILON_DEG = 0.01f;       // 小于该角度时不再执行转向

// ================= 连续视觉对齐 PID 参数 =================
// 输出均为 -1~1 的归一化车身速度权重，实际轮速由调用方传入的 speedRpm 决定。
// 顺序对应 VisionStartMode；各行独立调整，X/Y 为视觉单位，角度为度。
VisionAlignmentDeadzone ALIGN_DEADZONES[] = {
    {8.0f, 8.0f, 0.5f}, // DISC 圆盘对齐
    {3.0f, 3.0f, 0.2f}, // DISC_MATERIAL 圆盘物料对齐
    {3.0f, 3.0f, 0.2f}, // WORK_AREA
    {3.0f, 3.0f, 0.2f}, // WORK_AREA_LOADED
    {6.0f, 6.0f, 0.4f}, // CORNER
};
AlignmentFrameWaiter s_alignmentWaiter;
constexpr float ALIGN_DT_MIN_SECONDS = 0.02f;// 时间步长单位为秒
constexpr float ALIGN_DT_MAX_SECONDS = 0.30f;// 时间步长单位为秒
constexpr uint16_t OMNI_MIN_MOVING_RPM = 1;// 最小移动速度单位为转/分
constexpr uint8_t OMNI_ACCELERATION = 0;// 加速度单位为转/分^2

struct PidController {
    float kp;
    float ki;
    float kd;
    float integralLimit;      // 积分项上限
    float integral;           // 积分项
    float previousError;      // 上一次误差
    bool hasPreviousError;    // 是否有上一次误差
};

// 位置误差单位为视觉输出值；角度误差单位为度。
PidController s_alignVisualXPid = {
    0.01f, 0.0005f, 0.0f, 200.0f, 0.0f, 0.0f, false
};
PidController s_alignVisualYPid = {
    0.01f, 0.0005f, 0.0f, 300.0f, 0.0f, 0.0f, false
};
PidController s_alignAnglePid = {
    0.03f, 0.0005f, 0.0f, 20.0f, 0.0f, 0.0f, false
};

float clampFloat(float value, float minimum, float maximum) {
    return fmaxf(minimum, fminf(value, maximum));
}

void resetPid(PidController &pid) {
    pid.integral = 0.0f;
    pid.previousError = 0.0f;
    pid.hasPreviousError = false;
}

float updatePid(PidController &pid, float error, float dtSeconds) {
    pid.integral = clampFloat(
        pid.integral + error * dtSeconds,
        -pid.integralLimit, pid.integralLimit
    );
    float derivative = 0.0f;
    if (pid.hasPreviousError) {
        derivative = (error - pid.previousError) / dtSeconds;
    }
    pid.previousError = error;
    pid.hasPreviousError = true;

    return clampFloat(
        pid.kp * error + pid.ki * pid.integral + pid.kd * derivative,
        -1.0f, 1.0f
    );
}

// 根据节点序号读取坐标。序号只能是 0~24。
bool getNodePosition(uint8_t node, NodePosition &position) {
    if (node >= FIELD_NODE_COUNT) {
        return false;
    }
    position = NODE_POSITIONS[node];
    return true;
}

// 5×5 蛇形图上下或左右相邻：0-9、4-5 相邻，4-9 不相邻。
bool areAdjacentNodes(uint8_t first, uint8_t second) {
    if (first >= FIELD_NODE_COUNT || second >= FIELD_NODE_COUNT) return false;
    int firstRow = first / FIELD_GRID_SIZE;
    int secondRow = second / FIELD_GRID_SIZE;
    int firstColumn = first % FIELD_GRID_SIZE;
    int secondColumn = second % FIELD_GRID_SIZE;
    if (firstRow % 2) firstColumn = FIELD_GRID_SIZE - 1 - firstColumn;
    if (secondRow % 2) secondColumn = FIELD_GRID_SIZE - 1 - secondColumn;
    return abs(firstColumn - secondColumn) + abs(firstRow - secondRow) == 1;
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

    // 运动时间增加 30% 后向上取整，再加停车稳定时间，避免下一段过早开始。
    return static_cast<uint32_t>(ceil(durationSeconds * 1000.0 * MOTOR_MOTION_TIME_MARGIN))
           + ROUTE_SETTLE_TIME_MS;
}

// 按运动估算及等待余量阻塞当前 FreeRTOS 任务，实际到位仍需确认。
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

// 下发四轮同步车身 Y 轴直行命令；负距离为后退。
void commandSynchronizedStraight(float distance, uint16_t speedRpm,
                                uint8_t acceleration) {
    // 沿用原前进标定 X_PULSE，后退仅反转四轮方向，不做横向平移。
    uint32_t pulses =
        static_cast<uint32_t>(lroundf(fabsf(distance) * X_PULSE));
    // 与 MovePose(0, ...) 使用相同的“车头向前”电机方向。
    constexpr uint8_t FORWARD_DIRECTIONS[4] = {1, 0, 1, 0};

    // 先把相同的距离、速度和加速度装载到四个电机。
    for (uint8_t motor = 1; motor <= 4; ++motor) {
        Emm_V5_Pos_Control(
            motor, distance < 0.0f ? 1 - FORWARD_DIRECTIONS[motor - 1]
                                  : FORWARD_DIRECTIONS[motor - 1],
            speedRpm, acceleration,
            pulses, false, true
        );
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
    }
    // 四轮同步启动，避免逐个启动造成车身偏转。
    Emm_V5_Synchronous_motion(0);
    waitForPhysicalMotion(pulses, speedRpm, acceleration);
}

// 执行一段已经简化好的节点移动：选择转角较小的前进/后退朝向，再直行。
bool executeNodeSegment(uint8_t startNode, uint8_t endNode,
                        uint16_t speedRpm, uint8_t acceleration,
                        float finalHeading = NAN) {
    NodePosition start;
    NodePosition end;
    if (!getNodePosition(startNode, start) || !getNodePosition(endNode, end)) {
        return false;
    }

    float deltaX = end.x - start.x;       // 目标相对起点的 X 距离
    float deltaY = end.y - start.y;       // 目标相对起点的 Y 距离
    float distance = hypotf(deltaX, deltaY); // 本段直线距离
    // atan2 根据 X/Y 差值求出目标方向：+X 为 0 度，+Y 为 90 度。
    const float travelHeading =
        normalizeHeading(atan2f(deltaY, deltaX) * 180.0f / PI);
    const bool finish = !isnan(finalHeading);
    const auto plan = PlanNodeSegmentHeading(normalizeHeading(currentPose.theta),
        travelHeading, finish, finish ? normalizeHeading(finalHeading) : 0.0f);
    const float targetHeading = plan.heading;
    const float turn = plan.turn;
    const bool reverse = plan.reverse;

    Serial.printf(
        "[Route] Node %u -> %u, distance %.0f mm, target heading %.0f deg\n",
        startNode, endNode, distance, targetHeading
    );
    Serial.printf("[Route] Current heading %.0f deg, commanded turn %.1f deg\n",
                  currentPose.theta, turn);

    // 第一步：车辆原地转到目标方向，并等待转向完全结束。
    if (fabsf(turn) > ROUTE_ANGLE_EPSILON_DEG) {
        Serial.printf("[Route] Rotating in place %.1f deg\n", turn);
        commandSynchronizedRotation(turn, speedRpm, acceleration);
    }
    currentPose.theta = targetHeading;

    // 第二步：只沿车身 Y 轴前进或后退，不使用麦克纳姆轮横向平移。
    Serial.printf("[Route] Moving %s %.0f mm\n", reverse ? "backward" : "forward", distance);
    commandSynchronizedStraight(reverse ? -distance : distance, speedRpm, acceleration);
    // 当前没有外部定位反馈，因此运动结束后更新的是“理想位姿”。
    currentPose.x = end.x;
    currentPose.y = end.y;
    if (finish) {
        Serial.printf("[Route] Final heading %.0f deg, arrival turn %.1f deg\n",
                      normalizeHeading(finalHeading), plan.finishTurn);
        if (fabsf(plan.finishTurn) > ROUTE_ANGLE_EPSILON_DEG) {
            commandSynchronizedRotation(plan.finishTurn, speedRpm, acceleration);
        }
        currentPose.theta = normalizeHeading(finalHeading);
    }
    return true;
}

} // namespace

// 注册原变量地址，上位机修改后由底盘和 PID 算法直接使用。
void RegisterChassisParameters() {
    PidController *controllers[] = {&s_alignVisualXPid, &s_alignVisualYPid, &s_alignAnglePid};
    const char *axes[] = {"视觉X", "视觉Y", "角度"};
    for (uint8_t i = 0; i < 3; ++i) {
        char name[64];
        float *values[] = {&controllers[i]->kp, &controllers[i]->ki,
                           &controllers[i]->kd, &controllers[i]->integralLimit};
        const char *fields[] = {"Kp", "Ki", "Kd", "积分限幅"};
        for (uint8_t j = 0; j < 4; ++j) {
            snprintf(name, sizeof(name), "PID/%s/%s", axes[i], fields[j]);
            RegisterRuntimeFloat(name, *values[j], 0, j == 3 ? 100000 : 100);
        }
    }
    const char *modes[] = {"DISC", "DISC_MATERIAL", "WORK_AREA", "WORK_AREA_LOADED", "CORNER"};
    for (uint8_t i = 0; i < 5; ++i) {
        char name[80];
        snprintf(name, sizeof(name), "对齐/%s/X死区", modes[i]);
        RegisterRuntimeFloat(name, ALIGN_DEADZONES[i].visualX, 0, 1000);
        snprintf(name, sizeof(name), "对齐/%s/Y死区", modes[i]);
        RegisterRuntimeFloat(name, ALIGN_DEADZONES[i].visualY, 0, 1000);
        snprintf(name, sizeof(name), "对齐/%s/角度死区(deg)", modes[i]);
        RegisterRuntimeFloat(name, ALIGN_DEADZONES[i].angleDeg, 0, 180);
    }
    RegisterRuntimeUInt("底盘/停车稳定时间(ms)", ROUTE_SETTLE_TIME_MS, 0, 10000);
    float *pulses[] = {&X_PULSE, &Y_PULSE, &THETA_PULSE, &HEIGHT_PULSE, &LENGTH_PULSE};
    const char *names[] = {"标定/X脉冲每毫米", "标定/Y脉冲每毫米", "标定/旋转脉冲每度",
                           "标定/升降脉冲每毫米", "标定/伸缩脉冲每毫米"};
    for (uint8_t i = 0; i < 5; ++i) RegisterRuntimeFloat(names[i], *pulses[i], 0.001f, 100000);
    for (uint8_t i = 0; i < FIELD_NODE_COUNT; ++i) {
        char name[64];
        snprintf(name, sizeof(name), "地图/节点%u/X(mm)", i);
        RegisterRuntimeFloat(name, NODE_POSITIONS[i].x, 0, 2400);
        snprintf(name, sizeof(name), "地图/节点%u/Y(mm)", i);
        RegisterRuntimeFloat(name, NODE_POSITIONS[i].y, 0, 2400);
    }
}


/**
 * @brief 机械臂移动到指定位姿
 * @param high 大臂高度 (mm)
 * @param length 小臂伸出长度 (mm)
 * @param turret_angle 转台角度 (°)
 * @param pawl_angle 夹爪角度 (°)
 * @param speed 速度 (mm/s)
 * @note -1 表示不操作该轴
 */
bool MoveArm(float high, float length, float turret_angle, float pawl_angle, float speed) {
    if (taskMotionAborted) return false;
    int acc = 200;
    float safeTurretTarget = 0.0f;
    if (turret_angle != -1.0f) {
        if (s_turretStartupReferenceFault) {
            Serial.printf("[Arm] ERR: turret startup turn count unverified (%s); motion skipped\n",
                          s_turretStartupReferenceFaultReason);
            return false;
        }
        if (!isfinite(turret_angle) || turret_angle < TURRET_CABLE_MIN_DEG
                || turret_angle > TURRET_CABLE_MAX_DEG) {
            Serial.println("[Arm] ERR: turret request outside cable range [-180,360]");
            return false;
        }
        float measuredAngle = 0.0f;
        if (!Servo_QueryAngleMTurn(2, measuredAngle, 100) || !isfinite(measuredAngle)) {
            Serial.println("[Arm] ERR: turret multi-turn angle read failed; motion skipped");
            return false;
        }
        // 舵机的多圈读数就是线缆累计旋转角；已经越界时禁止自动动作。
        if (measuredAngle < TURRET_CABLE_MIN_DEG || measuredAngle > TURRET_CABLE_MAX_DEG) {
            Serial.printf("[Arm] ERR: turret actual %.1f outside cable range [-180,360]; motion skipped\n",
                          measuredAngle);
            return false;
        }
        // 同一物理方向有相差 360 度的目标，但只允许选择线缆边界内的目标。
        float bestDistance = INFINITY;
        for (int8_t turn = -1; turn <= 1; ++turn) {
            const float candidate = turret_angle + 360.0f * turn;
            if (candidate < TURRET_CABLE_MIN_DEG || candidate > TURRET_CABLE_MAX_DEG) continue;
            const float distance = fabsf(candidate - measuredAngle);
            if (distance < bestDistance) {
                bestDistance = distance;
                safeTurretTarget = candidate;
            }
        }
        if (!isfinite(bestDistance)) {
            Serial.println("[Arm] ERR: no turret target within cable range");
            return false;
        }
    }

        if (taskMotionAborted) return false;
        if (currentArm.high - high != 0 && high != -1) {
            if( high < 0 || high > ARM_HEIGHT_LIMIT_MM){//行程保护
                Serial.println("high out of range");
                return false;
            } else {
                uint8_t dir = (currentArm.high - high > 0) ? 0 : 1;
                uint32_t pulses = (uint32_t)(fabsf(currentArm.high - high) * HEIGHT_PULSE);
                Emm_V5_Pos_Control(5, dir, speed*3, acc, pulses, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
                currentArm.high = high;
            }
        }

        if (taskMotionAborted) return false;
        if (currentArm.length - length != 0 && length != -1) {
            if( length < 0 || length > 170){//行程保护
                Serial.println("length out of range");
                return false;
            } else {
                uint8_t dir = (currentArm.length - length > 0) ? 0 : 1;
                uint32_t pulses = (uint32_t)(fabsf(currentArm.length - length) * LENGTH_PULSE);
                Emm_V5_Pos_Control(6, dir, speed, acc, pulses, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
                currentArm.length = length;
            }
        }
        
        if (taskMotionAborted) return false;
        if(turret_angle != -1) {
            Servo_SetAngleMTurn(2, safeTurretTarget, (300-speed)*3, 0);
            currentArm.turret_angle = safeTurretTarget;
        }

        if (taskMotionAborted) return false;
        if(pawl_angle != -1) {
            if( pawl_angle < -360 || pawl_angle > 360){//行程保护
                Serial.println("pawl_angle out of range");
                return false;
            } else {
                float measuredAngle = 0.0f;
                if (!Servo_QueryAngleMTurn(1, measuredAngle, 100) || !isfinite(measuredAngle)) {
                    Serial.println("[Arm] ERR: gripper multi-turn angle read failed; motion skipped");
                    return false;
                } else {
                    const float nearestAngle = pawl_angle
                        + 360.0f * roundf((measuredAngle - pawl_angle) / 360.0f);
                    if (!isfinite(nearestAngle) || fabsf(nearestAngle) > 368640.0f) {
                        Serial.println("[Arm] ERR: gripper multi-turn target out of servo range");
                        return false;
                    }
                    if (taskMotionAborted) return false;
                    Servo_SetAngleMTurn(1, nearestAngle, (300-speed)*3, 0);
                    currentArm.pawl_angle = pawl_angle;
                }
            }
        }

    return true;
}


/**
 * @brief 速度模式平移或原地旋转(麦克纳姆轮, 通用)
 * @param direction 0=前进 1=后退 2=左移 3=右移 4=左转 5=右转
 * @param speed   电机转速 (RPM)
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

    if (direction < 0 || direction > 5) {
        Serial.println("MovePose direction error, should be 0=fwd, 1=back, 2=left, 3=right, 4=turn left, 5=turn right");
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
    } else if (direction == 4 || direction == 5) {
        // 沿用 GotoPose 正角左转、负角右转的四轮方向约定。
        const uint8_t motorDirection = (direction == 4) ? 1 : 0;
        for (uint8_t motor = 1; motor <= 4; ++motor) {
            Emm_V5_Vel_Control(motor, motorDirection, speed, 50, 1);
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
        }
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
        // 命令下发完成后更新开环目标估计；不代表电机已到位。
        // 相对 X/Y 属于车体坐标，先按移动前航向转换到世界坐标。
        const float heading = currentPose.theta * PI / 180.0f;
        currentPose.x += x * cosf(heading) - y * sinf(heading);
        currentPose.y += x * sinf(heading) + y * cosf(heading);
        currentPose.theta = normalizeHeading(currentPose.theta + theta);
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
 * @brief 麦克纳姆轮全向速度混合并同步下发四轮速度。
 */
void OmniMove(float xVelocity, float yVelocity, float rotationVelocity,
              uint16_t speedRpm) {
    if (!isfinite(xVelocity) || !isfinite(yVelocity) || !isfinite(rotationVelocity) || speedRpm > 5000) {
        Serial.println("[OmniMove] ERR: invalid velocity or speed");
        xVelocity = 0.0f;
        yVelocity = 0.0f;
        rotationVelocity = 0.0f;
        speedRpm = 0;
    }

    xVelocity = clampFloat(xVelocity, -1.0f, 1.0f);
    yVelocity = clampFloat(yVelocity, -1.0f, 1.0f);
    rotationVelocity = clampFloat(rotationVelocity, -1.0f, 1.0f);

    // 方向符号与现有 MovePose/GotoPose 的四轮接线约定保持一致。
    float wheelVelocity[4] = {
        -xVelocity + yVelocity + rotationVelocity,
        -xVelocity - yVelocity + rotationVelocity,
         xVelocity + yVelocity + rotationVelocity,
         xVelocity - yVelocity + rotationVelocity,
    };

    float maximumMagnitude = 0.0f;
    for (float value : wheelVelocity) {
        maximumMagnitude = fmaxf(maximumMagnitude, fabsf(value));// 计算最大绝对值
    }
    if (maximumMagnitude > 1.0f) {
        for (float &value : wheelVelocity) {
            value /= maximumMagnitude;
        }
        maximumMagnitude = 1.0f;
    }

    // 小误差时整体抬高四轮速度比例，使最大轮达到可启动转速，同时保持轮间比例。
    float rpmScale = static_cast<float>(speedRpm);
    const uint16_t minimumMovingRpm =
        (speedRpm < OMNI_MIN_MOVING_RPM) ? speedRpm : OMNI_MIN_MOVING_RPM;
    if (maximumMagnitude > 0.0f
            && maximumMagnitude * rpmScale < minimumMovingRpm) {
        rpmScale = static_cast<float>(minimumMovingRpm) / maximumMagnitude;
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    for (uint8_t motor = 1; motor <= 4; ++motor) {
        const float command = wheelVelocity[motor - 1];
        uint16_t motorRpm = static_cast<uint16_t>( lroundf(fabsf(command) * rpmScale));
        const uint8_t direction = (command >= 0.0f) ? 1 : 0;
        Emm_V5_Vel_Control( motor, direction, motorRpm, OMNI_ACCELERATION, true );
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
    }
    Emm_V5_Synchronous_motion(0);
}

bool MovePosition(float x, float y, float theta, float speed,
                  bool (*shouldStop)()) {
    // 完整验证后才下发命令，避免四轮只装载了一部分参数。
    if (!isfinite(x) || !isfinite(y) || !isfinite(theta)
            || !isfinite(speed) || speed < 1.0f || speed > 5000.0f
            || !isfinite(X_PULSE) || X_PULSE <= 0.0f
            || !isfinite(Y_PULSE) || Y_PULSE <= 0.0f
            || !isfinite(THETA_PULSE) || THETA_PULSE <= 0.0f
            || !isfinite(currentPose.x) || !isfinite(currentPose.y)
            || !isfinite(currentPose.theta)) {
        Serial.println("[MovePosition] ERR: invalid pose, speed or calibration");
        return false;
    }

    // SE(2) 指数映射的逆：x/y 是起始车身系中的终点位移。
    // 转动过程中车身系也在转动，不能直接把终点 x/y 当作累计车身位移。
    const double angle = static_cast<double>(theta) * PI / 180.0;
    double bodyX = x;
    double bodyY = y;
    if (fabs(angle) > 1.0e-6 && (x != 0.0f || y != 0.0f)) {
        const double a = sin(angle) / angle;
        const double b = (1.0 - cos(angle)) / angle;
        const double determinant = a * a + b * b;
        if (determinant < 1.0e-10) {
            Serial.println("[MovePosition] ERR: translation with full-turn rotation is singular");
            return false;
        }
        bodyX = (a * x + b * y) / determinant;
        bodyY = (-b * x + a * y) / determinant;
    }

    const double px = bodyX * X_PULSE;
    const double py = bodyY * Y_PULSE;
    const double pr = static_cast<double>(theta) * THETA_PULSE;
    // 与 GotoPose/OmniMove 相同的电机 1~4 符号约定，正脉冲使用 dir=1。
    const double wheelPulses[4] = {-px + py + pr, -px - py + pr,
                                   px + py + pr,  px - py + pr};
    uint32_t pulses[4];
    uint16_t wheelRpm[4];
    uint32_t maximumPulses = 0;
    for (size_t i = 0; i < 4; ++i) {
        const double rounded = floor(fabs(wheelPulses[i]) + 0.5);
        if (!isfinite(rounded) || rounded > UINT32_MAX) {
            Serial.println("[MovePosition] ERR: pulse count overflow");
            return false;
        }
        pulses[i] = static_cast<uint32_t>(rounded);
        if (pulses[i] > maximumPulses) maximumPulses = pulses[i];
    }
    if (maximumPulses == 0) return true; // 小于脉冲分辨率，不虚增理想位姿。

    const uint16_t maximumRpm = static_cast<uint16_t>(lroundf(speed));
    double waitMs = 0.0;
    for (size_t i = 0; i < 4; ++i) {
        // 各轮按行程比例分配转速；整数 RPM 会带来少量完成时间差。
        wheelRpm[i] = pulses[i] == 0 ? 0 : static_cast<uint16_t>(fmax(
            1.0, floor(static_cast<double>(pulses[i]) * maximumRpm / maximumPulses + 0.5)));
        if (wheelRpm[i] != 0) {
            waitMs = fmax(waitMs, static_cast<double>(pulses[i]) * 60000.0
                          / (MOTOR_PULSES_PER_REVOLUTION * wheelRpm[i]));
        }
    }
    waitMs = ceil(waitMs * MOTOR_MOTION_TIME_MARGIN) + ROUTE_SETTLE_TIME_MS;
    const double heading = normalizeHeading(currentPose.theta) * PI / 180.0;
    const double targetX = currentPose.x + x * cos(heading) - y * sin(heading);
    const double targetY = currentPose.y + x * sin(heading) + y * cos(heading);
    if (waitMs > UINT32_MAX || !isfinite(targetX) || !isfinite(targetY)
            || fabs(targetX) > FLT_MAX || fabs(targetY) > FLT_MAX) {
        Serial.println("[MovePosition] ERR: duration or target pose overflow");
        return false;
    }

    if (shouldStop != nullptr && shouldStop()) return false;
    for (uint8_t motor = 1; motor <= 4; ++motor) {
        // acc=0 直接启动，避免相同加速度档位破坏不同轮速的行程比例。
        if (pulses[motor - 1] == 0) {
            // 合成后不需转动的轮子同步停车，避免发送零转速位置命令。
            Emm_V5_Stop_Now(motor, true);
        } else {
            Emm_V5_Pos_Control(motor, wheelPulses[motor - 1] >= 0.0 ? 1 : 0,
                               wheelRpm[motor - 1], 0, pulses[motor - 1], false, true);
        }
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
    }
    Emm_V5_Synchronous_motion(0);
    uint32_t remainingMs = static_cast<uint32_t>(waitMs);
    while (remainingMs > 0) {
        if (shouldStop != nullptr && shouldStop()) {
            for (uint8_t motor = 1; motor <= 4; ++motor) {
                Emm_V5_Stop_Now(motor, true);
                vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));
            }
            Emm_V5_Synchronous_motion(0);
            return false;
        }
        const uint32_t pollMs = shouldStop != nullptr ? 10 : 1000;
        const uint32_t chunkMs = remainingMs > pollMs ? pollMs : remainingMs;
        vTaskDelay(pdMS_TO_TICKS(chunkMs));
        remainingMs -= chunkMs;
    }
    currentPose.x = static_cast<float>(targetX);
    currentPose.y = static_cast<float>(targetY);
    currentPose.theta = normalizeHeading(normalizeHeading(currentPose.theta)
                                        + normalizeHeading(theta));
    Serial.println("[MovePosition] Commands sent and estimated wait elapsed; arrival not verified");
    return true;
}

void ResetDiscAlignmentPid() {
    resetPid(s_alignVisualXPid);
    resetPid(s_alignVisualYPid);
    resetPid(s_alignAnglePid);
}

/**
 * @brief 根据连续视觉误差计算三个 PID 速度权重并进行全向闭环对齐。
 * @param angleErrorDeg 角度误差（度）
 * @param visualXError 视觉 X 误差（视觉输出值）
 * @param visualYError 视觉 Y 误差（视觉输出值）
 * @param dtSeconds 时间间隔（秒）
 * @param speedRpm 目标速度（RPM）
 * @return 是否成功对齐
 */
bool AlignToDiscContinuous(float angleErrorDeg, float visualXError,
                           float visualYError, float dtSeconds,
                           uint16_t speedRpm) {
    return AlignToVisionContinuous(VisionStartMode::DISC, angleErrorDeg,
                                  visualXError, visualYError, dtSeconds, speedRpm);
}

void ResetAlignmentWait() {
    s_alignmentWaiter.reset();
}

bool WaitForAlignment(VisionStartMode mode, float angleErrorDeg,
                      float visualXError, float visualYError,
                      float dtSeconds, uint16_t speedRpm) {
    const bool withinDeadzone = AlignToVisionContinuous(
        mode, angleErrorDeg, visualXError, visualYError, dtSeconds, speedRpm);
    return s_alignmentWaiter.update(mode, withinDeadzone);
}

bool AlignToVisionContinuous(VisionStartMode mode, float angleErrorDeg,
                             float visualXError, float visualYError,
                             float dtSeconds, uint16_t speedRpm) {
    if (!isfinite(angleErrorDeg) || !isfinite(visualXError)
            || !isfinite(visualYError) || !isfinite(dtSeconds)
            || dtSeconds <= 0.0f || speedRpm == 0 || speedRpm > 5000
            || static_cast<uint8_t>(mode) >= 5) {
        OmniMove(0.0f, 0.0f, 0.0f, 0);
        ResetDiscAlignmentPid();
        Serial.println("[Align PID] ERR: invalid feedback, dt, or speed");
        return false;
    }

    dtSeconds = clampFloat(
        dtSeconds, ALIGN_DT_MIN_SECONDS, ALIGN_DT_MAX_SECONDS
    );

    const auto &deadzone = ALIGN_DEADZONES[static_cast<uint8_t>(mode)];
    const bool angleAligned = fabsf(angleErrorDeg) <= deadzone.angleDeg;
    const bool xAligned = fabsf(visualXError) <= deadzone.visualX;
    const bool yAligned = fabsf(visualYError) <= deadzone.visualY;

    //bool angleAligned = 1;// 测试用，直接对齐角度
    //bool xAligned = 1;// 测试用，直接对齐视觉 X
    //bool yAligned = 1;// 测试用，直接对齐视觉 Y

    if (IsWithinAlignmentDeadzone(deadzone, angleErrorDeg, visualXError, visualYError)) {
        OmniMove(0.0f, 0.0f, 0.0f, 0);
        ResetDiscAlignmentPid();
        return true;
    }

    float visualXOutput = 0.0f;
    float visualYOutput = 0.0f;
    float angleOutput = 0.0f;

    if (xAligned) {
        resetPid(s_alignVisualXPid);
    } else {
        visualXOutput = updatePid( s_alignVisualXPid, visualXError, dtSeconds
        );
    }
    if (yAligned) {
        resetPid(s_alignVisualYPid);
    } else {
        visualYOutput = updatePid( s_alignVisualYPid, visualYError, dtSeconds
        );
    }
    if (angleAligned) {
        resetPid(s_alignAnglePid);
    } else {
        angleOutput = updatePid( s_alignAnglePid, angleErrorDeg, dtSeconds
        );
    }

    // 摄像头相对车身约旋转 90°：视觉 Y -> 底盘 X，视觉 X -> 底盘 Y。
    // 单轴实测表明两个映射均需取反；角度正方向也与底盘旋转方向相反。
    const float chassisXVelocity = -visualYOutput;
    const float chassisYVelocity = -visualXOutput;
    const float chassisRotationVelocity = -angleOutput;

    OmniMove(
        chassisXVelocity, chassisYVelocity,
        chassisRotationVelocity, speedRpm
    );
    return false;
}

/**
 * @brief 按节点序号路径移动，只原地转向和沿车身 Y 轴前后直行。
 */
bool MoveNodePath(const uint8_t *path, size_t pathLength,
                  uint16_t speedRpm, uint8_t acceleration, float finalHeading) {
    // 第 1 步：检查调用参数。
    if (path == nullptr || pathLength < 2) {
        Serial.println("[Route] Error: path requires at least two nodes");
        return false;
    }
    if (speedRpm == 0 || speedRpm > 5000) {
        Serial.println("[Route] Error: speed must be in 1~5000 RPM range");
        return false;
    }
    if (!isfinite(currentPose.theta) || (!isnan(finalHeading) && !isfinite(finalHeading))) {
        Serial.println("[Route] Error: heading must be finite (NaN final heading means unconstrained)");
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
    // 航向也只是开环估计；执行前必须确认车身实际朝向与记录一致。
    NodePosition firstPosition;
    getNodePosition(path[0], firstPosition);
    Serial.printf("[Route] Assuming car is at node %u, initial heading %.0f deg (open-loop)\n",
                  path[0], normalizeHeading(currentPose.theta));
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

    // 第 5 步：最后一段优先满足终点朝向，必要时到达后再原地调整。
    if (!executeNodeSegment(
            segmentStart, segmentEnd, speedRpm, acceleration, finalHeading)) {
        return false;
    }

    Serial.println("[Route] All commands sent and estimated waits elapsed; position not verified");
    return true;
}

/**
 * @brief 机械臂初始化归零位
 */
bool InitArm_start() {
    if (!MoveArm(160,-1,-1,0,150)) return false;
    vTaskDelay(pdMS_TO_TICKS(2000));
    if (!MoveArm(-1, -1, -55, -1, 150)) return false;
    vTaskDelay(pdMS_TO_TICKS(100));
    if (!MoveArm(150,40,-1,0,150)) return false;
    vTaskDelay(pdMS_TO_TICKS(500));
    return true;
}

/**
 * @brief 机械臂初始化归视觉位
 */
bool InitArm_look() {
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (!MoveArm(170,0,-1,60,200)) return false;
    vTaskDelay(pdMS_TO_TICKS(2000));
    if (!MoveArm(-1, 0, 90, 80, 200)) return false;
    vTaskDelay(pdMS_TO_TICKS(100));
    if (!MoveArm(170,-1,-1,80,200)) return false;
    vTaskDelay(pdMS_TO_TICKS(1000));
    return true;
}

/**
 * @brief 机械臂初始化归视觉位
 */
bool InitArm_look2() {
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (!MoveArm(170,0,-1,-1,150)) return false;
    vTaskDelay(pdMS_TO_TICKS(2000));
    if (!MoveArm(-1, 0, 90, 80, 150)) return false;
    return true;
}

/**
 * @brief 机械臂初始化归视觉位
 */
bool InitArm_look3() {
    if (!MoveArm(170,30,-1,-1,150)) return false;
    vTaskDelay(pdMS_TO_TICKS(2000));
    if (!MoveArm(-1, -1, 90, 80, 150)) return false;
    return true;
}
