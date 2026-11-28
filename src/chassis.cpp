// 底盘与机械臂的运动下发层。
// 包括车体速度换算、轮电机位置动作及视觉对齐 PID。
// 调试时先区分闭环误差、车体位移和驱动器脉冲三个层次。

#include "chassis.h"  // 引入 chassis.h 中的声明和定义。
#include "Emm_V5.h"  // 引入 Emm_V5.h 中的声明和定义。
#include "servo.h"  // 引入 servo.h 中的声明和定义。
#include "runtime_parameters.h"  // 引入 runtime_parameters.h 中的声明和定义。
#include "robot_runtime.h"  // 引入 robot_runtime.h 中的声明和定义。
#include "continuous_trial.h"  // 引入 continuous_trial.h 中的声明和定义。
#include "node_position_hardware.h"  // 引入 node_position_hardware.h 中的声明和定义。
#include "motor_bus.h"  // 引入 motor_bus.h 中的声明和定义。
#include "turret_motor.h"  // 引入 turret_motor.h 中的声明和定义。
volatile bool taskMotionAborted = false;  // 将 false 写入或初始化 taskMotionAborted（任务MotionAborted）。

// 理想位姿(由主控维护; 真实位姿的获取方案待定)
//车体位姿——X坐标、Y坐标、Theta角度
RobotPose currentPose = {0, 0, 0};//X,Y,Theta
//机械臂位姿——大臂高度、小臂伸出长度、转台角度、夹爪角度
ArmPose currentArm = {0, 0, 0, 0};//high,length,turret_angle,pawl_angle

namespace {  // 开始匿名命名空间，本文件内部共享其中的定义。

// ================= 节点路径移动的内部数据与参数 =================

// 地图节点的场地绝对坐标，单位为毫米。
struct NodePosition {  // 定义 NodePosition（节点位置） 的类型成员或作用域。
    float x;  // X 坐标
    float y;  // Y 坐标
};  // 结束此类型定义。

// 5×5 蛇形编号，下标就是节点编号 0~24。
// 从底向上、从左到右：0~4、9~5、10~14、19~15、20~24。
// 沿用项目 X 向右、Y 向上。
NodePosition NODE_POSITIONS[FIELD_NODE_COUNT] = {  // 将 { 写入或初始化 NODE_POSITIONS[FIELD_NODE_COUNT]。
    {320.0f, 240.0f}, {720.0f, 240.0f}, {1180.0f, 240.0f}, {1680.0f, 240.0f}, {2060.0f, 240.0f},  // 为当前列表或跨行表达式提供这一项 {320.0f, 240.0f}, {720.0f, 240.0f}, {1180.0f, 240.0f},…。
    {2060.0f, 720.0f}, {1680.0f, 720.0f}, {1180.0f, 720.0f}, {720.0f, 720.0f}, {320.0f, 720.0f},  // 为当前列表或跨行表达式提供这一项 {2060.0f, 720.0f}, {1680.0f, 720.0f}, {1180.0f, 720.0f…。
    {320.0f, 1200.0f}, {720.0f, 1200.0f}, {1180.0f, 1200.0f}, {1680.0f, 1200.0f}, {2060.0f, 1200.0f},  // 为当前列表或跨行表达式提供这一项 {320.0f, 1200.0f}, {720.0f, 1200.0f}, {1180.0f, 1200.0…。
    {2060.0f, 1680.0f}, {1680.0f, 1680.0f}, {1180.0f, 1680.0f}, {720.0f, 1680.0f}, {320.0f, 1680.0f},  // 为当前列表或跨行表达式提供这一项 {2060.0f, 1680.0f}, {1680.0f, 1680.0f}, {1180.0f, 1680…。
    {320.0f, 2025.0f}, {720.0f, 2025.0f}, {1180.0f, 2025.0f}, {1680.0f, 2025.0f}, {2060.0f, 2025.0f},  // 为当前列表或跨行表达式提供这一项 {320.0f, 2025.0f}, {720.0f, 2025.0f}, {1180.0f, 2025.0…。
};  // 结束此初始化列表或局部作用域。

constexpr uint32_t MOTOR_COMMAND_GAP_MS = 5;           // 连续发送两条电机命令的间隔
constexpr double MOTOR_MOTION_TIME_MARGIN = 1.00;      // 运动估算增加 0% 等待余量
uint32_t ROUTE_SETTLE_TIME_MS = 100;         // 每次运动结束后的停车稳定时间

// ================= 连续视觉对齐 PID 参数 =================
// 输出均为 -1~1 的归一化车身速度权重，实际轮速由调用方传入的 speedRpm 决定。
// 顺序对应 VisionStartMode；各行独立调整，X/Y 为视觉单位，角度为度。
// 下列源码初值与 robot_parameters.json 保持一致；启动时仍由编译进固件的 JSON 默认值覆盖。
VisionAlignmentDeadzone ALIGN_DEADZONES[] = {  // 声明 ALIGN_DEADZONES，类型和可用范围由所在声明决定。
    {10.0f, 10.0f, 0.3f}, // DISC 圆盘对齐
    {5.0f, 5.0f, 0.1f},   // DISC_MATERIAL 圆盘物料对齐
    {1.0f, 1.0f, 0.1f},   // WORK_AREA
    {3.0f, 3.0f, 0.1f},   // WORK_AREA_LOADED
    {8.0f, 8.0f, 0.3f},   // CORNER
};  // 结束此初始化列表或局部作用域。
AlignmentFrameWaiter s_alignmentWaiter;  // 声明 s_alignmentWaiter，类型和可用范围由所在声明决定。
constexpr float ALIGN_DT_MIN_SECONDS = 0.02f;// 时间步长单位为秒
constexpr float ALIGN_DT_MAX_SECONDS = 0.30f;// 时间步长单位为秒
constexpr uint16_t OMNI_MIN_MOVING_RPM = 1;// 小误差最低启动轮速，单位RPM；原为1
constexpr uint8_t OMNI_ACCELERATION = 0;// 加速度单位为转/分^2

struct PidController {  // 定义 PidController 的类型成员或作用域。
    float kp;  // 声明 kp，类型和可用范围由所在声明决定。
    float ki;  // 声明 ki，类型和可用范围由所在声明决定。
    float kd;  // 声明 kd，类型和可用范围由所在声明决定。
    float integralLimit;      // 积分项上限
    float integral;           // 积分项
    float previousError;      // 上一次误差
    bool hasPreviousError;    // 是否有上一次误差
};  // 结束此类型定义。

// 位置误差单位为视觉输出值；角度误差单位为度。
PidController s_alignVisualXPid = {  // 将 { 写入或初始化 s_alignVisualXPid（salignVisual横向PID）。
    0.01f, 0.0f, 0.0f, 300.0f, 0.0f, 0.0f, false  // 续写当前声明或运算，提供 0.01f, 0.0f, 0.0f, 300.0f, 0.0f, 0.0f, false。
};  // 结束此初始化列表或局部作用域。
PidController s_alignVisualYPid = {  // 将 { 写入或初始化 s_alignVisualYPid（salignVisual纵向PID）。
    0.01f, 0.0005f, 0.0f, 300.0f, 0.0f, 0.0f, false  // 续写当前声明或运算，提供 0.01f, 0.0005f, 0.0f, 300.0f, 0.0f, 0.0f, false。
};  // 结束此初始化列表或局部作用域。
PidController s_alignAnglePid = {  // 将 { 写入或初始化 s_alignAnglePid（salign角度PID）。
    0.03f, 0.0005f, 0.0f, 30.0f, 0.0f, 0.0f, false  // 续写当前声明或运算，提供 0.03f, 0.0005f, 0.0f, 30.0f, 0.0f, 0.0f, false。
};  // 结束此初始化列表或局部作用域。
// CORNER 独立调参，并独立保存积分和微分历史。
PidController s_cornerVisualXPid = {  // 将 { 写入或初始化 s_cornerVisualXPid（scornerVisual横向PID）。
    0.005f, 0.0f, 0.0f, 200.0f, 0.0f, 0.0f, false  // 续写当前声明或运算，提供 0.005f, 0.0f, 0.0f, 200.0f, 0.0f, 0.0f, false。
};  // 结束此初始化列表或局部作用域。
PidController s_cornerVisualYPid = {  // 将 { 写入或初始化 s_cornerVisualYPid（scornerVisual纵向PID）。
    0.005f, 0.0f, 0.0f, 300.0f, 0.0f, 0.0f, false  // 续写当前声明或运算，提供 0.005f, 0.0f, 0.0f, 300.0f, 0.0f, 0.0f, false。
};  // 结束此初始化列表或局部作用域。
PidController s_cornerAnglePid = {  // 将 { 写入或初始化 s_cornerAnglePid（scorner角度PID）。
    0.015f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f, false  // 续写当前声明或运算，提供 0.015f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f, false。
};  // 结束此初始化列表或局部作用域。

float clampFloat(float value, float minimum, float maximum) {  // 定义函数或成员 的入口 clampFloat，参数由此处的签名确定。
    return fmaxf(minimum, fminf(value, maximum));  // 结束当前函数并返回 fmaxf(minimum, fminf(value, maximum))。
}  // 结束此函数或控制块。

void resetPid(PidController &pid) {  // 定义函数或成员 的入口 resetPid，参数由此处的签名确定。
    pid.integral = 0.0f;  // 将 0.0f 写入或初始化 pid.integral。
    pid.previousError = 0.0f;  // 将 0.0f 写入或初始化 pid.previousError。
    pid.hasPreviousError = false;  // 将 false 写入或初始化 pid.hasPreviousError。
}  // 结束此函数或控制块。

float updatePid(PidController &pid, float error, float dtSeconds) {  // 定义函数或成员 的入口 updatePid，参数由此处的签名确定。
    pid.integral = clampFloat(  // 将 clampFloat( 写入或初始化 pid.integral。
        pid.integral + error * dtSeconds,  // 为当前列表或跨行表达式提供这一项 pid.integral + error * dtSeconds。
        -pid.integralLimit, pid.integralLimit  // 续写当前声明或运算，提供 -pid.integralLimit, pid.integralLimit。
    );  // 续写当前声明或运算，提供 );。
    float derivative = 0.0f;  // 将 0.0f 写入或初始化 derivative（微分）。
    if (pid.hasPreviousError) {  // 当 pid.hasPreviousError 成立时进入此分支，否则继续后续流程。
        derivative = (error - pid.previousError) / dtSeconds;  // 将 (error - pid.previousError) / dtSeconds 写入或初始化 derivative（微分）。
    }  // 结束此条件分支。
    pid.previousError = error;  // 将 error 写入或初始化 pid.previousError。
    pid.hasPreviousError = true;  // 将 true 写入或初始化 pid.hasPreviousError。

    return clampFloat(  // 结束当前函数并返回 clampFloat(。
        pid.kp * error + pid.ki * pid.integral + pid.kd * derivative,  // 为当前列表或跨行表达式提供这一项 pid.kp * error + pid.ki * pid.integral + pid.kd * deri…。
        -1.0f, 1.0f  // 续写当前声明或运算，提供 -1.0f, 1.0f。
    );  // 续写当前声明或运算，提供 );。
}  // 结束此函数或控制块。

// 根据节点序号读取坐标。序号只能是 0~24。
bool getNodePosition(uint8_t node, NodePosition &position) {  // 定义函数或成员 的入口 getNodePosition（get节点位置），参数由此处的签名确定。
    if (node >= FIELD_NODE_COUNT) {  // 当 node >= FIELD_NODE_COUNT 成立时进入此分支，否则继续后续流程。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。
    position = NODE_POSITIONS[node];  // 将 NODE_POSITIONS[node] 写入或初始化 position（位置）。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

// 把任意航向角换算到 0~360 度范围。
float normalizeHeading(float heading) {  // 定义函数或成员 的入口 normalizeHeading（normalize航向），参数由此处的签名确定。
    heading = fmodf(heading, 360.0f);  // 将 fmodf(heading, 360.0f) 写入或初始化 heading（航向）。
    if (heading < 0.0f) {  // 当 heading < 0.0f 成立时进入此分支，否则继续后续流程。
        heading += 360.0f;  // 把 360.0f 累加到 heading（航向）。
    }  // 结束此条件分支。
    return heading;  // 结束当前函数并返回 heading。
}  // 结束此函数或控制块。

} // namespace

/**
 * @brief 按节点编号读取当前地图X/Y坐标(mm)，成功时写入x和y并返回true。
 */
bool ChassisGetNodePosition(uint8_t node,float &x,float &y) {  // 定义函数或成员 的入口 ChassisGetNodePosition（ChassisGet节点位置），参数由此处的签名确定。
    NodePosition p;  // 声明 p，类型和可用范围由所在声明决定。
    if(!getNodePosition(node,p))return false;  // 若 !getNodePosition(node,p)，立即返回 false。
    x=p.x;y=p.y;return true;  // 将 p.x 写入或初始化 x（横向）。然后将 p.y 写入或初始化 y（纵向）。然后结束当前函数并返回 true。
}  // 结束此函数或控制块。

// 注册原变量地址，上位机修改后由底盘和 PID 算法直接使用。
/**
 * @brief 在参数表末尾追加角点专用PID，保持此前参数编号顺序。
 */
void RegisterCornerAlignmentParameters() {  // 定义函数或成员 的入口 RegisterCornerAlignmentParameters，参数由此处的签名确定。
    PidController *controllers[] = {&s_cornerVisualXPid, &s_cornerVisualYPid, &s_cornerAnglePid};  // 声明 controllers，类型和可用范围由所在声明决定。
    const char *axes[] = {"视觉X", "视觉Y", "角度"};  // 声明 axes，类型和可用范围由所在声明决定。
    const char *fields[] = {"Kp", "Ki", "Kd", "积分限幅"};  // 声明 fields，类型和可用范围由所在声明决定。
    for (uint8_t i = 0; i < 3; ++i) {  // 按 uint8_t i = 0; i < 3; ++i 组织循环的初值、继续条件和步进。
        float *values[] = {&controllers[i]->kp, &controllers[i]->ki,  // 续列跨行签名或声明中的参数，类型与名称为 float *values[] = {&controllers[i]->kp, &controllers[i…。
                           &controllers[i]->kd, &controllers[i]->integralLimit};  // 续写当前声明或运算，提供 &controllers[i]->kd, &controllers[i]->integralLimit};。
        for (uint8_t j = 0; j < 4; ++j) {  // 按 uint8_t j = 0; j < 4; ++j 组织循环的初值、继续条件和步进。
            char name[80];  // 声明 name，类型和可用范围由所在声明决定。
            snprintf(name, sizeof(name), "PID/CORNER/%s/%s", axes[i], fields[j]);  // 调用 snprintf，传入 name, sizeof(name), "PID/CORNER/%s/%s", axes[i], field…。
            RegisterRuntimeFloat(name, *values[j], 0, j == 3 ? 100000 : 100);  // 调用 RegisterRuntimeFloat，传入 name, *values[j], 0, j == 3 ? 100000 : 100。
        }  // 结束此循环体。
    }  // 结束此循环体。
}  // 结束此函数或控制块。

/**
 * @brief 注册底盘运动、视觉PID、死区、节点地图和脉冲标定参数。
 */
void RegisterChassisParameters() {  // 定义函数或成员 的入口 RegisterChassisParameters，参数由此处的签名确定。
    PidController *controllers[] = {&s_alignVisualXPid, &s_alignVisualYPid, &s_alignAnglePid};  // 声明 controllers，类型和可用范围由所在声明决定。
    const char *axes[] = {"视觉X", "视觉Y", "角度"};  // 声明 axes，类型和可用范围由所在声明决定。
    for (uint8_t i = 0; i < 3; ++i) {  // 按 uint8_t i = 0; i < 3; ++i 组织循环的初值、继续条件和步进。
        char name[64];  // 声明 name，类型和可用范围由所在声明决定。
        float *values[] = {&controllers[i]->kp, &controllers[i]->ki,  // 续列跨行签名或声明中的参数，类型与名称为 float *values[] = {&controllers[i]->kp, &controllers[i…。
                           &controllers[i]->kd, &controllers[i]->integralLimit};  // 续写当前声明或运算，提供 &controllers[i]->kd, &controllers[i]->integralLimit};。
        const char *fields[] = {"Kp", "Ki", "Kd", "积分限幅"};  // 声明 fields，类型和可用范围由所在声明决定。
        for (uint8_t j = 0; j < 4; ++j) {  // 按 uint8_t j = 0; j < 4; ++j 组织循环的初值、继续条件和步进。
            snprintf(name, sizeof(name), "PID/%s/%s", axes[i], fields[j]);  // 调用 snprintf，传入 name, sizeof(name), "PID/%s/%s", axes[i], fields[j]。
            RegisterRuntimeFloat(name, *values[j], 0, j == 3 ? 100000 : 100);  // 调用 RegisterRuntimeFloat，传入 name, *values[j], 0, j == 3 ? 100000 : 100。
        }  // 结束此循环体。
    }  // 结束此循环体。
    const char *modes[] = {"DISC", "DISC_MATERIAL", "WORK_AREA", "WORK_AREA_LOADED", "CORNER"};  // 声明 modes，类型和可用范围由所在声明决定。
    for (uint8_t i = 0; i < 5; ++i) {  // 按 uint8_t i = 0; i < 5; ++i 组织循环的初值、继续条件和步进。
        char name[80];  // 声明 name，类型和可用范围由所在声明决定。
        snprintf(name, sizeof(name), "对齐/%s/X死区", modes[i]);  // 调用 snprintf，传入 name, sizeof(name), "对齐/%s/X死区", modes[i]。
        RegisterRuntimeFloat(name, ALIGN_DEADZONES[i].visualX, 0, 1000);  // 调用 RegisterRuntimeFloat，传入 name, ALIGN_DEADZONES[i].visualX, 0, 1000。
        snprintf(name, sizeof(name), "对齐/%s/Y死区", modes[i]);  // 调用 snprintf，传入 name, sizeof(name), "对齐/%s/Y死区", modes[i]。
        RegisterRuntimeFloat(name, ALIGN_DEADZONES[i].visualY, 0, 1000);  // 调用 RegisterRuntimeFloat，传入 name, ALIGN_DEADZONES[i].visualY, 0, 1000。
        snprintf(name, sizeof(name), "对齐/%s/角度死区(deg)", modes[i]);  // 调用 snprintf，传入 name, sizeof(name), "对齐/%s/角度死区(deg)", modes[i]。
        RegisterRuntimeFloat(name, ALIGN_DEADZONES[i].angleDeg, 0, 180);  // 调用 RegisterRuntimeFloat，传入 name, ALIGN_DEADZONES[i].angleDeg, 0, 180。
    }  // 结束此循环体。
    RegisterRuntimeUInt("底盘/停车稳定时间(ms)", ROUTE_SETTLE_TIME_MS, 0, 10000);  // 调用 RegisterRuntimeUInt，传入 "底盘/停车稳定时间(ms)", ROUTE_SETTLE_TIME_MS, 0, 10000。
    float *pulses[] = {&X_PULSE, &Y_PULSE, &THETA_PULSE, &HEIGHT_PULSE, &LENGTH_PULSE};  // 声明 pulses（脉冲数），类型和可用范围由所在声明决定。
    const char *names[] = {"标定/X脉冲每毫米", "标定/Y脉冲每毫米", "标定/旋转脉冲每度",  // 续列跨行签名或声明中的参数，类型与名称为 const char *names[] = {"标定/X脉冲每毫米", "标定/Y脉冲每毫米", "标定/旋…。
                           "标定/升降脉冲每毫米", "标定/伸缩脉冲每毫米"};  // 续写当前声明或运算，提供 "标定/升降脉冲每毫米", "标定/伸缩脉冲每毫米"};。
    for (uint8_t i = 0; i < 5; ++i) RegisterRuntimeFloat(names[i], *pulses[i], 0.001f, 100000);  // 按 uint8_t i = 0; i < 5; ++i) RegisterRuntimeFloat(names[… 组织循环的初值、继续条件和步进。
    for (uint8_t i = 0; i < FIELD_NODE_COUNT; ++i) {  // 按 uint8_t i = 0; i < FIELD_NODE_COUNT; ++i 组织循环的初值、继续条件和步进。
        char name[64];  // 声明 name，类型和可用范围由所在声明决定。
        snprintf(name, sizeof(name), "地图/节点%u/X(mm)", i);  // 调用 snprintf，传入 name, sizeof(name), "地图/节点%u/X(mm)", i。
        RegisterRuntimeFloat(name, NODE_POSITIONS[i].x, 0, 2400);  // 调用 RegisterRuntimeFloat，传入 name, NODE_POSITIONS[i].x, 0, 2400。
        snprintf(name, sizeof(name), "地图/节点%u/Y(mm)", i);  // 调用 snprintf，传入 name, sizeof(name), "地图/节点%u/Y(mm)", i。
        RegisterRuntimeFloat(name, NODE_POSITIONS[i].y, 0, 2400);  // 调用 RegisterRuntimeFloat，传入 name, NODE_POSITIONS[i].y, 0, 2400。
    }  // 结束此循环体。
}  // 结束此函数或控制块。


/**
 * @brief 读取张大头转盘实际多圈角度(deg)，沿用线缆范围检查。
 */
bool ReadArmTurretAngle(float &angle) {  // 定义函数或成员 的入口 ReadArmTurretAngle（Read机械臂转台角度），参数由此处的签名确定。
    if (taskMotionAborted) return false;  // 若 taskMotionAborted，立即返回 false。
    float measuredAngle = 0.0f;  // 将 0.0f 写入或初始化 measuredAngle（measured角度）。
    if (!TurretMotorReadAngle(measuredAngle) || !isfinite(measuredAngle)) {  // 当 !TurretMotorReadAngle(measuredAngle) || !isfinite(meas… 成立时进入此分支，否则继续后续流程。
        Serial.println("[Arm] ERR: turret multi-turn angle read failed; motion skipped");  // 调用 Serial.println，传入 "[Arm] ERR: turret multi-turn angle read failed; motio…。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。
    // 多圈反馈代表线缆累计转角；读取失败时保留调用方原值。
    if (measuredAngle < TURRET_CABLE_MIN_DEG || measuredAngle > TURRET_CABLE_MAX_DEG) {  // 当 measuredAngle < TURRET_CABLE_MIN_DEG || measuredAngle … 成立时进入此分支，否则继续后续流程。
        Serial.printf("[Arm] ERR: turret actual %.1f outside cable range [-150,360]; motion skipped\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                      measuredAngle);  // 续写当前声明或运算，提供 measuredAngle);。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。
    if (taskMotionAborted) return false;  // 若 taskMotionAborted，立即返回 false。
    angle = measuredAngle;  // 将 measuredAngle 写入或初始化 angle（角度）。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

/**
 * @brief 机械臂移动到指定位姿
 * @param high 大臂高度 (mm)
 * @param length 小臂伸出长度 (mm)
 * @param turret_angle 转台角度 (°)
 * @param pawl_angle 夹爪角度 (°)
 * @param speed 速度 (mm/s)
 * @note 规划模式下转台使用精确多圈角度；其他情况下 -1 表示不操作该轴
 */
static bool MoveArmInternal(float high, float length, float turret_angle, float pawl_angle, float speed,  // 定义函数或成员 MoveArmInternal（Move机械臂Internal），参数由此处的签名确定。
    bool absoluteTurret, uint8_t turretAcceleration, bool slowLower, uint16_t turretRpm,  // 续列跨行签名或声明中的参数，类型与名称为 bool absoluteTurret, uint8_t turretAcceleration, bool …。
    uint8_t lengthAcceleration, uint8_t slowLowerAcceleration) {  // 声明 lengthAcceleration（长度加速度），类型和可用范围由所在声明决定。
    if (taskMotionAborted) return false;  // 若 taskMotionAborted，立即返回 false。
    // 夹爪原标定为 speed=200 对应约90°/s；时间缩短至1/5后等效为450°/s。
    // 实际指令耗时按回读角度到最近等效目标的行程计算，不再使用与角度无关的线性映射。
    constexpr float GRIPPER_REFERENCE_COMMAND_SPEED = 200.0f; // 夹爪角速度标定时采用的指令速度
    constexpr float GRIPPER_REFERENCE_ANGULAR_SPEED_DEG_S = 450.0f; // 缩短动作时间后的等效角速度
    constexpr uint32_t GRIPPER_MIN_TIME_MS = 40;  // 将 40 写入或初始化 GRIPPER_MIN_TIME_MS（GRIPPER最小时间毫秒）。
    float safeTurretTarget = 0.0f;  // 将 0.0f 写入或初始化 safeTurretTarget（safe转台目标）。
    const bool moveTurret = absoluteTurret || turret_angle != -1.0f;  // 将 absoluteTurret || turret_angle != -1.0f 写入或初始化 moveTurret（move转台）。
    if (moveTurret) {  // 当 moveTurret 成立时进入此分支，否则继续后续流程。
        float measuredAngle = 0.0f;  // 将 0.0f 写入或初始化 measuredAngle（measured角度）。
        if (!ReadArmTurretAngle(measuredAngle)) return false;  // 若 !ReadArmTurretAngle(measuredAngle)，立即返回 false。
        // 目标为带符号的累计绝对角度，保留圈数，不选择±360度等效角。
        safeTurretTarget = turret_angle;  // 将 turret_angle 写入或初始化 safeTurretTarget（safe转台目标）。
    }  // 结束此条件分支。

    if (taskMotionAborted) return false;  // 若 taskMotionAborted，立即返回 false。
    // 绝对坐标以开机清零位置为原点；缓存不是驱动器实际位置反馈。
    if (high != -1) {  // 当 high != -1 成立时进入此分支，否则继续后续流程。
        const uint32_t pulses = static_cast<uint32_t>(high * HEIGHT_PULSE);  // 将 static_cast<uint32_t>(high * HEIGHT_PULSE) 写入或初始化 pulses（脉冲数）。
        // RPM 写入协议的 16 位字段，在此完成数值饱和。
        const uint16_t rpm = speed >= UINT16_MAX / 15.0f ? UINT16_MAX : speed * 15.0f;  // 将 speed >= UINT16_MAX / 15.0f ? UINT16_MAX : speed * 15.… 写入或初始化 rpm（转速）。
        Emm_V5_Pos_Control(5, 1, slowLower ? rpm / 2 : rpm, slowLower ? slowLowerAcceleration : 252, pulses, true, false);  // 调用 Emm_V5_Pos_Control（EmmV5位置Control），传入 5, 1, slowLower ? rpm / 2 : rpm, slowLower ? slowLower…。
        vTaskDelay(pdMS_TO_TICKS(10));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        currentArm.high = high;  // 将 high 写入或初始化 currentArm.high。
    }  // 结束此条件分支。

    if (taskMotionAborted) return false;  // 若 taskMotionAborted，立即返回 false。
    if (length != -1) {  // 当 length != -1 成立时进入此分支，否则继续后续流程。
        const uint32_t pulses = static_cast<uint32_t>(length * LENGTH_PULSE);  // 将 static_cast<uint32_t>(length * LENGTH_PULSE) 写入或初始化 pulses（脉冲数）。
        const uint16_t rpm = speed >= UINT16_MAX / 6.0f ? UINT16_MAX : speed * 6.0f;  // 将 speed >= UINT16_MAX / 6.0f ? UINT16_MAX : speed * 6.0f 写入或初始化 rpm（转速）。
        Emm_V5_Pos_Control(6, 1, rpm, lengthAcceleration, pulses, true, false);  // 调用 Emm_V5_Pos_Control（EmmV5位置Control），传入 6, 1, rpm, lengthAcceleration, pulses, true, false。
        vTaskDelay(pdMS_TO_TICKS(10));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        currentArm.length = length;  // 将 length 写入或初始化 currentArm.length。
    }  // 结束此条件分支。

    if (taskMotionAborted) return false;  // 若 taskMotionAborted，立即返回 false。
    if (moveTurret) {  // 当 moveTurret 成立时进入此分支，否则继续后续流程。
        if(!TurretMotorMove(safeTurretTarget, turretAcceleration, turretRpm))return false;  // 若 !TurretMotorMove(safeTurretTarget, turretAcceleration,…，立即返回 false。
        currentArm.turret_angle = safeTurretTarget;  // 将 safeTurretTarget 写入或初始化 currentArm.turret_angle。
    }  // 结束此条件分支。

    if (taskMotionAborted) return false;  // 若 taskMotionAborted，立即返回 false。
    if (pawl_angle != -1) {  // 当 pawl_angle != -1 成立时进入此分支，否则继续后续流程。
        pawl_angle = LimitArmClawAngle(pawl_angle);  // 将 LimitArmClawAngle(pawl_angle) 写入或初始化 pawl_angle（夹爪角度）。
        float measuredAngle = 0.0f;  // 将 0.0f 写入或初始化 measuredAngle（measured角度）。
        if ((!Servo_QueryAngleMTurn(1, measuredAngle, 100)&&!Servo_QueryAngleMTurn(1, measuredAngle, 100))  // 当 (!Servo_QueryAngleMTurn(1, measuredAngle, 100)&&!Servo… 成立时进入此分支，否则继续后续流程。
                || !isfinite(measuredAngle)) {  // 继续上一行的组合条件，本行提供 || !isfinite(measuredAngle)) {。
            Serial.println("[Arm] ERR: gripper multi-turn angle read failed; motion skipped");  // 调用 Serial.println，传入 "[Arm] ERR: gripper multi-turn angle read failed; moti…。
            return false;  // 结束当前函数并返回 false。
        }  // 结束此函数或控制块。
        const float nearestAngle = pawl_angle  // 将 pawl_angle 写入或初始化 nearestAngle（nearest角度）。
            + 360.0f * roundf((measuredAngle - pawl_angle) / 360.0f);  // 声明接口 roundf，参数由此处的签名确定。
        const float travelDeg = fabsf(nearestAngle - measuredAngle);  // 将 fabsf(nearestAngle - measuredAngle) 写入或初始化 travelDeg（travel角度）。
        const float calculatedMs = travelDeg / speed  // 将 travelDeg / speed 写入或初始化 calculatedMs（calculated毫秒）。
            * (1000.0f * GRIPPER_REFERENCE_COMMAND_SPEED / GRIPPER_REFERENCE_ANGULAR_SPEED_DEG_S);  // 续写当前声明或运算，提供 * (1000.0f * GRIPPER_REFERENCE_COMMAND_SPEED / GRIPPER…。
        const uint32_t gripperIntervalMs = travelDeg == 0.0f ? 0  // 将 travelDeg == 0.0f ? 0 写入或初始化 gripperIntervalMs（gripperInterval毫秒）。
            : (calculatedMs < GRIPPER_MIN_TIME_MS ? GRIPPER_MIN_TIME_MS  // 续写上一行表达式的条件取值分支。
               : static_cast<uint32_t>(calculatedMs + 0.5f));  // 声明接口 uint32_t>，参数由此处的签名确定。
        if (taskMotionAborted) return false;  // 若 taskMotionAborted，立即返回 false。
        Servo_SetAngleMTurn(1, nearestAngle, gripperIntervalMs, 0);  // 调用 Servo_SetAngleMTurn（ServoSet角度MTurn），传入 1, nearestAngle, gripperIntervalMs, 0。
        currentArm.pawl_angle = pawl_angle;  // 将 pawl_angle 写入或初始化 currentArm.pawl_angle。
    }  // 结束此条件分支。

    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

bool MoveArm(float high, float length, float turret_angle, float pawl_angle, float speed,  // 定义函数或成员 MoveArm（Move机械臂），参数由此处的签名确定。
    uint8_t turretAcceleration, bool slowLower, uint8_t lengthAcceleration, uint8_t slowLowerAcceleration) {  // 声明 lengthAcceleration（长度加速度），类型和可用范围由所在声明决定。
    return MoveArmInternal(high, length, turret_angle, pawl_angle, speed, false,  // 结束当前函数并返回 MoveArmInternal(high, length, turret_angle, pawl_angle…。
        turretAcceleration, slowLower, TURRET_MOTOR_POSITION_RPM, lengthAcceleration, slowLowerAcceleration);  // 声明 lengthAcceleration（长度加速度），类型和可用范围由所在声明决定。
}  // 结束此函数或控制块。

/**
 * @brief 按已规划的多圈绝对转台目标发令，不再选择最近等效角；其他轴的-1仍表示跳过。
 * @return true 指令下发成功，不表示实测到位。
 */
bool MoveArmPlanned(float high, float length, float turret_angle, float pawl_angle, float speed,  // 定义函数或成员 MoveArmPlanned（Move机械臂Planned），参数由此处的签名确定。
    uint8_t turretAcceleration, bool slowLower, uint16_t turretRpm) {  // 声明 slowLower，类型和可用范围由所在声明决定。
    return MoveArmInternal(high, length, turret_angle, pawl_angle, speed, true,  // 结束当前函数并返回 MoveArmInternal(high, length, turret_angle, pawl_angle…。
        turretAcceleration, slowLower, turretRpm, ARM_EXTENSION_ACCELERATION, ARM_SLOW_LOWER_ACCELERATION);  // 声明 ARM_EXTENSION_ACCELERATION（机械臂伸缩加速度），类型和可用范围由所在声明决定。
}  // 结束此函数或控制块。


/**
 * @brief 速度模式平移或原地旋转(麦克纳姆轮, 通用)
 * @param direction 0=前进 1=后退 2=左移 3=右移 4=左转 5=右转
 * @param speed   电机转速 (RPM)
 * @param stop    true=停止 false=开始移动
 */
void MovePose(int direction, float speed, bool stop) {  // 定义函数或成员 的入口 MovePose，参数由此处的签名确定。
    static bool isMoving = false;  // 将 false 写入或初始化 isMoving。

    if (stop) {  // 当 stop 成立时进入此分支，否则继续后续流程。
        if (isMoving) {  // 当 isMoving 成立时进入此分支，否则继续后续流程。
            //Emm_V5_Stop_Now(0, 0);
            Emm_V5_Vel_Control(1, 0, 0, 230, 1);  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 1, 0, 0, 230, 1。
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
            Emm_V5_Vel_Control(2, 0, 0, 230, 1);  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 2, 0, 0, 230, 1。
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
            Emm_V5_Vel_Control(3, 0, 0, 230, 1);  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 3, 0, 0, 230, 1。
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
            Emm_V5_Vel_Control(4, 0, 0, 230, 1);  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 4, 0, 0, 230, 1。
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
            Emm_V5_Synchronous_motion(0);  // 调用 Emm_V5_Synchronous_motion，传入 0。
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
            // [TODO] 停止后按实际反馈更新位姿(如编码器/里程计/视觉)
            isMoving = false;  // 将 false 写入或初始化 isMoving。
            Serial.println("move stopped");  // 调用 Serial.println，传入 "move stopped"。
        }  // 结束此条件分支。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。

    if (direction < 0 || direction > 5) {  // 当 direction < 0 || direction > 5 成立时进入此分支，否则继续后续流程。
        Serial.println("MovePose direction error, should be 0=fwd, 1=back, 2=left, 3=right, 4=turn left, 5=turn right");  // 调用 Serial.println，传入 "MovePose direction error, should be 0=fwd, 1=back, 2=…。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。

    vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。

    if (direction == 0) { // 前进
        Emm_V5_Vel_Control(1, 1, speed, 50, 1);  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 1, 1, speed, 50, 1。
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        Emm_V5_Vel_Control(2, 0, speed, 50, 1);  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 2, 0, speed, 50, 1。
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        Emm_V5_Vel_Control(3, 1, speed, 50, 1);  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 3, 1, speed, 50, 1。
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        Emm_V5_Vel_Control(4, 0, speed, 50, 1);  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 4, 0, speed, 50, 1。
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        Emm_V5_Synchronous_motion(0);  // 调用 Emm_V5_Synchronous_motion，传入 0。
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    } else if (direction == 1) {       // 后退
        Emm_V5_Vel_Control(1, 0, speed, 50, 1);  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 1, 0, speed, 50, 1。
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        Emm_V5_Vel_Control(2, 1, speed, 50, 1);  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 2, 1, speed, 50, 1。
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        Emm_V5_Vel_Control(3, 0, speed, 50, 1);  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 3, 0, speed, 50, 1。
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        Emm_V5_Vel_Control(4, 1, speed, 50, 1);  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 4, 1, speed, 50, 1。
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        Emm_V5_Synchronous_motion(0);  // 调用 Emm_V5_Synchronous_motion，传入 0。
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    } else if (direction == 2) { // 左
        Emm_V5_Vel_Control(1, 1, speed, 50, 1);  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 1, 1, speed, 50, 1。
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        Emm_V5_Vel_Control(2, 1, speed, 50, 1);  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 2, 1, speed, 50, 1。
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        Emm_V5_Vel_Control(3, 0, speed, 50, 1);  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 3, 0, speed, 50, 1。
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        Emm_V5_Vel_Control(4, 0, speed, 50, 1);  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 4, 0, speed, 50, 1。
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        Emm_V5_Synchronous_motion(0);  // 调用 Emm_V5_Synchronous_motion，传入 0。
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    } else if (direction == 3) { // 右
        Emm_V5_Vel_Control(1, 0, speed, 50, 1);  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 1, 0, speed, 50, 1。
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        Emm_V5_Vel_Control(2, 0, speed, 50, 1);  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 2, 0, speed, 50, 1。
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        Emm_V5_Vel_Control(3, 1, speed, 50, 1);  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 3, 1, speed, 50, 1。
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        Emm_V5_Vel_Control(4, 1, speed, 50, 1);  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 4, 1, speed, 50, 1。
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        Emm_V5_Synchronous_motion(0);  // 调用 Emm_V5_Synchronous_motion，传入 0。
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    } else if (direction == 4 || direction == 5) {  // 仅在先前条件均不成立时，当 direction == 4 || direction == 5 成立时进入此分支，否则继续后续流程。
        // 沿用 GotoPose 正角左转、负角右转的四轮方向约定。
        const uint8_t motorDirection = (direction == 4) ? 1 : 0;  // 将 (direction == 4) ? 1 : 0 写入或初始化 motorDirection（motor方向）。
        for (uint8_t motor = 1; motor <= 4; ++motor) {  // 按 uint8_t motor = 1; motor <= 4; ++motor 组织循环的初值、继续条件和步进。
            Emm_V5_Vel_Control(motor, motorDirection, speed, 50, 1);  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 motor, motorDirection, speed, 50, 1。
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        }  // 结束此循环体。
        Emm_V5_Synchronous_motion(0);  // 调用 Emm_V5_Synchronous_motion，传入 0。
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    }  // 结束此条件分支。
    vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    isMoving = true;  // 将 true 写入或初始化 isMoving。
}  // 结束此函数或控制块。

/**
 * @brief 位置模式移动到位(麦克纳姆轮运动学, 通用)
 * @param isRelative true=相对坐标 false=绝对坐标
 * 注意: 4 轮同向=平移, 4 轮同转向=原地旋转. 具体轮序(direction)
 *       需按实际电机接线确认. [TODO]
 */
void GotoPose(float x, float y, float theta, bool isRelative) {  // 定义函数或成员 的入口 GotoPose，参数由此处的签名确定。
    int speed = 80;   // 移动速度

    if (isRelative) {  // 当 isRelative 成立时进入此分支，否则继续后续流程。
        // 平移 X (同向差速)
        if (x != 0) {  // 当 x != 0 成立时进入此分支，否则继续后续流程。
            uint8_t dir = (x > 0) ? 0 : 1;  // 将 (x > 0) ? 0 : 1 写入或初始化 dir。
            uint32_t pulses = (uint32_t)(fabsf(x) * X_PULSE);  // 将 (uint32_t)(fabsf(x) * X_PULSE) 写入或初始化 pulses（脉冲数）。
            Emm_V5_Pos_Control(1, dir, speed, 50, pulses, 0, 0);  // 调用 Emm_V5_Pos_Control（EmmV5位置Control），传入 1, dir, speed, 50, pulses, 0, 0。
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
            Emm_V5_Pos_Control(2, dir, speed, 50, pulses, 0, 0);  // 调用 Emm_V5_Pos_Control（EmmV5位置Control），传入 2, dir, speed, 50, pulses, 0, 0。
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
            Emm_V5_Pos_Control(3, !dir, speed, 50, pulses, 0, 0);  // 调用 Emm_V5_Pos_Control（EmmV5位置Control），传入 3, !dir, speed, 50, pulses, 0, 0。
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
            Emm_V5_Pos_Control(4, !dir, speed, 50, pulses, 0, 0);  // 调用 Emm_V5_Pos_Control（EmmV5位置Control），传入 4, !dir, speed, 50, pulses, 0, 0。
            vTaskDelay(pdMS_TO_TICKS(10));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        }  // 结束此条件分支。
        // 平移 Y
        if (y != 0) {  // 当 y != 0 成立时进入此分支，否则继续后续流程。
            uint8_t dir = (y > 0) ? 0 : 1;  // 将 (y > 0) ? 0 : 1 写入或初始化 dir。
            uint32_t pulses = (uint32_t)(fabsf(y) * Y_PULSE);  // 将 (uint32_t)(fabsf(y) * Y_PULSE) 写入或初始化 pulses（脉冲数）。
            Emm_V5_Pos_Control(1, !dir, speed, 50, pulses, 0, 0);  // 调用 Emm_V5_Pos_Control（EmmV5位置Control），传入 1, !dir, speed, 50, pulses, 0, 0。
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
            Emm_V5_Pos_Control(2,  dir, speed, 50, pulses, 0, 0);  // 调用 Emm_V5_Pos_Control（EmmV5位置Control），传入 2, dir, speed, 50, pulses, 0, 0。
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
            Emm_V5_Pos_Control(3, !dir, speed, 50, pulses, 0, 0);  // 调用 Emm_V5_Pos_Control（EmmV5位置Control），传入 3, !dir, speed, 50, pulses, 0, 0。
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
            Emm_V5_Pos_Control(4,  dir, speed, 50, pulses, 0, 0);  // 调用 Emm_V5_Pos_Control（EmmV5位置Control），传入 4, dir, speed, 50, pulses, 0, 0。
            vTaskDelay(pdMS_TO_TICKS(10));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        }  // 结束此条件分支。
        // 原地旋转
        if (theta != 0) {  // 当 theta != 0 成立时进入此分支，否则继续后续流程。
            uint8_t dir = (theta < 0) ? 0 : 1;  // 将 (theta < 0) ? 0 : 1 写入或初始化 dir。
            uint32_t pulses = (uint32_t)(fabsf(theta) * THETA_PULSE);  // 将 (uint32_t)(fabsf(theta) * THETA_PULSE) 写入或初始化 pulses（脉冲数）。
            Emm_V5_Pos_Control(1, dir, speed, 50, pulses, 0, 0);  // 调用 Emm_V5_Pos_Control（EmmV5位置Control），传入 1, dir, speed, 50, pulses, 0, 0。
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
            Emm_V5_Pos_Control(2, dir, speed, 50, pulses, 0, 0);  // 调用 Emm_V5_Pos_Control（EmmV5位置Control），传入 2, dir, speed, 50, pulses, 0, 0。
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
            Emm_V5_Pos_Control(3, dir, speed, 50, pulses, 0, 0);  // 调用 Emm_V5_Pos_Control（EmmV5位置Control），传入 3, dir, speed, 50, pulses, 0, 0。
            vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
            Emm_V5_Pos_Control(4, dir, speed, 50, pulses, 0, 0);  // 调用 Emm_V5_Pos_Control（EmmV5位置Control），传入 4, dir, speed, 50, pulses, 0, 0。
            vTaskDelay(pdMS_TO_TICKS(10));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
        }  // 结束此条件分支。
        // 命令下发完成后更新开环目标估计；不代表电机已到位。
        // 相对 X/Y 属于车体坐标，先按移动前航向转换到世界坐标。
        const float heading = currentPose.theta * PI / 180.0f;  // 将 currentPose.theta * PI / 180.0f 写入或初始化 heading（航向）。
        currentPose.x += x * cosf(heading) - y * sinf(heading);  // 把 x * cosf(heading) - y * sinf(heading) 累加到 currentPose.x。
        currentPose.y += x * sinf(heading) + y * cosf(heading);  // 把 x * sinf(heading) + y * cosf(heading) 累加到 currentPose.y。
        currentPose.theta = normalizeHeading(currentPose.theta + theta);  // 将 normalizeHeading(currentPose.theta + theta) 写入或初始化 currentPose.theta。
    } else {  // 前一个判断不成立时执行此分支。
        // 绝对坐标: 需先获取当前位姿, 算出位移增量再调用相对移动.
        // [TODO] 依赖新定位方案, 由负责定位的成员实现
        // 例:
        //   RobotPose p = currentPose;   // 或定位融合结果
        //   GotoPose(x - p.x, y - p.y, theta - p.theta, true, false);
        Serial.println("[TODO] GotoPose absolute needs localization");  // 调用 Serial.println，传入 "[TODO] GotoPose absolute needs localization"。
    }  // 结束此条件分支。
}  // 结束此函数或控制块。

/**
 * @brief 麦克纳姆轮全向速度混合并同步下发四轮速度。
 */
void OmniMove(float xVelocity, float yVelocity, float rotationVelocity, uint16_t speedRpm) {  // 定义函数或成员 的入口 OmniMove，参数由此处的签名确定。
    // 方向符号与现有 MovePose/GotoPose 的四轮接线约定保持一致。
    float wheelVelocity[4] = {  // 将 { 写入或初始化 wheelVelocity[4]。
        -xVelocity + yVelocity + rotationVelocity,  // 为当前列表或跨行表达式提供这一项 -xVelocity + yVelocity + rotationVelocity。
        -xVelocity - yVelocity + rotationVelocity,  // 为当前列表或跨行表达式提供这一项 -xVelocity - yVelocity + rotationVelocity。
         xVelocity + yVelocity + rotationVelocity,  // 为当前列表或跨行表达式提供这一项 xVelocity + yVelocity + rotationVelocity。
         xVelocity - yVelocity + rotationVelocity,  // 为当前列表或跨行表达式提供这一项 xVelocity - yVelocity + rotationVelocity。
    };  // 结束此初始化列表或局部作用域。

    float maximumMagnitude = 0.0f;  // 将 0.0f 写入或初始化 maximumMagnitude。
    for (float value : wheelVelocity) {  // 按 float value : wheelVelocity 组织循环的初值、继续条件和步进。
        maximumMagnitude = fmaxf(maximumMagnitude, fabsf(value));// 计算最大绝对值
    }  // 结束此循环体。
    if (maximumMagnitude > 1.0f) {  // 当 maximumMagnitude > 1.0f 成立时进入此分支，否则继续后续流程。
        for (float &value : wheelVelocity) {  // 按 float &value : wheelVelocity 组织循环的初值、继续条件和步进。
            value /= maximumMagnitude;  // 用 maximumMagnitude 对 value（测值） 应用 /= 更新。
        }  // 结束此循环体。
        maximumMagnitude = 1.0f;  // 将 1.0f 写入或初始化 maximumMagnitude。
    }  // 结束此条件分支。

    // 小误差时整体抬高四轮速度比例，使最大轮达到可启动转速，同时保持轮间比例。
    float rpmScale = static_cast<float>(speedRpm);  // 将 static_cast<float>(speedRpm) 写入或初始化 rpmScale（转速Scale）。
    const uint16_t minimumMovingRpm =  // 续写当前声明或运算，提供 const uint16_t minimumMovingRpm =。
        (speedRpm < OMNI_MIN_MOVING_RPM) ? speedRpm : OMNI_MIN_MOVING_RPM;  // 为当前列表或跨行表达式提供这一项 (speedRpm < OMNI_MIN_MOVING_RPM) ? speedRpm : OMNI_MIN…。
    if (maximumMagnitude > 0.0f  // 当 maximumMagnitude > 0.0f 成立时进入此分支，否则继续后续流程。
            && maximumMagnitude * rpmScale < minimumMovingRpm) {  // 继续上一行的组合条件，本行提供 && maximumMagnitude * rpmScale < minimumMovingRpm) {。
        rpmScale = static_cast<float>(minimumMovingRpm) / maximumMagnitude;  // 将 static_cast<float>(minimumMovingRpm) / maximumMagnitude 写入或初始化 rpmScale（转速Scale）。
    }  // 结束此函数或控制块。

    vTaskDelay(pdMS_TO_TICKS(1));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。

    for (uint8_t motor = 1; motor <= 4; ++motor) {  // 按 uint8_t motor = 1; motor <= 4; ++motor 组织循环的初值、继续条件和步进。
        const float command = wheelVelocity[motor - 1];  // 将 wheelVelocity[motor - 1] 写入或初始化 command。
        uint16_t motorRpm = static_cast<uint16_t>( lroundf(fabsf(command) * rpmScale));  // 将 static_cast<uint16_t>( lroundf(fabsf(command) * rpmSca… 写入或初始化 motorRpm（motor转速）。
        const uint8_t direction = (command >= 0.0f) ? 1 : 0;  // 将 (command >= 0.0f) ? 1 : 0 写入或初始化 direction（方向）。
        Emm_V5_Vel_Control( motor, direction, motorRpm, OMNI_ACCELERATION, true );  // 调用 Emm_V5_Vel_Control（EmmV5速度Control），传入 motor, direction, motorRpm, OMNI_ACCELERATION, true。
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    }  // 结束此循环体。
    Emm_V5_Synchronous_motion(0);  // 调用 Emm_V5_Synchronous_motion，传入 0。
}  // 结束此函数或控制块。

namespace {  // 开始匿名命名空间，本文件内部共享其中的定义。
struct PositionExecution {  // 定义 PositionExecution（位置Execution） 的类型成员或作用域。
    bool active=false,success=true;  // 将 false,success=true 写入或初始化 active（活跃）。
    uint32_t started=0,duration=0;  // 将 0,duration=0 写入或初始化 started。
    RobotPose target{};  // 声明 target（目标），类型和可用范围由所在声明决定。
} positionExecution;  // 续写当前声明或运算，提供 } positionExecution;。
}  // 结束命名空间。
/**
 * @brief 初始化四轮同步相对位置动作，不等待估算行程结束；由TickPositionMove推进。
 * @return true 准备成功，false 请求无法执行。
 */
// 此入口创建一次底盘位置动作。
// 轮组的位移、方向和转速在发令前确定，活动状态和完成判定由后续轮询处理。
bool BeginPositionMove(float x,float y,float theta,float speed,uint8_t acceleration) {  // 定义函数或成员 的入口 BeginPositionMove（Begin位置Move），参数由此处的签名确定。
    positionExecution.active=false;positionExecution.success=true;  // 将 false 写入或初始化 positionExecution.active。然后将 true 写入或初始化 positionExecution.success。
    if(taskMotionAborted)return false;  // 若 taskMotionAborted，立即返回 false。
    MotorBusGuard transaction;  // 声明 transaction，类型和可用范围由所在声明决定。
    // SE(2) 指数映射的逆：x/y 是起始车身系中的终点位移。
    // 转动过程中车身系也在转动，不能直接把终点 x/y 当作累计车身位移。
    const double angle = static_cast<double>(theta) * PI / 180.0;  // 将 static_cast<double>(theta) * PI / 180.0 写入或初始化 angle（角度）。
    double bodyX = x;  // 将 x 写入或初始化 bodyX（车体横向）。
    double bodyY = y;  // 将 y 写入或初始化 bodyY（车体纵向）。
    if (fabs(angle) > 1.0e-6 && (x != 0.0f || y != 0.0f)) {  // 当 fabs(angle) > 1.0e-6 && (x != 0.0f || y != 0.0f) 成立时进入此分支，否则继续后续流程。
        const double a = sin(angle) / angle;  // 将 sin(angle) / angle 写入或初始化 a。
        const double b = (1.0 - cos(angle)) / angle;  // 将 (1.0 - cos(angle)) / angle 写入或初始化 b。
        const double determinant = a * a + b * b;  // 将 a * a + b * b 写入或初始化 determinant。
        if (determinant < 1.0e-10) {  // 当 determinant < 1.0e-10 成立时进入此分支，否则继续后续流程。
            Serial.println("[MovePosition] ERR: translation with full-turn rotation is singular");  // 调用 Serial.println，传入 "[MovePosition] ERR: translation with full-turn rotati…。
            return false;  // 结束当前函数并返回 false。
        }  // 结束此条件分支。
        bodyX = (a * x + b * y) / determinant;  // 将 (a * x + b * y) / determinant 写入或初始化 bodyX（车体横向）。
        bodyY = (-b * x + a * y) / determinant;  // 将 (-b * x + a * y) / determinant 写入或初始化 bodyY（车体纵向）。
    }  // 结束此条件分支。

    const double px = bodyX * X_PULSE;  // 将 bodyX * X_PULSE 写入或初始化 px。
    const double py = bodyY * Y_PULSE;  // 将 bodyY * Y_PULSE 写入或初始化 py。
    const double pr = static_cast<double>(theta) * THETA_PULSE;  // 将 static_cast<double>(theta) * THETA_PULSE 写入或初始化 pr。
    // 与 GotoPose/OmniMove 相同的电机 1~4 符号约定，正脉冲使用 dir=1。
    const double wheelPulses[4] = {-px + py + pr, -px - py + pr,  // 续列跨行签名或声明中的参数，类型与名称为 const double wheelPulses[4] = {-px + py + pr, -px - py…。
                                   px + py + pr,  px - py + pr};  // 续写当前声明或运算，提供 px + py + pr, px - py + pr};。
    uint32_t pulses[4];  // 声明 pulses（脉冲数），类型和可用范围由所在声明决定。
    uint16_t wheelRpm[4];  // 声明 wheelRpm（轮转速），类型和可用范围由所在声明决定。
    uint32_t maximumPulses = 0;  // 将 0 写入或初始化 maximumPulses（maximum脉冲数）。
    for (size_t i = 0; i < 4; ++i) {  // 按 size_t i = 0; i < 4; ++i 组织循环的初值、继续条件和步进。
        const double rounded = floor(fabs(wheelPulses[i]) + 0.5);  // 将 floor(fabs(wheelPulses[i]) + 0.5) 写入或初始化 rounded。
        pulses[i] = static_cast<uint32_t>(rounded);  // 将 static_cast<uint32_t>(rounded) 写入或初始化 pulses[i]。
        if (pulses[i] > maximumPulses) maximumPulses = pulses[i];  // 当 pulses[i] > maximumPulses 成立时进入此分支，否则继续后续流程。
    }  // 结束此循环体。
    if (maximumPulses == 0) return true; // 小于脉冲分辨率，不虚增理想位姿。

    const uint16_t maximumRpm = static_cast<uint16_t>(lroundf(speed));  // 将 static_cast<uint16_t>(lroundf(speed)) 写入或初始化 maximumRpm（maximum转速）。
    double waitMs = 0.0;  // 将 0.0 写入或初始化 waitMs（wait毫秒）。
    for (size_t i = 0; i < 4; ++i) {  // 按 size_t i = 0; i < 4; ++i 组织循环的初值、继续条件和步进。
        // 各轮按行程比例分配转速；整数 RPM 会带来少量完成时间差。
        wheelRpm[i] = pulses[i] == 0 ? 0 : static_cast<uint16_t>(fmax(  // 将 pulses[i] == 0 ? 0 : static_cast<uint16_t>(fmax( 写入或初始化 wheelRpm[i]。
            1.0, floor(static_cast<double>(pulses[i]) * maximumRpm / maximumPulses + 0.5)));  // 声明接口 floor，参数由此处的签名确定。
        if (wheelRpm[i] != 0) {  // 当 wheelRpm[i] != 0 成立时进入此分支，否则继续后续流程。
            waitMs = fmax(waitMs, PositionMoveDurationMs(pulses[i],wheelRpm[i],acceleration));  // 将 fmax(waitMs, PositionMoveDurationMs(pulses[i],wheelRpm… 写入或初始化 waitMs（wait毫秒）。
        }  // 结束此条件分支。
    }  // 结束此循环体。
    waitMs = ceil(waitMs * MOTOR_MOTION_TIME_MARGIN) + ROUTE_SETTLE_TIME_MS;  // 将 ceil(waitMs * MOTOR_MOTION_TIME_MARGIN) + ROUTE_SETTLE… 写入或初始化 waitMs（wait毫秒）。
    const double heading = normalizeHeading(currentPose.theta) * PI / 180.0;  // 将 normalizeHeading(currentPose.theta) * PI / 180.0 写入或初始化 heading（航向）。
    // 与导航/扫码统一：theta=0时车头(+Y)朝地图+X，车身+X朝地图-Y。
    const double targetX = currentPose.x + x * sin(heading) + y * cos(heading);  // 将 currentPose.x + x * sin(heading) + y * cos(heading) 写入或初始化 targetX（目标横向）。
    const double targetY = currentPose.y - x * cos(heading) + y * sin(heading);  // 将 currentPose.y - x * cos(heading) + y * sin(heading) 写入或初始化 targetY（目标纵向）。
    for (uint8_t motor = 1; motor <= 4; ++motor) {  // 按 uint8_t motor = 1; motor <= 4; ++motor 组织循环的初值、继续条件和步进。
        // 耗时计算与实际下发使用同一加速度档位。
        if (pulses[motor - 1] == 0) {  // 当 pulses[motor - 1] == 0 成立时进入此分支，否则继续后续流程。
            // 合成后不需转动的轮子同步停车，避免发送零转速位置命令。
            Emm_V5_Stop_Now(motor, true);  // 调用 Emm_V5_Stop_Now（EmmV5停止Now），传入 motor, true。
        } else {  // 前一个判断不成立时执行此分支。
            Emm_V5_Pos_Control(motor, wheelPulses[motor - 1] >= 0.0 ? 1 : 0, wheelRpm[motor - 1], acceleration, pulses[motor - 1], false, true);  // 调用 Emm_V5_Pos_Control（EmmV5位置Control），传入 motor, wheelPulses[motor - 1] >= 0.0 ? 1 : 0, wheelRpm…。
        }  // 结束此条件分支。
        vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
    }  // 结束此循环体。
    Emm_V5_Synchronous_motion(0);  // 调用 Emm_V5_Synchronous_motion，传入 0。
    positionExecution.started=millis();  // 将 millis() 写入或初始化 positionExecution.started。
    positionExecution.duration=static_cast<uint32_t>(waitMs);  // 将 static_cast<uint32_t>(waitMs) 写入或初始化 positionExecution.duration。
    positionExecution.target={static_cast<float>(targetX),static_cast<float>(targetY),  // 将 {static_cast<float>(targetX),static_cast<float>(target… 写入或初始化 positionExecution.target。
        normalizeHeading(normalizeHeading(currentPose.theta)+normalizeHeading(theta))};  // 调用 normalizeHeading（normalize航向），传入 normalizeHeading(currentPose.theta)+normalizeHeading(t…。
    positionExecution.active=true;  // 将 true 写入或初始化 positionExecution.active。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。
/**
 * @brief 判断相对位置动作是否仍处于等待结束的运行状态。
 */
bool PositionMoveActive(){return positionExecution.active;}  // 定义 PositionMoveActive（位置Move活跃），调用时返回 positionExecution.active。
/**
 * @brief 停止当前相对位置移动并清除运行状态。
 */
void CancelPositionMove() {  // 定义函数或成员 的入口 CancelPositionMove（Cancel位置Move），参数由此处的签名确定。
    positionExecution.active=false;positionExecution.success=false;  // 将 false 写入或初始化 positionExecution.active。然后将 false 写入或初始化 positionExecution.success。
    MotorBusGuard transaction;  // 声明 transaction，类型和可用范围由所在声明决定。
    for(uint8_t id=1;id<=4;++id){Emm_V5_Stop_Now(id,true);vTaskDelay(pdMS_TO_TICKS(MOTOR_COMMAND_GAP_MS));}  // 按 uint8_t id=1;id<=4;++id 组织循环的初值、继续条件和步进。
    Emm_V5_Synchronous_motion(0);  // 调用 Emm_V5_Synchronous_motion，传入 0。
}  // 结束此函数或控制块。
/**
 * @brief 检查相对位置动作的估算时间并推进结束处理，不代表实测到位。
 */
void TickPositionMove() {  // 定义函数或成员 的入口 TickPositionMove（轮询位置Move），参数由此处的签名确定。
    if(!positionExecution.active)return;  // 当 !positionExecution.active 成立时进入此分支，否则继续后续流程。
    if(taskMotionAborted){CancelPositionMove();return;}  // 当 taskMotionAborted 成立时进入此分支，否则继续后续流程。
    if(millis()-positionExecution.started<positionExecution.duration)return;  // 当 millis()-positionExecution.started<positionExecution.d… 成立时进入此分支，否则继续后续流程。
    currentPose=positionExecution.target;positionExecution.active=false;  // 将 positionExecution.target 写入或初始化 currentPose（当前Pose）。然后将 false 写入或初始化 positionExecution.active。
    Serial.println("[MovePosition] Commands sent and estimated wait elapsed; arrival not verified");  // 调用 Serial.println，传入 "[MovePosition] Commands sent and estimated wait elaps…。
}  // 结束此函数或控制块。
bool MovePosition(float x,float y,float theta,float speed,bool (*shouldStop)(),uint8_t acceleration) {  // 定义函数或成员 的入口 MovePosition（Move位置），参数由此处的签名确定。
    if(shouldStop&&shouldStop())return false;  // 若 shouldStop&&shouldStop()，立即返回 false。
    if(!BeginPositionMove(x,y,theta,speed,acceleration))return false;  // 若 !BeginPositionMove(x,y,theta,speed,acceleration)，立即返回 false。
    while(PositionMoveActive()) {  // 当 PositionMoveActive() 成立时继续循环。
        if(shouldStop&&shouldStop()){CancelPositionMove();return false;}  // 当 shouldStop&&shouldStop() 成立时进入此分支，否则继续后续流程。
        TickPositionMove();if(PositionMoveActive())vTaskDelay(pdMS_TO_TICKS(10));  // 调用 TickPositionMove（轮询位置Move），使用无参形式推进相应操作。然后当 PositionMoveActive() 成立时进入此分支，否则继续后续流程。
    }  // 结束此循环体。
    return positionExecution.success;  // 结束当前函数并返回 positionExecution.success。
}  // 结束此函数或控制块。

/**
 * @brief 同步执行扫码往返直行，用编码器记录完整或提前停车后的行程；distance单位mm，speed单位RPM。
 */
bool MoveScanPosition(float distance,float speed,bool (*shouldStop)()) {  // 定义函数或成员 的入口 MoveScanPosition（Move扫码位置），参数由此处的签名确定。
    int64_t before[4],after[4];  // 声明 before，类型和可用范围由所在声明决定。
    if(!ReadChassisEncoderPositions(before))return false;  // 若 !ReadChassisEncoderPositions(before)，立即返回 false。
    const RobotPose start=currentPose;  // 将 currentPose 写入或初始化 start（起点）。
    const bool completed=MovePosition(0,distance,0,speed,shouldStop,SCAN_ACCELERATION);  // 将 MovePosition(0,distance,0,speed,shouldStop,SCAN_ACCELE… 写入或初始化 completed。
    if(!ReadChassisEncoderPositions(after))return false;  // 若 !ReadChassisEncoderPositions(after)，立即返回 false。
    // 四轮反馈为65536单位/圈；取行程绝对值兼容各驱动器的编码器方向。
    double travel=0;  // 将 0 写入或初始化 travel。
    for(size_t i=0;i<4;++i)travel+=fabs(static_cast<double>(after[i]-before[i]))/4;  // 按 size_t i=0;i<4;++i)travel+=fabs(static_cast<double>(af… 组织循环的初值、继续条件和步进。
    const float forward=copysignf(travel*3200.0/65536.0/Y_PULSE,distance);  // 将 copysignf(travel*3200.0/65536.0/Y_PULSE,distance) 写入或初始化 forward（前向）。
    const auto delta=continuous::bodyToWorld({0,forward},start.theta);  // 将 continuous::bodyToWorld({0,forward},start.theta) 写入或初始化 delta。
    currentPose={start.x+delta.x,start.y+delta.y,start.theta};  // 将 {start.x+delta.x,start.y+delta.y,start.theta} 写入或初始化 currentPose（当前Pose）。
    Serial.printf("[Scan] Travel=%.1f mm, pose=(%.1f, %.1f, %.1f)\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                  forward,currentPose.x,currentPose.y,currentPose.theta);  // 续写当前声明或运算，提供 forward,currentPose.x,currentPose.y,currentPose.theta);。
    return completed;  // 结束当前函数并返回 completed。
}  // 结束此函数或控制块。

/**
 * @brief 同步执行接近后降速扫码，距离单位mm，速度单位RPM；收到任务码回调为true时停车。
 */
bool MoveScanApproach(float approach,float search,uint16_t approachRpm,uint16_t searchRpm, bool (*received)()) {  // 定义函数或成员 的入口 MoveScanApproach（Move扫码Approach），参数由此处的签名确定。
    const RobotPose start=currentPose;  // 将 currentPose 写入或初始化 start（起点）。
    float forward=0;  // 将 0 写入或初始化 forward（前向）。
    const ScanMotionSettings settings={approach,search,Y_PULSE,approachRpm,searchRpm};  // 将 {approach,search,Y_PULSE,approachRpm,searchRpm} 写入或初始化 settings（设置）。
    const char *error=RunContinuousScan(settings,received,forward);  // 将 RunContinuousScan(settings,received,forward) 写入或初始化 error（错误）。
    const auto delta=continuous::bodyToWorld({0,forward},start.theta);  // 将 continuous::bodyToWorld({0,forward},start.theta) 写入或初始化 delta。
    currentPose={start.x+delta.x,start.y+delta.y,start.theta};  // 将 {start.x+delta.x,start.y+delta.y,start.theta} 写入或初始化 currentPose（当前Pose）。
    Serial.printf("[Scan] Continuous travel=%.1f mm, pose=(%.1f, %.1f, %.1f)\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                  forward,currentPose.x,currentPose.y,currentPose.theta);  // 续写当前声明或运算，提供 forward,currentPose.x,currentPose.y,currentPose.theta);。
    if(error)Serial.printf("[Scan] %s\n",error);  // 当 error 成立时进入此分支，否则继续后续流程。
    return error==nullptr;  // 结束当前函数并返回 error==nullptr。
}  // 结束此函数或控制块。

void ResetDiscAlignmentPid() {  // 定义函数或成员 的入口 ResetDiscAlignmentPid（Reset圆盘AlignmentPID），参数由此处的签名确定。
    resetPid(s_alignVisualXPid);  // 调用 resetPid，传入 s_alignVisualXPid。
    resetPid(s_alignVisualYPid);  // 调用 resetPid，传入 s_alignVisualYPid。
    resetPid(s_alignAnglePid);  // 调用 resetPid，传入 s_alignAnglePid。
    resetPid(s_cornerVisualXPid);  // 调用 resetPid，传入 s_cornerVisualXPid。
    resetPid(s_cornerVisualYPid);  // 调用 resetPid，传入 s_cornerVisualYPid。
    resetPid(s_cornerAnglePid);  // 调用 resetPid，传入 s_cornerAnglePid。
}  // 结束此函数或控制块。

void ResetAlignmentWait() {  // 定义函数或成员 的入口 ResetAlignmentWait，参数由此处的签名确定。
    s_alignmentWaiter.reset();  // 调用 s_alignmentWaiter.reset。
}  // 结束此函数或控制块。

bool WaitForAlignment(VisionStartMode mode, float angleErrorDeg, float visualXError, float visualYError, float dtSeconds, uint16_t speedRpm) {  // 定义函数或成员 的入口 WaitForAlignment，参数由此处的签名确定。
    const bool withinDeadzone = AlignToVisionContinuous(  // 将 AlignToVisionContinuous( 写入或初始化 withinDeadzone。
        mode, angleErrorDeg, visualXError, visualYError, dtSeconds, speedRpm);  // 声明 dtSeconds（时间差Seconds），类型和可用范围由所在声明决定。
    return s_alignmentWaiter.update(mode, withinDeadzone);  // 结束当前函数并返回 s_alignmentWaiter.update(mode, withinDeadzone)。
}  // 结束此函数或控制块。

// 视觉对齐循环入口。
// 输入角度与平移误差来自当前帧，dtSeconds 供 PID 计算，speedRpm 用于底盘命令。
bool AlignToVisionContinuous(VisionStartMode mode, float angleErrorDeg, float visualXError, float visualYError, float dtSeconds, uint16_t speedRpm) {  // 定义函数或成员 的入口 AlignToVisionContinuous（AlignTo视觉Continuous），参数由此处的签名确定。
    dtSeconds = clampFloat(  // 将 clampFloat( 写入或初始化 dtSeconds（时间差Seconds）。
        dtSeconds, ALIGN_DT_MIN_SECONDS, ALIGN_DT_MAX_SECONDS  // 声明 ALIGN_DT_MAX_SECONDS（ALIGN时间差最大SECONDS），类型和可用范围由所在声明决定。
    );  // 续写当前声明或运算，提供 );。

    const auto &deadzone = ALIGN_DEADZONES[static_cast<uint8_t>(mode)];  // 将 ALIGN_DEADZONES[static_cast<uint8_t>(mode)] 写入或初始化 deadzone。
    const bool angleAligned = fabsf(angleErrorDeg) <= deadzone.angleDeg;  // 将 fabsf(angleErrorDeg) <= deadzone.angleDeg 写入或初始化 angleAligned（角度Aligned）。
    const bool xAligned = fabsf(visualXError) <= deadzone.visualX;  // 将 fabsf(visualXError) <= deadzone.visualX 写入或初始化 xAligned（横向Aligned）。
    const bool yAligned = fabsf(visualYError) <= deadzone.visualY;  // 将 fabsf(visualYError) <= deadzone.visualY 写入或初始化 yAligned（纵向Aligned）。

    //bool angleAligned = 1;// 测试用，直接对齐角度
    //bool xAligned = 1;// 测试用，直接对齐视觉 X
    //bool yAligned = 1;// 测试用，直接对齐视觉 Y

    if (IsWithinAlignmentDeadzone(deadzone, angleErrorDeg, visualXError, visualYError)) {  // 当 IsWithinAlignmentDeadzone(deadzone, angleErrorDeg, vis… 成立时进入此分支，否则继续后续流程。
        OmniMove(0.0f, 0.0f, 0.0f, 0);  // 调用 OmniMove，传入 0.0f, 0.0f, 0.0f, 0。
        ResetDiscAlignmentPid();  // 调用 ResetDiscAlignmentPid（Reset圆盘AlignmentPID），使用无参形式推进相应操作。
        return true;  // 结束当前函数并返回 true。
    }  // 结束此条件分支。

    float visualXOutput = 0.0f;  // 将 0.0f 写入或初始化 visualXOutput（visual横向输出）。
    float visualYOutput = 0.0f;  // 将 0.0f 写入或初始化 visualYOutput（visual纵向输出）。
    float angleOutput = 0.0f;  // 将 0.0f 写入或初始化 angleOutput（角度输出）。
    const bool corner = mode == VisionStartMode::CORNER;  // 将 mode == VisionStartMode::CORNER 写入或初始化 corner。
    auto &visualXPid = corner ? s_cornerVisualXPid : s_alignVisualXPid;  // 将 corner ? s_cornerVisualXPid : s_alignVisualXPid 写入或初始化 visualXPid（visual横向PID）。
    auto &visualYPid = corner ? s_cornerVisualYPid : s_alignVisualYPid;  // 将 corner ? s_cornerVisualYPid : s_alignVisualYPid 写入或初始化 visualYPid（visual纵向PID）。
    auto &anglePid = corner ? s_cornerAnglePid : s_alignAnglePid;  // 将 corner ? s_cornerAnglePid : s_alignAnglePid 写入或初始化 anglePid（角度PID）。

    if (xAligned) {  // 当 xAligned 成立时进入此分支，否则继续后续流程。
        resetPid(visualXPid);  // 调用 resetPid，传入 visualXPid。
    } else {  // 前一个判断不成立时执行此分支。
        visualXOutput = updatePid( visualXPid, visualXError, dtSeconds  // 将 updatePid( visualXPid, visualXError, dtSeconds 写入或初始化 visualXOutput（visual横向输出）。
        );  // 续写当前声明或运算，提供 );。
    }  // 结束此条件分支。
    if (yAligned) {  // 当 yAligned 成立时进入此分支，否则继续后续流程。
        resetPid(visualYPid);  // 调用 resetPid，传入 visualYPid。
    } else {  // 前一个判断不成立时执行此分支。
        visualYOutput = updatePid( visualYPid, visualYError, dtSeconds  // 将 updatePid( visualYPid, visualYError, dtSeconds 写入或初始化 visualYOutput（visual纵向输出）。
        );  // 续写当前声明或运算，提供 );。
    }  // 结束此条件分支。
    if (angleAligned) {  // 当 angleAligned 成立时进入此分支，否则继续后续流程。
        resetPid(anglePid);  // 调用 resetPid，传入 anglePid。
    } else {  // 前一个判断不成立时执行此分支。
        angleOutput = updatePid( anglePid, angleErrorDeg, dtSeconds  // 将 updatePid( anglePid, angleErrorDeg, dtSeconds 写入或初始化 angleOutput（角度输出）。
        );  // 续写当前声明或运算，提供 );。
    }  // 结束此条件分支。

    // 摄像头相对车身约旋转 90°：视觉 Y -> 底盘 X，视觉 X -> 底盘 Y。
    // 单轴实测表明两个映射均需取反；角度正方向也与底盘旋转方向相反。
    const float chassisXVelocity = -visualYOutput;  // 将 -visualYOutput 写入或初始化 chassisXVelocity（chassis横向速度）。
    const float chassisYVelocity = -visualXOutput;  // 将 -visualXOutput 写入或初始化 chassisYVelocity（chassis纵向速度）。
    const float chassisRotationVelocity = -angleOutput;  // 将 -angleOutput 写入或初始化 chassisRotationVelocity（chassisRotation速度）。

    OmniMove(  // 调用 OmniMove，传入 。
        chassisXVelocity, chassisYVelocity,  // 声明 chassisYVelocity（chassis纵向速度），类型和可用范围由所在声明决定。
        chassisRotationVelocity, speedRpm  // 声明 speedRpm（速度转速），类型和可用范围由所在声明决定。
    );  // 续写当前声明或运算，提供 );。
    return false;  // 结束当前函数并返回 false。
}  // 结束此函数或控制块。

/**
 * @brief 按节点序号路径移动，只原地转向和沿车身 Y 轴前后直行。
 */
bool MoveNodePath(const uint8_t *path, size_t pathLength, float speedMmS, uint8_t acceleration, float finalHeading) {  // 定义函数或成员 的入口 MoveNodePath（Move节点路径），参数由此处的签名确定。
    // 正式节点导航使用固定位置目标和驱动器反馈，绝不走试跑的速度积分推进。
    // 不生成外绕/移动转向；同向共线节点合并，路口编码器到位并停稳后原地转。
    // acceleration是旧位置模式档位，新控制器使用独立mm/s²加速度限幅。
    (void)acceleration;  // 为当前列表或跨行表达式提供这一项 (void)acceleration;。
    return H30RunNodePath(path,pathLength,speedMmS,finalHeading);  // 结束当前函数并返回 H30RunNodePath(path,pathLength,speedMmS,finalHeading)。
}  // 结束此函数或控制块。
