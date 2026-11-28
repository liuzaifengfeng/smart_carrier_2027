// 定义物料位姿、动作参数和逐件或整轮搬运入口。
// 调试物料顺序时分别追踪圆盘位、颜色、货位和工作区位置，四者在任务中承担不同角色。

#include "material_transfer.h"  // 引入 material_transfer.h 中的声明和定义。
#include "arm_placement_log.h"  // 引入 arm_placement_log.h 中的声明和定义。
#include "turret_path.h"  // 引入 turret_path.h 中的声明和定义。
#include "servo.h"  // 引入 servo.h 中的声明和定义。
#include "disc_material_decision.h"  // 引入 disc_material_decision.h 中的声明和定义。
#include "node_position_hardware.h"  // 引入 node_position_hardware.h 中的声明和定义。
#include "ring_correction.h"  // 引入 ring_correction.h 中的声明和定义。

#include <math.h>  // 引入 math.h 中的声明和定义。

// ================= 实车标定区（主要修改这里） =================
// 每一项的填写顺序都是：{升降高度, 伸出长度, 转台角度}。
// NAN 表示“尚未标定”。请用实测值替换下列位置中的 NAN。
// 在全部位置填写完成前，MaterialTransferPosesReady() 返回 false，机械臂不会动作。
MaterialTransferLayout materialTransferLayout = {  // 将 { 写入或初始化 materialTransferLayout（物料TransferLayout）。
    {150, 50, 90}, // 圆盘取料位置
    {125, 15, -3.5f}, // 旋转料盘待操作区
    {  // 为当前列表或跨行表达式提供这一项 {。
        {2.5, 61, 48}, // 粗加工区/暂存区的 1 号位置
        {2.5, 5, 86}, // 粗加工区/暂存区的 2 号位置
        {2.5, 39, 125.3}, // 粗加工区/暂存区的 3 号位置
    },  // 结束此初始化列表或局部作用域。
    155.0f, // approachHeight，按物料实际高度修改
    60.0f,  // secondLayerOffset，按物料实际高度修改
    250.0f, // moveSpeed
    50.0f,   // clawOpenAngle
    30.0f,   // cargoClawOpenAngle
    -5.0f,  // clawClosedAngle
    1,      // motionWaitMs，不传等待参数时的默认毫秒数
};  // 结束此初始化列表或局部作用域。

float cargoTrayAngles[MATERIAL_STATION_COUNT] = {150, 213, 276};  // 将 {150, 213, 276} 写入或初始化 cargoTrayAngles[MATERIAL_STATION_COUNT]。

MaterialMotionSteps materialMotionSteps = {  // 将 { 写入或初始化 materialMotionSteps（物料MotionSteps）。
    {80.0f, 0.0f, 90.0f}, // 圆盘预抓：高度、伸出、转台
    90.0f,  // 圆盘预抓张爪角度
    150.0f, // 圆盘抓后提起高度
    250.0f, 250.0f, 250.0f, // 圆盘预抓、夹紧、提起
    250.0f, 250.0f,         // 移至工位上方：平移、转向
    250.0f, 250.0f, 250.0f, // 取料：下降、夹紧、提起
    250.0f, 250.0f, 250.0f, // 放料：下降、松爪、提起
};  // 结束此初始化列表或局部作用域。

// ACTION_GROUP_DELAYS_BEGIN
// 上位机只改写本区域的延时数值；单位 ms，0 表示不等待。
// 每行顺序：取料平移/稳定/转向，下降/夹紧/抬升，放料平移/稳定/转向，下降/松爪/抬升。
// 圆盘抓后：夹紧等待、抬升缩回等待，再使用放料六项控制转向、伸出下降及松爪抬升。
// 延时顺序(ms)：取料平移、稳定、转向、下降、夹紧、抬升(圆盘同时缩回0)；放料旋转前等待、稳定、旋转、伸出下降、松爪、抬升。
MaterialTransferDelays commonMaterialDelays =  // 续写当前声明或运算，提供 MaterialTransferDelays commonMaterialDelays =。
    {{100, 0, 900}, 700, 500, 300, {0, 0, 800}, 300, 100, 200};  // 为当前列表或跨行表达式提供这一项 {{100, 0, 900}, 700, 500, 300, {0, 0, 800}, 300, 100, …。
MaterialTransferDelays cargoToWorkAreaDelays[MATERIAL_STATION_COUNT][MATERIAL_STATION_COUNT] = {  // 声明 cargoToWorkAreaDelays（货位To工位AreaDelays），类型和可用范围由所在声明决定。
    {  // 为当前列表或跨行表达式提供这一项 {。
        {{100, 0, 1000}, 450, 500, 100, {100, 0, 1200}, 800, 200, 300}, // 1 -> 1
        {{100, 0, 600}, 600, 500, 400, {200, 0, 600}, 1100, 300, 1500}, // 1 -> 2
        {{100, 0, 600}, 600, 500, 400, {300, 0, 200}, 1200, 300, 1500}, // 1 -> 3
    },  // 结束此初始化列表或局部作用域。
    {  // 为当前列表或跨行表达式提供这一项 {。
        {{100, 0, 600}, 600, 500, 400, {300, 0, 800}, 1100, 300, 1500}, // 2 -> 1
        {{100, 0, 1000}, 450, 500, 100, {100, 0, 1200}, 800, 200, 300}, // 2 -> 2
        {{100, 0, 600}, 600, 500, 400, {300, 0, 800}, 800, 300, 1500}, // 2 -> 3
    },  // 结束此初始化列表或局部作用域。
    {  // 为当前列表或跨行表达式提供这一项 {。
        {{100, 0, 600}, 600, 500, 400, {300, 0, 600}, 800, 300, 1500}, // 3 -> 1
        {{100, 0, 600}, 600, 500, 400, {300, 0, 700}, 800, 300, 1500}, // 3 -> 2
        {{100, 0, 1000}, 450, 500, 100, {100, 0, 1200}, 800, 200, 300}, // 3 -> 3
    },  // 结束此初始化列表或局部作用域。
};  // 结束此初始化列表或局部作用域。
MaterialAuxiliaryDelays materialAuxiliaryDelays = {300, 340, 300, 1360, 680, 1360, 680};  // 将 {300, 340, 300, 1360, 680, 1360, 680} 写入或初始化 materialAuxiliaryDelays（物料AuxiliaryDelays）。
// ACTION_GROUP_DELAYS_END

// REVERSE_ACTION_GROUP_DELAYS_BEGIN
// 工作区 -> 载物台，按 [工作区编号-1][载物台编号-1] 选择；单位 ms。
// 每行与正向相同，共12项；取料抬升表示同时抬升、缩回到0之后的等待。
// 九组初值复制当前公共默认值，之后各自独立调整、保存。
MaterialTransferDelays workAreaToCargoDelays[MATERIAL_STATION_COUNT][MATERIAL_STATION_COUNT] = {  // 声明 workAreaToCargoDelays（工位AreaTo货位Delays），类型和可用范围由所在声明决定。
    {  // 为当前列表或跨行表达式提供这一项 {。
        {{100, 0, 1000}, 600, 400, 200, {200, 0, 1150}, 400, 100, 200}, // 工作区1 -> 载物台1
        {{100, 0, 600}, 600, 400, 200, {200, 0, 800}, 400, 100, 200}, // 工作区1 -> 载物台2
        {{100, 0, 600}, 600, 400, 200, {200, 0, 800}, 400, 100, 200}, // 工作区1 -> 载物台3
    },  // 结束此初始化列表或局部作用域。
    {  // 为当前列表或跨行表达式提供这一项 {。
        {{100, 0, 600}, 600, 400, 200, {200, 0, 800}, 400, 100, 200}, // 工作区2 -> 载物台1
        {{100, 0, 1000}, 600, 400, 200, {200, 0, 1150}, 400, 100, 200}, // 工作区2 -> 载物台2
        {{100, 0, 600}, 600, 400, 200, {200, 0, 800}, 400, 100, 200}, // 工作区2 -> 载物台3
    },  // 结束此初始化列表或局部作用域。
    {  // 为当前列表或跨行表达式提供这一项 {。
        {{100, 0, 600}, 600, 400, 200, {200, 0, 800}, 400, 100, 200}, // 工作区3 -> 载物台1
        {{100, 0, 600}, 600, 400, 200, {200, 0, 800}, 400, 100, 200}, // 工作区3 -> 载物台2
        {{100, 0, 1000}, 600, 400, 200, {200, 0, 1150}, 400, 100, 200}, // 工作区3 -> 载物台3
    },  // 结束此初始化列表或局部作用域。
};  // 结束此初始化列表或局部作用域。
// REVERSE_ACTION_GROUP_DELAYS_END

// 开机时三个载物台均为空载。
CargoPlatform cargoPlatforms[MATERIAL_STATION_COUNT] = {  // 将 { 写入或初始化 cargoPlatforms[MATERIAL_STATION_COUNT]。
    {MATERIAL_NONE}, {MATERIAL_NONE}, {MATERIAL_NONE}  // 为当前列表或跨行表达式提供这一项 {MATERIAL_NONE}, {MATERIAL_NONE}, {MATERIAL_NONE}。
};  // 结束此初始化列表或局部作用域。

namespace {  // 开始匿名命名空间，本文件内部共享其中的定义。

bool isMaterialCodeValid(int code) {  // 定义函数或成员 的入口 isMaterialCodeValid（is物料Code有效），参数由此处的签名确定。
    // 任务码中的合法颜色编号为 1~6。
    return code >= MATERIAL_RED && code <= MATERIAL_LIGHT_BLUE;  // 结束当前函数并返回 code >= MATERIAL_RED && code <= MATERIAL_LIGHT_BLUE。
}  // 结束此函数或控制块。

bool isStationCodeValid(int code) {  // 定义函数或成员 的入口 isStationCodeValid（isStationCode有效），参数由此处的签名确定。
    // 载物台编号和区域位置编号都只能是 1、2、3。
    return code >= 1 && code <= MATERIAL_STATION_COUNT;  // 结束当前函数并返回 code >= 1 && code <= MATERIAL_STATION_COUNT。
}  // 结束此函数或控制块。

bool isPoseValid(const MaterialStationPose &pose) {  // 定义函数或成员 的入口 isPoseValid（isPose有效），参数由此处的签名确定。
    // 检查标定值是否已经填写，并确保没有超过 MoveArm 的机械行程。
    return isfinite(pose.high) && pose.high >= 0.0f && pose.high <= ARM_HEIGHT_LIMIT_MM  // 结束当前函数并返回 isfinite(pose.high) && pose.high >= 0.0f && pose.high …。
        && isfinite(pose.length) && pose.length >= 0.0f && pose.length <= 170.0f  // 继续上一行的组合条件，本行提供 && isfinite(pose.length) && pose.length >= 0.0f && pose.length <= 170.0f。
        && isfinite(pose.turretAngle)  // 继续上一行的组合条件，本行提供 && isfinite(pose.turretAngle)。
        && pose.turretAngle >= TURRET_CABLE_MIN_DEG  // 继续上一行的组合条件，本行提供 && pose.turretAngle >= TURRET_CABLE_MIN_DEG。
        && pose.turretAngle <= TURRET_CABLE_MAX_DEG;  // 继续上一行的组合条件，本行提供 && pose.turretAngle <= TURRET_CABLE_MAX_DEG;。
}  // 结束此函数或控制块。

bool isLayoutParameterValid() {  // 定义函数或成员 的入口 isLayoutParameterValid（isLayoutParameter有效），参数由此处的签名确定。
    return isfinite(materialTransferLayout.approachHeight)  // 结束当前函数并返回 isfinite(materialTransferLayout.approachHeight)。
        && materialTransferLayout.approachHeight >= 0.0f  // 继续上一行的组合条件，本行提供 && materialTransferLayout.approachHeight >= 0.0f。
        && materialTransferLayout.approachHeight <= ARM_HEIGHT_LIMIT_MM  // 继续上一行的组合条件，本行提供 && materialTransferLayout.approachHeight <= ARM_HEIGHT_LIMIT_MM。
        && isfinite(materialTransferLayout.secondLayerOffset)  // 继续上一行的组合条件，本行提供 && isfinite(materialTransferLayout.secondLayerOffset)。
        && materialTransferLayout.secondLayerOffset >= 0.0f  // 继续上一行的组合条件，本行提供 && materialTransferLayout.secondLayerOffset >= 0.0f。
        && isfinite(materialTransferLayout.moveSpeed)  // 继续上一行的组合条件，本行提供 && isfinite(materialTransferLayout.moveSpeed)。
        && materialTransferLayout.moveSpeed > 0.0f  // 继续上一行的组合条件，本行提供 && materialTransferLayout.moveSpeed > 0.0f。
        && isfinite(materialTransferLayout.clawOpenAngle)  // 继续上一行的组合条件，本行提供 && isfinite(materialTransferLayout.clawOpenAngle)。
        && isfinite(materialTransferLayout.clawClosedAngle)  // 继续上一行的组合条件，本行提供 && isfinite(materialTransferLayout.clawClosedAngle)。
        && materialTransferLayout.motionWaitMs > 0;  // 继续上一行的组合条件，本行提供 && materialTransferLayout.motionWaitMs > 0;。
}  // 结束此函数或控制块。

void waitForArm(float durationMs = materialTransferLayout.motionWaitMs) {  // 定义函数或成员 的入口 waitForArm（waitFor机械臂），参数由此处的签名确定。
    // 参数直接表示毫秒：waitForArm(1000) 等待1秒，不乘基础时间。
    // 不传参数或参数非法时，使用配置中的默认等待时间。
    if (!isfinite(durationMs) || durationMs < 0.0f) {  // 当 !isfinite(durationMs) || durationMs < 0.0f 成立时进入此分支，否则继续后续流程。
        Serial.println("[Material] ERR: invalid arm wait duration");  // 调用 Serial.println，传入 "[Material] ERR: invalid arm wait duration"。
        durationMs = materialTransferLayout.motionWaitMs;  // 将 materialTransferLayout.motionWaitMs 写入或初始化 durationMs（持续时间毫秒）。
    }  // 结束此条件分支。
    if (durationMs == 0.0f) return;  // 当 durationMs == 0.0f 成立时进入此分支，否则继续后续流程。
    const uint32_t waitMs = static_cast<uint32_t>(  // 将 static_cast<uint32_t>( 写入或初始化 waitMs（wait毫秒）。
        lroundf(durationMs)  // 调用 lroundf，传入 durationMs。
    );  // 续写当前声明或运算，提供 );。
    vTaskDelay(pdMS_TO_TICKS(waitMs));  // 让当前 FreeRTOS 任务延时，等待期间调度其他任务。
}  // 结束此初始化列表或局部作用域。

// 同步兼容入口也用实测多圈角度结束转向，不追加固定延时。
bool waitForTurret(float targetLength = -1) {  // 定义函数或成员 的入口 waitForTurret（waitFor转台），参数由此处的签名确定。
    const float target=currentArm.turret_angle;  // 将 currentArm.turret_angle 写入或初始化 target（目标）。
    for(;;) {  // 按 ;; 组织循环的初值、继续条件和步进。
        if(taskMotionAborted)return false;  // 若 taskMotionAborted，立即返回 false。
        float measured=0;if(!ReadArmTurretAngle(measured))return false;  // 将 0 写入或初始化 measured。然后若 !ReadArmTurretAngle(measured)，立即返回 false。
        if(fabsf(measured-target)<=5.0f) {  // 当 fabsf(measured-target)<=5.0f 成立时进入此分支，否则继续后续流程。
            if(targetLength==-1)return true;  // 若 targetLength==-1，立即返回 true。
            node_position::Motor state;  // 声明 state（状态），类型和可用范围由所在声明决定。
            if(ReadMotorFeedback(6,state))return false;  // 若 ReadMotorFeedback(6,state)，立即返回 false。
            const double length=fabs(static_cast<double>(state.position))*3200/65536/LENGTH_PULSE;  // 将 fabs(static_cast<double>(state.position))*3200/65536/L… 写入或初始化 length（长度）。
            if((state.flags&2)&&fabs(length-targetLength)<=1.0)return true;  // 若 (state.flags&2)&&fabs(length-targetLength)<=1.0，立即返回 true。
        }  // 结束此条件分支。
        waitForArm(50);  // 调用 waitForArm（waitFor机械臂），传入 50。
    }  // 结束此循环体。
}  // 结束此初始化列表或局部作用域。

void startCargoTray(uint8_t cargoCode) {  // 定义函数或成员 的入口 startCargoTray（起点货位料盘），参数由此处的签名确定。
    Servo_MoveCargoTray(cargoTrayAngles[cargoCode - 1]);  // 调用 Servo_MoveCargoTray（ServoMove货位料盘），传入 cargoTrayAngles[cargoCode - 1]。
}  // 结束此函数或控制块。
bool waitForTrayAngle(float target);  // 声明接口 waitForTrayAngle（waitFor料盘角度），参数由此处的签名确定。

// 移动到目标上方
bool moveAbove(const MaterialStationPose &pose, float clawAngle,  // 定义函数或成员 moveAbove，参数由此处的签名确定。
               const MaterialApproachDelays &delays, const float *plannedAngle = nullptr,  // 将 nullptr, 写入或初始化 plannedAngle（planned角度）。
               uint8_t turretAcceleration = TURRET_ACCELERATION_FAST) {  // 将 TURRET_ACCELERATION_FAST) { 写入或初始化 turretAcceleration（转台加速度）。
    // 升降、伸缩同时发令；等待后转向。两次等待分别保留，供上位机逐项调整。
    if (!MoveArm(materialTransferLayout.approachHeight, pose.length,  // 当 !MoveArm(materialTransferLayout.approachHeight, pose.l… 成立时进入此分支，否则继续后续流程。
                 -1, clawAngle, materialMotionSteps.approachTranslateSpeed)) return false;  // 续写当前声明或运算，提供 -1, clawAngle, materialMotionSteps.approachTranslateSp…。
    waitForArm(delays.translateMs);  // 调用 waitForArm（waitFor机械臂），传入 delays.translateMs。
    waitForArm(delays.settleMs);  // 调用 waitForArm（waitFor机械臂），传入 delays.settleMs。
    if (!(plannedAngle  // 当 !(plannedAngle 成立时进入此分支，否则继续后续流程。
            ? MoveArmPlanned(materialTransferLayout.approachHeight, pose.length,  // 定义函数或成员 MoveArmPlanned（Move机械臂Planned），参数由此处的签名确定。
                             *plannedAngle, -1, materialMotionSteps.approachRotateSpeed, turretAcceleration)  // 续写当前声明或运算，提供 *plannedAngle, -1, materialMotionSteps.approachRotateS…。
            : MoveArm(materialTransferLayout.approachHeight, pose.length,  // 定义函数或成员 MoveArm（Move机械臂），参数由此处的签名确定。
                      pose.turretAngle, -1, materialMotionSteps.approachRotateSpeed, turretAcceleration))) return false;  // 续写当前声明或运算，提供 pose.turretAngle, -1, materialMotionSteps.approachRota…。
    if(!waitForTurret())return false;  // 若 !waitForTurret()，立即返回 false。
    return true;  // 结束当前函数并返回 true。
}  // 结束此初始化列表或局部作用域。

// 抓取物料
bool pickAt(const MaterialStationPose &pose,  // 定义函数或成员 pickAt，参数由此处的签名确定。
            const MaterialTransferDelays &delays = commonMaterialDelays,  // 将 commonMaterialDelays, 写入或初始化 delays。
            bool retractAfterGrip = false, bool retractWithLift = false,  // 续列跨行签名或声明中的参数，类型与名称为 bool retractAfterGrip = false, bool retractWithLift = …。
            const float *plannedAngle = nullptr, uint8_t turretAcceleration = TURRET_ACCELERATION_FAST, uint8_t cargoCode = 0) {  // 将 nullptr, uint8_t turretAcceleration = TURRET_ACCELERAT… 写入或初始化 plannedAngle（planned角度）。
    // 载物台出料先抬升、等待后再缩回；工作区取回可同时抬升并缩回到 0。
    const float openAngle = cargoCode ? materialTransferLayout.cargoClawOpenAngle : materialTransferLayout.clawOpenAngle;  // 将 cargoCode ? materialTransferLayout.cargoClawOpenAngle … 写入或初始化 openAngle（open角度）。
    if (!moveAbove(pose, openAngle, delays.pickAbove, plannedAngle, turretAcceleration)) return false;  // 若 !moveAbove(pose, openAngle, delays.pickAbove, plannedA…，立即返回 false。
    if (cargoCode && !waitForTrayAngle(cargoTrayAngles[cargoCode - 1])) return false;  // 若 cargoCode && !waitForTrayAngle(cargoTrayAngles[cargoCo…，立即返回 false。
    if (!MoveArm(pose.high, -1, -1, -1,  // 当 !MoveArm(pose.high, -1, -1, -1, 成立时进入此分支，否则继续后续流程。
                 materialMotionSteps.pickDescendSpeed)) return false;  // 续写当前声明或运算，提供 materialMotionSteps.pickDescendSpeed)) return false;。
    waitForArm(delays.pickDescendMs);  // 调用 waitForArm（waitFor机械臂），传入 delays.pickDescendMs。
    if (!MoveArm(-1, -1, -1, materialTransferLayout.clawClosedAngle,  // 当 !MoveArm(-1, -1, -1, materialTransferLayout.clawClosed… 成立时进入此分支，否则继续后续流程。
                 materialMotionSteps.pickGripSpeed)) return false;  // 续写当前声明或运算，提供 materialMotionSteps.pickGripSpeed)) return false;。
    waitForArm(delays.pickGripMs);  // 调用 waitForArm（waitFor机械臂），传入 delays.pickGripMs。
    if (!MoveArm(materialTransferLayout.approachHeight, retractWithLift ? 0.0f : -1.0f, -1, -1,  // 当 !MoveArm(materialTransferLayout.approachHeight, retrac… 成立时进入此分支，否则继续后续流程。
                 materialMotionSteps.pickLiftSpeed)) return false;  // 续写当前声明或运算，提供 materialMotionSteps.pickLiftSpeed)) return false;。
    waitForArm(delays.pickLiftMs);  // 调用 waitForArm（waitFor机械臂），传入 delays.pickLiftMs。
    if (retractAfterGrip && !retractWithLift  // 当 retractAfterGrip && !retractWithLift 成立时进入此分支，否则继续后续流程。
            && !MoveArm(-1, 0.0f, -1, -1, materialMotionSteps.pickLiftSpeed)) return false;  // 继续上一行的组合条件，本行提供 && !MoveArm(-1, 0.0f, -1, -1, materialMotionSteps.pickLiftSpeed)) return false;。
    return true;  // 结束当前函数并返回 true。
}  // 结束此初始化列表或局部作用域。

// 放置物料
//@param pose 目标位置  
//@param targetHeight 目标高度
bool placeAt(const MaterialStationPose &pose, float targetHeight,  // 定义函数或成员 placeAt，参数由此处的签名确定。
             const MaterialTransferDelays &delays = commonMaterialDelays,  // 将 commonMaterialDelays, 写入或初始化 delays。
             bool fromRetracted = false, const float *plannedAngle = nullptr, uint8_t turretAcceleration = TURRET_ACCELERATION_FAST, bool workPlacement = false,  // 续列跨行签名或声明中的参数，类型与名称为 bool fromRetracted = false, const float *plannedAngle …。
             uint8_t sourceCargo = 0, uint8_t destinationCargo = 0) {  // 将 0, uint8_t destinationCargo = 0) { 写入或初始化 sourceCargo（source货位）。
    const uint8_t workPosition = &pose == &materialTransferLayout.workArea[2] ? 3 : 0;  // 将 &pose == &materialTransferLayout.workArea[2] ? 3 : 0 写入或初始化 workPosition（工位位置）。
    if (fromRetracted) {  // 当 fromRetracted 成立时进入此分支，否则继续后续流程。
        if(sourceCargo)waitForArm(CARGO_RETRACT_BEFORE_TURN_MS);  // 当 sourceCargo 成立时进入此分支，否则继续后续流程。
        else {  // 前一个判断不成立时执行此分支。
            waitForArm(delays.placeAbove.translateMs);  // 调用 waitForArm（waitFor机械臂），传入 delays.placeAbove.translateMs。
            waitForArm(delays.placeAbove.settleMs);  // 调用 waitForArm（waitFor机械臂），传入 delays.placeAbove.settleMs。
        }  // 结束此条件分支。
        const float targetLength = destinationCargo ? pose.length : -1;  // 将 destinationCargo ? pose.length : -1 写入或初始化 targetLength（目标长度）。
        const uint8_t rotateAcceleration = destinationCargo ? TURRET_ACCELERATION_CARGO_PLACE : turretAcceleration;  // 将 destinationCargo ? TURRET_ACCELERATION_CARGO_PLACE : t… 写入或初始化 rotateAcceleration（rotate加速度）。
        if (!(plannedAngle  // 当 !(plannedAngle 成立时进入此分支，否则继续后续流程。
                ? MoveArmPlanned(-1, targetLength, *plannedAngle, -1, materialMotionSteps.approachRotateSpeed, rotateAcceleration)  // 定义函数或成员 MoveArmPlanned（Move机械臂Planned），参数由此处的签名确定。
                : MoveArm(-1, targetLength, pose.turretAngle, -1, materialMotionSteps.approachRotateSpeed, rotateAcceleration))) return false;  // 声明接口 MoveArm（Move机械臂），参数由此处的签名确定。
        if (CargoTrayNeedsClearance(sourceCargo)) {  // 当 CargoTrayNeedsClearance(sourceCargo) 成立时进入此分支，否则继续后续流程。
            Servo_SetAngleMTurn(SERVO_ID_CARGO_TRAY, CargoTrayClearanceAngle(sourceCargo));  // 调用 Servo_SetAngleMTurn（ServoSet角度MTurn），传入 SERVO_ID_CARGO_TRAY, CargoTrayClearanceAngle(sourceCar…。
        }  // 结束此条件分支。
        if(!waitForTurret(targetLength))return false;  // 若 !waitForTurret(targetLength)，立即返回 false。
    } else if (!moveAbove(pose, -1,  // 仅在先前条件均不成立时，当 !moveAbove(pose, -1, 成立时进入此分支，否则继续后续流程。
                          delays.placeAbove, plannedAngle, turretAcceleration)) return false;  // 续写当前声明或运算，提供 delays.placeAbove, plannedAngle, turretAcceleration)) …。
    if (destinationCargo && !waitForTrayAngle(cargoTrayAngles[destinationCargo - 1])) return false;  // 若 destinationCargo && !waitForTrayAngle(cargoTrayAngles[…，立即返回 false。
    if (workPosition == 3 && CargoTrayNeedsWork3Limit(sourceCargo)) {  // 当 workPosition == 3 && CargoTrayNeedsWork3Limit(sourceCa… 成立时进入此分支，否则继续后续流程。
        Servo_SetAngleMTurn(SERVO_ID_CARGO_TRAY, WORK_POSITION_3_TRAY_TARGET_DEG);  // 调用 Servo_SetAngleMTurn（ServoSet角度MTurn），传入 SERVO_ID_CARGO_TRAY, WORK_POSITION_3_TRAY_TARGET_DEG。
    }  // 结束此条件分支。
    if(turretAcceleration==TURRET_ACCELERATION_SLOW) {  // 当 turretAcceleration==TURRET_ACCELERATION_SLOW 成立时进入此分支，否则继续后续流程。
        // 第一段沿用原下降延时，从预放位降到0mm只等待100ms，不查到位反馈。
        if(!MoveArm(ring::HOVER_MM,fromRetracted?pose.length:-1,-1,  // 当 !MoveArm(ring::HOVER_MM,fromRetracted?pose.length:-1,-… 成立时进入此分支，否则继续后续流程。
                    -1,materialMotionSteps.placeDescendSpeed,turretAcceleration,true))return false;  // 续写当前声明或运算，提供 -1,materialMotionSteps.placeDescendSpeed,turretAcceler…。
        waitForArm(delays.placeDescendMs);  // 调用 waitForArm（waitFor机械臂），传入 delays.placeDescendMs。
        if(!MoveArm(0,-1,-1,-1,materialMotionSteps.placeDescendSpeed,turretAcceleration,true))return false;  // 若 !MoveArm(0,-1,-1,-1,materialMotionSteps.placeDescendSp…，立即返回 false。
        waitForArm(100);  // 调用 waitForArm（waitFor机械臂），传入 100。
    } else {  // 前一个判断不成立时执行此分支。
        const float releaseHeight=targetHeight+(destinationCargo?CARGO_TRAY_RELEASE_OFFSET_MM:0.0f);  // 将 targetHeight+(destinationCargo?CARGO_TRAY_RELEASE_OFFS… 写入或初始化 releaseHeight（release高度）。
        if(!MoveArm(releaseHeight,fromRetracted&&!destinationCargo?pose.length:-1,-1,  // 当 !MoveArm(releaseHeight,fromRetracted&&!destinationCarg… 成立时进入此分支，否则继续后续流程。
                    -1,materialMotionSteps.placeDescendSpeed,turretAcceleration))return false;  // 续写当前声明或运算，提供 -1,materialMotionSteps.placeDescendSpeed,turretAcceler…。
        waitForArm(delays.placeDescendMs);  // 调用 waitForArm（waitFor机械臂），传入 delays.placeDescendMs。
    }  // 结束此条件分支。
    if(workPlacement)LogArmPlacementReadback(0,static_cast<uint8_t>(&pose-materialTransferLayout.workArea+1),currentArm.turret_angle,currentArm.length);  // 当 workPlacement 成立时进入此分支，否则继续后续流程。
    if(turretAcceleration==TURRET_ACCELERATION_SLOW) { // 仅工位第一层放料，载物台和二层不分段松爪。
        if(!MoveArm(-1,-1,-1,0,materialMotionSteps.placeReleaseSpeed))return false;  // 若 !MoveArm(-1,-1,-1,0,materialMotionSteps.placeReleaseSp…，立即返回 false。
        waitForArm(300);  // 调用 waitForArm（waitFor机械臂），传入 300。
    }  // 结束此条件分支。
    const float openAngle = &pose == &materialTransferLayout.cargo  // 将 &pose == &materialTransferLayout.cargo 写入或初始化 openAngle（open角度）。
        ? materialTransferLayout.cargoClawOpenAngle : materialTransferLayout.clawOpenAngle;  // 续写上一行表达式的条件取值分支。
    if (!MoveArm(-1, -1, -1, openAngle, materialMotionSteps.placeReleaseSpeed)) return false;  // 若 !MoveArm(-1, -1, -1, openAngle, materialMotionSteps.pl…，立即返回 false。
    waitForArm(delays.placeReleaseMs);  // 调用 waitForArm（waitFor机械臂），传入 delays.placeReleaseMs。
    if (!MoveArm(materialTransferLayout.approachHeight, -1, -1, -1, materialMotionSteps.placeLiftSpeed)) return false;  // 若 !MoveArm(materialTransferLayout.approachHeight, -1, -1…，立即返回 false。
    waitForArm(delays.placeLiftMs);  // 调用 waitForArm（waitFor机械臂），传入 delays.placeLiftMs。
    return true;  // 结束当前函数并返回 true。
}  // 结束此初始化列表或局部作用域。

bool validateAction(const MaterialStationPose &source,  // 定义函数或成员 validateAction，参数由此处的签名确定。
                    const MaterialStationPose &destination) {  // 续写当前声明或运算，提供 const MaterialStationPose &destination) {。
    // 在抓起物料之前同时检查起点、终点及公共动作参数。
    // 这样可以避免抓起来以后才发现目标位置没有标定。
    if (!isLayoutParameterValid() || !isPoseValid(source)  // 当 !isLayoutParameterValid() || !isPoseValid(source 成立时进入此分支，否则继续后续流程。
            || !isPoseValid(destination)) {  // 继续上一行的组合条件，本行提供 || !isPoseValid(destination)) {。
        Serial.println("[Material] ERR: transfer poses are not calibrated");  // 调用 Serial.println，传入 "[Material] ERR: transfer poses are not calibrated"。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此函数或控制块。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

bool planTransferAngles(const float *angles, size_t count, TurretPathPlan &plan) {  // 定义函数或成员 的入口 planTransferAngles，参数由此处的签名确定。
    float actualAngle = 0.0f;  // 将 0.0f 写入或初始化 actualAngle（实测角度）。
    if (!ReadArmTurretAngle(actualAngle)) return false;  // 若 !ReadArmTurretAngle(actualAngle)，立即返回 false。
    if (!PlanTurretPath(actualAngle, angles, count,  // 当 !PlanTurretPath(actualAngle, angles, count, 成立时进入此分支，否则继续后续流程。
                        TURRET_CABLE_MIN_DEG, TURRET_CABLE_MAX_DEG, plan)) {  // 声明 TURRET_CABLE_MAX_DEG（转台CABLE最大角度），类型和可用范围由所在声明决定。
        Serial.println("[Material] ERR: cannot plan turret path");  // 调用 Serial.println，传入 "[Material] ERR: cannot plan turret path"。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此函数或控制块。
    Serial.printf("[Material] turret plan: %u targets, %.1f degrees\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                  static_cast<unsigned>(count), plan.totalDegrees);  // 声明接口 unsigned>，参数由此处的签名确定。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

bool planWorkTransfers(const int *workCodes, const int *cargoCodes,  // 定义函数或成员 planWorkTransfers（plan工位Transfers），参数由此处的签名确定。
                       bool retrieve, TurretPathPlan &plan) {  // 声明 retrieve，类型和可用范围由所在声明决定。
    // 任务顺序不变，整组三件一起比较六个取放目标，不逐段贪心选角。
    if (!workCodes || !cargoCodes) return false;  // 若 !workCodes || !cargoCodes，立即返回 false。
    float angles[TURRET_PATH_MAX_TARGETS] = {};  // 将 {} 写入或初始化 angles[TURRET_PATH_MAX_TARGETS]。
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {  // 按 uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i 组织循环的初值、继续条件和步进。
        if (!isStationCodeValid(workCodes[i]) || !isStationCodeValid(cargoCodes[i])) return false;  // 若 !isStationCodeValid(workCodes[i]) || !isStationCodeVal…，立即返回 false。
        const auto &cargo = materialTransferLayout.cargo;  // 将 materialTransferLayout.cargo 写入或初始化 cargo（货位）。
        const auto &work = materialTransferLayout.workArea[workCodes[i] - 1];  // 将 materialTransferLayout.workArea[workCodes[i] - 1] 写入或初始化 work（工位）。
        if (!validateAction(cargo, work)) return false;  // 若 !validateAction(cargo, work)，立即返回 false。
        angles[2 * i] = retrieve ? work.turretAngle : cargo.turretAngle;  // 将 retrieve ? work.turretAngle : cargo.turretAngle 写入或初始化 angles[2 * i]。
        angles[2 * i + 1] = retrieve ? cargo.turretAngle : work.turretAngle;  // 将 retrieve ? cargo.turretAngle : work.turretAngle 写入或初始化 angles[2 * i + 1]。
    }  // 结束此循环体。
    return planTransferAngles(angles, TURRET_PATH_MAX_TARGETS, plan);  // 结束当前函数并返回 planTransferAngles(angles, TURRET_PATH_MAX_TARGETS, pl…。
}  // 结束此函数或控制块。

bool waitForTrayAngle(float target) {  // 定义函数或成员 的入口 waitForTrayAngle（waitFor料盘角度），参数由此处的签名确定。
    while (!taskMotionAborted) {  // 当 !taskMotionAborted 成立时继续循环。
        float measured = 0;  // 将 0 写入或初始化 measured。
        if (!Servo_QueryAngleMTurn(SERVO_ID_CARGO_TRAY, measured)) break;  // 若 !Servo_QueryAngleMTurn(SERVO_ID_CARGO_TRAY, measured)，从当前循环或分支退出。
        if (fabsf(measured - target) <= CARGO_TRAY_ARRIVAL_TOLERANCE_DEG) return true;  // 若 fabsf(measured - target) <= CARGO_TRAY_ARRIVAL_TOLERAN…，立即返回 true。
        waitForArm(50);  // 调用 waitForArm（waitFor机械臂），传入 50。
    }  // 结束此循环体。
    Servo_HoldPosition(SERVO_ID_CARGO_TRAY);  // 调用 Servo_HoldPosition（Servo保持位置），传入 SERVO_ID_CARGO_TRAY。
    return false;  // 结束当前函数并返回 false。
}  // 结束此函数或控制块。

bool executeWorkTransfer(uint8_t cargoCode, const MaterialStationPose &source, const MaterialStationPose &destination,  // 定义函数或成员 executeWorkTransfer（execute工位Transfer），参数由此处的签名确定。
                         float height, const MaterialTransferDelays &delays, bool retrieve,  // 续列跨行签名或声明中的参数，类型与名称为 float height, const MaterialTransferDelays &delays, bo…。
                         const float *angles = nullptr, uint8_t layer = 1) {  // 将 nullptr, uint8_t layer = 1) { 写入或初始化 angles。
    TurretPathPlan plan;  // 声明 plan，类型和可用范围由所在声明决定。
    if (!angles) {  // 当 !angles 成立时进入此分支，否则继续后续流程。
        const float targets[] = {source.turretAngle, destination.turretAngle};  // 声明 targets，类型和可用范围由所在声明决定。
        if (!planTransferAngles(targets, 2, plan)) return false;  // 若 !planTransferAngles(targets, 2, plan)，立即返回 false。
        angles = plan.anglesDeg;  // 将 plan.anglesDeg 写入或初始化 angles。
    }  // 结束此条件分支。
    // 使用局部规划，不污染普通 MoveArm 和圆盘流程；每次旋转由底层再次回读校验。
    const uint8_t turretAcceleration=retrieve||layer!=1?TURRET_ACCELERATION_FAST:TURRET_ACCELERATION_SLOW;  // 将 retrieve||layer!=1?TURRET_ACCELERATION_FAST:TURRET_ACC… 写入或初始化 turretAcceleration（转台加速度）。
    startCargoTray(cargoCode);  // 调用 startCargoTray（起点货位料盘），传入 cargoCode。
    return pickAt(source, delays, !retrieve, retrieve, angles, turretAcceleration, retrieve ? 0 : cargoCode)  // 结束当前函数并返回 pickAt(source, delays, !retrieve, retrieve, angles, tu…。
        && placeAt(destination, height, delays, true, angles + 1, turretAcceleration, !retrieve,  // 继续上一行的组合条件，本行提供 && placeAt(destination, height, delays, true, angles + 1, turretAcceleration, !retrieve,。
                   retrieve ? 0 : cargoCode, retrieve ? cargoCode : 0);  // 续写当前声明或运算，提供 retrieve ? 0 : cargoCode, retrieve ? cargoCode : 0);。
}  // 结束此初始化列表或局部作用域。

bool validateCodeArrays(const int *first, const int *second, bool firstIsMaterial) {  // 定义函数或成员 的入口 validateCodeArrays，参数由此处的签名确定。
    // 整组动作开始前检查三个扫码值，防止执行到一半才发现任务码错误。
    if (first == nullptr) {  // 当 first == nullptr 成立时进入此分支，否则继续后续流程。
        Serial.println("[Material] ERR: null task code array");  // 调用 Serial.println，传入 "[Material] ERR: null task code array"。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {  // 按 uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i 组织循环的初值、继续条件和步进。
        if ((firstIsMaterial && !isMaterialCodeValid(first[i])) || (!firstIsMaterial && !isStationCodeValid(first[i])) || (second != nullptr && !isStationCodeValid(second[i]))) {  // 当 (firstIsMaterial && !isMaterialCodeValid(first[i])) ||… 成立时进入此分支，否则继续后续流程。
            Serial.println("[Material] ERR: invalid task code");  // 调用 Serial.println，传入 "[Material] ERR: invalid task code"。
            return false;  // 结束当前函数并返回 false。
        }  // 结束此条件分支。
    }  // 结束此循环体。
    const int *positions = firstIsMaterial ? second : first;  // 将 firstIsMaterial ? second : first 写入或初始化 positions。
    // 三个物料必须去三个不同位置，避免把两个物料放到同一个第一层位置。
    if (positions != nullptr && (positions[0] == positions[1] || positions[0] == positions[2] || positions[1] == positions[2])) {  // 当 positions != nullptr && (positions[0] == positions[1] … 成立时进入此分支，否则继续后续流程。
        Serial.println("[Material] ERR: duplicate work-area position code");  // 调用 Serial.println，传入 "[Material] ERR: duplicate work-area position code"。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

} // namespace

bool MaterialTransferPosesReady() {  // 定义函数或成员 的入口 MaterialTransferPosesReady（物料TransferPoses就绪），参数由此处的签名确定。
    if (!isLayoutParameterValid() || !isPoseValid(materialTransferLayout.disc)  // 当 !isLayoutParameterValid() || !isPoseValid(materialTran… 成立时进入此分支，否则继续后续流程。
        || !isPoseValid(materialTransferLayout.cargo)) {  // 继续上一行的组合条件，本行提供 || !isPoseValid(materialTransferLayout.cargo)) {。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此函数或控制块。
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {  // 按 uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i 组织循环的初值、继续条件和步进。
        if (!isPoseValid(materialTransferLayout.workArea[i])) {  // 当 !isPoseValid(materialTransferLayout.workArea[i]) 成立时进入此分支，否则继续后续流程。
            return false;  // 结束当前函数并返回 false。
        }  // 结束此条件分支。
    }  // 结束此循环体。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

bool MoveDiscToCargo(uint8_t materialCode, uint8_t cargoCode) {  // 定义函数或成员 的入口 MoveDiscToCargo（Move圆盘To货位），参数由此处的签名确定。
    // 数组下标从 0 开始，所以外部传入的 1~3 号要减 1。
    if (!isMaterialCodeValid(materialCode) || !isStationCodeValid(cargoCode)) {  // 当 !isMaterialCodeValid(materialCode) || !isStationCodeVa… 成立时进入此分支，否则继续后续流程。
        Serial.println("[Material] ERR: invalid material/cargo code");  // 调用 Serial.println，传入 "[Material] ERR: invalid material/cargo code"。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。
    CargoPlatform &cargo = cargoPlatforms[cargoCode - 1];  // 将 cargoPlatforms[cargoCode - 1] 写入或初始化 cargo（货位）。
    if (cargo.material != MATERIAL_NONE) {  // 当 cargo.material != MATERIAL_NONE 成立时进入此分支，否则继续后续流程。
        Serial.println("[Material] ERR: cargo platform is occupied");  // 调用 Serial.println，传入 "[Material] ERR: cargo platform is occupied"。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。
    const MaterialStationPose &destination = materialTransferLayout.cargo;  // 将 materialTransferLayout.cargo 写入或初始化 destination。
    if (!validateAction(materialTransferLayout.disc, destination)) return false;  // 若 !validateAction(materialTransferLayout.disc, destinati…，立即返回 false。

    Serial.printf("[Material] disc -> cargo %u, material %u\n", cargoCode, materialCode);  // 调用 Serial.printf，传入 "[Material] disc -> cargo %u, material %u\n", cargoCod…。
    // 先从圆盘抓起，再放到指定载物台；完成后才更新载物台状态。
    startCargoTray(cargoCode);  // 调用 startCargoTray（起点货位料盘），传入 cargoCode。
    if (!pickAt(materialTransferLayout.disc, commonMaterialDelays, false, true)  // 当 !pickAt(materialTransferLayout.disc, commonMaterialDel… 成立时进入此分支，否则继续后续流程。
            || !placeAt(destination, destination.high, commonMaterialDelays, true, nullptr,  // 继续上一行的组合条件，本行提供 || !placeAt(destination, destination.high, commonMaterialDelays, true, nullptr,。
                        TURRET_ACCELERATION_FAST, false, 0, cargoCode)) return false;  // 声明 0，类型和可用范围由所在声明决定。
    cargo.material = static_cast<MaterialType>(materialCode);  // 将 static_cast<MaterialType>(materialCode) 写入或初始化 cargo.material。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

bool MoveCargoToWorkArea(uint8_t cargoCode, uint8_t workAreaCode) {  // 定义函数或成员 的入口 MoveCargoToWorkArea（Move货位To工位Area），参数由此处的签名确定。
    if (!isStationCodeValid(cargoCode) || !isStationCodeValid(workAreaCode)) {  // 当 !isStationCodeValid(cargoCode) || !isStationCodeValid(… 成立时进入此分支，否则继续后续流程。
        Serial.println("[Material] ERR: invalid cargo/work-area code");  // 调用 Serial.println，传入 "[Material] ERR: invalid cargo/work-area code"。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。
    CargoPlatform &cargo = cargoPlatforms[cargoCode - 1];  // 将 cargoPlatforms[cargoCode - 1] 写入或初始化 cargo（货位）。
    if (cargo.material == MATERIAL_NONE) {  // 当 cargo.material == MATERIAL_NONE 成立时进入此分支，否则继续后续流程。
        Serial.println("[Material] ERR: cargo platform is empty");  // 调用 Serial.println，传入 "[Material] ERR: cargo platform is empty"。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。
    const MaterialStationPose &source = materialTransferLayout.cargo;  // 将 materialTransferLayout.cargo 写入或初始化 source。
    const MaterialStationPose &destination =  // 续写当前声明或运算，提供 const MaterialStationPose &destination =。
        materialTransferLayout.workArea[workAreaCode - 1];  // 续写当前声明或运算，提供 materialTransferLayout.workArea[workAreaCode - 1];。
    if (!validateAction(source, destination)) return false;  // 若 !validateAction(source, destination)，立即返回 false。

    Serial.printf("[Material] cargo %u -> work area %u, material %u\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                  cargoCode, workAreaCode, cargo.material);  // 声明 workAreaCode（工位AreaCode），类型和可用范围由所在声明决定。
    // 粗加工区和暂存区使用相同的 workArea 位姿。
    // 小车当前停在哪一个区域，动作就发生在哪一个区域。
    const auto &delays = cargoToWorkAreaDelays[cargoCode - 1][workAreaCode - 1];  // 将 cargoToWorkAreaDelays[cargoCode - 1][workAreaCode - 1] 写入或初始化 delays。
    if (!executeWorkTransfer(cargoCode, source, destination, destination.high, delays, false)) return false;  // 若 !executeWorkTransfer(cargoCode, source, destination, d…，立即返回 false。
    cargo.material = MATERIAL_NONE;  // 将 MATERIAL_NONE 写入或初始化 cargo.material。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

//放料
bool DemoCargoToRoughArea() {  // 定义函数或成员 的入口 DemoCargoToRoughArea（Demo货位ToRoughArea），参数由此处的签名确定。
    // 依次完成三次搬运：载物台 1 -> 粗加工区 1、2 -> 2、3 -> 3（第一层）。
    // 每次完整执行抓取、搬运、放置和抬升后，才开始下一次；本函数不负责底盘导航。
    constexpr uint8_t materialCode = MATERIAL_RED; // 物料颜色码：1~6
    static_assert(materialCode >= MATERIAL_RED && materialCode <= MATERIAL_LIGHT_BLUE,  // 调用 static_assert，传入 materialCode >= MATERIAL_RED && materialCode <= MATERI…。
                  "Invalid demo material code");  // 续写当前声明或运算，提供 "Invalid demo material code");。

    const int codes[] = {1, 2, 3};  // 声明 codes，类型和可用范围由所在声明决定。
    TurretPathPlan plan;  // 声明 plan，类型和可用范围由所在声明决定。
    if (!planWorkTransfers(codes, codes, false, plan)) return false;  // 若 !planWorkTransfers(codes, codes, false, plan)，立即返回 false。
    for (uint8_t cargoCode = 1; cargoCode <= MATERIAL_STATION_COUNT; ++cargoCode) {  // 按 uint8_t cargoCode = 1; cargoCode <= MATERIAL_STATION_C… 组织循环的初值、继续条件和步进。
        // 编号为 1~3，目标位置顺序对应为 1~3；数组下标需要减 1。
        const uint8_t workAreaCode = cargoCode;  // 将 cargoCode 写入或初始化 workAreaCode（工位AreaCode）。
        // 沿用原示例：手动装料后临时登记为红色物料，复用完整搬运接口。
        // 失败时恢复当前载物台记录并停止；已完成的载物台保持空载。
        CargoPlatform &cargo = cargoPlatforms[cargoCode - 1];  // 将 cargoPlatforms[cargoCode - 1] 写入或初始化 cargo（货位）。
        const MaterialType previousMaterial = cargo.material;  // 将 cargo.material 写入或初始化 previousMaterial（上一物料）。
        cargo.material = static_cast<MaterialType>(materialCode);  // 将 static_cast<MaterialType>(materialCode) 写入或初始化 cargo.material。
        const auto &delays = cargoToWorkAreaDelays[cargoCode - 1][workAreaCode - 1];  // 将 cargoToWorkAreaDelays[cargoCode - 1][workAreaCode - 1] 写入或初始化 delays。
        if (!executeWorkTransfer(cargoCode, materialTransferLayout.cargo,  // 当 !executeWorkTransfer(cargoCode, materialTransferLayout… 成立时进入此分支，否则继续后续流程。
                                 materialTransferLayout.workArea[workAreaCode - 1],  // 为当前列表或跨行表达式提供这一项 materialTransferLayout.workArea[workAreaCode - 1]。
                                 materialTransferLayout.workArea[workAreaCode - 1].high,  // 为当前列表或跨行表达式提供这一项 materialTransferLayout.workArea[workAreaCode - 1].high。
                                 delays, false, &plan.anglesDeg[2 * (cargoCode - 1)])) {  // 声明 false，类型和可用范围由所在声明决定。
            cargo.material = previousMaterial;  // 将 previousMaterial 写入或初始化 cargo.material。
            return false;  // 结束当前函数并返回 false。
        }  // 结束此函数或控制块。
        cargo.material = MATERIAL_NONE;  // 将 MATERIAL_NONE 写入或初始化 cargo.material。
    }  // 结束此循环体。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

bool DemoWorkAreaToCargo() {  // 定义函数或成员 的入口 DemoWorkAreaToCargo（Demo工位AreaTo货位），参数由此处的签名确定。
    // 调试时暂存区的三个位置各放一件物料；颜色只用于载物台状态记录。
    // 取料仍按区域 1、2、3；放回载物台时按内侧到外侧的 3、2、1 顺序。
    constexpr uint8_t materialCode = MATERIAL_RED;  // 将 MATERIAL_RED 写入或初始化 materialCode（物料Code）。
    static_assert(materialCode >= MATERIAL_RED && materialCode <= MATERIAL_LIGHT_BLUE,  // 调用 static_assert，传入 materialCode >= MATERIAL_RED && materialCode <= MATERI…。
                  "Invalid demo material code");  // 续写当前声明或运算，提供 "Invalid demo material code");。

    // 整组预检查：避免搬到一半才发现后续载物台已占用或位姿无效。
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {  // 按 uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i 组织循环的初值、继续条件和步进。
        if (cargoPlatforms[i].material != MATERIAL_NONE) {  // 当 cargoPlatforms[i].material != MATERIAL_NONE 成立时进入此分支，否则继续后续流程。
            Serial.println("[Material] ERR: cargo platform is occupied");  // 调用 Serial.println，传入 "[Material] ERR: cargo platform is occupied"。
            return false;  // 结束当前函数并返回 false。
        }  // 结束此条件分支。
        if (!validateAction(materialTransferLayout.workArea[i],  // 当 !validateAction(materialTransferLayout.workArea[i], 成立时进入此分支，否则继续后续流程。
                            materialTransferLayout.cargo)) return false;  // 续写当前声明或运算，提供 materialTransferLayout.cargo)) return false;。
    }  // 结束此循环体。

    const int workCodes[] = {1, 2, 3}, cargoCodes[] = {3, 2, 1};  // 声明 workCodes（工位Codes），类型和可用范围由所在声明决定。
    TurretPathPlan plan;  // 声明 plan，类型和可用范围由所在声明决定。
    if (!planWorkTransfers(workCodes, cargoCodes, true, plan)) return false;  // 若 !planWorkTransfers(workCodes, cargoCodes, true, plan)，立即返回 false。
    for (uint8_t workAreaCode = 1; workAreaCode <= MATERIAL_STATION_COUNT; ++workAreaCode) {  // 按 uint8_t workAreaCode = 1; workAreaCode <= MATERIAL_STA… 组织循环的初值、继续条件和步进。
        // 区域 1→载物台 3、区域 2→载物台 2、区域 3→载物台 1。
        // 单次接口会把外部 1~3 编号转换成数组 0~2 下标；每件放稳、抬升后才取下一件。
        const uint8_t cargoCode = MATERIAL_STATION_COUNT + 1 - workAreaCode;  // 将 MATERIAL_STATION_COUNT + 1 - workAreaCode 写入或初始化 cargoCode（货位Code）。
        const auto &destination = materialTransferLayout.cargo;  // 将 materialTransferLayout.cargo 写入或初始化 destination。
        const auto &delays = workAreaToCargoDelays[workAreaCode - 1][cargoCode - 1];  // 将 workAreaToCargoDelays[workAreaCode - 1][cargoCode - 1] 写入或初始化 delays。
        if (!executeWorkTransfer(cargoCode, materialTransferLayout.workArea[workAreaCode - 1], destination,  // 当 !executeWorkTransfer(cargoCode, materialTransferLayout… 成立时进入此分支，否则继续后续流程。
                                 destination.high, delays, true,  // 为当前列表或跨行表达式提供这一项 destination.high, delays, true。
                                 &plan.anglesDeg[2 * (workAreaCode - 1)])) return false;  // 续写当前声明或运算，提供 &plan.anglesDeg[2 * (workAreaCode - 1)])) return false;。
        cargoPlatforms[cargoCode - 1].material = static_cast<MaterialType>(materialCode);  // 将 static_cast<MaterialType>(materialCode) 写入或初始化 material（物料）。
    }  // 结束此循环体。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

//码放（与放料类似，高度增加）
bool DemoStackCargoToWorkArea() {  // 定义函数或成员 的入口 DemoStackCargoToWorkArea（DemoStack货位To工位Area），参数由此处的签名确定。
    // 手动在三个载物台装好物料，小车停在码放区后调用。
    // 与 MaterialDemo 一样依次执行 1 -> 1、2 -> 2、3 -> 3；
    // 仅松手高度增加 secondLayerOffset。
    float releaseHeights[MATERIAL_STATION_COUNT];  // 声明 releaseHeights，类型和可用范围由所在声明决定。
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {  // 按 uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i 组织循环的初值、继续条件和步进。
        const MaterialStationPose &destination = materialTransferLayout.workArea[i];  // 将 materialTransferLayout.workArea[i] 写入或初始化 destination。
        releaseHeights[i] = destination.high + materialTransferLayout.secondLayerOffset;  // 将 destination.high + materialTransferLayout.secondLayerO… 写入或初始化 releaseHeights[i]。
        // 三个位置全部检查通过后才抓料，避免执行到一半才发现高度越界。
        if (!validateAction(materialTransferLayout.cargo, destination)) return false;  // 若 !validateAction(materialTransferLayout.cargo, destinat…，立即返回 false。
        if (!isfinite(releaseHeights[i]) || releaseHeights[i] > ARM_HEIGHT_LIMIT_MM) {  // 当 !isfinite(releaseHeights[i]) || releaseHeights[i] > AR… 成立时进入此分支，否则继续后续流程。
            Serial.println("[Material] ERR: invalid demo stacking height");  // 调用 Serial.println，传入 "[Material] ERR: invalid demo stacking height"。
            return false;  // 结束当前函数并返回 false。
        }  // 结束此条件分支。
    }  // 结束此循环体。

    const int codes[] = {1, 2, 3};  // 声明 codes，类型和可用范围由所在声明决定。
    TurretPathPlan plan;  // 声明 plan，类型和可用范围由所在声明决定。
    if (!planWorkTransfers(codes, codes, false, plan)) return false;  // 若 !planWorkTransfers(codes, codes, false, plan)，立即返回 false。
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {  // 按 uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i 组织循环的初值、继续条件和步进。
        // 数组下标 0~2 对应载物台和区域编号 1~3。
        // 手动装料按红色登记，完成抓取、码放和收回后再清空记录。
        cargoPlatforms[i].material = MATERIAL_RED;  // 将 MATERIAL_RED 写入或初始化 material（物料）。
        Serial.printf("[Material] demo stack cargo %u -> work area %u, release height %.1f\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                      i + 1, i + 1, releaseHeights[i]);  // 续写当前声明或运算，提供 i + 1, i + 1, releaseHeights[i]);。
        const auto &delays = cargoToWorkAreaDelays[i][i];  // 将 cargoToWorkAreaDelays[i][i] 写入或初始化 delays。
        if (!executeWorkTransfer(i + 1, materialTransferLayout.cargo, materialTransferLayout.workArea[i],  // 当 !executeWorkTransfer(i + 1, materialTransferLayout.car… 成立时进入此分支，否则继续后续流程。
                                 releaseHeights[i], delays, false, &plan.anglesDeg[2 * i], 2)) return false;  // 续写当前声明或运算，提供 releaseHeights[i], delays, false, &plan.anglesDeg[2 * …。
        cargoPlatforms[i].material = MATERIAL_NONE;  // 将 MATERIAL_NONE 写入或初始化 material（物料）。
    }  // 结束此循环体。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

bool DemoStackCargoToWorkArea3() {  // 定义函数或成员 的入口 DemoStackCargoToWorkArea3（DemoStack货位To工位Area3），参数由此处的签名确定。
    // 手动在三个载物台装好物料，小车停在码放区后调用。
    // 与 MaterialDemo 一样依次执行 1 -> 1、2 -> 2、3 -> 3；
    // 仅松手高度增加 两倍secondLayerOffset。
    float releaseHeights[MATERIAL_STATION_COUNT];  // 声明 releaseHeights，类型和可用范围由所在声明决定。
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {  // 按 uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i 组织循环的初值、继续条件和步进。
        const MaterialStationPose &destination = materialTransferLayout.workArea[i];  // 将 materialTransferLayout.workArea[i] 写入或初始化 destination。
        releaseHeights[i] = destination.high + materialTransferLayout.secondLayerOffset * 2;  // 将 destination.high + materialTransferLayout.secondLayerO… 写入或初始化 releaseHeights[i]。
        // 三个位置全部检查通过后才抓料，避免执行到一半才发现高度越界。
        if (!validateAction(materialTransferLayout.cargo, destination)) return false;  // 若 !validateAction(materialTransferLayout.cargo, destinat…，立即返回 false。
        if (!isfinite(releaseHeights[i]) || releaseHeights[i] > ARM_HEIGHT_LIMIT_MM) {  // 当 !isfinite(releaseHeights[i]) || releaseHeights[i] > AR… 成立时进入此分支，否则继续后续流程。
            Serial.println("[Material] ERR: invalid demo stacking height");  // 调用 Serial.println，传入 "[Material] ERR: invalid demo stacking height"。
            return false;  // 结束当前函数并返回 false。
        }  // 结束此条件分支。
    }  // 结束此循环体。

    const int codes[] = {1, 2, 3};  // 声明 codes，类型和可用范围由所在声明决定。
    TurretPathPlan plan;  // 声明 plan，类型和可用范围由所在声明决定。
    if (!planWorkTransfers(codes, codes, false, plan)) return false;  // 若 !planWorkTransfers(codes, codes, false, plan)，立即返回 false。
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {  // 按 uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i 组织循环的初值、继续条件和步进。
        // 数组下标 0~2 对应载物台和区域编号 1~3。
        // 手动装料按红色登记，完成抓取、码放和收回后再清空记录。
        cargoPlatforms[i].material = MATERIAL_RED;  // 将 MATERIAL_RED 写入或初始化 material（物料）。
        Serial.printf("[Material] demo stack cargo %u -> work area %u, release height %.1f\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                      i + 1, i + 1, releaseHeights[i]);  // 续写当前声明或运算，提供 i + 1, i + 1, releaseHeights[i]);。
        const auto &delays = cargoToWorkAreaDelays[i][i];  // 将 cargoToWorkAreaDelays[i][i] 写入或初始化 delays。
        if (!executeWorkTransfer(i + 1, materialTransferLayout.cargo, materialTransferLayout.workArea[i],  // 当 !executeWorkTransfer(i + 1, materialTransferLayout.car… 成立时进入此分支，否则继续后续流程。
                                 releaseHeights[i], delays, false, &plan.anglesDeg[2 * i], 3)) return false;  // 续写当前声明或运算，提供 releaseHeights[i], delays, false, &plan.anglesDeg[2 * …。
        cargoPlatforms[i].material = MATERIAL_NONE;  // 将 MATERIAL_NONE 写入或初始化 material（物料）。
    }  // 结束此循环体。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

bool PrepareLidarScanPose(uint8_t startZoneCode) {  // 定义函数或成员 的入口 PrepareLidarScanPose（PrepareLidar扫码Pose），参数由此处的签名确定。
    switch (startZoneCode) {  // 按 startZoneCode 的值选择对应分支。
        case 1:  // 当前值匹配 1 时从这里执行。
            return MovePosition(100,100,0,200) && MovePosition(0,0,-45,200);  // 结束当前函数并返回 MovePosition(100,100,0,200) && MovePosition(0,0,-45,20…。
        case 2:  // 当前值匹配 2 时从这里执行。
            return MovePosition(-100,100,0,200) && MovePosition(0,0,45,200);  // 结束当前函数并返回 MovePosition(-100,100,0,200) && MovePosition(0,0,45,20…。
        default:  // 没有匹配的 case 时走此默认分支。
            Serial.println("[LidarPose] ERR: start zone must be 1 or 2");  // 调用 Serial.println，传入 "[LidarPose] ERR: start zone must be 1 or 2"。
            return false;  // 结束当前函数并返回 false。
    }  // 结束此函数或控制块。

}  // 结束此函数或控制块。

bool MoveWorkAreaToCargo(uint8_t materialCode, uint8_t workAreaCode, uint8_t cargoCode) {  // 定义函数或成员 的入口 MoveWorkAreaToCargo（Move工位AreaTo货位），参数由此处的签名确定。
    if (!isMaterialCodeValid(materialCode) || !isStationCodeValid(workAreaCode) || !isStationCodeValid(cargoCode)) {  // 当 !isMaterialCodeValid(materialCode) || !isStationCodeVa… 成立时进入此分支，否则继续后续流程。
        Serial.println("[Material] ERR: invalid material/work-area/cargo code");  // 调用 Serial.println，传入 "[Material] ERR: invalid material/work-area/cargo code"。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。
    CargoPlatform &cargo = cargoPlatforms[cargoCode - 1];  // 将 cargoPlatforms[cargoCode - 1] 写入或初始化 cargo（货位）。
    if (cargo.material != MATERIAL_NONE) {  // 当 cargo.material != MATERIAL_NONE 成立时进入此分支，否则继续后续流程。
        Serial.println("[Material] ERR: cargo platform is occupied");  // 调用 Serial.println，传入 "[Material] ERR: cargo platform is occupied"。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。
    const MaterialStationPose &source =  // 续写当前声明或运算，提供 const MaterialStationPose &source =。
        materialTransferLayout.workArea[workAreaCode - 1];  // 续写当前声明或运算，提供 materialTransferLayout.workArea[workAreaCode - 1];。
    const MaterialStationPose &destination = materialTransferLayout.cargo;  // 将 materialTransferLayout.cargo 写入或初始化 destination。
    if (!validateAction(source, destination)) return false;  // 若 !validateAction(source, destination)，立即返回 false。

    Serial.printf("[Material] work area %u -> cargo %u, material %u\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                  workAreaCode, cargoCode, materialCode);  // 声明 cargoCode（货位Code），类型和可用范围由所在声明决定。
    // 取回：同时抬升、缩回 -> 等待 -> 缩回转向 -> 等待 -> 同时伸出、下降。
    const auto &delays = workAreaToCargoDelays[workAreaCode - 1][cargoCode - 1];  // 将 workAreaToCargoDelays[workAreaCode - 1][cargoCode - 1] 写入或初始化 delays。
    if (!executeWorkTransfer(cargoCode, source, destination, destination.high, delays, true)) return false;  // 若 !executeWorkTransfer(cargoCode, source, destination, d…，立即返回 false。
    // 抓取成功并放到车上以后，记录该载物台装入了什么颜色的物料。
    cargo.material = static_cast<MaterialType>(materialCode);  // 将 static_cast<MaterialType>(materialCode) 写入或初始化 cargo.material。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

bool StackCargoToWorkArea(uint8_t cargoCode, uint8_t workAreaCode) {  // 定义函数或成员 的入口 StackCargoToWorkArea（Stack货位To工位Area），参数由此处的签名确定。
    if (!isStationCodeValid(cargoCode) || !isStationCodeValid(workAreaCode)) {  // 当 !isStationCodeValid(cargoCode) || !isStationCodeValid(… 成立时进入此分支，否则继续后续流程。
        Serial.println("[Material] ERR: invalid cargo/work-area code");  // 调用 Serial.println，传入 "[Material] ERR: invalid cargo/work-area code"。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。
    CargoPlatform &cargo = cargoPlatforms[cargoCode - 1];  // 将 cargoPlatforms[cargoCode - 1] 写入或初始化 cargo（货位）。
    if (cargo.material == MATERIAL_NONE) {  // 当 cargo.material == MATERIAL_NONE 成立时进入此分支，否则继续后续流程。
        Serial.println("[Material] ERR: cargo platform is empty");  // 调用 Serial.println，传入 "[Material] ERR: cargo platform is empty"。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。
    const MaterialStationPose &source = materialTransferLayout.cargo;  // 将 materialTransferLayout.cargo 写入或初始化 source。
    const MaterialStationPose &destination =  // 续写当前声明或运算，提供 const MaterialStationPose &destination =。
        materialTransferLayout.workArea[workAreaCode - 1];  // 续写当前声明或运算，提供 materialTransferLayout.workArea[workAreaCode - 1];。
    // 第二层的放置高度 = 第一层标定高度 + 一个物料的码放高度。
    const float secondLayerHeight =  // 续写当前声明或运算，提供 const float secondLayerHeight =。
        destination.high + materialTransferLayout.secondLayerOffset;  // 续写当前声明或运算，提供 destination.high + materialTransferLayout.secondLayerO…。
    if (!validateAction(source, destination)) return false;  // 若 !validateAction(source, destination)，立即返回 false。
    if (secondLayerHeight > ARM_HEIGHT_LIMIT_MM) {  // 当 secondLayerHeight > ARM_HEIGHT_LIMIT_MM 成立时进入此分支，否则继续后续流程。
        Serial.println("[Material] ERR: invalid second-layer height");  // 调用 Serial.println，传入 "[Material] ERR: invalid second-layer height"。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。

    Serial.printf("[Material] cargo %u -> work area %u layer 2, material %u\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                  cargoCode, workAreaCode, cargo.material);  // 声明 workAreaCode（工位AreaCode），类型和可用范围由所在声明决定。
    const auto &delays = cargoToWorkAreaDelays[cargoCode - 1][workAreaCode - 1];  // 将 cargoToWorkAreaDelays[cargoCode - 1][workAreaCode - 1] 写入或初始化 delays。
    if (!executeWorkTransfer(cargoCode, source, destination, secondLayerHeight, delays, false, nullptr, 2)) return false;  // 若 !executeWorkTransfer(cargoCode, source, destination, s…，立即返回 false。
    cargo.material = MATERIAL_NONE;  // 将 MATERIAL_NONE 写入或初始化 cargo.material。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

bool LoadRoundFromDisc(const int materialCodes[MATERIAL_STATION_COUNT]) {  // 定义函数或成员 的入口 LoadRoundFromDisc（Load轮次From圆盘），参数由此处的签名确定。
    // materialCodes[0..2] 对应扫码结果中的三个颜色码。
    // 三个物料固定依次放入 3、2、1 号载物台。
    if (!validateCodeArrays(materialCodes, nullptr, true)  // 当 !validateCodeArrays(materialCodes, nullptr, true 成立时进入此分支，否则继续后续流程。
            || !MaterialTransferPosesReady()) return false;  // 继续上一行的组合条件，本行提供 || !MaterialTransferPosesReady()) return false;。
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {  // 按 uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i 组织循环的初值、继续条件和步进。
        if (cargoPlatforms[i].material != MATERIAL_NONE) {  // 当 cargoPlatforms[i].material != MATERIAL_NONE 成立时进入此分支，否则继续后续流程。
            Serial.println("[Material] ERR: cargo platform is occupied");  // 调用 Serial.println，传入 "[Material] ERR: cargo platform is occupied"。
            return false;  // 结束当前函数并返回 false。
        }  // 结束此条件分支。
    }  // 结束此循环体。
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {  // 按 uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i 组织循环的初值、继续条件和步进。
        if (!MoveDiscToCargo(materialCodes[i], CargoForGrabIndex(i))) return false;  // 若 !MoveDiscToCargo(materialCodes[i], CargoForGrabIndex(i…，立即返回 false。
    }  // 结束此循环体。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

bool PlaceTaskCargoToWorkArea(  // 定义函数或成员 PlaceTaskCargoToWorkArea（Place任务货位To工位Area），参数由此处的签名确定。
        const int positionCodes[MATERIAL_STATION_COUNT], uint8_t layer) {  // 声明 positionCodes（位置Codes），类型和可用范围由所在声明决定。
    // 1. 整组预检查：无效任务码、层数或目标高度不能让机械臂先动起来。
    if (layer < 1 || layer > 3) {  // 当 layer < 1 || layer > 3 成立时进入此分支，否则继续后续流程。
        Serial.println("[Material] ERR: layer must be 1, 2 or 3");  // 调用 Serial.println，传入 "[Material] ERR: layer must be 1, 2 or 3"。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。
    if (!validateCodeArrays(positionCodes, nullptr, false)  // 当 !validateCodeArrays(positionCodes, nullptr, false 成立时进入此分支，否则继续后续流程。
            || !MaterialTransferPosesReady()) return false;  // 继续上一行的组合条件，本行提供 || !MaterialTransferPosesReady()) return false;。

    float releaseHeights[MATERIAL_STATION_COUNT];  // 声明 releaseHeights，类型和可用范围由所在声明决定。
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {  // 按 uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i 组织循环的初值、继续条件和步进。
        if (cargoPlatforms[i].material == MATERIAL_NONE) {  // 当 cargoPlatforms[i].material == MATERIAL_NONE 成立时进入此分支，否则继续后续流程。
            Serial.println("[Material] ERR: cargo platform is empty");  // 调用 Serial.println，传入 "[Material] ERR: cargo platform is empty"。
            return false;  // 结束当前函数并返回 false。
        }  // 结束此条件分支。
        const MaterialStationPose &source = materialTransferLayout.cargo;  // 将 materialTransferLayout.cargo 写入或初始化 source。
        const MaterialStationPose &destination =  // 续写当前声明或运算，提供 const MaterialStationPose &destination =。
            materialTransferLayout.workArea[positionCodes[i] - 1];  // 续写当前声明或运算，提供 materialTransferLayout.workArea[positionCodes[i] - 1];。
        releaseHeights[i] = destination.high  // 将 destination.high 写入或初始化 releaseHeights[i]。
            + static_cast<float>(layer - 1) * materialTransferLayout.secondLayerOffset;  // 声明接口 float>，参数由此处的签名确定。
        if (!validateAction(source, destination)) return false;  // 若 !validateAction(source, destination)，立即返回 false。
        if (!isfinite(releaseHeights[i]) || releaseHeights[i] < 0.0f  // 当 !isfinite(releaseHeights[i]) || releaseHeights[i] < 0.… 成立时进入此分支，否则继续后续流程。
                || releaseHeights[i] > ARM_HEIGHT_LIMIT_MM) {  // 继续上一行的组合条件，本行提供 || releaseHeights[i] > ARM_HEIGHT_LIMIT_MM) {。
            Serial.println("[Material] ERR: invalid stacking height");  // 调用 Serial.println，传入 "[Material] ERR: invalid stacking height"。
            return false;  // 结束当前函数并返回 false。
        }  // 结束此函数或控制块。
    }  // 结束此循环体。

    // 2. 与已测试 Demo 相同的逐件动作：抓取、转向放置、松手、抬升。
    // 编号从 1 开始，数组下标从 0 开始；成功放完一件才清空其载物台记录。
    const int cargoCodes[] = {CargoForGrabIndex(0), CargoForGrabIndex(1), CargoForGrabIndex(2)};  // 调用 CargoForGrabIndex（货位ForGrab下标），传入 0), CargoForGrabIndex(1), CargoForGrabIndex(2)}。
    TurretPathPlan plan;  // 声明 plan，类型和可用范围由所在声明决定。
    if (!planWorkTransfers(positionCodes, cargoCodes, false, plan)) return false;  // 若 !planWorkTransfers(positionCodes, cargoCodes, false, p…，立即返回 false。
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {  // 按 uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i 组织循环的初值、继续条件和步进。
        const uint8_t cargoCode = cargoCodes[i];  // 将 cargoCodes[i] 写入或初始化 cargoCode（货位Code）。
        const uint8_t workAreaCode = static_cast<uint8_t>(positionCodes[i]);  // 将 static_cast<uint8_t>(positionCodes[i]) 写入或初始化 workAreaCode（工位AreaCode）。
        Serial.printf("[Material] cargo %u -> work area %u, layer %u, release height %.1f\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                      cargoCode, workAreaCode, layer, releaseHeights[i]);  // 声明 releaseHeights，类型和可用范围由所在声明决定。
        const auto &delays = cargoToWorkAreaDelays[cargoCode - 1][workAreaCode - 1];  // 将 cargoToWorkAreaDelays[cargoCode - 1][workAreaCode - 1] 写入或初始化 delays。
        if (!executeWorkTransfer(cargoCode, materialTransferLayout.cargo,  // 当 !executeWorkTransfer(cargoCode, materialTransferLayout… 成立时进入此分支，否则继续后续流程。
                                 materialTransferLayout.workArea[workAreaCode - 1],  // 为当前列表或跨行表达式提供这一项 materialTransferLayout.workArea[workAreaCode - 1]。
                                 releaseHeights[i], delays, false, &plan.anglesDeg[2 * i], layer)) return false;  // 续写当前声明或运算，提供 releaseHeights[i], delays, false, &plan.anglesDeg[2 * …。
        cargoPlatforms[cargoCode - 1].material = MATERIAL_NONE;  // 将 MATERIAL_NONE 写入或初始化 material（物料）。
    }  // 结束此循环体。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

bool PlaceRoundToWorkArea(const int positionCodes[MATERIAL_STATION_COUNT]) {  // 定义函数或成员 的入口 PlaceRoundToWorkArea（Place轮次To工位Area），参数由此处的签名确定。
    return PlaceTaskCargoToWorkArea(positionCodes, 1);  // 结束当前函数并返回 PlaceTaskCargoToWorkArea(positionCodes, 1)。
}  // 结束此函数或控制块。

bool RetrieveRoundToCargo(  // 定义函数或成员 RetrieveRoundToCargo（Retrieve轮次To货位），参数由此处的签名确定。
        const int materialCodes[MATERIAL_STATION_COUNT],  // 续列跨行签名或声明中的参数，类型与名称为 const int materialCodes[MATERIAL_STATION_COUNT]。
        const int positionCodes[MATERIAL_STATION_COUNT]) {  // 声明 positionCodes（位置Codes），类型和可用范围由所在声明决定。
    // 从 positionCodes 指定的位置取出 materialCodes 指定的物料，
    // 并按照原顺序分别放回 3、2、1 号载物台。
    if (!validateCodeArrays(materialCodes, positionCodes, true)  // 当 !validateCodeArrays(materialCodes, positionCodes, true 成立时进入此分支，否则继续后续流程。
            || !MaterialTransferPosesReady()) return false;  // 继续上一行的组合条件，本行提供 || !MaterialTransferPosesReady()) return false;。
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {  // 按 uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i 组织循环的初值、继续条件和步进。
        if (cargoPlatforms[i].material != MATERIAL_NONE) {  // 当 cargoPlatforms[i].material != MATERIAL_NONE 成立时进入此分支，否则继续后续流程。
            Serial.println("[Material] ERR: cargo platform is occupied");  // 调用 Serial.println，传入 "[Material] ERR: cargo platform is occupied"。
            return false;  // 结束当前函数并返回 false。
        }  // 结束此条件分支。
    }  // 结束此循环体。
    const int cargoCodes[] = {CargoForGrabIndex(0), CargoForGrabIndex(1), CargoForGrabIndex(2)};  // 调用 CargoForGrabIndex（货位ForGrab下标），传入 0), CargoForGrabIndex(1), CargoForGrabIndex(2)}。
    TurretPathPlan plan;  // 声明 plan，类型和可用范围由所在声明决定。
    if (!planWorkTransfers(positionCodes, cargoCodes, true, plan)) return false;  // 若 !planWorkTransfers(positionCodes, cargoCodes, true, pl…，立即返回 false。
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {  // 按 uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i 组织循环的初值、继续条件和步进。
        const auto &destination = materialTransferLayout.cargo;  // 将 materialTransferLayout.cargo 写入或初始化 destination。
        const uint8_t cargoCode = cargoCodes[i];  // 将 cargoCodes[i] 写入或初始化 cargoCode（货位Code）。
        const auto &delays = workAreaToCargoDelays[positionCodes[i] - 1][cargoCode - 1];  // 将 workAreaToCargoDelays[positionCodes[i] - 1][cargoCode … 写入或初始化 delays。
        if (!executeWorkTransfer(cargoCode, materialTransferLayout.workArea[positionCodes[i] - 1], destination,  // 当 !executeWorkTransfer(cargoCode, materialTransferLayout… 成立时进入此分支，否则继续后续流程。
                                 destination.high, delays, true,  // 为当前列表或跨行表达式提供这一项 destination.high, delays, true。
                                 &plan.anglesDeg[2 * i])) return false;  // 续写当前声明或运算，提供 &plan.anglesDeg[2 * i])) return false;。
        cargoPlatforms[cargoCode - 1].material = static_cast<MaterialType>(materialCodes[i]);  // 将 static_cast<MaterialType>(materialCodes[i]) 写入或初始化 material（物料）。
    }  // 结束此循环体。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

bool StackRoundToWorkArea(const int positionCodes[MATERIAL_STATION_COUNT]) {  // 定义函数或成员 的入口 StackRoundToWorkArea（Stack轮次To工位Area），参数由此处的签名确定。
    return PlaceTaskCargoToWorkArea(positionCodes, 2);  // 结束当前函数并返回 PlaceTaskCargoToWorkArea(positionCodes, 2)。
}  // 结束此函数或控制块。

// 圆盘抓取并放入指定载物台。
bool GrabDiscMaterial(uint8_t materialCode, uint8_t cargoCode, bool force) {  // 定义函数或成员 的入口 GrabDiscMaterial（Grab圆盘物料），参数由此处的签名确定。
    // 在底盘和机械臂动作前检查编号、占用及放料位姿，避免抓起后无处放置。
    if (!isMaterialCodeValid(materialCode) || !isStationCodeValid(cargoCode)) {  // 当 !isMaterialCodeValid(materialCode) || !isStationCodeVa… 成立时进入此分支，否则继续后续流程。
        Serial.println("[Material] ERR: invalid material/cargo code");  // 调用 Serial.println，传入 "[Material] ERR: invalid material/cargo code"。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。
    CargoPlatform &cargo = cargoPlatforms[cargoCode - 1];  // 将 cargoPlatforms[cargoCode - 1] 写入或初始化 cargo（货位）。
    if (cargo.material != MATERIAL_NONE) {  // 当 cargo.material != MATERIAL_NONE 成立时进入此分支，否则继续后续流程。
        if (!force) {  // 当 !force 成立时进入此分支，否则继续后续流程。
            Serial.println("[Material] ERR: cargo platform is occupied");  // 调用 Serial.println，传入 "[Material] ERR: cargo platform is occupied"。
            return false;  // 结束当前函数并返回 false。
        }  // 结束此条件分支。
        Serial.printf("{EVT,VISION,FORCE_GRAB,WARN,OCCUPIED,%u,%u}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                      static_cast<unsigned>(cargo.material), cargoCode);  // 声明接口 unsigned>，参数由此处的签名确定。
        Serial.printf("[Material] WARN: cargo %u already contains material %u; force grab continues\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                      cargoCode, static_cast<unsigned>(cargo.material));  // 声明接口 unsigned>，参数由此处的签名确定。
    }  // 结束此条件分支。
    const MaterialStationPose &destination = materialTransferLayout.cargo;  // 将 materialTransferLayout.cargo 写入或初始化 destination。
    if (!isLayoutParameterValid() || !isPoseValid(destination)) {  // 当 !isLayoutParameterValid() || !isPoseValid(destination) 成立时进入此分支，否则继续后续流程。
        Serial.println("[Material] ERR: transfer poses are not calibrated");  // 调用 Serial.println，传入 "[Material] ERR: transfer poses are not calibrated"。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。

    if (!isPoseValid(materialMotionSteps.discApproach)  // 当 !isPoseValid(materialMotionSteps.discApproach 成立时进入此分支，否则继续后续流程。
            || !isfinite(materialMotionSteps.discOpenAngle)  // 继续上一行的组合条件，本行提供 || !isfinite(materialMotionSteps.discOpenAngle)。
            || !isfinite(materialMotionSteps.discLiftHeight)  // 继续上一行的组合条件，本行提供 || !isfinite(materialMotionSteps.discLiftHeight)。
            || materialMotionSteps.discLiftHeight < 0  // 继续上一行的组合条件，本行提供 || materialMotionSteps.discLiftHeight < 0。
            || materialMotionSteps.discLiftHeight > ARM_HEIGHT_LIMIT_MM) return false;  // 继续上一行的组合条件，本行提供 || materialMotionSteps.discLiftHeight > ARM_HEIGHT_LIMIT_MM) return false;。
    startCargoTray(cargoCode);  // 调用 startCargoTray（起点货位料盘），传入 cargoCode。
    if (!MoveArm(materialMotionSteps.discApproach.high,  // 当 !MoveArm(materialMotionSteps.discApproach.high, 成立时进入此分支，否则继续后续流程。
                 materialMotionSteps.discApproach.length,  // 为当前列表或跨行表达式提供这一项 materialMotionSteps.discApproach.length。
                 materialMotionSteps.discApproach.turretAngle,  // 为当前列表或跨行表达式提供这一项 materialMotionSteps.discApproach.turretAngle。
                 materialMotionSteps.discOpenAngle,  // 为当前列表或跨行表达式提供这一项 materialMotionSteps.discOpenAngle。
                 materialMotionSteps.discApproachSpeed, 20)) return false;  // 续写当前声明或运算，提供 materialMotionSteps.discApproachSpeed, 20)) return fal…。
    waitForArm(materialAuxiliaryDelays.discApproachMs);  // 调用 waitForArm（waitFor机械臂），传入 materialAuxiliaryDelays.discApproachMs。
    if (!MoveArm(materialMotionSteps.discApproach.high,  // 当 !MoveArm(materialMotionSteps.discApproach.high, 成立时进入此分支，否则继续后续流程。
                 materialMotionSteps.discApproach.length,  // 为当前列表或跨行表达式提供这一项 materialMotionSteps.discApproach.length。
                 materialMotionSteps.discApproach.turretAngle,  // 为当前列表或跨行表达式提供这一项 materialMotionSteps.discApproach.turretAngle。
                 materialTransferLayout.clawClosedAngle,  // 为当前列表或跨行表达式提供这一项 materialTransferLayout.clawClosedAngle。
                 materialMotionSteps.discGripSpeed, 20)) return false;  // 续写当前声明或运算，提供 materialMotionSteps.discGripSpeed, 20)) return false;。
    waitForArm(commonMaterialDelays.pickGripMs);  // 调用 waitForArm（waitFor机械臂），传入 commonMaterialDelays.pickGripMs。
    if (!MoveArm(materialTransferLayout.approachHeight, 0, -1, -1,  // 当 !MoveArm(materialTransferLayout.approachHeight, 0, -1,… 成立时进入此分支，否则继续后续流程。
                 materialMotionSteps.discLiftSpeed)) return false;  // 续写当前声明或运算，提供 materialMotionSteps.discLiftSpeed)) return false;。
    waitForArm(commonMaterialDelays.pickLiftMs);  // 调用 waitForArm（waitFor机械臂），传入 commonMaterialDelays.pickLiftMs。

    if (!placeAt(destination, destination.high, commonMaterialDelays, true, nullptr,  // 当 !placeAt(destination, destination.high, commonMaterial… 成立时进入此分支，否则继续后续流程。
                 TURRET_ACCELERATION_FAST, false, 0, cargoCode)) return false;  // 声明 0，类型和可用范围由所在声明决定。
    cargo.material = static_cast<MaterialType>(materialCode);  // 将 static_cast<MaterialType>(materialCode) 写入或初始化 cargo.material。
    Serial.printf("[Material] disc -> cargo %u, material %u placed\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                  cargoCode, materialCode);  // 续写当前声明或运算，提供 cargoCode, materialCode);。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

bool ValidateDiscPositionTransfer(uint8_t materialCode, uint8_t cargoCode) {  // 定义函数或成员 的入口 ValidateDiscPositionTransfer（Validate圆盘位置Transfer），参数由此处的签名确定。
    if (!isMaterialCodeValid(materialCode) || !isStationCodeValid(cargoCode)) {  // 当 !isMaterialCodeValid(materialCode) || !isStationCodeVa… 成立时进入此分支，否则继续后续流程。
        Serial.println("[DISC_POSITION] ERR: invalid material/cargo code");  // 调用 Serial.println，传入 "[DISC_POSITION] ERR: invalid material/cargo code"。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。
    if (cargoPlatforms[cargoCode - 1].material != MATERIAL_NONE) {  // 当 cargoPlatforms[cargoCode - 1].material != MATERIAL_NONE 成立时进入此分支，否则继续后续流程。
        Serial.println("[DISC_POSITION] ERR: cargo platform is occupied");  // 调用 Serial.println，传入 "[DISC_POSITION] ERR: cargo platform is occupied"。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。
    const MaterialStationPose &destination = materialTransferLayout.cargo;  // 将 materialTransferLayout.cargo 写入或初始化 destination。
    if (!isLayoutParameterValid() || !isPoseValid(destination)) {  // 当 !isLayoutParameterValid() || !isPoseValid(destination) 成立时进入此分支，否则继续后续流程。
        Serial.println("[DISC_POSITION] ERR: cargo pose is not calibrated");  // 调用 Serial.println，传入 "[DISC_POSITION] ERR: cargo pose is not calibrated"。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

bool PlaceHeldDiscMaterialToCargo(uint8_t materialCode, uint8_t cargoCode) {  // 定义函数或成员 的入口 PlaceHeldDiscMaterialToCargo（PlaceHeld圆盘物料To货位），参数由此处的签名确定。
    if (!ValidateDiscPositionTransfer(materialCode, cargoCode)) return false;  // 若 !ValidateDiscPositionTransfer(materialCode, cargoCode)，立即返回 false。
    CargoPlatform &cargo = cargoPlatforms[cargoCode - 1];  // 将 cargoPlatforms[cargoCode - 1] 写入或初始化 cargo（货位）。
    const MaterialStationPose &destination = materialTransferLayout.cargo;  // 将 materialTransferLayout.cargo 写入或初始化 destination。
    startCargoTray(cargoCode);  // 调用 startCargoTray（起点货位料盘），传入 cargoCode。
    if (!MoveArm(materialTransferLayout.approachHeight, 0, -1, -1,  // 当 !MoveArm(materialTransferLayout.approachHeight, 0, -1,… 成立时进入此分支，否则继续后续流程。
                 materialMotionSteps.discLiftSpeed)) return false;  // 续写当前声明或运算，提供 materialMotionSteps.discLiftSpeed)) return false;。
    waitForArm(commonMaterialDelays.pickLiftMs);  // 调用 waitForArm（waitFor机械臂），传入 commonMaterialDelays.pickLiftMs。
    if (!placeAt(destination, destination.high, commonMaterialDelays, true, nullptr,  // 当 !placeAt(destination, destination.high, commonMaterial… 成立时进入此分支，否则继续后续流程。
                 TURRET_ACCELERATION_FAST, false, 0, cargoCode)) return false;  // 声明 0，类型和可用范围由所在声明决定。
    cargo.material = static_cast<MaterialType>(materialCode);  // 将 static_cast<MaterialType>(materialCode) 写入或初始化 cargo.material。
    Serial.printf("[DISC_POSITION] material %u placed on cargo %u\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                  materialCode, cargoCode);  // 续写当前声明或运算，提供 materialCode, cargoCode);。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。
