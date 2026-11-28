// 把参数名绑定到固件实际变量，应用编译默认值并处理串口读写。
// 调参时先核对注册的数据类型、范围和对象地址。

#include "runtime_parameters.h"  // 引入 runtime_parameters.h 中的声明和定义。
#include "chassis.h"  // 引入 chassis.h 中的声明和定义。
#include "serial_frame.h"  // 引入 serial_frame.h 中的声明和定义。
#include "material_transfer.h"  // 引入 material_transfer.h 中的声明和定义。
#include "motion_executor.h"  // 引入 motion_executor.h 中的声明和定义。
#include "ring_correction.h"  // 引入 ring_correction.h 中的声明和定义。
#include "generated_parameter_defaults.h"  // 引入 generated_parameter_defaults.h 中的声明和定义。
#include <math.h>  // 引入 math.h 中的声明和定义。
#include <float.h>  // 引入 float.h 中的声明和定义。
#include <stdlib.h>  // 引入 stdlib.h 中的声明和定义。
#include <string.h>  // 引入 string.h 中的声明和定义。

extern uint32_t ALIGN_PID_MAX_SPEED_RPM;  // 声明外部定义 uint32_t ALIGN_PID_MAX_SPEED_RPM，供本编译单元引用。
extern float NODE_PATH_SPEED_MM_S;  // 声明外部定义 float NODE_PATH_SPEED_MM_S，供本编译单元引用。
extern uint32_t NODE_PATH_OUTER_ACCEL_LEVEL;  // 声明外部定义 uint32_t NODE_PATH_OUTER_ACCEL_LEVEL，供本编译单元引用。
extern float NODE_PATH_MIDDLE_SPEED_MM_S;  // 声明外部定义 float NODE_PATH_MIDDLE_SPEED_MM_S，供本编译单元引用。
extern uint32_t NODE_PATH_MIDDLE_ACCEL_LEVEL;  // 声明外部定义 uint32_t NODE_PATH_MIDDLE_ACCEL_LEVEL，供本编译单元引用。
extern float NODE_PATH_TURN_DEG_S;  // 声明外部定义 float NODE_PATH_TURN_DEG_S，供本编译单元引用。
extern float NODE_PATH_TURN_ACCEL_DEG_S2;  // 声明外部定义 float NODE_PATH_TURN_ACCEL_DEG_S2，供本编译单元引用。
extern float NODE_PATH_HEADING_DEADBAND_DEG;  // 声明外部定义 float NODE_PATH_HEADING_DEADBAND_DEG，供本编译单元引用。
extern uint32_t SCAN_APPROACH_SPEED_RPM;  // 声明外部定义 uint32_t SCAN_APPROACH_SPEED_RPM，供本编译单元引用。
extern uint32_t SCAN_SEARCH_SPEED_RPM;  // 声明外部定义 uint32_t SCAN_SEARCH_SPEED_RPM，供本编译单元引用。

namespace {  // 开始匿名命名空间，本文件内部共享其中的定义。
constexpr size_t MAX_PARAMETERS = 400; // 包含角点独立 PID 的12项参数，并预留空间。
struct Parameter {  // 定义 Parameter 的类型成员或作用域。
    char name[80];  // 声明 name，类型和可用范围由所在声明决定。
    void *address;  // 声明 address（地址），类型和可用范围由所在声明决定。
    float minimum, maximum;  // 声明 maximum，类型和可用范围由所在声明决定。
    bool integer;  // 声明 integer，类型和可用范围由所在声明决定。
};  // 结束此类型定义。
Parameter parameters[MAX_PARAMETERS];  // 声明 parameters，类型和可用范围由所在声明决定。
float staged[MAX_PARAMETERS];  // 声明 staged，类型和可用范围由所在声明决定。
bool changed[MAX_PARAMETERS];  // 声明 changed，类型和可用范围由所在声明决定。
size_t count = 0;  // 将 0 写入或初始化 count（计数）。
bool initialized = false;  // 将 false 写入或初始化 initialized。
bool registrationFailed = false;  // 将 false 写入或初始化 registrationFailed（registration失败）。
bool transaction = false;  // 将 false 写入或初始化 transaction。
uint32_t transactionId = 0;  // 将 0 写入或初始化 transactionId（transaction编号）。

float readValue(const Parameter &p) {  // 定义函数或成员 的入口 readValue（read测值），参数由此处的签名确定。
    return p.integer ? static_cast<float>(*static_cast<uint32_t *>(p.address))  // 结束当前函数并返回 p.integer ? static_cast<float>(*static_cast<uint32_t *…。
                     : *static_cast<float *>(p.address);  // 续写上一行表达式的条件取值分支。
}  // 结束此函数或控制块。

void add(const char *name, void *address, float minimum, float maximum, bool integer) {  // 定义函数或成员 的入口 add，参数由此处的签名确定。
    if (count == MAX_PARAMETERS || strlen(name) >= sizeof(parameters[0].name)) {  // 当 count == MAX_PARAMETERS || strlen(name) >= sizeof(para… 成立时进入此分支，否则继续后续流程。
        registrationFailed = true;  // 将 true 写入或初始化 registrationFailed（registration失败）。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    Parameter &p = parameters[count++];  // 将 parameters[count++] 写入或初始化 p。
    strcpy(p.name, name);  // 调用 strcpy，传入 p.name, name。
    p.address = address;  // 将 address 写入或初始化 p.address。
    p.minimum = minimum;  // 将 minimum 写入或初始化 p.minimum。
    p.maximum = maximum;  // 将 maximum 写入或初始化 p.maximum。
    p.integer = integer;  // 将 integer 写入或初始化 p.integer。
}  // 结束此函数或控制块。

void registerPose(const char *name, MaterialStationPose &pose) {  // 定义函数或成员 的入口 registerPose，参数由此处的签名确定。
    char label[80];  // 声明 label，类型和可用范围由所在声明决定。
    snprintf(label, sizeof(label), "%s/高度(mm)", name);  // 调用 snprintf，传入 label, sizeof(label), "%s/高度(mm)", name。
    RegisterRuntimeFloat(label, pose.high, 0, ARM_HEIGHT_LIMIT_MM);  // 调用 RegisterRuntimeFloat，传入 label, pose.high, 0, ARM_HEIGHT_LIMIT_MM。
    snprintf(label, sizeof(label), "%s/伸出(mm)", name);  // 调用 snprintf，传入 label, sizeof(label), "%s/伸出(mm)", name。
    RegisterRuntimeFloat(label, pose.length, 0, 170);  // 调用 RegisterRuntimeFloat，传入 label, pose.length, 0, 170。
    snprintf(label, sizeof(label), "%s/转台(deg)", name);  // 调用 snprintf，传入 label, sizeof(label), "%s/转台(deg)", name。
    RegisterRuntimeFloat(label, pose.turretAngle,  // 调用 RegisterRuntimeFloat，传入 label, pose.turretAngle,。
                         TURRET_CABLE_MIN_DEG, TURRET_CABLE_MAX_DEG);  // 续写当前声明或运算，提供 TURRET_CABLE_MIN_DEG, TURRET_CABLE_MAX_DEG);。
}  // 结束此函数或控制块。

void registerTransferDelays(const char *prefix, MaterialTransferDelays &delays) {  // 定义函数或成员 的入口 registerTransferDelays，参数由此处的签名确定。
    const char *const steps[] = {"取料平移", "取料稳定", "取料转向", "取料下降", "取料夹紧", "取料抬升",  // 续列跨行签名或声明中的参数，类型与名称为 const char *const steps[] = {"取料平移", "取料稳定", "取料转向", "…。
                                "放料平移", "放料稳定", "放料转向", "放料下降", "放料松爪", "放料抬升"};  // 续写当前声明或运算，提供 "放料平移", "放料稳定", "放料转向", "放料下降", "放料松爪", "放料抬升"};。
    uint32_t *const values[] = {&delays.pickAbove.translateMs, &delays.pickAbove.settleMs,  // 续列跨行签名或声明中的参数，类型与名称为 uint32_t *const values[] = {&delays.pickAbove.translat…。
        &delays.pickAbove.rotateMs, &delays.pickDescendMs, &delays.pickGripMs, &delays.pickLiftMs,  // 为当前列表或跨行表达式提供这一项 &delays.pickAbove.rotateMs, &delays.pickDescendMs, &de…。
        &delays.placeAbove.translateMs, &delays.placeAbove.settleMs, &delays.placeAbove.rotateMs,  // 为当前列表或跨行表达式提供这一项 &delays.placeAbove.translateMs, &delays.placeAbove.set…。
        &delays.placeDescendMs, &delays.placeReleaseMs, &delays.placeLiftMs};  // 续写当前声明或运算，提供 &delays.placeDescendMs, &delays.placeReleaseMs, &delay…。
    for (size_t i = 0; i < 12; ++i) {  // 按 size_t i = 0; i < 12; ++i 组织循环的初值、继续条件和步进。
        char name[80];  // 声明 name，类型和可用范围由所在声明决定。
        snprintf(name, sizeof(name), "%s/%s(ms)", prefix, steps[i]);  // 调用 snprintf，传入 name, sizeof(name), "%s/%s(ms)", prefix, steps[i]。
        RegisterRuntimeUInt(name, *values[i], 0, 60000);  // 调用 RegisterRuntimeUInt，传入 name, *values[i], 0, 60000。
    }  // 结束此循环体。
}  // 结束此函数或控制块。

void registerMaterialDelays() {  // 定义函数或成员 的入口 registerMaterialDelays（register物料Delays），参数由此处的签名确定。
    registerTransferDelays("动作组延时/公共", commonMaterialDelays);  // 调用 registerTransferDelays，传入 "动作组延时/公共", commonMaterialDelays。
    for (uint8_t c = 0; c < MATERIAL_STATION_COUNT; ++c) {  // 按 uint8_t c = 0; c < MATERIAL_STATION_COUNT; ++c 组织循环的初值、继续条件和步进。
        for (uint8_t w = 0; w < MATERIAL_STATION_COUNT; ++w) {  // 按 uint8_t w = 0; w < MATERIAL_STATION_COUNT; ++w 组织循环的初值、继续条件和步进。
            char prefix[80];  // 声明 prefix，类型和可用范围由所在声明决定。
            snprintf(prefix, sizeof(prefix), "动作组延时/载物台%u到工作区%u", c + 1, w + 1);  // 调用 snprintf，传入 prefix, sizeof(prefix), "动作组延时/载物台%u到工作区%u", c + 1, w …。
            registerTransferDelays(prefix, cargoToWorkAreaDelays[c][w]);  // 调用 registerTransferDelays，传入 prefix, cargoToWorkAreaDelays[c][w]。
        }  // 结束此循环体。
    }  // 结束此循环体。
    RegisterRuntimeUInt("动作组延时/圆盘/预抓(ms)", materialAuxiliaryDelays.discApproachMs, 0, 60000);  // 调用 RegisterRuntimeUInt，传入 "动作组延时/圆盘/预抓(ms)", materialAuxiliaryDelays.discApproac…。
    RegisterRuntimeUInt("动作组延时/圆盘/夹紧(ms)", materialAuxiliaryDelays.discGripMs, 0, 60000);  // 调用 RegisterRuntimeUInt，传入 "动作组延时/圆盘/夹紧(ms)", materialAuxiliaryDelays.discGripMs,…。
    RegisterRuntimeUInt("动作组延时/圆盘/抬升(ms)", materialAuxiliaryDelays.discLiftMs, 0, 60000);  // 调用 RegisterRuntimeUInt，传入 "动作组延时/圆盘/抬升(ms)", materialAuxiliaryDelays.discLiftMs,…。
    RegisterRuntimeUInt("动作组延时/扫码准备1/抬升(ms)", materialAuxiliaryDelays.lidarZone1LiftMs, 0, 60000);  // 调用 RegisterRuntimeUInt，传入 "动作组延时/扫码准备1/抬升(ms)", materialAuxiliaryDelays.lidarZon…。
    RegisterRuntimeUInt("动作组延时/扫码准备1/转向(ms)", materialAuxiliaryDelays.lidarZone1TurnMs, 0, 60000);  // 调用 RegisterRuntimeUInt，传入 "动作组延时/扫码准备1/转向(ms)", materialAuxiliaryDelays.lidarZon…。
    RegisterRuntimeUInt("动作组延时/扫码准备2/抬升(ms)", materialAuxiliaryDelays.lidarZone2LiftMs, 0, 60000);  // 调用 RegisterRuntimeUInt，传入 "动作组延时/扫码准备2/抬升(ms)", materialAuxiliaryDelays.lidarZon…。
    RegisterRuntimeUInt("动作组延时/扫码准备2/转向(ms)", materialAuxiliaryDelays.lidarZone2TurnMs, 0, 60000);  // 调用 RegisterRuntimeUInt，传入 "动作组延时/扫码准备2/转向(ms)", materialAuxiliaryDelays.lidarZon…。
}  // 结束此函数或控制块。

void registerReverseMaterialDelays() {  // 定义函数或成员 的入口 registerReverseMaterialDelays（registerReverse物料Delays），参数由此处的签名确定。
    for (uint8_t w = 0; w < MATERIAL_STATION_COUNT; ++w) {  // 按 uint8_t w = 0; w < MATERIAL_STATION_COUNT; ++w 组织循环的初值、继续条件和步进。
        for (uint8_t c = 0; c < MATERIAL_STATION_COUNT; ++c) {  // 按 uint8_t c = 0; c < MATERIAL_STATION_COUNT; ++c 组织循环的初值、继续条件和步进。
            char prefix[80];  // 声明 prefix，类型和可用范围由所在声明决定。
            snprintf(prefix, sizeof(prefix), "动作组延时/工作区%u到载物台%u", w + 1, c + 1);  // 调用 snprintf，传入 prefix, sizeof(prefix), "动作组延时/工作区%u到载物台%u", w + 1, c …。
            registerTransferDelays(prefix, workAreaToCargoDelays[w][c]);  // 调用 registerTransferDelays，传入 prefix, workAreaToCargoDelays[w][c]。
        }  // 结束此循环体。
    }  // 结束此循环体。
}  // 结束此函数或控制块。

// 所有搬运速度共用同一校验：正数且可表示为float，无人为速度上限。
void registerArmSpeed(const char *name, float &value) {  // 定义函数或成员 的入口 registerArmSpeed（register机械臂速度），参数由此处的签名确定。
    RegisterRuntimeFloat(name, value, 1, FLT_MAX);  // 调用 RegisterRuntimeFloat，传入 name, value, 1, FLT_MAX。
}  // 结束此函数或控制块。

void initialize() {  // 定义函数或成员 的入口 initialize，参数由此处的签名确定。
    if (initialized) return;  // 当 initialized 成立时进入此分支，否则继续后续流程。
    initialized = true;  // 将 true 写入或初始化 initialized。
    registerPose("搬运/圆盘", materialTransferLayout.disc);  // 调用 registerPose，传入 "搬运/圆盘", materialTransferLayout.disc。
    registerPose("搬运/待操作区", materialTransferLayout.cargo);  // 调用 registerPose，传入 "搬运/待操作区", materialTransferLayout.cargo。
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {  // 按 uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i 组织循环的初值、继续条件和步进。
        char name[64];  // 声明 name，类型和可用范围由所在声明决定。
        snprintf(name, sizeof(name), "搬运/旋转料盘%u/角度(deg)", i + 1);  // 调用 snprintf，传入 name, sizeof(name), "搬运/旋转料盘%u/角度(deg)", i + 1。
        RegisterRuntimeFloat(name, cargoTrayAngles[i], -368640, 368640);  // 调用 RegisterRuntimeFloat，传入 name, cargoTrayAngles[i], -368640, 368640。
        snprintf(name, sizeof(name), "搬运/工作区%u", i + 1);  // 调用 snprintf，传入 name, sizeof(name), "搬运/工作区%u", i + 1。
        registerPose(name, materialTransferLayout.workArea[i]);  // 调用 registerPose，传入 name, materialTransferLayout.workArea[i]。
    }  // 结束此循环体。
    RegisterRuntimeFloat("搬运/安全高度(mm)", materialTransferLayout.approachHeight, 0, ARM_HEIGHT_LIMIT_MM);  // 调用 RegisterRuntimeFloat，传入 "搬运/安全高度(mm)", materialTransferLayout.approachHeight, …。
    RegisterRuntimeFloat("搬运/二层高度增量(mm)", materialTransferLayout.secondLayerOffset, 0, ARM_HEIGHT_LIMIT_MM);  // 调用 RegisterRuntimeFloat，传入 "搬运/二层高度增量(mm)", materialTransferLayout.secondLayerOff…。
    registerArmSpeed("搬运/速度", materialTransferLayout.moveSpeed);  // 调用 registerArmSpeed（register机械臂速度），传入 "搬运/速度", materialTransferLayout.moveSpeed。
    RegisterRuntimeFloat("搬运/夹爪张开(deg)", materialTransferLayout.clawOpenAngle, -360, ARM_CLAW_MAX_DEG);  // 调用 RegisterRuntimeFloat，传入 "搬运/夹爪张开(deg)", materialTransferLayout.clawOpenAngle, …。
    RegisterRuntimeFloat("搬运/夹爪夹紧(deg)", materialTransferLayout.clawClosedAngle, -360, ARM_CLAW_MAX_DEG);  // 调用 RegisterRuntimeFloat，传入 "搬运/夹爪夹紧(deg)", materialTransferLayout.clawClosedAngle…。
    RegisterRuntimeUInt("搬运/等待时间(ms)", materialTransferLayout.motionWaitMs, 1, 60000);  // 调用 RegisterRuntimeUInt，传入 "搬运/等待时间(ms)", materialTransferLayout.motionWaitMs, 1,…。
    registerPose("搬运/圆盘预抓", materialMotionSteps.discApproach);  // 调用 registerPose，传入 "搬运/圆盘预抓", materialMotionSteps.discApproach。
    RegisterRuntimeFloat("搬运/圆盘预抓/张爪(deg)", materialMotionSteps.discOpenAngle, -360, ARM_CLAW_MAX_DEG);  // 调用 RegisterRuntimeFloat，传入 "搬运/圆盘预抓/张爪(deg)", materialMotionSteps.discOpenAngle, …。
    RegisterRuntimeFloat("搬运/圆盘提起/高度(mm)", materialMotionSteps.discLiftHeight, 0, ARM_HEIGHT_LIMIT_MM);  // 调用 RegisterRuntimeFloat，传入 "搬运/圆盘提起/高度(mm)", materialMotionSteps.discLiftHeight, …。
    registerArmSpeed("搬运/速度/圆盘预抓", materialMotionSteps.discApproachSpeed);  // 调用 registerArmSpeed（register机械臂速度），传入 "搬运/速度/圆盘预抓", materialMotionSteps.discApproachSpeed。
    registerArmSpeed("搬运/速度/圆盘夹紧", materialMotionSteps.discGripSpeed);  // 调用 registerArmSpeed（register机械臂速度），传入 "搬运/速度/圆盘夹紧", materialMotionSteps.discGripSpeed。
    registerArmSpeed("搬运/速度/圆盘提起", materialMotionSteps.discLiftSpeed);  // 调用 registerArmSpeed（register机械臂速度），传入 "搬运/速度/圆盘提起", materialMotionSteps.discLiftSpeed。
    registerArmSpeed("搬运/速度/移至工位平移", materialMotionSteps.approachTranslateSpeed);  // 调用 registerArmSpeed（register机械臂速度），传入 "搬运/速度/移至工位平移", materialMotionSteps.approachTranslateS…。
    registerArmSpeed("搬运/速度/移至工位转向", materialMotionSteps.approachRotateSpeed);  // 调用 registerArmSpeed（register机械臂速度），传入 "搬运/速度/移至工位转向", materialMotionSteps.approachRotateSpeed。
    registerArmSpeed("搬运/速度/取料下降", materialMotionSteps.pickDescendSpeed);  // 调用 registerArmSpeed（register机械臂速度），传入 "搬运/速度/取料下降", materialMotionSteps.pickDescendSpeed。
    registerArmSpeed("搬运/速度/取料夹紧", materialMotionSteps.pickGripSpeed);  // 调用 registerArmSpeed（register机械臂速度），传入 "搬运/速度/取料夹紧", materialMotionSteps.pickGripSpeed。
    registerArmSpeed("搬运/速度/取料提起", materialMotionSteps.pickLiftSpeed);  // 调用 registerArmSpeed（register机械臂速度），传入 "搬运/速度/取料提起", materialMotionSteps.pickLiftSpeed。
    registerArmSpeed("搬运/速度/放料下降", materialMotionSteps.placeDescendSpeed);  // 调用 registerArmSpeed（register机械臂速度），传入 "搬运/速度/放料下降", materialMotionSteps.placeDescendSpeed。
    registerArmSpeed("搬运/速度/放料松爪", materialMotionSteps.placeReleaseSpeed);  // 调用 registerArmSpeed（register机械臂速度），传入 "搬运/速度/放料松爪", materialMotionSteps.placeReleaseSpeed。
    registerArmSpeed("搬运/速度/放料提起", materialMotionSteps.placeLiftSpeed);  // 调用 registerArmSpeed（register机械臂速度），传入 "搬运/速度/放料提起", materialMotionSteps.placeLiftSpeed。
    RegisterChassisParameters();  // 调用 RegisterChassisParameters，使用无参形式推进相应操作。
    RegisterRuntimeFloat("底盘/节点导航速度(mm/s)", NODE_PATH_SPEED_MM_S, 10, 5000);  // 调用 RegisterRuntimeFloat，传入 "底盘/节点导航速度(mm/s)", NODE_PATH_SPEED_MM_S, 10, 5000。
    RegisterRuntimeUInt("底盘/扫码接近轮速(RPM)", SCAN_APPROACH_SPEED_RPM, 1, 5000);  // 调用 RegisterRuntimeUInt，传入 "底盘/扫码接近轮速(RPM)", SCAN_APPROACH_SPEED_RPM, 1, 5000。
    RegisterRuntimeUInt("底盘/扫码往返轮速(RPM)", SCAN_SEARCH_SPEED_RPM, 1, 5000);  // 调用 RegisterRuntimeUInt，传入 "底盘/扫码往返轮速(RPM)", SCAN_SEARCH_SPEED_RPM, 1, 5000。
    RegisterRuntimeUInt("对齐/最大轮速(RPM)", ALIGN_PID_MAX_SPEED_RPM, 1, 5000);  // 调用 RegisterRuntimeUInt，传入 "对齐/最大轮速(RPM)", ALIGN_PID_MAX_SPEED_RPM, 1, 5000。
    // 追加注册，保留原有参数索引；默认值来自 material_transfer.cpp。
    registerMaterialDelays();  // 调用 registerMaterialDelays（register物料Delays），使用无参形式推进相应操作。
    // 只追加，不改变既有参数索引。
    RegisterRuntimeUInt("底盘/节点导航外圈加速度档位(EMM V5)", NODE_PATH_OUTER_ACCEL_LEVEL, 1, 255);  // 调用 RegisterRuntimeUInt，传入 "底盘/节点导航外圈加速度档位(EMM V5)", NODE_PATH_OUTER_ACCEL_LEVEL,…。
    RegisterRuntimeFloat("底盘/节点导航转向速度(°/s)", NODE_PATH_TURN_DEG_S, 10, 500);  // 调用 RegisterRuntimeFloat，传入 "底盘/节点导航转向速度(°/s)", NODE_PATH_TURN_DEG_S, 10, 500。
    RegisterRuntimeFloat("底盘/节点导航转向加速度(°/s²)", NODE_PATH_TURN_ACCEL_DEG_S2, 30, 720);  // 调用 RegisterRuntimeFloat，传入 "底盘/节点导航转向加速度(°/s²)", NODE_PATH_TURN_ACCEL_DEG_S2, 30,…。
    RegisterRuntimeFloat("底盘/节点导航独立纠偏死区(°)", NODE_PATH_HEADING_DEADBAND_DEG, 0, 8);  // 调用 RegisterRuntimeFloat，传入 "底盘/节点导航独立纠偏死区(°)", NODE_PATH_HEADING_DEADBAND_DEG, 0,…。
    // 反向九组最后追加，保留公共、正向、辅助及底盘参数的既有编号。
    registerReverseMaterialDelays();  // 调用 registerReverseMaterialDelays（registerReverse物料Delays），使用无参形式推进相应操作。
    // 角点 PID 最后追加，保留既有参数编号。
    RegisterCornerAlignmentParameters();  // 调用 RegisterCornerAlignmentParameters，使用无参形式推进相应操作。
    // 只追加；0 保持原有到站展开与标定计时，实车验证后再逐项启用。
    RegisterRuntimeFloat("运动执行/look末段提前量(mm)",MOTION_LOOK_LEAD_MM,0,FLT_MAX);  // 调用 RegisterRuntimeFloat，传入 "运动执行/look末段提前量(mm)",MOTION_LOOK_LEAD_MM,0,FLT_MAX。
    RegisterRuntimeFloat("运动执行/look2末段提前量(mm)",MOTION_LOOK2_LEAD_MM,0,FLT_MAX);  // 调用 RegisterRuntimeFloat，传入 "运动执行/look2末段提前量(mm)",MOTION_LOOK2_LEAD_MM,0,FLT_MAX。
    RegisterRuntimeUInt("运动执行/机械臂反馈完成",ARM_FEEDBACK_ENABLED,0,1);  // 调用 RegisterRuntimeUInt，传入 "运动执行/机械臂反馈完成",ARM_FEEDBACK_ENABLED,0,1。
    RegisterRuntimeUInt("运动执行/反馈后夹持保持(ms)",ARM_GRIP_HOLD_MS,0,UINT32_MAX);  // 调用 RegisterRuntimeUInt，传入 "运动执行/反馈后夹持保持(ms)",ARM_GRIP_HOLD_MS,0,UINT32_MAX。
    RegisterRuntimeUInt("运动执行/反馈后释放保持(ms)",ARM_RELEASE_HOLD_MS,0,UINT32_MAX);  // 调用 RegisterRuntimeUInt，传入 "运动执行/反馈后释放保持(ms)",ARM_RELEASE_HOLD_MS,0,UINT32_MAX。
    // 追加独立中间道路参数，保留已有上位机参数编号。
    RegisterRuntimeFloat("底盘/节点导航中间路段速度(mm/s)",NODE_PATH_MIDDLE_SPEED_MM_S,10,5000);  // 调用 RegisterRuntimeFloat，传入 "底盘/节点导航中间路段速度(mm/s)",NODE_PATH_MIDDLE_SPEED_MM_S,10,5…。
    RegisterRuntimeUInt("底盘/节点导航中间路段加速度档位(EMM V5)",NODE_PATH_MIDDLE_ACCEL_LEVEL,1,255);  // 调用 RegisterRuntimeUInt，传入 "底盘/节点导航中间路段加速度档位(EMM V5)",NODE_PATH_MIDDLE_ACCEL_LEVE…。
    RegisterRuntimeFloat("搬运/载物台/夹爪张开(deg)", materialTransferLayout.cargoClawOpenAngle, -360, ARM_CLAW_MAX_DEG);  // 调用 RegisterRuntimeFloat，传入 "搬运/载物台/夹爪张开(deg)", materialTransferLayout.cargoClawOp…。
    // CFG 以 float 暂存参数，2^24 是可逐个精确表示的整数上界。
    RegisterRuntimeUInt("圆环纠偏/连续达标帧数",ring::requiredStableFrames(),1,16777216);  // 调用 RegisterRuntimeUInt，传入 "圆环纠偏/连续达标帧数",ring::requiredStableFrames(),1,16777216。
    RegisterRuntimeFloat("圆环纠偏/X死区(px)",ring::configuredDeadzone().x,FLT_MIN,FLT_MAX);  // 调用 RegisterRuntimeFloat，传入 "圆环纠偏/X死区(px)",ring::configuredDeadzone().x,FLT_MIN,FL…。
    RegisterRuntimeFloat("圆环纠偏/Y死区(px)",ring::configuredDeadzone().y,FLT_MIN,FLT_MAX);  // 调用 RegisterRuntimeFloat，传入 "圆环纠偏/Y死区(px)",ring::configuredDeadzone().y,FLT_MIN,FL…。
    RegisterRuntimeUInt("圆环纠偏/超时(ms)",ring::configuredTimeoutMs(),0,16777216);  // 调用 RegisterRuntimeUInt，传入 "圆环纠偏/超时(ms)",ring::configuredTimeoutMs(),0,16777216。
}  // 结束此函数或控制块。

bool parseUnsigned(const char *text, uint32_t &value) {  // 定义函数或成员 的入口 parseUnsigned，参数由此处的签名确定。
    if (!text || !*text) return false;  // 若 !text || !*text，立即返回 false。
    for (const char *p = text; *p; ++p) if (*p < '0' || *p > '9') return false;  // 按 const char *p = text; *p; ++p) if (*p < '0' || *p > '9… 组织循环的初值、继续条件和步进。
    char *end;  // 声明 end（终点），类型和可用范围由所在声明决定。
    const unsigned long long number = strtoull(text, &end, 10);  // 将 strtoull(text, &end, 10) 写入或初始化 number。
    if (*end || number > UINT32_MAX) return false;  // 若 *end || number > UINT32_MAX，立即返回 false。
    value = static_cast<uint32_t>(number);  // 将 static_cast<uint32_t>(number) 写入或初始化 value（测值）。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

void reply(const char *action, uint32_t id, const char *status, const char *detail) {  // 定义函数或成员 的入口 reply，参数由此处的签名确定。
    if (strcmp(status, "OK") == 0) {  // 当 strcmp(status, "OK") == 0 成立时进入此分支，否则继续后续流程。
        Serial.printf("{RSP,CFG,%s,OK,%lu}\n", action, static_cast<unsigned long>(id));  // 调用 Serial.printf，传入 "{RSP,CFG,%s,OK,%lu}\n", action, static_cast<unsigned …。
    } else {  // 前一个判断不成立时执行此分支。
        Serial.printf("{RSP,CFG,%s,ERR,%lu,%s}\n", action,  // 通过串口执行 printf，输出内容或参数由本行给出。
                      static_cast<unsigned long>(id), detail);  // 声明接口 long>，参数由此处的签名确定。
    }  // 结束此条件分支。
}  // 结束此函数或控制块。
} // namespace

/**
 * @brief 将浮点参数引用注册到运行时参数表，供串口读取和修改；name为上位机显示名称。
 */
void RegisterRuntimeFloat(const char *name, float &value, float minimum, float maximum) {  // 定义函数或成员 的入口 RegisterRuntimeFloat，参数由此处的签名确定。
    add(name, &value, minimum, maximum, false);  // 调用 add，传入 name, &value, minimum, maximum, false。
}  // 结束此函数或控制块。
/**
 * @brief 将无符号整型参数引用注册到运行时参数表，保留名称及可写范围信息。
 */
void RegisterRuntimeUInt(const char *name, uint32_t &value, float minimum, float maximum) {  // 定义函数或成员 的入口 RegisterRuntimeUInt，参数由此处的签名确定。
    add(name, &value, minimum, maximum, true);  // 调用 add，传入 name, &value, minimum, maximum, true。
}  // 结束此函数或控制块。

/**
 * @brief 应用由robot_parameters.json生成并编译进固件的默认值。
 * @return true 所有默认值应用成功，false 存在参数应用错误。
 */
// 默认参数表由构建脚本生成，运行时按注册项匹配名字再应用。
// 字段新增或改名时，检查注册入口和构建输入是否使用相同名称。
bool ApplyCompiledParameterDefaults() {  // 定义函数或成员 的入口 ApplyCompiledParameterDefaults，参数由此处的签名确定。
    initialize();  // 调用 initialize，使用无参形式推进相应操作。
    if (registrationFailed) {  // 当 registrationFailed 成立时进入此分支，否则继续后续流程。
        Serial.println("[CFG] ERR: parameter registry overflow");  // 调用 Serial.println，传入 "[CFG] ERR: parameter registry overflow"。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。
    // 先检查全部条目，再写入任何变量；避免错误文件造成部分参数生效。
    for (size_t j = 0; j < COMPILED_PARAMETER_DEFAULT_COUNT; ++j) {  // 按 size_t j = 0; j < COMPILED_PARAMETER_DEFAULT_COUNT; ++j 组织循环的初值、继续条件和步进。
        const auto &entry = COMPILED_PARAMETER_DEFAULTS[j];  // 将 COMPILED_PARAMETER_DEFAULTS[j] 写入或初始化 entry。
        size_t i = 0;  // 将 0 写入或初始化 i。
        while (i < count && strcmp(parameters[i].name, entry.name) != 0) ++i;  // 当 i < count && strcmp(parameters[i].name, entry.name) !=… 成立时继续循环。
        if (i == count || !isfinite(entry.value)  // 当 i == count || !isfinite(entry.value 成立时进入此分支，否则继续后续流程。
                || entry.value < parameters[i].minimum  // 继续上一行的组合条件，本行提供 || entry.value < parameters[i].minimum。
                || entry.value > parameters[i].maximum  // 继续上一行的组合条件，本行提供 || entry.value > parameters[i].maximum。
                || (parameters[i].integer && (floor(entry.value) != entry.value  // 继续上一行的组合条件，本行提供 || (parameters[i].integer && (floor(entry.value) != entry.value。
                    || entry.value < 0 || entry.value > UINT32_MAX))) {  // 继续上一行的组合条件，本行提供 || entry.value < 0 || entry.value > UINT32_MAX))) {。
            Serial.printf("[CFG] ERR: invalid compiled parameter: %s\n", entry.name);  // 调用 Serial.printf，传入 "[CFG] ERR: invalid compiled parameter: %s\n", entry.n…。
            return false;  // 结束当前函数并返回 false。
        }  // 结束此函数或控制块。
    }  // 结束此循环体。
    for (size_t j = 0; j < COMPILED_PARAMETER_DEFAULT_COUNT; ++j) {  // 按 size_t j = 0; j < COMPILED_PARAMETER_DEFAULT_COUNT; ++j 组织循环的初值、继续条件和步进。
        const auto &entry = COMPILED_PARAMETER_DEFAULTS[j];  // 将 COMPILED_PARAMETER_DEFAULTS[j] 写入或初始化 entry。
        for (size_t i = 0; i < count; ++i) {  // 按 size_t i = 0; i < count; ++i 组织循环的初值、继续条件和步进。
            Parameter &p = parameters[i];  // 将 parameters[i] 写入或初始化 p。
            if (strcmp(p.name, entry.name) != 0) continue;  // 若 strcmp(p.name, entry.name) != 0，直接进入下一轮循环。
            if (p.integer) *static_cast<uint32_t *>(p.address) = static_cast<uint32_t>(entry.value);  // 当 p.integer 成立时进入此分支，否则继续后续流程。
            else *static_cast<float *>(p.address) = static_cast<float>(entry.value);  // 前一个判断不成立时执行此分支。
            break;  // 退出当前循环或 switch 分支。
        }  // 结束此循环体。
    }  // 结束此循环体。
    Serial.printf("[CFG] Applied %u compiled defaults from robot_parameters.json\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                  static_cast<unsigned>(COMPILED_PARAMETER_DEFAULT_COUNT));  // 声明接口 unsigned>，参数由此处的签名确定。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

/**
 * @brief 处理CFG参数读取、写入等串口动作；由串口层提供已解析字段和可写状态。
 * @return true 已处理对应请求，false 请求无法处理。
 */
bool HandleRuntimeParameters(const char *const *fields, size_t size, bool writable) {  // 定义函数或成员 的入口 HandleRuntimeParameters，参数由此处的签名确定。
    initialize();  // 调用 initialize，使用无参形式推进相应操作。
    const char *action = size > 0 ? fields[0] : "FRAME";  // 将 size > 0 ? fields[0] : "FRAME" 写入或初始化 action。
    uint32_t id = 0;  // 将 0 写入或初始化 id（编号）。
    if (size < 2 || size > 4 || !parseUnsigned(fields[1], id) || id == 0) {  // 当 size < 2 || size > 4 || !parseUnsigned(fields[1], id) … 成立时进入此分支，否则继续后续流程。
        reply(action, 0, "ERR", "FORMAT");  // 调用 reply，传入 action, 0, "ERR", "FORMAT"。
        return true;  // 结束当前函数并返回 true。
    }  // 结束此条件分支。
    const bool correctSize = (strcmp(action, "GET") == 0 && (size == 2 || size == 3))  // 将 (strcmp(action, "GET") == 0 && (size == 2 || size == 3… 写入或初始化 correctSize（correct大小）。
        || (strcmp(action, "BEGIN") == 0 && size == 2)  // 继续上一行的组合条件，本行提供 || (strcmp(action, "BEGIN") == 0 && size == 2)。
        || (strcmp(action, "SET") == 0 && size == 4)  // 继续上一行的组合条件，本行提供 || (strcmp(action, "SET") == 0 && size == 4)。
        || (strcmp(action, "COMMIT") == 0 && size == 2);  // 继续上一行的组合条件，本行提供 || (strcmp(action, "COMMIT") == 0 && size == 2);。
    if (!correctSize) {  // 当 !correctSize 成立时进入此分支，否则继续后续流程。
        reply(action, id, "ERR", "FORMAT");  // 调用 reply，传入 action, id, "ERR", "FORMAT"。
        return true;  // 结束当前函数并返回 true。
    }  // 结束此条件分支。
    if (registrationFailed) { reply(action, id, "ERR", "REGISTRY"); return true; }  // 当 registrationFailed 成立时进入此分支，否则继续后续流程。
    if (strcmp(action, "GET") == 0) {  // 当 strcmp(action, "GET") == 0 成立时进入此分支，否则继续后续流程。
        transaction = false;  // 将 false 写入或初始化 transaction。
        size_t first = 0, end = count;  // 将 0, end = count 写入或初始化 first（首个）。
        if (size == 3) {  // 当 size == 3 成立时进入此分支，否则继续后续流程。
            uint32_t index = 0;  // 将 0 写入或初始化 index（下标）。
            if (parseUnsigned(fields[2], index)) {  // 当 parseUnsigned(fields[2], index) 成立时进入此分支，否则继续后续流程。
                first = index;  // 将 index 写入或初始化 first（首个）。
            } else {  // 前一个判断不成立时执行此分支。
                const void *address = nullptr;  // 将 nullptr 写入或初始化 address（地址）。
                if (strcmp(fields[2], "RING_X") == 0) address = &ring::configuredDeadzone().x;  // 当 strcmp(fields[2], "RING_X") == 0 成立时进入此分支，否则继续后续流程。
                if (strcmp(fields[2], "RING_Y") == 0) address = &ring::configuredDeadzone().y;  // 当 strcmp(fields[2], "RING_Y") == 0 成立时进入此分支，否则继续后续流程。
                if (strcmp(fields[2], "RING_TIMEOUT") == 0) address = &ring::configuredTimeoutMs();  // 当 strcmp(fields[2], "RING_TIMEOUT") == 0 成立时进入此分支，否则继续后续流程。
                first = 0;  // 将 0 写入或初始化 first（首个）。
                while (address && first < count && parameters[first].address != address) ++first;  // 当 address && first < count && parameters[first].address … 成立时继续循环。
                if (!address) first = count;  // 当 !address 成立时进入此分支，否则继续后续流程。
            }  // 结束此条件分支。
            if (first >= count) { reply(action, id, "ERR", "RANGE"); return true; }  // 当 first >= count 成立时进入此分支，否则继续后续流程。
            end = first + 1;  // 将 first + 1 写入或初始化 end（终点）。
        }  // 结束此条件分支。
        for (size_t i = first; i < end; ++i) {  // 按 size_t i = first; i < end; ++i 组织循环的初值、继续条件和步进。
            const Parameter &p = parameters[i];  // 将 parameters[i] 写入或初始化 p。
            // 参数值保持便于阅读的9位精度；范围使用足够精度，避免上位机导出时
            // 将 FLT_MAX 舍入成略大的数。uint32上限在float表中会舍入到2^32，回传时恢复准确边界。
            const double minimum = static_cast<double>(p.minimum);  // 将 static_cast<double>(p.minimum) 写入或初始化 minimum。
            const double maximum = p.integer && static_cast<double>(p.maximum) > static_cast<double>(UINT32_MAX)  // 将 p.integer && static_cast<double>(p.maximum) > static_c… 写入或初始化 maximum。
                ? static_cast<double>(UINT32_MAX) : static_cast<double>(p.maximum);  // 声明接口 double>，参数由此处的签名确定。
            Serial.printf("{RSP,CFG,GET,VALUE,%lu,%u,%s,%.9g,%.17g,%.17g,%u}\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
                static_cast<unsigned long>(id), static_cast<unsigned>(i), p.name,  // 定义函数或成员 long>，参数由此处的签名确定。
                readValue(p), minimum, maximum, p.integer ? 1 : 0);  // 调用 readValue（read测值），传入 p), minimum, maximum, p.integer ? 1 : 0。
            // 逐帧排空串口，避免扩充后的参数表连续发送时被 USB/UART 桥丢弃。
            Serial.flush();  // 调用 Serial.flush。
            delay(2);  // 调用 delay（延时），传入 2。
        }  // 结束此循环体。
        Serial.printf("{RSP,CFG,GET,END,%lu,%u}\n", static_cast<unsigned long>(id), static_cast<unsigned>(end-first));  // 调用 Serial.printf，传入 "{RSP,CFG,GET,END,%lu,%u}\n", static_cast<unsigned lon…。
        return true;  // 结束当前函数并返回 true。
    }  // 结束此条件分支。
    if (!writable) {  // 当 !writable 成立时进入此分支，否则继续后续流程。
        transaction = false;  // 将 false 写入或初始化 transaction。
        reply(action, id, "ERR", "BUSY_OR_MODE");  // 调用 reply，传入 action, id, "ERR", "BUSY_OR_MODE"。
        return true;  // 结束当前函数并返回 true。
    }  // 结束此条件分支。
    if (strcmp(fields[0], "BEGIN") == 0 && size == 2) {  // 当 strcmp(fields[0], "BEGIN") == 0 && size == 2 成立时进入此分支，否则继续后续流程。
        for (size_t i = 0; i < count; ++i) { staged[i] = readValue(parameters[i]); changed[i] = false; }  // 按 size_t i = 0; i < count; ++i 组织循环的初值、继续条件和步进。
        transaction = true;  // 将 true 写入或初始化 transaction。
        transactionId = id;  // 将 id 写入或初始化 transactionId（transaction编号）。
        reply(action, id, "OK", "BEGIN");  // 调用 reply，传入 action, id, "OK", "BEGIN"。
    } else if (!transaction || transactionId != id) {  // 仅在先前条件均不成立时，当 !transaction || transactionId != id 成立时进入此分支，否则继续后续流程。
        reply(action, id, "ERR", "TRANSACTION");  // 调用 reply，传入 action, id, "ERR", "TRANSACTION"。
    } else if (strcmp(fields[0], "SET") == 0 && size == 4) {  // 仅在先前条件均不成立时，当 strcmp(fields[0], "SET") == 0 && size == 4 成立时进入此分支，否则继续后续流程。
        uint32_t index = 0;  // 将 0 写入或初始化 index（下标）。
        float value = 0;  // 将 0 写入或初始化 value（测值）。
        if (!parseUnsigned(fields[2], index) || index >= count  // 当 !parseUnsigned(fields[2], index) || index >= count 成立时进入此分支，否则继续后续流程。
                || !serialFloat(fields[3], value, parameters[index].minimum,  // 继续上一行的组合条件，本行提供 || !serialFloat(fields[3], value, parameters[index].minimum,。
                                parameters[index].maximum)  // 续写当前声明或运算，提供 parameters[index].maximum)。
                || (parameters[index].integer && floorf(value) != value)) {  // 继续上一行的组合条件，本行提供 || (parameters[index].integer && floorf(value) != value)) {。
            transaction = false;  // 将 false 写入或初始化 transaction。
            reply(action, id, "ERR", "RANGE");  // 调用 reply，传入 action, id, "ERR", "RANGE"。
        } else {  // 前一个判断不成立时执行此分支。
            staged[index] = value;  // 将 value 写入或初始化 staged[index]。
            changed[index] = true;  // 将 true 写入或初始化 changed[index]。
            Serial.printf("{RSP,CFG,SET,OK,%lu,%lu}\n", static_cast<unsigned long>(id), static_cast<unsigned long>(index));  // 调用 Serial.printf，传入 "{RSP,CFG,SET,OK,%lu,%lu}\n", static_cast<unsigned lon…。
        }  // 结束此条件分支。
    } else if (strcmp(fields[0], "COMMIT") == 0 && size == 2) {  // 仅在先前条件均不成立时，当 strcmp(fields[0], "COMMIT") == 0 && size == 2 成立时进入此分支，否则继续后续流程。
        // 本函数在串口任务中运行且持有对齐锁；提交期间没有机械动作。
        for (size_t i = 0; i < count; ++i) {  // 按 size_t i = 0; i < count; ++i 组织循环的初值、继续条件和步进。
            if (!changed[i]) continue;  // 若 !changed[i]，直接进入下一轮循环。
            Parameter &p = parameters[i];  // 将 parameters[i] 写入或初始化 p。
            if (p.integer) *static_cast<uint32_t *>(p.address) = static_cast<uint32_t>(staged[i]);  // 当 p.integer 成立时进入此分支，否则继续后续流程。
            else *static_cast<float *>(p.address) = staged[i];  // 前一个判断不成立时执行此分支。
        }  // 结束此循环体。
        ResetDiscAlignmentPid();  // 调用 ResetDiscAlignmentPid（Reset圆盘AlignmentPID），使用无参形式推进相应操作。
        transaction = false;  // 将 false 写入或初始化 transaction。
        reply(action, id, "OK", "COMMIT");  // 调用 reply，传入 action, id, "OK", "COMMIT"。
    } else {  // 前一个判断不成立时执行此分支。
        transaction = false;  // 将 false 写入或初始化 transaction。
        reply(action, id, "ERR", "FORMAT");  // 调用 reply，传入 action, id, "ERR", "FORMAT"。
    }  // 结束此条件分支。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。
