// 物料位置、位姿、速度与等待时间的共享定义。
// 传统搬运入口和新动作编排共用这些数据，定位参数来源时可与 runtime_parameters 对照。

#ifndef MATERIAL_TRANSFER_H  // 宏 MATERIAL_TRANSFER_H 未定义时纳入此分支。
#define MATERIAL_TRANSFER_H  // 为后续文本展开定义宏 MATERIAL_TRANSFER_H。

#include <Arduino.h>  // 引入 Arduino.h 中的声明和定义。

#include "chassis.h"  // 引入 chassis.h 中的声明和定义。

// ================= 物料编号 =================
// 数值与二维码任务码中的颜色编号完全一致。
// 例如任务码中的 1 表示红色物料，6 表示浅蓝色物料。
// 0 不属于任务码，用来表示载物台当前没有物料。
enum MaterialType : uint8_t {  // 定义 MaterialType（物料Type） 的类型成员或作用域。
    MATERIAL_NONE = 0,  // 将 0, 写入或初始化 MATERIAL_NONE（物料NONE）。
    MATERIAL_RED = 1,  // 将 1, 写入或初始化 MATERIAL_RED（物料RED）。
    MATERIAL_YELLOW = 2,  // 将 2, 写入或初始化 MATERIAL_YELLOW（物料YELLOW）。
    MATERIAL_BLUE = 3,  // 将 3, 写入或初始化 MATERIAL_BLUE（物料BLUE）。
    MATERIAL_GREEN = 4,  // 将 4, 写入或初始化 MATERIAL_GREEN（物料GREEN）。
    MATERIAL_BLACK = 5,  // 将 5, 写入或初始化 MATERIAL_BLACK（物料BLACK）。
    MATERIAL_LIGHT_BLUE = 6  // 将 6 写入或初始化 MATERIAL_LIGHT_BLUE（物料LIGHTBLUE）。
};  // 结束此类型定义。

struct CargoPlatform {  // 定义 CargoPlatform（货位Platform） 的类型成员或作用域。
    MaterialType material; // 这个载物台当前放置的物料；MATERIAL_NONE 表示空载
};  // 结束此类型定义。

// ================= 单个工位的机械臂位姿 =================
// 这里保存的是小车停稳后，机械臂相对于车体抓取或放置物料时的位置。
// 夹爪的张开、夹紧角度不放在这里，由搬运流程统一控制。
constexpr float MaterialPickHeight(float height, uint8_t color) {
    return height + (color == MATERIAL_BLACK ? 5.0f : 0.0f);
}

constexpr float MaterialStackHeight(float height, uint8_t color) {
    return height + (color == MATERIAL_BLACK ? 12.0f : 0.0f);
}

struct MaterialStationPose {  // 定义 MaterialStationPose（物料StationPose） 的类型成员或作用域。
    float high;          // 升降高度 (mm)
    float length;        // 伸出长度 (mm)
    float turretAngle;   // 转台角度 (deg)
};  // 结束此类型定义。

constexpr uint8_t MATERIAL_STATION_COUNT = 3;  // 将 3 写入或初始化 MATERIAL_STATION_COUNT（物料STATION计数）。

// ================= 全部固定工位和动作参数 =================
// cargo 是车上三个小料盘共用的待操作区。
// workArea[0]、workArea[1]、workArea[2] 分别是区域内 1、2、3 号物料位。
//
// 粗加工区和暂存区虽然位于场地中的不同位置，但小车停好以后，三个物料位
// 相对于车体的位置相同。因此二者共用 workArea，不需要分别标定两套位姿。
struct MaterialTransferLayout {  // 定义 MaterialTransferLayout（物料TransferLayout） 的类型成员或作用域。
    MaterialStationPose disc;                         // 圆盘取料位
    MaterialStationPose cargo;  // 声明 cargo（货位），类型和可用范围由所在声明决定。
    MaterialStationPose workArea[MATERIAL_STATION_COUNT];  // 声明 workArea（工位Area），类型和可用范围由所在声明决定。
    float approachHeight;        // 横向运动前的安全高度 (mm)
    float secondLayerOffset;     // 第二层相对第一层的高度增量 (mm)
    float moveSpeed;             // 传给 MoveArm 的运动速度
    float clawOpenAngle;         // 夹爪张开角度
    float cargoClawOpenAngle;    // 旋转料盘取料前和装料后夹爪张开角度
    float clawClosedAngle;       // 夹爪夹紧角度
    uint32_t motionWaitMs;       // waitForArm() 不传参数时的默认等待毫秒数
};  // 结束此类型定义。

// 每个 MoveArm 阶段独立调速；目标位置仍由上面的工位位姿决定。
struct MaterialMotionSteps {  // 定义 MaterialMotionSteps（物料MotionSteps） 的类型成员或作用域。
    MaterialStationPose discApproach;  // 声明 discApproach（圆盘Approach），类型和可用范围由所在声明决定。
    float discOpenAngle;  // 声明 discOpenAngle（圆盘Open角度），类型和可用范围由所在声明决定。
    float discLiftHeight;  // 声明 discLiftHeight（圆盘升降高度），类型和可用范围由所在声明决定。
    float discApproachSpeed;  // 声明 discApproachSpeed（圆盘Approach速度），类型和可用范围由所在声明决定。
    float discGripSpeed;  // 声明 discGripSpeed（圆盘Grip速度），类型和可用范围由所在声明决定。
    float discLiftSpeed;  // 声明 discLiftSpeed（圆盘升降速度），类型和可用范围由所在声明决定。
    float approachTranslateSpeed;  // 声明 approachTranslateSpeed（approachTranslate速度），类型和可用范围由所在声明决定。
    float approachRotateSpeed;  // 声明 approachRotateSpeed（approachRotate速度），类型和可用范围由所在声明决定。
    float pickDescendSpeed;  // 声明 pickDescendSpeed（pickDescend速度），类型和可用范围由所在声明决定。
    float pickGripSpeed;  // 声明 pickGripSpeed（pickGrip速度），类型和可用范围由所在声明决定。
    float pickLiftSpeed;  // 声明 pickLiftSpeed（pick升降速度），类型和可用范围由所在声明决定。
    float placeDescendSpeed;  // 声明 placeDescendSpeed（placeDescend速度），类型和可用范围由所在声明决定。
    float placeReleaseSpeed;  // 声明 placeReleaseSpeed（placeRelease速度），类型和可用范围由所在声明决定。
    float placeLiftSpeed;  // 声明 placeLiftSpeed（place升降速度），类型和可用范围由所在声明决定。
};  // 结束此类型定义。

extern MaterialMotionSteps materialMotionSteps;  // 声明外部定义 MaterialMotionSteps materialMotionSteps，供本编译单元引用。
extern float cargoTrayAngles[MATERIAL_STATION_COUNT];  // 声明外部定义 float cargoTrayAngles[MATERIAL_STATION_COUNT]，供本编译单元引用。
constexpr float CARGO_TRAY_CLEARANCE_DEG = 30.0f;  // 将 30.0f 写入或初始化 CARGO_TRAY_CLEARANCE_DEG（货位料盘CLEARANCE角度）。
constexpr uint32_t CARGO_RETRACT_BEFORE_TURN_MS = 100;  // 将 100 写入或初始化 CARGO_RETRACT_BEFORE_TURN_MS（货位RETRACTBEFORETURN毫秒）。
constexpr float CARGO_TRAY_RELEASE_OFFSET_MM = 3.0f;  // 将 3.0f 写入或初始化 CARGO_TRAY_RELEASE_OFFSET_MM（货位料盘RELEASE偏移毫米）。
constexpr float CARGO_TRAY_ARRIVAL_TOLERANCE_DEG = 1.0f;  // 将 1.0f 写入或初始化 CARGO_TRAY_ARRIVAL_TOLERANCE_DEG（货位料盘ARRIVAL容差角度）。
constexpr float WORK_POSITION_3_TRAY_LIMIT_DEG = 240.0f;  // 将 240.0f 写入或初始化 WORK_POSITION_3_TRAY_LIMIT_DEG（工位位置3料盘限值角度）。
constexpr float WORK_POSITION_3_TRAY_TARGET_DEG =  // 续写当前声明或运算，提供 constexpr float WORK_POSITION_3_TRAY_TARGET_DEG =。
    WORK_POSITION_3_TRAY_LIMIT_DEG - CARGO_TRAY_ARRIVAL_TOLERANCE_DEG;  // 续写当前声明或运算，提供 WORK_POSITION_3_TRAY_LIMIT_DEG - CARGO_TRAY_ARRIVAL_TO…。

// 每一项均为毫秒，0 表示跳过该处等待；只控制衔接，不改变 MoveArm 速度。
struct MaterialApproachDelays {  // 定义 MaterialApproachDelays（物料ApproachDelays） 的类型成员或作用域。
    uint32_t translateMs, settleMs, rotateMs;  // 声明 rotateMs（rotate毫秒），类型和可用范围由所在声明决定。
};  // 结束此类型定义。

struct MaterialTransferDelays {  // 定义 MaterialTransferDelays（物料TransferDelays） 的类型成员或作用域。
    MaterialApproachDelays pickAbove;  // 声明 pickAbove，类型和可用范围由所在声明决定。
    uint32_t pickDescendMs, pickGripMs, pickLiftMs;  // 声明 pickLiftMs（pick升降毫秒），类型和可用范围由所在声明决定。
    MaterialApproachDelays placeAbove;  // 声明 placeAbove，类型和可用范围由所在声明决定。
    uint32_t placeDescendMs, placeReleaseMs, placeLiftMs;  // 声明 placeLiftMs（place升降毫秒），类型和可用范围由所在声明决定。
};  // 结束此类型定义。

struct MaterialAuxiliaryDelays {  // 定义 MaterialAuxiliaryDelays（物料AuxiliaryDelays） 的类型成员或作用域。
    uint32_t discApproachMs, discGripMs, discLiftMs;  // 声明 discLiftMs（圆盘升降毫秒），类型和可用范围由所在声明决定。
    uint32_t lidarZone1LiftMs, lidarZone1TurnMs;  // 声明 lidarZone1TurnMs（lidarZone1Turn毫秒），类型和可用范围由所在声明决定。
    uint32_t lidarZone2LiftMs, lidarZone2TurnMs;  // 声明 lidarZone2TurnMs（lidarZone2Turn毫秒），类型和可用范围由所在声明决定。
};  // 结束此类型定义。

// 正反向各9组独立延时，分别按 [载物台-1][工作区-1]、[工作区-1][载物台-1] 索引。
// 圆盘流程继续使用公共配置。
extern MaterialTransferDelays commonMaterialDelays;  // 声明外部定义 MaterialTransferDelays commonMaterialDelays，供本编译单元引用。
extern MaterialTransferDelays cargoToWorkAreaDelays[MATERIAL_STATION_COUNT][MATERIAL_STATION_COUNT];  // 声明外部定义 MaterialTransferDelays cargoToWorkAreaDelays[MATERIAL_…，供本编译单元引用。
extern MaterialTransferDelays workAreaToCargoDelays[MATERIAL_STATION_COUNT][MATERIAL_STATION_COUNT];  // 声明外部定义 MaterialTransferDelays workAreaToCargoDelays[MATERIAL_…，供本编译单元引用。
extern MaterialAuxiliaryDelays materialAuxiliaryDelays;  // 声明外部定义 MaterialAuxiliaryDelays materialAuxiliaryDelays，供本编译单元引用。

// materialTransferLayout：所有固定位置和动作参数，标定值在 cpp 文件顶部填写。
// cargoPlatforms：三个载物台当前的物料状态，下标 0~2 对应载物台编号 1~3。
// 契约：调用前已完成标定；任务码已在扫码/串口入口校验。
extern MaterialTransferLayout materialTransferLayout;  // 声明外部定义 MaterialTransferLayout materialTransferLayout，供本编译单元引用。
extern CargoPlatform cargoPlatforms[MATERIAL_STATION_COUNT];  // 声明外部定义 CargoPlatform cargoPlatforms[MATERIAL_STATION_COUNT]，供本编译单元引用。
inline bool CargoTrayNeedsClearance(uint8_t sourceCargo) {  // 定义函数或成员 的入口 CargoTrayNeedsClearance（货位料盘NeedsClearance），参数由此处的签名确定。
    return sourceCargo > 1 && cargoPlatforms[sourceCargo - 2].material != MATERIAL_NONE;  // 结束当前函数并返回 sourceCargo > 1 && cargoPlatforms[sourceCargo - 2].mat…。
}  // 结束此函数或控制块。
inline float CargoTrayClearanceAngle(uint8_t sourceCargo) {  // 定义函数或成员 的入口 CargoTrayClearanceAngle（货位料盘Clearance角度），参数由此处的签名确定。
    return cargoTrayAngles[sourceCargo - 1]  // 结束当前函数并返回 cargoTrayAngles[sourceCargo - 1]。
        + (CargoTrayNeedsClearance(sourceCargo) ? CARGO_TRAY_CLEARANCE_DEG : 0.0f);  // 调用 CargoTrayNeedsClearance（货位料盘NeedsClearance），传入 sourceCargo) ? CARGO_TRAY_CLEARANCE_DEG : 0.0f。
}  // 结束此函数或控制块。
inline bool CargoTrayNeedsWork3Limit(uint8_t sourceCargo) {  // 定义函数或成员 的入口 CargoTrayNeedsWork3Limit（货位料盘Needs工位3限值），参数由此处的签名确定。
    return sourceCargo && cargoPlatforms[0].material != MATERIAL_NONE  // 结束当前函数并返回 sourceCargo && cargoPlatforms[0].material != MATERIAL_…。
        && CargoTrayClearanceAngle(sourceCargo) > WORK_POSITION_3_TRAY_TARGET_DEG;  // 继续上一行的组合条件，本行提供 && CargoTrayClearanceAngle(sourceCargo) > WORK_POSITION_3_TRAY_TARGET_DEG;。
}  // 结束此函数或控制块。

// ================= 四个单次搬运动作 =================
// 所有载物台编号和区域位置编号都从 1 开始，而不是从 0 开始。

// 从圆盘抓取一种物料，然后放到指定载物台。
// materialCode：物料颜色编号 1~6；cargoCode：载物台编号 1~3。
/**
 * @brief 同步执行圆盘取料并放入指定载物台；materialCode为颜色，cargoCode为1~3号载物台，返回作业是否完成。
 */
bool MoveDiscToCargo(uint8_t materialCode, uint8_t cargoCode);  // 声明接口 MoveDiscToCargo（Move圆盘To货位），参数由此处的签名确定。

// 从指定载物台抓取物料，然后放到粗加工区或暂存区的指定位置（第一层）。
// cargoCode：载物台编号 1 ~ 3；workAreaCode：区域位置编号 1 ~ 3。
/**
 * @brief 同步执行载物台到工位的第一层放料；cargoCode和workAreaCode均为1~3，实际序列由BuildArmProgram构建。
 */
bool MoveCargoToWorkArea(uint8_t cargoCode, uint8_t workAreaCode);  // 声明接口 MoveCargoToWorkArea（Move货位To工位Area），参数由此处的签名确定。

// 调试示例：在三个载物台手动放好物料并将小车停在粗加工区后调用。
// 每次调用依次完整搬运三次：载物台 1 -> 区域 1、2 -> 2、3 -> 3。
// 临时颜色码在函数内指定，也可在 Debug 模式发送 {CMD,ARM,DEMO1} 调用。
/**
 * @brief 同步演示三件载物台到工位的放料动作，按1到1、2到2、3到3执行；不负责底盘导航。
 */
bool DemoCargoToRoughArea();  // 声明接口 DemoCargoToRoughArea（Demo货位ToRoughArea），参数由此处的签名确定。

// 取回调试示例：小车停在暂存区、三个区域位置各有一件物料且三个载物台为空时调用。
// 由内到外依次放置：区域 1 -> 载物台 3、区域 2 -> 载物台 2、区域 3 -> 载物台 1。
// Debug 模式发送 {CMD,ARM,DEMO4}；本函数不负责底盘导航或检测区域内是否真的有物料。
/**
 * @brief 同步演示三件工位取回，按工位1到载物台3、2到2、3到1执行；不负责导航。
 */
bool DemoWorkAreaToCargo();  // 声明接口 DemoWorkAreaToCargo（Demo工位AreaTo货位），参数由此处的签名确定。

// 码放调试示例：手动装好三个载物台并将小车停在码放区后，发送 {CMD,ARM,DEMO2}。
// 顺序为 1 -> 1、2 -> 2、3 -> 3；松手高度为区域基础高度 + secondLayerOffset。
// 调用方保证目标高度在机械行程内；本函数不负责底盘导航。
/**
 * @brief 同步演示第二层叠放，目标高度为工位高度加一倍层高；不负责导航。
 */
bool DemoStackCargoToWorkArea();  // 声明接口 DemoStackCargoToWorkArea（DemoStack货位To工位Area），参数由此处的签名确定。

// 码放第三层调试示例：手动装好三个载物台并将小车停在码放区后，发送 {CMD,ARM,DEMO3}。
// 顺序为 1 -> 1、2 -> 2、3 -> 3；松手高度为区域基础高度 + 2 倍 secondLayerOffset。
// 调用方保证目标高度在机械行程内；本函数不负责底盘导航。
/**
 * @brief 同步演示第三层叠放，目标高度为工位高度加两倍层高；不负责导航。
 */
bool DemoStackCargoToWorkArea3();  // 声明接口 DemoStackCargoToWorkArea3（DemoStack货位To工位Area3），参数由此处的签名确定。

// ================= 开局雷达扫描位姿 =================
// 根据当前启停区执行不同的底盘移动，再执行若干机械臂动作。
// startZoneCode：1 表示启停区 1，2 表示启停区 2；其他值会直接返回 false。
// 具体距离和机械臂动作在 material_transfer.cpp 的函数内填写。
/**
 * @brief 按启停区同步执行雷达扫描前的底盘平移和转向。
 */
bool PrepareLidarScanPose(uint8_t startZoneCode);  // 声明接口 PrepareLidarScanPose（PrepareLidar扫码Pose），参数由此处的签名确定。

// 从粗加工区或暂存区的指定位置抓取物料，然后放到指定载物台。
// materialCode 用于恢复载物台中的物料状态记录。
/**
 * @brief 同步执行工位取回到载物台；materialCode用于完成后恢复物料登记，工位和载物台编号为1~3。
 */
bool MoveWorkAreaToCargo(uint8_t materialCode, uint8_t workAreaCode, uint8_t cargoCode);  // 声明接口 MoveWorkAreaToCargo（Move工位AreaTo货位），参数由此处的签名确定。

// 从指定载物台抓取物料，然后码放到区域指定位置的第二层。
/**
 * @brief 同步执行载物台到指定工位的第二层叠放，返回作业是否完成。
 */
bool StackCargoToWorkArea(uint8_t cargoCode, uint8_t workAreaCode);  // 声明接口 StackCargoToWorkArea（Stack货位To工位Area），参数由此处的签名确定。

// ================= 三个物料的整组搬运动作 =================
// 下方批量接口都循环执行三次，可以直接传入扫码得到的 TaskCode 数组。

// 按任务码中的一组位置码放置三个载物台上的物料。
// positionCodes[0..2]：按抓取顺序对应 3、2、1 号载物台的区域位置。
// layer：1=基础高度，2=基础高度+1 倍层高，3=基础高度+2 倍层高。
// 例如第一批 PlaceTaskCargoToWorkArea(currentTask.round1_pos, 1)，
// 第二批 PlaceTaskCargoToWorkArea(currentTask.round2_pos, 2)。
// 调用前必须已装料并停在目标区域；本函数不负责底盘导航。
// 调用方保证目标高度在机械行程内；执行失败返回 false。
/**
 * @brief 按本轮位置码同步放置三件物料，实际序列由BuildArmProgram构建。
 * @param positionCodes 三个载物台各自对应的工位编号1~3。
 * @param layer 叠放层数，1为基础层，2为第二层。
 * @return true 作业DONE，false 提交或执行失败。
 */
bool PlaceTaskCargoToWorkArea(const int positionCodes[MATERIAL_STATION_COUNT], uint8_t layer);  // 声明接口 PlaceTaskCargoToWorkArea（Place任务货位To工位Area），参数由此处的签名确定。

// 按颜色码依次从圆盘抓取三个物料，分别放到 3、2、1 号载物台。
// 示例：LoadRoundFromDisc(currentTask.round1_colors);
/**
 * @brief 按颜色数组顺序同步抓取圆盘三件物料，分别放到载物台3、2、1；不负责底盘导航。
 */
bool LoadRoundFromDisc(const int materialCodes[MATERIAL_STATION_COUNT]);  // 声明接口 LoadRoundFromDisc（Load轮次From圆盘），参数由此处的签名确定。

// 第一层兼容入口，等价于 PlaceTaskCargoToWorkArea(positionCodes, 1)。
// 示例：PlaceRoundToWorkArea(currentTask.round1_pos);
/**
 * @brief 第一层整组放料兼容入口，等价于PlaceTaskCargoToWorkArea传入layer=1。
 */
bool PlaceRoundToWorkArea(const int positionCodes[MATERIAL_STATION_COUNT]);  // 声明接口 PlaceRoundToWorkArea（Place轮次To工位Area），参数由此处的签名确定。

// 按位置码从区域中取回三个物料，分别放回 3、2、1 号载物台。
// 示例：RetrieveRoundToCargo(currentTask.round1_colors,
//                              currentTask.round1_pos);
/**
 * @brief 按本轮颜色和位置数组同步从工位取回三件，放入载物台1~3并恢复物料登记。
 */
bool RetrieveRoundToCargo(const int materialCodes[MATERIAL_STATION_COUNT], const int positionCodes[MATERIAL_STATION_COUNT]);  // 声明接口 RetrieveRoundToCargo（Retrieve轮次To货位），参数由此处的签名确定。

// 第二层兼容入口，等价于 PlaceTaskCargoToWorkArea(positionCodes, 2)。
// 示例：StackRoundToWorkArea(currentTask.round2_pos);
/**
 * @brief 第二层整组叠放兼容入口，等价于PlaceTaskCargoToWorkArea传入layer=2。
 */
bool StackRoundToWorkArea(const int positionCodes[MATERIAL_STATION_COUNT]);  // 声明接口 StackRoundToWorkArea（Stack轮次To工位Area），参数由此处的签名确定。

// 圆盘抓取接口
// force=true 时，载物台占用仅警告并继续。
// true 表示预设动作执行完成并已登记物料；不代表传感器确认实物。
/**
 * @brief 同步提交圆盘抓放作业；force选择强制抓取入口，成功只表示预设作业完成并登记。
 */
bool GrabDiscMaterial(uint8_t materialCode, uint8_t cargoCode, bool force = false);  // 声明接口 GrabDiscMaterial（Grab圆盘物料），参数由此处的签名确定。

// 新版圆盘位置抓取分为“位置专用夹取”和“公共载物台放置”两段。
// 夹取前先调用校验；夹住并抬升后调用放置，成功后登记载物台颜色。
/**
 * @brief 检查目标载物台是否为空；当前实现不使用颜色参数，true表示允许继续放入。
 */
bool ValidateDiscPositionTransfer(uint8_t materialCode, uint8_t cargoCode);  // 声明接口 ValidateDiscPositionTransfer（Validate圆盘位置Transfer），参数由此处的签名确定。
/**
 * @brief 将已经夹住的圆盘物料同步放入指定载物台，完成后登记颜色；不重复夹取。
 */
bool PlaceHeldDiscMaterialToCargo(uint8_t materialCode, uint8_t cargoCode);  // 声明接口 PlaceHeldDiscMaterialToCargo（PlaceHeld圆盘物料To货位），参数由此处的签名确定。



#endif // MATERIAL_TRANSFER_H
