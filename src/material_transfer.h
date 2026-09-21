#ifndef MATERIAL_TRANSFER_H
#define MATERIAL_TRANSFER_H

#include <Arduino.h>

#include "chassis.h"

// ================= 物料编号 =================
// 数值与二维码任务码中的颜色编号完全一致。
// 例如任务码中的 1 表示红色物料，6 表示浅蓝色物料。
// 0 不属于任务码，用来表示载物台当前没有物料。
enum MaterialType : uint8_t {
    MATERIAL_NONE = 0,
    MATERIAL_RED = 1,
    MATERIAL_YELLOW = 2,
    MATERIAL_BLUE = 3,
    MATERIAL_GREEN = 4,
    MATERIAL_BLACK = 5,
    MATERIAL_LIGHT_BLUE = 6
};

struct CargoPlatform {
    MaterialType material; // 这个载物台当前放置的物料；MATERIAL_NONE 表示空载
};

// ================= 单个工位的机械臂位姿 =================
// 这里保存的是小车停稳后，机械臂相对于车体抓取或放置物料时的位置。
// 夹爪的张开、夹紧角度不放在这里，由搬运流程统一控制。
struct MaterialStationPose {
    float high;          // 升降高度 (mm)
    float length;        // 伸出长度 (mm)
    float turretAngle;   // 转台角度 (deg)
};

constexpr uint8_t MATERIAL_STATION_COUNT = 3;

// ================= 全部固定工位和动作参数 =================
// cargo[0]、cargo[1]、cargo[2] 分别是车上 1、2、3 号载物台。
// workArea[0]、workArea[1]、workArea[2] 分别是区域内 1、2、3 号物料位。
//
// 粗加工区和暂存区虽然位于场地中的不同位置，但小车停好以后，三个物料位
// 相对于车体的位置相同。因此二者共用 workArea，不需要分别标定两套位姿。
struct MaterialTransferLayout {
    MaterialStationPose disc;                         // 圆盘取料位
    MaterialStationPose cargo[MATERIAL_STATION_COUNT];
    MaterialStationPose workArea[MATERIAL_STATION_COUNT];
    float approachHeight;        // 横向运动前的安全高度 (mm)
    float secondLayerOffset;     // 第二层相对第一层的高度增量 (mm)
    float moveSpeed;             // 传给 MoveArm 的运动速度
    float clawOpenAngle;         // 夹爪张开角度
    float clawClosedAngle;       // 夹爪夹紧角度
    uint32_t motionWaitMs;       // 每步动作的机械稳定等待时间
};

// materialTransferLayout：所有固定位置和动作参数，标定值在 cpp 文件顶部填写。
// cargoPlatforms：三个载物台当前的物料状态，下标 0~2 对应载物台编号 1~3。
// 位姿未填写完整时，程序会拒绝执行搬运动作，不会控制机械臂移动。
extern MaterialTransferLayout materialTransferLayout;
extern CargoPlatform cargoPlatforms[MATERIAL_STATION_COUNT];

// 检查圆盘、三个载物台和三个区域位置是否已经全部标定。
bool MaterialTransferPosesReady();

// ================= 四个单次搬运动作 =================
// 所有载物台编号和区域位置编号都从 1 开始，而不是从 0 开始。

// 从圆盘抓取一种物料，然后放到指定载物台。
// materialCode：物料颜色编号 1~6；cargoCode：载物台编号 1~3。
bool MoveDiscToCargo(uint8_t materialCode, uint8_t cargoCode);

// 从指定载物台抓取物料，然后放到粗加工区或暂存区的指定位置（第一层）。
// cargoCode：载物台编号 1~3；workAreaCode：区域位置编号 1~3。
bool MoveCargoToWorkArea(uint8_t cargoCode, uint8_t workAreaCode);

// 从粗加工区或暂存区的指定位置抓取物料，然后放到指定载物台。
// materialCode 用于恢复载物台中的物料状态记录。
bool MoveWorkAreaToCargo(uint8_t materialCode, uint8_t workAreaCode,
                         uint8_t cargoCode);

// 从指定载物台抓取物料，然后码放到区域指定位置的第二层。
bool StackCargoToWorkArea(uint8_t cargoCode, uint8_t workAreaCode);

// ================= 三个物料的整组搬运动作 =================
// 下面四个函数都循环执行三次，可以直接传入扫码得到的 TaskCode 数组。

// 按颜色码依次从圆盘抓取三个物料，分别放到 1、2、3 号载物台。
// 示例：LoadRoundFromDisc(currentTask.round1_colors);
bool LoadRoundFromDisc(const int materialCodes[MATERIAL_STATION_COUNT]);

// 将 1、2、3 号载物台上的物料，分别放到 positionCodes 指定的第一层位置。
// 示例：PlaceRoundToWorkArea(currentTask.round1_pos);
bool PlaceRoundToWorkArea(const int positionCodes[MATERIAL_STATION_COUNT]);

// 按位置码从区域中取回三个物料，分别放回 1、2、3 号载物台。
// 示例：RetrieveRoundToCargo(currentTask.round1_colors,
//                              currentTask.round1_pos);
bool RetrieveRoundToCargo(
    const int materialCodes[MATERIAL_STATION_COUNT],
    const int positionCodes[MATERIAL_STATION_COUNT]);

// 将三个载物台上的物料，分别码放到 positionCodes 指定位置的第二层。
// 示例：StackRoundToWorkArea(currentTask.round2_pos);
bool StackRoundToWorkArea(const int positionCodes[MATERIAL_STATION_COUNT]);

#endif // MATERIAL_TRANSFER_H
