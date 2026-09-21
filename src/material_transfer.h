#ifndef MATERIAL_TRANSFER_H
#define MATERIAL_TRANSFER_H

#include <Arduino.h>

#include "chassis.h"

// 物料编号与扫码任务码一致，0 表示空载。
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
    MaterialType material;
};

// 一个抓取/放置点相对于车体的机械臂位姿。夹爪角度由搬运流程统一控制。
struct MaterialStationPose {
    float high;          // 升降高度 (mm)
    float length;        // 伸出长度 (mm)
    float turretAngle;   // 转台角度 (deg)
};

constexpr uint8_t MATERIAL_STATION_COUNT = 3;

/**
 * @brief 物料搬运所需的全部固定参数。
 *
 * cargo[0..2] 对应车上 1~3 号载物台；workArea[0..2] 对应粗加工区或
 * 暂存区的 1~3 号位置。小车在两个区域的停车位姿相同时，两者共用
 * workArea，无需维护两套机械臂坐标。
 */
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

// 在 material_transfer.cpp 顶部填写实车标定值；未填写时所有动作会安全拒绝。
extern MaterialTransferLayout materialTransferLayout;
extern CargoPlatform cargoPlatforms[MATERIAL_STATION_COUNT];

bool MaterialTransferPosesReady();

// 四个基本动作。编号均从 1 开始，与扫码任务码一致。
bool MoveDiscToCargo(uint8_t materialCode, uint8_t cargoCode);
bool MoveCargoToWorkArea(uint8_t cargoCode, uint8_t workAreaCode);
bool MoveWorkAreaToCargo(uint8_t materialCode, uint8_t workAreaCode,
                         uint8_t cargoCode);
bool StackCargoToWorkArea(uint8_t cargoCode, uint8_t workAreaCode);

// 整组动作：参数可直接传 TaskCode 中对应的三个元素数组。
bool LoadRoundFromDisc(const int materialCodes[MATERIAL_STATION_COUNT]);
bool PlaceRoundToWorkArea(const int positionCodes[MATERIAL_STATION_COUNT]);
bool RetrieveRoundToCargo(
    const int materialCodes[MATERIAL_STATION_COUNT],
    const int positionCodes[MATERIAL_STATION_COUNT]);
bool StackRoundToWorkArea(const int positionCodes[MATERIAL_STATION_COUNT]);

#endif // MATERIAL_TRANSFER_H
