#include "material_transfer.h"
#include "turret_path.h"

#include <math.h>

// ================= 实车标定区（主要修改这里） =================
// 每一项的填写顺序都是：{升降高度, 伸出长度, 转台角度}。
// 固定位置由参数配置提供，调用搬运动作前应完成实车标定。
MaterialTransferLayout materialTransferLayout = {
    {150, 50, 90}, // 圆盘取料位置
    {
        {102, 20, 242}, // 1 号载物台
        {102, 20, 271}, // 2 号载物台
        {102, 21, 300}, // 3 号载物台
    },
    {
        {2.5, 61, 48}, // 粗加工区/暂存区的 1 号位置
        {2.5, 0, 86}, // 粗加工区/暂存区的 2 号位置
        {2.5, 39, 125.3}, // 粗加工区/暂存区的 3 号位置
    },
    170.0f, // approachHeight，按物料实际高度修改
    60.0f,  // secondLayerOffset，按物料实际高度修改
    250.0f, // moveSpeed
    50.0f,   // clawOpenAngle
    -5.0f,  // clawClosedAngle
    1,      // motionWaitMs，不传等待参数时的默认毫秒数
};

MaterialMotionSteps materialMotionSteps = {
    {80.0f, 0.0f, 90.0f}, // 圆盘预抓：高度、伸出、转台
    90.0f,  // 圆盘预抓张爪角度
    150.0f, // 圆盘抓后提起高度
    250.0f, 250.0f, 250.0f, // 圆盘预抓、夹紧、提起
    250.0f, 250.0f,         // 移至工位上方：平移、转向
    250.0f, 250.0f, 250.0f, // 取料：下降、夹紧、提起
    250.0f, 250.0f, 250.0f, // 放料：下降、松爪、提起
};

// ACTION_GROUP_DELAYS_BEGIN
// 上位机只改写本区域的延时数值；单位 ms，0 表示不等待。
// 每行顺序：取料平移/稳定/转向，下降/夹紧/抬升，放料平移/稳定/转向，下降/松爪/抬升。
MaterialTransferDelays commonMaterialDelays =
    {{1000, 0, 700}, 700, 700, 700, {700, 0, 1000}, 700, 300, 300};
MaterialTransferDelays cargoToWorkAreaDelays[MATERIAL_STATION_COUNT][MATERIAL_STATION_COUNT] = {
    {
        {{100, 0, 800}, 600, 500, 400, {300, 0, 800}, 1100, 300, 1500}, // 1 -> 1
        {{100, 0, 800}, 600, 500, 400, {200, 0, 600}, 1100, 300, 1500}, // 1 -> 2
        {{100, 0, 800}, 600, 500, 400, {300, 0, 200}, 1200, 300, 1500}, // 1 -> 3
    },
    {
        {{100, 0, 800}, 600, 500, 400, {300, 0, 800}, 1100, 300, 1500}, // 2 -> 1
        {{100, 0, 800}, 600, 500, 400, {200, 0, 900}, 1100, 300, 1500}, // 2 -> 2
        {{100, 0, 800}, 600, 500, 400, {300, 0, 800}, 800, 300, 1500}, // 2 -> 3
    },
    {
        {{100, 0, 800}, 600, 500, 400, {300, 0, 600}, 800, 300, 1500}, // 3 -> 1
        {{100, 0, 800}, 600, 500, 400, {300, 0, 700}, 800, 300, 1500}, // 3 -> 2
        {{100, 0, 800}, 600, 500, 300, {200, 0, 900}, 800, 300, 1500}, // 3 -> 3
    },
};
MaterialAuxiliaryDelays materialAuxiliaryDelays = {300, 340, 300, 1360, 680, 1360, 680};
// ACTION_GROUP_DELAYS_END

// REVERSE_ACTION_GROUP_DELAYS_BEGIN
// 工作区 -> 载物台，按 [工作区编号-1][载物台编号-1] 选择；单位 ms。
// 每行与正向相同，共12项；取料抬升表示同时抬升、缩回到0之后的等待。
// 九组初值复制当前公共默认值，之后各自独立调整、保存。
MaterialTransferDelays workAreaToCargoDelays[MATERIAL_STATION_COUNT][MATERIAL_STATION_COUNT] = {
    {
        {{1000, 0, 700}, 700, 700, 700, {700, 0, 1000}, 700, 300, 300}, // 工作区1 -> 载物台1
        {{1000, 0, 700}, 700, 700, 700, {700, 0, 1000}, 700, 300, 300}, // 工作区1 -> 载物台2
        {{1000, 0, 700}, 700, 700, 700, {700, 0, 1000}, 700, 300, 300}, // 工作区1 -> 载物台3
    },
    {
        {{1000, 0, 700}, 700, 700, 700, {700, 0, 1000}, 700, 300, 300}, // 工作区2 -> 载物台1
        {{1000, 0, 700}, 700, 700, 700, {700, 0, 1000}, 700, 300, 300}, // 工作区2 -> 载物台2
        {{1000, 0, 700}, 700, 700, 700, {700, 0, 1000}, 700, 300, 300}, // 工作区2 -> 载物台3
    },
    {
        {{1000, 0, 700}, 700, 700, 700, {700, 0, 1000}, 700, 300, 300}, // 工作区3 -> 载物台1
        {{1000, 0, 700}, 700, 700, 700, {700, 0, 1000}, 700, 300, 300}, // 工作区3 -> 载物台2
        {{1000, 0, 700}, 700, 700, 700, {700, 0, 1000}, 700, 300, 300}, // 工作区3 -> 载物台3
    },
};
// REVERSE_ACTION_GROUP_DELAYS_END

// 开机时三个载物台均为空载。
CargoPlatform cargoPlatforms[MATERIAL_STATION_COUNT] = {
    {MATERIAL_NONE}, {MATERIAL_NONE}, {MATERIAL_NONE}
};

namespace {

void waitForArm(float durationMs = materialTransferLayout.motionWaitMs) {
    // 参数直接表示毫秒：waitForArm(1000) 等待1秒，不乘基础时间。
    // 0 表示不等待。
    if (durationMs == 0.0f) return;
    const uint32_t waitMs = static_cast<uint32_t>(
        lroundf(durationMs)
    );
    vTaskDelay(pdMS_TO_TICKS(waitMs));
}

// 移动到目标上方
bool moveAbove(const MaterialStationPose &pose, float clawAngle, const MaterialApproachDelays &delays, const float *plannedAngle = nullptr) {
    // 升降、伸缩同时发令；等待后转向。两次等待分别保留，供上位机逐项调整。
    if (!MoveArm(materialTransferLayout.approachHeight, pose.length,
                 -1, clawAngle, materialMotionSteps.approachTranslateSpeed)) return false;
    waitForArm(delays.translateMs);
    waitForArm(delays.settleMs);
    if (!(plannedAngle
            ? MoveArmPlanned(materialTransferLayout.approachHeight, pose.length,
                             *plannedAngle, clawAngle, materialMotionSteps.approachRotateSpeed)
            : MoveArm(materialTransferLayout.approachHeight, pose.length,
                      pose.turretAngle, clawAngle, materialMotionSteps.approachRotateSpeed))) return false;
    waitForArm(delays.rotateMs);
    return true;
}

// 抓取物料
bool pickAt(const MaterialStationPose &pose,
            const MaterialTransferDelays &delays = commonMaterialDelays,
            bool retractAfterGrip = false, bool retractWithLift = false,
            const float *plannedAngle = nullptr) {
    // 载物台出料先抬升、等待后再缩回；工作区取回可同时抬升并缩回到 0。
    if (!moveAbove(pose, materialTransferLayout.clawOpenAngle, delays.pickAbove, plannedAngle)) return false;
    if (!MoveArm(pose.high, -1, -1, materialTransferLayout.clawOpenAngle,
                 materialMotionSteps.pickDescendSpeed)) return false;
    waitForArm(delays.pickDescendMs);
    if (!MoveArm(-1, -1, -1, materialTransferLayout.clawClosedAngle,
                 materialMotionSteps.pickGripSpeed)) return false;
    waitForArm(delays.pickGripMs);
    if (!MoveArm(materialTransferLayout.approachHeight, retractWithLift ? 0.0f : -1.0f, -1, -1,
                 materialMotionSteps.pickLiftSpeed)) return false;
    waitForArm(delays.pickLiftMs);
    if (retractAfterGrip && !retractWithLift
            && !MoveArm(-1, 0.0f, -1, -1, materialMotionSteps.pickLiftSpeed)) return false;
    return true;
}

// 放置物料
//@param pose 目标位置  
//@param targetHeight 目标高度
bool placeAt(const MaterialStationPose &pose, float targetHeight,
             const MaterialTransferDelays &delays = commonMaterialDelays,
             bool fromRetracted = false, const float *plannedAngle = nullptr) {
    if (fromRetracted) {
        // pickAt 已完成抬升和缩回；此处额外等待并稳定，再保持缩回转向。
        waitForArm(delays.placeAbove.translateMs);
        waitForArm(delays.placeAbove.settleMs);
        if (!(plannedAngle
                ? MoveArmPlanned(-1, -1, *plannedAngle, -1, materialMotionSteps.approachRotateSpeed)
                : MoveArm(-1, -1, pose.turretAngle, -1, materialMotionSteps.approachRotateSpeed))) return false;
        waitForArm(delays.placeAbove.rotateMs);
    } else if (!moveAbove(pose, materialTransferLayout.clawClosedAngle,
                          delays.placeAbove, plannedAngle)) return false;
    // 缩回后的搬运在转向后同时伸出、下降；其他放料仍沿用原先的垂直下降。
    if (!MoveArm(targetHeight, fromRetracted ? pose.length : -1.0f, -1,
                 materialTransferLayout.clawClosedAngle, materialMotionSteps.placeDescendSpeed)) return false;
    waitForArm(delays.placeDescendMs);
    if (!MoveArm(-1, -1, -1, materialTransferLayout.clawOpenAngle, materialMotionSteps.placeReleaseSpeed)) return false;
    waitForArm(delays.placeReleaseMs);
    if (!MoveArm(materialTransferLayout.approachHeight, -1, -1, -1, materialMotionSteps.placeLiftSpeed)) return false;
    waitForArm(delays.placeLiftMs);
    return true;
}

bool planTransferAngles(const float *angles, size_t count, TurretPathPlan &plan) {
    float actualAngle = 0.0f;
    if (!ReadArmTurretAngle(actualAngle)) return false;
    if (!PlanTurretPath(actualAngle, angles, count,
                        TURRET_CABLE_MIN_DEG, TURRET_CABLE_MAX_DEG, plan)) {
        Serial.println("[Material] ERR: cannot plan turret path");
        return false;
    }
    Serial.printf("[Material] turret plan: %u targets, %.1f degrees\n",
                  static_cast<unsigned>(count), plan.totalDegrees);
    return true;
}

bool planWorkTransfers(const int *workCodes, const int *cargoCodes, bool retrieve, TurretPathPlan &plan) {
    // 任务顺序不变，整组三件一起比较六个取放目标，不逐段贪心选角。
    float angles[TURRET_PATH_MAX_TARGETS] = {};
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        const auto &cargo = materialTransferLayout.cargo[cargoCodes[i] - 1];
        const auto &work = materialTransferLayout.workArea[workCodes[i] - 1];

        angles[2 * i] = retrieve ? work.turretAngle : cargo.turretAngle;
        angles[2 * i + 1] = retrieve ? cargo.turretAngle : work.turretAngle;
    }
    return planTransferAngles(angles, TURRET_PATH_MAX_TARGETS, plan);
}

bool executeWorkTransfer(const MaterialStationPose &source, const MaterialStationPose &destination,
                         float height, const MaterialTransferDelays &delays, bool retrieve,
                         const float *angles = nullptr) {
    TurretPathPlan plan;
    if (!angles) {
        const float targets[] = {source.turretAngle, destination.turretAngle};
        if (!planTransferAngles(targets, 2, plan)) return false;
        angles = plan.anglesDeg;
    }
    // 整组规划选择多圈目标，每次旋转按实测起点计算耗时。
    return pickAt(source, delays, !retrieve, retrieve, angles)
        && placeAt(destination, height, delays, true, angles + 1);
}

} // namespace

bool MoveDiscToCargo(uint8_t materialCode, uint8_t cargoCode) {
    // 数组下标从 0 开始，所以外部传入的 1~3 号要减 1。
    CargoPlatform &cargo = cargoPlatforms[cargoCode - 1];
    if (cargo.material != MATERIAL_NONE) {
        Serial.println("[Material] ERR: cargo platform is occupied");
        return false;
    }
    const MaterialStationPose &destination = materialTransferLayout.cargo[cargoCode - 1];

    Serial.printf("[Material] disc -> cargo %u, material %u\n", cargoCode, materialCode);
    // 先从圆盘抓起，再放到指定载物台；完成后才更新载物台状态。
    if (!pickAt(materialTransferLayout.disc) || !placeAt(destination, destination.high)) return false;
    cargo.material = static_cast<MaterialType>(materialCode);
    return true;
}

bool MoveCargoToWorkArea(uint8_t cargoCode, uint8_t workAreaCode) {
    CargoPlatform &cargo = cargoPlatforms[cargoCode - 1];
    if (cargo.material == MATERIAL_NONE) {
        Serial.println("[Material] ERR: cargo platform is empty");
        return false;
    }
    const MaterialStationPose &source = materialTransferLayout.cargo[cargoCode - 1];
    const MaterialStationPose &destination =
        materialTransferLayout.workArea[workAreaCode - 1];

    Serial.printf("[Material] cargo %u -> work area %u, material %u\n",
                  cargoCode, workAreaCode, cargo.material);
    // 粗加工区和暂存区使用相同的 workArea 位姿。
    // 小车当前停在哪一个区域，动作就发生在哪一个区域。
    const auto &delays = cargoToWorkAreaDelays[cargoCode - 1][workAreaCode - 1];
    if (!executeWorkTransfer(source, destination, destination.high, delays, false)) return false;
    cargo.material = MATERIAL_NONE;
    return true;
}

//放料
bool DemoCargoToRoughArea() {
    // 依次完成三次搬运：载物台 1 -> 粗加工区 1、2 -> 2、3 -> 3（第一层）。
    // 每次完整执行抓取、搬运、放置和抬升后，才开始下一次；本函数不负责底盘导航。
    constexpr uint8_t materialCode = MATERIAL_RED; // 物料颜色码：1~6

    const int codes[] = {1, 2, 3};
    TurretPathPlan plan;
    if (!planWorkTransfers(codes, codes, false, plan)) return false;
    for (uint8_t cargoCode = 1; cargoCode <= MATERIAL_STATION_COUNT; ++cargoCode) {
        // 编号为 1~3，目标位置顺序对应为 1~3；数组下标需要减 1。
        const uint8_t workAreaCode = cargoCode;
        // 沿用原示例：手动装料后临时登记为红色物料，复用完整搬运接口。
        // 失败时恢复当前载物台记录并停止；已完成的载物台保持空载。
        CargoPlatform &cargo = cargoPlatforms[cargoCode - 1];
        const MaterialType previousMaterial = cargo.material;
        cargo.material = static_cast<MaterialType>(materialCode);
        const auto &delays = cargoToWorkAreaDelays[cargoCode - 1][workAreaCode - 1];
        if (!executeWorkTransfer(materialTransferLayout.cargo[cargoCode - 1],
                                 materialTransferLayout.workArea[workAreaCode - 1],
                                 materialTransferLayout.workArea[workAreaCode - 1].high,
                                 delays, false, &plan.anglesDeg[2 * (cargoCode - 1)])) {
            cargo.material = previousMaterial;
            return false;
        }
        cargo.material = MATERIAL_NONE;
    }
    return true;
}

bool DemoWorkAreaToCargo() {
    // 调试时暂存区的三个位置各放一件物料；颜色只用于载物台状态记录。
    // 取料仍按区域 1、2、3；放回载物台时按内侧到外侧的 3、2、1 顺序。
    constexpr uint8_t materialCode = MATERIAL_RED;

    // 检查三个载物台的占用状态。
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        if (cargoPlatforms[i].material != MATERIAL_NONE) {
            Serial.println("[Material] ERR: cargo platform is occupied");
            return false;
        }
    }

    const int workCodes[] = {1, 2, 3}, cargoCodes[] = {3, 2, 1};
    TurretPathPlan plan;
    if (!planWorkTransfers(workCodes, cargoCodes, true, plan)) return false;
    for (uint8_t workAreaCode = 1; workAreaCode <= MATERIAL_STATION_COUNT; ++workAreaCode) {
        // 区域 1→载物台 3、区域 2→载物台 2、区域 3→载物台 1。
        // 单次接口会把外部 1~3 编号转换成数组 0~2 下标；每件放稳、抬升后才取下一件。
        const uint8_t cargoCode = MATERIAL_STATION_COUNT + 1 - workAreaCode;
        const auto &destination = materialTransferLayout.cargo[cargoCode - 1];
        const auto &delays = workAreaToCargoDelays[workAreaCode - 1][cargoCode - 1];
        if (!executeWorkTransfer(materialTransferLayout.workArea[workAreaCode - 1], destination,
                                 destination.high, delays, true,
                                 &plan.anglesDeg[2 * (workAreaCode - 1)])) return false;
        cargoPlatforms[cargoCode - 1].material = static_cast<MaterialType>(materialCode);
    }
    return true;
}

//码放（与放料类似，高度增加）
bool DemoStackCargoToWorkArea() {
    // 手动在三个载物台装好物料，小车停在码放区后调用。
    // 与 MaterialDemo 一样依次执行 1 -> 1、2 -> 2、3 -> 3；
    // 仅松手高度增加 secondLayerOffset。
    float releaseHeights[MATERIAL_STATION_COUNT];
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        const MaterialStationPose &destination = materialTransferLayout.workArea[i];
        releaseHeights[i] = destination.high + materialTransferLayout.secondLayerOffset;
    }

    const int codes[] = {1, 2, 3};
    TurretPathPlan plan;
    if (!planWorkTransfers(codes, codes, false, plan)) return false;
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        // 数组下标 0~2 对应载物台和区域编号 1~3。
        // 手动装料按红色登记，完成抓取、码放和收回后再清空记录。
        cargoPlatforms[i].material = MATERIAL_RED;
        Serial.printf("[Material] demo stack cargo %u -> work area %u, release height %.1f\n",
                      i + 1, i + 1, releaseHeights[i]);
        const auto &delays = cargoToWorkAreaDelays[i][i];
        if (!executeWorkTransfer(materialTransferLayout.cargo[i], materialTransferLayout.workArea[i],
                                 releaseHeights[i], delays, false, &plan.anglesDeg[2 * i])) return false;
        cargoPlatforms[i].material = MATERIAL_NONE;
    }
    return true;
}

bool DemoStackCargoToWorkArea3() {
    // 手动在三个载物台装好物料，小车停在码放区后调用。
    // 与 MaterialDemo 一样依次执行 1 -> 1、2 -> 2、3 -> 3；
    // 仅松手高度增加 两倍secondLayerOffset。
    float releaseHeights[MATERIAL_STATION_COUNT];
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        const MaterialStationPose &destination = materialTransferLayout.workArea[i];
        releaseHeights[i] = destination.high + materialTransferLayout.secondLayerOffset * 2;
    }

    const int codes[] = {1, 2, 3};
    TurretPathPlan plan;
    if (!planWorkTransfers(codes, codes, false, plan)) return false;
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        // 数组下标 0~2 对应载物台和区域编号 1~3。
        // 手动装料按红色登记，完成抓取、码放和收回后再清空记录。
        cargoPlatforms[i].material = MATERIAL_RED;
        Serial.printf("[Material] demo stack cargo %u -> work area %u, release height %.1f\n",
                      i + 1, i + 1, releaseHeights[i]);
        const auto &delays = cargoToWorkAreaDelays[i][i];
        if (!executeWorkTransfer(materialTransferLayout.cargo[i], materialTransferLayout.workArea[i],
                                 releaseHeights[i], delays, false, &plan.anglesDeg[2 * i])) return false;
        cargoPlatforms[i].material = MATERIAL_NONE;
    }
    return true;
}

bool PrepareLidarScanPose(uint8_t startZoneCode) {

    switch (startZoneCode) {
        case 1:
            Serial.println("[LidarPose] start zone 1");

            if (!MovePosition(125,100,0,200)) return false;

            if (!MoveArm(170, 100, -1, -1, 200)) return false;
            if (!MoveArm(-1, -1, 45, -1, 200)) return false;
            waitForArm(materialAuxiliaryDelays.lidarZone1TurnMs);
            if (!MoveArm(0, 100, -1, -1, 200)) return false;
            waitForArm(materialAuxiliaryDelays.lidarZone1LiftMs);
            break;

        case 2:
            Serial.println("[LidarPose] start zone 2");

            if (!MovePosition(125,-100,0,200)) return false;
            
            if (!MoveArm(170, 100, 135, -1, 200)) return false;
            waitForArm(materialAuxiliaryDelays.lidarZone2LiftMs);
            if (!MoveArm(0, 100, 135, -1, 200)) return false;
            waitForArm(materialAuxiliaryDelays.lidarZone2LiftMs);
            break;

        default:
            Serial.println("[LidarPose] ERR: start zone must be 1 or 2");
            return false;
    }

    // 已发完指令并等待配置时长；此处没有硬件到位反馈。
    return true;
}

bool MoveWorkAreaToCargo(uint8_t materialCode, uint8_t workAreaCode, uint8_t cargoCode) {
    CargoPlatform &cargo = cargoPlatforms[cargoCode - 1];
    if (cargo.material != MATERIAL_NONE) {
        Serial.println("[Material] ERR: cargo platform is occupied");
        return false;
    }
    const MaterialStationPose &source =
        materialTransferLayout.workArea[workAreaCode - 1];
    const MaterialStationPose &destination = materialTransferLayout.cargo[cargoCode - 1];

    Serial.printf("[Material] work area %u -> cargo %u, material %u\n",
                  workAreaCode, cargoCode, materialCode);
    // 取回：同时抬升、缩回 -> 等待 -> 缩回转向 -> 等待 -> 同时伸出、下降。
    const auto &delays = workAreaToCargoDelays[workAreaCode - 1][cargoCode - 1];
    if (!executeWorkTransfer(source, destination, destination.high, delays, true)) return false;
    // 抓取成功并放到车上以后，记录该载物台装入了什么颜色的物料。
    cargo.material = static_cast<MaterialType>(materialCode);
    return true;
}

bool StackCargoToWorkArea(uint8_t cargoCode, uint8_t workAreaCode) {
    CargoPlatform &cargo = cargoPlatforms[cargoCode - 1];
    if (cargo.material == MATERIAL_NONE) {
        Serial.println("[Material] ERR: cargo platform is empty");
        return false;
    }
    const MaterialStationPose &source = materialTransferLayout.cargo[cargoCode - 1];
    const MaterialStationPose &destination =
        materialTransferLayout.workArea[workAreaCode - 1];
    // 第二层的放置高度 = 第一层标定高度 + 一个物料的码放高度。
    const float secondLayerHeight =
        destination.high + materialTransferLayout.secondLayerOffset;

    Serial.printf("[Material] cargo %u -> work area %u layer 2, material %u\n",
                  cargoCode, workAreaCode, cargo.material);
    const auto &delays = cargoToWorkAreaDelays[cargoCode - 1][workAreaCode - 1];
    if (!executeWorkTransfer(source, destination, secondLayerHeight, delays, false)) return false;
    cargo.material = MATERIAL_NONE;
    return true;
}

bool LoadRoundFromDisc(const int materialCodes[MATERIAL_STATION_COUNT]) {
    // materialCodes[0..2] 对应扫码结果中的三个颜色码。
    // 三个物料固定依次放入 1、2、3 号载物台。
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        if (cargoPlatforms[i].material != MATERIAL_NONE) {
            Serial.println("[Material] ERR: cargo platform is occupied");
            return false;
        }
    }
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        if (!MoveDiscToCargo(materialCodes[i], i + 1)) return false;
    }
    return true;
}

bool PlaceTaskCargoToWorkArea(const int positionCodes[MATERIAL_STATION_COUNT], uint8_t layer) {
    float releaseHeights[MATERIAL_STATION_COUNT];
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        if (cargoPlatforms[i].material == MATERIAL_NONE) {
            Serial.println("[Material] ERR: cargo platform is empty");
            return false;
        }
        const MaterialStationPose &destination =
            materialTransferLayout.workArea[positionCodes[i] - 1];
        releaseHeights[i] = destination.high
            + static_cast<float>(layer - 1) * materialTransferLayout.secondLayerOffset;
    }

    // 逐件抓取、转向放置、松手、抬升。
    // 编号从 1 开始，数组下标从 0 开始；成功放完一件才清空其载物台记录。
    const int cargoCodes[] = {1, 2, 3};
    TurretPathPlan plan;
    if (!planWorkTransfers(positionCodes, cargoCodes, false, plan)) return false;
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        const uint8_t workAreaCode = static_cast<uint8_t>(positionCodes[i]);
        Serial.printf("[Material] cargo %u -> work area %u, layer %u, release height %.1f\n",
                      i + 1, workAreaCode, layer, releaseHeights[i]);
        const auto &delays = cargoToWorkAreaDelays[i][workAreaCode - 1];
        if (!executeWorkTransfer(materialTransferLayout.cargo[i],
                                 materialTransferLayout.workArea[workAreaCode - 1],
                                 releaseHeights[i], delays, false, &plan.anglesDeg[2 * i])) return false;
        cargoPlatforms[i].material = MATERIAL_NONE;
    }
    return true;
}

bool PlaceRoundToWorkArea(const int positionCodes[MATERIAL_STATION_COUNT]) {
    return PlaceTaskCargoToWorkArea(positionCodes, 1);
}

bool RetrieveRoundToCargo(const int materialCodes[MATERIAL_STATION_COUNT], const int positionCodes[MATERIAL_STATION_COUNT]) {
    // 从 positionCodes 指定的位置取出 materialCodes 指定的物料，
    // 并按照原顺序分别放回 1、2、3 号载物台。
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        if (cargoPlatforms[i].material != MATERIAL_NONE) {
            Serial.println("[Material] ERR: cargo platform is occupied");
            return false;
        }
    }
    const int cargoCodes[] = {1, 2, 3};
    TurretPathPlan plan;
    if (!planWorkTransfers(positionCodes, cargoCodes, true, plan)) return false;
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        const auto &destination = materialTransferLayout.cargo[i];
        const auto &delays = workAreaToCargoDelays[positionCodes[i] - 1][i];
        if (!executeWorkTransfer(materialTransferLayout.workArea[positionCodes[i] - 1], destination,
                                 destination.high, delays, true,
                                 &plan.anglesDeg[2 * i])) return false;
        cargoPlatforms[i].material = static_cast<MaterialType>(materialCodes[i]);
    }
    return true;
}

bool StackRoundToWorkArea(const int positionCodes[MATERIAL_STATION_COUNT]) {
    return PlaceTaskCargoToWorkArea(positionCodes, 2);
}

// 圆盘抓取并放入指定载物台。
bool GrabDiscMaterial(uint8_t materialCode, uint8_t cargoCode, bool force) {
    // force 允许覆盖载物台登记状态。
    CargoPlatform &cargo = cargoPlatforms[cargoCode - 1];
    if (cargo.material != MATERIAL_NONE) {
        if (!force) {
            Serial.println("[Material] ERR: cargo platform is occupied");
            return false;
        }
        Serial.printf("{EVT,VISION,FORCE_GRAB,WARN,OCCUPIED,%u,%u}\n",
                      static_cast<unsigned>(cargo.material), cargoCode);
        Serial.printf("[Material] WARN: cargo %u already contains material %u; force grab continues\n",
                      cargoCode, static_cast<unsigned>(cargo.material));
    }
    const MaterialStationPose &destination = materialTransferLayout.cargo[cargoCode - 1];

    if (!MoveArm(materialMotionSteps.discApproach.high,
                 materialMotionSteps.discApproach.length,
                 materialMotionSteps.discApproach.turretAngle,
                 materialMotionSteps.discOpenAngle,
                 materialMotionSteps.discApproachSpeed)) return false;
    waitForArm(materialAuxiliaryDelays.discApproachMs);
    if (!MoveArm(materialMotionSteps.discApproach.high,
                 materialMotionSteps.discApproach.length,
                 materialMotionSteps.discApproach.turretAngle,
                 materialTransferLayout.clawClosedAngle,
                 materialMotionSteps.discGripSpeed)) return false;
    waitForArm(materialAuxiliaryDelays.discGripMs);
    if (!MoveArm(materialMotionSteps.discLiftHeight, -1, -1, -1,
                 materialMotionSteps.discLiftSpeed)) return false;
    waitForArm(materialAuxiliaryDelays.discLiftMs);

    if (!placeAt(destination, destination.high)) return false;
    cargo.material = static_cast<MaterialType>(materialCode);
    Serial.printf("[Material] disc -> cargo %u, material %u placed\n",
                  cargoCode, materialCode);
    return true;
}

bool ValidateDiscPositionTransfer(uint8_t materialCode, uint8_t cargoCode) {
    if (cargoPlatforms[cargoCode - 1].material != MATERIAL_NONE) {
        Serial.println("[DISC_POSITION] ERR: cargo platform is occupied");
        return false;
    }
    return true;
}

bool PlaceHeldDiscMaterialToCargo(uint8_t materialCode, uint8_t cargoCode) {
    if (!ValidateDiscPositionTransfer(materialCode, cargoCode)) return false;
    CargoPlatform &cargo = cargoPlatforms[cargoCode - 1];
    const MaterialStationPose &destination = materialTransferLayout.cargo[cargoCode - 1];
    if (!placeAt(destination, destination.high)) return false;
    cargo.material = static_cast<MaterialType>(materialCode);
    Serial.printf("[DISC_POSITION] material %u placed on cargo %u\n",
                  materialCode, cargoCode);
    return true;
}
