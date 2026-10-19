#include "material_transfer.h"

#include <math.h>

// ================= 实车标定区（主要修改这里） =================
// 每一项的填写顺序都是：{升降高度, 伸出长度, 转台角度}。
// NAN 表示“尚未标定”。请用实测值替换下面共 7 个位置中的 NAN。
// 在全部位置填写完成前，MaterialTransferPosesReady() 返回 false，机械臂不会动作。
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
        {{100, 0, 300}, 700, 500, 400, {300, 0, 300}, 1200, 300, 1500}, // 1 -> 1
        {{100, 0, 300}, 700, 500, 400, {200, 0, 250}, 1200, 300, 1500}, // 1 -> 2
        {{100, 0, 300}, 700, 500, 400, {300, 0, 200}, 1200, 300, 1500}, // 1 -> 3
    },
    {
        {{100, 0, 300}, 700, 500, 400, {300, 0, 300}, 1200, 300, 1500}, // 2 -> 1
        {{100, 0, 300}, 700, 500, 300, {200, 0, 300}, 1300, 300, 1500}, // 2 -> 2
        {{100, 0, 300}, 700, 500, 400, {300, 0, 300}, 1300, 300, 1500}, // 2 -> 3
    },
    {
        {{100, 0, 200}, 700, 500, 300, {200, 0, 100}, 1200, 300, 1500}, // 3 -> 1
        {{100, 0, 300}, 700, 500, 300, {200, 0, 200}, 1250, 300, 1500}, // 3 -> 2
        {{100, 0, 300}, 700, 500, 300, {200, 0, 200}, 1250, 300, 1500}, // 3 -> 3
    },
};
MaterialAuxiliaryDelays materialAuxiliaryDelays = {300, 340, 300, 1360, 680, 1360, 680};
// ACTION_GROUP_DELAYS_END

// 开机时三个载物台均为空载。
CargoPlatform cargoPlatforms[MATERIAL_STATION_COUNT] = {
    {MATERIAL_NONE}, {MATERIAL_NONE}, {MATERIAL_NONE}
};

namespace {

bool isMaterialCodeValid(int code) {
    // 任务码中的合法颜色编号为 1~6。
    return code >= MATERIAL_RED && code <= MATERIAL_LIGHT_BLUE;
}

bool isStationCodeValid(int code) {
    // 载物台编号和区域位置编号都只能是 1、2、3。
    return code >= 1 && code <= MATERIAL_STATION_COUNT;
}

bool isPoseValid(const MaterialStationPose &pose) {
    // 检查标定值是否已经填写，并确保没有超过 MoveArm 的机械行程。
    return isfinite(pose.high) && pose.high >= 0.0f && pose.high <= ARM_HEIGHT_LIMIT_MM
        && isfinite(pose.length) && pose.length >= 0.0f && pose.length <= 170.0f
        && isfinite(pose.turretAngle)
        && pose.turretAngle >= TURRET_CABLE_MIN_DEG
        && pose.turretAngle <= TURRET_CABLE_MAX_DEG;
}

bool isLayoutParameterValid() {
    return isfinite(materialTransferLayout.approachHeight)
        && materialTransferLayout.approachHeight >= 0.0f
        && materialTransferLayout.approachHeight <= ARM_HEIGHT_LIMIT_MM
        && isfinite(materialTransferLayout.secondLayerOffset)
        && materialTransferLayout.secondLayerOffset >= 0.0f
        && isfinite(materialTransferLayout.moveSpeed)
        && materialTransferLayout.moveSpeed > 0.0f
        && isfinite(materialTransferLayout.clawOpenAngle)
        && isfinite(materialTransferLayout.clawClosedAngle)
        && materialTransferLayout.motionWaitMs > 0;
}

void waitForArm(float durationMs = materialTransferLayout.motionWaitMs) {
    // 参数直接表示毫秒：waitForArm(1000) 等待1秒，不乘基础时间。
    // 不传参数或参数非法时，使用配置中的默认等待时间。
    if (!isfinite(durationMs) || durationMs < 0.0f) {
        Serial.println("[Material] ERR: invalid arm wait duration");
        durationMs = materialTransferLayout.motionWaitMs;
    }
    if (durationMs == 0.0f) return;
    const uint32_t waitMs = static_cast<uint32_t>(
        lroundf(durationMs)
    );
    vTaskDelay(pdMS_TO_TICKS(waitMs));
}

// 移动到目标上方
bool moveAbove(const MaterialStationPose &pose, float clawAngle,
               const MaterialApproachDelays &delays) {
    // 升降、伸缩同时发令；等待后转向。两次等待分别保留，供上位机逐项调整。
    if (!MoveArm(materialTransferLayout.approachHeight, pose.length,
                 -1, clawAngle, materialMotionSteps.approachTranslateSpeed)) return false;
    waitForArm(delays.translateMs);
    waitForArm(delays.settleMs);
    if (!MoveArm(materialTransferLayout.approachHeight, pose.length,
                 pose.turretAngle, clawAngle, materialMotionSteps.approachRotateSpeed)) return false;
    waitForArm(delays.rotateMs);
    return true;
}

// 抓取物料
bool pickAt(const MaterialStationPose &pose,
            const MaterialTransferDelays &delays = commonMaterialDelays,
            bool retractAfterGrip = false) {
    // 夹紧后先保持伸出长度抬升；载物台出料在抬升等待结束后再缩回到 0。
    if (!moveAbove(pose, materialTransferLayout.clawOpenAngle, delays.pickAbove)) return false;
    if (!MoveArm(pose.high, -1, -1, materialTransferLayout.clawOpenAngle,
                 materialMotionSteps.pickDescendSpeed)) return false;
    waitForArm(delays.pickDescendMs);
    if (!MoveArm(-1, -1, -1, materialTransferLayout.clawClosedAngle,
                 materialMotionSteps.pickGripSpeed)) return false;
    waitForArm(delays.pickGripMs);
    if (!MoveArm(materialTransferLayout.approachHeight, -1, -1, -1,
                 materialMotionSteps.pickLiftSpeed)) return false;
    waitForArm(delays.pickLiftMs);
    if (retractAfterGrip
            && !MoveArm(-1, 0.0f, -1, -1, materialMotionSteps.pickLiftSpeed)) return false;
    return true;
}

// 放置物料
//@param pose 目标位置  
//@param targetHeight 目标高度
bool placeAt(const MaterialStationPose &pose, float targetHeight,
             const MaterialTransferDelays &delays = commonMaterialDelays,
             bool fromRetracted = false) {
    if (fromRetracted) {
        // pickAt 已先抬升、等待、再缩回；此处等待缩回并稳定，再保持缩回转向。
        waitForArm(delays.placeAbove.translateMs);
        waitForArm(delays.placeAbove.settleMs);
        if (!MoveArm(-1, -1, pose.turretAngle, -1, materialMotionSteps.approachRotateSpeed)) return false;
        waitForArm(delays.placeAbove.rotateMs);
    } else if (!moveAbove(pose, materialTransferLayout.clawClosedAngle, delays.placeAbove)) return false;
    // 载物台到工作区转向后同时伸出、下降；其他放料仍沿用原先的垂直下降。
    if (!MoveArm(targetHeight, fromRetracted ? pose.length : -1.0f, -1,
                 materialTransferLayout.clawClosedAngle, materialMotionSteps.placeDescendSpeed)) return false;
    waitForArm(delays.placeDescendMs);
    if (!MoveArm(-1, -1, -1, materialTransferLayout.clawOpenAngle, materialMotionSteps.placeReleaseSpeed)) return false;
    waitForArm(delays.placeReleaseMs);
    if (!MoveArm(materialTransferLayout.approachHeight, -1, -1, -1, materialMotionSteps.placeLiftSpeed)) return false;
    waitForArm(delays.placeLiftMs);
    return true;
}

bool validateAction(const MaterialStationPose &source,
                    const MaterialStationPose &destination) {
    // 在抓起物料之前同时检查起点、终点及公共动作参数。
    // 这样可以避免抓起来以后才发现目标位置没有标定。
    if (!isLayoutParameterValid() || !isPoseValid(source)
            || !isPoseValid(destination)) {
        Serial.println("[Material] ERR: transfer poses are not calibrated");
        return false;
    }
    return true;
}

bool validateCodeArrays(const int *first, const int *second, bool firstIsMaterial) {
    // 整组动作开始前检查三个扫码值，防止执行到一半才发现任务码错误。
    if (first == nullptr) {
        Serial.println("[Material] ERR: null task code array");
        return false;
    }
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        if ((firstIsMaterial && !isMaterialCodeValid(first[i])) || (!firstIsMaterial && !isStationCodeValid(first[i])) || (second != nullptr && !isStationCodeValid(second[i]))) {
            Serial.println("[Material] ERR: invalid task code");
            return false;
        }
    }
    const int *positions = firstIsMaterial ? second : first;
    // 三个物料必须去三个不同位置，避免把两个物料放到同一个第一层位置。
    if (positions != nullptr && (positions[0] == positions[1] || positions[0] == positions[2] || positions[1] == positions[2])) {
        Serial.println("[Material] ERR: duplicate work-area position code");
        return false;
    }
    return true;
}

} // namespace

bool MaterialTransferPosesReady() {
    // 七个固定位置只要有一个仍是 NAN，整组动作就不能开始。
    if (!isLayoutParameterValid() || !isPoseValid(materialTransferLayout.disc)) {
        return false;
    }
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        if (!isPoseValid(materialTransferLayout.cargo[i]) || !isPoseValid(materialTransferLayout.workArea[i])) {
            return false;
        }
    }
    return true;
}

bool MoveDiscToCargo(uint8_t materialCode, uint8_t cargoCode) {
    // 数组下标从 0 开始，所以外部传入的 1~3 号要减 1。
    if (!isMaterialCodeValid(materialCode) || !isStationCodeValid(cargoCode)) {
        Serial.println("[Material] ERR: invalid material/cargo code");
        return false;
    }
    CargoPlatform &cargo = cargoPlatforms[cargoCode - 1];
    if (cargo.material != MATERIAL_NONE) {
        Serial.println("[Material] ERR: cargo platform is occupied");
        return false;
    }
    const MaterialStationPose &destination = materialTransferLayout.cargo[cargoCode - 1];
    if (!validateAction(materialTransferLayout.disc, destination)) return false;

    Serial.printf("[Material] disc -> cargo %u, material %u\n", cargoCode, materialCode);
    // 先从圆盘抓起，再放到指定载物台；完成后才更新载物台状态。
    if (!pickAt(materialTransferLayout.disc) || !placeAt(destination, destination.high)) return false;
    cargo.material = static_cast<MaterialType>(materialCode);
    return true;
}

bool MoveCargoToWorkArea(uint8_t cargoCode, uint8_t workAreaCode) {
    if (!isStationCodeValid(cargoCode) || !isStationCodeValid(workAreaCode)) {
        Serial.println("[Material] ERR: invalid cargo/work-area code");
        return false;
    }
    CargoPlatform &cargo = cargoPlatforms[cargoCode - 1];
    if (cargo.material == MATERIAL_NONE) {
        Serial.println("[Material] ERR: cargo platform is empty");
        return false;
    }
    const MaterialStationPose &source = materialTransferLayout.cargo[cargoCode - 1];
    const MaterialStationPose &destination =
        materialTransferLayout.workArea[workAreaCode - 1];
    if (!validateAction(source, destination)) return false;

    Serial.printf("[Material] cargo %u -> work area %u, material %u\n",
                  cargoCode, workAreaCode, cargo.material);
    // 粗加工区和暂存区使用相同的 workArea 位姿。
    // 小车当前停在哪一个区域，动作就发生在哪一个区域。
    const auto &delays = cargoToWorkAreaDelays[cargoCode - 1][workAreaCode - 1];
    if (!pickAt(source, delays, true) || !placeAt(destination, destination.high, delays, true)) return false;
    cargo.material = MATERIAL_NONE;
    return true;
}

//放料
bool DemoCargoToRoughArea() {
    // 依次完成三次搬运：载物台 1 -> 粗加工区 1、2 -> 2、3 -> 3（第一层）。
    // 每次完整执行抓取、搬运、放置和抬升后，才开始下一次；本函数不负责底盘导航。
    constexpr uint8_t materialCode = MATERIAL_RED; // 物料颜色码：1~6
    static_assert(materialCode >= MATERIAL_RED && materialCode <= MATERIAL_LIGHT_BLUE,
                  "Invalid demo material code");

    for (uint8_t cargoCode = 1; cargoCode <= MATERIAL_STATION_COUNT; ++cargoCode) {
        // 编号为 1~3，目标位置顺序对应为 1~3；数组下标需要减 1。
        const uint8_t workAreaCode = cargoCode;
        // 沿用原示例：手动装料后临时登记为红色物料，复用完整搬运接口。
        // 失败时恢复当前载物台记录并停止；已完成的载物台保持空载。
        CargoPlatform &cargo = cargoPlatforms[cargoCode - 1];
        const MaterialType previousMaterial = cargo.material;
        cargo.material = static_cast<MaterialType>(materialCode);
        if (!MoveCargoToWorkArea(cargoCode, workAreaCode)) {
            cargo.material = previousMaterial;
            return false;
        }
    }
    return true;
}

bool DemoWorkAreaToCargo() {
    // 调试时暂存区的三个位置各放一件物料；颜色只用于载物台状态记录。
    // 取料仍按区域 1、2、3；放回载物台时按内侧到外侧的 3、2、1 顺序。
    constexpr uint8_t materialCode = MATERIAL_RED;
    static_assert(materialCode >= MATERIAL_RED && materialCode <= MATERIAL_LIGHT_BLUE,
                  "Invalid demo material code");

    // 整组预检查：避免搬到一半才发现后续载物台已占用或位姿无效。
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        if (cargoPlatforms[i].material != MATERIAL_NONE) {
            Serial.println("[Material] ERR: cargo platform is occupied");
            return false;
        }
        const uint8_t cargoIndex = MATERIAL_STATION_COUNT - 1 - i;
        if (!validateAction(materialTransferLayout.workArea[i],
                            materialTransferLayout.cargo[cargoIndex])) return false;
    }

    for (uint8_t workAreaCode = 1; workAreaCode <= MATERIAL_STATION_COUNT; ++workAreaCode) {
        // 区域 1→载物台 3、区域 2→载物台 2、区域 3→载物台 1。
        // 单次接口会把外部 1~3 编号转换成数组 0~2 下标；每件放稳、抬升后才取下一件。
        const uint8_t cargoCode = MATERIAL_STATION_COUNT + 1 - workAreaCode;
        if (!MoveWorkAreaToCargo(materialCode, workAreaCode, cargoCode)) return false;
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
        // 三个位置全部检查通过后才抓料，避免执行到一半才发现高度越界。
        if (!validateAction(materialTransferLayout.cargo[i], destination)) return false;
        if (!isfinite(releaseHeights[i]) || releaseHeights[i] > ARM_HEIGHT_LIMIT_MM) {
            Serial.println("[Material] ERR: invalid demo stacking height");
            return false;
        }
    }

    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        // 数组下标 0~2 对应载物台和区域编号 1~3。
        // 手动装料按红色登记，完成抓取、码放和收回后再清空记录。
        cargoPlatforms[i].material = MATERIAL_RED;
        Serial.printf("[Material] demo stack cargo %u -> work area %u, release height %.1f\n",
                      i + 1, i + 1, releaseHeights[i]);
        const auto &delays = cargoToWorkAreaDelays[i][i];
        if (!pickAt(materialTransferLayout.cargo[i], delays, true)
                || !placeAt(materialTransferLayout.workArea[i], releaseHeights[i], delays, true)) return false;
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
        // 三个位置全部检查通过后才抓料，避免执行到一半才发现高度越界。
        if (!validateAction(materialTransferLayout.cargo[i], destination)) return false;
        if (!isfinite(releaseHeights[i]) || releaseHeights[i] > ARM_HEIGHT_LIMIT_MM) {
            Serial.println("[Material] ERR: invalid demo stacking height");
            return false;
        }
    }

    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        // 数组下标 0~2 对应载物台和区域编号 1~3。
        // 手动装料按红色登记，完成抓取、码放和收回后再清空记录。
        cargoPlatforms[i].material = MATERIAL_RED;
        Serial.printf("[Material] demo stack cargo %u -> work area %u, release height %.1f\n",
                      i + 1, i + 1, releaseHeights[i]);
        const auto &delays = cargoToWorkAreaDelays[i][i];
        if (!pickAt(materialTransferLayout.cargo[i], delays, true)
                || !placeAt(materialTransferLayout.workArea[i], releaseHeights[i], delays, true)) return false;
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
    if (!isMaterialCodeValid(materialCode) || !isStationCodeValid(workAreaCode) || !isStationCodeValid(cargoCode)) {
        Serial.println("[Material] ERR: invalid material/work-area/cargo code");
        return false;
    }
    CargoPlatform &cargo = cargoPlatforms[cargoCode - 1];
    if (cargo.material != MATERIAL_NONE) {
        Serial.println("[Material] ERR: cargo platform is occupied");
        return false;
    }
    const MaterialStationPose &source =
        materialTransferLayout.workArea[workAreaCode - 1];
    const MaterialStationPose &destination = materialTransferLayout.cargo[cargoCode - 1];
    if (!validateAction(source, destination)) return false;

    Serial.printf("[Material] work area %u -> cargo %u, material %u\n",
                  workAreaCode, cargoCode, materialCode);
    // 抓取成功并放到车上以后，记录该载物台装入了什么颜色的物料。
    if (!pickAt(source) || !placeAt(destination, destination.high)) return false;
    cargo.material = static_cast<MaterialType>(materialCode);
    return true;
}

bool StackCargoToWorkArea(uint8_t cargoCode, uint8_t workAreaCode) {
    if (!isStationCodeValid(cargoCode) || !isStationCodeValid(workAreaCode)) {
        Serial.println("[Material] ERR: invalid cargo/work-area code");
        return false;
    }
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
    if (!validateAction(source, destination)) return false;
    if (secondLayerHeight > ARM_HEIGHT_LIMIT_MM) {
        Serial.println("[Material] ERR: invalid second-layer height");
        return false;
    }

    Serial.printf("[Material] cargo %u -> work area %u layer 2, material %u\n",
                  cargoCode, workAreaCode, cargo.material);
    const auto &delays = cargoToWorkAreaDelays[cargoCode - 1][workAreaCode - 1];
    if (!pickAt(source, delays, true) || !placeAt(destination, secondLayerHeight, delays, true)) return false;
    cargo.material = MATERIAL_NONE;
    return true;
}

bool LoadRoundFromDisc(const int materialCodes[MATERIAL_STATION_COUNT]) {
    // materialCodes[0..2] 对应扫码结果中的三个颜色码。
    // 三个物料固定依次放入 1、2、3 号载物台。
    if (!validateCodeArrays(materialCodes, nullptr, true)
            || !MaterialTransferPosesReady()) return false;
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

bool PlaceTaskCargoToWorkArea(
        const int positionCodes[MATERIAL_STATION_COUNT], uint8_t layer) {
    // 1. 整组预检查：无效任务码、层数或目标高度不能让机械臂先动起来。
    if (layer < 1 || layer > 3) {
        Serial.println("[Material] ERR: layer must be 1, 2 or 3");
        return false;
    }
    if (!validateCodeArrays(positionCodes, nullptr, false)
            || !MaterialTransferPosesReady()) return false;

    float releaseHeights[MATERIAL_STATION_COUNT];
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        if (cargoPlatforms[i].material == MATERIAL_NONE) {
            Serial.println("[Material] ERR: cargo platform is empty");
            return false;
        }
        const MaterialStationPose &source = materialTransferLayout.cargo[i];
        const MaterialStationPose &destination =
            materialTransferLayout.workArea[positionCodes[i] - 1];
        releaseHeights[i] = destination.high
            + static_cast<float>(layer - 1) * materialTransferLayout.secondLayerOffset;
        if (!validateAction(source, destination)) return false;
        if (!isfinite(releaseHeights[i]) || releaseHeights[i] < 0.0f
                || releaseHeights[i] > ARM_HEIGHT_LIMIT_MM) {
            Serial.println("[Material] ERR: invalid stacking height");
            return false;
        }
    }

    // 2. 与已测试 Demo 相同的逐件动作：抓取、转向放置、松手、抬升。
    // 编号从 1 开始，数组下标从 0 开始；成功放完一件才清空其载物台记录。
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        const uint8_t workAreaCode = static_cast<uint8_t>(positionCodes[i]);
        Serial.printf("[Material] cargo %u -> work area %u, layer %u, release height %.1f\n",
                      i + 1, workAreaCode, layer, releaseHeights[i]);
        const auto &delays = cargoToWorkAreaDelays[i][workAreaCode - 1];
        if (!pickAt(materialTransferLayout.cargo[i], delays, true)
                || !placeAt(materialTransferLayout.workArea[workAreaCode - 1], releaseHeights[i], delays, true)) return false;
        cargoPlatforms[i].material = MATERIAL_NONE;
    }
    return true;
}

bool PlaceRoundToWorkArea(const int positionCodes[MATERIAL_STATION_COUNT]) {
    return PlaceTaskCargoToWorkArea(positionCodes, 1);
}

bool RetrieveRoundToCargo(
        const int materialCodes[MATERIAL_STATION_COUNT],
        const int positionCodes[MATERIAL_STATION_COUNT]) {
    // 从 positionCodes 指定的位置取出 materialCodes 指定的物料，
    // 并按照原顺序分别放回 1、2、3 号载物台。
    if (!validateCodeArrays(materialCodes, positionCodes, true)
            || !MaterialTransferPosesReady()) return false;
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        if (cargoPlatforms[i].material != MATERIAL_NONE) {
            Serial.println("[Material] ERR: cargo platform is occupied");
            return false;
        }
    }
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        if (!MoveWorkAreaToCargo(materialCodes[i], positionCodes[i], i + 1)) {
            return false;
        }
    }
    return true;
}

bool StackRoundToWorkArea(const int positionCodes[MATERIAL_STATION_COUNT]) {
    return PlaceTaskCargoToWorkArea(positionCodes, 2);
}

// 圆盘抓取并放入指定载物台。
bool GrabDiscMaterial(uint8_t materialCode, uint8_t cargoCode, bool force) {
    // 在底盘和机械臂动作前检查编号、占用及放料位姿，避免抓起后无处放置。
    if (!isMaterialCodeValid(materialCode) || !isStationCodeValid(cargoCode)) {
        Serial.println("[Material] ERR: invalid material/cargo code");
        return false;
    }
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
    if (!isLayoutParameterValid() || !isPoseValid(destination)) {
        Serial.println("[Material] ERR: transfer poses are not calibrated");
        return false;
    }

    if (!isPoseValid(materialMotionSteps.discApproach)
            || !isfinite(materialMotionSteps.discOpenAngle)
            || !isfinite(materialMotionSteps.discLiftHeight)
            || materialMotionSteps.discLiftHeight < 0
            || materialMotionSteps.discLiftHeight > ARM_HEIGHT_LIMIT_MM) return false;
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
    if (!isMaterialCodeValid(materialCode) || !isStationCodeValid(cargoCode)) {
        Serial.println("[DISC_POSITION] ERR: invalid material/cargo code");
        return false;
    }
    if (cargoPlatforms[cargoCode - 1].material != MATERIAL_NONE) {
        Serial.println("[DISC_POSITION] ERR: cargo platform is occupied");
        return false;
    }
    const MaterialStationPose &destination = materialTransferLayout.cargo[cargoCode - 1];
    if (!isLayoutParameterValid() || !isPoseValid(destination)) {
        Serial.println("[DISC_POSITION] ERR: cargo pose is not calibrated");
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
