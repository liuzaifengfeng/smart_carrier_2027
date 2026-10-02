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
    150.0f, // moveSpeed
    50.0f,   // clawOpenAngle
    -3.0f,  // clawClosedAngle
    1000,   // motionWaitMs
};

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

void waitForArm(float multiplier = 1.0f) {
    // multiplier 是基础等待时间的倍数：
    // waitForArm() 等待 1 倍，waitForArm(2.5f) 等待 2.5 倍。
    if (!isfinite(multiplier) || multiplier <= 0.0f) {
        Serial.println("[Material] ERR: invalid arm wait multiplier");
        multiplier = 1.0f; // 非法参数仍按基础时间等待，避免机械臂未停稳就执行下一步。
    }
    const uint32_t waitMs = static_cast<uint32_t>(
        lroundf(materialTransferLayout.motionWaitMs * multiplier)
    );
    vTaskDelay(pdMS_TO_TICKS(waitMs));
}

// 移动到目标上方
bool moveAbove(const MaterialStationPose &pose, float clawAngle) {
    // 先升到安全高度，再转向目标并伸出，防止横向移动时碰到物料或车体。
    if (!MoveArm(materialTransferLayout.approachHeight, pose.length,
                 -1, clawAngle, materialTransferLayout.moveSpeed)) return false;
    waitForArm();    
    waitForArm();
    if (!MoveArm(materialTransferLayout.approachHeight, pose.length,
                 pose.turretAngle, clawAngle, materialTransferLayout.moveSpeed)) return false;
    waitForArm();
    return true;
}

// 抓取物料
bool pickAt(const MaterialStationPose &pose) {
    // 抓取顺序：张开夹爪到目标上方 -> 下降 -> 夹紧 -> 提升到安全高度。
    if (!moveAbove(pose, materialTransferLayout.clawOpenAngle)) return false;
    if (!MoveArm(pose.high, -1, -1, materialTransferLayout.clawOpenAngle,
                 materialTransferLayout.moveSpeed)) return false;
    waitForArm(2);
    if (!MoveArm(-1, -1, -1, materialTransferLayout.clawClosedAngle,
                 materialTransferLayout.moveSpeed)) return false;
    waitForArm();
    if (!MoveArm(materialTransferLayout.approachHeight, -1, -1, -1,
                 materialTransferLayout.moveSpeed)) return false;
    waitForArm();
    return true;
}

// 放置物料
bool placeAt(const MaterialStationPose &pose, float targetHeight) {
    // 放置顺序：夹持物料到目标上方 -> 下降 -> 张开夹爪 -> 提升到安全高度。
    if (!moveAbove(pose, materialTransferLayout.clawClosedAngle)) return false;
    if (!MoveArm(targetHeight, -1, -1, materialTransferLayout.clawClosedAngle,
                 materialTransferLayout.moveSpeed)) return false;
    waitForArm();
    waitForArm();
    if (!MoveArm(-1, -1, -1, materialTransferLayout.clawOpenAngle,
                 materialTransferLayout.moveSpeed)) return false;
    waitForArm();
    if (!MoveArm(materialTransferLayout.approachHeight, -1, -1, -1,
                 materialTransferLayout.moveSpeed)) return false;
    waitForArm();
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
    if (!pickAt(materialTransferLayout.disc)
            || !placeAt(destination, destination.high)) return false;
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
    if (!pickAt(source) || !placeAt(destination, destination.high)) return false;
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
        if (!pickAt(materialTransferLayout.cargo[i])
                || !placeAt(materialTransferLayout.workArea[i], releaseHeights[i])) return false;
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
        if (!pickAt(materialTransferLayout.cargo[i])
                || !placeAt(materialTransferLayout.workArea[i], releaseHeights[i])) return false;
        cargoPlatforms[i].material = MATERIAL_NONE;
    }
    return true;
}

bool PrepareLidarScanPose(uint8_t startZoneCode) {

    switch (startZoneCode) {
        case 1:
            Serial.println("[LidarPose] start zone 1");
            GotoPose(125, 0, 0, true);
            MoveArm(150, 100, -1, -1, 150);
            waitForArm(3);
            GotoPose(0, 100, 0, true);
            MoveArm(-1, 100, 45, -1, 150);
            waitForArm(2);
            MoveArm(0, 100, -1, -1, 150);

            break;

        case 2:
            Serial.println("[LidarPose] start zone 2");
            GotoPose(125, 0, 0, true);
            MoveArm(150, 100, -1, -1, 150);
            waitForArm(3);
            GotoPose(0, -100, 0, true);
            MoveArm(0, 100, 135, -1, 150);
            break;

        default:
            Serial.println("[LidarPose] ERR: start zone must be 1 or 2");
            return false;
    }

    // 只有底盘和全部机械臂动作均执行完毕后才返回 true；调用方随后发送 OK 应答。
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
    if (!pickAt(source) || !placeAt(destination, secondLayerHeight)) return false;
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
        if (!pickAt(materialTransferLayout.cargo[i])
                || !placeAt(materialTransferLayout.workArea[workAreaCode - 1], releaseHeights[i])) return false;
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
