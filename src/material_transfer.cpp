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
        {5, 65, 49}, // 粗加工区/暂存区的 1 号位置
        {5, 0, 86}, // 粗加工区/暂存区的 2 号位置
        {5, 43, 125}, // 粗加工区/暂存区的 3 号位置
    },
    150.0f, // approachHeight，按物料实际高度修改
    60.0f,  // secondLayerOffset，按物料实际高度修改
    150.0f, // moveSpeed
    60.0f,   // clawOpenAngle
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
    return isfinite(pose.high) && pose.high >= 0.0f && pose.high <= 160.0f
        && isfinite(pose.length) && pose.length >= 0.0f && pose.length <= 170.0f
        && isfinite(pose.turretAngle)
        && pose.turretAngle >= -360.0f && pose.turretAngle <= 360.0f;
}

bool isLayoutParameterValid() {
    return isfinite(materialTransferLayout.approachHeight)
        && materialTransferLayout.approachHeight >= 0.0f
        && materialTransferLayout.approachHeight <= 160.0f
        && isfinite(materialTransferLayout.secondLayerOffset)
        && materialTransferLayout.secondLayerOffset >= 0.0f
        && isfinite(materialTransferLayout.moveSpeed)
        && materialTransferLayout.moveSpeed > 0.0f
        && isfinite(materialTransferLayout.clawOpenAngle)
        && isfinite(materialTransferLayout.clawClosedAngle)
        && materialTransferLayout.motionWaitMs > 0;
}

void waitForArm() {
    // MoveArm 发出指令后等待机械结构运动和稳定。
    vTaskDelay(pdMS_TO_TICKS(materialTransferLayout.motionWaitMs));
}

// 移动到目标上方
void moveAbove(const MaterialStationPose &pose, float clawAngle) {
    // 先升到安全高度，再转向目标并伸出，防止横向移动时碰到物料或车体。
    MoveArm(materialTransferLayout.approachHeight, pose.length,
            -1, clawAngle, materialTransferLayout.moveSpeed);
    waitForArm();    
    waitForArm();
    MoveArm(materialTransferLayout.approachHeight, pose.length,
            pose.turretAngle, clawAngle, materialTransferLayout.moveSpeed);
    waitForArm();
}

// 抓取物料
void pickAt(const MaterialStationPose &pose) {
    // 抓取顺序：张开夹爪到目标上方 -> 下降 -> 夹紧 -> 提升到安全高度。
    moveAbove(pose, materialTransferLayout.clawOpenAngle);
    MoveArm(pose.high, -1, -1, materialTransferLayout.clawOpenAngle,
            materialTransferLayout.moveSpeed);
    waitForArm();
    MoveArm(-1, -1, -1, materialTransferLayout.clawClosedAngle,
            materialTransferLayout.moveSpeed);
    waitForArm();
    MoveArm(materialTransferLayout.approachHeight, -1, -1, -1,
            materialTransferLayout.moveSpeed);
    waitForArm();
}

// 放置物料
void placeAt(const MaterialStationPose &pose, float targetHeight) {
    // 放置顺序：夹持物料到目标上方 -> 下降 -> 张开夹爪 -> 提升到安全高度。
    moveAbove(pose, materialTransferLayout.clawClosedAngle);
    MoveArm(targetHeight, -1, -1, materialTransferLayout.clawClosedAngle,
            materialTransferLayout.moveSpeed);
    waitForArm();
    waitForArm();
    MoveArm(-1, -1, -1, materialTransferLayout.clawOpenAngle,
            materialTransferLayout.moveSpeed);
    waitForArm();
    MoveArm(materialTransferLayout.approachHeight, -1, -1, -1,
            materialTransferLayout.moveSpeed);
    waitForArm();
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
    pickAt(materialTransferLayout.disc);
    placeAt(destination, destination.high);
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
    pickAt(source);
    placeAt(destination, destination.high);
    cargo.material = MATERIAL_NONE;
    return true;
}

bool DemoCargoToRoughArea() {
    // 依次完成三次搬运：载物台 1 -> 粗加工区 3、2 -> 2、3 -> 1（第一层）。
    // 每次完整执行抓取、搬运、放置和抬升后，才开始下一次；本函数不负责底盘导航。
    constexpr uint8_t materialCode = MATERIAL_RED; // 物料颜色码：1~6
    static_assert(materialCode >= MATERIAL_RED && materialCode <= MATERIAL_LIGHT_BLUE,
                  "Invalid demo material code");

    for (uint8_t cargoCode = 1; cargoCode <= MATERIAL_STATION_COUNT; ++cargoCode) {
        // 编号为 1~3，目标位置反向对应为 3~1；数组下标需要减 1。
        const uint8_t workAreaCode = MATERIAL_STATION_COUNT + 1 - cargoCode;
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
    pickAt(source);
    placeAt(destination, destination.high);
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
    if (secondLayerHeight > 160.0f) {
        Serial.println("[Material] ERR: invalid second-layer height");
        return false;
    }

    Serial.printf("[Material] cargo %u -> work area %u layer 2, material %u\n",
                  cargoCode, workAreaCode, cargo.material);
    pickAt(source);
    placeAt(destination, secondLayerHeight);
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

bool PlaceRoundToWorkArea(const int positionCodes[MATERIAL_STATION_COUNT]) {
    // positionCodes[i] 表示第 i+1 号载物台上的物料应放到哪个区域位置。
    if (!validateCodeArrays(positionCodes, nullptr, false)
            || !MaterialTransferPosesReady()) return false;
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        if (cargoPlatforms[i].material == MATERIAL_NONE) {
            Serial.println("[Material] ERR: cargo platform is empty");
            return false;
        }
    }
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        if (!MoveCargoToWorkArea(i + 1, positionCodes[i])) return false;
    }
    return true;
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
    // 与第一层放置流程相同，但最终下降到第二层高度。
    if (!validateCodeArrays(positionCodes, nullptr, false) || !MaterialTransferPosesReady()) return false;
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        if (cargoPlatforms[i].material == MATERIAL_NONE) {
            Serial.println("[Material] ERR: cargo platform is empty");
            return false;
        }
    }
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        if (!StackCargoToWorkArea(i + 1, positionCodes[i])) return false;
    }
    return true;
}
