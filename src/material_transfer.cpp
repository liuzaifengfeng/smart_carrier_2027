#include "material_transfer.h"

#include <math.h>

// ================= 实车标定区 =================
// NAN 表示尚未标定。请将圆盘、三个载物台和三个工作区位置替换为实测值。
// 在全部位置填写完成前，MaterialTransferPosesReady() 返回 false，机械臂不会动作。
MaterialTransferLayout materialTransferLayout = {
    {NAN, NAN, NAN},
    {
        {NAN, NAN, NAN},
        {NAN, NAN, NAN},
        {NAN, NAN, NAN},
    },
    {
        {NAN, NAN, NAN},
        {NAN, NAN, NAN},
        {NAN, NAN, NAN},
    },
    150.0f, // approachHeight
    30.0f,  // secondLayerOffset，按物料实际高度修改
    100.0f, // moveSpeed
    0.0f,   // clawOpenAngle
    30.0f,  // clawClosedAngle
    1000,   // motionWaitMs
};

CargoPlatform cargoPlatforms[MATERIAL_STATION_COUNT] = {
    {MATERIAL_NONE}, {MATERIAL_NONE}, {MATERIAL_NONE}
};

namespace {

bool isMaterialCodeValid(int code) {
    return code >= MATERIAL_RED && code <= MATERIAL_LIGHT_BLUE;
}

bool isStationCodeValid(int code) {
    return code >= 1 && code <= MATERIAL_STATION_COUNT;
}

bool isPoseValid(const MaterialStationPose &pose) {
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
    vTaskDelay(pdMS_TO_TICKS(materialTransferLayout.motionWaitMs));
}

void moveAbove(const MaterialStationPose &pose, float clawAngle) {
    MoveArm(materialTransferLayout.approachHeight, pose.length,
            pose.turretAngle, clawAngle, materialTransferLayout.moveSpeed);
    waitForArm();
}

void pickAt(const MaterialStationPose &pose) {
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

void placeAt(const MaterialStationPose &pose, float targetHeight) {
    moveAbove(pose, materialTransferLayout.clawClosedAngle);
    MoveArm(targetHeight, -1, -1, materialTransferLayout.clawClosedAngle,
            materialTransferLayout.moveSpeed);
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
    if (!isLayoutParameterValid() || !isPoseValid(source)
            || !isPoseValid(destination)) {
        Serial.println("[Material] ERR: transfer poses are not calibrated");
        return false;
    }
    return true;
}

bool validateCodeArrays(const int *first, const int *second,
                        bool firstIsMaterial) {
    if (first == nullptr) {
        Serial.println("[Material] ERR: null task code array");
        return false;
    }
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        if ((firstIsMaterial && !isMaterialCodeValid(first[i]))
                || (!firstIsMaterial && !isStationCodeValid(first[i]))
                || (second != nullptr && !isStationCodeValid(second[i]))) {
            Serial.println("[Material] ERR: invalid task code");
            return false;
        }
    }
    const int *positions = firstIsMaterial ? second : first;
    if (positions != nullptr
            && (positions[0] == positions[1]
                || positions[0] == positions[2]
                || positions[1] == positions[2])) {
        Serial.println("[Material] ERR: duplicate work-area position code");
        return false;
    }
    return true;
}

} // namespace

bool MaterialTransferPosesReady() {
    if (!isLayoutParameterValid() || !isPoseValid(materialTransferLayout.disc)) {
        return false;
    }
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        if (!isPoseValid(materialTransferLayout.cargo[i])
                || !isPoseValid(materialTransferLayout.workArea[i])) {
            return false;
        }
    }
    return true;
}

bool MoveDiscToCargo(uint8_t materialCode, uint8_t cargoCode) {
    if (!isMaterialCodeValid(materialCode) || !isStationCodeValid(cargoCode)) {
        Serial.println("[Material] ERR: invalid material/cargo code");
        return false;
    }
    CargoPlatform &cargo = cargoPlatforms[cargoCode - 1];
    if (cargo.material != MATERIAL_NONE) {
        Serial.println("[Material] ERR: cargo platform is occupied");
        return false;
    }
    const MaterialStationPose &destination =
        materialTransferLayout.cargo[cargoCode - 1];
    if (!validateAction(materialTransferLayout.disc, destination)) return false;

    Serial.printf("[Material] disc -> cargo %u, material %u\n",
                  cargoCode, materialCode);
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
    pickAt(source);
    placeAt(destination, destination.high);
    cargo.material = MATERIAL_NONE;
    return true;
}

bool MoveWorkAreaToCargo(uint8_t materialCode, uint8_t workAreaCode,
                         uint8_t cargoCode) {
    if (!isMaterialCodeValid(materialCode)
            || !isStationCodeValid(workAreaCode)
            || !isStationCodeValid(cargoCode)) {
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
    if (!validateCodeArrays(positionCodes, nullptr, false)
            || !MaterialTransferPosesReady()) return false;
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
