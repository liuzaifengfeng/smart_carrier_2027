#include "runtime_parameters.h"
#include "chassis.h"
#include "serial_frame.h"
#include "material_transfer.h"
#include "generated_parameter_defaults.h"
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <string.h>

extern uint32_t ALIGN_PID_MAX_SPEED_RPM;
extern float NODE_PATH_SPEED_MM_S;
extern float NODE_PATH_ACCEL_MM_S2;
extern float NODE_PATH_TURN_DEG_S;
extern float NODE_PATH_TURN_ACCEL_DEG_S2;
extern float NODE_PATH_HEADING_DEADBAND_DEG;
extern uint32_t SCAN_APPROACH_SPEED_RPM;
extern uint32_t SCAN_SEARCH_SPEED_RPM;

namespace {
constexpr size_t MAX_PARAMETERS = 400; // 包含角点独立 PID 的12项参数，并预留空间。
struct Parameter {
    char name[80];
    void *address;
    float minimum, maximum;
    bool integer;
};
Parameter parameters[MAX_PARAMETERS];
float staged[MAX_PARAMETERS];
bool changed[MAX_PARAMETERS];
size_t count = 0;
bool initialized = false;
bool registrationFailed = false;
bool transaction = false;
uint32_t transactionId = 0;

float readValue(const Parameter &p) {
    return p.integer ? static_cast<float>(*static_cast<uint32_t *>(p.address))
                     : *static_cast<float *>(p.address);
}

void add(const char *name, void *address, float minimum, float maximum, bool integer) {
    if (count == MAX_PARAMETERS || strlen(name) >= sizeof(parameters[0].name)) {
        registrationFailed = true;
        return;
    }
    Parameter &p = parameters[count++];
    strcpy(p.name, name);
    p.address = address;
    p.minimum = minimum;
    p.maximum = maximum;
    p.integer = integer;
}

void registerPose(const char *name, MaterialStationPose &pose) {
    char label[80];
    snprintf(label, sizeof(label), "%s/高度(mm)", name);
    RegisterRuntimeFloat(label, pose.high, 0, ARM_HEIGHT_LIMIT_MM);
    snprintf(label, sizeof(label), "%s/伸出(mm)", name);
    RegisterRuntimeFloat(label, pose.length, 0, 170);
    snprintf(label, sizeof(label), "%s/转台(deg)", name);
    RegisterRuntimeFloat(label, pose.turretAngle,
                         TURRET_CABLE_MIN_DEG, TURRET_CABLE_MAX_DEG);
}

void registerTransferDelays(const char *prefix, MaterialTransferDelays &delays) {
    const char *const steps[] = {"取料平移", "取料稳定", "取料转向", "取料下降", "取料夹紧", "取料抬升",
                                "放料平移", "放料稳定", "放料转向", "放料下降", "放料松爪", "放料抬升"};
    uint32_t *const values[] = {&delays.pickAbove.translateMs, &delays.pickAbove.settleMs,
        &delays.pickAbove.rotateMs, &delays.pickDescendMs, &delays.pickGripMs, &delays.pickLiftMs,
        &delays.placeAbove.translateMs, &delays.placeAbove.settleMs, &delays.placeAbove.rotateMs,
        &delays.placeDescendMs, &delays.placeReleaseMs, &delays.placeLiftMs};
    for (size_t i = 0; i < 12; ++i) {
        char name[80];
        snprintf(name, sizeof(name), "%s/%s(ms)", prefix, steps[i]);
        RegisterRuntimeUInt(name, *values[i], 0, 60000);
    }
}

void registerMaterialDelays() {
    registerTransferDelays("动作组延时/公共", commonMaterialDelays);
    for (uint8_t c = 0; c < MATERIAL_STATION_COUNT; ++c) {
        for (uint8_t w = 0; w < MATERIAL_STATION_COUNT; ++w) {
            char prefix[80];
            snprintf(prefix, sizeof(prefix), "动作组延时/载物台%u到工作区%u", c + 1, w + 1);
            registerTransferDelays(prefix, cargoToWorkAreaDelays[c][w]);
        }
    }
    RegisterRuntimeUInt("动作组延时/圆盘/预抓(ms)", materialAuxiliaryDelays.discApproachMs, 0, 60000);
    RegisterRuntimeUInt("动作组延时/圆盘/夹紧(ms)", materialAuxiliaryDelays.discGripMs, 0, 60000);
    RegisterRuntimeUInt("动作组延时/圆盘/抬升(ms)", materialAuxiliaryDelays.discLiftMs, 0, 60000);
    RegisterRuntimeUInt("动作组延时/扫码准备1/抬升(ms)", materialAuxiliaryDelays.lidarZone1LiftMs, 0, 60000);
    RegisterRuntimeUInt("动作组延时/扫码准备1/转向(ms)", materialAuxiliaryDelays.lidarZone1TurnMs, 0, 60000);
    RegisterRuntimeUInt("动作组延时/扫码准备2/抬升(ms)", materialAuxiliaryDelays.lidarZone2LiftMs, 0, 60000);
    RegisterRuntimeUInt("动作组延时/扫码准备2/转向(ms)", materialAuxiliaryDelays.lidarZone2TurnMs, 0, 60000);
}

void registerReverseMaterialDelays() {
    for (uint8_t w = 0; w < MATERIAL_STATION_COUNT; ++w) {
        for (uint8_t c = 0; c < MATERIAL_STATION_COUNT; ++c) {
            char prefix[80];
            snprintf(prefix, sizeof(prefix), "动作组延时/工作区%u到载物台%u", w + 1, c + 1);
            registerTransferDelays(prefix, workAreaToCargoDelays[w][c]);
        }
    }
}

// 所有搬运速度共用同一校验：正数且可表示为float，无人为速度上限。
void registerArmSpeed(const char *name, float &value) {
    RegisterRuntimeFloat(name, value, 1, FLT_MAX);
}

void initialize() {
    if (initialized) return;
    initialized = true;
    registerPose("搬运/圆盘", materialTransferLayout.disc);
    for (uint8_t i = 0; i < MATERIAL_STATION_COUNT; ++i) {
        char name[64];
        snprintf(name, sizeof(name), "搬运/载物台%u", i + 1);
        registerPose(name, materialTransferLayout.cargo[i]);
        snprintf(name, sizeof(name), "搬运/工作区%u", i + 1);
        registerPose(name, materialTransferLayout.workArea[i]);
    }
    RegisterRuntimeFloat("搬运/安全高度(mm)", materialTransferLayout.approachHeight, 0, ARM_HEIGHT_LIMIT_MM);
    RegisterRuntimeFloat("搬运/二层高度增量(mm)", materialTransferLayout.secondLayerOffset, 0, ARM_HEIGHT_LIMIT_MM);
    registerArmSpeed("搬运/速度", materialTransferLayout.moveSpeed);
    RegisterRuntimeFloat("搬运/夹爪张开(deg)", materialTransferLayout.clawOpenAngle, -360, 360);
    RegisterRuntimeFloat("搬运/夹爪夹紧(deg)", materialTransferLayout.clawClosedAngle, -360, 360);
    RegisterRuntimeUInt("搬运/等待时间(ms)", materialTransferLayout.motionWaitMs, 1, 60000);
    registerPose("搬运/圆盘预抓", materialMotionSteps.discApproach);
    RegisterRuntimeFloat("搬运/圆盘预抓/张爪(deg)", materialMotionSteps.discOpenAngle, -360, 360);
    RegisterRuntimeFloat("搬运/圆盘提起/高度(mm)", materialMotionSteps.discLiftHeight, 0, ARM_HEIGHT_LIMIT_MM);
    registerArmSpeed("搬运/速度/圆盘预抓", materialMotionSteps.discApproachSpeed);
    registerArmSpeed("搬运/速度/圆盘夹紧", materialMotionSteps.discGripSpeed);
    registerArmSpeed("搬运/速度/圆盘提起", materialMotionSteps.discLiftSpeed);
    registerArmSpeed("搬运/速度/移至工位平移", materialMotionSteps.approachTranslateSpeed);
    registerArmSpeed("搬运/速度/移至工位转向", materialMotionSteps.approachRotateSpeed);
    registerArmSpeed("搬运/速度/取料下降", materialMotionSteps.pickDescendSpeed);
    registerArmSpeed("搬运/速度/取料夹紧", materialMotionSteps.pickGripSpeed);
    registerArmSpeed("搬运/速度/取料提起", materialMotionSteps.pickLiftSpeed);
    registerArmSpeed("搬运/速度/放料下降", materialMotionSteps.placeDescendSpeed);
    registerArmSpeed("搬运/速度/放料松爪", materialMotionSteps.placeReleaseSpeed);
    registerArmSpeed("搬运/速度/放料提起", materialMotionSteps.placeLiftSpeed);
    RegisterChassisParameters();
    RegisterRuntimeFloat("底盘/节点导航速度(mm/s)", NODE_PATH_SPEED_MM_S, 10, 5000);
    RegisterRuntimeUInt("底盘/扫码接近轮速(RPM)", SCAN_APPROACH_SPEED_RPM, 1, 5000);
    RegisterRuntimeUInt("底盘/扫码往返轮速(RPM)", SCAN_SEARCH_SPEED_RPM, 1, 5000);
    RegisterRuntimeUInt("对齐/最大轮速(RPM)", ALIGN_PID_MAX_SPEED_RPM, 1, 5000);
    // 追加注册，保留原有参数索引；默认值来自 material_transfer.cpp。
    registerMaterialDelays();
    // 只追加，不改变既有参数索引。
    RegisterRuntimeFloat("底盘/节点导航平移加速度(mm/s²)", NODE_PATH_ACCEL_MM_S2, 1, 4000);
    RegisterRuntimeFloat("底盘/节点导航转向速度(°/s)", NODE_PATH_TURN_DEG_S, 10, 500);
    RegisterRuntimeFloat("底盘/节点导航转向加速度(°/s²)", NODE_PATH_TURN_ACCEL_DEG_S2, 30, 720);
    RegisterRuntimeFloat("底盘/节点导航独立纠偏死区(°)", NODE_PATH_HEADING_DEADBAND_DEG, 0, 8);
    // 反向九组最后追加，保留公共、正向、辅助及底盘参数的既有编号。
    registerReverseMaterialDelays();
    // 角点 PID 最后追加，保留既有参数编号。
    RegisterCornerAlignmentParameters();
}

bool parseUnsigned(const char *text, uint32_t &value) {
    if (!text || !*text) return false;
    for (const char *p = text; *p; ++p) if (*p < '0' || *p > '9') return false;
    char *end;
    const unsigned long long number = strtoull(text, &end, 10);
    if (*end || number > UINT32_MAX) return false;
    value = static_cast<uint32_t>(number);
    return true;
}

void reply(const char *action, uint32_t id, const char *status, const char *detail) {
    if (strcmp(status, "OK") == 0) {
        Serial.printf("{RSP,CFG,%s,OK,%lu}\n", action, static_cast<unsigned long>(id));
    } else {
        Serial.printf("{RSP,CFG,%s,ERR,%lu,%s}\n", action,
                      static_cast<unsigned long>(id), detail);
    }
}
} // namespace

void RegisterRuntimeFloat(const char *name, float &value, float minimum, float maximum) {
    add(name, &value, minimum, maximum, false);
}
void RegisterRuntimeUInt(const char *name, uint32_t &value, float minimum, float maximum) {
    add(name, &value, minimum, maximum, true);
}

bool ApplyCompiledParameterDefaults() {
    initialize();
    if (registrationFailed) {
        Serial.println("[CFG] ERR: parameter registry overflow");
        return false;
    }
    // 先检查全部条目，再写入任何变量；避免错误文件造成部分参数生效。
    for (size_t j = 0; j < COMPILED_PARAMETER_DEFAULT_COUNT; ++j) {
        const auto &entry = COMPILED_PARAMETER_DEFAULTS[j];
        size_t i = 0;
        while (i < count && strcmp(parameters[i].name, entry.name) != 0) ++i;
        if (i == count || !isfinite(entry.value)
                || entry.value < parameters[i].minimum
                || entry.value > parameters[i].maximum
                || (parameters[i].integer && (floor(entry.value) != entry.value
                    || entry.value < 0 || entry.value > UINT32_MAX))) {
            Serial.printf("[CFG] ERR: invalid compiled parameter: %s\n", entry.name);
            return false;
        }
    }
    for (size_t j = 0; j < COMPILED_PARAMETER_DEFAULT_COUNT; ++j) {
        const auto &entry = COMPILED_PARAMETER_DEFAULTS[j];
        for (size_t i = 0; i < count; ++i) {
            Parameter &p = parameters[i];
            if (strcmp(p.name, entry.name) != 0) continue;
            if (p.integer) *static_cast<uint32_t *>(p.address) = static_cast<uint32_t>(entry.value);
            else *static_cast<float *>(p.address) = static_cast<float>(entry.value);
            break;
        }
    }
    Serial.printf("[CFG] Applied %u compiled defaults from robot_parameters.json\n",
                  static_cast<unsigned>(COMPILED_PARAMETER_DEFAULT_COUNT));
    return true;
}

bool HandleRuntimeParameters(const char *const *fields, size_t size, bool writable) {
    initialize();
    const char *action = size > 0 ? fields[0] : "FRAME";
    uint32_t id = 0;
    if (size < 2 || size > 4 || !parseUnsigned(fields[1], id) || id == 0) {
        reply(action, 0, "ERR", "FORMAT");
        return true;
    }
    const bool correctSize = (strcmp(action, "GET") == 0 && size == 2)
        || (strcmp(action, "BEGIN") == 0 && size == 2)
        || (strcmp(action, "SET") == 0 && size == 4)
        || (strcmp(action, "COMMIT") == 0 && size == 2);
    if (!correctSize) {
        reply(action, id, "ERR", "FORMAT");
        return true;
    }
    if (registrationFailed) { reply(action, id, "ERR", "REGISTRY"); return true; }
    if (strcmp(fields[0], "GET") == 0 && size == 2) {
        transaction = false;
        for (size_t i = 0; i < count; ++i) {
            const Parameter &p = parameters[i];
            Serial.printf("{RSP,CFG,GET,VALUE,%lu,%u,%s,%.9g,%.9g,%.9g,%u}\n",
                static_cast<unsigned long>(id), static_cast<unsigned>(i), p.name,
                readValue(p), p.minimum, p.maximum, p.integer ? 1 : 0);
            // 逐帧排空串口，避免扩充后的参数表连续发送时被 USB/UART 桥丢弃。
            Serial.flush();
            delay(2);
        }
        Serial.printf("{RSP,CFG,GET,END,%lu,%u}\n", static_cast<unsigned long>(id), static_cast<unsigned>(count));
        return true;
    }
    if (!writable) {
        transaction = false;
        reply(action, id, "ERR", "BUSY_OR_MODE");
        return true;
    }
    if (strcmp(fields[0], "BEGIN") == 0 && size == 2) {
        for (size_t i = 0; i < count; ++i) { staged[i] = readValue(parameters[i]); changed[i] = false; }
        transaction = true;
        transactionId = id;
        reply(action, id, "OK", "BEGIN");
    } else if (!transaction || transactionId != id) {
        reply(action, id, "ERR", "TRANSACTION");
    } else if (strcmp(fields[0], "SET") == 0 && size == 4) {
        uint32_t index = 0;
        float value = 0;
        if (!parseUnsigned(fields[2], index) || index >= count
                || !serialFloat(fields[3], value, parameters[index].minimum,
                                parameters[index].maximum)
                || (parameters[index].integer && floorf(value) != value)) {
            transaction = false;
            reply(action, id, "ERR", "RANGE");
        } else {
            staged[index] = value;
            changed[index] = true;
            Serial.printf("{RSP,CFG,SET,OK,%lu,%lu}\n", static_cast<unsigned long>(id), static_cast<unsigned long>(index));
        }
    } else if (strcmp(fields[0], "COMMIT") == 0 && size == 2) {
        // 本函数在串口任务中运行且持有对齐锁；提交期间没有机械动作。
        for (size_t i = 0; i < count; ++i) {
            if (!changed[i]) continue;
            Parameter &p = parameters[i];
            if (p.integer) *static_cast<uint32_t *>(p.address) = static_cast<uint32_t>(staged[i]);
            else *static_cast<float *>(p.address) = staged[i];
        }
        ResetDiscAlignmentPid();
        transaction = false;
        reply(action, id, "OK", "COMMIT");
    } else {
        transaction = false;
        reply(action, id, "ERR", "FORMAT");
    }
    return true;
}
