#include "runtime_parameters.h"
#include "material_transfer.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

extern uint32_t ALIGN_PID_MAX_SPEED_RPM;

namespace {
constexpr size_t MAX_PARAMETERS = 96;
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
    RegisterRuntimeFloat(label, pose.high, 0, 160);
    snprintf(label, sizeof(label), "%s/伸出(mm)", name);
    RegisterRuntimeFloat(label, pose.length, 0, 170);
    snprintf(label, sizeof(label), "%s/转台(deg)", name);
    RegisterRuntimeFloat(label, pose.turretAngle, -360, 360);
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
    RegisterRuntimeFloat("搬运/安全高度(mm)", materialTransferLayout.approachHeight, 0, 160);
    RegisterRuntimeFloat("搬运/二层高度增量(mm)", materialTransferLayout.secondLayerOffset, 0, 160);
    // MoveArm 中舵机时间使用 (300-speed)*3，速度不得超过 300。
    RegisterRuntimeFloat("搬运/速度", materialTransferLayout.moveSpeed, 1, 300);
    RegisterRuntimeFloat("搬运/夹爪张开(deg)", materialTransferLayout.clawOpenAngle, -360, 360);
    RegisterRuntimeFloat("搬运/夹爪夹紧(deg)", materialTransferLayout.clawClosedAngle, -360, 360);
    RegisterRuntimeUInt("搬运/等待时间(ms)", materialTransferLayout.motionWaitMs, 1, 60000);
    RegisterChassisParameters();
    RegisterRuntimeUInt("对齐/最大轮速(RPM)", ALIGN_PID_MAX_SPEED_RPM, 1, 5000);
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

void reply(uint32_t id, const char *status, const char *detail) {
    Serial.printf("{CFG:%s,%lu,%s}\n", status, static_cast<unsigned long>(id), detail);
}
} // namespace

void RegisterRuntimeFloat(const char *name, float &value, float minimum, float maximum) {
    add(name, &value, minimum, maximum, false);
}
void RegisterRuntimeUInt(const char *name, uint32_t &value, float minimum, float maximum) {
    add(name, &value, minimum, maximum, true);
}

bool HandleRuntimeParameters(const char *frame, bool writable) {
    if (strncmp(frame, "{CFG:", 5) != 0) return false;
    initialize();
    char buffer[100];
    const size_t length = strlen(frame);
    if (length >= sizeof(buffer) || length < 8 || frame[length - 1] != '}') {
        reply(0, "ERR", "FORMAT");
        return true;
    }
    memcpy(buffer, frame + 5, length - 6);
    buffer[length - 6] = '\0';
    char *fields[5];
    size_t size = 0;
    char *cursor = buffer;
    // 保留空字段，使缺失参数和多余逗号不能被当作合法命令。
    while (cursor && size < 5) {
        fields[size++] = cursor;
        char *comma = strchr(cursor, ',');
        if (comma) *comma++ = '\0';
        cursor = comma;
    }
    uint32_t id = 0;
    if (cursor || size < 2 || !parseUnsigned(fields[1], id) || id == 0) {
        reply(0, "ERR", "FORMAT");
        return true;
    }
    if (registrationFailed) { reply(id, "ERR", "REGISTRY"); return true; }
    if (strcmp(fields[0], "GET") == 0 && size == 2) {
        transaction = false;
        for (size_t i = 0; i < count; ++i) {
            const Parameter &p = parameters[i];
            Serial.printf("{CFG:VALUE,%lu,%u,%s,%.9g,%.9g,%.9g,%u}\n",
                static_cast<unsigned long>(id), static_cast<unsigned>(i), p.name,
                readValue(p), p.minimum, p.maximum, p.integer ? 1 : 0);
        }
        Serial.printf("{CFG:END,%lu,%u}\n", static_cast<unsigned long>(id), static_cast<unsigned>(count));
        return true;
    }
    if (!writable) {
        transaction = false;
        reply(id, "ERR", "BUSY_OR_MODE");
        return true;
    }
    if (strcmp(fields[0], "BEGIN") == 0 && size == 2) {
        for (size_t i = 0; i < count; ++i) { staged[i] = readValue(parameters[i]); changed[i] = false; }
        transaction = true;
        transactionId = id;
        reply(id, "OK", "BEGIN");
    } else if (!transaction || transactionId != id) {
        reply(id, "ERR", "TRANSACTION");
    } else if (strcmp(fields[0], "SET") == 0 && size == 4) {
        uint32_t index = 0;
        char *end = nullptr;
        const float value = strtof(fields[3], &end);
        if (!parseUnsigned(fields[2], index) || index >= count || end == fields[3]
                || *end || !isfinite(value) || value < parameters[index].minimum
                || value > parameters[index].maximum
                || (parameters[index].integer && floorf(value) != value)) {
            transaction = false;
            reply(id, "ERR", "RANGE");
        } else {
            staged[index] = value;
            changed[index] = true;
            Serial.printf("{CFG:OK,%lu,SET,%lu}\n", static_cast<unsigned long>(id), static_cast<unsigned long>(index));
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
        reply(id, "OK", "COMMIT");
    } else {
        transaction = false;
        reply(id, "ERR", "FORMAT");
    }
    return true;
}
