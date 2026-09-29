#pragma once

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// Serial0 协议 v2：一行一帧。此处只负责结构与数值解析，不执行任何动作。
constexpr size_t SERIAL_FRAME_BYTES = 160;
constexpr size_t SERIAL_MAX_FIELDS = 32;

struct SerialFrame {
    char *fields[SERIAL_MAX_FIELDS];
    size_t count = 0;

    bool is(const char *category, const char *action, size_t arguments) const {
        return count == arguments + 3 && strcmp(fields[1], category) == 0
            && strcmp(fields[2], action) == 0;
    }
    bool starts(const char *category, const char *action) const {
        return count >= 3 && strcmp(fields[1], category) == 0
            && strcmp(fields[2], action) == 0;
    }
};

inline bool parseSerialFrame(char *line, SerialFrame &frame) {
    frame.count = 0;
    if (!line || line[0] != '{') return false;
    const size_t length = strlen(line);
    if (length < 7 || line[length - 1] != '}') return false;
    line[length - 1] = '\0';
    frame.fields[frame.count++] = line + 1;
    for (char *p = line + 1; *p; ++p) {
        if (static_cast<uint8_t>(*p) < 0x21 || static_cast<uint8_t>(*p) > 0x7e
                || *p == '{' || *p == '}') return false;
        if (*p != ',') continue;
        if (frame.count == SERIAL_MAX_FIELDS) return false;
        *p = '\0';
        frame.fields[frame.count++] = p + 1;
    }
    for (size_t i = 0; i < frame.count; ++i) {
        if (!*frame.fields[i]) return false;
    }
    return frame.count >= 3 && strcmp(frame.fields[0], "CMD") == 0;
}

inline bool serialFloat(const char *text, float &value, float minimum, float maximum) {
    if (!text || !*text) return false;
    // 只接受十进制（可带科学计数法）；strtof 自身也会接受十六进制，故先查语法。
    const char *p = text;
    if (*p == '-' || *p == '+') ++p;
    bool digits = false;
    while (*p >= '0' && *p <= '9') { digits = true; ++p; }
    if (*p == '.') {
        ++p;
        while (*p >= '0' && *p <= '9') { digits = true; ++p; }
    }
    if (!digits) return false;
    if (*p == 'e' || *p == 'E') {
        ++p;
        if (*p == '-' || *p == '+') ++p;
        const char *exponent = p;
        while (*p >= '0' && *p <= '9') ++p;
        if (p == exponent) return false;
    }
    if (*p != '\0') return false;
    char *end = nullptr;
    value = strtof(text, &end);
    return end != text && *end == '\0' && isfinite(value)
        && value >= minimum && value <= maximum;
}

inline bool serialUInt(const char *text, uint32_t &value, uint32_t maximum) {
    if (!text || !*text) return false;
    uint32_t result = 0;
    for (const char *p = text; *p; ++p) {
        if (*p < '0' || *p > '9') return false;
        const uint32_t digit = static_cast<uint32_t>(*p - '0');
        if (digit > maximum || result > (maximum - digit) / 10) return false;
        result = result * 10 + digit;
    }
    value = result;
    return true;
}

inline void serialError(const SerialFrame &frame, const char *reason) {
    Serial.printf("{RSP,%s,%s,ERR,%s}\n", frame.fields[1], frame.fields[2], reason);
}
