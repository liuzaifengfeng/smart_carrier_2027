// 串口帧的结构和基础解析。
// 分隔字段与数值转换在进业务分发前后分层处理，便于定位收帧问题和命令参数问题。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。

#include <Arduino.h>  // 引入 Arduino.h 中的声明和定义。
#include <math.h>  // 引入 math.h 中的声明和定义。
#include <stdlib.h>  // 引入 stdlib.h 中的声明和定义。
#include <string.h>  // 引入 string.h 中的声明和定义。

// Serial0 协议 v2：一行一帧。此处只负责结构与数值解析，不执行任何动作。
constexpr size_t SERIAL_FRAME_BYTES = 160;  // 将 160 写入或初始化 SERIAL_FRAME_BYTES（SERIAL帧字节）。
constexpr size_t SERIAL_MAX_FIELDS = 32;  // 将 32 写入或初始化 SERIAL_MAX_FIELDS（SERIAL最大FIELDS）。

struct SerialFrame {  // 定义 SerialFrame（Serial帧） 的类型成员或作用域。
    char *fields[SERIAL_MAX_FIELDS];  // 声明 fields，类型和可用范围由所在声明决定。
    size_t count = 0;  // 将 0 写入或初始化 count（计数）。

    /**
     * @brief 判断串口帧类别、动作名及参数个数是否全部匹配。
     */
    bool is(const char *category, const char *action, size_t arguments) const {  // 定义函数或成员 的入口 is，参数由此处的签名确定。
        return count == arguments + 3 && strcmp(fields[1], category) == 0  // 结束当前函数并返回 count == arguments + 3 && strcmp(fields[1], category) …。
            && strcmp(fields[2], action) == 0;  // 继续上一行的组合条件，本行提供 && strcmp(fields[2], action) == 0;。
    }  // 结束此函数或控制块。
    /**
     * @brief 判断串口帧类别和动作名是否匹配，不检查参数个数。
     */
    bool starts(const char *category, const char *action) const {  // 定义函数或成员 的入口 starts，参数由此处的签名确定。
        return count >= 3 && strcmp(fields[1], category) == 0  // 结束当前函数并返回 count >= 3 && strcmp(fields[1], category) == 0。
            && strcmp(fields[2], action) == 0;  // 继续上一行的组合条件，本行提供 && strcmp(fields[2], action) == 0;。
    }  // 结束此函数或控制块。
};  // 结束此类型定义。

/**
 * @brief 原地拆分串口帧并填写字段指针；frame字段依赖line缓冲区存活。
 * @return true 帧格式正确，false 解析失败。
 */
// 输入是可修改的行缓冲区，解析时用分隔符切出字段。
// 字段指针的有效期跟随该缓冲区，跨帧保存时要留意它会被接收任务复用。
inline bool parseSerialFrame(char *line, SerialFrame &frame) {  // 定义函数或成员 的入口 parseSerialFrame（parseSerial帧），参数由此处的签名确定。
    frame.count = 0;  // 将 0 写入或初始化 frame.count。
    if (!line || line[0] != '{') return false;  // 当 !line || line[0] != ' 成立时进入此分支，否则继续后续流程。
    const size_t length = strlen(line);  // 将 strlen(line) 写入或初始化 length（长度）。
    if (length < 7 || line[length - 1] != '}') return false;  // 若 length < 7 || line[length - 1] != '}'，立即返回 false。
    line[length - 1] = '\0';  // 将 '\0' 写入或初始化 line[length - 1]。
    frame.fields[frame.count++] = line + 1;  // 将 line + 1 写入或初始化 frame.fields[frame.count++]。
    for (char *p = line + 1; *p; ++p) {  // 按 char *p = line + 1; *p; ++p 组织循环的初值、继续条件和步进。
        if (static_cast<uint8_t>(*p) < 0x21 || static_cast<uint8_t>(*p) > 0x7e  // 当 static_cast<uint8_t>(*p) < 0x21 || static_cast<uint8_t… 成立时进入此分支，否则继续后续流程。
                || *p == '{' || *p == '}') return false;  // 继续上一行的组合条件，本行提供 || *p == '{' || *p == '}') return false;。
        if (*p != ',') continue;  // 若 *p != ','，直接进入下一轮循环。
        if (frame.count == SERIAL_MAX_FIELDS) return false;  // 若 frame.count == SERIAL_MAX_FIELDS，立即返回 false。
        *p = '\0';  // 将 '\0' 写入或初始化 p。
        frame.fields[frame.count++] = p + 1;  // 将 p + 1 写入或初始化 frame.fields[frame.count++]。
    }  // 结束此循环体。
    for (size_t i = 0; i < frame.count; ++i) {  // 按 size_t i = 0; i < frame.count; ++i 组织循环的初值、继续条件和步进。
        if (!*frame.fields[i]) return false;  // 若 !*frame.fields[i]，立即返回 false。
    }  // 结束此循环体。
    return frame.count >= 3 && strcmp(frame.fields[0], "CMD") == 0;  // 结束当前函数并返回 frame.count >= 3 && strcmp(frame.fields[0], "CMD") == 0。
}  // 结束此函数或控制块。

/**
 * @brief 解析有限浮点参数并按入口范围判断，成功时写入value；返回是否有效。
 */
inline bool serialFloat(const char *text, float &value, float minimum, float maximum) {  // 定义函数或成员 的入口 serialFloat，参数由此处的签名确定。
    if (!text || !*text) return false;  // 若 !text || !*text，立即返回 false。
    // 只接受十进制（可带科学计数法）；strtof 自身也会接受十六进制，故先查语法。
    const char *p = text;  // 将 text 写入或初始化 p。
    if (*p == '-' || *p == '+') ++p;  // 当 *p == '-' || *p == '+' 成立时进入此分支，否则继续后续流程。
    bool digits = false;  // 将 false 写入或初始化 digits。
    while (*p >= '0' && *p <= '9') { digits = true; ++p; }  // 当 *p >= '0' && *p <= '9' 成立时继续循环。
    if (*p == '.') {  // 当 *p == '.' 成立时进入此分支，否则继续后续流程。
        ++p;  // 续写当前声明或运算，提供 ++p;。
        while (*p >= '0' && *p <= '9') { digits = true; ++p; }  // 当 *p >= '0' && *p <= '9' 成立时继续循环。
    }  // 结束此条件分支。
    if (!digits) return false;  // 若 !digits，立即返回 false。
    if (*p == 'e' || *p == 'E') {  // 当 *p == 'e' || *p == 'E' 成立时进入此分支，否则继续后续流程。
        ++p;  // 续写当前声明或运算，提供 ++p;。
        if (*p == '-' || *p == '+') ++p;  // 当 *p == '-' || *p == '+' 成立时进入此分支，否则继续后续流程。
        const char *exponent = p;  // 将 p 写入或初始化 exponent。
        while (*p >= '0' && *p <= '9') ++p;  // 当 *p >= '0' && *p <= '9' 成立时继续循环。
        if (p == exponent) return false;  // 若 p == exponent，立即返回 false。
    }  // 结束此条件分支。
    if (*p != '\0') return false;  // 若 *p != '\0'，立即返回 false。
    char *end = nullptr;  // 将 nullptr 写入或初始化 end（终点）。
    value = strtof(text, &end);  // 将 strtof(text, &end) 写入或初始化 value（测值）。
    return end != text && *end == '\0' && isfinite(value)  // 结束当前函数并返回 end != text && *end == '\0' && isfinite(value)。
        && value >= minimum && value <= maximum;  // 继续上一行的组合条件，本行提供 && value >= minimum && value <= maximum;。
}  // 结束此函数或控制块。

/**
 * @brief 解析无符号整数参数并检查maximum，成功时写入value；返回是否有效。
 */
inline bool serialUInt(const char *text, uint32_t &value, uint32_t maximum) {  // 定义函数或成员 的入口 serialUInt，参数由此处的签名确定。
    if (!text || !*text) return false;  // 若 !text || !*text，立即返回 false。
    uint32_t result = 0;  // 将 0 写入或初始化 result（结果）。
    for (const char *p = text; *p; ++p) {  // 按 const char *p = text; *p; ++p 组织循环的初值、继续条件和步进。
        if (*p < '0' || *p > '9') return false;  // 若 *p < '0' || *p > '9'，立即返回 false。
        const uint32_t digit = static_cast<uint32_t>(*p - '0');  // 将 static_cast<uint32_t>(*p - '0') 写入或初始化 digit。
        if (digit > maximum || result > (maximum - digit) / 10) return false;  // 若 digit > maximum || result > (maximum - digit) / 10，立即返回 false。
        result = result * 10 + digit;  // 将 result * 10 + digit 写入或初始化 result（结果）。
    }  // 结束此循环体。
    value = result;  // 将 result 写入或初始化 value（测值）。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

/**
 * @brief 按收到的类别与动作发送ERR协议应答，reason为错误原因。
 */
inline void serialError(const SerialFrame &frame, const char *reason) {  // 定义函数或成员 的入口 serialError（serial错误），参数由此处的签名确定。
    Serial.printf("{RSP,%s,%s,ERR,%s}\n", frame.fields[1], frame.fields[2], reason);  // 调用 Serial.printf，传入 "{RSP,%s,%s,ERR,%s}\n", frame.fields[1], frame.fields[…。
}  // 结束此函数或控制块。
