// 任务码解析与数据组织。
// 任务中同时包含颜色和位置序列，使用时按轮次和用途选取对应字段。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。

#include <stddef.h>  // 引入 stddef.h 中的声明和定义。

// 两轮颜色段用 X/x 表示黑色，任务码统一使用编号 5。
inline void NormalizeScannerTaskCode(char *code) {
    for (size_t i = 0; code[i] != '\0'; ++i) {
        if ((i < 3 || (i >= 8 && i < 11)) && (code[i] == 'X' || code[i] == 'x')) {
            code[i] = '5';
        }
    }
}

// 每个数组按该轮任务中的三件物料索引。
// 粗加工位置来自对应轮的 pos；第二轮暂存叠放的位置映射在比赛流程中处理。
struct TaskCode {  // 定义 TaskCode（任务Code） 的类型成员或作用域。
    int round1_colors[3];  // 声明 round1_colors（轮次1colors），类型和可用范围由所在声明决定。
    int round1_pos[3];  // 声明 round1_pos（轮次1位置），类型和可用范围由所在声明决定。
    int round2_colors[3];  // 声明 round2_colors（轮次2colors），类型和可用范围由所在声明决定。
    int round2_pos[3];  // 声明 round2_pos（轮次2位置），类型和可用范围由所在声明决定。
    bool valid;  // 声明 valid（有效），类型和可用范围由所在声明决定。
};  // 结束此类型定义。

/**
 * @brief 计算以空字符结尾的任务码长度；空指针返回0。
 */
constexpr size_t TaskCodeLength(const char *text) {  // 定义函数或成员 的入口 TaskCodeLength（任务Code长度），参数由此处的签名确定。
    return text == nullptr || *text == '\0' ? 0 : 1 + TaskCodeLength(text + 1);  // 结束当前函数并返回 text == nullptr || *text == '\0' ? 0 : 1 + TaskCodeLen…。
}  // 结束此函数或控制块。
/**
 * @brief 检查三个任务颜色字符是否在1~6之间且互不重复。
 */
constexpr bool TaskColorsValid(char a, char b, char c) {  // 定义函数或成员 的入口 TaskColorsValid（任务Colors有效），参数由此处的签名确定。
    return a >= '1' && a <= '6' && b >= '1' && b <= '6' && c >= '1' && c <= '6'  // 结束当前函数并返回 a >= '1' && a <= '6' && b >= '1' && b <= '6' && c >= '…。
        && a != b && b != c && a != c;  // 继续上一行的组合条件，本行提供 && a != b && b != c && a != c;。
}  // 结束此函数或控制块。
/**
 * @brief 检查三个工位字符是否为1~3且互不重复。
 */
constexpr bool TaskPositionsValid(char a, char b, char c) {  // 定义函数或成员 的入口 TaskPositionsValid（任务Positions有效），参数由此处的签名确定。
    return a >= '1' && a <= '3' && b >= '1' && b <= '3' && c >= '1' && c <= '3'  // 结束当前函数并返回 a >= '1' && a <= '3' && b >= '1' && b <= '3' && c >= '…。
        && a != b && b != c && a != c;  // 继续上一行的组合条件，本行提供 && a != b && b != c && a != c;。
}  // 结束此函数或控制块。
/**
 * @brief 检查任务码是否符合颜色三位+工位三位+颜色三位+工位三位的15字符格式。
 */
// 四段分别为第一轮颜色、第一轮工位、第二轮颜色、第二轮工位。
// 合法示例 123+123+231+321，两轮必须使用同一组颜色，各轮内部顺序可以不同。
constexpr bool IsTaskCodeValid(const char *text) {  // 定义函数或成员 的入口 IsTaskCodeValid（Is任务Code有效），参数由此处的签名确定。
    return text != nullptr && TaskCodeLength(text) == 15  // 结束当前函数并返回 text != nullptr && TaskCodeLength(text) == 15。
        && text[3] == '+' && text[7] == '+' && text[11] == '+'  // 继续上一行的组合条件，本行提供 && text[3] == '+' && text[7] == '+' && text[11] == '+'。
        && TaskColorsValid(text[0], text[1], text[2])  // 继续上一行的组合条件，本行提供 && TaskColorsValid(text[0], text[1], text[2])。
        && TaskPositionsValid(text[4], text[5], text[6])  // 继续上一行的组合条件，本行提供 && TaskPositionsValid(text[4], text[5], text[6])。
        && TaskColorsValid(text[8], text[9], text[10])  // 继续上一行的组合条件，本行提供 && TaskColorsValid(text[8], text[9], text[10])。
        && (text[8] == text[0] || text[8] == text[1] || text[8] == text[2])  // 继续上一行的组合条件，本行提供 && (text[8] == text[0] || text[8] == text[1] || text[8] == text[2])。
        && (text[9] == text[0] || text[9] == text[1] || text[9] == text[2])  // 继续上一行的组合条件，本行提供 && (text[9] == text[0] || text[9] == text[1] || text[9] == text[2])。
        && (text[10] == text[0] || text[10] == text[1] || text[10] == text[2])  // 继续上一行的组合条件，本行提供 && (text[10] == text[0] || text[10] == text[1] || text[10] == text[2])。
        && TaskPositionsValid(text[12], text[13], text[14]);  // 继续上一行的组合条件，本行提供 && TaskPositionsValid(text[12], text[13], text[14]);。
}  // 结束此函数或控制块。
/**
 * @brief 把任务码解析为两轮颜色和工位数组；未通过格式检查时返回valid=false。
 * @param code 示例 123+123+231+321，两轮颜色集合相同。
 * @return 含两轮各三件颜色、位置及valid标志的任务结构。
 */
inline TaskCode parseTaskCode(const char *code) {  // 定义函数或成员 的入口 parseTaskCode（parse任务Code），参数由此处的签名确定。
    TaskCode task = {};  // 将 {} 写入或初始化 task（任务）。
    if (!IsTaskCodeValid(code)) return task;  // 若 !IsTaskCodeValid(code)，立即返回 task。
    for (size_t i = 0; i < 3; ++i) {  // 按 size_t i = 0; i < 3; ++i 组织循环的初值、继续条件和步进。
        task.round1_colors[i] = code[i] - '0';  // 将 code[i] - '0' 写入或初始化 task.round1_colors[i]。
        task.round1_pos[i] = code[4 + i] - '0';  // 将 code[4 + i] - '0' 写入或初始化 task.round1_pos[i]。
        task.round2_colors[i] = code[8 + i] - '0';  // 将 code[8 + i] - '0' 写入或初始化 task.round2_colors[i]。
        task.round2_pos[i] = code[12 + i] - '0';  // 将 code[12 + i] - '0' 写入或初始化 task.round2_pos[i]。
    }  // 结束此循环体。
    task.valid = true;  // 将 true 写入或初始化 task.valid。
    return task;  // 结束当前函数并返回 task。
}  // 结束此函数或控制块。
