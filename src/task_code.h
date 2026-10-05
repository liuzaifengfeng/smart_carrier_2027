#pragma once

#include <stddef.h>

struct TaskCode {
    int round1_colors[3];
    int round1_pos[3];
    int round2_colors[3];
    int round2_pos[3];
    bool valid;
};

constexpr size_t TaskCodeLength(const char *text) {
    return text == nullptr || *text == '\0' ? 0 : 1 + TaskCodeLength(text + 1);
}
constexpr bool TaskColorsValid(char a, char b, char c) {
    return a >= '1' && a <= '6' && b >= '1' && b <= '6' && c >= '1' && c <= '6';
}
constexpr bool TaskPositionsValid(char a, char b, char c) {
    return a >= '1' && a <= '3' && b >= '1' && b <= '3' && c >= '1' && c <= '3'
        && a != b && b != c && a != c;
}
constexpr bool IsTaskCodeValid(const char *text) {
    return text != nullptr && TaskCodeLength(text) == 15
        && text[3] == '+' && text[7] == '+' && text[11] == '+'
        && TaskColorsValid(text[0], text[1], text[2])
        && TaskPositionsValid(text[4], text[5], text[6])
        && TaskColorsValid(text[8], text[9], text[10])
        && TaskPositionsValid(text[12], text[13], text[14]);
}
inline TaskCode parseTaskCode(const char *code) {
    TaskCode task = {};
    if (!IsTaskCodeValid(code)) return task;
    for (size_t i = 0; i < 3; ++i) {
        task.round1_colors[i] = code[i] - '0';
        task.round1_pos[i] = code[4 + i] - '0';
        task.round2_colors[i] = code[8 + i] - '0';
        task.round2_pos[i] = code[12 + i] - '0';
    }
    task.valid = true;
    return task;
}
