#pragma once

#include <stdint.h>

// 三个圆盘位置各自的动作占位。color=1~6，cargo=1~3。
// 目前仅打印日志，不运动、不登记载物台、不推进 roundProgress。
// 接入真实动作时，必须同时接入完成/失败及恢复观察流程。
void GrabDiscPosition1(uint8_t color, uint8_t cargo);
void GrabDiscPosition2(uint8_t color, uint8_t cargo);
void GrabDiscPosition3(uint8_t color, uint8_t cargo);
