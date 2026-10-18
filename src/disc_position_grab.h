#pragma once

#include <stdint.h>

// 三个圆盘位置各自完成夹取、抬升和公共载物台放置。
// true 表示整套预设动作下发完成并已登记载物台，不代表传感器确认实物。
bool GrabDiscPosition1(uint8_t color, uint8_t cargo);
bool GrabDiscPosition2(uint8_t color, uint8_t cargo);
bool GrabDiscPosition3(uint8_t color, uint8_t cargo);
