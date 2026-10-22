#pragma once

#include <stdint.h>
extern int armSpeed;

// 三个圆盘位置各自完成夹取、抬升和公共载物台放置。
// true 表示整套预设动作下发完成并已登记载物台，不代表传感器确认实物。
/**
 * @brief 同步提交并等待圆盘位置1的夹取、抬升及载物台放置动作。
 * @param color 物料颜色编号。
 * @param cargo 目标载物台编号1~3。
 * @return true 作业完成并登记，不代表传感器确认实物。
 */
bool GrabDiscPosition1(uint8_t color, uint8_t cargo);
/**
 * @brief 同步提交并等待圆盘位置2的夹取、抬升及载物台放置动作；color为颜色，cargo为载物台1~3。
 */
bool GrabDiscPosition2(uint8_t color, uint8_t cargo);
/**
 * @brief 同步提交并等待圆盘位置3的夹取、抬升及载物台放置动作；color为颜色，cargo为载物台1~3。
 */
bool GrabDiscPosition3(uint8_t color, uint8_t cargo);
