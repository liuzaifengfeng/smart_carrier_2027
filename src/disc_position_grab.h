// 按圆盘物理位号抓料的接口。
// 阅读调用时需区分抓取点位置、物料颜色和目标货位。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。

#include <stdint.h>  // 引入 stdint.h 中的声明和定义。
extern int armSpeed;  // 声明外部定义 int armSpeed，供本编译单元引用。

// 三个圆盘位置各自完成夹取、抬升和公共载物台放置。
// true 表示整套预设动作下发完成并已登记载物台，不代表传感器确认实物。
/**
 * @brief 同步提交并等待圆盘位置1的夹取、抬升及载物台放置动作。
 * @param color 物料颜色编号。
 * @param cargo 目标载物台编号1~3。
 * @return true 作业完成并登记，不代表传感器确认实物。
 */
bool GrabDiscPosition1(uint8_t color, uint8_t cargo);  // 声明接口 GrabDiscPosition1（Grab圆盘位置1），参数由此处的签名确定。
/**
 * @brief 同步提交并等待圆盘位置2的夹取、抬升及载物台放置动作；color为颜色，cargo为载物台1~3。
 */
bool GrabDiscPosition2(uint8_t color, uint8_t cargo);  // 声明接口 GrabDiscPosition2（Grab圆盘位置2），参数由此处的签名确定。
/**
 * @brief 同步提交并等待圆盘位置3的夹取、抬升及载物台放置动作；color为颜色，cargo为载物台1~3。
 */
bool GrabDiscPosition3(uint8_t color, uint8_t cargo);  // 声明接口 GrabDiscPosition3（Grab圆盘位置3），参数由此处的签名确定。
