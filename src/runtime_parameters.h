#pragma once

#include <Arduino.h>

// 注册可调参数，不暴露 PID 积分历史等运行状态。
// 数组显示编号从 1 开始，内部仍使用原数组下标。
/**
 * @brief 将浮点参数引用注册到运行时参数表，供串口读取和修改；name为上位机显示名称。
 */
void RegisterRuntimeFloat(const char *name, float &value, float minimum, float maximum);
/**
 * @brief 将无符号整型参数引用注册到运行时参数表，保留名称及可写范围信息。
 */
void RegisterRuntimeUInt(const char *name, uint32_t &value, float minimum, float maximum);
/**
 * @brief 注册底盘运动、视觉PID、死区、节点地图和脉冲标定参数。
 */
void RegisterChassisParameters();
// 在参数表末尾追加角点专用 PID，保持已有编号。
/**
 * @brief 在参数表末尾追加角点专用PID，保持此前参数编号顺序。
 */
void RegisterCornerAlignmentParameters();
// 启动时应用从 robot_parameters.json 编译进固件的默认值；失败时禁止开始动作。
/**
 * @brief 应用由robot_parameters.json生成并编译进固件的默认值。
 * @return true 所有默认值应用成功，false 存在参数应用错误。
 */
bool ApplyCompiledParameterDefaults();
// 调用方持有对齐互斥锁；仅在调试模式且停止对齐时允许写入。
// fields = [动作,事务编号,可选参数...]；解析结构由串口接收层完成。
/**
 * @brief 处理CFG参数读取、写入等串口动作；由串口层提供已解析字段和可写状态。
 * @return true 已处理对应请求，false 请求无法处理。
 */
bool HandleRuntimeParameters(const char *const *fields, size_t size, bool writable);
