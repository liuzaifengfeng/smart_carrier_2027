#pragma once

#include <Arduino.h>

// 注册可调参数，不暴露 PID 积分历史等运行状态。
// 数组显示编号从 1 开始，内部仍使用原数组下标。
void RegisterRuntimeFloat(const char *name, float &value, float minimum, float maximum);
void RegisterRuntimeUInt(const char *name, uint32_t &value, float minimum, float maximum);
void RegisterChassisParameters();
// 在参数表末尾追加角点专用 PID，保持已有编号。
void RegisterCornerAlignmentParameters();
// 启动时应用从 robot_parameters.json 编译进固件的默认值；失败时禁止开始动作。
bool ApplyCompiledParameterDefaults();
// 调用方持有对齐互斥锁；仅在调试模式且停止对齐时允许写入。
// fields = [动作,事务编号,可选参数...]；解析结构由串口接收层完成。
bool HandleRuntimeParameters(const char *const *fields, size_t size, bool writable);
