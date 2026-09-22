#pragma once
#include <Arduino.h>

// 注册可调参数，不暴露 PID 积分历史等运行状态。
// 数组显示编号从 1 开始，内部仍使用原数组下标。
void RegisterRuntimeFloat(const char *name, float &value, float minimum, float maximum);
void RegisterRuntimeUInt(const char *name, uint32_t &value, float minimum, float maximum);
void RegisterChassisParameters();
// 调用方持有对齐互斥锁；仅在调试模式且停止对齐时允许写入。
bool HandleRuntimeParameters(const char *frame, bool writable);
