// 电池监测对外接口。
// 电压读数和告警阈值以毫伏为单位，更新由调用方周期触发。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include <stdint.h>  // 引入 stdint.h 中的声明和定义。

void BatteryMonitorBegin();  // 声明接口 BatteryMonitorBegin，参数由此处的签名确定。
void BatteryMonitorUpdate();  // 声明接口 BatteryMonitorUpdate，参数由此处的签名确定。
void BatteryMonitorSetCue(bool active,bool on);  // 声明接口 BatteryMonitorSetCue，参数由此处的签名确定。
uint32_t BatteryVoltageMv();  // 声明接口 BatteryVoltageMv，参数由此处的签名确定。
uint32_t BatteryAlarmMv();  // 声明接口 BatteryAlarmMv，参数由此处的签名确定。
bool BatterySetAlarmMv(uint32_t millivolts);  // 声明接口 BatterySetAlarmMv，参数由此处的签名确定。
