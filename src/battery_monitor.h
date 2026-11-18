#pragma once
#include <stdint.h>

void BatteryMonitorBegin();
void BatteryMonitorUpdate();
uint32_t BatteryVoltageMv();
uint32_t BatteryAlarmMv();
bool BatterySetAlarmMv(uint32_t millivolts);
