// 周期更新电池电压并管理蜂鸣器输出。
// 提示音与低电压告警共用输出，修改其中一路时应同时查看 updateBuzzer 如何合并状态。

#include "battery_monitor.h"  // 引入 battery_monitor.h 中的声明和定义。
#include <Arduino.h>  // 引入 Arduino.h 中的声明和定义。
#include <Preferences.h>  // 引入 Preferences.h 中的声明和定义。
#include <atomic>  // 引入 atomic 中的声明和定义。

namespace {  // 开始匿名命名空间，本文件内部共享其中的定义。
constexpr uint8_t adcPin = 10;  // ADC_BAT，R6 100 kΩ / R5 10 kΩ
constexpr uint8_t buzzerPin = 9;  // BZE，Q2 低边开关
constexpr uint32_t calibrationBatteryMv = 24440;  // 将 24440 写入或初始化 calibrationBatteryMv。
constexpr uint32_t calibrationAdcMv = 2220;  // 将 2220 写入或初始化 calibrationAdcMv。
constexpr uint32_t samplePeriodMs = 200;  // 将 200 写入或初始化 samplePeriodMs（采样Period毫秒）。
std::atomic<uint32_t> voltageMv{0};  // 声明 voltageMv，类型和可用范围由所在声明决定。
std::atomic<uint32_t> alarmMv{0};  // 声明 alarmMv，类型和可用范围由所在声明决定。
uint32_t sampledAt = 0;  // 将 0 写入或初始化 sampledAt。
portMUX_TYPE buzzerMux = portMUX_INITIALIZER_UNLOCKED;  // 将 portMUX_INITIALIZER_UNLOCKED 写入或初始化 buzzerMux（buzzer临界区锁）。
bool cueActive = false;  // 将 false 写入或初始化 cueActive（cue活跃）。
bool cueOn = false;  // 将 false 写入或初始化 cueOn。

void updateBuzzer() {  // 定义函数或成员 的入口 updateBuzzer，参数由此处的签名确定。
    digitalWrite(buzzerPin, (cueActive ? cueOn : voltageMv.load() < alarmMv.load()) ? HIGH : LOW);  // 调用 digitalWrite，传入 buzzerPin, (cueActive ? cueOn : voltageMv.load() < ala…。
}  // 结束此函数或控制块。
}  // 结束命名空间。

void BatteryMonitorBegin() {  // 定义函数或成员 的入口 BatteryMonitorBegin，参数由此处的签名确定。
    analogSetPinAttenuation(adcPin, ADC_11db);  // 调用 analogSetPinAttenuation，传入 adcPin, ADC_11db。
    digitalWrite(buzzerPin, LOW);  // 调用 digitalWrite，传入 buzzerPin, LOW。
    pinMode(buzzerPin, OUTPUT);  // 调用 pinMode（pin模式），传入 buzzerPin, OUTPUT。
    Preferences settings;  // 声明 settings（设置），类型和可用范围由所在声明决定。
    if (settings.begin("battery", true)) alarmMv = settings.getUInt("alarm_mv", 0);  // 当 settings.begin("battery", true) 成立时进入此分支，否则继续后续流程。
    sampledAt = millis() - samplePeriodMs;  // 将 millis() - samplePeriodMs 写入或初始化 sampledAt。
    BatteryMonitorUpdate();  // 调用 BatteryMonitorUpdate，使用无参形式推进相应操作。
}  // 结束此函数或控制块。

void BatteryMonitorUpdate() {  // 定义函数或成员 的入口 BatteryMonitorUpdate，参数由此处的签名确定。
    const uint32_t now = millis();  // 将 millis() 写入或初始化 now。
    if (now - sampledAt >= samplePeriodMs) {  // 当 now - sampledAt >= samplePeriodMs 成立时进入此分支，否则继续后续流程。
        sampledAt = now;  // 将 now 写入或初始化 sampledAt。
        // 先取 ADC 引脚毫伏读数，再按实测整机电压与 ADC 电压的比例恢复电池端电压。
        // 除法前加半个分母实现整数四舍五入。
        const uint32_t adcMv = analogReadMilliVolts(adcPin);  // 将 analogReadMilliVolts(adcPin) 写入或初始化 adcMv。
        voltageMv = (adcMv * calibrationBatteryMv + calibrationAdcMv / 2) / calibrationAdcMv;  // 将 (adcMv * calibrationBatteryMv + calibrationAdcMv / 2) … 写入或初始化 voltageMv。
    }  // 结束此条件分支。
    portENTER_CRITICAL(&buzzerMux);  // 进入临界区，保护共享状态的本段访问。
    updateBuzzer();  // 调用 updateBuzzer，使用无参形式推进相应操作。
    portEXIT_CRITICAL(&buzzerMux);  // 离开临界区，保护共享状态的本段访问。
}  // 结束此函数或控制块。

// 面板提示通过 active 与 on 两个状态表达当前是否接管和是否鸣响。
// 蜂鸣器实际电平统一由 updateBuzzer 更新。
void BatteryMonitorSetCue(bool active,bool on) {  // 定义函数或成员 的入口 BatteryMonitorSetCue，参数由此处的签名确定。
    portENTER_CRITICAL(&buzzerMux);  // 进入临界区，保护共享状态的本段访问。
    cueActive=active;  // 将 active 写入或初始化 cueActive（cue活跃）。
    cueOn=on;  // 将 on 写入或初始化 cueOn。
    updateBuzzer();  // 调用 updateBuzzer，使用无参形式推进相应操作。
    portEXIT_CRITICAL(&buzzerMux);  // 离开临界区，保护共享状态的本段访问。
}  // 结束此函数或控制块。
uint32_t BatteryVoltageMv() { return voltageMv.load(); }  // 定义函数或成员 BatteryVoltageMv，参数由此处的签名确定。
uint32_t BatteryAlarmMv() { return alarmMv.load(); }  // 定义函数或成员 BatteryAlarmMv，参数由此处的签名确定。

bool BatterySetAlarmMv(uint32_t millivolts) {  // 定义函数或成员 的入口 BatterySetAlarmMv，参数由此处的签名确定。
    if (millivolts == alarmMv.load()) return true;  // 若 millivolts == alarmMv.load()，立即返回 true。
    Preferences settings;  // 声明 settings（设置），类型和可用范围由所在声明决定。
    if (!settings.begin("battery", false) || settings.putUInt("alarm_mv", millivolts) != sizeof(millivolts)) return false;  // 若 !settings.begin("battery", false) || settings.putUInt(…，立即返回 false。
    alarmMv = millivolts;  // 将 millivolts 写入或初始化 alarmMv。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。
