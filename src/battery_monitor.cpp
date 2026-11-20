#include "battery_monitor.h"
#include <Arduino.h>
#include <Preferences.h>
#include <atomic>

namespace {
constexpr uint8_t adcPin = 10;  // ADC_BAT，R6 100 kΩ / R5 10 kΩ
constexpr uint8_t buzzerPin = 9;  // BZE，Q2 低边开关
constexpr uint32_t calibrationBatteryMv = 24440;
constexpr uint32_t calibrationAdcMv = 2220;
constexpr uint32_t samplePeriodMs = 200;
std::atomic<uint32_t> voltageMv{0};
std::atomic<uint32_t> alarmMv{0};
uint32_t sampledAt = 0;
portMUX_TYPE buzzerMux = portMUX_INITIALIZER_UNLOCKED;
bool cueActive = false;
bool cueOn = false;

void updateBuzzer() {
    digitalWrite(buzzerPin, (cueActive ? cueOn : voltageMv.load() < alarmMv.load()) ? HIGH : LOW);
}
}

void BatteryMonitorBegin() {
    analogSetPinAttenuation(adcPin, ADC_11db);
    digitalWrite(buzzerPin, LOW);
    pinMode(buzzerPin, OUTPUT);
    Preferences settings;
    if (settings.begin("battery", true)) alarmMv = settings.getUInt("alarm_mv", 0);
    sampledAt = millis() - samplePeriodMs;
    BatteryMonitorUpdate();
}

void BatteryMonitorUpdate() {
    const uint32_t now = millis();
    if (now - sampledAt >= samplePeriodMs) {
        sampledAt = now;
        const uint32_t adcMv = analogReadMilliVolts(adcPin);
        voltageMv = (adcMv * calibrationBatteryMv + calibrationAdcMv / 2) / calibrationAdcMv;
    }
    portENTER_CRITICAL(&buzzerMux);
    updateBuzzer();
    portEXIT_CRITICAL(&buzzerMux);
}

void BatteryMonitorSetCue(bool active,bool on) {
    portENTER_CRITICAL(&buzzerMux);
    cueActive=active;
    cueOn=on;
    updateBuzzer();
    portEXIT_CRITICAL(&buzzerMux);
}
uint32_t BatteryVoltageMv() { return voltageMv.load(); }
uint32_t BatteryAlarmMv() { return alarmMv.load(); }

bool BatterySetAlarmMv(uint32_t millivolts) {
    if (millivolts == alarmMv.load()) return true;
    Preferences settings;
    if (!settings.begin("battery", false) || settings.putUInt("alarm_mv", millivolts) != sizeof(millivolts)) return false;
    alarmMv = millivolts;
    return true;
}
