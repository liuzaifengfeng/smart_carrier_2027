#pragma once
#include <Arduino.h>

constexpr uint8_t LED_PWM_PIN = 5;
constexpr uint32_t LED_PWM_FREQUENCY_HZ = 8896;
constexpr uint32_t LED_PWM_MIN_FREQUENCY_HZ = 100;
constexpr uint32_t LED_PWM_MAX_FREQUENCY_HZ = 9000; // 40 MHz 时钟下保留 12 位分辨率。
constexpr uint8_t LED_PWM_CHANNEL = 0; // 独占 LEDC 通道 0 及其定时器。
constexpr uint8_t LED_PWM_RESOLUTION_BITS = 12;
constexpr uint32_t LED_FAULT_BLINK_INTERVAL_MS = 500;

// AO3400 低边开关，高电平导通；初始化后默认关闭，不保存到 Flash。
bool LedPwm_Init();
bool LedPwm_IsReady();
bool LedPwm_SetBrightness(uint32_t percent);
uint8_t LedPwm_GetBrightness();
bool LedPwm_SetFrequency(uint32_t hz);
uint32_t LedPwm_GetFrequency();
// 周期调用；故障时覆盖实际输出，退出故障后恢复设定亮度。
void LedPwm_UpdateFaultBlink(bool fault);
