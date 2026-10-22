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
/**
 * @brief 初始化灯光PWM通道，默认关闭输出；返回硬件配置是否成功。
 */
bool LedPwm_Init();
/**
 * @brief 判断灯光PWM是否初始化成功。
 */
bool LedPwm_IsReady();
/**
 * @brief 设置灯光亮度百分比，不保存到Flash；返回是否应用成功。
 */
bool LedPwm_SetBrightness(uint32_t percent);
/**
 * @brief 读取设定亮度百分比，不等于故障闪烁期间的瞬时实际输出。
 */
uint8_t LedPwm_GetBrightness();
/**
 * @brief 设置PWM频率Hz；返回是否成功应用。
 */
bool LedPwm_SetFrequency(uint32_t hz);
/**
 * @brief 读取当前配置的PWM频率Hz。
 */
uint32_t LedPwm_GetFrequency();
// 周期调用；故障时覆盖实际输出，退出故障后恢复设定亮度。
/**
 * @brief 周期更新故障闪烁；fault=true时覆盖实际输出，故障解除后恢复设定亮度。
 */
void LedPwm_UpdateFaultBlink(bool fault);
