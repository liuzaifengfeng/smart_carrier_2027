// 灯光通道公开接口。
// 亮度和 PWM 频率用于配置输出，故障闪烁和动作提示通过不同入口更新。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include <Arduino.h>  // 引入 Arduino.h 中的声明和定义。

constexpr uint8_t LED_PWM_PIN = 5;  // 将 5 写入或初始化 LED_PWM_PIN（LED占空比输出PIN）。
constexpr uint32_t LED_PWM_FREQUENCY_HZ = 8896;  // 将 8896 写入或初始化 LED_PWM_FREQUENCY_HZ（LED占空比输出FREQUENCYHZ）。
constexpr uint32_t LED_PWM_MIN_FREQUENCY_HZ = 100;  // 将 100 写入或初始化 LED_PWM_MIN_FREQUENCY_HZ（LED占空比输出最小FREQUENCYHZ）。
constexpr uint32_t LED_PWM_MAX_FREQUENCY_HZ = 9000; // 40 MHz 时钟下保留 12 位分辨率。
constexpr uint8_t LED_PWM_CHANNEL = 0; // 独占 LEDC 通道 0 及其定时器。
constexpr uint8_t LED_PWM_RESOLUTION_BITS = 12;  // 将 12 写入或初始化 LED_PWM_RESOLUTION_BITS（LED占空比输出RESOLUTIONBITS）。
constexpr uint32_t LED_FAULT_EFFECT_PERIOD_MS = 1400;  // 将 1400 写入或初始化 LED_FAULT_EFFECT_PERIOD_MS（LEDFAULTEFFECTPERIOD毫秒）。

// AO3400 低边开关，高电平导通；初始化后默认关闭，不保存到 Flash。
/**
 * @brief 初始化灯光PWM通道，默认关闭输出；返回硬件配置是否成功。
 */
bool LedPwm_Init();  // 声明接口 LedPwm_Init（Led占空比输出Init），参数由此处的签名确定。
/**
 * @brief 判断灯光PWM是否初始化成功。
 */
bool LedPwm_IsReady();  // 声明接口 LedPwm_IsReady（Led占空比输出Is就绪），参数由此处的签名确定。
/**
 * @brief 设置灯光亮度百分比，不保存到Flash；返回是否应用成功。
 */
bool LedPwm_SetBrightness(uint32_t percent);  // 声明接口 LedPwm_SetBrightness（Led占空比输出SetBrightness），参数由此处的签名确定。
/** @brief 圆环视觉期间以 15% 补光，结束后恢复设定亮度。 */
void LedPwm_SetRingVisionActive(bool active);  // 声明接口 LedPwm_SetRingVisionActive（Led占空比输出Set圆环视觉活跃），参数由此处的签名确定。
/**
 * @brief 读取设定亮度百分比，不等于故障闪烁期间的瞬时实际输出。
 */
uint8_t LedPwm_GetBrightness();  // 声明接口 LedPwm_GetBrightness（Led占空比输出GetBrightness），参数由此处的签名确定。
/**
 * @brief 设置PWM频率Hz；返回是否成功应用。
 */
bool LedPwm_SetFrequency(uint32_t hz);  // 声明接口 LedPwm_SetFrequency（Led占空比输出SetFrequency），参数由此处的签名确定。
/**
 * @brief 读取当前配置的PWM频率Hz。
 */
uint32_t LedPwm_GetFrequency();  // 声明接口 LedPwm_GetFrequency（Led占空比输出GetFrequency），参数由此处的签名确定。
/**
 * @brief 周期更新三连爆闪和渐强故障光效；fault=true时覆盖实际输出，故障解除后恢复设定亮度。
 */
void LedPwm_UpdateFaultBlink(bool fault);  // 声明接口 LedPwm_UpdateFaultBlink（Led占空比输出UpdateFaultBlink），参数由此处的签名确定。

/** @brief 非阻塞播放同步声光提示，每次亮响67ms、熄灭静音67ms。 */
void LedPwm_PlayCue(uint8_t pulses);  // 声明接口 LedPwm_PlayCue（Led占空比输出PlayCue），参数由此处的签名确定。

/** @brief 非阻塞播放短、长、短的同步声光提示。 */
void LedPwm_PlayCompletionCue();  // 声明接口 LedPwm_PlayCompletionCue（Led占空比输出PlayCompletionCue），参数由此处的签名确定。
