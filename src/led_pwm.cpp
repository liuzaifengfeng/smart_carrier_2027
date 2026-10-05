#include "led_pwm.h"

namespace {
bool ready = false;
uint8_t brightness = 0;
uint32_t frequencyHz = LED_PWM_FREQUENCY_HZ;
constexpr uint32_t maxDuty = (1UL << LED_PWM_RESOLUTION_BITS) - 1;
}

bool LedPwm_Init() {
    if (ready) return LedPwm_SetBrightness(0);
    digitalWrite(LED_PWM_PIN, LOW);
    pinMode(LED_PWM_PIN, OUTPUT);
    if (ledcSetup(LED_PWM_CHANNEL, LED_PWM_FREQUENCY_HZ,
                  LED_PWM_RESOLUTION_BITS) != LED_PWM_FREQUENCY_HZ) return false;
    ledcWrite(LED_PWM_CHANNEL, 0);
    ledcAttachPin(LED_PWM_PIN, LED_PWM_CHANNEL);
    brightness = 0;
    frequencyHz = LED_PWM_FREQUENCY_HZ;
    ready = true;
    return true;
}

bool LedPwm_IsReady() { return ready; }

bool LedPwm_SetBrightness(uint32_t percent) {
    if (!ready || percent > 100) return false;
    // Arduino LEDC 将 maxDuty 转换为持续高电平，确保 100% 全亮。
    ledcWrite(LED_PWM_CHANNEL, (percent * maxDuty + 50) / 100);
    brightness = static_cast<uint8_t>(percent);
    return true;
}

uint8_t LedPwm_GetBrightness() { return brightness; }

bool LedPwm_SetFrequency(uint32_t hz) {
    if (!ready || hz < LED_PWM_MIN_FREQUENCY_HZ || hz > LED_PWM_MAX_FREQUENCY_HZ)
        return false;
    const uint32_t previous = LedPwm_GetFrequency();
    const uint32_t actual = ledcChangeFrequency(LED_PWM_CHANNEL, hz, LED_PWM_RESOLUTION_BITS);
    if (actual == 0) {
        // 配置失败时尝试恢复原频率；恢复失败则关闭输出并报告 PWM 故障。
        const uint32_t restored = ledcChangeFrequency(LED_PWM_CHANNEL, previous, LED_PWM_RESOLUTION_BITS);
        if (restored == 0) {
            ledcWrite(LED_PWM_CHANNEL, 0);
            brightness = 0;
            ready = false;
        } else {
            frequencyHz = restored;
            LedPwm_SetBrightness(brightness);
        }
        return false;
    }
    frequencyHz = actual;
    return LedPwm_SetBrightness(brightness);
}

// Arduino 2.x 的 ledcReadFreq 在占空比为 0 时返回 0，故保存配置接口的回读值。
uint32_t LedPwm_GetFrequency() { return ready ? frequencyHz : 0; }
