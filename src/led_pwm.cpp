#include "led_pwm.h"

namespace {
bool ready = false;
uint8_t brightness = 0;
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
