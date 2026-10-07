#include "led_pwm.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace {
bool ready = false;
uint8_t brightness = 0;
uint32_t frequencyHz = LED_PWM_FREQUENCY_HZ;
constexpr uint32_t maxDuty = (1UL << LED_PWM_RESOLUTION_BITS) - 1;
SemaphoreHandle_t pwmMutex = nullptr;
bool faultBlink = false;
bool faultLightOn = false;
uint32_t lastToggleMs = 0;
class PwmLock {
public:
    PwmLock() { if (pwmMutex) xSemaphoreTake(pwmMutex, portMAX_DELAY); }
    ~PwmLock() { if (pwmMutex) xSemaphoreGive(pwmMutex); }
};
void applyBrightness() {
    const uint32_t percent = faultBlink ? (faultLightOn ? 100 : 0) : brightness;
    ledcWrite(LED_PWM_CHANNEL, (percent * maxDuty + 50) / 100);
}
}

bool LedPwm_Init() {
    // setup() 在其他任务启动前初始化互斥锁。
    if (!pwmMutex) pwmMutex = xSemaphoreCreateMutex();
    if (!pwmMutex) return false;
    PwmLock lock;
    if (ready) {
        brightness = 0;
        applyBrightness();
        return true;
    }
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

bool LedPwm_IsReady() { PwmLock lock; return ready; }

bool LedPwm_SetBrightness(uint32_t percent) {
    PwmLock lock;
    if (!ready || percent > 100) return false;
    // Arduino LEDC 将 maxDuty 转换为持续高电平，确保 100% 全亮。
    brightness = static_cast<uint8_t>(percent);
    applyBrightness();
    return true;
}

uint8_t LedPwm_GetBrightness() { PwmLock lock; return brightness; }

bool LedPwm_SetFrequency(uint32_t hz) {
    PwmLock lock;
    if (!ready || hz < LED_PWM_MIN_FREQUENCY_HZ || hz > LED_PWM_MAX_FREQUENCY_HZ)
        return false;
    const uint32_t previous = frequencyHz;
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
            applyBrightness();
        }
        return false;
    }
    frequencyHz = actual;
    applyBrightness();
    return true;
}

// Arduino 2.x 的 ledcReadFreq 在占空比为 0 时返回 0，故保存配置接口的回读值。
uint32_t LedPwm_GetFrequency() { PwmLock lock; return ready ? frequencyHz : 0; }

void LedPwm_UpdateFaultBlink(bool fault) {
    PwmLock lock;
    if (!ready) return;
    const uint32_t now = millis();
    if (fault) {
        if (!faultBlink) {
            faultBlink = true;
            faultLightOn = true;
            lastToggleMs = now;
            applyBrightness();
        } else if (static_cast<uint32_t>(now - lastToggleMs) >= LED_FAULT_BLINK_INTERVAL_MS) {
            faultLightOn = !faultLightOn;
            lastToggleMs = now;
            applyBrightness();
        }
    } else if (faultBlink) {
        faultBlink = false;
        applyBrightness();
    }
}
