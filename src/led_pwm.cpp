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
uint8_t faultEffectBrightness = 0;
uint32_t faultEffectStartMs = 0;
bool calibrationBlink = false;
uint32_t calibrationBlinkStartMs = 0;
uint8_t calibrationEffectBrightness = 100;
class PwmLock {
public:
    PwmLock() { if (pwmMutex) xSemaphoreTake(pwmMutex, portMAX_DELAY); }
    ~PwmLock() { if (pwmMutex) xSemaphoreGive(pwmMutex); }
};
void applyBrightness() {
    const uint32_t percent = faultBlink ? faultEffectBrightness : calibrationBlink ? calibrationEffectBrightness : brightness;
    ledcWrite(LED_PWM_CHANNEL, (percent * maxDuty + 50) / 100);
}

uint8_t faultPatternBrightness(uint32_t elapsedMs) {
    const uint32_t phase = elapsedMs % LED_FAULT_EFFECT_PERIOD_MS;

    // 三次高亮爆闪，最后一次刻意拉长，形成明显的故障冲击感。
    if (phase < 80) return 100;
    if (phase < 140) return 0;
    if (phase < 220) return 100;
    if (phase < 280) return 0;
    if (phase < 440) return 100;
    if (phase < 620) return 0;

    // 暗场后从 10% 快速蓄亮到 100%，随后骤灭并进入下一轮。
    if (phase < 1040) {
        const uint32_t ramp = phase - 620;
        return static_cast<uint8_t>(10 + ramp * 90 / 400);
    }
    return 0;
}
}

/**
 * @brief 初始化灯光PWM通道，默认关闭输出；返回硬件配置是否成功。
 */
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

/**
 * @brief 判断灯光PWM是否初始化成功。
 */
bool LedPwm_IsReady() { PwmLock lock; return ready; }

/**
 * @brief 设置灯光亮度百分比，不保存到Flash；返回是否应用成功。
 */
bool LedPwm_SetBrightness(uint32_t percent) {
    PwmLock lock;
    if (!ready || percent > 100) return false;
    // Arduino LEDC 将 maxDuty 转换为持续高电平，确保 100% 全亮。
    brightness = static_cast<uint8_t>(percent);
    applyBrightness();
    return true;
}

/**
 * @brief 读取设定亮度百分比，不等于故障闪烁期间的瞬时实际输出。
 */
uint8_t LedPwm_GetBrightness() { PwmLock lock; return brightness; }

/**
 * @brief 设置PWM频率Hz；返回是否成功应用。
 */
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
/**
 * @brief 读取当前配置的PWM频率Hz。
 */
uint32_t LedPwm_GetFrequency() { PwmLock lock; return ready ? frequencyHz : 0; }

/**
 * @brief 周期更新故障闪烁；fault=true时覆盖实际输出，故障解除后恢复设定亮度。
 */
void LedPwm_UpdateFaultBlink(bool fault) {
    PwmLock lock;
    if (!ready) return;
    const uint32_t now = millis();
    if(calibrationBlink) {
        const uint32_t elapsed=now-calibrationBlinkStartMs;
        if(fault||elapsed>=800)calibrationBlink=false;
        else calibrationEffectBrightness=(elapsed/200)%2==0?100:0;
        if(!fault){faultBlink=false;applyBrightness();return;}
    }
    if (fault) {
        if (!faultBlink) {
            faultBlink = true;
            faultEffectStartMs = now;
        }
        const uint8_t nextBrightness = faultPatternBrightness(now - faultEffectStartMs);
        if (nextBrightness == faultEffectBrightness) return;
        faultEffectBrightness = nextBrightness;
        applyBrightness();
    } else if (faultBlink) {
        faultBlink = false;
        faultEffectBrightness = 0;
        applyBrightness();
    }
}

void LedPwm_BlinkCalibrationComplete() {
    PwmLock lock;
    calibrationBlinkStartMs=millis();calibrationEffectBrightness=100;calibrationBlink=true;
    applyBrightness();
}
