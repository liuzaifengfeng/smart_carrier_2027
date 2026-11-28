// PWM 灯光输出，同时管理手动亮度、视觉补光及提示闪烁。
// 调试亮度时要核对上层状态和当前时间是否正在覆盖基础亮度。

#include "led_pwm.h"  // 引入 led_pwm.h 中的声明和定义。
#include "battery_monitor.h"  // 引入 battery_monitor.h 中的声明和定义。
#include <freertos/FreeRTOS.h>  // 引入 freertos/FreeRTOS.h 中的声明和定义。
#include <freertos/semphr.h>  // 引入 freertos/semphr.h 中的声明和定义。

namespace {  // 开始匿名命名空间，本文件内部共享其中的定义。
bool ready = false;  // 将 false 写入或初始化 ready（就绪）。
uint8_t brightness = 0;  // 将 0 写入或初始化 brightness。
bool ringVisionActive = false;  // 将 false 写入或初始化 ringVisionActive（圆环视觉活跃）。
constexpr uint8_t ringVisionBrightness = 15;  // 将 15 写入或初始化 ringVisionBrightness（圆环视觉Brightness）。
uint32_t frequencyHz = LED_PWM_FREQUENCY_HZ;  // 将 LED_PWM_FREQUENCY_HZ 写入或初始化 frequencyHz。
constexpr uint32_t maxDuty = (1UL << LED_PWM_RESOLUTION_BITS) - 1;  // 将 (1UL << LED_PWM_RESOLUTION_BITS) - 1 写入或初始化 maxDuty（最大占空比）。
SemaphoreHandle_t pwmMutex = nullptr;  // 将 nullptr 写入或初始化 pwmMutex（占空比输出Mutex）。
bool faultBlink = false;  // 将 false 写入或初始化 faultBlink。
uint8_t faultEffectBrightness = 0;  // 将 0 写入或初始化 faultEffectBrightness。
uint32_t faultEffectStartMs = 0;  // 将 0 写入或初始化 faultEffectStartMs（faultEffect起点毫秒）。
uint8_t cuePulses = 0;  // 将 0 写入或初始化 cuePulses（cue脉冲数）。
bool cueLongMiddle = false;  // 将 false 写入或初始化 cueLongMiddle。
constexpr uint32_t cuePhaseMs = 67;  // 将 67 写入或初始化 cuePhaseMs（cue阶段毫秒）。
uint32_t cueStartMs = 0;  // 将 0 写入或初始化 cueStartMs（cue起点毫秒）。
uint8_t cueBrightness = 100;  // 将 100 写入或初始化 cueBrightness。
class PwmLock {  // 定义 PwmLock（占空比输出Lock） 的类型成员或作用域。
public:  // 此后的成员采用 public 访问权限，直到下一个权限标签。
    PwmLock() { if (pwmMutex) xSemaphoreTake(pwmMutex, portMAX_DELAY); }  // 调用 PwmLock（占空比输出Lock），传入 ) { if (pwmMutex) xSemaphoreTake(pwmMutex, portMAX_DEL…。
    ~PwmLock() { if (pwmMutex) xSemaphoreGive(pwmMutex); }  // 定义函数或成员 PwmLock（占空比输出Lock），参数由此处的签名确定。
};  // 结束此类型定义。
void applyBrightness() {  // 定义函数或成员 的入口 applyBrightness，参数由此处的签名确定。
    const uint8_t steadyBrightness = ringVisionActive ? ringVisionBrightness : brightness;  // 将 ringVisionActive ? ringVisionBrightness : brightness 写入或初始化 steadyBrightness。
    const uint32_t percent = faultBlink ? faultEffectBrightness : cuePulses ? cueBrightness : steadyBrightness;  // 将 faultBlink ? faultEffectBrightness : cuePulses ? cueBr… 写入或初始化 percent。
    ledcWrite(LED_PWM_CHANNEL, (percent * maxDuty + 50) / 100);  // 调用 ledcWrite，传入 LED_PWM_CHANNEL, (percent * maxDuty + 50) / 100。
}  // 结束此函数或控制块。

void startCue(uint8_t pulses, bool longMiddle) {  // 定义函数或成员 的入口 startCue（起点Cue），参数由此处的签名确定。
    PwmLock lock;  // 声明 lock，类型和可用范围由所在声明决定。
    cueStartMs=millis();cueBrightness=100;cuePulses=pulses;cueLongMiddle=longMiddle;  // 将 millis() 写入或初始化 cueStartMs（cue起点毫秒）。然后将 100 写入或初始化 cueBrightness。然后将 pulses 写入或初始化 cuePulses（cue脉冲数）。然后将 longMiddle 写入或初始化 cueLongMiddle。
    BatteryMonitorSetCue(pulses!=0,pulses!=0);  // 调用 BatteryMonitorSetCue，传入 pulses!=0,pulses!=0。
    if(ready)applyBrightness();  // 当 ready 成立时进入此分支，否则继续后续流程。
}  // 结束此函数或控制块。

uint8_t faultPatternBrightness(uint32_t elapsedMs) {  // 定义函数或成员 的入口 faultPatternBrightness，参数由此处的签名确定。
    const uint32_t phase = elapsedMs % LED_FAULT_EFFECT_PERIOD_MS;  // 将 elapsedMs % LED_FAULT_EFFECT_PERIOD_MS 写入或初始化 phase（阶段）。

    // 三次高亮爆闪，最后一次刻意拉长，形成明显的故障冲击感。
    if (phase < 80) return 100;  // 若 phase < 80，立即返回 100。
    if (phase < 140) return 0;  // 若 phase < 140，立即返回 0。
    if (phase < 220) return 100;  // 若 phase < 220，立即返回 100。
    if (phase < 280) return 0;  // 若 phase < 280，立即返回 0。
    if (phase < 440) return 100;  // 若 phase < 440，立即返回 100。
    if (phase < 620) return 0;  // 若 phase < 620，立即返回 0。

    // 暗场后从 10% 快速蓄亮到 100%，随后骤灭并进入下一轮。
    if (phase < 1040) {  // 当 phase < 1040 成立时进入此分支，否则继续后续流程。
        const uint32_t ramp = phase - 620;  // 将 phase - 620 写入或初始化 ramp。
        return static_cast<uint8_t>(10 + ramp * 90 / 400);  // 结束当前函数并返回 static_cast<uint8_t>(10 + ramp * 90 / 400)。
    }  // 结束此条件分支。
    return 0;  // 结束当前函数并返回 0。
}  // 结束此函数或控制块。
}  // 结束命名空间。

/**
 * @brief 初始化灯光PWM通道，默认关闭输出；返回硬件配置是否成功。
 */
bool LedPwm_Init() {  // 定义函数或成员 的入口 LedPwm_Init（Led占空比输出Init），参数由此处的签名确定。
    // setup() 在其他任务启动前初始化互斥锁。
    if (!pwmMutex) pwmMutex = xSemaphoreCreateMutex();  // 当 !pwmMutex 成立时进入此分支，否则继续后续流程。
    if (!pwmMutex) return false;  // 若 !pwmMutex，立即返回 false。
    PwmLock lock;  // 声明 lock，类型和可用范围由所在声明决定。
    if (ready) {  // 当 ready 成立时进入此分支，否则继续后续流程。
        brightness = 0;  // 将 0 写入或初始化 brightness。
        applyBrightness();  // 调用 applyBrightness，使用无参形式推进相应操作。
        return true;  // 结束当前函数并返回 true。
    }  // 结束此条件分支。
    digitalWrite(LED_PWM_PIN, LOW);  // 调用 digitalWrite，传入 LED_PWM_PIN, LOW。
    pinMode(LED_PWM_PIN, OUTPUT);  // 调用 pinMode（pin模式），传入 LED_PWM_PIN, OUTPUT。
    if (ledcSetup(LED_PWM_CHANNEL, LED_PWM_FREQUENCY_HZ,  // 当 ledcSetup(LED_PWM_CHANNEL, LED_PWM_FREQUENCY_HZ, 成立时进入此分支，否则继续后续流程。
                  LED_PWM_RESOLUTION_BITS) != LED_PWM_FREQUENCY_HZ) return false;  // 续写当前声明或运算，提供 LED_PWM_RESOLUTION_BITS) != LED_PWM_FREQUENCY_HZ) retu…。
    ledcWrite(LED_PWM_CHANNEL, 0);  // 调用 ledcWrite，传入 LED_PWM_CHANNEL, 0。
    ledcAttachPin(LED_PWM_PIN, LED_PWM_CHANNEL);  // 调用 ledcAttachPin，传入 LED_PWM_PIN, LED_PWM_CHANNEL。
    brightness = 0;  // 将 0 写入或初始化 brightness。
    frequencyHz = LED_PWM_FREQUENCY_HZ;  // 将 LED_PWM_FREQUENCY_HZ 写入或初始化 frequencyHz。
    ready = true;  // 将 true 写入或初始化 ready（就绪）。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

/**
 * @brief 判断灯光PWM是否初始化成功。
 */
bool LedPwm_IsReady() { PwmLock lock; return ready; }  // 定义函数或成员 LedPwm_IsReady（Led占空比输出Is就绪），参数由此处的签名确定。

/**
 * @brief 设置灯光亮度百分比，不保存到Flash；返回是否应用成功。
 */
bool LedPwm_SetBrightness(uint32_t percent) {  // 定义函数或成员 的入口 LedPwm_SetBrightness（Led占空比输出SetBrightness），参数由此处的签名确定。
    PwmLock lock;  // 声明 lock，类型和可用范围由所在声明决定。
    if (!ready || percent > 100) return false;  // 若 !ready || percent > 100，立即返回 false。
    // Arduino LEDC 将 maxDuty 转换为持续高电平，确保 100% 全亮。
    brightness = static_cast<uint8_t>(percent);  // 将 static_cast<uint8_t>(percent) 写入或初始化 brightness。
    applyBrightness();  // 调用 applyBrightness，使用无参形式推进相应操作。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

void LedPwm_SetRingVisionActive(bool active) {  // 定义函数或成员 的入口 LedPwm_SetRingVisionActive（Led占空比输出Set圆环视觉活跃），参数由此处的签名确定。
    PwmLock lock;  // 声明 lock，类型和可用范围由所在声明决定。
    ringVisionActive = active;  // 将 active 写入或初始化 ringVisionActive（圆环视觉活跃）。
    if (ready) applyBrightness();  // 当 ready 成立时进入此分支，否则继续后续流程。
}  // 结束此函数或控制块。

/**
 * @brief 读取设定亮度百分比，不等于故障闪烁期间的瞬时实际输出。
 */
uint8_t LedPwm_GetBrightness() { PwmLock lock; return brightness; }  // 定义函数或成员 LedPwm_GetBrightness（Led占空比输出GetBrightness），参数由此处的签名确定。

/**
 * @brief 设置PWM频率Hz；返回是否成功应用。
 */
bool LedPwm_SetFrequency(uint32_t hz) {  // 定义函数或成员 的入口 LedPwm_SetFrequency（Led占空比输出SetFrequency），参数由此处的签名确定。
    PwmLock lock;  // 声明 lock，类型和可用范围由所在声明决定。
    if (!ready || hz < LED_PWM_MIN_FREQUENCY_HZ || hz > LED_PWM_MAX_FREQUENCY_HZ)  // 当 !ready || hz < LED_PWM_MIN_FREQUENCY_HZ || hz > LED_PW… 成立时进入此分支，否则继续后续流程。
        return false;  // 结束当前函数并返回 false。
    const uint32_t previous = frequencyHz;  // 将 frequencyHz 写入或初始化 previous（上一）。
    const uint32_t actual = ledcChangeFrequency(LED_PWM_CHANNEL, hz, LED_PWM_RESOLUTION_BITS);  // 将 ledcChangeFrequency(LED_PWM_CHANNEL, hz, LED_PWM_RESOL… 写入或初始化 actual（实测）。
    if (actual == 0) {  // 当 actual == 0 成立时进入此分支，否则继续后续流程。
        // 配置失败时尝试恢复原频率；恢复失败则关闭输出并报告 PWM 故障。
        const uint32_t restored = ledcChangeFrequency(LED_PWM_CHANNEL, previous, LED_PWM_RESOLUTION_BITS);  // 将 ledcChangeFrequency(LED_PWM_CHANNEL, previous, LED_PWM… 写入或初始化 restored。
        if (restored == 0) {  // 当 restored == 0 成立时进入此分支，否则继续后续流程。
            ledcWrite(LED_PWM_CHANNEL, 0);  // 调用 ledcWrite，传入 LED_PWM_CHANNEL, 0。
            brightness = 0;  // 将 0 写入或初始化 brightness。
            ready = false;  // 将 false 写入或初始化 ready（就绪）。
        } else {  // 前一个判断不成立时执行此分支。
            frequencyHz = restored;  // 将 restored 写入或初始化 frequencyHz。
            applyBrightness();  // 调用 applyBrightness，使用无参形式推进相应操作。
        }  // 结束此条件分支。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。
    frequencyHz = actual;  // 将 actual 写入或初始化 frequencyHz。
    applyBrightness();  // 调用 applyBrightness，使用无参形式推进相应操作。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

// Arduino 2.x 的 ledcReadFreq 在占空比为 0 时返回 0，故保存配置接口的回读值。
/**
 * @brief 读取当前配置的PWM频率Hz。
 */
uint32_t LedPwm_GetFrequency() { PwmLock lock; return ready ? frequencyHz : 0; }  // 定义函数或成员 LedPwm_GetFrequency（Led占空比输出GetFrequency），参数由此处的签名确定。

/**
 * @brief 周期更新故障闪烁；fault=true时覆盖实际输出，故障解除后恢复设定亮度。
 */
// 由周期调用驱动时间变化，停留在某个亮度的时长取决于 elapsed 对应的模式，而非调用次数。
void LedPwm_UpdateFaultBlink(bool fault) {  // 定义函数或成员 的入口 LedPwm_UpdateFaultBlink（Led占空比输出UpdateFaultBlink），参数由此处的签名确定。
    PwmLock lock;  // 声明 lock，类型和可用范围由所在声明决定。
    const uint32_t now = millis();  // 将 millis() 写入或初始化 now。
    if(cuePulses) {  // 当 cuePulses 成立时进入此分支，否则继续后续流程。
        const uint32_t phase=(now-cueStartMs)/cuePhaseMs;  // 将 (now-cueStartMs)/cuePhaseMs 写入或初始化 phase（阶段）。
        if(fault||phase>=cuePulses*2U+(cueLongMiddle?2U:0U))cuePulses=0;  // 当 fault||phase>=cuePulses*2U+(cueLongMiddle?2U:0U) 成立时进入此分支，否则继续后续流程。
        const bool on=cuePulses&&(cueLongMiddle?(phase==0||(phase>=2&&phase<=4)||phase==6):phase%2==0);  // 将 cuePulses&&(cueLongMiddle?(phase==0||(phase>=2&&phase<… 写入或初始化 on。
        cueBrightness=on?100:0;  // 将 on?100:0 写入或初始化 cueBrightness。
        BatteryMonitorSetCue(cuePulses!=0,on);  // 调用 BatteryMonitorSetCue，传入 cuePulses!=0,on。
        if(!fault){faultBlink=false;if(ready)applyBrightness();return;}  // 当 !fault 成立时进入此分支，否则继续后续流程。
    }  // 结束此条件分支。
    if(!ready)return;  // 当 !ready 成立时进入此分支，否则继续后续流程。
    if (fault) {  // 当 fault 成立时进入此分支，否则继续后续流程。
        if (!faultBlink) {  // 当 !faultBlink 成立时进入此分支，否则继续后续流程。
            faultBlink = true;  // 将 true 写入或初始化 faultBlink。
            faultEffectStartMs = now;  // 将 now 写入或初始化 faultEffectStartMs（faultEffect起点毫秒）。
        }  // 结束此条件分支。
        const uint8_t nextBrightness = faultPatternBrightness(now - faultEffectStartMs);  // 将 faultPatternBrightness(now - faultEffectStartMs) 写入或初始化 nextBrightness（下一Brightness）。
        if (nextBrightness == faultEffectBrightness) return;  // 当 nextBrightness == faultEffectBrightness 成立时进入此分支，否则继续后续流程。
        faultEffectBrightness = nextBrightness;  // 将 nextBrightness 写入或初始化 faultEffectBrightness。
        applyBrightness();  // 调用 applyBrightness，使用无参形式推进相应操作。
    } else if (faultBlink) {  // 仅在先前条件均不成立时，当 faultBlink 成立时进入此分支，否则继续后续流程。
        faultBlink = false;  // 将 false 写入或初始化 faultBlink。
        faultEffectBrightness = 0;  // 将 0 写入或初始化 faultEffectBrightness。
        applyBrightness();  // 调用 applyBrightness，使用无参形式推进相应操作。
    }  // 结束此条件分支。
}  // 结束此函数或控制块。

void LedPwm_PlayCue(uint8_t pulses) {  // 定义函数或成员 的入口 LedPwm_PlayCue（Led占空比输出PlayCue），参数由此处的签名确定。
    startCue(pulses,false);  // 调用 startCue（起点Cue），传入 pulses,false。
}  // 结束此函数或控制块。

void LedPwm_PlayCompletionCue() {  // 定义函数或成员 的入口 LedPwm_PlayCompletionCue（Led占空比输出PlayCompletionCue），参数由此处的签名确定。
    startCue(3,true);  // 调用 startCue（起点Cue），传入 3,true。
}  // 结束此函数或控制块。
