#include <stdint.h>
#include <assert.h>
using SemaphoreHandle_t=void*;
constexpr int LOW=0,OUTPUT=1,portMAX_DELAY=0;
uint32_t clockMs=0,duty=0;
uint32_t millis(){return clockMs;}
SemaphoreHandle_t xSemaphoreCreateMutex(){return reinterpret_cast<void*>(1);}
void xSemaphoreTake(SemaphoreHandle_t,int){}
void xSemaphoreGive(SemaphoreHandle_t){}
void digitalWrite(int,int){}
void pinMode(int,int){}
void ledcWrite(int,uint32_t value){duty=value;}
void ledcAttachPin(int,int){}
uint32_t ledcSetup(int,uint32_t hz,int){return hz;}
uint32_t ledcChangeFrequency(int,uint32_t hz,int){return hz;}
#include "led_pwm_actual.inc"
int main() {
    assert(LedPwm_Init()&&LedPwm_SetBrightness(37));
    const uint32_t saved=duty;
    LedPwm_BlinkCalibrationComplete();assert(duty==4095);
    clockMs=199;LedPwm_UpdateFaultBlink(false);assert(duty==4095);
    clockMs=200;LedPwm_UpdateFaultBlink(false);assert(duty==0);
    clockMs=400;LedPwm_UpdateFaultBlink(false);assert(duty==4095);
    clockMs=600;LedPwm_UpdateFaultBlink(false);assert(duty==0);
    clockMs=800;LedPwm_UpdateFaultBlink(false);assert(duty==saved&&LedPwm_GetBrightness()==37);
    LedPwm_BlinkCalibrationComplete();clockMs=1000;LedPwm_UpdateFaultBlink(true);assert(duty==4095);
    clockMs=1080;LedPwm_UpdateFaultBlink(true);assert(duty==0);
    LedPwm_UpdateFaultBlink(false);assert(duty==saved);
}
