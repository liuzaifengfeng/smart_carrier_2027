#include <stdint.h>
#include <assert.h>
using SemaphoreHandle_t=void*;
constexpr int LOW=0,OUTPUT=1,portMAX_DELAY=0;
uint32_t clockMs=0,duty=0;
bool cueActive=false,cueOn=false;
void BatteryMonitorSetCue(bool active,bool on){cueActive=active;cueOn=on;}
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
    LedPwm_PlayCue(2);assert(duty==4095&&cueActive&&cueOn);
    clockMs=66;LedPwm_UpdateFaultBlink(false);assert(duty==4095);
    clockMs=67;LedPwm_UpdateFaultBlink(false);assert(duty==0&&cueActive&&!cueOn);
    clockMs=134;LedPwm_UpdateFaultBlink(false);assert(duty==4095&&cueOn);
    clockMs=201;LedPwm_UpdateFaultBlink(false);assert(duty==0&&!cueOn);
    clockMs=268;LedPwm_UpdateFaultBlink(false);assert(duty==saved&&LedPwm_GetBrightness()==37&&!cueActive);
    LedPwm_PlayCue(3);
    for(uint32_t i=0;i<6;++i) {
        clockMs=268+i*67;LedPwm_UpdateFaultBlink(false);
        assert(cueActive&&cueOn==(i%2==0)&&duty==(cueOn?4095:0));
    }
    clockMs=670;LedPwm_UpdateFaultBlink(false);assert(duty==saved&&!cueActive);
    LedPwm_PlayCompletionCue();
    const bool completionPhases[]={true,false,true,true,true,false,true,false};
    for(uint32_t i=0;i<8;++i) {
        clockMs=670+i*67;LedPwm_UpdateFaultBlink(false);
        assert(cueActive&&cueOn==completionPhases[i]&&duty==(cueOn?4095:0));
    }
    clockMs=1206;LedPwm_UpdateFaultBlink(false);assert(duty==saved&&!cueActive);
    LedPwm_PlayCue(2);clockMs=1250;LedPwm_UpdateFaultBlink(true);assert(duty==4095&&!cueActive);
    clockMs=1330;LedPwm_UpdateFaultBlink(true);assert(duty==0);
    LedPwm_UpdateFaultBlink(false);assert(duty==saved);
}
