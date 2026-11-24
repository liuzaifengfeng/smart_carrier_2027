#pragma once
#include <stdint.h>
#include "robot_state.h"

constexpr uint8_t START_ZONE_SWITCH_PIN=1; // ZONE_ADC 网络，R13 已拆除
constexpr uint8_t START_BUTTON_PIN=2; // START_ADC 网络，R10 已短接
constexpr uint8_t BUTTON_LED_PIN=11;
constexpr uint8_t BUTTON_LED_PWM_CHANNEL=2; // 与补光灯通道 0 使用不同定时器
constexpr uint32_t BUTTON_LED_PWM_HZ=1000;
constexpr uint8_t BUTTON_LED_PWM_BITS=8;
constexpr uint32_t BUTTON_LED_FULL_DUTY=(1U<<BUTTON_LED_PWM_BITS)-1;

enum class StartPanelLight { OFF, STEADY, SINGLE_FLASH, DOUBLE_FLASH, BREATH };

constexpr bool StartPanelTaskActive(RobotState state) {
    return state==STATE_READ_TASK||(state>=STATE_GRAB_ROUND1&&state<=STATE_RETURN_HOME);
}

inline uint32_t StartPanelLightDuty(StartPanelLight light,uint32_t elapsedMs) {
    switch(light) {
    case StartPanelLight::STEADY: return BUTTON_LED_FULL_DUTY;
    case StartPanelLight::SINGLE_FLASH:
    case StartPanelLight::DOUBLE_FLASH: {
        const uint32_t phase=elapsedMs%1000;
        return phase<100||(light==StartPanelLight::DOUBLE_FLASH&&phase>=200&&phase<300)
            ?BUTTON_LED_FULL_DUTY:0;
    }
    case StartPanelLight::BREATH: {
        const uint32_t phase=elapsedMs%2000;
        const uint32_t ramp=phase<1000?phase:2000-phase;
        return ramp*BUTTON_LED_FULL_DUTY/1000;
    }
    case StartPanelLight::OFF: return 0;
    }
    return 0;
}
