// 启动面板引脚、任务活动判据及指示灯占空比计算。
// 起始区和启动请求的采样处理见 serial_commands.inc。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include <stdint.h>  // 引入 stdint.h 中的声明和定义。
#include "robot_state.h"  // 引入 robot_state.h 中的声明和定义。

constexpr uint8_t START_ZONE_SWITCH_PIN=1; // ZONE_ADC 网络，R13 已拆除
constexpr uint8_t START_BUTTON_PIN=2; // START_ADC 网络，R10 已短接
constexpr uint8_t BUTTON_LED_PIN=11;  // 将 11 写入或初始化 BUTTON_LED_PIN。
constexpr uint8_t BUTTON_LED_PWM_CHANNEL=2; // 与补光灯通道 0 使用不同定时器
constexpr uint32_t BUTTON_LED_PWM_HZ=1000;  // 将 1000 写入或初始化 BUTTON_LED_PWM_HZ（BUTTONLED占空比输出HZ）。
constexpr uint8_t BUTTON_LED_PWM_BITS=8;  // 将 8 写入或初始化 BUTTON_LED_PWM_BITS（BUTTONLED占空比输出BITS）。
constexpr uint32_t BUTTON_LED_FULL_DUTY=(1U<<BUTTON_LED_PWM_BITS)-1;  // 将 (1U<<BUTTON_LED_PWM_BITS)-1 写入或初始化 BUTTON_LED_FULL_DUTY（BUTTONLEDFULL占空比）。

enum class StartPanelLight { OFF, STEADY, SINGLE_FLASH, DOUBLE_FLASH, BREATH };  // 定义 StartPanelLight（起点PanelLight） 的类型成员或作用域。

constexpr bool StartPanelTaskActive(RobotState state) {  // 定义函数或成员 的入口 StartPanelTaskActive（起点Panel任务活跃），参数由此处的签名确定。
    return state==STATE_READ_TASK||(state>=STATE_GRAB_ROUND1&&state<=STATE_RETURN_HOME);  // 结束当前函数并返回 state==STATE_READ_TASK||(state>=STATE_GRAB_ROUND1&&sta…。
}  // 结束此函数或控制块。

// 调用方选定灯效并传入经过时间。
// 闪烁以 1000 ms 为周期，呼吸在 2000 ms 内线性升降占空比，函数只返回占空比供外部写 PWM。
inline uint32_t StartPanelLightDuty(StartPanelLight light,uint32_t elapsedMs) {  // 定义函数或成员 的入口 StartPanelLightDuty（起点PanelLight占空比），参数由此处的签名确定。
    switch(light) {  // 按 light 的值选择对应分支。
    case StartPanelLight::STEADY: return BUTTON_LED_FULL_DUTY;  // 当前值匹配 StartPanelLight::STEADY 时从这里执行。
    case StartPanelLight::SINGLE_FLASH:  // 当前值匹配 StartPanelLight::SINGLE_FLASH 时从这里执行。
    case StartPanelLight::DOUBLE_FLASH: {  // 当前值匹配 StartPanelLight::DOUBLE_FLASH 时从这里执行。
        const uint32_t phase=elapsedMs%1000;  // 将 elapsedMs%1000 写入或初始化 phase（阶段）。
        return phase<100||(light==StartPanelLight::DOUBLE_FLASH&&phase>=200&&phase<300)  // 结束当前函数并返回 phase<100||(light==StartPanelLight::DOUBLE_FLASH&&phas…。
            ?BUTTON_LED_FULL_DUTY:0;  // 续写当前声明或运算，提供 ?BUTTON_LED_FULL_DUTY:0;。
    }  // 结束此条件分支。
    case StartPanelLight::BREATH: {  // 当前值匹配 StartPanelLight::BREATH 时从这里执行。
        const uint32_t phase=elapsedMs%2000;  // 将 elapsedMs%2000 写入或初始化 phase（阶段）。
        const uint32_t ramp=phase<1000?phase:2000-phase;  // 将 phase<1000?phase:2000-phase 写入或初始化 ramp。
        return ramp*BUTTON_LED_FULL_DUTY/1000;  // 结束当前函数并返回 ramp*BUTTON_LED_FULL_DUTY/1000。
    }  // 结束此条件分支。
    case StartPanelLight::OFF: return 0;  // 当前值匹配 StartPanelLight::OFF 时从这里执行。
    }  // 结束此函数或控制块。
    return 0;  // 结束当前函数并返回 0。
}  // 结束此函数或控制块。
