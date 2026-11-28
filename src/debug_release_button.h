// Debug 模式释放按键的输入状态判断。
// 将按键采样与模式切换请求拆开，实际切换在运行层统一处理。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include <stdint.h>  // 引入 stdint.h 中的声明和定义。

#if __cplusplus >= 201402L  // 在编译阶段判断是否纳入此分支。
#define DEBUG_RELEASE_CONSTEXPR constexpr  // 为后续文本展开定义宏 DEBUG_RELEASE_CONSTEXPR constexpr。
#else  // 前面的编译条件均未选中时使用此分支。
#define DEBUG_RELEASE_CONSTEXPR  // 为后续文本展开定义宏 DEBUG_RELEASE_CONSTEXPR。
#endif  // 结束本组条件编译。

// 按下为true。上电先等待稳定松开；忙碌时按下不排队，长按只触发一次。
struct DebugReleaseButton {  // 定义 DebugReleaseButton 的类型成员或作用域。
    static constexpr uint32_t DEBOUNCE_MS = 40;  // 将 40 写入或初始化 DEBOUNCE_MS（DEBOUNCE毫秒）。
    bool rawPressed = true;  // 将 true 写入或初始化 rawPressed。
    bool stablePressed = true;  // 将 true 写入或初始化 stablePressed（稳定Pressed）。
    bool armed = false;  // 将 false 写入或初始化 armed。
    bool idle = false;  // 将 false 写入或初始化 idle。
    bool allowedOnPress = false;  // 将 false 写入或初始化 allowedOnPress。
    bool pending = false;  // 将 false 写入或初始化 pending。
    uint32_t changedAtMs = 0;  // 将 0 写入或初始化 changedAtMs（changedAt毫秒）。

    /**
     * @brief 设置是否允许按钮触发Release；忙碌时清除待处理请求。
     */
    DEBUG_RELEASE_CONSTEXPR void setIdle(bool value) {  // 定义函数或成员 的入口 setIdle，参数由此处的签名确定。
        idle = value;  // 将 value 写入或初始化 idle。
        if (!idle) pending = false;  // 当 !idle 成立时进入此分支，否则继续后续流程。
    }  // 结束此函数或控制块。

    /**
     * @brief 输入按钮电平采样和当前毫秒数，完成消抖与单次按下判定；不直接切换模式。
     */
    DEBUG_RELEASE_CONSTEXPR void update(bool pressed, uint32_t nowMs) {  // 定义函数或成员 的入口 update，参数由此处的签名确定。
        if (pressed != rawPressed) {  // 当 pressed != rawPressed 成立时进入此分支，否则继续后续流程。
            rawPressed = pressed;  // 将 pressed 写入或初始化 rawPressed。
            changedAtMs = nowMs;  // 将 nowMs 写入或初始化 changedAtMs（changedAt毫秒）。
            if (pressed) allowedOnPress = idle;  // 当 pressed 成立时进入此分支，否则继续后续流程。
        }  // 结束此条件分支。
        if (rawPressed == stablePressed || nowMs - changedAtMs < DEBOUNCE_MS) return;  // 当 rawPressed == stablePressed || nowMs - changedAtMs < D… 成立时进入此分支，否则继续后续流程。
        stablePressed = rawPressed;  // 将 rawPressed 写入或初始化 stablePressed（稳定Pressed）。
        if (!stablePressed) {  // 当 !stablePressed 成立时进入此分支，否则继续后续流程。
            armed = true;  // 将 true 写入或初始化 armed。
        } else {  // 前一个判断不成立时执行此分支。
            if (armed && allowedOnPress && idle) pending = true;  // 当 armed && allowedOnPress && idle 成立时进入此分支，否则继续后续流程。
            armed = false;  // 将 false 写入或初始化 armed。
        }  // 结束此条件分支。
    }  // 结束此函数或控制块。

    /**
     * @brief 消费一次允许执行的Release请求；成功消费后退出空闲状态。
     * @return true 本次有请求，false 无请求。
     */
    DEBUG_RELEASE_CONSTEXPR bool takeRequest() {  // 定义函数或成员 的入口 takeRequest（take请求），参数由此处的签名确定。
        const bool requested = idle && pending;  // 将 idle && pending 写入或初始化 requested。
        pending = false;  // 将 false 写入或初始化 pending。
        if (requested) idle = false;  // 当 requested 成立时进入此分支，否则继续后续流程。
        return requested;  // 结束当前函数并返回 requested。
    }  // 结束此函数或控制块。
};  // 结束此类型定义。

#undef DEBUG_RELEASE_CONSTEXPR  // 撤销宏 DEBUG_RELEASE_CONSTEXPR，限定其作用范围。
