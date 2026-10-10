#pragma once
#include <stdint.h>

#if __cplusplus >= 201402L
#define DEBUG_RELEASE_CONSTEXPR constexpr
#else
#define DEBUG_RELEASE_CONSTEXPR
#endif

// 按下为true。上电先等待稳定松开；忙碌时按下不排队，长按只触发一次。
struct DebugReleaseButton {
    static constexpr uint32_t DEBOUNCE_MS = 40;
    bool rawPressed = true;
    bool stablePressed = true;
    bool armed = false;
    bool idle = false;
    bool allowedOnPress = false;
    bool pending = false;
    uint32_t changedAtMs = 0;

    DEBUG_RELEASE_CONSTEXPR void setIdle(bool value) {
        idle = value;
        if (!idle) pending = false;
    }

    DEBUG_RELEASE_CONSTEXPR void update(bool pressed, uint32_t nowMs) {
        if (pressed != rawPressed) {
            rawPressed = pressed;
            changedAtMs = nowMs;
            if (pressed) allowedOnPress = idle;
        }
        if (rawPressed == stablePressed || nowMs - changedAtMs < DEBOUNCE_MS) return;
        stablePressed = rawPressed;
        if (!stablePressed) {
            armed = true;
        } else {
            if (armed && allowedOnPress && idle) pending = true;
            armed = false;
        }
    }

    DEBUG_RELEASE_CONSTEXPR bool takeRequest() {
        const bool requested = idle && pending;
        pending = false;
        if (requested) idle = false;
        return requested;
    }
};

#undef DEBUG_RELEASE_CONSTEXPR
