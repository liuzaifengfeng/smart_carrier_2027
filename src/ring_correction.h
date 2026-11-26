#pragma once

#include <cmath>
#include <cstdint>
#include <mutex>

namespace ring {
constexpr float HOVER_MM = 6.0f;
constexpr float MAX_TURRET_CORRECTION_DEG = 10.0f;
constexpr float MAX_EXTENSION_CORRECTION_MM = 20.0f;
constexpr uint8_t TURRET_CORRECTION_ACCELERATION = 160;
constexpr uint16_t TURRET_CORRECTION_RPM = 10;
constexpr float RING_X_FINE_PX = 3.0f;
constexpr uint32_t RING_X_FINE_SEND_MS = 120;

inline uint32_t &requiredStableFrames() {
    static uint32_t value = 2;
    return value;
}

inline uint32_t &configuredTimeoutMs() {
    static uint32_t value = 2000;
    return value;
}

struct Deadzone {
    float x = 1.0f, y = 1.0f;
};

inline Deadzone &configuredDeadzone() {
    static Deadzone value;
    return value;
}

struct Gains {
    float kp, ki, kd;
};

struct Settings {
    Gains turret{0.02f, 0.047f, 0}; // X 像素 → 转台角度修正（deg）
    Gains extension{0.053f, 0.038f, 0}; // Y 像素 → 伸缩位移修正（mm）
};

struct Frame {
    float x = 0, y = 0;
    uint32_t timeMs = 0;
};

inline bool withinDeadzone(float error, float limit) {
    return std::fabs(error) < limit;
}

inline bool withinDeadzone(const Frame &frame, const Deadzone &limits) {
    return withinDeadzone(frame.x, limits.x) && withinDeadzone(frame.y, limits.y);
}

class VisionInput {
    std::mutex mutex;
    Settings config;
    Deadzone deadzone;
    Frame latest;
    uint32_t generation = 0;
    uint32_t required = 0, consecutive = 0;
    bool accepting = false, pending = false;
public:
    Settings settings() {
        std::lock_guard<std::mutex> lock(mutex);
        return config;
    }
    void settings(const Settings &value) {
        std::lock_guard<std::mutex> lock(mutex);
        config = value;
    }
    uint32_t ready(uint32_t requiredFrames, const Deadzone &limits) {
        std::lock_guard<std::mutex> lock(mutex);
        required = requiredFrames;
        deadzone = limits;
        consecutive = 0;
        pending = false;
        accepting = true;
        return ++generation;
    }
    void close() {
        std::lock_guard<std::mutex> lock(mutex);
        accepting = pending = false;
    }
    bool submit(uint32_t token, const Frame &frame) {
        std::lock_guard<std::mutex> lock(mutex);
        if (!accepting || token != generation) return false;
        if (consecutive == required) return true;
        consecutive = withinDeadzone(frame, deadzone) ? consecutive + 1 : 0;
        latest = frame;
        pending = true;
        return true;
    }
    bool take(Frame &frame, bool &stable) {
        std::lock_guard<std::mutex> lock(mutex);
        if (!pending) return false;
        frame = latest;
        stable = consecutive == required;
        pending = false;
        return true;
    }
};

inline VisionInput &input() {
    static VisionInput instance;
    return instance;
}

struct Pid {
    float integral = 0, previous = 0;
    bool hasPrevious = false;

    float update(float error, float dt, const Gains &cfg) {
        float derivative = 0;
        if (hasPrevious && dt > 0) {
            if (cfg.ki != 0) integral += error * dt;
            derivative = (error - previous) / dt;
        }
        hasPrevious = true;
        previous = error;
        return cfg.kp * error + cfg.ki * integral + cfg.kd * derivative;
    }
};

}
