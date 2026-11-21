#pragma once

#include <cmath>
#include <cstdint>
#include <mutex>

namespace ring {
constexpr float HOVER_MM = 10.0f;
constexpr float DEADZONE_PX = 1.0f;
constexpr float MAX_TURRET_CORRECTION_DEG = 10.0f;
constexpr float MAX_EXTENSION_CORRECTION_MM = 20.0f;
constexpr uint8_t TURRET_CORRECTION_ACCELERATION = 200;

inline uint32_t &requiredStableFrames() {
    static uint32_t value = 2;
    return value;
}

struct Gains {
    float kp, ki, kd;
};

struct Settings {
    Gains turret{0.06f, 0, 0}; // X 像素 → 转台角度修正（deg）
    Gains extension{0.05f, 0, 0}; // Y 像素 → 伸缩位移修正（mm）
};

struct Frame {
    float x = 0, y = 0;
    uint32_t timeMs = 0;
};

inline bool withinDeadzone(float error) {
    return std::fabs(error) < DEADZONE_PX;
}

inline bool withinDeadzone(const Frame &frame) {
    return withinDeadzone(frame.x) && withinDeadzone(frame.y);
}

class VisionInput {
    std::mutex mutex;
    Settings config;
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
    uint32_t ready(uint32_t requiredFrames) {
        std::lock_guard<std::mutex> lock(mutex);
        required = requiredFrames;
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
        if (!withinDeadzone(frame)) consecutive = 0;
        else if (consecutive < required) ++consecutive;
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
