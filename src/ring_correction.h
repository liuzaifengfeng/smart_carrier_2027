// 圆环视觉修正所用的估算和数据处理辅助代码。
// 分析放料偏差时应区分视觉误差和机械臂需要执行的修正量。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。

#include <cmath>  // 引入 cmath 中的声明和定义。
#include <cstdint>  // 引入 cstdint 中的声明和定义。
#include <mutex>  // 引入 mutex 中的声明和定义。

namespace ring {  // 定义 ring（圆环） 的类型成员或作用域。
constexpr float HOVER_MM = 6.0f;  // 将 6.0f 写入或初始化 HOVER_MM（HOVER毫米）。
constexpr float MAX_TURRET_CORRECTION_DEG = 10.0f;  // 将 10.0f 写入或初始化 MAX_TURRET_CORRECTION_DEG（最大转台修正量角度）。
constexpr float MAX_EXTENSION_CORRECTION_MM = 20.0f;  // 将 20.0f 写入或初始化 MAX_EXTENSION_CORRECTION_MM（最大伸缩修正量毫米）。
constexpr uint8_t TURRET_CORRECTION_ACCELERATION = 160;  // 将 160 写入或初始化 TURRET_CORRECTION_ACCELERATION（转台修正量加速度）。
constexpr uint16_t TURRET_CORRECTION_RPM = 10;  // 将 10 写入或初始化 TURRET_CORRECTION_RPM（转台修正量转速）。
constexpr float RING_X_FINE_PX = 3.0f;  // 将 3.0f 写入或初始化 RING_X_FINE_PX（圆环横向FINEPX）。
constexpr uint32_t RING_X_FINE_SEND_MS = 120;  // 将 120 写入或初始化 RING_X_FINE_SEND_MS（圆环横向FINESEND毫秒）。

inline uint32_t &requiredStableFrames() {  // 定义函数或成员 的入口 requiredStableFrames（required稳定Frames），参数由此处的签名确定。
    static uint32_t value = 2;  // 将 2 写入或初始化 value（测值）。
    return value;  // 结束当前函数并返回 value。
}  // 结束此函数或控制块。

inline uint32_t &configuredTimeoutMs() {  // 定义函数或成员 的入口 configuredTimeoutMs（configured超时毫秒），参数由此处的签名确定。
    static uint32_t value = 1100;  // 将 1100 写入或初始化 value（测值）。
    return value;  // 结束当前函数并返回 value。
}  // 结束此函数或控制块。

struct Deadzone {  // 定义 Deadzone 的类型成员或作用域。
    float x = 1.0f, y = 1.0f;  // 将 1.0f, y = 1.0f 写入或初始化 x（横向）。
};  // 结束此类型定义。

inline Deadzone &configuredDeadzone() {  // 定义函数或成员 的入口 configuredDeadzone，参数由此处的签名确定。
    static Deadzone value;  // 声明 value（测值），类型和可用范围由所在声明决定。
    return value;  // 结束当前函数并返回 value。
}  // 结束此函数或控制块。

struct Gains {  // 定义 Gains 的类型成员或作用域。
    float kp, ki, kd;  // 声明 kd，类型和可用范围由所在声明决定。
};  // 结束此类型定义。

struct Settings {  // 定义 Settings（设置） 的类型成员或作用域。
    Gains turret{0.04f, 0.047f, 0}; // X 像素 → 转台角度修正（deg）
    Gains extension{0.053f, 0.038f, 0}; // Y 像素 → 伸缩位移修正（mm）
};  // 结束此类型定义。

struct Frame {  // 定义 Frame（帧） 的类型成员或作用域。
    float x = 0, y = 0;  // 将 0, y = 0 写入或初始化 x（横向）。
    uint32_t timeMs = 0;  // 将 0 写入或初始化 timeMs（时间毫秒）。
};  // 结束此类型定义。

inline bool withinDeadzone(float error, float limit) {  // 定义函数或成员 的入口 withinDeadzone，参数由此处的签名确定。
    return std::fabs(error) < limit;  // 结束当前函数并返回 std::fabs(error) < limit。
}  // 结束此函数或控制块。

inline bool withinDeadzone(const Frame &frame, const Deadzone &limits) {  // 定义函数或成员 的入口 withinDeadzone，参数由此处的签名确定。
    return withinDeadzone(frame.x, limits.x) && withinDeadzone(frame.y, limits.y);  // 结束当前函数并返回 withinDeadzone(frame.x, limits.x) && withinDeadzone(fr…。
}  // 结束此函数或控制块。

// 此处保存视觉修正帧和会话状态。
// 启停与取帧连在同一动作链上，排查旧帧时可对照 MotionHardware 内开启修正和结束清理的位置。
class VisionInput {  // 定义 VisionInput（视觉输入） 的类型成员或作用域。
    std::mutex mutex;  // 声明 mutex，类型和可用范围由所在声明决定。
    Settings config;  // 声明 config（配置），类型和可用范围由所在声明决定。
    Deadzone deadzone;  // 声明 deadzone，类型和可用范围由所在声明决定。
    Frame latest;  // 声明 latest，类型和可用范围由所在声明决定。
    uint32_t generation = 0;  // 将 0 写入或初始化 generation。
    uint32_t required = 0, consecutive = 0;  // 将 0, consecutive = 0 写入或初始化 required。
    bool accepting = false, pending = false;  // 将 false, pending = false 写入或初始化 accepting。
public:  // 此后的成员采用 public 访问权限，直到下一个权限标签。
    Settings settings() {  // 定义函数或成员 的入口 settings（设置），参数由此处的签名确定。
        std::lock_guard<std::mutex> lock(mutex);  // 声明接口 lock，参数由此处的签名确定。
        return config;  // 结束当前函数并返回 config。
    }  // 结束此函数或控制块。
    void settings(const Settings &value) {  // 定义函数或成员 的入口 settings（设置），参数由此处的签名确定。
        std::lock_guard<std::mutex> lock(mutex);  // 声明接口 lock，参数由此处的签名确定。
        config = value;  // 将 value 写入或初始化 config（配置）。
    }  // 结束此函数或控制块。
    uint32_t ready(uint32_t requiredFrames, const Deadzone &limits) {  // 定义函数或成员 的入口 ready（就绪），参数由此处的签名确定。
        std::lock_guard<std::mutex> lock(mutex);  // 声明接口 lock，参数由此处的签名确定。
        required = requiredFrames;  // 将 requiredFrames 写入或初始化 required。
        deadzone = limits;  // 将 limits 写入或初始化 deadzone。
        consecutive = 0;  // 将 0 写入或初始化 consecutive。
        pending = false;  // 将 false 写入或初始化 pending。
        accepting = true;  // 将 true 写入或初始化 accepting。
        return ++generation;  // 结束当前函数并返回 ++generation。
    }  // 结束此函数或控制块。
    void close() {  // 定义函数或成员 的入口 close，参数由此处的签名确定。
        std::lock_guard<std::mutex> lock(mutex);  // 声明接口 lock，参数由此处的签名确定。
        accepting = pending = false;  // 将 pending = false 写入或初始化 accepting。
    }  // 结束此函数或控制块。
    bool submit(uint32_t token, const Frame &frame) {  // 定义函数或成员 的入口 submit，参数由此处的签名确定。
        std::lock_guard<std::mutex> lock(mutex);  // 声明接口 lock，参数由此处的签名确定。
        if (!accepting || token != generation) return false;  // 若 !accepting || token != generation，立即返回 false。
        if (consecutive == required) return true;  // 若 consecutive == required，立即返回 true。
        consecutive = withinDeadzone(frame, deadzone) ? consecutive + 1 : 0;  // 将 withinDeadzone(frame, deadzone) ? consecutive + 1 : 0 写入或初始化 consecutive。
        latest = frame;  // 将 frame 写入或初始化 latest。
        pending = true;  // 将 true 写入或初始化 pending。
        return true;  // 结束当前函数并返回 true。
    }  // 结束此函数或控制块。
    bool take(Frame &frame, bool &stable) {  // 定义函数或成员 的入口 take，参数由此处的签名确定。
        std::lock_guard<std::mutex> lock(mutex);  // 声明接口 lock，参数由此处的签名确定。
        if (!pending) return false;  // 若 !pending，立即返回 false。
        frame = latest;  // 将 latest 写入或初始化 frame（帧）。
        stable = consecutive == required;  // 将 consecutive == required 写入或初始化 stable（稳定）。
        pending = false;  // 将 false 写入或初始化 pending。
        return true;  // 结束当前函数并返回 true。
    }  // 结束此函数或控制块。
};  // 结束此类型定义。

inline VisionInput &input() {  // 定义函数或成员 的入口 input（输入），参数由此处的签名确定。
    static VisionInput instance;  // 声明 instance，类型和可用范围由所在声明决定。
    return instance;  // 结束当前函数并返回 instance。
}  // 结束此函数或控制块。

struct Pid {  // 定义 Pid 的类型成员或作用域。
    float integral = 0, previous = 0;  // 将 0, previous = 0 写入或初始化 integral（积分）。
    bool hasPrevious = false;  // 将 false 写入或初始化 hasPrevious（has上一）。

    float update(float error, float dt, const Gains &cfg) {  // 定义函数或成员 的入口 update，参数由此处的签名确定。
        float derivative = 0;  // 将 0 写入或初始化 derivative（微分）。
        if (hasPrevious && dt > 0) {  // 当 hasPrevious && dt > 0 成立时进入此分支，否则继续后续流程。
            if (cfg.ki != 0) integral += error * dt;  // 当 cfg.ki != 0 成立时进入此分支，否则继续后续流程。
            derivative = (error - previous) / dt;  // 将 (error - previous) / dt 写入或初始化 derivative（微分）。
        }  // 结束此条件分支。
        hasPrevious = true;  // 将 true 写入或初始化 hasPrevious（has上一）。
        previous = error;  // 将 error 写入或初始化 previous（上一）。
        return cfg.kp * error + cfg.ki * integral + cfg.kd * derivative;  // 结束当前函数并返回 cfg.kp * error + cfg.ki * integral + cfg.kd * derivati…。
    }  // 结束此函数或控制块。
};  // 结束此类型定义。

}  // 结束命名空间。
