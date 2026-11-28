// 视觉对齐辅助逻辑及输入数据。
// 上层按视觉工作模式选择对齐行为，底盘闭环以收到的误差计算控制输出。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。

#include <stdint.h>  // 引入 stdint.h 中的声明和定义。
#include <math.h>  // 引入 math.h 中的声明和定义。

enum class VisionStartMode : uint8_t {  // 定义 VisionStartMode（视觉起点模式） 的类型成员或作用域。
    DISC, DISC_MATERIAL, WORK_AREA, WORK_AREA_LOADED, CORNER, NONE  // 声明 NONE，类型和可用范围由所在声明决定。
};  // 结束此类型定义。

/**
 * @brief 判断该视觉模式是否需要底盘对齐；单件圆盘识别不属于底盘整体对齐。
 */
constexpr bool VisionModeUsesAlignment(VisionStartMode mode) {  // 定义函数或成员 的入口 VisionModeUsesAlignment（视觉模式UsesAlignment），参数由此处的签名确定。
    return mode == VisionStartMode::DISC || mode == VisionStartMode::WORK_AREA  // 结束当前函数并返回 mode == VisionStartMode::DISC || mode == VisionStartMo…。
        || mode == VisionStartMode::WORK_AREA_LOADED || mode == VisionStartMode::CORNER;  // 继续上一行的组合条件，本行提供 || mode == VisionStartMode::WORK_AREA_LOADED || mode == VisionStartMode::CORNER;。
}  // 结束此函数或控制块。

struct VisionAlignmentDeadzone {  // 定义 VisionAlignmentDeadzone（视觉AlignmentDeadzone） 的类型成员或作用域。
    float visualX;  // 声明 visualX（visual横向），类型和可用范围由所在声明决定。
    float visualY;  // 声明 visualY（visual纵向），类型和可用范围由所在声明决定。
    float angleDeg;  // 声明 angleDeg（角度角度），类型和可用范围由所在声明决定。
};  // 结束此类型定义。

/**
 * @brief 检查视觉X、Y及角度误差是否同时进入指定死区；XY单位沿用视觉输出。
 */
constexpr bool IsWithinAlignmentDeadzone(const VisionAlignmentDeadzone &zone, float angle, float x, float y) {  // 定义函数或成员 的入口 IsWithinAlignmentDeadzone，参数由此处的签名确定。
    return (x<0?-x:x) <= zone.visualX && (y<0?-y:y) <= zone.visualY  // 结束当前函数并返回 (x<0?-x:x) <= zone.visualX && (y<0?-y:y) <= zone.visua…。
        && (angle<0?-angle:angle) <= zone.angleDeg;  // 继续上一行的组合条件，本行提供 && (angle<0?-angle:angle) <= zone.angleDeg;。
}  // 结束此函数或控制块。

constexpr uint8_t ALIGNMENT_REQUIRED_FRAMES = 2;  // 将 2 写入或初始化 ALIGNMENT_REQUIRED_FRAMES。
constexpr uint8_t DISC_ALIGNMENT_REQUIRED_FRAMES = 1;  // 将 1 写入或初始化 DISC_ALIGNMENT_REQUIRED_FRAMES（圆盘ALIGNMENTREQUIREDFRAMES）。
constexpr uint8_t WORK_AREA_ALIGNMENT_REQUIRED_FRAMES = 5;  // 将 5 写入或初始化 WORK_AREA_ALIGNMENT_REQUIRED_FRAMES（工位AREAALIGNMENTREQUIREDFRAMES）。

constexpr uint8_t RequiredAlignmentFrames(VisionStartMode mode) {  // 定义函数或成员 的入口 RequiredAlignmentFrames，参数由此处的签名确定。
    return mode == VisionStartMode::DISC  // 结束当前函数并返回 mode == VisionStartMode::DISC。
        ? DISC_ALIGNMENT_REQUIRED_FRAMES  // 续写上一行表达式的条件取值分支。
        : mode == VisionStartMode::WORK_AREA ? WORK_AREA_ALIGNMENT_REQUIRED_FRAMES : ALIGNMENT_REQUIRED_FRAMES;  // 续写上一行表达式的条件取值分支。
}  // 结束此函数或控制块。

/**
 * @brief 根据本帧死区判定和模式是否连续，更新连续到位帧数。
 */
constexpr uint8_t NextAlignmentFrameCount(uint8_t count, bool withinDeadzone, bool sameMode,  // 定义函数或成员 NextAlignmentFrameCount（下一Alignment帧计数），参数由此处的签名确定。
                                         uint8_t requiredFrames = ALIGNMENT_REQUIRED_FRAMES) {  // 将 ALIGNMENT_REQUIRED_FRAMES) { 写入或初始化 requiredFrames。
    return !withinDeadzone ? 0 : !sameMode ? 1  // 结束当前函数并返回 !withinDeadzone ? 0 : !sameMode ? 1。
        : count < requiredFrames ? count + 1 : requiredFrames;  // 续写上一行表达式的条件取值分支。
}  // 结束此初始化列表或局部作用域。

// 每个新反馈帧调用一次；重复轮询不得调用 update。
// 记录等待对齐时的帧判定状态。
// 它与底盘 PID 分工，调用者在启动新对齐时通过运行层重置相应状态。
struct AlignmentFrameWaiter {  // 定义 AlignmentFrameWaiter（Alignment帧Waiter） 的类型成员或作用域。
    static constexpr uint8_t REQUIRED_FRAMES = ALIGNMENT_REQUIRED_FRAMES;  // 将 ALIGNMENT_REQUIRED_FRAMES 写入或初始化 REQUIRED_FRAMES。
    uint8_t consecutiveFrames = 0;  // 将 0 写入或初始化 consecutiveFrames。
    VisionStartMode mode = VisionStartMode::NONE;  // 将 VisionStartMode::NONE 写入或初始化 mode（模式）。

    /**
     * @brief 清空连续到位帧数与视觉模式，开始新一次对齐判定。
     */
    void reset() {  // 定义函数或成员 的入口 reset，参数由此处的签名确定。
        consecutiveFrames = 0;  // 将 0 写入或初始化 consecutiveFrames。
        mode = VisionStartMode::NONE;  // 将 VisionStartMode::NONE 写入或初始化 mode（模式）。
    }  // 结束此函数或控制块。

    /**
     * @brief 处理一帧新的视觉反馈，更新连续到位帧数；重复轮询同一帧不得调用。
     * @return true 连续到位帧数达到要求。
     */
    bool update(VisionStartMode newMode, bool withinDeadzone) {  // 定义函数或成员 的入口 update，参数由此处的签名确定。
        const uint8_t requiredFrames = RequiredAlignmentFrames(newMode);  // 将 RequiredAlignmentFrames(newMode) 写入或初始化 requiredFrames。
        consecutiveFrames = NextAlignmentFrameCount(consecutiveFrames,  // 将 NextAlignmentFrameCount(consecutiveFrames, 写入或初始化 consecutiveFrames。
            newMode != VisionStartMode::NONE && withinDeadzone, mode == newMode, requiredFrames);  // 续写当前声明或运算，提供 newMode != VisionStartMode::NONE && withinDeadzone, mo…。
        mode = newMode;  // 将 newMode 写入或初始化 mode（模式）。
        return consecutiveFrames >= requiredFrames;  // 结束当前函数并返回 consecutiveFrames >= requiredFrames。
    }  // 结束此函数或控制块。
};  // 结束此类型定义。
