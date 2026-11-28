// 在可选的等效角度中规划机械臂转盘路径。
// 检查规划输出时应结合当前角度及连续多个目标，不只看单次目标。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。

#include <float.h>  // 引入 float.h 中的声明和定义。
#include <stddef.h>  // 引入 stddef.h 中的声明和定义。
#include <stdint.h>  // 引入 stdint.h 中的声明和定义。

constexpr size_t TURRET_PATH_MAX_TARGETS = 6; // 三件物料，各有取料、放料两个目标。

struct TurretPathPlan {  // 定义 TurretPathPlan（转台路径Plan） 的类型成员或作用域。
    float anglesDeg[TURRET_PATH_MAX_TARGETS] = {};  // 将 {} 写入或初始化 anglesDeg[TURRET_PATH_MAX_TARGETS]。
    float totalDegrees = 0;  // 将 0 写入或初始化 totalDegrees（总计Degrees）。
    size_t count = 0;  // 将 0 写入或初始化 count（计数）。
};  // 结束此类型定义。

/**
 * @brief 检查转台规划输入是否为有限浮点数。
 */
constexpr bool TurretPathFinite(float value) {  // 定义函数或成员 的入口 TurretPathFinite（转台路径Finite），参数由此处的签名确定。
    return value == value && value >= -FLT_MAX && value <= FLT_MAX;  // 结束当前函数并返回 value == value && value >= -FLT_MAX && value <= FLT_MAX。
}  // 结束此函数或控制块。
/**
 * @brief 计算两个多圈绝对角之间的直接角度距离(deg)，不做360度环绕。
 */
constexpr float TurretPathDistance(float from, float to) {  // 定义函数或成员 的入口 TurretPathDistance（转台路径距离），参数由此处的签名确定。
    return from > to ? from - to : to - from;  // 结束当前函数并返回 from > to ? from - to : to - from。
}  // 结束此函数或控制块。

// 起点和所有目标使用同一个多圈绝对坐标，保留调用方给出的角度及圈数。
// 仅检查现有线缆范围并累计直接行程，不选取±360度等效角。
// 输出目标可能等于-1；执行层不能将其当作跳过值。
/**
 * @brief 保留整组绝对目标及固定取放顺序，计算累计转角；不检查空间障碍。
 * @param currentAngle 转盘实测多圈起始角(deg)。
 * @param logicalAngles 工位的带符号多圈绝对目标角数组。
 * @param count 目标个数，最多6个。
 * @param minimumAngle 允许的最小多圈角(deg)。
 * @param maximumAngle 允许的最大多圈角(deg)。
 * @param output 成功时输出原始目标及累计角度，失败保留原输出。
 */
// 输入是当前实际角度、目标逻辑角序列和允许的多圈角度边界。
// 输出按目标顺序给出可下发的多圈角，供后续步骤逐个使用。
inline bool PlanTurretPath(float currentAngle, const float *logicalAngles, size_t count, float minimumAngle, float maximumAngle, TurretPathPlan &output) {  // 定义函数或成员 的入口 PlanTurretPath（Plan转台路径），参数由此处的签名确定。
    if (!logicalAngles || count == 0 || count > TURRET_PATH_MAX_TARGETS  // 当 !logicalAngles || count == 0 || count > TURRET_PATH_MA… 成立时进入此分支，否则继续后续流程。
            || !TurretPathFinite(currentAngle) || !TurretPathFinite(minimumAngle)  // 继续上一行的组合条件，本行提供 || !TurretPathFinite(currentAngle) || !TurretPathFinite(minimumAngle)。
            || !TurretPathFinite(maximumAngle) || minimumAngle >= maximumAngle  // 继续上一行的组合条件，本行提供 || !TurretPathFinite(maximumAngle) || minimumAngle >= maximumAngle。
            || currentAngle < minimumAngle || currentAngle > maximumAngle) return false;  // 继续上一行的组合条件，本行提供 || currentAngle < minimumAngle || currentAngle > maximumAngle) return false;。

    TurretPathPlan result;  // 声明 result（结果），类型和可用范围由所在声明决定。
    result.count = count;  // 将 count 写入或初始化 result.count。
    float previous = currentAngle;  // 将 currentAngle 写入或初始化 previous（上一）。
    for (size_t i = 0; i < count; ++i) {  // 按 size_t i = 0; i < count; ++i 组织循环的初值、继续条件和步进。
        const float target = logicalAngles[i];  // 将 logicalAngles[i] 写入或初始化 target（目标）。
        if (!TurretPathFinite(target) || target < minimumAngle || target > maximumAngle) return false;  // 若 !TurretPathFinite(target) || target < minimumAngle || …，立即返回 false。
        result.anglesDeg[i] = target;  // 将 target 写入或初始化 result.anglesDeg[i]。
        result.totalDegrees += TurretPathDistance(previous, target);  // 把 TurretPathDistance(previous, target) 累加到 result.totalDegrees。
        previous = target;  // 将 target 写入或初始化 previous（上一）。
    }  // 结束此循环体。
    output = result;  // 将 result 写入或初始化 output（输出）。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。
