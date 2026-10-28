#pragma once

#include <float.h>
#include <stddef.h>
#include <stdint.h>

constexpr size_t TURRET_PATH_MAX_TARGETS = 6; // 三件物料，各有取料、放料两个目标。

struct TurretPathPlan {
    float anglesDeg[TURRET_PATH_MAX_TARGETS] = {};
    float totalDegrees = 0;
    size_t count = 0;
};

/**
 * @brief 检查转台规划输入是否为有限浮点数。
 */
constexpr bool TurretPathFinite(float value) {
    return value == value && value >= -FLT_MAX && value <= FLT_MAX;
}
/**
 * @brief 计算两个多圈绝对角之间的直接角度距离(deg)，不做360度环绕。
 */
constexpr float TurretPathDistance(float from, float to) {
    return from > to ? from - to : to - from;
}

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
inline bool PlanTurretPath(float currentAngle, const float *logicalAngles, size_t count, float minimumAngle, float maximumAngle, TurretPathPlan &output) {
    if (!logicalAngles || count == 0 || count > TURRET_PATH_MAX_TARGETS
            || !TurretPathFinite(currentAngle) || !TurretPathFinite(minimumAngle)
            || !TurretPathFinite(maximumAngle) || minimumAngle >= maximumAngle
            || currentAngle < minimumAngle || currentAngle > maximumAngle) return false;

    TurretPathPlan result;
    result.count = count;
    float previous = currentAngle;
    for (size_t i = 0; i < count; ++i) {
        const float target = logicalAngles[i];
        if (!TurretPathFinite(target) || target < minimumAngle || target > maximumAngle) return false;
        result.anglesDeg[i] = target;
        result.totalDegrees += TurretPathDistance(previous, target);
        previous = target;
    }
    output = result;
    return true;
}
