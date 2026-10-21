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

constexpr bool TurretPathFinite(float value) {
    return value == value && value >= -FLT_MAX && value <= FLT_MAX;
}
constexpr float TurretPathDistance(float from, float to) {
    return from > to ? from - to : to - from;
}

// 起点和标定角度须在限位内；范围宽度小于720度，因此每个目标最多两个等效角。
// 限位内等效角思路参考 https://github.com/ros/angles ，本模块比较整组累计转角。
// 固定任务顺序，最小化总转角；不检测空间障碍物。失败时保留原有输出。
// 输出是真实多圈绝对目标，可能等于-1；执行层不能把它当跳过值或再次就近选角。
inline bool PlanTurretPath(float currentAngle, const float *logicalAngles, size_t count, float minimumAngle, float maximumAngle, TurretPathPlan &output) {
    if (!logicalAngles || count == 0 || count > TURRET_PATH_MAX_TARGETS
            || !TurretPathFinite(currentAngle) || !TurretPathFinite(minimumAngle)
            || !TurretPathFinite(maximumAngle) || minimumAngle >= maximumAngle
            || !(maximumAngle - minimumAngle < 720.0f)
            || currentAngle < minimumAngle || currentAngle > maximumAngle) return false;

    float candidates[TURRET_PATH_MAX_TARGETS][2] = {};
    uint8_t candidateCounts[TURRET_PATH_MAX_TARGETS] = {};
    float costs[TURRET_PATH_MAX_TARGETS][2] = {};
    uint8_t previous[TURRET_PATH_MAX_TARGETS][2] = {};
    for (size_t i = 0; i < count; ++i) {
        if (!TurretPathFinite(logicalAngles[i]) || logicalAngles[i] < minimumAngle
                || logicalAngles[i] > maximumAngle) return false;
        for (int turn = -1; turn <= 1; ++turn) {
            const float angle = logicalAngles[i] + 360.0f * turn;
            if (angle < minimumAngle || angle > maximumAngle) continue;
            const uint8_t n = candidateCounts[i];
            if (n != 0 && angle == candidates[i][n - 1]) continue;
            if (n == 2) return false;
            candidates[i][n] = angle;
            ++candidateCounts[i];
        }
        if (candidateCounts[i] == 0) return false;
    }

    // 每层代表一个取放目标，比较所有上一层分支，避免单段就近选角导致后续绕远。
    for (uint8_t c = 0; c < candidateCounts[0]; ++c)
        costs[0][c] = TurretPathDistance(currentAngle, candidates[0][c]);
    for (size_t i = 1; i < count; ++i) {
        for (uint8_t c = 0; c < candidateCounts[i]; ++c) {
            float best = FLT_MAX;
            for (uint8_t p = 0; p < candidateCounts[i - 1]; ++p) {
                const float cost = costs[i - 1][p]
                    + TurretPathDistance(candidates[i - 1][p], candidates[i][c]);
                if (cost < best) {
                    best = cost;
                    previous[i][c] = p;
                }
            }
            costs[i][c] = best;
        }
    }

    // 等代价时保留先枚举的较低角度分支，保证同一输入的结果稳定。
    uint8_t state = 0;
    for (uint8_t c = 1; c < candidateCounts[count - 1]; ++c)
        if (costs[count - 1][c] < costs[count - 1][state]) state = c;
    TurretPathPlan result;
    result.count = count;
    result.totalDegrees = costs[count - 1][state];
    for (size_t i = count; i > 0; --i) {
        result.anglesDeg[i - 1] = candidates[i - 1][state];
        state = previous[i - 1][state];
    }
    output = result;
    return true;
}
