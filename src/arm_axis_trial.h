#pragma once
#include <stdint.h>

/**
 * @brief 启动5号升降或6号伸缩的单段绝对位置试验，成功返回nullptr。
 * @param positionMm 目标位置(mm)，使用当前上电零点及HEIGHT_PULSE/LENGTH_PULSE。
 * @param rpm 电机速度(1~65535 RPM)，不经过正式动作组的速度倍率。
 * @param rampMs 加减速参考时间(ms)，按EMM整数档位就近换算；0表示不加减速。
 * @param periodMs 请求采样周期(ms)，实际间隔包含总线通信耗时。
 * @param durationMs 从发令起的完整观察时长(ms)，结束不代表已到位或已停车。
 * @note 仅由统一运动任务调用；Debug/空闲检查及物理停车由调用方负责。
 */
const char *ArmAxisTrialStart(uint8_t id,float positionMm,uint16_t rpm,uint16_t rampMs,
    uint32_t periodMs,uint32_t durationMs);
/** @brief 推进一次采样；只在单次总线事务期间持锁，不阻塞等待整个行程。 */
void ArmAxisTrialPoll();
bool ArmAxisTrialActive();
/** @brief 结束记录；物理停车仍由执行器的统一停止入口处理。 */
void ArmAxisTrialCancel();
/** @brief 返回下次采样或观察结束前的等待毫秒数。 */
uint32_t ArmAxisTrialNextDelay();
/** @brief 返回当前/最终错误；正常观察结束/取消为nullptr，瞬时失败允许后续采样恢复。 */
const char *ArmAxisTrialError();
