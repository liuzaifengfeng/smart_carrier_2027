// 单轴试验的启动与轮询接口。
// 启动和获取终态分开，调用方可以用下一次采样延时安排任务唤醒。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include <stdint.h>  // 引入 stdint.h 中的声明和定义。

/**
 * @brief 启动5号升降或6号伸缩的单段绝对位置试验，成功返回nullptr。
 * @param positionMm 目标位置(mm)，使用当前上电零点及HEIGHT_PULSE/LENGTH_PULSE。
 * @param rpm 电机速度(1~65535 RPM)，不经过正式动作组的速度倍率。
 * @param rampMs 加减速参考时间(ms)，按EMM整数档位就近换算；0表示不加减速。
 * @param periodMs 请求采样周期(ms)，实际间隔包含总线通信耗时。
 * @param durationMs 从发令起的完整观察时长(ms)，结束不代表已到位或已停车。
 * @note 仅由统一运动任务调用；Debug/空闲检查及物理停车由调用方负责。
 */
const char *ArmAxisTrialStart(uint8_t id,float positionMm,uint16_t rpm,uint16_t rampMs,  // 定义函数或成员 ArmAxisTrialStart（机械臂AxisTrial起点），参数由此处的签名确定。
    uint32_t periodMs,uint32_t durationMs);  // 声明 periodMs（period毫秒），类型和可用范围由所在声明决定。
/** @brief 推进一次采样；只在单次总线事务期间持锁，不阻塞等待整个行程。 */
void ArmAxisTrialPoll();  // 声明接口 ArmAxisTrialPoll（机械臂AxisTrialPoll），参数由此处的签名确定。
bool ArmAxisTrialActive();  // 声明接口 ArmAxisTrialActive（机械臂AxisTrial活跃），参数由此处的签名确定。
/** @brief 结束记录；物理停车仍由执行器的统一停止入口处理。 */
void ArmAxisTrialCancel();  // 声明接口 ArmAxisTrialCancel（机械臂AxisTrialCancel），参数由此处的签名确定。
/** @brief 返回下次采样或观察结束前的等待毫秒数。 */
uint32_t ArmAxisTrialNextDelay();  // 声明接口 ArmAxisTrialNextDelay（机械臂AxisTrial下一延时），参数由此处的签名确定。
/** @brief 返回当前/最终错误；正常观察结束/取消为nullptr，瞬时失败允许后续采样恢复。 */
const char *ArmAxisTrialError();  // 声明接口 ArmAxisTrialError（机械臂AxisTrial错误），参数由此处的签名确定。
