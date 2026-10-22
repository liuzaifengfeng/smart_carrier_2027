#pragma once
#include "motion_types.h"

/**
 * @brief 创建统一运动任务及作业队列；后续底盘、机械臂和视觉闭环由该任务推进。
 * @return true 初始化成功，false 队列或任务创建失败。
 */
bool MotionExecutorInit();
/**
 * @brief 按值复制动作请求到队列，不等待动作执行完成。工作任务按 kind 分发到 scanTick、navigationTick 或机械臂序列。
 * @param request 动作类型、目标位置及本轮任务数据。
 * @param launch true 立即允许执行；false 等串口回复 ACK 后调用 LaunchMotion 放行。
 * @return 非零作业编号；0 表示未提交。提交成功不等于动作完成。
 */
uint32_t SubmitMotion(const MotionRequest &request,bool launch=true);
/**
 * @brief 放行已入队的指定作业，并唤醒运动任务；用于保证串口 ACK 先于动作执行。
 * @param id SubmitMotion 返回的作业编号。
 */
void LaunchMotion(uint32_t id);
/**
 * @brief 读取指定作业的状态快照，可查询当前作业及最近保留的终态。
 * @param id 提交动作时取得的作业编号。
 * @return 含执行状态、阶段、错误原因及底盘/机械臂完成标志的快照。
 */
MotionStatus GetMotionStatus(uint32_t id);
/**
 * @brief 判断是否存在排队中或执行中的运动作业。
 * @return true 表示执行器忙碌。
 */
bool MotionExecutorActive();
/**
 * @brief 取消当前作业代次并请求工作任务停车；旧作业不会在取消后继续执行。
 */
void CancelMotion();
/**
 * @brief 设置 OTA 占用状态；进入 OTA 时取消运动作业。
 * @param active true 进入 OTA，false 退出 OTA。
 */
void MotionExecutorSetOta(bool active);
/**
 * @brief 通知运动任务立即检查待执行作业、取消请求或新视觉反馈。
 */
void WakeMotionExecutor();
/**
 * @brief 读取当前运动作业编号，供运动任务内部输出分项日志。
 */
uint32_t CurrentMotionId(); // 仅供运动任务内部的分项日志使用。
// 同步接口保留给旧调试代码；正式状态机和串口任务不调用此入口。
/**
 * @brief 阻塞当前调用任务，轮询指定作业直至终态；只供旧调试接口使用。
 * @param id 待等待的作业编号。
 * @return true 作业 DONE，false 编号无效或作业失败、取消。
 */
bool WaitMotion(uint32_t id);
// 视觉任务只投递反馈；闭环发令由同一运动任务完成。
/**
 * @brief 处理最新视觉反馈并推进一次底盘对齐闭环；由统一运动任务调用。
 */
void TickVisualAlignment();

extern float MOTION_LOOK_LEAD_MM,MOTION_LOOK2_LEAD_MM;
extern uint32_t ARM_FEEDBACK_ENABLED,ARM_GRIP_HOLD_MS,ARM_RELEASE_HOLD_MS;
