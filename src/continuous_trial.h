// 公开路径试验和连续运动的分阶段接口。
// 节点上传、准备、启动和查询各有入口，调用端用试验编号关联操作。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include "continuous_route.h"  // 引入 continuous_route.h 中的声明和定义。

/**
 * @brief 初始化连续路径试跑模块，准备其状态和同步资源。
 */
bool ContinuousTrialInit();  // 声明接口 ContinuousTrialInit，参数由此处的签名确定。
/**
 * @brief 判断独立连续路径试跑是否正在运行或停止中。
 */
bool ContinuousTrialActive();  // 声明接口 ContinuousTrialActive（ContinuousTrial活跃），参数由此处的签名确定。
/**
 * @brief 建立指定编号的试跑配置，供后续写入节点及策略；成功返回 nullptr。
 */
const char *ContinuousTrialBegin(const continuous::Config &config);  // 声明接口 ContinuousTrialBegin，参数由此处的签名确定。
/**
 * @brief 设置试跑路径策略、外扩和几何版本；成功返回 nullptr。
 */
const char *ContinuousTrialPolicy(uint32_t id,uint8_t policy,float outset,float margin,uint32_t geometry);  // 声明接口 ContinuousTrialPolicy，参数由此处的签名确定。
/**
 * @brief 写入一个试跑节点及转弯半径，并核对上位机期望坐标；成功返回 nullptr。
 */
const char *ContinuousTrialNode(uint32_t id,uint8_t index,uint8_t node,float radius,float expectedX,float expectedY);  // 声明接口 ContinuousTrialNode（ContinuousTrial节点），参数由此处的签名确定。
// Prepare 之后即为 RUNNING，串口先回复 ACK，再 Launch 通知工作任务。
/**
 * @brief 完成试跑准备并登记RUNNING；实际启动等待ACK后的Launch通知。
 * @return nullptr 准备成功，否则为错误原因。
 */
const char *ContinuousTrialPrepare(uint32_t id);  // 声明接口 ContinuousTrialPrepare，参数由此处的签名确定。
/**
 * @brief 提交已准备的连续试跑作业；在串口ACK发出后调用。
 */
void ContinuousTrialLaunch();  // 声明接口 ContinuousTrialLaunch，参数由此处的签名确定。
/**
 * @brief 请求停止指定编号试跑，不在调用任务中直接操作电机；成功返回 nullptr。
 */
const char *ContinuousTrialStop(uint32_t id);  // 声明接口 ContinuousTrialStop（ContinuousTrial停止），参数由此处的签名确定。
/**
 * @brief 输出指定试跑状态并续约1500ms心跳；成功返回 nullptr。
 */
const char *ContinuousTrialStatus(uint32_t id); // 输出状态，同时续约 1500ms 心跳
// OTA 等外部取消源；不在调用方操作电机。
/**
 * @brief 向当前试跑发出取消请求，供OTA或其他外部停止源调用。
 */
void ContinuousTrialCancel();  // 声明接口 ContinuousTrialCancel，参数由此处的签名确定。
/**
 * @brief 设置试跑OTA状态，进入OTA时请求停止现有试跑。
 */
void ContinuousTrialSetOta(bool active);  // 声明接口 ContinuousTrialSetOta（ContinuousTrialSet固件更新），参数由此处的签名确定。
// 正式节点导航共用控制任务。异步入口供串口使用，避免阻塞STOP/查询接收。
// 契约：path 包含 count 个节点，2 <= count <= MAX_NODES，速度与配置已由入口校验。
/**
 * @brief 提交正式节点导航作业；async 控制异步返回或同步等待完成。
 * @param path 节点编号数组。
 * @param count 节点数。
 * @param speedMmS 路径平移速度(mm/s)。
 * @param finalHeading 终点地图航向(deg)。
 * @param async true 异步提交，false 等待作业结果。
 * @return true 提交或执行成功，具体含义由 async 决定。
 */
bool H30StartNodePath(const uint8_t *path,size_t count,float speedMmS,float finalHeading,bool async=true);  // 声明接口 H30StartNodePath（H30起点节点路径），参数由此处的签名确定。
/**
 * @brief 同步执行正式节点导航，参数为节点数组、节点数、平移速度及终点航向。
 */
bool H30RunNodePath(const uint8_t *path,size_t count,float speedMmS,float finalHeading);  // 声明接口 H30RunNodePath（H30Run节点路径），参数由此处的签名确定。

// 以下入口仅由统一运动任务调用，试跑与正式路径使用同一执行任务。
/**
 * @brief 在运动任务内构建正式路径并冻结本次速度/加速度配置；不等待行程完成。
 */
bool H30PrepareNodePath(const uint8_t *path,size_t count,float speedMmS,float finalHeading,bool async,  // 定义函数或成员 H30PrepareNodePath（H30Prepare节点路径），参数由此处的签名确定。
                       float startYOffset=0,float *plannedArrivalHeading=nullptr,bool preserveStartPosition=false);  // 将 0,float *plannedArrivalHeading=nullptr,bool preserveSt… 写入或初始化 startYOffset（起点纵向偏移）。
/**
 * @brief 启动已经准备好的节点导航或连续试跑执行过程，仅由运动任务调用。
 */
void ContinuousMotionBegin();  // 声明接口 ContinuousMotionBegin，参数由此处的签名确定。
/**
 * @brief 推进一次正式导航或试跑；实际节点运动由 node_position::Runner 执行。
 */
void ContinuousMotionTick();  // 声明接口 ContinuousMotionTick（ContinuousMotion轮询），参数由此处的签名确定。
/**
 * @brief 取消当前路径执行并结束其运动状态。
 */
void ContinuousMotionCancel();  // 声明接口 ContinuousMotionCancel，参数由此处的签名确定。
/**
 * @brief 判断正式导航或连续试跑执行过程是否仍在运行。
 */
bool ContinuousMotionActive();  // 声明接口 ContinuousMotionActive（ContinuousMotion活跃），参数由此处的签名确定。
/**
 * @brief 读取当前路径执行错误原因；nullptr 表示未记录错误。
 */
const char *ContinuousMotionFailure();  // 声明接口 ContinuousMotionFailure，参数由此处的签名确定。
/**
 * @brief 查询路径控制器下一次推进前所需等待毫秒数。
 */
uint32_t ContinuousMotionNextDelay();  // 声明接口 ContinuousMotionNextDelay（ContinuousMotion下一延时），参数由此处的签名确定。
