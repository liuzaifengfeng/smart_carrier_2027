#pragma once
#include "node_position_control.h"
#include "scan_motion_control.h"
/**
 * @brief 同步运行连续扫码底盘行程；成功返回 nullptr，结束行程(mm)写入 forwardTravel。
 */
const char *RunContinuousScan(const ScanMotionSettings &settings,bool (*received)(),float &forwardTravel);
/**
 * @brief 回读四个底盘电机的编码器位置，数组顺序为电机1到4。
 * @return true 四轮读取成功；位置单位为65536/圈。
 */
bool ReadChassisEncoderPositions(int64_t (&positions)[4]);
// 调用者持有底盘运动锁；不会用估计时间替代驱动器反馈宣布成功。
/**
 * @brief 同步运行节点位置路径直至结束；调用者持有底盘运动锁。
 * @return nullptr 成功，否则为错误原因；result 为编码器/航向推算结果。
 */
const char *RunPositionNodeRoute(const continuous::Route &route,const node_position::Settings &settings,
    uint32_t referenceVersion,bool (*cancelled)(),node_position::Result &result);
/**
 * @brief 绑定路径、启动配置、航向参考代次及取消回调，初始化节点路径 Runner；不等待行程。
 */
void BeginPositionNodeRoute(const continuous::Route &route,const node_position::Settings &settings,
    uint32_t referenceVersion,bool (*cancelled)(),node_position::Result &result);
/**
 * @brief 推进节点位置路径一次，由统一运动任务周期调用。
 */
void TickPositionNodeRoute();
/**
 * @brief 取得正式节点路径 Runner，供执行器查询进度和观察姿态提前触发阶段。
 */
node_position::Runner &PositionNodeRunner();
/**
 * @brief 读取指定电机的到位标志、编码器位置和转速。
 * @param id 电机地址。
 * @param state 输出反馈，位置65536单位/圈，转速RPM。
 * @return nullptr 成功，否则为通信错误原因。
 */
const char *ReadMotorFeedback(uint8_t id,node_position::Motor &state);
// 单独读取/清零多圈位置，沿用电机总线事务锁和应答解析。
const char *ReadMotorEncoderPosition(uint8_t id,int64_t &position);
// 一条0x43应答中同时取得单圈编码器和累计位置。
const char *ReadMotorEncoderSnapshot(uint8_t id,uint16_t &encoder,int64_t &position,int16_t &rpm,uint8_t &flags);
const char *ReadMotorPositionPid(uint8_t id,uint32_t (&values)[3]);
const char *WriteMotorPositionPid(uint8_t id,const uint32_t (&values)[3],bool save=false);
const char *SetMotorPositionPidVerified(uint8_t id,const uint32_t (&values)[3],bool save);
const char *ConfigureEmmMotor(uint8_t id,uint8_t function,uint8_t value,bool save);
const char *ZeroMotorEncoderPosition(uint8_t id);
/**
 * @brief 绑定扫码硬件并启动 ScanMotionRunner；received 回调由 pollTaskCode 提供。
 */
void BeginContinuousScan(const ScanMotionSettings &settings,bool (*received)());
/**
 * @brief 调用扫码 Runner 的 tick，推进一次接近/搜索行程。
 */
void TickContinuousScan();
/**
 * @brief 取得扫码 Runner，供执行器查询结束、错误及编码器行程。
 */
ScanMotionRunner &ContinuousScanRunner();
