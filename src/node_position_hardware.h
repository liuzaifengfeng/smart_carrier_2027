// 公开节点路线、扫码运动及电机反馈接口。
// 使用圆盘返回相关函数时，需要保留进入圆盘区时记录的编码器与航向基准。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include "node_position_control.h"  // 引入 node_position_control.h 中的声明和定义。
#include "scan_motion_control.h"  // 引入 scan_motion_control.h 中的声明和定义。
/**
 * @brief 同步运行连续扫码底盘行程；成功返回 nullptr，结束行程(mm)写入 forwardTravel。
 */
const char *RunContinuousScan(const ScanMotionSettings &settings,bool (*received)(),float &forwardTravel);  // 声明接口 RunContinuousScan（RunContinuous扫码），参数由此处的签名确定。
/**
 * @brief 回读四个底盘电机的编码器位置，数组顺序为电机1到4。
 * @return true 四轮读取成功；位置单位为65536/圈。
 */
bool ReadChassisEncoderPositions(int64_t (&positions)[4]);  // 声明接口 ReadChassisEncoderPositions（ReadChassis编码器Positions），参数由此处的签名确定。
const char *RecordDiscEncoderPositions(int64_t (&positions)[4],bool (*cancelled)());  // 声明接口 RecordDiscEncoderPositions（Record圆盘编码器Positions），参数由此处的签名确定。
const char *ReadDiscYOffset(const int64_t (&before)[4],float beforeHeading,float &yOffset,bool (*cancelled)());  // 声明接口 ReadDiscYOffset（Read圆盘纵向偏移），参数由此处的签名确定。
const char *ReadDiscReturnMotorFeedback(uint8_t id,node_position::Motor &state,bool (*cancelled)());  // 声明接口 ReadDiscReturnMotorFeedback（Read圆盘ReturnMotor反馈），参数由此处的签名确定。
// 按视觉校准前后的编码器差值，只补偿地图 Y，返回用于确认到位的四轮目标。
const char *BeginDiscYReturn(const int64_t (&before)[4],float beforeHeading,uint16_t maximumRpm,  // 定义函数或成员 BeginDiscYReturn（Begin圆盘纵向Return），参数由此处的签名确定。
    int64_t (&targets)[4],bool (*cancelled)());  // 调用 int64_t，传入 &targets)[4],bool (*cancelled)()。
// 调用者持有底盘运动锁；不会用估计时间替代驱动器反馈宣布成功。
/**
 * @brief 同步运行节点位置路径直至结束；调用者持有底盘运动锁。
 * @return nullptr 成功，否则为错误原因；result 为编码器/航向推算结果。
 */
const char *RunPositionNodeRoute(const continuous::Route &route,const node_position::Settings &settings,  // 定义函数或成员 RunPositionNodeRoute（Run位置节点路线），参数由此处的签名确定。
    uint32_t referenceVersion,bool (*cancelled)(),node_position::Result &result);  // 声明接口 bool，参数由此处的签名确定。
/**
 * @brief 绑定路径、启动配置、航向参考代次及取消回调，初始化节点路径 Runner；不等待行程。
 */
void BeginPositionNodeRoute(const continuous::Route &route,const node_position::Settings &settings,  // 定义函数或成员 BeginPositionNodeRoute（Begin位置节点路线），参数由此处的签名确定。
    uint32_t referenceVersion,bool (*cancelled)(),node_position::Result &result);  // 声明接口 bool，参数由此处的签名确定。
/**
 * @brief 推进节点位置路径一次，由统一运动任务周期调用。
 */
void TickPositionNodeRoute();  // 声明接口 TickPositionNodeRoute（轮询位置节点路线），参数由此处的签名确定。
/**
 * @brief 取得正式节点路径 Runner，供执行器查询进度和观察姿态提前触发阶段。
 */
node_position::Runner &PositionNodeRunner();  // 声明接口 PositionNodeRunner（位置节点Runner），参数由此处的签名确定。
/**
 * @brief 读取指定电机的到位标志、编码器位置和转速。
 * @param id 电机地址。
 * @param state 输出反馈，位置65536单位/圈，转速RPM。
 * @return nullptr 成功，否则为通信错误原因。
 */
const char *ReadMotorFeedback(uint8_t id,node_position::Motor &state);  // 声明接口 ReadMotorFeedback（ReadMotor反馈），参数由此处的签名确定。
// 单独读取/清零多圈位置，沿用电机总线事务锁和应答解析。
const char *ReadMotorEncoderPosition(uint8_t id,int64_t &position);  // 声明接口 ReadMotorEncoderPosition（ReadMotor编码器位置），参数由此处的签名确定。
// 一条0x43应答中同时取得单圈编码器和累计位置。
const char *ReadMotorEncoderSnapshot(uint8_t id,uint16_t &encoder,int64_t &position,int16_t &rpm,uint8_t &flags);  // 声明接口 ReadMotorEncoderSnapshot（ReadMotor编码器Snapshot），参数由此处的签名确定。
const char *ReadMotorPositionPid(uint8_t id,uint32_t (&values)[3]);  // 声明接口 ReadMotorPositionPid（ReadMotor位置PID），参数由此处的签名确定。
const char *WriteMotorPositionPid(uint8_t id,const uint32_t (&values)[3],bool save=false);  // 声明接口 WriteMotorPositionPid（WriteMotor位置PID），参数由此处的签名确定。
const char *SetMotorPositionPidVerified(uint8_t id,const uint32_t (&values)[3],bool save);  // 声明接口 SetMotorPositionPidVerified（SetMotor位置PIDVerified），参数由此处的签名确定。
const char *ReadMotorDriveParams(uint8_t id,uint16_t (&values)[7]);  // 声明接口 ReadMotorDriveParams，参数由此处的签名确定。
// 成功读回后values更新为实际值，包括驱动器拒用/限幅导致的不一致。
const char *SetMotorDriveParamsVerified(uint8_t id,uint16_t (&values)[7],bool save);  // 声明接口 SetMotorDriveParamsVerified，参数由此处的签名确定。
const char *ConfigureEmmMotor(uint8_t id,uint8_t function,uint8_t value,bool save);  // 声明接口 ConfigureEmmMotor，参数由此处的签名确定。
const char *ZeroMotorEncoderPosition(uint8_t id);  // 声明接口 ZeroMotorEncoderPosition（零点Motor编码器位置），参数由此处的签名确定。
/**
 * @brief 绑定扫码硬件并启动 ScanMotionRunner；received 回调由 pollTaskCode 提供。
 */
void BeginContinuousScan(const ScanMotionSettings &settings,bool (*received)());  // 声明接口 BeginContinuousScan（BeginContinuous扫码），参数由此处的签名确定。
/**
 * @brief 调用扫码 Runner 的 tick，推进一次接近/搜索行程。
 */
void TickContinuousScan();  // 声明接口 TickContinuousScan（轮询Continuous扫码），参数由此处的签名确定。
/**
 * @brief 取得扫码 Runner，供执行器查询结束、错误及编码器行程。
 */
ScanMotionRunner &ContinuousScanRunner();  // 声明接口 ContinuousScanRunner（Continuous扫码Runner），参数由此处的签名确定。
