// 扫码动作参数及运行状态。
// 设置保存接近、搜索及速度参数，运行器与 IO 和扫码回调配合推进。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include "node_position_control.h"  // 引入 node_position_control.h 中的声明和定义。

constexpr uint8_t SCAN_ACCELERATION = 170;  // 将 170 写入或初始化 SCAN_ACCELERATION（扫码加速度）。

struct ScanMotionSettings {  // 定义 ScanMotionSettings（扫码Motion设置） 的类型成员或作用域。
    float approachMm,searchMm,forwardPpm;  // 声明 approachMm（approach毫米），类型和可用范围由所在声明决定。
    uint16_t approachRpm,searchRpm;  // 声明 approachRpm（approach转速），类型和可用范围由所在声明决定。
};  // 结束此类型定义。
// 一次连续行程：接近->降速扫码。编码器用于行程推进，不增加故障门槛。
/**
 * @brief 同步执行接近与降速扫码连续行程，途中收到任务码立即停车。
 * @param settings 接近/搜索距离(mm)、脉冲每毫米及两段轮速(RPM)。
 * @param io 底盘读写接口。
 * @param received 查询是否已收到任务码的回调。
 * @param forwardTravel 输出编码器记录的有符号前后行程(mm)。
 * @return nullptr 完成，否则返回错误原因。
 */
const char *RunScanMotion(const ScanMotionSettings &settings,node_position::IO &io, bool (*received)(),float &forwardTravel);  // 声明接口 RunScanMotion（Run扫码Motion），参数由此处的签名确定。
class ScanMotionRunner {  // 定义 ScanMotionRunner（扫码MotionRunner） 的类型成员或作用域。
    ScanMotionSettings config{};  // 声明 config（配置），类型和可用范围由所在声明决定。
    node_position::IO *io=nullptr;  // 将 nullptr 写入或初始化 io（设备接口）。
    bool (*received)()=nullptr;  // 调用 bool，传入 *received)()=nullptr。
    int64_t origin[4]={},positions[4]={};  // 将 {},positions[4]={} 写入或初始化 origin[4]。
    bool running=false,initialized=false;  // 将 false,initialized=false 写入或初始化 running。
    const char *error=nullptr;  // 将 nullptr 写入或初始化 error（错误）。
    float speed=0,travel=0;  // 将 0,travel=0 写入或初始化 speed（速度）。
    uint32_t previous=0;  // 将 0 写入或初始化 previous（上一）。
    /**
     * @brief 停止四轮，读取结束编码器位置并保存本次实际前后行程。
     */
    void finish();  // 声明接口 finish，参数由此处的签名确定。
public:  // 此后的成员采用 public 访问权限，直到下一个权限标签。
    /**
     * @brief 初始化连续扫码行程，绑定底盘接口和任务码查询回调；运动由 tick 推进。
     */
    void begin(const ScanMotionSettings &settings,node_position::IO &hardware,bool (*poll)());  // 声明接口 begin，参数由此处的签名确定。
    /**
     * @brief 推进一次连续扫码：读编码器、检查任务码、计算接近/搜索速度并下发四轮轮速。
     */
    void tick();  // 声明接口 tick（轮询），参数由此处的签名确定。
    /**
     * @brief 判断连续扫码行程是否仍在运行。
     */
    bool active() const{return running;}  // 定义 active（活跃），调用时返回 running。
    /**
     * @brief 读取连续扫码错误原因；nullptr 表示没有记录错误。
     */
    const char *failure() const{return error;}  // 定义函数或成员 failure，参数由此处的签名确定。
    /**
     * @brief 读取结束时编码器计算的有符号前后行程(mm)，用于更新估算位姿。
     */
    float forwardTravel() const{return travel;}  // 定义 forwardTravel（前向Travel），调用时返回 travel。
};  // 结束此类型定义。
