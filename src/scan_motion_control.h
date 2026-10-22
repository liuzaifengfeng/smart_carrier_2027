#pragma once
#include "node_position_control.h"

struct ScanMotionSettings {
    float approachMm,searchMm,forwardPpm;
    uint16_t approachRpm,searchRpm;
};
// 一次连续行程：接近->降速扫码。编码器用于行程推进，不增加故障门槛。
/**
 * @brief 同步执行接近与降速扫码连续行程，途中收到任务码立即停车。
 * @param settings 接近/搜索距离(mm)、脉冲每毫米及两段轮速(RPM)。
 * @param io 底盘读写接口。
 * @param received 查询是否已收到任务码的回调。
 * @param forwardTravel 输出编码器记录的有符号前后行程(mm)。
 * @return nullptr 完成，否则返回错误原因。
 */
const char *RunScanMotion(const ScanMotionSettings &settings,node_position::IO &io, bool (*received)(),float &forwardTravel);
class ScanMotionRunner {
    ScanMotionSettings config{};
    node_position::IO *io=nullptr;
    bool (*received)()=nullptr;
    int64_t origin[4]={},positions[4]={};
    bool running=false,initialized=false;
    const char *error=nullptr;
    float speed=0,travel=0;
    uint32_t previous=0;
    /**
     * @brief 停止四轮，读取结束编码器位置并保存本次实际前后行程。
     */
    void finish();
public:
    /**
     * @brief 初始化连续扫码行程，绑定底盘接口和任务码查询回调；运动由 tick 推进。
     */
    void begin(const ScanMotionSettings &settings,node_position::IO &hardware,bool (*poll)());
    /**
     * @brief 推进一次连续扫码：读编码器、检查任务码、计算接近/搜索速度并下发四轮轮速。
     */
    void tick();
    /**
     * @brief 判断连续扫码行程是否仍在运行。
     */
    bool active() const{return running;}
    /**
     * @brief 读取连续扫码错误原因；nullptr 表示没有记录错误。
     */
    const char *failure() const{return error;}
    /**
     * @brief 读取结束时编码器计算的有符号前后行程(mm)，用于更新估算位姿。
     */
    float forwardTravel() const{return travel;}
};
