// 扫码接近和搜索动作的轮询控制。
// 以运动反馈和收到任务码的回调决定当前阶段何时结束，并记录前向位移。

#include "scan_motion_control.h"  // 引入 scan_motion_control.h 中的声明和定义。

/**
 * @brief 初始化连续扫码行程，绑定底盘接口和任务码查询回调；运动由 tick 推进。
 */
void ScanMotionRunner::begin(const ScanMotionSettings &settings,node_position::IO &hardware,bool (*poll)()) {  // 定义函数或成员 的入口 ScanMotionRunner::begin（扫码MotionRunnerbegin），参数由此处的签名确定。
    *this=ScanMotionRunner{};config=settings;io=&hardware;received=poll;running=true;  // 将 ScanMotionRunner{};config=settings;io=&hardware;receiv… 写入或初始化 this。
}  // 结束此函数或控制块。
/**
 * @brief 停止四轮，读取结束编码器位置并保存本次实际前后行程。
 */
void ScanMotionRunner::finish() {  // 定义函数或成员 的入口 ScanMotionRunner::finish（扫码MotionRunnerfinish），参数由此处的签名确定。
    io->halt();double units=0;  // 命令设备停止。然后将 0 写入或初始化 units。
    for(uint8_t id=1;id<=4;++id) {  // 按 uint8_t id=1;id<=4;++id 组织循环的初值、继续条件和步进。
        const char *failure=io->readPosition(id,positions[id-1]);  // 将 io->readPosition(id,positions[id-1]) 写入或初始化 failure。
        if(failure){if(!error)error=failure;break;}  // 当 failure 成立时进入此分支，否则继续后续流程。
        units+=fabs(static_cast<double>(positions[id-1]-origin[id-1]))/4;  // 把 fabs(static_cast<double>(positions[id-1]-origin[id-1])… 累加到 units。
    }  // 结束此循环体。
    travel=(config.approachMm>=0?1:-1)*units*3200/65536/config.forwardPpm;running=false;  // 将 (config.approachMm>=0?1:-1)*units*3200/65536/config.fo… 写入或初始化 travel。然后将 false 写入或初始化 running。
}  // 结束此函数或控制块。
/**
 * @brief 推进一次连续扫码：读编码器、检查任务码、计算接近/搜索速度并下发四轮轮速。
 */
void ScanMotionRunner::tick() {  // 定义函数或成员 的入口 ScanMotionRunner::tick（扫码MotionRunner轮询），参数由此处的签名确定。
    if(!running)return;  // 当 !running 成立时进入此分支，否则继续后续流程。
    if(io->cancelled()){error="STOPPED";finish();return;}  // 当 io->cancelled() 成立时进入此分支，否则继续后续流程。
    if(!initialized) {  // 当 !initialized 成立时进入此分支，否则继续后续流程。
        for(uint8_t id=1;id<=4;++id){error=io->readPosition(id,origin[id-1]);if(error){running=false;return;}}  // 按 uint8_t id=1;id<=4;++id 组织循环的初值、继续条件和步进。
        previous=io->now();initialized=true;return;  // 将 io->now() 写入或初始化 previous（上一）。然后将 true 写入或初始化 initialized。然后结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    double units=0;  // 将 0 写入或初始化 units。
    for(uint8_t id=1;id<=4;++id) {  // 按 uint8_t id=1;id<=4;++id 组织循环的初值、继续条件和步进。
        error=io->readPosition(id,positions[id-1]);if(error){finish();return;}  // 将 io->readPosition(id,positions[id-1]);if(error){finish(… 写入或初始化 error（错误）。
        units+=fabs(static_cast<double>(positions[id-1]-origin[id-1]))/4;  // 把 fabs(static_cast<double>(positions[id-1]-origin[id-1])… 累加到 units。
    }  // 结束此循环体。
    const float approach=fabsf(config.approachMm),total=approach+fabsf(config.searchMm);  // 将 fabsf(config.approachMm),total=approach+fabsf(config.s… 写入或初始化 approach。
    const float distance=units*3200/65536/config.forwardPpm;  // 将 units*3200/65536/config.forwardPpm 写入或初始化 distance（距离）。
    if(received()||distance>=total){finish();return;}  // 当 received()||distance>=total 成立时进入此分支，否则继续后续流程。
    const float rpmPerSecond=20000.0f/(256-SCAN_ACCELERATION),rpmSecondsPerMm=config.forwardPpm*60/3200;  // 将 20000.0f/(256-SCAN_ACCELERATION),rpmSecondsPerMm=confi… 写入或初始化 rpmPerSecond（转速PerSecond）。
    const float entryCap=distance<approach?sqrtf(config.searchRpm*float(config.searchRpm)+2*rpmPerSecond*rpmSecondsPerMm*(approach-distance)):config.searchRpm;  // 将 distance<approach?sqrtf(config.searchRpm*float(config.… 写入或初始化 entryCap。
    const float endCap=sqrtf(2*rpmPerSecond*rpmSecondsPerMm*(total-distance));  // 将 sqrtf(2*rpmPerSecond*rpmSecondsPerMm*(total-distance)) 写入或初始化 endCap（终点Cap）。
    const float cap=fminf(config.approachRpm,fminf(entryCap,endCap));  // 将 fminf(config.approachRpm,fminf(entryCap,endCap)) 写入或初始化 cap。
    const uint32_t now=io->now();const float step=rpmPerSecond*(now-previous)*.001f;previous=now;  // 将 io->now() 写入或初始化 now。然后将 rpmPerSecond*(now-previous)*.001f 写入或初始化 step。然后将 now 写入或初始化 previous（上一）。
    speed+=continuous::clamp(cap-speed,-step,step);  // 把 continuous::clamp(cap-speed,-step,step) 累加到 speed（速度）。
    const int16_t rpm=static_cast<int16_t>(fmaxf(1,lroundf(speed)))*(config.approachMm>=0?1:-1);  // 将 static_cast<int16_t>(fmaxf(1,lroundf(speed)))*(config.… 写入或初始化 rpm（转速）。
    continuous::WheelCommand command;command.rpm[0]=command.rpm[2]=rpm;command.rpm[1]=command.rpm[3]=-rpm;  // 声明 command，类型和可用范围由所在声明决定。然后将 command.rpm[2]=rpm 写入或初始化 command.rpm[0]。然后将 command.rpm[3]=-rpm 写入或初始化 command.rpm[1]。
    error=io->velocity(command);if(error)finish();  // 将 io->velocity(command) 写入或初始化 error（错误）。然后当 error 成立时进入此分支，否则继续后续流程。
}  // 结束此函数或控制块。
/**
 * @brief 同步执行接近与降速扫码连续行程，途中收到任务码立即停车。
 * @param settings 接近/搜索距离(mm)、脉冲每毫米及两段轮速(RPM)。
 * @param io 底盘读写接口。
 * @param received 查询是否已收到任务码的回调。
 * @param forwardTravel 输出编码器记录的有符号前后行程(mm)。
 * @return nullptr 完成，否则返回错误原因。
 */
const char *RunScanMotion(const ScanMotionSettings &settings,node_position::IO &io,bool (*received)(),float &travel) {  // 定义函数或成员 的入口 RunScanMotion（Run扫码Motion），参数由此处的签名确定。
    ScanMotionRunner runner;runner.begin(settings,io,received);  // 声明 runner，类型和可用范围由所在声明决定。然后调用 runner.begin，传入 settings,io,received。
    while(runner.active())runner.tick();  // 当 runner.active() 成立时继续循环。
    travel=runner.forwardTravel();return runner.failure();  // 将 runner.forwardTravel() 写入或初始化 travel。然后结束当前函数并返回 runner.failure()。
}  // 结束此函数或控制块。
