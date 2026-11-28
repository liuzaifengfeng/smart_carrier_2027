// 动作步骤及其轮询执行规则。
// 步骤数据保存要下发的目标和等待条件，轮询状态保存正在执行的步骤。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include <stdint.h>  // 引入 stdint.h 中的声明和定义。
#include "material_transfer.h"  // 引入 material_transfer.h 中的声明和定义。
#include <stddef.h>  // 引入 stddef.h 中的声明和定义。
#include <string.h>  // 引入 string.h 中的声明和定义。

namespace motion {  // 定义 motion 的类型成员或作用域。
inline bool retryableFeedback(const char *error) {  // 定义函数或成员 的入口 retryableFeedback（retryable反馈），参数由此处的签名确定。
    return error&&(!strcmp(error,"MOTOR_FEEDBACK_TIMEOUT")||!strcmp(error,"MOTOR_READ")||!strcmp(error,"SERVO_READ"));  // 结束当前函数并返回 error&&(!strcmp(error,"MOTOR_FEEDBACK_TIMEOUT")||!strc…。
}  // 结束此函数或控制块。
enum class StepKind : uint8_t { ARM, WAIT, TURRET_READY, POSITION, GOTO, COMMIT, RAW_SERVO, ENABLE, VELOCITY, TRAY_ROTATE, TRAY_START, TRAY_READY, RING_CORRECT };  // 定义 StepKind 的类型成员或作用域。
// 同一组字段由 kind 决定如何解释，例如 ARM 的 high/length 是高度和伸长量，POSITION 则用它们保存底盘 x/y 位移。
// 修改一种动作的字段前，先核对 MotionHardware::issue 的对应分支。
struct Step {  // 定义 Step 的类型成员或作用域。
    StepKind kind=StepKind::WAIT;  // 将 StepKind::WAIT 写入或初始化 kind。
    const char *name="WAIT";  // 将 "WAIT" 写入或初始化 name。
    float high=-1,length=-1,turret=-1,pawl=-1,speed=0;  // 将 -1,length=-1,turret=-1,pawl=-1,speed=0 写入或初始化 high。
    uint32_t waitMs=0,holdMs=0;  // 将 0,holdMs=0 写入或初始化 waitMs（wait毫秒）。
    uint16_t trayPowerMw=0;  // 将 0 写入或初始化 trayPowerMw（料盘功率Mw）。
    uint8_t cargo=0,color=0;  // 将 0,color=0 写入或初始化 cargo（货位）。
    uint8_t trayClearanceCargo=0; // 放料转向时检查取料盘的相邻小一号盘是否占用。
    uint8_t trayWork3LimitCargo=0; // 下放到工位 3 前，按取料盘号判断是否需要限制料盘角度。
    const char *materialEvent=nullptr;  // 将 nullptr 写入或初始化 materialEvent（物料Event）。
    uint8_t materialPosition=0;  // 将 0 写入或初始化 materialPosition（物料位置）。
    uint8_t placementPosition=0; // 非零：工位放料下降结束时回读角度和伸出长度并记录偏差。
    bool planned=false;  // 将 false 写入或初始化 planned。
    bool turretArrival=false; // 独立转向步骤由转盘电机实测多圈角度决定完成。
    bool lengthArrival=false; // 转向时同时检查伸缩轴到位。
    bool lengthArrivalByPosition=false; // LOOK4 仅按伸缩轴实测位置确认到位。
    bool cargoPlaceTurn=false; // 持料转向料盘使用独立的加速度配置。
    bool workPlaceLiftArrival=false; // 工位放料后抬升：实测高度误差小于10mm即继续。
    bool liftArrival=false; // 初始化抬升实测到位后，才允许后续转盘动作。
    bool timed=false; // 此步按waitMs计时，即使开启机械臂反馈模式也不改为等待到位。
    bool slowLower=false; // 第一层工位放料下降：升降转速减半。
    uint8_t ringVisionPosition=0; // 非零时在预放下降前上报工位号并开启相机。
    uint8_t turretAcceleration=TURRET_ACCELERATION_FAST; // 张大头原生加速度档位，由动作阶段指定。
    uint8_t lengthAcceleration=ARM_EXTENSION_ACCELERATION;  // 将 ARM_EXTENSION_ACCELERATION 写入或初始化 lengthAcceleration（长度加速度）。
    uint8_t slowLowerAcceleration=ARM_SLOW_LOWER_ACCELERATION;  // 将 ARM_SLOW_LOWER_ACCELERATION 写入或初始化 slowLowerAcceleration（slowLower加速度）。
    uint8_t positionAcceleration=POSITION_MOVE_DEFAULT_ACCELERATION;  // 将 POSITION_MOVE_DEFAULT_ACCELERATION 写入或初始化 positionAcceleration（位置加速度）。
    float trayArrivalToleranceDeg=CARGO_TRAY_ARRIVAL_TOLERANCE_DEG;  // 将 CARGO_TRAY_ARRIVAL_TOLERANCE_DEG 写入或初始化 trayArrivalToleranceDeg（料盘Arrival容差角度）。
};  // 结束此类型定义。
struct Program {  // 定义 Program 的类型成员或作用域。
    // 静态存储，运行中不申请堆内存。
    static constexpr size_t CAPACITY=64;  // 将 64 写入或初始化 CAPACITY。
    Step steps[CAPACITY];  // 声明 steps，类型和可用范围由所在声明决定。
    size_t count=0;  // 将 0 写入或初始化 count（计数）。
    /**
     * @brief 向动作程序末尾追加一步；调用方保证静态步骤容量足够。
     */
    void append(const Step &step){steps[count++]=step;}  // 将 step;} 写入或初始化 steps[count++]。
};  // 结束此类型定义。
struct SequenceIO {  // 定义 SequenceIO（Sequence设备接口） 的类型成员或作用域。
    virtual ~SequenceIO()=default;  // 声明接口 SequenceIO（Sequence设备接口），参数由此处的签名确定。
    /**
     * @brief 读取动作步骤计时所用的设备运行毫秒数。
     */
    virtual uint32_t now()=0;  // 声明接口 now，参数由此处的签名确定。
    /**
     * @brief 查询序列取消状态；true 时不再执行后续步骤。
     */
    virtual bool cancelled()=0;  // 声明接口 cancelled（已取消），参数由此处的签名确定。
    /**
     * @brief 下发当前步骤的硬件命令；不等待机构走完整个行程。
     * @return nullptr 发令成功，否则返回错误原因。
     */
    virtual const char *issue(const Step &step)=0;  // 声明接口 issue，参数由此处的签名确定。
    // ready 只读取实际反馈；计时模式由 Runner 显式选择，不自动降级。
    /**
     * @brief 读取当前步骤涉及的实际反馈并更新 done；不会用目标缓存代替实测。
     * @return nullptr 查询正常，否则返回错误原因；done=true 表示反馈已满足完成条件。
     */
    virtual const char *ready(const Step &step,bool &done)=0;  // 声明接口 ready（就绪），参数由此处的签名确定。
    /**
     * @brief 判断该步骤应采用实际反馈还是固定等待时间来决定完成。
     */
    virtual bool feedback(const Step &step)=0;  // 声明接口 feedback（反馈），参数由此处的签名确定。
    /**
     * @brief 输出步骤开始或结束事件，用于日志耗时分析；begin=true 表示开始。
     */
    virtual void event(size_t index,const Step &step,bool begin)=0;  // 声明接口 event，参数由此处的签名确定。
};  // 结束此类型定义。
class Sequence {  // 定义 Sequence 的类型成员或作用域。
    const Program *program=nullptr;  // 将 nullptr 写入或初始化 program。
    SequenceIO *io=nullptr;  // 将 nullptr 写入或初始化 io（设备接口）。
    size_t index=0;  // 将 0 写入或初始化 index（下标）。
    uint32_t issuedAt=0,readyAt=0,due=0;  // 将 0,readyAt=0,due=0 写入或初始化 issuedAt。
    bool issued=false,settled=false,finished=true,retriedFeedback=false;  // 将 false,settled=false,finished=true,retriedFeedback=false 写入或初始化 issued。
    const char *error=nullptr;  // 将 nullptr 写入或初始化 error（错误）。
public:  // 此后的成员采用 public 访问权限，直到下一个权限标签。
    /**
     * @brief 绑定动作程序及硬件接口并重置序列进度；首次发令由 tick 执行。
     */
    void begin(const Program &steps,SequenceIO &hardware,size_t firstStep=0) {  // 定义函数或成员 的入口 begin，参数由此处的签名确定。
        *this=Sequence{};program=&steps;io=&hardware;index=firstStep;finished=false;due=hardware.now();  // 将 Sequence{};program=&steps;io=&hardware;index=firstStep… 写入或初始化 this。
    }  // 结束此初始化列表或局部作用域。
    /**
     * @brief 判断机械臂步骤序列是否仍在执行。
     */
    bool active() const{return !finished;}  // 定义 active（活跃），调用时返回 !finished。
    /**
     * @brief 读取序列错误原因；nullptr 表示尚未记录错误。
     */
    const char *failure() const{return error;}  // 定义函数或成员 failure，参数由此处的签名确定。
    /**
     * @brief 取得当前执行步骤的数组下标，从0开始。
     */
    size_t stepIndex() const{return index;}  // 定义 stepIndex（step下标），调用时返回 index。
    /**
     * @brief 读取当前步骤名称，序列结束后返回 DONE。
     */
    const char *phase() const{return active()&&index<program->count?program->steps[index].name:"DONE";}  // 定义函数或成员 phase（阶段），参数由此处的签名确定。
    /**
     * @brief 计算距离下一次允许推进还需等待的毫秒数；0 表示可立即 tick。
     */
    uint32_t nextDelay() const {int32_t left=static_cast<int32_t>(due-io->now());return left>0?left:0;}  // 将 static_cast<int32_t>(due-io->now());return left>0?left… 写入或初始化 left（左）。
    /**
     * @brief 推进当前步骤一次：先发令，再按反馈或 waitMs 检查完成，随后切换下一步；不阻塞等待整个动作。
     */
    void tick() {  // 定义函数或成员 的入口 tick（轮询），参数由此处的签名确定。
        if(finished)return;  // 当 finished 成立时进入此分支，否则继续后续流程。
        if(io->cancelled()){error="STOPPED";finished=true;return;}  // 收到取消时记录 STOPPED，标记序列结束并返回。
        if(nextDelay())return;  // 距离下次可推进的时刻仍有延时，本轮提前返回。
        if(index==program->count){finished=true;return;}  // 下标已到步骤总数，标记整个序列结束后返回。
        const Step &step=program->steps[index];  // 将 program->steps[index] 写入或初始化 step。
        if(!issued) {  // 当 !issued 成立时进入此分支，否则继续后续流程。
            io->event(index,step,true);  // 向执行接口报告步骤事件，传入 index,step,true。
            error=io->issue(step);if(error){finished=true;return;}  // 下发当前步骤并记录错误。若发令失败，立即结束序列。
            issued=true;issuedAt=io->now();  // 将 true 写入或初始化 issued。然后将 io->now() 写入或初始化 issuedAt。
            // 发令和等待分离：本次 tick 发出命令后立即让出执行权。
            return;  // 结束当前函数，将执行权交回调用者。
        }  // 结束此条件分支。
        // 反馈步骤先等 ready 确认到位，再从 readyAt 起算保持时间。
        // 定时步骤从 issuedAt 起算 waitMs，所以这两种时间不能直接互换。
        if(io->feedback(step)) {  // 当 io->feedback(step) 成立时进入此分支，否则继续后续流程。
            if(!settled) {  // 当 !settled 成立时进入此分支，否则继续后续流程。
                bool done=false;error=io->ready(step,done);  // 将 false 写入或初始化 done（完成）。然后将 io->ready(step,done) 写入或初始化 error（错误）。
                if(error) {  // 当 error 成立时进入此分支，否则继续后续流程。
                    // 只有列出的机构步骤和通信错误可以延后再查，且保留当前步骤与已发令状态。
                    // 下一次 tick 继续查询反馈。
                    if(!retriedFeedback&&retryableFeedback(error)&&(step.kind==StepKind::ARM||  // 当 !retriedFeedback&&retryableFeedback(error)&&(step.kind… 成立时进入此分支，否则继续后续流程。
                        step.kind==StepKind::TURRET_READY||step.kind==StepKind::TRAY_READY||step.kind==StepKind::TRAY_ROTATE)) {  // 继续上一行的组合条件，本行提供 step.kind==StepKind::TURRET_READY||step.kind==StepKind::TRAY_READY||step.kind==StepKind::…。
                        retriedFeedback=true;error=nullptr;due=io->now()+50;return;  // 将 true 写入或初始化 retriedFeedback（retried反馈）。然后将 nullptr 写入或初始化 error（错误）。然后将 io->now()+50 写入或初始化 due。然后结束当前函数，将执行权交回调用者。
                    }  // 结束此初始化列表或局部作用域。
                    finished=true;return;  // 将 true 写入或初始化 finished。然后结束当前函数，将执行权交回调用者。
                }  // 结束此条件分支。
                retriedFeedback=false;  // 将 false 写入或初始化 retriedFeedback（retried反馈）。
                if(!done){due=io->now()+50;return;}  // 反馈表明尚未到位时，安排 50 ms 后再次查询并结束本轮轮询。
                settled=true;readyAt=io->now();  // 将 true 写入或初始化 settled。然后将 io->now() 写入或初始化 readyAt（就绪At）。
            }  // 结束此条件分支。
            if(io->now()-readyAt<step.holdMs){due=readyAt+step.holdMs;return;}  // 当 io->now()-readyAt<step.holdMs 成立时进入此分支，否则继续后续流程。
        } else if(io->now()-issuedAt<step.waitMs){due=issuedAt+step.waitMs;return;}  // 仅在先前条件均不成立时，当 io->now()-issuedAt<step.waitMs 成立时进入此分支，否则继续后续流程。
        if(io->cancelled()){error="STOPPED";finished=true;return;}  // 收到取消时记录 STOPPED，标记序列结束并返回。
        io->event(index,step,false);++index;issued=settled=retriedFeedback=false;due=io->now();  // 向执行接口报告步骤事件，传入 index,step,false);++index;issued=settled=retriedFeedba…。
        if(index==program->count)finished=true;  // 当 index==program->count 成立时进入此分支，否则继续后续流程。
    }  // 结束此函数或控制块。
};  // 结束此类型定义。
}  // 结束命名空间。
