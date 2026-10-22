#pragma once
#include <stdint.h>
#include <stddef.h>

namespace motion {
enum class StepKind : uint8_t { ARM, WAIT, TURRET_READY, POSITION, GOTO, COMMIT, RAW_SERVO, ENABLE, VELOCITY };
struct Step {
    StepKind kind=StepKind::WAIT;
    const char *name="WAIT";
    float high=-1,length=-1,turret=-1,pawl=-1,speed=0;
    uint32_t waitMs=0,holdMs=0;
    uint8_t cargo=0,color=0;
    bool planned=false;
    uint16_t turretRampMs=600; // 2号转台加速、减速各自时长；由动作阶段指定。
};
struct Program {
    // 整组三件最多 45 步；静态存储，运行中不申请堆内存。
    static constexpr size_t CAPACITY=64;
    Step steps[CAPACITY];
    size_t count=0;
    /**
     * @brief 向动作程序末尾追加一步；调用方保证静态步骤容量足够。
     */
    void append(const Step &step){steps[count++]=step;}
};
struct SequenceIO {
    virtual ~SequenceIO()=default;
    /**
     * @brief 读取动作步骤计时所用的设备运行毫秒数。
     */
    virtual uint32_t now()=0;
    /**
     * @brief 查询序列取消状态；true 时不再执行后续步骤。
     */
    virtual bool cancelled()=0;
    /**
     * @brief 下发当前步骤的硬件命令；不等待机构走完整个行程。
     * @return nullptr 发令成功，否则返回错误原因。
     */
    virtual const char *issue(const Step &step)=0;
    // ready 只读取实际反馈；计时模式由 Runner 显式选择，不自动降级。
    /**
     * @brief 读取当前步骤涉及的实际反馈并更新 done；不会用目标缓存代替实测。
     * @return nullptr 查询正常，否则返回错误原因；done=true 表示反馈已满足完成条件。
     */
    virtual const char *ready(const Step &step,bool &done)=0;
    /**
     * @brief 判断该步骤应采用实际反馈还是固定等待时间来决定完成。
     */
    virtual bool feedback(const Step &step)=0;
    /**
     * @brief 输出步骤开始或结束事件，用于日志耗时分析；begin=true 表示开始。
     */
    virtual void event(size_t index,const Step &step,bool begin)=0;
};
class Sequence {
    const Program *program=nullptr;
    SequenceIO *io=nullptr;
    size_t index=0;
    uint32_t issuedAt=0,readyAt=0,due=0;
    bool issued=false,settled=false,finished=true;
    const char *error=nullptr;
public:
    /**
     * @brief 绑定动作程序及硬件接口并重置序列进度；首次发令由 tick 执行。
     */
    void begin(const Program &steps,SequenceIO &hardware) {
        *this=Sequence{};program=&steps;io=&hardware;finished=false;due=hardware.now();
    }
    /**
     * @brief 判断机械臂步骤序列是否仍在执行。
     */
    bool active() const{return !finished;}
    /**
     * @brief 读取序列错误原因；nullptr 表示尚未记录错误。
     */
    const char *failure() const{return error;}
    /**
     * @brief 取得当前执行步骤的数组下标，从0开始。
     */
    size_t stepIndex() const{return index;}
    /**
     * @brief 读取当前步骤名称，序列结束后返回 DONE。
     */
    const char *phase() const{return active()&&index<program->count?program->steps[index].name:"DONE";}
    /**
     * @brief 计算距离下一次允许推进还需等待的毫秒数；0 表示可立即 tick。
     */
    uint32_t nextDelay() const {int32_t left=static_cast<int32_t>(due-io->now());return left>0?left:0;}
    /**
     * @brief 推进当前步骤一次：先发令，再按反馈或 waitMs 检查完成，随后切换下一步；不阻塞等待整个动作。
     */
    void tick() {
        if(finished)return;
        if(io->cancelled()){error="STOPPED";finished=true;return;}
        if(nextDelay())return;
        if(index==program->count){finished=true;return;}
        const Step &step=program->steps[index];
        if(!issued) {
            io->event(index,step,true);
            error=io->issue(step);if(error){finished=true;return;}
            issued=true;issuedAt=io->now();
            // 发令和等待分离：本次 tick 发出命令后立即让出执行权。
            return;
        }
        if(io->feedback(step)) {
            if(!settled) {
                bool done=false;error=io->ready(step,done);
                if(error){finished=true;return;}
                if(!done){due=io->now()+50;return;}
                settled=true;readyAt=io->now();
            }
            if(io->now()-readyAt<step.holdMs){due=readyAt+step.holdMs;return;}
        } else if(io->now()-issuedAt<step.waitMs){due=issuedAt+step.waitMs;return;}
        if(io->cancelled()){error="STOPPED";finished=true;return;}
        io->event(index,step,false);++index;issued=settled=false;due=io->now();
        if(index==program->count)finished=true;
    }
};
}
