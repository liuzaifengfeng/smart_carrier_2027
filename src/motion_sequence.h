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
};
struct Program {
    // 整组三件最多 45 步；静态存储，运行中不申请堆内存。
    static constexpr size_t CAPACITY=64;
    Step steps[CAPACITY];
    size_t count=0;
    void append(const Step &step){steps[count++]=step;}
};
struct SequenceIO {
    virtual ~SequenceIO()=default;
    virtual uint32_t now()=0;
    virtual bool cancelled()=0;
    virtual const char *issue(const Step &step)=0;
    // ready 只读取实际反馈；计时模式由 Runner 显式选择，不自动降级。
    virtual const char *ready(const Step &step,bool &done)=0;
    virtual bool feedback(const Step &step)=0;
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
    void begin(const Program &steps,SequenceIO &hardware) {
        *this=Sequence{};program=&steps;io=&hardware;finished=false;due=hardware.now();
    }
    bool active() const{return !finished;}
    const char *failure() const{return error;}
    size_t stepIndex() const{return index;}
    const char *phase() const{return active()&&index<program->count?program->steps[index].name:"DONE";}
    uint32_t nextDelay() const {int32_t left=static_cast<int32_t>(due-io->now());return left>0?left:0;}
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
