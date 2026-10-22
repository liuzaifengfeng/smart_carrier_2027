#pragma once
#include "motion_types.h"

namespace motion {
struct LaunchGate {
    uint32_t generation=0,releasedId=0;
    void prepare(uint32_t id,bool launch){releasedId=launch?id:0;}
    void release(uint32_t id){releasedId=id;}
    void cancel(){++generation;}
    bool cancelled(uint32_t submittedGeneration) const{return submittedGeneration!=generation;}
    bool runnable(uint32_t id,uint32_t submittedGeneration,bool ota,bool aborted) const {
        return id==releasedId||cancelled(submittedGeneration)||ota||aborted;
    }
};
// 路径和观察姿态是同一作业的两个完成条件；从最终平移的前一运动阶段展开。
struct Join {
    bool armStarted=false,chassisDone=false,armReady=false;
    bool shouldStartArm(bool chassisFinished,bool observationApproach) {
        chassisDone=chassisFinished;
        if(armStarted)return false;
        if(!chassisFinished&&!observationApproach)return false;
        armStarted=true;return true;
    }
    bool complete() const{return chassisDone&&armReady;}
};
// 业务状态进入时提交一次，后续轮询同一编号，不以 ACK 或提交成功推进业务。
struct Ticket {
    uint32_t id=0;
    template<class Submit> bool submitOnce(const MotionRequest &request,Submit submit) {
        if(id)return true;id=submit(request);return id!=0;
    }
    void clear(){id=0;}
};
}
