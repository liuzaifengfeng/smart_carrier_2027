#pragma once
#include "motion_types.h"

namespace motion {
struct LaunchGate {
    uint32_t generation=0,releasedId=0;
    /**
     * @brief 记录新作业是否已允许启动；launch=false 时需等待 ACK 后放行。
     */
    void prepare(uint32_t id,bool launch){releasedId=launch?id:0;}
    /**
     * @brief 允许指定作业开始执行，不代表作业已经完成。
     */
    void release(uint32_t id){releasedId=id;}
    /**
     * @brief 递增取消代次，使此前提交的作业失效。
     */
    void cancel(){++generation;}
    /**
     * @brief 比较提交代次与当前代次；true 表示该旧作业已被取消。
     */
    bool cancelled(uint32_t submittedGeneration) const{return submittedGeneration!=generation;}
    /**
     * @brief 判断队首作业是否应被工作任务消费；已放行作业可执行，取消或 OTA 作业被消费后结束。
     */
    bool runnable(uint32_t id,uint32_t submittedGeneration,bool ota,bool aborted) const {
        return id==releasedId||cancelled(submittedGeneration)||ota||aborted;
    }
};
// 路径和观察姿态是同一作业的两个完成条件；从最终平移的前一运动阶段展开。
struct Join {
    bool armStarted=false,chassisDone=false,armReady=false;
    /**
     * @brief 记录底盘完成状态，在观察准备阶段或底盘完成时仅触发一次机械臂展开。
     * @return true 本次应启动观察姿态，false 尚未到触发阶段或已经启动。
     */
    bool shouldStartArm(bool chassisFinished,bool observationApproach) {
        chassisDone=chassisFinished;
        if(armStarted)return false;
        if(!chassisFinished&&!observationApproach)return false;
        armStarted=true;return true;
    }
    /**
     * @brief 判断底盘到位与机械臂准备是否同时完成；合流作业须满足两者。
     */
    bool complete() const{return chassisDone&&armReady;}
};
// 业务状态进入时提交一次，后续轮询同一编号，不以 ACK 或提交成功推进业务。
struct Ticket {
    uint32_t id=0;
    /**
     * @brief 首次提交动作并记住编号；后续调用复用编号，不重复提交。
     * @return true 已持有有效作业编号，false 本次提交失败。
     */
    template<class Submit> bool submitOnce(const MotionRequest &request,Submit submit) {
        if(id)return true;id=submit(request);return id!=0;
    }
    /**
     * @brief 清除已消费的作业编号，使下一阶段可以提交新动作。
     */
    void clear(){id=0;}
};
}
