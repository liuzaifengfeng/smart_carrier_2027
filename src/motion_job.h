// 维护作业启动、取消与接续所需的状态。
// 作业编号及代际信息用于关联当前请求与异步控制信号。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include "motion_types.h"  // 引入 motion_types.h 中的声明和定义。

namespace motion {  // 定义 motion 的类型成员或作用域。
struct LaunchGate {  // 定义 LaunchGate 的类型成员或作用域。
    uint32_t generation=0,releasedId=0;  // 将 0,releasedId=0 写入或初始化 generation。
    /**
     * @brief 记录新作业是否已允许启动；launch=false 时需等待 ACK 后放行。
     */
    void prepare(uint32_t id,bool launch){releasedId=launch?id:0;}  // 将 launch?id:0;} 写入或初始化 releasedId（released编号）。
    /**
     * @brief 允许指定作业开始执行，不代表作业已经完成。
     */
    void release(uint32_t id){releasedId=id;}  // 将 id;} 写入或初始化 releasedId（released编号）。
    /**
     * @brief 递增取消代次，使此前提交的作业失效。
     */
    void cancel(){++generation;}  // 定义函数或成员 cancel，参数由此处的签名确定。
    /**
     * @brief 比较提交代次与当前代次；true 表示该旧作业已被取消。
     */
    bool cancelled(uint32_t submittedGeneration) const{return submittedGeneration!=generation;}  // 定义函数或成员 cancelled（已取消），参数由此处的签名确定。
    /**
     * @brief 判断队首作业是否应被工作任务消费；已放行作业可执行，取消或 OTA 作业被消费后结束。
     */
    bool runnable(uint32_t id,uint32_t submittedGeneration,bool ota,bool aborted) const {  // 定义函数或成员 的入口 runnable，参数由此处的签名确定。
        return id==releasedId||cancelled(submittedGeneration)||ota||aborted;  // 结束当前函数并返回 id==releasedId||cancelled(submittedGeneration)||ota||a…。
    }  // 结束此函数或控制块。
};  // 结束此类型定义。
// 路径和观察姿态是同一作业的两个完成条件；从最终平移的前一运动阶段展开。
struct Join {  // 定义 Join 的类型成员或作用域。
    bool armStarted=false,chassisDone=false,armReady=false;  // 将 false,chassisDone=false,armReady=false 写入或初始化 armStarted（机械臂Started）。
    /**
     * @brief 记录底盘完成状态，在观察准备阶段或底盘完成时仅触发一次机械臂展开。
     * @return true 本次应启动观察姿态，false 尚未到触发阶段或已经启动。
     */
    bool shouldStartArm(bool chassisFinished,bool observationApproach) {  // 定义函数或成员 的入口 shouldStartArm（should起点机械臂），参数由此处的签名确定。
        chassisDone=chassisFinished;  // 将 chassisFinished 写入或初始化 chassisDone（chassis完成）。
        if(armStarted)return false;  // 若 armStarted，立即返回 false。
        if(!chassisFinished&&!observationApproach)return false;  // 若 !chassisFinished&&!observationApproach，立即返回 false。
        armStarted=true;return true;  // 将 true 写入或初始化 armStarted（机械臂Started）。然后结束当前函数并返回 true。
    }  // 结束此函数或控制块。
    /**
     * @brief 判断底盘到位与机械臂准备是否同时完成；合流作业须满足两者。
     */
    bool complete() const{return chassisDone&&armReady;}  // 定义 complete，调用时返回 chassisDone&&armReady。
};  // 结束此类型定义。
// 业务状态进入时提交一次，后续轮询同一编号，不以 ACK 或提交成功推进业务。
struct Ticket {  // 定义 Ticket 的类型成员或作用域。
    uint32_t id=0;  // 将 0 写入或初始化 id（编号）。
    /**
     * @brief 首次提交动作并记住编号；后续调用复用编号，不重复提交。
     * @return true 已持有有效作业编号，false 本次提交失败。
     */
    template<class Submit> bool submitOnce(const MotionRequest &request,Submit submit) {  // 为下一条声明提供模板参数 <class Submit> bool submitOnce(const MotionRequest &re…。
        if(id)return true;id=submit(request);return id!=0;  // 若 id，立即返回 true;id=submit(request);return id!=0。
    }  // 结束此函数或控制块。
    /**
     * @brief 清除已消费的作业编号，使下一阶段可以提交新动作。
     */
    void clear(){id=0;}  // 将 0;} 写入或初始化 id（编号）。
};  // 结束此类型定义。
}  // 结束命名空间。
