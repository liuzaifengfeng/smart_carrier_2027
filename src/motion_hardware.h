#pragma once
#include "motion_sequence.h"

class MotionHardware : public motion::SequenceIO {
    uint32_t job=0,turretStarted=0;
    bool (*cancel)()=nullptr;
public:
    /**
     * @brief 绑定当前作业编号与取消回调，供步骤执行和日志记录使用。
     */
    void configure(uint32_t id,bool (*cancelled)()){job=id;cancel=cancelled;}
    /**
     * @brief 读取系统毫秒计时。
     */
    uint32_t now() override;
    /**
     * @brief 检查总任务及当前作业的取消状态。
     */
    bool cancelled() override;
    /**
     * @brief 按步骤类型调用 MoveArm、BeginPositionMove 等硬件入口；发令后立即返回。
     * @return nullptr 成功，否则返回执行错误。
     */
    const char *issue(const motion::Step &step) override;
    /**
     * @brief 检查位置移动、转台或机械臂实测到位条件，并通过 done 返回本次结果。
     * @return nullptr 查询正常，否则返回错误原因。
     */
    const char *ready(const motion::Step &step,bool &done) override;
    /**
     * @brief 判断当前步骤是否使用反馈完成；POSITION、TURRET_READY 或显式开启的机械臂反馈模式使用反馈。
     */
    bool feedback(const motion::Step &step) override;
    /**
     * @brief 输出当前作业的 STEP 开始/结束日志，index 为步骤下标。
     */
    void event(size_t index,const motion::Step &step,bool begin) override;
};
