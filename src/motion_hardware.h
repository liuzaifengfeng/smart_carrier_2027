// 动作步骤执行用的硬件适配接口。
// 动作序列通过此层获得时间、取消状态、到位反馈和步骤事件。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include "motion_sequence.h"  // 引入 motion_sequence.h 中的声明和定义。
#include "ring_correction.h"  // 引入 ring_correction.h 中的声明和定义。
#include "turret_motor.h"  // 引入 turret_motor.h 中的声明和定义。

class MotionHardware : public motion::SequenceIO {  // 定义 MotionHardware 的类型成员或作用域。
    uint32_t job=0,turretStarted=0;  // 将 0,turretStarted=0 写入或初始化 job。
    float turretTarget=0; // 保存本次发令后实际选择的等效多圈目标。
    bool trayMoving=false;  // 将 false 写入或初始化 trayMoving（料盘Moving）。
    bool trayPreselected=false;  // 将 false 写入或初始化 trayPreselected（料盘Preselected）。
    float preselectedTrayAngle=0;  // 将 0 写入或初始化 preselectedTrayAngle（preselected料盘角度）。
    bool (*cancel)()=nullptr;  // 调用 bool，传入 *cancel)()=nullptr。
    bool ringVisionActive=false, ringHasLastFrame=false;  // 将 false, ringHasLastFrame=false 写入或初始化 ringVisionActive（圆环视觉活跃）。
    uint32_t ringLastFrameMs=0;  // 将 0 写入或初始化 ringLastFrameMs（圆环最后帧毫秒）。
    uint32_t ringStartedMs=0,ringTimeoutMs=0;  // 将 0,ringTimeoutMs=0 写入或初始化 ringStartedMs（圆环Started毫秒）。
    float ringMinTurret=0,ringMaxTurret=0,ringMinLength=0,ringMaxLength=0;  // 将 0,ringMaxTurret=0,ringMinLength=0,ringMaxLength=0 写入或初始化 ringMinTurret（圆环最小转台）。
    ring::Settings ringSettings;  // 声明 ringSettings（圆环设置），类型和可用范围由所在声明决定。
    ring::Deadzone ringDeadzone;  // 声明 ringDeadzone（圆环Deadzone），类型和可用范围由所在声明决定。
    ring::Pid ringX,ringY;  // 声明 ringX（圆环横向），类型和可用范围由所在声明决定。
    struct {  // 续写当前声明或运算，提供 struct {。
        uint32_t id=0,sentAtMs=0;  // 将 0,sentAtMs=0 写入或初始化 id（编号）。
        TurretMotorPosition position{};  // 声明 position（位置），类型和可用范围由所在声明决定。
    } lastRingXCommand;  // 续写当前声明或运算，提供 } lastRingXCommand;。
    const char *correctRing(const motion::Step &step,bool &done);  // 声明接口 correctRing（correct圆环），参数由此处的签名确定。
public:  // 此后的成员采用 public 访问权限，直到下一个权限标签。
    /**
     * @brief 绑定当前作业编号与取消回调，供步骤执行和日志记录使用。
     */
    void configure(uint32_t id,bool (*cancelled)()){job=id;cancel=cancelled;}  // 将 id;cancel=cancelled;} 写入或初始化 job。
    void stopTray();  // 声明接口 stopTray（停止料盘），参数由此处的签名确定。
    void stopRingVision();  // 声明接口 stopRingVision（停止圆环视觉），参数由此处的签名确定。
    void preselectTray(float angle);  // 声明接口 preselectTray（preselect料盘），参数由此处的签名确定。
    /**
     * @brief 读取系统毫秒计时。
     */
    uint32_t now() override;  // 声明接口 now，参数由此处的签名确定。
    /**
     * @brief 检查总任务及当前作业的取消状态。
     */
    bool cancelled() override;  // 声明接口 cancelled（已取消），参数由此处的签名确定。
    /**
     * @brief 按步骤类型调用 MoveArm、BeginPositionMove 等硬件入口；发令后立即返回。
     * @return nullptr 成功，否则返回执行错误。
     */
    const char *issue(const motion::Step &step) override;  // 声明接口 issue，参数由此处的签名确定。
    /**
     * @brief 检查位置移动、转台或机械臂实测到位条件，并通过 done 返回本次结果。
     * @return nullptr 查询正常，否则返回错误原因。
     */
    const char *ready(const motion::Step &step,bool &done) override;  // 声明接口 ready（就绪），参数由此处的签名确定。
    /**
     * @brief 判断当前步骤是否使用反馈完成；POSITION、TURRET_READY 或显式开启的机械臂反馈模式使用反馈。
     */
    bool feedback(const motion::Step &step) override;  // 声明接口 feedback（反馈），参数由此处的签名确定。
    /**
     * @brief 输出当前作业的 STEP 开始/结束日志，index 为步骤下标。
     */
    void event(size_t index,const motion::Step &step,bool begin) override;  // 声明接口 event，参数由此处的签名确定。
};  // 结束此类型定义。
