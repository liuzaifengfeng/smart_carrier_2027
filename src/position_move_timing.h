// 位置动作的时间估算辅助代码。
// 这里计算轮询调度所需的时间，查到位判据还需查看调用处的反馈逻辑。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include <stdint.h>  // 引入 stdint.h 中的声明和定义。
#include <math.h>  // 引入 math.h 中的声明和定义。

constexpr uint8_t POSITION_MOVE_DEFAULT_ACCELERATION = 180;  // 将 180 写入或初始化 POSITION_MOVE_DEFAULT_ACCELERATION（位置MOVEDEFAULT加速度）。

// EMM V5位置模式，1~255加速度档位：每增加1 RPM需(256-acc)*50us。
// 从静止加速、减速到静止；短行程达不到设定轮速时按三角形速度曲线计算。
/**
 * @brief 按EMM V5加减速模型估算位置动作总时间(ms)，短行程按三角形速度曲线计算。
 * @param pulses 行程脉冲数，按3200脉冲/圈换算。
 * @param rpm 最大转速(RPM)。
 * @param acceleration 1~255加速度档位。
 * @return 理论运行时间，不代表实际反馈到位。
 */
// 把脉冲目标、转速和加速度档位换成动作时间。
// 上层使用这个时间安排位置动作轮询。
inline double PositionMoveDurationMs(uint32_t pulses,uint16_t rpm,uint8_t acceleration) {  // 定义函数或成员 的入口 PositionMoveDurationMs（位置Move持续时间毫秒），参数由此处的签名确定。
    const double rpmPerSecond=20000.0/(256-acceleration);  // 将 20000.0/(256-acceleration) 写入或初始化 rpmPerSecond（转速PerSecond）。
    const double distanceRpmSeconds=static_cast<double>(pulses)*60.0/3200.0;  // 将 static_cast<double>(pulses)*60.0/3200.0 写入或初始化 distanceRpmSeconds（距离转速Seconds）。
    const double rampDistance=static_cast<double>(rpm)*rpm/rpmPerSecond;  // 将 static_cast<double>(rpm)*rpm/rpmPerSecond 写入或初始化 rampDistance（ramp距离）。
    const double seconds=distanceRpmSeconds>=rampDistance  // 将 distanceRpmSeconds>=rampDistance 写入或初始化 seconds。
        ? distanceRpmSeconds/rpm+rpm/rpmPerSecond  // 续写上一行表达式的条件取值分支。
        : 2.0*sqrt(distanceRpmSeconds/rpmPerSecond);  // 声明接口 sqrt，参数由此处的签名确定。
    return seconds*1000.0;  // 结束当前函数并返回 seconds*1000.0。
}  // 结束此函数或控制块。
