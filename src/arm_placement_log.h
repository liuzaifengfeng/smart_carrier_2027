// 放料日志格式的公共辅助代码。
// 位置字段和动作标识用于关联放料过程，修改输出前应同时查看串口日志消费端。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include "chassis.h"  // 引入 chassis.h 中的声明和定义。
#include "node_position_hardware.h"  // 引入 node_position_hardware.h 中的声明和定义。

// 工位下降结束、松爪之前各回读一次；仅记录，不参与动作完成判定。
inline void LogArmPlacementReadback(uint32_t job,uint8_t position,float targetAngle,float targetLength) {  // 定义函数或成员 的入口 LogArmPlacementReadback（Log机械臂PlacementReadback），参数由此处的签名确定。
    float angle=0;  // 将 0 写入或初始化 angle（角度）。
    if(ReadArmTurretAngle(angle))  // 当 ReadArmTurretAngle(angle) 成立时进入此分支，否则继续后续流程。
        Serial.printf("[MaterialPlace] job=%lu position=%u axis=turret target=%.3f actual=%.3f error=%+.3f unit=deg\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
            static_cast<unsigned long>(job),position,targetAngle,angle,angle-targetAngle);  // 声明接口 long>，参数由此处的签名确定。
    else Serial.printf("[MaterialPlace] job=%lu position=%u axis=turret read=ERR\n",  // 前一个判断不成立时执行此分支。
        static_cast<unsigned long>(job),position);  // 声明接口 long>，参数由此处的签名确定。

    int64_t encoder=0;  // 将 0 写入或初始化 encoder（编码器）。
    const char *error=ReadMotorEncoderPosition(6,encoder);  // 将 ReadMotorEncoderPosition(6,encoder) 写入或初始化 error（错误）。
    if(!error) {  // 当 !error 成立时进入此分支，否则继续后续流程。
        // 伸出正方向为CCW，对应负编码器坐标；保留符号以显示反向偏差。
        const double length=-static_cast<double>(encoder)*3200.0/65536.0/LENGTH_PULSE;  // 将 -static_cast<double>(encoder)*3200.0/65536.0/LENGTH_PU… 写入或初始化 length（长度）。
        Serial.printf("[MaterialPlace] job=%lu position=%u axis=length target=%.3f actual=%.3f error=%+.3f unit=mm\n",  // 通过串口执行 printf，输出内容或参数由本行给出。
            static_cast<unsigned long>(job),position,targetLength,length,length-targetLength);  // 声明接口 long>，参数由此处的签名确定。
    } else Serial.printf("[MaterialPlace] job=%lu position=%u axis=length read=ERR reason=%s\n",  // 前一个判断不成立时执行此分支。
        static_cast<unsigned long>(job),position,error);  // 声明接口 long>，参数由此处的签名确定。
}  // 结束此函数或控制块。
