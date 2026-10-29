#pragma once
#include "chassis.h"
#include "node_position_hardware.h"

// 工位下降结束、松爪之前各回读一次；仅记录，不参与动作完成判定。
inline void LogArmPlacementReadback(uint32_t job,uint8_t position,float targetAngle,float targetLength) {
    float angle=0;
    if(ReadArmTurretAngle(angle))
        Serial.printf("[MaterialPlace] job=%lu position=%u axis=turret target=%.3f actual=%.3f error=%+.3f unit=deg\n",
            static_cast<unsigned long>(job),position,targetAngle,angle,angle-targetAngle);
    else Serial.printf("[MaterialPlace] job=%lu position=%u axis=turret read=ERR\n",
        static_cast<unsigned long>(job),position);

    int64_t encoder=0;
    const char *error=ReadMotorEncoderPosition(6,encoder);
    if(!error) {
        // 伸出正方向为CCW，对应负编码器坐标；保留符号以显示反向偏差。
        const double length=-static_cast<double>(encoder)*3200.0/65536.0/LENGTH_PULSE;
        Serial.printf("[MaterialPlace] job=%lu position=%u axis=length target=%.3f actual=%.3f error=%+.3f unit=mm\n",
            static_cast<unsigned long>(job),position,targetLength,length,length-targetLength);
    } else Serial.printf("[MaterialPlace] job=%lu position=%u axis=length read=ERR reason=%s\n",
        static_cast<unsigned long>(job),position,error);
}
