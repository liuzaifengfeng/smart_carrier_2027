#pragma once
#include <Arduino.h>
#include <FreeRTOS.h>
#include <semphr.h>

void MotorBusInit();
// 一次请求到应答读取共用事务锁；物理动作等待不持锁。
class MotorBusGuard {
    bool held;
public:
    MotorBusGuard();
    ~MotorBusGuard();
    MotorBusGuard(const MotorBusGuard &)=delete;
    MotorBusGuard &operator=(const MotorBusGuard &)=delete;
};
