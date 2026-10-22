#pragma once
#include <Arduino.h>
#include <FreeRTOS.h>
#include <semphr.h>

/**
 * @brief 创建电机总线递归事务锁，使一次请求及其回复读取不被其他任务交叉打断。
 */
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
