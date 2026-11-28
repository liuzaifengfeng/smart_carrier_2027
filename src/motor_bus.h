// 声明电机串口的作用域锁。
// 读取反馈时，应把发出查询和读取对应应答作为一段总线操作来阅读。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include <Arduino.h>  // 引入 Arduino.h 中的声明和定义。
#include <FreeRTOS.h>  // 引入 FreeRTOS.h 中的声明和定义。
#include <semphr.h>  // 引入 semphr.h 中的声明和定义。

/**
 * @brief 创建电机总线递归事务锁，使一次请求及其回复读取不被其他任务交叉打断。
 */
void MotorBusInit();  // 声明接口 MotorBusInit（Motor总线Init），参数由此处的签名确定。
// 一次请求到应答读取共用事务锁；物理动作等待不持锁。
class MotorBusGuard {  // 定义 MotorBusGuard（Motor总线Guard） 的类型成员或作用域。
    bool held;  // 声明 held，类型和可用范围由所在声明决定。
public:  // 此后的成员采用 public 访问权限，直到下一个权限标签。
    MotorBusGuard();  // 调用 MotorBusGuard（Motor总线Guard），使用无参形式推进相应操作。
    ~MotorBusGuard();  // 声明接口 MotorBusGuard（Motor总线Guard），参数由此处的签名确定。
    MotorBusGuard(const MotorBusGuard &)=delete;  // 调用 MotorBusGuard（Motor总线Guard），传入 const MotorBusGuard &)=delete。
    MotorBusGuard &operator=(const MotorBusGuard &)=delete;  // 将 (const MotorBusGuard &)=delete 写入或初始化 operator。
};  // 结束此类型定义。
