// 通过递归互斥锁串行化电机串口访问。
// 守卫持有期间可调用内部同样使用守卫的函数，析构时释放本次持锁。

#include "motor_bus.h"  // 引入 motor_bus.h 中的声明和定义。
namespace {SemaphoreHandle_t bus=nullptr;}  // 将 nullptr;} 写入或初始化 bus（总线）。
/**
 * @brief 创建电机总线递归事务锁，使一次请求及其回复读取不被其他任务交叉打断。
 */
void MotorBusInit(){if(!bus)bus=xSemaphoreCreateRecursiveMutex();}  // 将 xSemaphoreCreateRecursiveMutex();} 写入或初始化 bus（总线）。
MotorBusGuard::MotorBusGuard():held(bus&&xSemaphoreTakeRecursive(bus,portMAX_DELAY)==pdTRUE){}  // 调用 MotorBusGuard::MotorBusGuard（Motor总线GuardMotor总线Guard），传入 ):held(bus&&xSemaphoreTakeRecursive(bus,portMAX_DELAY)…。
MotorBusGuard::~MotorBusGuard(){if(held)xSemaphoreGiveRecursive(bus);}  // 调用 MotorBusGuard::~MotorBusGuard（Motor总线GuardMotor总线Guard），传入 ){if(held)xSemaphoreGiveRecursive(bus);}。
