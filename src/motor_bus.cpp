#include "motor_bus.h"
namespace {SemaphoreHandle_t bus=nullptr;}
/**
 * @brief 创建电机总线递归事务锁，使一次请求及其回复读取不被其他任务交叉打断。
 */
void MotorBusInit(){if(!bus)bus=xSemaphoreCreateRecursiveMutex();}
MotorBusGuard::MotorBusGuard():held(bus&&xSemaphoreTakeRecursive(bus,portMAX_DELAY)==pdTRUE){}
MotorBusGuard::~MotorBusGuard(){if(held)xSemaphoreGiveRecursive(bus);}
