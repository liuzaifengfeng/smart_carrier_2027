#include "motor_bus.h"
namespace {SemaphoreHandle_t bus=nullptr;}
void MotorBusInit(){if(!bus)bus=xSemaphoreCreateRecursiveMutex();}
MotorBusGuard::MotorBusGuard():held(bus&&xSemaphoreTakeRecursive(bus,portMAX_DELAY)==pdTRUE){}
MotorBusGuard::~MotorBusGuard(){if(held)xSemaphoreGiveRecursive(bus);}
