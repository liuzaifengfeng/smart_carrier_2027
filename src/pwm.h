#ifndef PWM_H
#define PWM_H

#include <Arduino.h>

// 引脚定义
#define PWM1_PIN 4
#define PWM2_PIN 6
#define PWM3_PIN 7
#define PWM4_PIN 5
#define PWM5_PIN 8


// 函数声明
void initPWM();
int angleToDuty(int angle);

#endif // PWM_H
