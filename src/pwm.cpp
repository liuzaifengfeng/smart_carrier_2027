#include "pwm.h"

// 初始化PWM设置
void initPWM() {
  // 初始化PWM引脚
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);
  pinMode(PWM3_PIN, OUTPUT);
  pinMode(PWM4_PIN, OUTPUT);
  pinMode(PWM5_PIN, OUTPUT);

  // 1. 配置 LEDC 通道
  ledcSetup(1, 50, 13);
  ledcSetup(2, 50, 13);
  ledcSetup(3, 50, 13);
  //ledcSetup(4, 50, 13);
  ledcSetup(5, 50, 13);
  // 2. 将引脚绑定到通道
  ledcAttachPin(PWM1_PIN, 1);
  ledcAttachPin(PWM2_PIN, 2);
  ledcAttachPin(PWM3_PIN, 3);
  //ledcAttachPin(PWM4_PIN, 4);
  ledcAttachPin(PWM5_PIN, 5);

  ledcWrite(1,angleToDuty(270) );// [TODO] 舵机通道1 初始角度待标定(原:图像大臂)
  ledcWrite(2,angleToDuty(300) );// [TODO] 舵机通道2 初始角度待标定(原:夹臂)
  ledcWrite(3,angleToDuty(170) );// [TODO] 舵机通道3 初始角度待标定(原:夹爪)
  //ledcWrite(4,angleToDuty(0) );
  ledcWrite(5,angleToDuty(240) );// [TODO] 舵机通道5 初始角度待标定(原:料筒)

}
 
// 计算角度对应的 PWM 数值
int angleToDuty(int angle) {
  //0到360，对应0.5ms-2.5ms 20ms

  return map(angle, 0, 360, 204, 1024);
}

//ledcWrite( 3, angleToDuty(120)); //这样使用