# 舵机内部PID查询与修改

上位机“机械臂”页下方的“舵机内部 PID”面板提供查询及写入并回读。ID 1为夹爪舵机；转盘已改用Serial1上的7号张大头电机，不使用此面板，逻辑ID 2返回UNSUPPORTED，使用现有Debug调试入口，运动作业执行中沿用现有BUSY回复。

字段顺序为Kp、Kd、Ki、PwmBias、HoldKp、HoldKd、HoldPwmBias，均为舵机协议的16位无符号原始整数。这些是舵机内部参数，与小车视觉对齐PID无关。输入框初始为空，查询后填入实测值。

串口命令：

```text
{CMD,ARM,SERVO_PID_GET,1}
{CMD,ARM,SERVO_PID_SET,1,Kp,Kd,Ki,PwmBias,HoldKp,HoldKd,HoldPwmBias}
{RSP,ARM,SERVO_PID_GET,READBACK,1,Kp,Kd,Ki,PwmBias,HoldKp,HoldKd,HoldPwmBias}
{RSP,ARM,SERVO_PID_SET,READBACK,1,Kp,Kd,Ki,PwmBias,HoldKp,HoldKd,HoldPwmBias}
```

查询或写入失败通过原有ERR协议回复，界面显示错误；READBACK显示实际读回值，不用发送值冒充已写入结果。

实现依据[厂家官方Web UART调试工具](https://fashionstar.com.cn/wiki/software/servo/data/web-controller/?lang=zh&v=29)中的进阶参数读写：使用13 4D请求帧头，C5读取ID及25字节参数块，前14字节是7个小端uint16参数。写入时先读取原块，保留剩余11字节，按官方顺序释放锁力并等待60ms，再以C4写入；写入后按实机调试要求等待300ms，再恢复锁力、重新读取并回传。新增300ms只用于PID写入，不影响机械臂运动时序。不会自动改变项目默认PID或开机自动写入参数。

需要更新ESP32固件并重启新版上位机。报文、保留字段和界面回填已用无硬件测试验证，实际参数效果及断电保存情况需实机确认。
