# smart_carrier_2027

**2027 中国大学生工程实践与创新能力大赛 · 智能搬运赛项 · 主控框架**

> 由原「超市机器人」项目删减移植而来。保留底层驱动，移除旧的定位/雷达/货架逻辑。

## 团队分工（本项目只含你的部分）
- **ESP32 主控**：运动控制 + 任务编排 + 决策 ← 本代码
- **机载电脑**：视觉识别（颜色/位置/二维码/转盘物料）+ 下发指令
- **机械组**：整车机械构造、物理尺寸（投影≤300×300mm、高≤400mm）

## 硬件架构（设备清单）
> 由团队硬件规划确认。注意与旧代码的差异：雷达/相机接 **jetson_nano**，
> ESP32 只保留运动/IO/总线职责；**6x_motor_bus = 4 底盘麦克纳姆轮 + 2 机械爪电机**
> （沿用旧 4 轮麦轮运动学，另 2 电机供机械爪使用）。

```
robot_chassis:
├─ power_system(24V)
│  ├─ battery_lithium: 24V 原始动力电源 → 直供 6x_motor_bus
│  ├─ dcdc_12v(24V→12V) → servo_control_board (舵机控制板)
│  │  └─ dcdc_19v(12V→19V) → jetson_nano
│  ├─ dcdc_5v(24V→5V)  → esp32_s3 / scan_module / radar_module
├─ compute_mcu
│  ├─ jetson_nano: 视觉/AI 算力核心
│  ├─ esp32_s3: 底层主控(运动/IO/总线)
│  └─ link: Type-C UART ⇄ 双向通信 (jetson_nano ↔ esp32_s3)
├─ sensor_layer
│  ├─ usb_cam_1: USB → jetson_nano (前向视觉)
│  ├─ usb_cam_2: USB → jetson_nano (全景/俯视)
│  ├─ scan_module: UART TTL → esp32_s3 (条码/二维码)
│  ├─ radar_module: UART TTL → jetson_nano (激光/超声波测距)
│  └─ servo_feedback: UART ⇄ esp32_s3 (通过控制板返回状态/负载/角度)
└─ actuator_layer
   ├─ 6x_motor_bus: UART BUS ⇄ esp32_s3 (6x 轮毂/底盘电机, 指令/反馈)
   └─ 2x_servo: UART ⇄ esp32_s3 → 12V servo_control_board (双独立舵机, 转向/云台)
```

**关键接线对应（主控侧）**
| 外设 | 接主控(ESP32) | 接 jetson_nano | 说明 |
|---|---|---|---|
| 6x 轮毂/底盘电机 | ✅ UART BUS | — | 经由双向串行总线控制，直连24V供电 |
| 12V 舵机控制板 | ✅ UART | — | 控制2个转向/云台舵机并反馈数据 |
| 二维码扫描 | ✅ UART TTL | — | scan_module |
| USB 摄像头 ×2 | — | ✅ USB | 前向/俯视视觉 |
| 雷达模块 | — | ✅ UART | 定位/避障(由 jetson 处理) |
| jetson_nano | ✅ Type-C UART | — | 双向通信 |

## 目录结构
```
src/
├── main.cpp        # 主控：新赛制状态机 + 机载电脑通信 + 任务码解析
├── chassis.h/.cpp  # 底盘运动原语（movepose / GotoPose，麦克纳姆轮）
├── Emm_V5.h/.cpp   # 电机驱动协议（原样保留，底层驱动）
├── servo.h/.cpp    # UART 总线舵机驱动（Fashion Star 协议，HA8-U25H-M）
└── ota_service.*   # WiFi + OTA 无线烧录（原样保留）
```

## 新赛制业务流程（状态机）
```
待机WAIT → 读任务码READ_TASK → 第一批抓取GRAB_R1
       → 放粗加工PLACE_C1 → 放暂存PLACE_T1
       → 第二批抓取GRAB_R2 → 放粗加工PLACE_C2
       → 码垛STACK_T2 → 回启停区HOME → 完成DONE
```

## 任务码格式（已实现解析 parseTaskCode）
四组三位数，如 `156+123+516+231`：
- 颜色编号：红1 黄2 蓝3 绿4 黑5 浅蓝6
- 第一批颜色顺序 / 第一批放置位置 / 第二批颜色顺序 / 第二批放置位置

## 机载电脑 ↔ 主控 串口协议（Serial0）
```
机载电脑 → 主控:
  "ready"             机载电脑就绪
  "task:156+123+..."  下发任务码
  "color:N"           识别到颜色N物料，请求抓取
  "target:x,y,theta"  下发目标坐标
  "ok"                视觉确认到位
主控 → 机载电脑:
  "[TASK:OK]" / "[GRAB_OK]" / "[PLACE_OK]"
```

## 待办事项 [TODO]（按优先级）
1. **定位方案**：原雷达/货架定位已删。新开放场地如何获取位姿？
   候选：机载电脑视觉融合 / 编码器里程计 / 雷达边界测距 / SLAM。
   绝对坐标 `GotoPose(...,false)` 依赖此方案。
2. **底盘标定**：`X_PULSE / Y_PULSE / THETA_PULSE / HEIGHT_PULSE` 需按新底盘重新标定。
3. **任务码显示装置**：硬性要求（字高≥12mm、醒目、不被遮挡），需接显示硬件。
4. **转盘动态抓取**：原料区为旋转转盘（6-10s/圈、转向随机、120°分布）。
5. **抓取/放置/码垛**：禁止手爪夹持运送，需放上载物台；码垛需高度控制。
6. **避障**：场地有随机黑色障碍物（φ50×100mm）。
7. **舵机初始角度**：`servo.cpp` / 业务抓取逻辑中待机械组确认后标定。

## 备份
原版完整项目已备份至 `../chaoshi_backup_2027/`
（含全部旧逻辑，如需回看/对照可查阅）。
