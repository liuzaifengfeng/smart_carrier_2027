# smart_carrier_2027

**2027 中国大学生工程实践与创新能力大赛 · 智能搬运赛项 · 主控框架**

统一运动任务、非阻塞两轮流程、异步串口及计时使用说明见 [统一运动执行](Document/motion_executor.md)。提前展开与反馈完成默认关闭。

## 代码约定

上位机新功能统一在 [Web 上位机](web/connection-test/README.md) 实现。Python/Tkinter 上位机（`upper_computer.py`）作为旧版保留，新功能无需同步到 Python 版。

内部运动、搬运和控制函数采用契约式设计：调用方提供有效参数、已标定配置和已解析任务码，内部不重复校验或静默替换参数。串口、扫码、配置文件和硬件协议入口负责检查外部数据。

保留控制算法所需的限幅、协议字段饱和、任务取消、通信错误上报和载物台占用判断。舵机查询失败直接返回错误，后续调用可重新查询；开机角度校验失败不会永久锁住转台，也不阻断 OTA、串口维护通道。

## 团队分工（本项目只含你的部分）
- **ESP32 主控**：运动控制 + 任务编排 + 决策 ← 本代码
- **机载电脑**：视觉识别（颜色/位置/二维码/转盘物料）+ 下发指令
- **机械组**：整车机械构造、物理尺寸（投影≤300×300mm、高≤400mm）

## 硬件架构（设备清单）
> 由团队硬件规划确认。注意与旧代码的差异：雷达/相机接 **jetson_nano**，
> ESP32 只保留运动/IO/总线职责；**7x_motor_bus = 4 底盘麦克纳姆轮 + 升降 + 伸缩 + 转盘**
> （沿用旧 4 轮麦轮运动学，另 3 电机供机械臂使用）。

```
robot_chassis:
├─ power_system(24V)
│  ├─ battery_lithium: 24V 原始动力电源 → 直供 7x_motor_bus
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
   ├─ 7x_motor_bus: UART BUS ⇄ esp32_s3 (地址1～4底盘、5升降、6伸缩、7转盘)
   └─ 1x_servo: UART ⇄ esp32_s3 → 12V servo_control_board (ID 1夹爪)
```

**关键接线对应（主控侧）**
| 外设 | 接主控(ESP32) | 接 jetson_nano | 说明 |
|---|---|---|---|
| 7x 张大头电机 | Serial1 TX=GPIO16、RX=GPIO15 | — | 经由双向串行总线控制，直连24V供电 |
| 12V 舵机控制板 | Serial2 TX=GPIO18、RX=GPIO17 | — | 控制1号夹爪舵机并反馈数据 |
| 二维码扫描模块 | SoftwareSerial RX=GPIO4 | — | 单向接收，9600 bps |
| USB 摄像头 | — | ✅ USB | 前向/俯视视觉 |
| 雷达模块 | — | ✅ UART | 定位/避障(由 jetson 处理) |
| jetson_nano | ✅ Type-C UART | — | 双向通信 |

新版主板其余接口的原理图分配为 `ZONE_ADC` GPIO1、`START_ADC` GPIO2、`EXPAND1` GPIO6、`EXPAND2` GPIO7、`PWM1` GPIO8、`BZE` GPIO9、`ADC_BAT` GPIO10、`BUTTON_LED` GPIO11。

启动面板使用数字输入，硬件需短接 R10、拆除 R13。Debug 空闲时拨杆变化会触发 Release；上电只记录初始档位。机械臂初始化和航向采样结束后，按下 GPIO2 启动按钮锁定当时的拨杆档位，随后执行雷达位姿准备并自动进入扫码。CN12.2 接通 CN12.1 时 GPIO1 为低电平，对应区 1；接通 CN12.3 时 GPIO1 为高电平，对应区 2。拨杆断开也会读到低电平。Q3 在 Debug 空闲时常亮；Release 后未按下启动按钮时，区 1 每秒单闪、区 2 每秒双闪。正常任务期间每 2 秒呼吸一次。

## 目录结构

H30 惯导已接入 ESP32-S3 原生 USB Host，上位机连接栏提供三轴姿态单次查询。接线、协议和验收参见 [H30 USB 接入说明](Document/H30_USB.md)。
正式节点导航已关闭漂移切弯，外侧与中间十字均采用有限脉冲位置模式：直行到节点、停车转向、再前行，共线同向节点合并。保留现有陀螺仪辅助、校角死区和参数页速度/加速度，不新增等待或故障锁死。独立试跑中的旧曲线功能保留，不被正式导航调用。说明见 [节点导航](Document/continuous_trial.md)。

```
src/
├── main.cpp                         # 程序入口与各硬件/业务模块初始化
├── robot_runtime.* / robot_state.h   # FreeRTOS 主任务、赛事状态机与串口命令
├── competition_flow.h / task_code.h # 任务码解析与新赛制业务流程
├── motion_* / arm_motion.*           # 统一运动队列、动作序列与机械臂规划
├── chassis.* / node_position_*       # 底盘原语与节点脉冲位置导航
├── continuous_* / field_geometry.*   # 独立试跑路径与场地几何
├── vision_alignment.h / ring_correction.h # 视觉对齐与圆环位置修正
├── scanner.* / scan_motion_control.* # 任务码扫描与扫码行程控制
├── material_transfer.* / disc_*      # 物料判定、抓取、放置与码垛
├── h30_* / start_panel.h             # H30 USB 惯导与启动面板
├── Emm_V5.* / motor_bus.*            # EMM V5 电机协议与 Serial1 总线事务锁
├── turret_motor.* / servo.*          # 转盘电机与 Fashion Star 总线舵机驱动
├── runtime_parameters.*              # 运行时参数注册、读写与持久化
└── ota_service.*                     # WiFi + OTA 无线烧录与恢复通道
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

在完整工程目录内运行 `python upper_computer.py`。上位机需要同目录的参数/动作组模块、
`continuous_trial.py`、`hybrid_plan.py`、`field_geometry.py` 以及 `src/field_geometry.def`，
不要仅复制一个上位机文件。需要安装 `pyserial`，图形界面使用 Python 自带的 Tkinter。
`tests/` 里是开发时运行的自动测试，不是上位机启动文件。可用
`python -m unittest discover -s tests -v` 执行。

Serial0 使用 v2 单行文本帧；命令、回复和主动事件分别以 `CMD`、`RSP`、`EVT` 开头。例如：

```text
{CMD,SYS,HELLO}
{RSP,SYS,HELLO,OK,2}
{CMD,VISION,ALIGN_DATA,0,-120,-121}
{CMD,NAV,ROUTE,0,1,2,7}
{EVT,NAV,ROUTE_REQUEST,0,7}
{CMD,NAV,START_ZONE,1}
```

完整指令、回复、旧版迁移对照及仓库外设备切换顺序见
[串口通信协议](Document/serial_protocol.md)。

圆盘按位置抓取新增 `{CMD,VISION,DISC_POSITIONS,1,5,2,1,3,6}`：视觉电脑负责判断可抓时机，
一帧可回传 1～3 组位置与颜色，ESP 按当前轮次下一件任务颜色选择一个位置。
三个位置分别执行专用夹取动作，再放入当前任务对应载物台；成功后推进任务进度，恢复观察前忽略新位置帧。
调试上位机“视觉”页可上报可抓位置；编号、回包和联调步骤见协议文末“按圆盘位置抓取”。

## 待办事项 [TODO]（按优先级）


## 备份
原版完整项目已备份至 `../chaoshi_backup_2027/`
（含全部旧逻辑，如需回看/对照可查阅）。
