# 固件源码索引

在这页用 Ctrl+F 搜功能名，从对应入口进入源码；已经知道函数名时，可以在 VS Code 直接跳转。

## 在 VS Code 里怎么找

| 已知线索 | 操作 |
| --- | --- |
| 只知道功能，例如“扫码” | Ctrl+Shift+F 全项目搜索，在“要包含的文件”填 `src/**`；可依次试 `扫码`、`SCAN_TASK`、`Scanner_WaitCode`。 |
| 知道串口命令或报错 | 先搜带引号的协议关键字，如 `"TRANSFER"`、`"HEADING_ERROR"`。命令匹配集中在 `serial_commands.inc`，错误字符串通常能定位到返回或上报位置。 |
| 知道函数或类型名 | Ctrl+T 搜工作区符号，Ctrl+Shift+O 搜当前文件符号。在调用点按 F12 看定义，按 Shift+F12 看引用。符号索引尚未建好时用 Ctrl+Shift+F 搜完整名称。 |
| 知道文件名 | Ctrl+P 输入文件名，如 `arm_motion.cpp`。按 Alt+左方向键可回到上一个代码位置。 |

也可以在项目根目录的终端里搜索。第一条找阶段与入口，第二条用正则区分带括号的函数名。

```powershell
rg -n 'DISC_GRAB_MATERIALS|BuildArmProgram' src
rg -n 'SubmitMotion\(' src
```

## 按功能找入口

| 想找的功能 | 建议先打开 | 可以直接搜 |
| --- | --- | --- |
| 上电初始化、主循环 | [main.cpp](../src/main.cpp) | `setup`、`loop`、`Task_MainStateMachine` |
| 正式比赛各阶段、两轮取放和返家 | [competition_flow.h](../src/competition_flow.h) | `CompetitionFlow`、`DISC_GRAB_MATERIALS`、`COARSE_PLACE`、`TEMP_PLACE` |
| 串口命令接收和分发 | [serial_commands.inc](../src/serial_commands.inc) | `Task_Serial_CMD`、`serialDispatch`、`serialTransfer` |
| 运动作业如何排队、启动、结束 | [motion_executor.cpp](../src/motion_executor.cpp) | `SubmitMotion`、`LaunchMotion`、`GetMotionStatus` |
| 机械臂取放料顺序、夹爪和料盘时序 | [arm_motion.cpp](../src/arm_motion.cpp) | `BuildArmProgram`、`PICK_GRIP`、`PLACE_DESCEND` |
| 机械臂实际发令、到位和圆环修正 | [motion_hardware.cpp](../src/motion_hardware.cpp) | `MotionHardware::issue`、`MotionHardware::ready`、`MotionHardware::correctRing` |
| 正式节点导航与直行、转向 | [continuous_trial.cpp](../src/continuous_trial.cpp)、[node_position_control.cpp](../src/node_position_control.cpp) | `H30PrepareNodePath`、`Runner::tick`、`Runner::straightVelocityTick` |
| 独立连续路径和外绕曲线试跑 | [continuous_route.cpp](../src/continuous_route.cpp)、[hybrid_route.cpp](../src/hybrid_route.cpp) | `Route::build`、`Route::buildHybrid`、`Tracker::updateHybrid` |
| 扫码、任务码格式、扫码时移动 | [scanner.cpp](../src/scanner.cpp)、[task_code.h](../src/task_code.h)、[scan_motion_control.cpp](../src/scan_motion_control.cpp) | `Scanner_WaitCode`、`parseTaskCode`、`ScanMotionRunner::tick` |
| 视觉对齐启停、误差和 PID | [robot_runtime.cpp](../src/robot_runtime.cpp)、[chassis.cpp](../src/chassis.cpp) | `setAlignmentEnabled`、`TickVisualAlignment`、`AlignToVisionContinuous` |
| H30 航向读取与标定 | [h30_usb.cpp](../src/h30_usb.cpp) | `H30UsbGetHeadingSnapshot`、`H30UsbCalibrateHeading` |
| 张大头电机命令和轮子反馈 | [Emm_V5.cpp](../src/Emm_V5.cpp)、[node_position_hardware.cpp](../src/node_position_hardware.cpp) | `Emm_V5_Pos_Control`、`ReadMotorFeedback` |
| 启动面板、提示灯、电池告警 | [main.cpp](../src/main.cpp)、[start_panel.h](../src/start_panel.h)、[battery_monitor.cpp](../src/battery_monitor.cpp) | `StartPanelLightDuty`、`BatteryMonitorUpdate` |
| 参数默认值和串口调参 | [runtime_parameters.cpp](../src/runtime_parameters.cpp) | `ApplyCompiledParameterDefaults`、`HandleRuntimeParameters` |
| OTA 无线更新 | [ota_service.cpp](../src/ota_service.cpp) | `init_ota_service`、`ota_task` |

例如要改夹爪夹紧后保持多久，先搜 `PICK_GRIP` 找到 `arm_motion.cpp` 中的步骤，再搜该步使用的 `ARM_GRIP_HOLD_MS` 找到参数定义。要理解保持时间何时开始，继续看 [motion_sequence.h](../src/motion_sequence.h) 的 `Sequence::tick`，确认它何时记录 `readyAt`。

## 按文件查职责

`generated_parameter_defaults.h` 是构建时生成的默认参数表，生成入口见 [scripts/generate_parameter_defaults.py](../scripts/generate_parameter_defaults.py)，固件通过 `ApplyCompiledParameterDefaults` 应用该表。

| 文件 | 职责 |
| --- | --- |
| [alignment_start_policy.h](../src/alignment_start_policy.h) | 集中表达视觉对齐启动策略。修改启动条件时可先检查这里的纯判断，再追踪 robot_runtime 中负责切换会话的调用。 |
| [arm_axis_trial.cpp](../src/arm_axis_trial.cpp) | 单轴调试采样。试验入口建立本次动作与采样时间轴，轮询读取实际编码器位置、转速及状态，并在试验完成或取消时结束采样。 |
| [arm_axis_trial.h](../src/arm_axis_trial.h) | 单轴试验的启动与轮询接口。启动和获取终态分开，调用方可以用下一次采样延时安排任务唤醒。 |
| [arm_motion.cpp](../src/arm_motion.cpp) | 把机械臂请求编译成动作序列。这里组织伸缩、升降、转盘和夹爪的先后关系，具体下发与到位判断在 MotionHardware 中执行。 |
| [arm_motion.h](../src/arm_motion.h) | 机械臂动作编排入口。请求描述动作意图，Program 保存展开后的步骤，执行器随后按步骤推进。 |
| [arm_placement_log.h](../src/arm_placement_log.h) | 放料日志格式的公共辅助代码。位置字段和动作标识用于关联放料过程，修改输出前应同时查看串口日志消费端。 |
| [battery_monitor.cpp](../src/battery_monitor.cpp) | 周期更新电池电压并管理蜂鸣器输出。提示音与低电压告警共用输出，修改其中一路时应同时查看 updateBuzzer 如何合并状态。 |
| [battery_monitor.h](../src/battery_monitor.h) | 电池监测对外接口。电压读数和告警阈值以毫伏为单位，更新由调用方周期触发。 |
| [chassis.cpp](../src/chassis.cpp) | 底盘与机械臂的运动下发层。包括车体速度换算、轮电机位置动作及视觉对齐 PID。调试时先区分闭环误差、车体位移和驱动器脉冲三个层次。 |
| [chassis.h](../src/chassis.h) | 集中声明底盘、机械臂的标定常量与运动接口。修改轮径、轴比例或速度参数时，需沿调用点核对毫米、角度、转速和脉冲之间的换算。 |
| [competition_flow.h](../src/competition_flow.h) | 比赛阶段编排。Phase 保存内部细分阶段，RobotState 对外报告进度。流程通过接口提交运动并轮询结果，物理时序由运动执行层管理。 |
| [continuous_route.cpp](../src/continuous_route.cpp) | 根据节点和圆角构造连续路径并计算跟踪指令。用沿程距离查找路径位置、切向和航向，用速度曲线协调弯角与停车点。 |
| [continuous_route.h](../src/continuous_route.h) | 连续路径规划与跟踪使用的数据结构。节点是输入，段保存几何结果，跟踪器记录当前进度；排查问题时可按这三层定位。 |
| [continuous_trial.cpp](../src/continuous_trial.cpp) | 连续导航试验及节点路径执行入口。串口上传、路径准备、执行和结束分别维护状态；底盘轮询推进时可以同时检查停止请求和航向数据。 |
| [continuous_trial.h](../src/continuous_trial.h) | 公开路径试验和连续运动的分阶段接口。节点上传、准备、启动和查询各有入口，调用端用试验编号关联操作。 |
| [debug_release_button.h](../src/debug_release_button.h) | Debug 模式释放按键的输入状态判断。将按键采样与模式切换请求拆开，实际切换在运行层统一处理。 |
| [disc_material_decision.h](../src/disc_material_decision.h) | 根据当前取料阶段、颜色和货位状态决定能否抓取。这里保留判断结果供串口层回传，机械臂动作由后续作业执行。 |
| [disc_position_grab.cpp](../src/disc_position_grab.cpp) | 圆盘三个抓取入口共用同一个抓放过程，并分别传入物理位置编号。颜色和货位随调用传递，动作实现见物料转运模块。 |
| [disc_position_grab.h](../src/disc_position_grab.h) | 按圆盘物理位号抓料的接口。阅读调用时需区分抓取点位置、物料颜色和目标货位。 |
| [emm_drive_config.h](../src/emm_drive_config.h) | 描述张大头驱动器参数配置所用的协议字段。参数在串口设置、驱动器写入和回读验证之间使用相同顺序。 |
| [emm_feedback.h](../src/emm_feedback.h) | 电机反馈字节解析与编码器位置换算。调用者拿到的状态用于运动反馈；查看位移时要区分原始编码器值与累计位置。 |
| [Emm_V5.cpp](../src/Emm_V5.cpp) | 张大头驱动器的串口封包层。每个函数按照命令号填充字节数组，多字节参数逐字节拆分，最后写入电机串口。同步标志决定命令是立即执行还是等待广播触发。 |
| [Emm_V5.h](../src/Emm_V5.h) | 张大头电机命令声明与系统参数查询类型。调用位置模式接口时，要同时考虑方向、驱动器脉冲数、绝对或相对模式以及同步执行标志。 |
| [field_geometry.def](../src/field_geometry.def) | 地图几何的宏展开输入。宏参数以毫米为单位，描述场地、车体和障碍矩形。固件头文件与 Python 预览共同读取这些条目。 |
| [field_geometry.h](../src/field_geometry.h) | 地图坐标和线路几何的共享定义。节点路线使用同一套几何，修改坐标时应核对路线规划和上位机下发所依据的版本。 |
| [h30_heading.h](../src/h30_heading.h) | 将 H30 姿态航向绑定到地图航向，并提供跨角度边界的处理。读取角度时应区分传感器参考系与已校准的地图参考系。 |
| [h30_motion.h](../src/h30_motion.h) | 为运动控制整理 H30 样本中的姿态与运动信息。路径控制使用样本时间及角度变化判断是否可以推进当前阶段。 |
| [h30_protocol.h](../src/h30_protocol.h) | H30 串行字节流的帧解析。USB 层输入收到的字节，解析结果供上层构造姿态样本。 |
| [h30_usb.cpp](../src/h30_usb.cpp) | H30 USB Host 适配层。设备事件管理连接状态，接收回调更新姿态数据，上层通过快照读取角度及采样时间。设备失效和航向校准各有独立入口。 |
| [h30_usb.h](../src/h30_usb.h) | H30 USB 的采样与地图航向接口。样本年龄、样本时间和参考版本用于不同的读取场景，调用时应区分其含义。 |
| [hybrid_route.cpp](../src/hybrid_route.cpp) | 混合路径把平移、外绕曲线和原地转向组织成阶段。曲线的几何采样、航向插值和跟踪阶段切换分别处理。 |
| [led_pwm.cpp](../src/led_pwm.cpp) | PWM 灯光输出，同时管理手动亮度、视觉补光及提示闪烁。调试亮度时要核对上层状态和当前时间是否正在覆盖基础亮度。 |
| [led_pwm.h](../src/led_pwm.h) | 灯光通道公开接口。亮度和 PWM 频率用于配置输出，故障闪烁和动作提示通过不同入口更新。 |
| [main.cpp](../src/main.cpp) | 固件装配入口。setup 初始化总线、硬件与任务，比赛适配层把流程请求交给运行动作接口，loop 维护需要周期刷新的面板与提示状态。 |
| [material_transfer.cpp](../src/material_transfer.cpp) | 定义物料位姿、动作参数和逐件或整轮搬运入口。调试物料顺序时分别追踪圆盘位、颜色、货位和工作区位置，四者在任务中承担不同角色。 |
| [material_transfer.h](../src/material_transfer.h) | 物料位置、位姿、速度与等待时间的共享定义。传统搬运入口和新动作编排共用这些数据，定位参数来源时可与 runtime_parameters 对照。 |
| [motion_executor.cpp](../src/motion_executor.cpp) | 运动作业的调度任务。提交方写入请求，执行器依动作类型推进子状态并发布完成或阶段事件；作业编号用于关联提交、接续和结果。 |
| [motion_executor.h](../src/motion_executor.h) | 运动执行器的提交、启动、查询及取消接口。允许先排队再启动，串口层可先应答并发送作业编号。 |
| [motion_hardware.cpp](../src/motion_hardware.cpp) | 将动作步骤映射到物理设备。issue 发出命令，ready 轮询动作条件，event 记录时间线。圆环对齐需要关联当前视觉会话与步骤状态。 |
| [motion_hardware.h](../src/motion_hardware.h) | 动作步骤执行用的硬件适配接口。动作序列通过此层获得时间、取消状态、到位反馈和步骤事件。 |
| [motion_job.h](../src/motion_job.h) | 维护作业启动、取消与接续所需的状态。作业编号及代际信息用于关联当前请求与异步控制信号。 |
| [motion_sequence.h](../src/motion_sequence.h) | 动作步骤及其轮询执行规则。步骤数据保存要下发的目标和等待条件，轮询状态保存正在执行的步骤。 |
| [motion_types.h](../src/motion_types.h) | 定义调度器与请求端共享的运动类型。请求描述要做什么，状态对象用于追踪作业编号、阶段和终态。 |
| [motor_bus.cpp](../src/motor_bus.cpp) | 通过递归互斥锁串行化电机串口访问。守卫持有期间可调用内部同样使用守卫的函数，析构时释放本次持锁。 |
| [motor_bus.h](../src/motor_bus.h) | 声明电机串口的作用域锁。读取反馈时，应把发出查询和读取对应应答作为一段总线操作来阅读。 |
| [node_position_control.cpp](../src/node_position_control.cpp) | 位置路线状态机。依次处理起步静止、阶段选择、校角和轮位移，直线速度阶段还将四轮编码器变化分解为车体位移。 |
| [node_position_control.h](../src/node_position_control.h) | 节点位置控制所需的硬件抽象、配置及运行数据。控制算法只通过 IO 参数读反馈和发运动指令，硬件实现另见 node_position_hardware。 |
| [node_position_hardware.cpp](../src/node_position_hardware.cpp) | 节点运动的串口和传感器适配。把状态机的读写转换为电机命令，并提供圆盘基准编码器记录和回程位移计算。 |
| [node_position_hardware.h](../src/node_position_hardware.h) | 公开节点路线、扫码运动及电机反馈接口。使用圆盘返回相关函数时，需要保留进入圆盘区时记录的编码器与航向基准。 |
| [node_route_heading.h](../src/node_route_heading.h) | 节点线路的航向选择辅助逻辑。路径的平移方向和车体朝向分别计算，查看终点行为时应核对调用方传入的目标航向。 |
| [ota_service.cpp](../src/ota_service.cpp) | OTA 独立任务负责网络接入和更新回调。开始和结束事件会通知运动执行层，维护任务自身由 FreeRTOS 调度。 |
| [ota_service.h](../src/ota_service.h) | OTA 启动入口。调用者提供网络和主机名，具体连接及维护循环由实现中的任务负责。 |
| [position_move_timing.h](../src/position_move_timing.h) | 位置动作的时间估算辅助代码。这里计算轮询调度所需的时间，查到位判据还需查看调用处的反馈逻辑。 |
| [ring_correction.h](../src/ring_correction.h) | 圆环视觉修正所用的估算和数据处理辅助代码。分析放料偏差时应区分视觉误差和机械臂需要执行的修正量。 |
| [robot_runtime.cpp](../src/robot_runtime.cpp) | 运行期共享业务状态及视觉会话控制。串口入口在文件末尾文本包含，沿用本文件内的状态和函数；视觉启停与反馈处理应结合阅读。 |
| [robot_runtime.h](../src/robot_runtime.h) | 比赛流程、串口任务与视觉运行层使用的声明。查看同一状态由谁更新时，应从这里找到声明，再追踪定义和写入点。 |
| [robot_state.h](../src/robot_state.h) | 比赛对外状态定义。状态用于日志和上位机进度，内部每个状态可对应 competition_flow 中的多个子阶段。 |
| [runtime_parameters.cpp](../src/runtime_parameters.cpp) | 把参数名绑定到固件实际变量，应用编译默认值并处理串口读写。调参时先核对注册的数据类型、范围和对象地址。 |
| [runtime_parameters.h](../src/runtime_parameters.h) | 运行参数注册及串口访问入口。注册项直接引用业务变量，参数可写与否由调用方传入。 |
| [scan_motion_control.cpp](../src/scan_motion_control.cpp) | 扫码接近和搜索动作的轮询控制。以运动反馈和收到任务码的回调决定当前阶段何时结束，并记录前向位移。 |
| [scan_motion_control.h](../src/scan_motion_control.h) | 扫码动作参数及运行状态。设置保存接近、搜索及速度参数，运行器与 IO 和扫码回调配合推进。 |
| [scanner.cpp](../src/scanner.cpp) | 扫码串口的接收任务。监听端组装任务码并交给等待端，业务任务通过队列接口取得扫码结果。 |
| [scanner.h](../src/scanner.h) | 扫码模块的串口初始化和等待接口。等待端提供输出缓冲区和超时，收到的内容再由任务码解析层解释。 |
| [serial_commands.inc](../src/serial_commands.inc) | 串口命令分发的文本包含文件。任务负责读 Serial0，分发层按命令组和动作处理，异步运动用编号和后续事件表示实际结果。 |
| [serial_frame.h](../src/serial_frame.h) | 串口帧的结构和基础解析。分隔字段与数值转换在进业务分发前后分层处理，便于定位收帧问题和命令参数问题。 |
| [servo.cpp](../src/servo.cpp) | 总线舵机协议实现。收发封包共用串口，具体命令分别编码普通角度、多圈角度、阻尼或参数操作。 |
| [servo.h](../src/servo.h) | 总线舵机地址、协议常量及接口。普通与多圈角度接口的参数长度和编码方式不同，更改目标时需查看对应调用。 |
| [start_panel.h](../src/start_panel.h) | 启动面板引脚、任务活动判据及指示灯占空比计算。起始区和启动请求的采样处理见 serial_commands.inc。 |
| [task_code.h](../src/task_code.h) | 任务码解析与数据组织。任务中同时包含颜色和位置序列，使用时按轮次和用途选取对应字段。 |
| [turret_motor.cpp](../src/turret_motor.cpp) | 机械臂转盘步进电机的角度与脉冲适配。零点、编码器反馈、角度量化和试验采样在此统一管理。 |
| [turret_motor.h](../src/turret_motor.h) | 机械臂转盘反馈、移动和试验接口。转盘角度和驱动器脉冲位置是不同表示，量化接口负责两者转换。 |
| [turret_path.h](../src/turret_path.h) | 在可选的等效角度中规划机械臂转盘路径。检查规划输出时应结合当前角度及连续多个目标，不只看单次目标。 |
| [vision_alignment.h](../src/vision_alignment.h) | 视觉对齐辅助逻辑及输入数据。上层按视觉工作模式选择对齐行为，底盘闭环以收到的误差计算控制输出。 |
