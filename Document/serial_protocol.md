# ESP32 Serial0 串口协议 v2

本文是机载电脑、仓库上位机和 ESP32 之间的**唯一机器指令规范**。固件不接收旧版 `{way:...}` 等指令。仓库外发送端必须先按下表迁移，再切换固件。舵机 Serial2、扫码软串口不属于本协议。

## 1. 传输与帧结构

- 115200 bit/s，8N1。每行恰好一帧，发送端以 `\n` 结尾；固件兼容 `\r\n`。
- 发送给 ESP32：`{CMD,类别,动作,参数...}`。ESP32 的命令回复：`{RSP,类别,动作,状态,参数...}`。ESP32 主动发出的状态：`{EVT,类别,事件,参数...}`。
- 固定字和输入参数只用可打印 ASCII，区分大小写；不得包含空字段、空格、逗号、花括号或换行。参数目录的**返回名称**与显示事件的**正文**可以是 UTF-8 中文。显示正文可以有空格，但不能含逗号、花括号或控制字符。
- 输入帧最多 **159 字节**（不含换行），最多 32 个逗号字段。超长行会被整行丢弃并回复 `{RSP,SYS,FRAME,ERR,TOO_LONG}`；结构错误回复 `{RSP,SYS,FRAME,ERR,FORMAT}`。固件不会执行截断或格式错误的指令。
- 数字用十进制，必须完整且有限；`NaN`、`inf`、缺字段、多字段均无效。`ACK` 表示已接令，**不代表物理到位**；`ROUTE_DONE,ESTIMATED` 表示有限位置动作的四轮位置/转速/到位反馈及稳定确认通过，H30可辅助但非必需，仍不代表世界X/Y实测到位或无打滑。
- 上位机连接后先发 `{CMD,SYS,HELLO}`，仅收到 `{RSP,SYS,HELLO,OK,2}` 才允许控制。超时或版本不符时断开连接。启动日志如 `version:`、`Debug mode` 只供人阅读，不参与机器帧解析。

## 2. 命令总表与旧版迁移对照

Debug空闲时按BOOT0（GPIO0，低电平）等效执行 `{CMD,SYS,RELEASE}`，使用相同回复及自动开局流程；40 ms消抖，长按只触发一次，上电按住需先松开。串口动作执行或半帧接收期间按下不会排队，Release中不重复创建自动任务。切换后仍需选区及发送SYS START。

下表中的参数顺序就是实际发送顺序。`两者` 指 Debug 与 Release；`Debug` 指上电调试模式。错误通常为 `{RSP,类别,动作,ERR,原因}`。下表未标为 Release 的 Debug 动作在 Release 中回复 `MODE`，不会执行。

| 旧指令 | v2 指令 | 模式 | 成功回复 / 后续事件 |
|---|---|---|---|
| 无 | `{CMD,SYS,HELLO}` | 两者 | `{RSP,SYS,HELLO,OK,2}` |
| 无 | `{CMD,LED,SET,percent}` | 两者 | `{RSP,LED,SET,OK,percent,hz}`；亮度为 0～100 的整数 |
| 无 | `{CMD,LED,FREQ,hz}` | 两者 | `{RSP,LED,FREQ,OK,percent,hz}`；频率为 100～9000 Hz 的整数，保持亮度 |
| 无 | `{CMD,LED,GET}` | 两者 | `{RSP,LED,GET,OK,percent,hz}`；读取固件 PWM 设置值；故障闪烁期间 percent 为恢复用的设定亮度，实际输出在 100%/0% 间每 500 ms 切换 |
| `{Mode:Release}` | `{CMD,SYS,RELEASE}` | Debug | `{RSP,SYS,RELEASE,OK}`；失败 `TIMER` / `TASK` |
| `{ready}` | `{CMD,SYS,READY}` | Release | `{RSP,SYS,READY,OK}` |
| `{start}` | `{CMD,SYS,START}` | Release | `{RSP,SYS,START,OK}`；上位机“姿态 → 启动位置 → 雷达扫描完成（继续运行）”可代发，解除 `WAIT START` 等待并进入扫码行驶 |
| `{help}` | `{CMD,SYS,HELP}` | 两者 | `{RSP,SYS,HELP,OK}`；帮助文字参见本文件 |
| `{StartZone:1}` / `:2` | `{CMD,NAV,START_ZONE,1}` / `2` | 两者 | `{RSP,NAV,START_ZONE,OK,1}` / `2` |
| `{way:0-1-8}` | `{CMD,NAV,ROUTE,0,1,8}` | 两者 | `{RSP,NAV,ROUTE,ACK,3}`，随后路径事件 |
| 无 | `{CMD,NAV,STOP}` | 两者 | `{RSP,NAV,STOP,ACK}`，请求取消当前节点导航/试跑，等待RESULT或终态确认停车 |
| 无 | `{CMD,VISION,START_REQUEST,mode}` | 两者 | `{RSP,VISION,START_REQUEST,ACK,mode}`，随后 `{EVT,VISION,START_REQUEST,mode}` |
| 无 | `{CMD,VISION,STOP_REQUEST,mode}` | 两者 | `{RSP,VISION,STOP_REQUEST,ACK,mode}`，随后 `{EVT,VISION,STOP_REQUEST,mode}` |
| `{ALIGN:START}` | `{CMD,VISION,ALIGN_START}` | 两者 | `{RSP,VISION,ALIGN_START,OK}` |
| 无 | `{CMD,VISION,ALIGN_START,模式}` | 两者 | `{RSP,VISION,ALIGN_START,OK}`，按指定模式死区连续对齐 |
| `{ALIGN:STOP}` | `{CMD,VISION,ALIGN_STOP}` | 两者 | `{RSP,VISION,ALIGN_STOP,OK}` |
| `{ALIGN:angle,x,y}` | `{CMD,VISION,ALIGN_DATA,angle,x,y}` | 两者 | 正常连续反馈不逐帧回复 |
| `{color:N}` | `{CMD,VISION,COLOR,N}` | 两者，须开启 DISC_MATERIAL | `{RSP,VISION,COLOR,OK}`，随后报告 GRAB/SKIP/BLOCKED 决策 |
| 无 | `{CMD,VISION,DISC_POSITIONS,pos1,color1[,pos2,color2[,pos3,color3]]}` | 两者，须开启 DISC_MATERIAL | ACK 后报告 SELECTED、DONE/FAILED 或 SKIP；按位置抓取新入口，详见文末使用说明 |
| 无 | `{CMD,VISION,GRAB,cargo}` | 两者，须开启 DISC_MATERIAL | 使用最近 COLOR，回复 ACK 后发送 DONE/FAILED（动作结果） |
| 无 | `{CMD,VISION,GRAB,color,cargo}` | 两者，须开启 DISC_MATERIAL | 同帧指定颜色与载物台，回复 ACK 后发送 DONE/FAILED（动作结果） |
| `{ok}` | `{CMD,VISION,CONFIRM}` | Release | `{RSP,VISION,CONFIRM,ACK}` |
| `{LidarPose}` | `{CMD,VISION,LIDAR_POSE}` | 两者 | 首次执行返回 `{RSP,VISION,LIDAR_POSE,ACK}`，随后完成或失败事件；执行中返回 `ERR,BUSY`；Release 成功后的重复请求返回 `OK` 和 `LIDAR_POSE_DONE`，不重复移动；Debug 允许重新执行 |
| `{POSE:GET}` | `{CMD,POSE,GET}` | 两者 | `{RSP,POSE,GET,OK,x,y,theta,h,l,turret,pawl}` |
| 无 | `{CMD,IMU,GET}` | 两者 | 单次快照：`{RSP,IMU,GET,OK,pitch,roll,yaw,age_ms}`；角度单位 °，H30 原始坐标，数据年龄不超过 200 ms |
| 无 | `{CMD,IMU,HEADING}` | 两者 | 单次地图航向：`{RSP,IMU,HEADING,OK,heading,age_ms}`；heading 为 0~360°（不含 360），无参考时返回 `ERR,NOT_CALIBRATED` |
| 无 | `{CMD,NAV,TRIAL_BEGIN,...}` / `TRIAL_POLICY` / `TRIAL_NODE` / `TRIAL_START` / `TRIAL_STATUS` / `TRIAL_STOP` | Debug | 独立混合路径试跑：窄路停转、外围外绕、完整车身碰障碍校验、H30转向完成确认。POLICY及几何签名必需；完整参数和回复见 [连续试跑说明](continuous_trial.md) |
| `{SetPose:x,y,theta}` | `{CMD,POSE,SET,x,y,theta}` | Debug | `{RSP,POSE,SET,ACK,x,y,theta}` |
| `{GOTOpose:x,y,theta}` | `{CMD,POSE,GOTO_REL,x,y,theta}` | Debug | `{RSP,POSE,GOTO_REL,ACK,x,y,theta}` |
| `{Movepose:dir,speed,stop}` | `{CMD,CHASSIS,MOVE,dir,speed,stop}` | Debug | `{RSP,CHASSIS,MOVE,ACK,dir,speed,stop}` |
| 无 | `{CMD,CHASSIS,MOVE_POSITION,x,y,theta,speed}` | Debug | `{RSP,CHASSIS,MOVE_POSITION,ACK,x,y,theta,speed}`，随后 `{EVT,CHASSIS,MOVE_POSITION,DONE,ESTIMATED}` 或 `{EVT,CHASSIS,MOVE_POSITION,FAILED}` |
| `{MoveArm_1:h,l,speed}` | `{CMD,ARM,MOVE1,h,l,speed}` | Debug | `{RSP,ARM,MOVE1,ACK,h,l,speed}`，随后 `{EVT,ARM,MOVE1,ISSUED}` 或 `FAILED` |
| 无（新增） | `{CMD,ARM,INIT_START}` | Debug | 调用 `InitArm_start()`；`{RSP,ARM,INIT_START,ACK}`，随后 `{EVT,ARM,INIT_START,ISSUED}` 或 `FAILED` |
| 无（新增） | `{CMD,ARM,INIT_LOOK}` | Debug | 调用 `InitArm_look()`；`{RSP,ARM,INIT_LOOK,ACK}`，随后 `{EVT,ARM,INIT_LOOK,ISSUED}` 或 `FAILED` |
| 无（新增） | `{CMD,ARM,INIT_LOOK2}` | Debug | 调用 `InitArm_look2()`；`{RSP,ARM,INIT_LOOK2,ACK}`，随后 `{EVT,ARM,INIT_LOOK2,ISSUED}` 或 `FAILED` |
| `{MoveArm_2:turret,pawl,speed}` | `{CMD,ARM,MOVE2,turret,pawl,speed}` | Debug | `{RSP,ARM,MOVE2,ACK,turret,pawl,speed}`，随后 `{EVT,ARM,MOVE2,ISSUED}` 或 `FAILED`；`ISSUED` 只表示指令已下发 |
| `{SERVO:id,angle}` | `{CMD,ARM,SERVO,id,angle}` | Debug | `{RSP,ARM,SERVO,ACK,id,angle}` |
| `{En_C:enable}` | `{CMD,ARM,ENABLE,enable}` | Debug | `{RSP,ARM,ENABLE,ACK,enable}` |
| `{MaterialDemo}` / `2` / `3` | `{CMD,ARM,DEMO1}` / `2` / `3` | Debug | `{RSP,ARM,DEMO1,ACK}`，随后 `{EVT,ARM,DEMO1,DONE}` 或 `FAILED`；其他编号同理 |
| `{MaterialDemo4}` | `{CMD,ARM,DEMO4}` | Debug | `{RSP,ARM,DEMO4,ACK}`，随后 `{EVT,ARM,DEMO4,DONE}` 或 `FAILED` |

### 参数与限制

手动强制抓取：`{CMD,VISION,FORCE_GRAB,color,cargo}`，color=1～6、cargo=1～3，Debug/Release 均可用。无需开启 DISC_MATERIAL、任务码或任务顺序；先取消业务对齐等待，向统一运动任务提交抓放。接令返回 `{RSP,VISION,FORCE_GRAB,ACK,color,cargo}` 与 `{EVT,VISION,FORCE_GRAB,REQUESTED,color,cargo}`，二者只表示提交。动作结束后发送 `{EVT,VISION,FORCE_GRAB,DONE,color,cargo}` 或 `FAILED`；上位机等待匹配颜色/载物台的终态。目标载物台已有料时仍发送 `{EVT,VISION,FORCE_GRAB,WARN,OCCUPIED,原颜色,cargo}` 并继续；原有动作边界和标定继续生效。成功末端更新载物台记录，提前失败保留原记录，不推进自动任务进度。错误参数 RANGE、参数数量错误 FORMAT。

上位机“视觉 → 手动输入任务码”输入 `156+123+516+231` 后点击下发或按回车，发送 `{CMD,TASK,SET,156+123+516+231}`。固件回传 `{RSP,TASK,SET,OK,任务码}` 并发送 TASK_CODE 显示事件；界面仅在确认后更新当前任务码。四组都须恰好三位，第一/三组颜色为 1～6，第二/四组各为 1、2、3 的排列；格式或数值错误回复 RANGE，参数数量错误回复 FORMAT，不覆盖已有任务码。

TASK SET 在 Debug 可修改任务码并重置抓取进度为 0；开启 DISC_MATERIAL 且停止连续对齐后，COLOR 按第一轮任务顺序判断，并在目标载物台为空时调用现有抓取动作。下发任务码不会清空载物台记录。Release 只允许 WAIT_START 预置或 READ_TASK 尚未收到任务时输入；进入抓取/放置等执行阶段回复 BUSY。手动预置的任务码在进入 READ_TASK 时直接使用，扫码等待期间收到手动码也会结束扫码并停车；迟到的扫码结果不能覆盖已接受的手动码。任务码仅在 RAM 中保存，重启清空。手动码不确认车辆位置，后续导航仍要求车辆实际位于路径起点。

五种视觉模式各有独立的 X、Y、角度死区。默认每种模式均为 X=3、Y=3（视觉单位）、角度=0.2°，可在停止对齐后通过上位机“参数”页分别修改 `对齐/模式/X死区`、`Y死区`、`角度死区(deg)`；参数只在 RAM 生效，重启恢复默认值，修改目录后须重新 GET 参数 ID。

`WaitForAlignment(mode, angle, x, y, dt, speed)` 是非阻塞逐帧函数：使用该模式死区更新 PID，三个偏差绝对值同时 **小于或等于** 各自死区才累计一帧，连续第 5 帧返回 `true` 并报告 `ALIGN_DONE`。任一轴越界、无效反馈、启停、模式切换、300 ms 断流或最新帧队列覆盖导致的丢帧均清零计数。第 1～4 帧全部在死区时已停车，但尚未宣布完成；没有新帧时不会增加计数。第 5 帧锁存达标状态供业务流程读取，但连续对齐仍开启，后续误差越界会继续修正，直到显式停止。帧数依据接收的有效反馈，协议没有相机帧号，发送端须保证每个相机帧只发送一次。

手动连续对齐可发送 `{CMD,VISION,ALIGN_START,模式}` 指定上述五种模式之一；无自动对齐时，无模式的旧命令和 Debug 上电默认使用 `DISC` 死区；自动对齐期间，无模式命令沿用当前自动模式。未知模式回复 `RANGE`，多余参数回复 `FORMAT`。此命令只开启底盘闭环，相机识别仍通过 `START_REQUEST` 控制。

`START_REQUEST,WORK_AREA` 和 `START_REQUEST,WORK_AREA_LOADED` 在串口 ACK 后由统一运动任务执行 look；成功才发 START_REQUEST 并开始 PID/对齐计时，失败发送 `{EVT,VISION,START_FAILED,模式,ARM}`。自动流程将路径与 look 作为联合完成条件，确认二者完成后才启动视觉；粗加工放料后直接取回，不再启动带物料定位。ACK 不代表姿态或视觉已就绪。

键盘微调：Shift+Q 左转、Shift+E 右转，按住启动，松开 Q/E 或 Shift 停止。MOVE 的 dir 支持 0=前进、1=后退、2=左移、3=右移、4=左转、5=右转；speed 为电机 RPM，stop=0 启动、stop=1 停止。单独 Q/E 仍执行 ±90° 相对旋转。

- `DEMO4`：小车停稳在暂存区、区域 1～3 号位各有一件物料且三个载物台为空时使用。按内侧到外侧依次完整执行“暂存区 1 → 载物台 3、2 → 2、3 → 1”。固件在运动前检查三个载物台状态和位姿；物料颜色临时按红色登记，动作不含底盘导航，也不能检测实际有无物料。`DONE` 表示预设动作序列执行结束，不代表传感器确认抓取成功。

- `START_ZONE` 只能为 1 或 2。Debug 中设置后同步软件估计位姿；Release 中保留现有业务状态行为。
- `ROUTE`：2～25 个节点，节点编号 0～24，5×5 蛇形地图中相邻节点必须物理上下或左右相邻。路径的第一个节点必须是小车实际所在节点。固件执行前再检查相邻性；上位机输入框仍可输入 `0-1-8`，发送时转换为逗号字段。
- `ALIGN_DATA`：三个有限浮点值，依次为角度偏差（度）、视觉 X 与 Y 偏差（视觉单位）；20 Hz 连续输入时只保留最新一帧。停止对齐后收到有效反馈也不驱动电机，并最多每秒回传一次 `{EVT,VISION,ALIGN_IGNORED,DISABLED}`；反馈不会自行重启运动。
- `DISC` 圆盘整体对齐的角度环改用 H30 已标定地图航向，目标为 `DISC_AREA_HEADING=90°`（与导航终点一致），以环绕到 ±180° 的“实测航向−目标航向”驱动现有角度 PID；连续 5 帧到位判定也使用该 H30 角差和原 DISC 死区。视觉 angle 字段仍须为有限数，但不参与 DISC 控制或到位判断；X/Y、PID增益、最大轮速和控制节奏保持原设置。其他模式仍使用视觉角差。H30 查询失败（含未标定或样本超过200ms）时，该帧旋转输出为0并清空角度PID历史，X/Y继续对齐且清零连续到位计数，不退回视觉角度；数据恢复后自动继续。异常首次输出 `{EVT,VISION,DISC_HEADING,ERR,原因}`，恢复输出 `{EVT,VISION,DISC_HEADING,OK}`。`[Align]` 日志中 angle 为实际参与控制的角差，尾部 `source=IMU/IMU_UNAVAILABLE/VISION` 表示来源；原有自动对齐超时规则保留。
- 首轮圆盘自动流程：路径结束后固件开启连续对齐，再发 `START_REQUEST,DISC`；电脑仅需回传 `ALIGN_DATA`。连续 5 帧达标时发 `ALIGN_DONE`，主流程随后发 `STOP_REQUEST,DISC` 停止对齐，再发 `START_REQUEST,DISC_MATERIAL`。包含首帧等待的首次达标限时为 15 秒，300 ms 断流停车/失败分支暂时注释，断流期间保持最后一次轮速；等待失败发 `ALIGN_FAILED,TIMEOUT`，主流程结束 DISC 请求并保持故障。自动对齐期间，同模式或无模式 `ALIGN_START` 回复 OK 并保留当前模式、WAITING/DONE、反馈、PID、连续帧计数及原超时起点；不同模式回复 `{RSP,VISION,ALIGN_START,ERR,BUSY}`，不改变当前对齐。FAILED 尚未被主状态机处理时也拒绝启动，避免覆盖失败结果。`ALIGN_STOP` 仍会停车并取消业务等待，不能作为自动完成信号。
- `COLOR`：整数 1～6，依次为红、黄、蓝、绿、黑、浅蓝。DISC_MATERIAL 运行时，每条颜色反馈按当前轮次任务码判断：第一轮使用 round1_colors，第二轮使用 round2_colors，只与下标 roundProgress 对应的下一件颜色比较。匹配且当前抓取阶段已准备、底盘对齐停止、目标载物台为空时，发送 `{EVT,VISION,COLOR,GRAB,color,cargo}` 并调用 `GrabDiscMaterial(color,cargo)`。仅颜色不匹配时发送 `{EVT,VISION,COLOR,SKIP,color,cargo,COLOR_MISMATCH}`；状态阻止时发送 `{EVT,VISION,COLOR,BLOCKED,color,cargo,reason}`，reason 为 NO_TASK、NOT_WAITING、ROUND_COMPLETE、ALIGN_ACTIVE、NOT_READY 或 OCCUPIED。cargo 为 roundProgress+1，无有效进度时为 0；有有效任务和进度时，拒绝日志还打印 received、expected、cargo、reason。上位机兼容旧固件六字段 SKIP，但会提示旧回复未提供原因。`CONFIRM` 仍仅 Release 接令入队，不执行抓取。
- `GRAB`：保留显式任务校验抓取入口。cargo 为 1～3；单参数使用最近 COLOR，双参数为 color,cargo。须开启物料视觉、停止底盘对齐、进入有效任务码的抓取阶段，颜色及载物台顺序须匹配，载物台须为空。拒绝原因保留 NOT_ACTIVE、ALIGN_ACTIVE、NO_COLOR、NOT_WAITING、ORDER、COLOR_MISMATCH、OCCUPIED；非法参数为 RANGE，参数数量错误为 FORMAT。Debug 不绕过任务码校验。
- Release 首轮还要求主流程已完成圆盘导航及定位，再进入物料识别等待；提前手动开启 DISC_MATERIAL 不绕过准备条件。COLOR 此时报告 BLOCKED,NOT_READY，显式 GRAB 回复 NOT_WAITING。Debug 手动测试不要求该准备状态，但仍检查对齐停止和载物台为空。
- 有效 GRAB 先回复 `{RSP,VISION,GRAB,ACK,color,cargo}`；COLOR 匹配先发送 COLOR,GRAB 决策。两者共用抓取完成路径：`GrabDiscMaterial` 返回 true 后发送 `{EVT,VISION,GRAB,DONE,color,cargo}` 并推进 roundProgress；返回 false 后发送 FAILED，保留进度和目标颜色。true 表示预设动作完成且已登记载物台颜色，不代表传感器确认实物。第三件成功后关闭 DISC_MATERIAL；Release 主流程进入下一放料阶段，Debug 保留完成进度。上位机兼容旧 REQUESTED 回复，但它不表示完成。强制入口不推进任务进度。
- `VISION START_REQUEST`：恰好一个模式参数，只允许 `DISC`、`DISC_MATERIAL`、`WORK_AREA`、`WORK_AREA_LOADED`、`CORNER`。未知模式回复 `RANGE`，缺参数或多参数回复 `FORMAT`，均不发送视觉请求事件。Debug 和 Release 均可调用。`DISC` / `WORK_AREA` / `WORK_AREA_LOADED` / `CORNER` 在发送启动事件前清空旧反馈并同步开启连续 PID 对齐，电脑只需发送 `ALIGN_DATA`；首次连续 5 帧达标限时 15 秒，300 ms 断流停车/失败分支暂时注释，断流期间保持最后一次轮速，达标后连续闭环仍开启。`DISC_MATERIAL` 仅发送视觉请求。重复开启定位会重置等待计数；手动请求不主动切换业务阶段。
- `VISION STOP_REQUEST`：模式、参数数量、运行模式及错误规则与 `START_REQUEST` 相同。先回复 ACK，再同步停止属于 `DISC` / `WORK_AREA` / `WORK_AREA_LOADED` / `CORNER` 对应模式的底盘对齐，清零 PID 和等待计数，再发出 STOP_REQUEST 事件。结束旧模式不会停止随后开启的另一定位模式。`DISC_MATERIAL` 仅请求结束视觉。无模式 `ALIGN_START` 的连续对齐用 `{CMD,VISION,ALIGN_STOP}` 停车；显式指定上述四种模式的手动连续对齐也可通过对应 STOP_REQUEST 停止。
- `POSE GET`：`x,y,theta,h,l` 是软件维护的理想/开环值，单位依次为 mm、mm、度、mm、mm；`turret` 是 2 号转台舵机现场读取的真实多圈角度，`pawl` 是 1 号夹爪舵机现场读取的单圈角度（度）。读取失败回复 `{RSP,POSE,GET,ERR,SERVO_READ,2}` 或 `1`，不会用旧值冒充实测。
- `ARM MOVE2` 的转台线缆机械范围为真实多圈角度 **-180°～360°**。固件先读取真实多圈角度；若当前已越界或读取失败，跳过转台动作。否则仅从范围内的等效目标（相差 360°）中选择最近的一项，绝不向范围外下发转台目标。转台输入也限制为 -180°～360°；夹爪仍为 -360°～360°。搬运 Demo 遇到转台失败会停止并报告 `FAILED`，失败后需人工确认物料位置。调试 `SERVO` 指令的 2 号舵机走相同保护，成功时回复 `{EVT,ARM,SERVO,ISSUED}`；不允许广播 ID 254 绕过保护。
- 转台开机参考位由使用者保证：**上电前必须实际处于 0°～180°且线缆未缠绕**。固件等待舵机启动，并对开机读数及重置后回读做有限重试。若开机多圈读数不在 0°～180°，固件只在单圈读数处于 0°～180°并与多圈余角吻合时，对 2 号舵机执行“停止并释放锁力 → 重置圈数 `0x11` → 当前位置恢复锁力 → 多圈回读验证”。例如多圈 817.3°、单圈 97.3°，成功后多圈约为 97.3°；整个过程不发送旋转目标。任何校验失败都会锁住自动转台动作，需检查机械姿态并重新上电；动作报错中的 `INITIAL_MULTI_READ`、`SINGLE_READ_OR_MISMATCH`、`RESET_VERIFY` 分别表示初始多圈读取失败、单圈读取或角度比对失败、重置后回读未验证。单圈角度相同无法证明线缆没有额外缠绕，因此开机参考位的物理保证不可省略。
- `POSE SET`：x、y 为 0～2400 mm，theta 为有限角度。`GOTO_REL`：x、y 各为 ±3394.113 mm，theta 为 ±360 度；都是**相对移动量**，ACK 后执行动作。
- `CHASSIS MOVE_POSITION`：调用 `MovePosition(x,y,theta,speed)`，x/y 是移动前车身坐标系中的终点位移，各为 ±3394.113 mm；theta 是本次转角（±360°，正值左转），speed 是最大轮速（1～5000 RPM，驱动器按整数执行）。四輪同步启动，平移和旋转同时进行，混合运动走圆弧。参数数量错误回复 FORMAT，非法数值回复 RANGE，Release 回复 MODE，锁失败回复 LOCK；均不执行位置动作。有效参数先关闭视觉对齐并独占底盘运动锁，再回复 ACK；函数参数/标定检查失败会报告 FAILED，成功报告 DONE,ESTIMATED。全圈旋转同时平移不能用单段圆弧表示，函数拒绝。ACK 只表示接令，DONE,ESTIMATED 只表示下发和估算等待结束，不是实测到位。等待期间串口控制任务阻塞，后续串口指令延后处理；结束后视觉对齐保持关闭。上位机入口为“底盘 → 麦轮位置移动”，结束后可查询 POSE GET 回读底盘理想位姿。
- `CHASSIS MOVE`：方向 `0` 前、`1` 后、`2` 左、`3` 右；速度 0～1000；`stop` 为 0 或 1。
- `ARM MOVE1`：高度 `-1` 或 0～175 mm，伸出 `-1` 或 0～170 mm；`ARM MOVE2`：转台目标 -150～360 度、夹爪目标 -360～360 度，`-1` 表示跳过该轴；两种动作速度均为 1～300。`SERVO`：ID 0～253 整数，角度 -135～135 度；ID 2 通过转台线缆保护。`ENABLE`：0 或 1。
- 升降（5号电机）和伸缩（6号电机）均通过 `Emm_V5_Pos_Control` 的绝对位置模式执行：`raF=true`、`snF=false`，脉冲分别为目标高度×`HEIGHT_PULSE`、目标伸出×`LENGTH_PULSE`，非负目标使用方向1。目标减小时由驱动器根据当前位置反向运行；重复相同目标也重新发令，0发送绝对零点目标，`-1`不发该轴命令。开机 `Emm_V5_Init()` 广播 `Emm_V5_Reset_CurPos_To_Zero(0)`，将当时位置作为零点，不自动寻找机械零点；需要在已知起始姿态开机。清零目前只等待UART发送及命令间隔，没有驱动器应答确认。`POSE GET`的升降/伸缩值仍是最近下发目标，不是实测位置。
- 参数个数错误回复 `FORMAT`，越界回复 `RANGE`，模式不符回复 `MODE`，未知动作回复 `UNKNOWN_ACTION`；这些错误均不触发新运动。转台动作仍受开机多圈读数和线缆范围保护。
- `ARM INIT_START` / `INIT_LOOK` / `INIT_LOOK2` 不带参数，上位机“机械臂 → 机械臂初始化”提供三个按钮。起始姿态目标为高度 150 mm、伸出 40 mm、转台 -55°、夹爪 0°；观察姿态最终为高度 120 mm、伸出 0 mm、转台 90°、夹爪 0°。观察姿态2调用 `InitArm_look2()`：先升到 170 mm、夹爪设为 0°，等待 2 秒，再收回伸出至 0 mm、转台转至 90°、夹爪打开至 80°，速度均为 150。沿用固件动作顺序与延时，任一步下发失败即结束并报告 `FAILED`；`ACK` 仅表示接令，`ISSUED` 仅表示动作指令已下发，不证明物理到位。执行期间串口命令任务会等待该流程返回。

## 3. 主动事件与路径交互

两轮自动抓取中，一旦某个位置被选中，主控就发送 `STOP_REQUEST,DISC_MATERIAL`，明确结束本次位置识别。抓取、载物台放置及恢复观察姿态期间，COLOR 不触发抓取；DISC_POSITIONS 回复 `ERR,NOT_READY`，不执行、不缓存。前两件完成 `look3` 后，主控重新发送 `START_REQUEST,DISC_MATERIAL`，并把启停边界附近已积压的数据处理到完整帧边界，随后才允许下一件抓取。诊断日志 `[DISC] Material handling resumed: expected=..., cargo=...` 表示新识别会话已开放；此后新收到的位置消息才会作为圆盘旋转后的有效位置。

`{CMD,NAV,ROUTE,...}` 使用有限绝对脉冲位置动作，每个动作四轮各发一次带最大轮速和加速度的位置命令，由驱动板执行加减速；运行中不更新目标或轮速，不使用独立试跑的速度积分执行。正式默认平移2000 mm/s、加速度2000 mm/s²、转向200°/s、转向加速度400°/s²；软件上限分别5000、4000、500、720，均可通过CFG调整并在路径启动时冻结，Profile日志输出实际值。允许倒退，同向共线节点连续通过，转角停转，不外绕。四轮各自进入位置容差且近零转速为接续基础，驱动到位标志不作为额外接续门槛，不再等待100ms编码器稳定窗口；H30完整新鲜数据用于偏航保护和停车后的独立校角，移动中不进行差动纠偏，保留有限终点。H30缺项/旧数据/参考不可信时退回编码器模式，只作日志提示，不因此中断正式任务；参考不可用时航向为规划/编码器估算，不声称±1°实测确认。起步四轮转速未近零时继续查询，清零前不重复再检。末端静止小残差500ms未收敛即失败，不补位重发；真实电机故障、反馈缺失、越界等仍失败停车。Debug异步执行，不阻塞查询和STOP；Release由业务任务等待结果。功能区终点朝向：14为90°、10为270°、22为180°，其他不约束。完成发送 `ROUTE_DONE,ESTIMATED`，不能排除打滑或证明夹爪对准。最多25个节点，首节点须与实车位置一致。详见[辅助与反馈要求](continuous_trial.md)。

正式模式诊断事件 `{EVT,NAV,ROUTE_POSITION,s,x,y,reference,heading}`，距离/坐标单位mm、角度单位度。阶段为POSITION_STRAIGHT/POSITION_TURN。静止残差超时或超限时输出 `{EVT,NAV,ROUTE_MOTOR,id,reason,target,actual,rpm,flags}`，target/actual为65536单位/圈，actual保留驱动符号。静止小残差稳定500ms仍不收敛报MOTOR_RESIDUAL_TIMEOUT，不重发位置命令；硬超限报MOTOR_POSITION_OVERRUN。其他失败包括MOTOR_FEEDBACK_TIMEOUT、MOTOR_DISABLED、MOTOR_ZERO_FAILED和MOTOR_ARRIVAL_TIMEOUT；不能以估时成功替代失败。ACK回复与反馈接线要求见上述说明。

诊断接续耗时：`{EVT,NAV,ROUTE_TIMING,stage,ms}` 中stage为STRAIGHT_SENT、STRAIGHT_ARRIVED、TURN_SENT、TURN_ARRIVED，ms为固件开机毫秒数；ARRIVED到下一SENT的差值为软件确认到位后的接续发令耗时，SENT不表示驱动已产生可见运动。四轮转速均近零但尚未通过到位条件时，每200ms最多输出一次 `{EVT,NAV,ROUTE_WAIT,ms,max_residual,flags1,rpm1,flags2,rpm2,flags3,rpm3,flags4,rpm4}`；残差为直行mm或转向度，flags为驱动原始状态字。此日志用于检查位置收敛、原始驱动状态和额外校角；驱动到位标志不再作为等待条件，不改变到位条件。

正式导航快速接续：仅起步清零一次，后续从停稳反馈累计绝对目标；四轮各自进入位置容差且近零转速即接续，不等待驱动到位标志，取消100ms稳定窗口；起步删除重复查询，清零后仅验证位置，串口查询/发令耗时另计。独立校角采用CFG参数 `底盘/节点导航独立纠偏死区(°)`，默认1、范围0～8，设为0表示存在角度误差就校正；移动中差动纠偏已禁用，每段合并直线四轮各自进入位置容差且近零转速后直接按下一动作目标和此死区校角，避免先修旧航向再转弯。终点同样允许此死区内误差，不再声称±1°航向确认。

| 旧回复或事件 | v2 事件 / 回复 | 含义 |
|---|---|---|
| `{way:WAITING}` | `{EVT,NAV,ROUTE_WAITING}` | Release 主状态机等待路径 |
| `{way:start-end?}` | `{EVT,NAV,ROUTE_REQUEST,start,end}` | 上位机应返回完整路径 |
| `{way:RUNNING}` | `{EVT,NAV,ROUTE_RUNNING}` | 开始执行 |
| `{way:DONE,ESTIMATED}` | `{EVT,NAV,ROUTE_DONE,ESTIMATED}` | 四轮到位及稳定确认通过，H30为可降级辅助，世界XY仍为估算 |
| `{way:ERR,ENDPOINT/EXECUTION/TIMEOUT}` | `{EVT,NAV,ROUTE_FAILED,原因}` | 路径业务失败；主状态机不进入后续抓取 |
| `{way:ERR,NOT_WAITING}` | `{RSP,NAV,ROUTE,ERR,NOT_WAITING}` | Release 尚未请求路径 |
| `{ALIGN:OK}` | `{EVT,VISION,ALIGN_DONE}` | 对齐已完成 |
| `{ALIGN:ERR,TIMEOUT}` | `{EVT,VISION,ALIGN_FAILED,TIMEOUT}` | 反馈超时 |
| `{LidarPose:OK}` / `ERR` | `{EVT,VISION,LIDAR_POSE_DONE}` / `FAILED` | 雷达扫描位姿动作结果 |
| Release 自动触发雷达位姿 | `{EVT,VISION,LIDAR_POSE_RUNNING}` | 自动流程开始，不发送命令回复 |

### 主控请求开启机载视觉功能

主控按业务阶段发送以下五种**单次请求**。机载电脑收到后切换对应的相机识别流程；请求事件方向为 ESP32 → 机载电脑。上位机也可以发送 `{CMD,VISION,START_REQUEST,mode}`，让小车主动发出相同的请求事件。切换请求本身不表示相机已启动、视觉已对齐或物料已识别，也不代替原有 `ALIGN_DATA`、`COLOR`、`CONFIRM` 等回传。当前协议不要求机载电脑对这五种事件逐条回复；仓库外机载视觉程序需按模式实现切换。

| 事件帧 | 机载电脑应开启的功能 | 典型发送时机 |
|---|---|---|
| `{EVT,VISION,START_REQUEST,DISC}` | 圆盘定位／对齐视觉 | 到达圆盘区域、准备定位时 |
| `{EVT,VISION,START_REQUEST,DISC_MATERIAL}` | 圆盘物料识别视觉 | 圆盘定位结束、准备识别待抓物料时 |
| `{EVT,VISION,START_REQUEST,WORK_AREA}` | 粗加工区或暂存区的空工位定位视觉 | 准备向无物料的工位放料时 |
| `{EVT,VISION,START_REQUEST,WORK_AREA_LOADED}` | 粗加工区或暂存区的带物料工位定位视觉 | 准备从已有物料的工位取回或码放时 |
| `{EVT,VISION,START_REQUEST,CORNER}` | 角点视觉识别 | 回家路径执行结束、准备确认初始启停区位置时 |

五个模式值是固定 ASCII 标识，不附带区域编号或任务码。当前粗加工区和暂存区共用一套机械臂工位位姿；机载电脑需结合当前业务阶段区分所在区域。固件提供 `requestVisionStart(VisionStartMode)` 发送接口；当前 Release 流程已在首轮圆盘区自动切换 `DISC`/`DISC_MATERIAL`，并在粗加工放料前开启 `WORK_AREA`。当前两轮粗加工放料后的带物料定位临时停用：相关 look、`WORK_AREA_LOADED` 启停和对齐等待已注释。`STATE_PLACE_TEMP1` / `STATE_STACK_TEMP2` 取回前停车并等待 1000 ms，确认未中止后直接按对应轮次颜色/位置码调用 `RetrieveRoundToCargo`；仅全部取回成功后允许导航。首轮全部成功后请求节点 10→22 路径，到达暂存区并完成 180° 终点朝向后请求 `WORK_AREA`，对齐达标即停止该模式，再执行第一层放料，成功进入第二轮抓取。对齐、路径、搬运失败停止总计时器并进入对应故障状态，不重复取回或导航。第二轮已接入节点 22→14 圆盘抓取、14→10 粗加工第一层放料及取回、10→22 暂存区第二层码垛，分别沿用 DISC/DISC_MATERIAL、WORK_AREA、WORK_AREA_LOADED 请求；第二轮暂存区已有第一层物料，码垛前使用带物料识别，完成或失败后均结束 WORK_AREA_LOADED。两轮抓取均在每件完成后恢复观察并清理旧帧后再开放下一件；动作失败不自动重试。仓库内调试上位机的“视觉”页提供五种请求按钮，显示发送、接令、请求事件及错误状态，同时保留原始日志，不包含相机算法。正常返家使用开局保存的启停区，请求 22→4（区 1，180°）或 22→0（区 2，180°，与区1同样车头朝左）路径，节点取自当前地图的启停区附近位置，需实车核对。路径后开启 CORNER，机载电脑沿用 ALIGN_DATA 回传角点误差；固件等待连续 5 帧达标，停止 CORNER 后复位机械臂，成功才进入 DONE。启动事件、路径估算完成及预设动作成功均不代表传感器确认物理归位。

上位机手动触发角点视觉的完整交互：

```text
电脑   {CMD,VISION,START_REQUEST,CORNER}
ESP32  {RSP,VISION,START_REQUEST,ACK,CORNER}
ESP32  {EVT,VISION,START_REQUEST,CORNER}
电脑   由机载视觉程序处理事件并开启角点识别
```

### 主控请求结束机载视觉功能

五种功能各有对应的结束命令和事件，上位机“视觉”页每一行提供“开启”和“结束”按钮。固件业务流程可调用 `requestVisionStop(VisionStartMode)` 发出结束事件。

| 视觉功能 | 上位机 → 小车结束命令 | 小车 → 机载电脑结束事件 |
|---|---|---|
| 圆盘定位 | `{CMD,VISION,STOP_REQUEST,DISC}` | `{EVT,VISION,STOP_REQUEST,DISC}` |
| 圆盘物料识别 | `{CMD,VISION,STOP_REQUEST,DISC_MATERIAL}` | `{EVT,VISION,STOP_REQUEST,DISC_MATERIAL}` |
| 暂存区／粗加工区定位 | `{CMD,VISION,STOP_REQUEST,WORK_AREA}` | `{EVT,VISION,STOP_REQUEST,WORK_AREA}` |
| 暂存区／粗加工区带物料视觉 | `{CMD,VISION,STOP_REQUEST,WORK_AREA_LOADED}` | `{EVT,VISION,STOP_REQUEST,WORK_AREA_LOADED}` |
| 角点识别 | `{CMD,VISION,STOP_REQUEST,CORNER}` | `{EVT,VISION,STOP_REQUEST,CORNER}` |

机载电脑收到结束事件后停止指定功能的数据采集／识别与反馈；重复结束请求应可安全处理。ACK 只表示小车接令，事件只表示已发出结束请求，均不证明视觉程序已停止，也不表示识别任务成功完成。当前协议不要求对结束事件逐条回复，仓库外机载视觉程序需实现这五种事件的处理。开启另一种视觉不会自动发送上一种的结束事件，切换时按业务阶段显式结束上一种。

```text
电脑   {CMD,VISION,STOP_REQUEST,CORNER}
ESP32  {RSP,VISION,STOP_REQUEST,ACK,CORNER}
ESP32  {EVT,VISION,STOP_REQUEST,CORNER}
电脑   由机载视觉程序处理事件并结束角点识别
```

### 视觉切换示例

```text
ESP32  {EVT,NAV,ROUTE_DONE,ESTIMATED}
ESP32  {EVT,VISION,START_REQUEST,DISC}
电脑   开启圆盘定位视觉，随后按现有协议回传对齐数据
ESP32  {EVT,VISION,ALIGN_DONE}
ESP32  {EVT,VISION,STOP_REQUEST,DISC}
ESP32  {EVT,VISION,START_REQUEST,DISC_MATERIAL}
电脑   开启圆盘物料识别视觉，随后按现有协议回传颜色
```

### 机载电脑显示事件

固件调用 `updateDisplay(命令类型, 显示类型, 正文)`，三个参数依次为：固定命令类型 `DISPLAY`、显示类型 `TASK_CODE` 或 `DEBUG`、要显示的字符串。串口发出 `{EVT,DISPLAY,显示类型,正文}`，无须电脑回复。相同的连续正文只发一次，避免状态机轮询刷屏；切换到其他正文后可再次发送。

```text
{EVT,DISPLAY,TASK_CODE,156+123+516+231}
{EVT,DISPLAY,DEBUG,READ TASK}
```

上位机把 `TASK_CODE` 显示在任务码栏，把 `DEBUG` 显示在调试信息栏，同时保留原始串口日志。扫描成功后发送的是**已解析的四组三位任务码**；`TASK ERR` 等错误只作为 `DEBUG` 消息发送，不会覆盖上次的任务码。这里实现的是机载电脑屏幕显示，赛场实体显示器仍需单独接入。

示例（Release 主状态机请求 2 → 14）：

```text
ESP32  {EVT,NAV,ROUTE_WAITING}
ESP32  {EVT,NAV,ROUTE_REQUEST,2,14}
电脑   {CMD,NAV,ROUTE,2,7,12,13,14}
ESP32  {RSP,NAV,ROUTE,ACK,5}
ESP32  {EVT,NAV,ROUTE_RUNNING}
ESP32  {EVT,NAV,ROUTE_DONE,ESTIMATED}
```

5×5 节点编号（上方是场地北侧）：

```text
20 21 22 23 24
19 18 17 16 15
10 11 12 13 14
 9  8  7  6  5
 0  1  2  3  4
```

## 4. 运行参数 CFG

CFG 参数只在 RAM 中生效，重启恢复默认值。读取须停止视觉对齐；写入还须处于 Debug 模式。`GET` 输出目录可能阻塞视觉反馈，所以对齐开启时拒绝读写。参数 ID 是**本次固件目录的 0 起始下标**，必须先 GET，不可跨版本写死；界面中的数组名称按 1 起始编号。中文名称返回时使用 UTF-8。

| 旧帧 | v2 帧 |
|---|---|
| `{CFG:GET,id}` | `{CMD,CFG,GET,id}` |
| `{CFG:VALUE,id,index,name,value,min,max,int}` | `{RSP,CFG,GET,VALUE,id,index,name,value,min,max,int}` |
| `{CFG:END,id,count}` | `{RSP,CFG,GET,END,id,count}` |
| `{CFG:BEGIN,id}` | `{CMD,CFG,BEGIN,id}` → `{RSP,CFG,BEGIN,OK,id}` |
| `{CFG:SET,id,index,value}` | `{CMD,CFG,SET,id,index,value}` → `{RSP,CFG,SET,OK,id,index}` |
| `{CFG:COMMIT,id}` | `{CMD,CFG,COMMIT,id}` → `{RSP,CFG,COMMIT,OK,id}` |
| `{CFG:ERR,id,reason}` | `{RSP,CFG,动作,ERR,id,reason}` |

`id` 为非零 uint32，`index` 为有效参数 ID；值必须有限、在该项范围内，整数项必须为整数。BEGIN 建立暂存副本；SET 只修改暂存值；COMMIT 持有对齐锁统一写入并清除 PID 历史。GET 或新的 BEGIN 丢弃旧事务。上位机逐条等待确认，COMMIT 后自动 GET 核对；错误或超时立即停止后续发送，超时不自动重发 COMMIT。常见原因：`BUSY_OR_MODE`、`RANGE`、`TRANSACTION`、`FORMAT`、`REGISTRY`、`LOCK`。

## 5. 切换顺序

1. 依据本页逐条修改仓库外发送端，并确认其能发送 HELLO、识别 v2 的 `RSP`/`EVT`、完整处理 CFG 事务和路径请求。
2. 在测试环境先运行新上位机与新固件，核对串口输出和动作安全门。编译通过不代表实车串口或机械动作已验证。
3. 外部发送端适配并测试完成后再切换实车固件；旧指令在 v2 固件中只会收到 `FRAME` 格式错误。

## GPIO5 LED 调光

GPIO5 接 AO3400 栅极，按低边 N 沟道开关、高电平导通驱动。LEDC 通道 0 默认使用 2000 Hz、12 位分辨率，频率可调范围 100～9000 Hz。亮度百分比线性映射占空比，0% 常低、100% 常高（这两个端点没有周期性脉冲）。该通道及共享定时器保留给此驱动。重启恢复关闭和 2000 Hz，不写入 Flash；串口断开和 Debug/Release 切换保持设置。

上位机“LED 调光”页拖动滑块后点击“应用亮度”，输入频率后点击“应用频率”，也可关闭或查询。例：`{CMD,LED,SET,50}` 设置 50%；`{CMD,LED,FREQ,3500}` 改为 3500 Hz 并保持亮度；`{CMD,LED,SET,0}` 关闭并保留频率。每帧末尾发送换行。回复的 hz 是定时器配置回读值，可能与请求值有取整差异。参数不合法返回 `ERR,RANGE`，数量错误返回 `ERR,FORMAT`，PWM 配置失败返回 `ERR,PWM`；频率配置失败时尝试恢复，恢复失败则关闭输出并锁定 PWM 故障。旧固件可能返回 `ERR,UNKNOWN_ACTION`，旧版仅亮度回复仍可显示但标注未报告频率。新固件回复新增频率字段，外部解析器需同步适配。`OK` 和查询仅表示软件 PWM 设置，不代表波形或光强实测。

临时调试修改：`Task_VisualAlignment` 中 300 ms 断流触发停车和 `ALIGN_FAILED,TIMEOUT` 的分支已注释。队列单次等待仍为 300 ms，恢复反馈后帧间隔达到 300 ms 仍清零连续达标计数；这两处不会因断流报告失败。首次对齐 15 秒总超时、显式停止和无效反馈处理保留。手动 `ALIGN_START` 没有首次达标总超时，断流后须显式停止。

总任务超时沿用 3000 秒配置；STATE_TIMEOUT_FAILED 结束全部视觉、停止六个步进电机、锁止后续 MoveArm 发令并暂停主任务，保留夹爪位置，等待人工恢复和重启。仍发送 `{EVT,NAV,ROUTE_FAILED,TIMEOUT}`，不从未知位置自动请求返家路径。自动流程状态提示使用既有 `{EVT,DISPLAY,DEBUG,正文}`，无需新增命令。

## 按圆盘位置抓取：机载电脑使用说明

新流程由视觉电脑判断抓取时机：物料即将到位或刚刚到位，只要电脑认为当前可以从该位置抓取，就上报该位置及颜色。ESP 不再等待这个物料转到原固定夹爪位置，也不自行预测圆盘转速或延时。三个位置分别执行各自的安全接近、夹取和抬升动作，随后复用公共载物台放置动作。

### 指令与选择规则

```text
{CMD,VISION,DISC_POSITIONS,位置,颜色}
{CMD,VISION,DISC_POSITIONS,位置,颜色,位置,颜色}
{CMD,VISION,DISC_POSITIONS,位置,颜色,位置,颜色,位置,颜色}
```

- 复用 Serial0：115200、8N1、ASCII，一行一帧，以 LF 换行。无需新增物理串口。
- 位置编号为 1～3，指圆盘上的三个抓取位置。机载视觉与机械动作需约定相同编号及坐标含义；本版未指定实际几何坐标。
- 颜色编号：红 1、黄 2、蓝 3、绿 4、黑 5、浅蓝 6。位置编号与颜色不绑定；位置编号也不是车上载物台编号。
- 每帧 1～3 组，出现的所有位置均视为当前可抓，位置不得重复；同颜色可以出现在不同位置。没有可抓位置时不发送该指令。
- 一次同时看到三个候选时，推荐合并为一帧上报，保证 ESP 在同一个候选集合内选择。分成三帧时，每帧独立决策，不等后续帧凑齐。
- ESP 按当前轮次任务码的下一件颜色选择：首轮 round1_colors[roundProgress]，次轮 round2_colors[roundProgress]；目标载物台固定为 roundProgress+1。不越过当前颜色抓后面的任务。同颜色有多个候选时选位置号较小者，与发送顺序无关。
- 每帧最多调用一个位置函数；未选候选不缓存、不排队，也不会在以后自动执行。下一次抓取必须由电脑重新判断时机并上报新帧。
- 选中位置后，ESP 立即发送 `STOP_REQUEST,DISC_MATERIAL` 结束旧识别会话，并令 `firstDiscGrabReady=false`。从夹取、放入载物台到主状态机恢复 `InitArm_look3()` 期间，收到的 `DISC_POSITIONS` 均回复 `ERR,NOT_READY`，不执行、不缓存、不在恢复后重放。
- 前两件恢复 `InitArm_look3()` 后，ESP 重新发送 `START_REQUEST,DISC_MATERIAL`。串口任务再排完启停边界附近旧数据的完整帧边界，才令 `firstDiscGrabReady=true`；机载电脑必须根据旋转后的实时画面重新计算并上报位置。第三件完成后不再重启该识别。
- DISC_MATERIAL 未开启、任务码无效、非抓取阶段、任务进度完成、连续对齐未停止、自动流程未准备好、目标载物台有料时，不调用位置抓取动作。

### 回复与事件

| 帧 | 含义 |
|---|---|
| `{RSP,VISION,DISC_POSITIONS,ACK,n}` | 接受本帧，n 为候选数量，不表示抓取成功 |
| `{EVT,VISION,DISC_POSITIONS,SELECTED,pos,color,cargo}` | ESP 已选择位置、颜色和目标载物台，随后结束旧 DISC_MATERIAL 识别会话并开始抓放 |
| `{EVT,VISION,DISC_POSITIONS,DONE,pos,color,cargo}` | 位置夹取、载物台放置和载物台颜色登记成功；任务进度随后推进 |
| `{EVT,VISION,DISC_POSITIONS,FAILED,pos,color,cargo}` | 夹取或放置失败；不推进进度，自动流程停止物料视觉并进入搬运故障 |
| `{EVT,VISION,DISC_POSITIONS,SKIP,expected,cargo,COLOR_MISMATCH}` | 本帧无当前所需颜色，等待新报告 |
| `{RSP,VISION,DISC_POSITIONS,ERR,reason}` | 整帧拒绝，无 ACK、不调用位置函数 |

reason：参数数量不符为 FORMAT；位置/颜色超范围或非整数为 RANGE；重复位置为 DUPLICATE_POSITION；状态拒绝为 NOT_ACTIVE、NO_TASK、NOT_WAITING、ROUND_COMPLETE、ALIGN_ACTIVE、NOT_READY、OCCUPIED。连续试跑期间沿用全局 BUSY 限制。整帧全部校验后才调用函数，后面的错误候选不会让前面的有效候选先执行。

### 联调示例

上电 Debug 模式可按以下顺序测试（各行均需追加换行）：

```text
电脑 -> {CMD,TASK,SET,156+123+516+231}
ESP  -> {RSP,TASK,SET,OK,156+123+516+231}
电脑 -> {CMD,VISION,START_REQUEST,DISC_MATERIAL}
ESP  -> {RSP,VISION,START_REQUEST,ACK,DISC_MATERIAL}
ESP  -> {EVT,VISION,START_REQUEST,DISC_MATERIAL}
电脑 -> {CMD,VISION,DISC_POSITIONS,1,5,2,1,3,6}
ESP  -> {RSP,VISION,DISC_POSITIONS,ACK,3}
ESP  -> {EVT,VISION,DISC_POSITIONS,SELECTED,2,1,1}
ESP  -> [DISC_POSITION] material 1 placed on cargo 1
ESP  -> {EVT,VISION,DISC_POSITIONS,DONE,2,1,1}
```

首次连接仍须先完成 SYS HELLO 握手。上例任务首色为红 1，因此从三个可抓位置中选位置 2，目标载物台为 1；位置 1 的黑色和位置 3 的浅蓝不会抢先抓取。选中后旧 DISC_MATERIAL 会话结束；成功后进度变为 1，主状态机恢复观察姿态并启动新会话。恢复开放前重复发送会收到 NOT_READY且不会再次动作，开放后应按圆盘旋转后的画面发送全新的位置帧。仅发 `{CMD,VISION,DISC_POSITIONS,1,5,3,6}` 则 ACK 后返回 `{EVT,VISION,DISC_POSITIONS,SKIP,1,1,COLOR_MISMATCH}`。

Release 自动流程继续使用 DISC 定位完成后开启 DISC_MATERIAL 的切换：电脑收到 START_REQUEST,DISC_MATERIAL 后开始判断时机并发送新指令；收到 STOP_REQUEST,DISC_MATERIAL 后停止报告。自动流程还要求 ESP 已准备好抓取，NOT_READY 时等待恢复后重新判断，不重放旧候选。

调试上位机“视觉 → 圆盘物料回传与抓取”中，给圆盘位置 1～3 选择颜色（不参与本帧的位置选“不报送”），点击“上报可抓位置”。Python 发送端也可复用 `upper_computer.build_disc_positions_command([(1, 5), (2, 1), (3, 6)])` 生成同样的帧。

### 动作接入与旧接口

`src/disc_position_grab.cpp` 中的 `GrabDiscPosition1/2/3(color,cargo)` 分别完成三个位置的夹取和抬升，再调用 `PlaceHeldDiscMaterialToCargo` 放到目标载物台。只有整个动作成功才登记载物台颜色并推进 `roundProgress`；失败保持进度并进入搬运故障。成功后的下一件仍由主状态机恢复观察姿态和串口旧帧排空机制开放。

原 COLOR、GRAB、FORCE_GRAB 保留兼容，仍会调用旧固定位置的真实动作。机载电脑迁移到本流程后，只发 DISC_POSITIONS；不要同时发旧 COLOR/GRAB，否则旧入口仍可能触发固定位置运动。FORCE_GRAB 仅用于原动作调试。

固件堵转检查已临时禁用：`src/node_position_control.h` 的 `MOTOR_STALL_CHECK_ENABLED=false` 对全部四轮生效，不再仅凭堵转状态位报告 `MOTOR_STALL`；驱动器自身保护未修改，其他错误和到位条件保持有效。恢复时将开关改为 `true` 后重新编译烧录。

机械臂速度参数（ARM MOVE1/MOVE2 与全部搬运速度）统一接受有限且≥1的float数值，取消300/500人为上限。转换到升降/伸缩驱动转速时，仅按16位RPM字段饱和，避免整数回绕。

## 统一执行与计时扩展（2026-10-21）

串口运动入口改为提交作业，保留上述 ACK 与既有完成事件字段。先保留作业、发送 ACK，再放行工作任务。抓放期间仍处理 POSE、IMU、CFG GET、MOTION STATUS、停止及所需路径/视觉输入；竞争运动命令可回复 BUSY，串口不等待整段动作。旧 ARM INIT/MOVE 的 ISSUED 语义保留，不能按该事件宣称实测到位。

新增 `{CMD,MOTION,STATUS,id}`，返回 `{RSP,MOTION,STATUS,OK,id,state,phase,elapsed_ms}`；未知编号返回 `ERR,ID`。state 为 QUEUED/RUNNING/DONE/FAILED/CANCELLED；保留当前和最近八个作业。已有命令 ACK 不新增 id 字段，日志 RESULT/FLAG 可用于关联作业。NAV STOP 取消统一作业及识别业务，ACK 仅表示请求停止，终态和现场停车另行确认。

粗加工区、暂存区每件放料在下降步骤结束、松爪前各回读一次转盘和伸缩电机，输出 `[MaterialPlace] job=作业号 position=工位号 axis=turret/length target=目标 actual=回读值 error=偏差 unit=deg/mm`。角度保留多圈坐标，伸出长度由6号电机有符号编码器位置换算；偏差统一为 `actual-target`。兼容旧同步接口的作业号为0。读取失败输出对应轴的 `read=ERR`（伸缩轴附reason），仅作日志诊断，不重试、不追加到位等待、不因偏差或诊断读取失败中止放料。原串口日志直接显示，无需新增查询命令。

计时帧不改变协议版本 2：

```text
{EVT,MOTION,PHASE,0,time_ms,state_name}
{EVT,MOTION,FLAG,id,time_ms,name,value}
{EVT,MOTION,SPAN,id,time_ms,name,BEGIN|END}
{EVT,MOTION,STEP,id,time_ms,index,name,BEGIN|END}
{EVT,MOTION,RESULT,id,time_ms,DONE|FAILED|CANCELLED,elapsed_ms}
{EVT,MOTION,RUN_START,0,time_ms}
{EVT,MOTION,RUN_RESULT,0,time_ms,DONE,elapsed_ms}
{EVT,MOTION,ERROR,id,time_ms,reason}
```

设备时间为开机毫秒，编号 0 为业务/视觉。SPAN/STEP 可嵌套或并行，不得累计成总成绩；RUN_RESULT 缺失则本次计时不完整。未收到首帧便停止视觉时 ERROR 而不生成首帧 END。分析命令、完成策略及实车验收见 [统一运动执行](motion_executor.md)。

### 舵机当前位置设为零点
机械臂页“机械臂零点设置”发送 `{CMD,ARM,SERVO_ZERO,ID}`，仅Debug且运动空闲时执行。ID 2为转盘逻辑轴，映射到Serial1的7号张大头电机，清零当前位置，并将机械零位的校准后单圈编码器值保存到主控NVS；回复的两个角度字段均为设零后的实测多圈角度。其他ID下发舵机厂商0x17设置原点指令，内容为ID、reset=0；执行后回读并回复 `{RSP,ARM,SERVO_ZERO,READBACK,ID,单圈角度,多圈角度}`，界面显示实测角度。该操作改变舵机后续角度目标的基准，不等于0x11重置圈数。
协议依据：https://fashionstar.com.cn/wiki/protocols/servo/uart-rs485/#22

SERVO_ZERO补充（仅华馨京舵机，逻辑ID 2转盘不适用）：按厂商SDK要求先用0x18释放锁力，再发0x17设原点，之后0x18恢复当前位置锁力，帧间隔10ms。完成后读取0x0A单圈及0x10多圈角度，返回 `{RSP,ARM,SERVO_ZERO,READBACK,ID,单圈角度,多圈角度}`，界面显示实测结果；回读失败返回SERVO_READ。参考：https://fashionstar.com.cn/wiki/sdk/servo/arduino-esp32-sdk/


## 转盘张大头电机

转盘与底盘共用Serial1，115200bps，主控TX=17、RX=18。转盘物理地址为7，16细分（3200脉冲/圈），传动比1:1；配置常量在`src/turret_motor.h`。

原机械臂角度命令、路径规划与线缆范围-150～360度保持使用。`{CMD,ARM,SERVO,2,角度}`中的2是兼容上位机的逻辑轴号，实际发送7号张大头绝对位置指令，不再发送2号华馨京舵机指令。正角度对应电机CCW（与原舵机一致）；转盘通过0x43系统状态同步读取单圈编码器和累计位置，按保存的机械零位换算，保留符号与圈数。`POSE GET`转盘回读失败返回`{RSP,POSE,GET,ERR,MOTOR_READ,7}`。

转盘上电只使能并读取驱动器坐标，不自动旋转或清零。更换电机后需要先校准机械零位；Debug下将机构摆到0度，再发`{CMD,ARM,SERVO_ZERO,2}`。该命令保存机械零位到主控NVS，并清零本次电机位置计数，不执行回零运动。每次电机上电时必须位于机械0～180°；读取系统状态后据保存的零位重建坐标。

转盘500度/秒参考速度量化为83RPM（498°/s）。正式转盘动作直接使用`TURRET_MOTOR_ACCELERATION=126`（acc=126），理论角加速度约923.1°/s²，相对acc=100提高20%，从静止加速到83RPM理论约539.5ms，旧舵机动作阶段20/500/600ms参数不再直接用于转盘。转盘不启用多机同步，底盘广播同步不会启动转盘命令。停止当前运动时包含7号转盘；底盘ENABLE仅控制1～4号电机。


### 升降和伸缩轴零点

上位机“机械臂零点设置”的轴ID下拉框支持1夹爪、2转盘、5升降、6伸缩。将机构放到对应机械零位，在Debug且运动空闲时选择轴并点击“将当前位置设为零点”：

```text
{CMD,ARM,SERVO_ZERO,5}
{CMD,ARM,SERVO_ZERO,6}
```

5、6号通过Serial1发送张大头当前位置清零命令，确认应答后读取编码器位置，按对应轴脉冲标定换算成毫米，并更新`currentArm.high`或`currentArm.length`。回复沿用`{RSP,ARM,SERVO_ZERO,READBACK,ID,位置,位置}`，两个位置字段均为同一实测值，单位mm；上位机按升降/伸缩轴显示毫米结果。清零命令失败返回`MOTOR_WRITE`，回读失败返回`MOTOR_READ`，失败后不改写主控缓存，后续可重新操作。

此操作只改变本次上电的位置计数，不触发运动或保存回零点。6号伸缩轴原有上电回到已保存单圈零点的行为保持不变；本命令不会改写该已保存零点。


### 电机通用错误帧与查询匹配

总线收到`地址 00 EE 6B`时，会输出原始错误帧。该帧没有携带原请求功能码，因此记录错误后，在本次原有25ms响应窗口内继续接收对应地址和功能码的合法应答；若收到，输出`matched after unassigned error`并使用该实测回复。若窗口内只有错误帧而没有匹配应答，仍返回`MOTOR_COMMAND_ERROR`；完全无匹配帧且未收到通用错误帧时返回`MOTOR_FEEDBACK_TIMEOUT`。不重发请求，不延长响应窗口，不用目标缓存代替实测位置。


转盘连续通信时序验证：经用户确认，在`TurretMotorReadAngle`取得电机总线事务锁后，等待`TURRET_QUERY_GAP_MS=5`毫秒，再发送7号位置查询；间隔期间其他任务不能插入电机命令。该间隔只用于验证连续通信时序假设，不改变升降实测到位条件，也不改变查询发送后的25ms响应窗口。是否消除Release初始化中的`07 00 EE 6B`仍需实车复测。


### 转盘绝对圈数与方向

转盘位置指令继续使用EMM多圈绝对脉冲位置（raF=true），不触发单圈就近回零。普通MoveArm和整组搬运路径均严格保留标定/指令给出的有符号累计角度，已移除最近等效角及整组±360度选角：350°保持350°，不会转换成-10°；0°与360°为不同位置，从350°回到0°应返回累计零位。线缆范围仍为-150～360°。普通MoveArm的-1仍表示跳过转盘，规划入口MoveArmPlanned的-1表示真实-1°。

转盘方向已按原舵机方向翻转：正角度发送CCW（dir=1），负角度发送CW（dir=0）；原始编码器反馈同步反号后换算为转盘角度，避免控制方向与到位判定相反。上电仍只使能和读取，不自动回零。机械零点需要与原舵机的0°参考一致；清零不应掩盖已有线缆圈数。


### 转盘机械零位掉电保存

在Debug且运动空闲时，将转盘放到原机械0°位置，执行`{CMD,ARM,SERVO_ZERO,2}`。主控先读取`07 43 7A 6B`系统状态快照，确认当前位置清零成功后，将快照中的校准后单圈编码器值保存到ESP32的NVS（命名空间turret，键zero_enc）。仅设置零位时写入存储，平时读取和运动不写NVS。

用户已约定：每次给转盘电机上电，物理角度都在机械0～180°，传动比为1:1。系统状态的一帧回复含单圈编码器E及累计位置P（均65536单位/圈），由`(E-P) mod 65536`反推出本次电机累计位置原点的单圈相位，再相对保存的机械零位换算坐标。这个过程只读取和换算，不会自动旋转回零。主控重启和电机单独断电重启均通过下一次回读恢复参考坐标；运行中仍保留完整累计圈数。

绝对运动目标同步减去重建出的电机原点角后换算为脉冲，因此在90°上电后发送机械0°，仍返回原机械零位；0°和360°始终是两个不同的累计位置。方向仍与原舵机一致。首次使用该固件必须重新执行一次转盘机械零点设置，旧版临时清零没有可迁移的标定。未设置零位时只报告`mechanical zero not calibrated`，串口和OTA继续运行，可用原零点设置命令完成标定。

0x43帧格式依据[厂家Emm V5说明书的系统状态参数](https://devpress.csdn.net/xian/68945b69080e555a88d61cd3.html)。升降、伸缩轴的零点设置仍为本次上电清零。


### 转盘过冲首轮调整与测量

本轮保留42RPM峰值速度和原有动作重叠：`TURRET_ARRIVED`仍在目标±5°时放行下一步骤，不新增停稳等待。仅转盘曲线改为独立100ms参考加减速，避免沿用原舵机20ms急加减速和第一层500ms长曲线。原第一层长行程的曲线时间约缩短0.4秒；总体耗时和过冲改善程度需实车比较。

转盘位置发令不再读写或自动放大 PID。此前的 Kd×1.5 候选逻辑已移除，旧 NVS 的 `turret/pid_base` 不再使用，机械零点存储不受影响。驱动器当前 PID 由 EMM 参数页显式读写；固件更新本身不会把电机 RAM 中已生效的旧 PID 恢复为原值。

后台每50ms读取同一0x43帧的绝对角、转速和电机标志，输出：

```text
{EVT,TURRET,COMMAND,转动号,毫秒,目标角,起始角,RPM,加速度档位}
{EVT,TURRET,SAMPLE,转动号,毫秒,目标角,实测角,有符号RPM,电机标志,采样最大过冲角}
{EVT,TURRET,RESULT,转动号,耗时毫秒,状态,目标角,最近实测角,采样最大过冲角}
```

过冲按本次指令从起点到目标的方向计算。连续三个样本满足驱动器到位、0RPM、误差≤1°后，仅结束后台记录，状态为SETTLED；新目标提前到来为REPLACED（只代表上一段记录被截断），停止作业为CANCELLED。这些状态均不阻止动作重叠。采样峰值只反映50ms离散采样结果，不能视为连续运动的精确峰值。

协议字段依据[厂家Emm V5说明书](https://devpress.csdn.net/xian/68945b69080e555a88d61cd3.html)的位置PID和系统状态定义。
# EMM V5 电机参数调试

### 驱动配置中的7项调节

EMM页新增截图红圈的最大输出电压、开环工作电流、堵转最大电流、堵转检测转速/电流/时间、位置到达窗口。输入框初始为空，读取后填入实际值；切换地址清空旧值。驱动配置有独立的掉电保存选项，默认仅RAM。

```
{CMD,EMM,DRIVE_GET,id}
{CMD,EMM,DRIVE_SET,id,max_voltage,open_current,max_stall_current,stall_rpm,stall_current,stall_ms,arrival_window,save}
{RSP,EMM,DRIVE_GET,READBACK,id,max_voltage,open_current,max_stall_current,stall_rpm,stall_current,stall_ms,arrival_window}
{RSP,EMM,DRIVE_SET,READBACK,id,max_voltage,open_current,max_stall_current,stall_rpm,stall_current,stall_ms,arrival_window}
```

字段依次为mV、mA、mA、RPM、mA、ms、0.1度，均为大端uint16原始值；save为0/1。界面窗口输入以度显示，0.8°封包为8；接受0.1°整数倍，不进行四舍五入。输入范围只按协议表示，具体可用值取决于驱动器型号和固件。开环工作电流用于开环模式；到达窗口影响驱动器到位位，不改变调参页±1°稳定时间统计标准。

固件读取0x42 6C的33字节配置回复（长度字段33，参数数21），保留28字节完整配置载荷；仅修改对应7项，按0x48 D1写回。原有电机类型、脉冲/通讯模式、波特率、细分、校验、应答方式、保护使能及保留地址字段等均从本次实读复制，不使用默认配置覆盖。

读、改、写、回读由一个总线事务锁保护。经用户确认，读取完成到写入间隔5ms，写入ACK成功到回读：RAM间隔5ms，Flash间隔100ms（经用户确认的时序验证值，尚需实机确认）；ACK超时路径不追加间隔，仍用实际回读确认。无重试，沿用25ms应答窗口。初始读取失败不下发写入，明确拒绝直接返回错误，7项回读不一致时内部标记`DRIVE_VERIFY_MISMATCH`，串口返回`{RSP,EMM,DRIVE_SET,READBACK_MISMATCH,id,七项实际值}`，上位机显示不一致并将输入框更新为实际值，不将其报告为全部写入成功。参数错误不影响后续串口或OTA恢复。Flash选项仅设置保存标志，运行回读不是断电重启验证。

DRIVE_SET诊断日志区分`BASE_READ`（写入前读完整配置）、`WRITE_ACK`（配置写入应答）、`READBACK`（写后读取）、`VERIFY`（数值比较）。BASE_READ失败日志明确标记`write not sent`。同时输出成功配置读取的完整RX、0x48写入TX和收到的ACK原帧；VERIFY失败显示字段索引及期望/实际值。单独的0x42超时不能区分写前读取和写后回读，应结合阶段日志判断。

PID_SET 每次只发送一次写入。写入 ACK 在既有 25 ms 窗口内超时时，仍执行一次 PID 回读；Kp、Ki、Kd 全部等于目标值才返回 READBACK。回读不一致返回 `PID_VERIFY_MISMATCH`，回读失败返回通信错误；明确的驱动器拒绝不转为成功。掉电保存选项控制写入帧的保存标志，运行参数回读不等于断电重启验证。

经用户确认，在写入 ACK 成功后持总线事务锁间隔5ms，再发送0x21回读，用于验证连续命令时序问题；ACK超时路径不追加此间隔。保持25ms应答窗口，不重发写入或读取。此修改是否解决实机写后回读无回复仍需验证。

上位机新增与姿态、底盘并列的「EMM 电机」页。以下命令限 Debug 且动作空闲，电机地址 1～7；1～4 底盘、5 升降、6 伸出、7 转盘。

| 命令 | 参数 | 成功回复 |
| --- | --- | --- |
| `{CMD,EMM,PID_GET,id}` | 地址 | `{RSP,EMM,PID_GET,READBACK,id,kp,ki,kd}` |
| `{CMD,EMM,PID_SET,id,kp,ki,kd,save}` | PID 为 uint32 原始整数，save=0 RAM / 1 Flash | `{RSP,EMM,PID_SET,READBACK,id,kp,ki,kd}`，数值来自写入后的读取 |
| `{CMD,EMM,STATE_GET,id}` | 地址 | `{RSP,EMM,STATE_GET,READBACK,id,encoder,position,rpm,flags}` |
| `{CMD,EMM,MODE_SET,id,mode,save}` | P_Pul 模式 0～3，save 同上 | `{RSP,EMM,MODE_SET,ACK,id}` |
| `{CMD,EMM,ENABLE,id,enabled}` | 0 失能 / 1 使能 | `{RSP,EMM,ENABLE,ACK,id}` |
| `{CMD,EMM,CLEAR_STALL,id}` | 解除堵转保护 | `{RSP,EMM,CLEAR_STALL,ACK,id}` |

模式 0 关闭脉冲输入，1 开环，2 闭环，3 En/Dir 复用为限位输入/到位输出。状态位置单位为 65536/圈，转速为 RPM，flags 位 0～3 分别为使能、到位、堵转、堵转保护。配置 ACK 来自驱动器应答；通信失败返回 `{RSP,EMM,action,ERR,reason}`，不会锁死主流程或维护通道。

7 号转盘运动直接使用驱动器当前 PID，不再按主控 NVS 基准自动修改。



## 转盘半自动 PID 试验

EMM 页选择7号、读取或输入 PID 后，点击“写入 RAM 并开始试验”。上位机将本组 PID 仅写 RAM，确认三项实际回读一致，再先记录移动到 A 的过程，然后按 A→B→A 重复指定次数。不同组的结果保留在表中；选中一行可查看对应 PID 的角度曲线。导出 CSV 包含所有组、各段、各采样点及其参数。

单段命令：`{CMD,EMM,TURRET_TRIAL,angle,speed_deg_s,ramp_ms,period_ms,duration_ms}`。限 Debug、统一运动作业空闲。角度沿用 -150～360°有符号多圈坐标；速度量化为整数 RPM（最小非零1 RPM），加减速时间量化为驱动器档位，COMMAND 中记录实际 RPM/档位。period/duration 为正 uint32 毫秒，ramp 为 uint16 毫秒。

固件先回复 `{RSP,EMM,TURRET_TRIAL,ACK}`，统一运动任务读取实际起点后发令，并记录完整 duration 时段。到位不提前结束记录；期满仅结束记录，不是实测到位或停车承诺。观察时长应覆盖运动与到位后的观察，过短时下一段可能从运动中接续。现有普通转盘运动仍使用50ms目标采样及到位后3次采样结束记录。

调参 SAMPLE 比普通格式多三个尾部字段：

```
{EVT,TURRET,SAMPLE,trace_id,received_ms,target_deg,angle_deg,signed_rpm,flags,overshoot_deg,encoder,position_counts,query_ms}
```

encoder/position 来自同一0x43回复；query_ms 包含既有5ms查询间隔及串口等待，received_ms 是主控完整读取回复后的时间。请求 period 默认为20ms，实际采样受查询耗时、任务调度、串口日志带宽影响，不保证100Hz。上位机保留实际时间戳并统计平均/最大间隔。转盘样本不是独立外部角度传感器测量，无法直接量化联轴器、齿隙或机构末端振动。

记录结束产生 `TURRET RESULT`，状态 OBSERVED；随后作业完成事件 `{EVT,EMM,TURRET_TRIAL,DONE}` 才触发下一段。数据读取失败仍保留现有总线错误日志和实际采样空隙；观察期满不锁死主流程。界面停止按钮复用 `{CMD,NAV,STOP}`，取消当前作业并停止电机，不只是停止绘图；断线/设备重启/模式变化时取消上位机后续发令序列。

指标：按方向计算最大采样过冲；±1°内持续到记录结束且至少0.5秒的稳定时间；最后0.5秒角度峰峰值；最后采样误差。末段仍运动时峰峰值不能解释为静止抖动。CSV包含 PID、请求速度/曲线、实际 RPM/档位、观察时长、原始计数、转速、标志和查询耗时，可据此离线复算。

驱动配置实机记录：该7号驱动器曾对6000/10000mV请求保留5000mV，对6000mA检测电流请求回读5000mA；这是当前硬件/固件的实测行为，不据此在主控中增加通用5000限值。写入ACK不代表所有字段都采用请求值，始终以回读为准。
