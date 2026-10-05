# ESP32 Serial0 串口协议 v2

本文是机载电脑、仓库上位机和 ESP32 之间的**唯一机器指令规范**。固件不接收旧版 `{way:...}` 等指令。仓库外发送端必须先按下表迁移，再切换固件。舵机 Serial2、扫码软串口不属于本协议。

## 1. 传输与帧结构

- 115200 bit/s，8N1。每行恰好一帧，发送端以 `\n` 结尾；固件兼容 `\r\n`。
- 发送给 ESP32：`{CMD,类别,动作,参数...}`。ESP32 的命令回复：`{RSP,类别,动作,状态,参数...}`。ESP32 主动发出的状态：`{EVT,类别,事件,参数...}`。
- 固定字和输入参数只用可打印 ASCII，区分大小写；不得包含空字段、空格、逗号、花括号或换行。参数目录的**返回名称**与显示事件的**正文**可以是 UTF-8 中文。显示正文可以有空格，但不能含逗号、花括号或控制字符。
- 输入帧最多 **159 字节**（不含换行），最多 32 个逗号字段。超长行会被整行丢弃并回复 `{RSP,SYS,FRAME,ERR,TOO_LONG}`；结构错误回复 `{RSP,SYS,FRAME,ERR,FORMAT}`。固件不会执行截断或格式错误的指令。
- 数字用十进制，必须完整且有限；`NaN`、`inf`、缺字段、多字段均无效。`ACK` 表示已接令，**不代表物理到位**；`ROUTE_DONE,ESTIMATED` 也只是开环估计结束。
- 上位机连接后先发 `{CMD,SYS,HELLO}`，仅收到 `{RSP,SYS,HELLO,OK,2}` 才允许控制。超时或版本不符时断开连接。启动日志如 `version:`、`Debug mode` 只供人阅读，不参与机器帧解析。

## 2. 命令总表与旧版迁移对照

下表中的参数顺序就是实际发送顺序。`两者` 指 Debug 与 Release；`Debug` 指上电调试模式。错误通常为 `{RSP,类别,动作,ERR,原因}`。下表未标为 Release 的 Debug 动作在 Release 中回复 `MODE`，不会执行。

| 旧指令 | v2 指令 | 模式 | 成功回复 / 后续事件 |
|---|---|---|---|
| 无 | `{CMD,SYS,HELLO}` | 两者 | `{RSP,SYS,HELLO,OK,2}` |
| 无 | `{CMD,LED,SET,percent}` | 两者 | `{RSP,LED,SET,OK,percent,hz}`；亮度为 0～100 的整数 |
| 无 | `{CMD,LED,FREQ,hz}` | 两者 | `{RSP,LED,FREQ,OK,percent,hz}`；频率为 100～9000 Hz 的整数，保持亮度 |
| 无 | `{CMD,LED,GET}` | 两者 | `{RSP,LED,GET,OK,percent,hz}`；读取固件 PWM 设置值 |
| `{Mode:Release}` | `{CMD,SYS,RELEASE}` | Debug | `{RSP,SYS,RELEASE,OK}`；失败 `TIMER` / `TASK` |
| `{ready}` | `{CMD,SYS,READY}` | Release | `{RSP,SYS,READY,OK}` |
| `{start}` | `{CMD,SYS,START}` | Release | `{RSP,SYS,START,OK}`；上位机“姿态 → 启动位置 → 雷达扫描完成（继续运行）”可代发，解除 `WAIT START` 等待并进入扫码行驶 |
| `{help}` | `{CMD,SYS,HELP}` | 两者 | `{RSP,SYS,HELP,OK}`；帮助文字参见本文件 |
| `{StartZone:1}` / `:2` | `{CMD,NAV,START_ZONE,1}` / `2` | 两者 | `{RSP,NAV,START_ZONE,OK,1}` / `2` |
| `{way:0-1-8}` | `{CMD,NAV,ROUTE,0,1,8}` | 两者 | `{RSP,NAV,ROUTE,ACK,3}`，随后路径事件 |
| 无 | `{CMD,VISION,START_REQUEST,mode}` | 两者 | `{RSP,VISION,START_REQUEST,ACK,mode}`，随后 `{EVT,VISION,START_REQUEST,mode}` |
| 无 | `{CMD,VISION,STOP_REQUEST,mode}` | 两者 | `{RSP,VISION,STOP_REQUEST,ACK,mode}`，随后 `{EVT,VISION,STOP_REQUEST,mode}` |
| `{ALIGN:START}` | `{CMD,VISION,ALIGN_START}` | 两者 | `{RSP,VISION,ALIGN_START,OK}` |
| 无 | `{CMD,VISION,ALIGN_START,模式}` | 两者 | `{RSP,VISION,ALIGN_START,OK}`，按指定模式死区连续对齐 |
| `{ALIGN:STOP}` | `{CMD,VISION,ALIGN_STOP}` | 两者 | `{RSP,VISION,ALIGN_STOP,OK}` |
| `{ALIGN:angle,x,y}` | `{CMD,VISION,ALIGN_DATA,angle,x,y}` | 两者 | 正常连续反馈不逐帧回复 |
| `{color:N}` | `{CMD,VISION,COLOR,N}` | 两者，须开启 DISC_MATERIAL | `{RSP,VISION,COLOR,OK}`，随后报告 GRAB/SKIP/BLOCKED 决策 |
| 无 | `{CMD,VISION,GRAB,cargo}` | 两者，须开启 DISC_MATERIAL | 使用最近 COLOR，回复 ACK 后发送 DONE/FAILED（动作结果） |
| 无 | `{CMD,VISION,GRAB,color,cargo}` | 两者，须开启 DISC_MATERIAL | 同帧指定颜色与载物台，回复 ACK 后发送 DONE/FAILED（动作结果） |
| `{ok}` | `{CMD,VISION,CONFIRM}` | Release | `{RSP,VISION,CONFIRM,ACK}` |
| `{LidarPose}` | `{CMD,VISION,LIDAR_POSE}` | 两者 | `{RSP,VISION,LIDAR_POSE,ACK}`，随后完成或失败事件 |
| `{POSE:GET}` | `{CMD,POSE,GET}` | 两者 | `{RSP,POSE,GET,OK,x,y,theta,h,l,turret,pawl}` |
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

手动强制抓取：`{CMD,VISION,FORCE_GRAB,color,cargo}`，color=1～6、cargo=1～3，Debug/Release 均可用。无需开启 DISC_MATERIAL、无需任务码、不检查任务颜色或抓取顺序；接令 ACK 后先停止连续对齐并取消其业务等待，再直接调用当前 `GrabDiscMaterial(color,cargo)`。强制模式下，目标载物台已有料时发送 `{EVT,VISION,FORCE_GRAB,WARN,OCCUPIED,原颜色,cargo}` 并继续执行；普通抓取仍拒绝占用。标定位姿及运动边界检查继续生效。警告在上位机状态栏及日志显示，函数返回后状态栏仍保留本次占用警告。成功放置后将载物台颜色记录更新为本次颜色；动作失败提前返回时保留原记录。当前该函数已实现底盘与机械臂抓取、放到载物台的动作，强制入口不修改任务进度。函数返回后发送 `{EVT,VISION,FORCE_GRAB,REQUESTED,color,cargo}`；强制入口不使用完成结果推进任务，REQUESTED 不证明成功，必须结合物料日志及现场结果判断，拒绝错误参数回复 RANGE，参数数量错误回复 FORMAT。上位机“视觉 → 圆盘物料回传与抓取”选择颜色和载物台后点击“强制抓取”。

上位机“视觉 → 手动输入任务码”输入 `156+123+516+231` 后点击下发或按回车，发送 `{CMD,TASK,SET,156+123+516+231}`。固件回传 `{RSP,TASK,SET,OK,任务码}` 并发送 TASK_CODE 显示事件；界面仅在确认后更新当前任务码。四组都须恰好三位，第一/三组颜色为 1～6，第二/四组各为 1、2、3 的排列；格式或数值错误回复 RANGE，参数数量错误回复 FORMAT，不覆盖已有任务码。

TASK SET 在 Debug 可修改任务码并重置抓取进度为 0；开启 DISC_MATERIAL 且停止连续对齐后，COLOR 按第一轮任务顺序判断，并在目标载物台为空时调用现有抓取动作。下发任务码不会清空载物台记录。Release 只允许 WAIT_START 预置或 READ_TASK 尚未收到任务时输入；进入抓取/放置等执行阶段回复 BUSY。手动预置的任务码在进入 READ_TASK 时直接使用，扫码等待期间收到手动码也会结束扫码并停车；迟到的扫码结果不能覆盖已接受的手动码。任务码仅在 RAM 中保存，重启清空。手动码不确认车辆位置，后续导航仍要求车辆实际位于路径起点。

五种视觉模式各有独立的 X、Y、角度死区。默认每种模式均为 X=3、Y=3（视觉单位）、角度=0.2°，可在停止对齐后通过上位机“参数”页分别修改 `对齐/模式/X死区`、`Y死区`、`角度死区(deg)`；参数只在 RAM 生效，重启恢复默认值，修改目录后须重新 GET 参数 ID。

`WaitForAlignment(mode, angle, x, y, dt, speed)` 是非阻塞逐帧函数：使用该模式死区更新 PID，三个偏差绝对值同时 **小于或等于** 各自死区才累计一帧，连续第 5 帧返回 `true` 并报告 `ALIGN_DONE`。任一轴越界、无效反馈、启停、模式切换、300 ms 断流或最新帧队列覆盖导致的丢帧均清零计数。第 1～4 帧全部在死区时已停车，但尚未宣布完成；没有新帧时不会增加计数。第 5 帧锁存达标状态供业务流程读取，但连续对齐仍开启，后续误差越界会继续修正，直到显式停止。帧数依据接收的有效反馈，协议没有相机帧号，发送端须保证每个相机帧只发送一次。

手动连续对齐可发送 `{CMD,VISION,ALIGN_START,模式}` 指定上述五种模式之一；无模式的旧命令和 Debug 上电默认使用 `DISC` 死区。未知模式回复 `RANGE`，多余参数回复 `FORMAT`。此命令只开启底盘闭环，相机识别仍通过 `START_REQUEST` 控制。

键盘微调：Shift+Q 左转、Shift+E 右转，按住启动，松开 Q/E 或 Shift 停止。MOVE 的 dir 支持 0=前进、1=后退、2=左移、3=右移、4=左转、5=右转；speed 为电机 RPM，stop=0 启动、stop=1 停止。单独 Q/E 仍执行 ±90° 相对旋转。

- `DEMO4`：小车停稳在暂存区、区域 1～3 号位各有一件物料且三个载物台为空时使用。按内侧到外侧依次完整执行“暂存区 1 → 载物台 3、2 → 2、3 → 1”。固件在运动前检查三个载物台状态和位姿；物料颜色临时按红色登记，动作不含底盘导航，也不能检测实际有无物料。`DONE` 表示预设动作序列执行结束，不代表传感器确认抓取成功。

- `START_ZONE` 只能为 1 或 2。Debug 中设置后同步软件估计位姿；Release 中保留现有业务状态行为。
- `ROUTE`：2～25 个节点，节点编号 0～24，5×5 蛇形地图中相邻节点必须物理上下或左右相邻。路径的第一个节点必须是小车实际所在节点。固件执行前再检查相邻性；上位机输入框仍可输入 `0-1-8`，发送时转换为逗号字段。
- `ALIGN_DATA`：三个有限浮点值，依次为角度偏差（度）、视觉 X 与 Y 偏差（视觉单位）；20 Hz 连续输入时只保留最新一帧。停止对齐后收到有效反馈也不驱动电机，并最多每秒回传一次 `{EVT,VISION,ALIGN_IGNORED,DISABLED}`；反馈不会自行重启运动。
- 首轮圆盘自动流程：路径结束后固件开启连续对齐，再发 `START_REQUEST,DISC`；电脑仅需回传 `ALIGN_DATA`。连续 5 帧达标时发 `ALIGN_DONE`，主流程随后发 `STOP_REQUEST,DISC` 停止对齐，再发 `START_REQUEST,DISC_MATERIAL`。包含首帧等待的首次达标限时为 15 秒，运动反馈断流限时为 300 ms；等待失败发 `ALIGN_FAILED,TIMEOUT`，主流程结束 DISC 请求并保持故障。手动 `ALIGN_START` / `ALIGN_STOP` 会取消业务等待，不能作为自动完成信号。
- `COLOR`：整数 1～6，依次为红、黄、蓝、绿、黑、浅蓝。DISC_MATERIAL 运行时，每条颜色反馈按当前轮次任务码判断：第一轮使用 round1_colors，第二轮使用 round2_colors，只与下标 roundProgress 对应的下一件颜色比较。匹配且当前抓取阶段已准备、底盘对齐停止、目标载物台为空时，发送 `{EVT,VISION,COLOR,GRAB,color,cargo}` 并调用 `GrabDiscMaterial(color,cargo)`。仅颜色不匹配时发送 `{EVT,VISION,COLOR,SKIP,color,cargo,COLOR_MISMATCH}`；状态阻止时发送 `{EVT,VISION,COLOR,BLOCKED,color,cargo,reason}`，reason 为 NO_TASK、NOT_WAITING、ROUND_COMPLETE、ALIGN_ACTIVE、NOT_READY 或 OCCUPIED。cargo 为 roundProgress+1，无有效进度时为 0；有有效任务和进度时，拒绝日志还打印 received、expected、cargo、reason。上位机兼容旧固件六字段 SKIP，但会提示旧回复未提供原因。`CONFIRM` 仍仅 Release 接令入队，不执行抓取。
- `GRAB`：保留显式任务校验抓取入口。cargo 为 1～3；单参数使用最近 COLOR，双参数为 color,cargo。须开启物料视觉、停止底盘对齐、进入有效任务码的抓取阶段，颜色及载物台顺序须匹配，载物台须为空。拒绝原因保留 NOT_ACTIVE、ALIGN_ACTIVE、NO_COLOR、NOT_WAITING、ORDER、COLOR_MISMATCH、OCCUPIED；非法参数为 RANGE，参数数量错误为 FORMAT。Debug 不绕过任务码校验。
- Release 首轮还要求主流程已完成圆盘导航及定位，再进入物料识别等待；提前手动开启 DISC_MATERIAL 不绕过准备条件。COLOR 此时报告 BLOCKED,NOT_READY，显式 GRAB 回复 NOT_WAITING。Debug 手动测试不要求该准备状态，但仍检查对齐停止和载物台为空。
- 有效 GRAB 先回复 `{RSP,VISION,GRAB,ACK,color,cargo}`；COLOR 匹配先发送 COLOR,GRAB 决策。两者共用抓取完成路径：`GrabDiscMaterial` 返回 true 后发送 `{EVT,VISION,GRAB,DONE,color,cargo}` 并推进 roundProgress；返回 false 后发送 FAILED，保留进度和目标颜色。true 表示预设动作完成且已登记载物台颜色，不代表传感器确认实物。第三件成功后关闭 DISC_MATERIAL；Release 主流程进入下一放料阶段，Debug 保留完成进度。上位机兼容旧 REQUESTED 回复，但它不表示完成。强制入口不推进任务进度。
- `VISION START_REQUEST`：恰好一个模式参数，只允许 `DISC`、`DISC_MATERIAL`、`WORK_AREA`、`WORK_AREA_LOADED`、`CORNER`。未知模式回复 `RANGE`，缺参数或多参数回复 `FORMAT`，均不发送视觉请求事件。Debug 和 Release 均可调用。`DISC` / `WORK_AREA` / `WORK_AREA_LOADED` / `CORNER` 在发送启动事件前清空旧反馈并同步开启连续 PID 对齐，电脑只需发送 `ALIGN_DATA`；首次连续 5 帧达标限时 15 秒，运动反馈断流限时 300 ms，达标后连续闭环仍开启。`DISC_MATERIAL` 仅发送视觉请求。重复开启定位会重置等待计数；手动请求不主动切换业务阶段。
- `VISION STOP_REQUEST`：模式、参数数量、运行模式及错误规则与 `START_REQUEST` 相同。先回复 ACK，再同步停止属于 `DISC` / `WORK_AREA` / `WORK_AREA_LOADED` / `CORNER` 对应模式的底盘对齐，清零 PID 和等待计数，再发出 STOP_REQUEST 事件。结束旧模式不会停止随后开启的另一定位模式。`DISC_MATERIAL` 仅请求结束视觉。无模式 `ALIGN_START` 的连续对齐用 `{CMD,VISION,ALIGN_STOP}` 停车；显式指定上述四种模式的手动连续对齐也可通过对应 STOP_REQUEST 停止。
- `POSE GET`：`x,y,theta,h,l` 是软件维护的理想/开环值，单位依次为 mm、mm、度、mm、mm；`turret` 是 2 号转台舵机现场读取的真实多圈角度，`pawl` 是 1 号夹爪舵机现场读取的单圈角度（度）。读取失败回复 `{RSP,POSE,GET,ERR,SERVO_READ,2}` 或 `1`，不会用旧值冒充实测。
- `ARM MOVE2` 的转台线缆机械范围为真实多圈角度 **-180°～360°**。固件先读取真实多圈角度；若当前已越界或读取失败，跳过转台动作。否则仅从范围内的等效目标（相差 360°）中选择最近的一项，绝不向范围外下发转台目标。转台输入也限制为 -180°～360°；夹爪仍为 -360°～360°。搬运 Demo 遇到转台失败会停止并报告 `FAILED`，失败后需人工确认物料位置。调试 `SERVO` 指令的 2 号舵机走相同保护，成功时回复 `{EVT,ARM,SERVO,ISSUED}`；不允许广播 ID 254 绕过保护。
- 转台开机参考位由使用者保证：**上电前必须实际处于 0°～180°且线缆未缠绕**。固件等待舵机启动，并对开机读数及重置后回读做有限重试。若开机多圈读数不在 0°～180°，固件只在单圈读数处于 0°～180°并与多圈余角吻合时，对 2 号舵机执行“停止并释放锁力 → 重置圈数 `0x11` → 当前位置恢复锁力 → 多圈回读验证”。例如多圈 817.3°、单圈 97.3°，成功后多圈约为 97.3°；整个过程不发送旋转目标。任何校验失败都会锁住自动转台动作，需检查机械姿态并重新上电；动作报错中的 `INITIAL_MULTI_READ`、`SINGLE_READ_OR_MISMATCH`、`RESET_VERIFY` 分别表示初始多圈读取失败、单圈读取或角度比对失败、重置后回读未验证。单圈角度相同无法证明线缆没有额外缠绕，因此开机参考位的物理保证不可省略。
- `POSE SET`：x、y 为 0～2400 mm，theta 为有限角度。`GOTO_REL`：x、y 各为 ±3394.113 mm，theta 为 ±360 度；都是**相对移动量**，ACK 后执行动作。
- `CHASSIS MOVE_POSITION`：调用 `MovePosition(x,y,theta,speed)`，x/y 是移动前车身坐标系中的终点位移，各为 ±3394.113 mm；theta 是本次转角（±360°，正值左转），speed 是最大轮速（1～5000 RPM，驱动器按整数执行）。四輪同步启动，平移和旋转同时进行，混合运动走圆弧。参数数量错误回复 FORMAT，非法数值回复 RANGE，Release 回复 MODE，锁失败回复 LOCK；均不执行位置动作。有效参数先关闭视觉对齐并独占底盘运动锁，再回复 ACK；函数参数/标定检查失败会报告 FAILED，成功报告 DONE,ESTIMATED。全圈旋转同时平移不能用单段圆弧表示，函数拒绝。ACK 只表示接令，DONE,ESTIMATED 只表示下发和估算等待结束，不是实测到位。等待期间串口控制任务阻塞，后续串口指令延后处理；结束后视觉对齐保持关闭。上位机入口为“底盘 → 麦轮位置移动”，结束后可查询 POSE GET 回读底盘理想位姿。
- `CHASSIS MOVE`：方向 `0` 前、`1` 后、`2` 左、`3` 右；速度 0～1000；`stop` 为 0 或 1。
- `ARM MOVE1`：高度 `-1` 或 0～175 mm，伸出 `-1` 或 0～170 mm；`ARM MOVE2`：转台目标 -180～360 度、夹爪目标 -360～360 度，`-1` 表示跳过该轴；两种动作速度均为 1～1000。`SERVO`：ID 0～253 整数，角度 -135～135 度；ID 2 通过转台线缆保护。`ENABLE`：0 或 1。
- 参数个数错误回复 `FORMAT`，越界回复 `RANGE`，模式不符回复 `MODE`，未知动作回复 `UNKNOWN_ACTION`；这些错误均不触发新运动。机械臂动作仍受现有未标定位姿安全门限制。
- `ARM INIT_START` / `INIT_LOOK` / `INIT_LOOK2` 不带参数，上位机“机械臂 → 机械臂初始化”提供三个按钮。起始姿态目标为高度 150 mm、伸出 40 mm、转台 -55°、夹爪 0°；观察姿态最终为高度 120 mm、伸出 0 mm、转台 90°、夹爪 0°。观察姿态2调用 `InitArm_look2()`：先升到 170 mm、夹爪设为 0°，等待 2 秒，再收回伸出至 0 mm、转台转至 90°、夹爪打开至 80°，速度均为 150。沿用固件动作顺序与延时，任一步下发失败即结束并报告 `FAILED`；`ACK` 仅表示接令，`ISSUED` 仅表示动作指令已下发，不证明物理到位。执行期间串口命令任务会等待该流程返回。

## 3. 主动事件与路径交互

| 旧回复或事件 | v2 事件 / 回复 | 含义 |
|---|---|---|
| `{way:WAITING}` | `{EVT,NAV,ROUTE_WAITING}` | Release 主状态机等待路径 |
| `{way:start-end?}` | `{EVT,NAV,ROUTE_REQUEST,start,end}` | 上位机应返回完整路径 |
| `{way:RUNNING}` | `{EVT,NAV,ROUTE_RUNNING}` | 开始执行 |
| `{way:DONE,ESTIMATED}` | `{EVT,NAV,ROUTE_DONE,ESTIMATED}` | 开环估计完成 |
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

五个模式值是固定 ASCII 标识，不附带区域编号或任务码。当前粗加工区和暂存区共用一套机械臂工位位姿；机载电脑需结合当前业务阶段区分所在区域。固件提供 `requestVisionStart(VisionStartMode)` 发送接口；当前 Release 流程已在首轮圆盘区自动切换 `DISC`/`DISC_MATERIAL`，并在第一批粗加工区自动切换 `WORK_AREA`/`WORK_AREA_LOADED`。其余阶段要等对应的导航、到位及视觉切换时机接入状态机后才会自动发送。仓库内调试上位机的“视觉”页提供五种请求按钮，显示发送、接令、请求事件及错误状态，同时保留原始日志，不包含相机算法。`CORNER` 本次定义的是启动请求；角点结果与自动返家完成判定尚未接入固件，不能把请求事件当作到达确认。

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
