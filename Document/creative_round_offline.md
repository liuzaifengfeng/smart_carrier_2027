# 创意赛离线改规则练习手册

以下 52 题是基于这辆车现有固件和串口协议设计的练习题。题里的节点、时间和速度是改法示例，现场按裁判给定的条件取值。每次演练只选一题，便于从日志确认哪项改动生效。

## 断网前准备

1. 在参赛电脑上保留完整工程、可运行的 PlatformIO 工具链及依赖、Python 和已安装的 pyserial、机载端的导航及视觉程序，以及已验证的基线固件。正式路径由机载端接收 `ROUTE_REQUEST` 后回传 `NAV ROUTE`，断网演练时要确认机载端的路径回传仍能工作。
2. 提前断开互联网，实测一次本地编译、USB 串口上传、`SYS HELLO` 握手、机载端路径回传和视觉反馈。桌面工具可从项目根目录运行 `python upper_computer.py`。固件串口使用 115200。
3. 保存当前实车参数和基线版本。现场改数值优先查 `robot_parameters.json` 中的对应名称，只改 `value`。编译烧录后的默认值重启仍生效。允许临时调参时，可在 Debug、视觉关闭和运动空闲状态下走参数页写入并回读，这些写入存于 RAM。
4. 修改后用 VS Code 的源代码管理检查差异，再做与题目对应的小范围试验。路径先看请求的起终点和经过节点，机械臂先看单件实物，流程看作业完成事件和下一阶段。串口 ACK 表示收令，验收要等实际结果。

PowerShell 中，在项目根目录可执行下列命令。上传前关闭占用同一 COM 口的上位机，`COMx` 换成比赛电脑上的下载口。

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e esp32-s3-devkitm-1
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e esp32-s3-devkitm-1 -t upload --upload-port COMx
```

找代码时用 VS Code 全局搜索 `Ctrl+Shift+F`，文件范围填 `src/**`。拿到函数名后按 `F12` 看定义，按 `Shift+F12` 查调用点。路径和任务格式变更还要搜机载项目及实际使用的上位机。

相关资料见[固件源码索引](src_index.md)、[参数说明](robot_parameters.md)和[串口协议](serial_protocol.md)。

## 每道题怎样在 VS Code 练一遍

在 VS Code 打开本文，按 `Ctrl+Shift+O` 后输入题号或题名，可跳到对应标题。

1. **把题意写成可验证的结果。** 例如「两区互换」写成「粗加工节点 22、到站 180°，暂存节点 10、到站 270°」。分开写车体停靠点、朝向、取放工序和完成条件。
2. **保存现场基线。** 用已经验证的工程副本练习，保留烧录前的固件和实车参数。打开左侧「源代码管理」先看哪些文件已有改动，保存原值后再修改题目对应部分。
3. **按示例放到指定作用域。** 代码块是原函数内的替换或插入片段。通过 `Ctrl+P` 打开题中指定文件，在里面按 `Ctrl+F` 搜函数名，找到旧语句后再按题意替换。参数值的示例保留周围 JSON 层级。
4. **一次只验证一个假设。** 先读串口确认参数或请求，再试一段路线、一次对齐或一件物料，最后跑整个受影响阶段。实车验收要以观测到的停靠、走位和抓取结果为准。
5. **从失败处反查。** 路线看 `ROUTE_REQUEST → ACK → ROUTE_PATH → ROUTE_DONE`；放料看视觉结果、对应作业号的 `WORK_PLACE` 和整组终态；完赛看 `RUN_START` 与 `RUN_RESULT`。有 ACK 却无后续事件时，沿这一条链找最后出现的一步。

如果上位机已导出纯文本串口日志，可用 PowerShell 快速筛选这一轮。把文件名换成自己的日志。

```powershell
Select-String -Path .\run.log -Pattern 'ROUTE_REQUEST|ROUTE_PATH|ROUTE_DONE|ALIGN_RESULT|WORK_PLACE|WORK_PICK|RUN_START|RUN_RESULT'
```

## 场地与导航

### 01 指定只能从 2 号启停区出发

**定位与修改。** 搜 `NAV START_ZONE`、`RuntimeCompetition::zone` 或 `zone()override`。现有流程支持两个区，现场把实体拨杆置于 2 区并按流程启动。若赛题允许串口选区，发送 `{CMD,NAV,START_ZONE,2}`，准备完成后再使用 `SYS START`。题目若要求固件拒绝选 1 区，再同时检查实体面板和串口选区入口，避免一个入口仍能选 1 区。

**照着操作。**

1. 上电后，在 115200 波特率的串口工具里发 `{CMD,SYS,PANEL_GET}`。拨杆拨到 2 区并松开按钮时，应回 `{RSP,SYS,PANEL_GET,OK,0,1,2}`。末三项分别是按钮电平、拨杆电平、按拨杆计算的区号。
2. **实体启动**用拨杆进入 Release，待收臂和航向标定完成后按实体按钮。**串口启动**在规则许可时，从 Debug 发 `{CMD,NAV,START_ZONE,2}`，核对 `{RSP,NAV,START_ZONE,OK,2}`，再发 `{CMD,SYS,RELEASE}`。等待车辆准备完成，看到 `WAIT START` 再发 `{CMD,SYS,START}`。实体启动会锁定按钮按下时的拨杆区号。
3. 如需程序强制只用 2 区，在 `competition_flow.h` 的 `case Phase::START_WAIT_ZONE` 将原来的条件分支替换成下面这段，保持 `io.zone()` 每次只读取一次。还要在 `serial_commands.inc` 的 `PollDebugBootButton()` 将 `const bool startRequested=panelStartButton.takeRequest();` 改为 `const bool startRequested=panelStartButton.takeRequest() && zone==START_ZONE_2;`。这样 1 区按键不会把 `panelStartZone` 锁为 1，拨到 2 区后松开再按可以选区。

```cpp
if (const uint8_t zone = io.zone() == 2 ? 2 : 0) {
    homeZone = zone;
    phase = Phase::START_PREPARE_LIDAR;
}
```

4. 如果要求串口收到选区 1 时也明确回复拒绝，在 `serial_commands.inc` 的 `NAV START_ZONE` 合法数字检查之后、赋值 `currentStartZone = ...` 之前加 `if (zone != 2) { serialError(frame, "RANGE"); return; }`。保留 `SYS PANEL_GET` 查询，它用来观察拨杆。

**判读。** `PANEL_GET` 只代表面板输入，正式选区以扫码发出的 `{EVT,NAV,ROUTE_REQUEST,0,2}` 为准。测试错误拨杆档位时，应停在选区准备阶段，拨回 2 区并松开再按，确认能按正常流程继续。测试期间用车辆实际停放位置核对区号。

**验证。** 看面板或串口回报区号 2，首个扫码路线请求从节点 `0` 出发。核对实际启停区、初始位姿和航向一致。

[选区逻辑](../src/serial_commands.inc) · [正式启动](../src/main.cpp) · [启停节点](../src/robot_runtime.h)

### 02 交换粗加工区和暂存区

**定位与修改。** 在 `robot_runtime.h` 搜 `COARSE_AREA_NODE`、`TEMP_AREA_NODE`。沿用两个物理工位原进站朝向时，将粗加工从 `10 / 270°` 改为 `22 / 180°`，暂存从 `22 / 180°` 改为 `10 / 270°`。`main.cpp` 用这些常量填充正式地图，`endpointHeading()` 用同一组配置选到站朝向。独立测试如果直接构造默认 `CompetitionMap`，也要核对测试初始值。

**逐行改配置。** VS Code 按 `Ctrl+P` 打开 `src/robot_runtime.h`，搜索四个常量并在原位置替换右边的值。如下四行在文件里分开摆放，保留相邻的圆盘、扫码和返家配置。

```cpp
constexpr uint8_t COARSE_AREA_NODE = 22;
constexpr uint8_t TEMP_AREA_NODE = 10;
constexpr float COARSE_AREA_HEADING = 180.0f;
constexpr float TEMP_AREA_HEADING = 270.0f;
```

沿 `main.cpp` 的 `map.coarse=COARSE_AREA_NODE` 回看赋值，再到 `motion_executor.cpp` 的 `endpointHeading()` 核对终点选择。正式编译只改这四处即可把站点和进站航向一起换掉。如果两区互换后工件相对车体的摆位也发生变化，再按第 17 题标定机械臂位姿。

**分段验收。** 第一次只检查场地方位和日志。从圆盘离开时看到 `{EVT,NAV,ROUTE_REQUEST,14,22}`，留意实车到节点 22 时朝向为约 180°。粗加工放下并取回后检查 `{EVT,NAV,ROUTE_REQUEST,22,10}`，到节点 10 时朝向约 270°。第二轮的首条请求为 `10,14`。如果请求正确而回传起终点错误，先查机载端路线表。

**验证。** 查请求顺序 `14→22→10`、第二轮 `10→14`。看两区车头朝向及放料动作，粗加工区仍有取回阶段。

[工区节点与航向](../src/robot_runtime.h) · [到站航向](../src/motion_executor.cpp) · [业务流程](../src/competition_flow.h)

### 03 扫码牌移到另一处

**定位与修改。** 搜 `SCAN_AREA_NODE`、`endpointHeading` 和 `slowApproachEdge`。先选可达的新扫码节点，并修改 `SCAN_AREA_NODE`。再按扫码牌朝向处理 `SCAN_TASK` 专用终点航向分支，留意目前它对倒数第二个节点 `7` 有特殊处理。根据新来向复核 `H30PrepareNodePath()` 的慢速段、`scanAtNode()` 的后退方向和距离，以及扫码后从退后位置继续出发的首段补偿。扫码枪还要能在新姿态读到码。

**演练参数。** 假定现场确认扫码牌移到节点 `3`，到站需要车头朝 `270°`。打开 `robot_runtime.h`，把 `SCAN_AREA_NODE` 改为 `3`，并在区域航向常量附近增加 `constexpr float SCAN_AREA_HEADING = 270.0f;`。打开 `motion_executor.cpp`，把 `endpointHeading()` 的第一条 `SCAN_TASK` 专用分支替换为下行。正式比赛用实测节点和朝向替代演练值。

```cpp
if (r.kind == MotionKind::SCAN_TASK) return SCAN_AREA_HEADING;
```

**按顺序确认依赖。**

1. `beginPath()` 把规划终点航向换成料盘扫码角 `wrap(plannedHeading+90)`。到站后先试静止读码，用实物核对扫码器的朝向与读码距离。
2. `H30PrepareNodePath()` 中 `path[count-1]==SCAN_AREA_NODE` 会跟随新值。函数内部用于慢速段边界的 `3`、`7` 是原进站方案的特定节点，分别从两个启停区计算新路径，再决定这些节点是否适合作为提前限速点。
3. `scanAtNode()` 的 `BeginPositionMove(0,-100,0,200)` 是**车体坐标**后退 100 mm，轮速 200 RPM。先确认有后退空间，必要时按裁判布置改方向和距离。收到有效码后再核对 `scanRetreated` 的出站偏移是否反映实际位置。
4. 演练应依次遇到 `ROUTE_REQUEST,4,3` 或 `ROUTE_REQUEST,0,3`，读码成功后再请求 `3,14`。刻意遮挡一次码，验证 `SCAN_WAIT → SCAN_RETREAT → SCAN_SWEEP`，撤掉遮挡后能完成扫码。

**验证。** 从两区各试一次，分别覆盖途中读到码和到站后仍未读到码。检查停靠、摆扫空间和扫码点到圆盘的首段。

[导航到扫码节点](scan_position.md) · [扫码动作及终点航向](../src/motion_executor.cpp) · [进站慢速段](../src/continuous_trial.cpp)

### 04 圆盘移动到另一侧或改变观察朝向

**定位与修改。** 搜 `DISC_AREA_NODE` 和 `DISC_AREA_HEADING`，把它们设为新停靠节点和朝向。再搜 `MergeDiscYIntoFirstStraight`、`ReadDiscYOffset` 和 `BeginDiscYReturn`。现有离站补偿沿地图 Y 轴，且根据首个相邻节点选恢复方式。若新摆放让对齐主要沿另一地图轴发生，按新轴重新核对位置投影和回退向量。工位相对车身的方向或距离也变了时，单独检查 `LOOK2`、`LOOK3` 及三个圆盘抓取位置。

**把修改分成地图、视觉和离站三处。**

1. 用地图确认新圆盘旁的停靠节点和车头方向，在 `robot_runtime.h` 改 `DISC_AREA_NODE` 与 `DISC_AREA_HEADING`。例如同一停靠点需要改为朝 180° 时，只把 `DISC_AREA_HEADING` 改为 `180.0f`。接着搜引用，在 `TickVisualAlignment()` 能看到圆盘角度误差使用 `remainderf(heading-DISC_AREA_HEADING,360.0f)`，它会同步采用新值。
2. 在 `arm_motion.cpp` 找 `LOOK2`、`LOOK3` 和 `DISC_GRAB`。空载到站后先用原观察位姿查看目标是否入镜，然后逐个验证位置 1、2、3 的抓取。区域节点是**地图上的车体停靠点**，圆盘抓取位姿是**相对车体的机械臂目标**，根据实测分别处理。
3. 在 `motion_types.h` 找 `MergeDiscYIntoFirstStraight()`，现有判据是首个相邻节点为 5 或 15。圆盘换站后，按新首段方向重新确认哪些首段能吸收地图 Y 偏移。若物理纠偏方向改成地图 X，继续追 `ReadDiscYOffset()` 和 `BeginDiscYReturn()` 的世界轴投影与返回目标，连同 `beginPath(startYOffset)` 的参数含义一起设计。只换朝向且仍沿地图 Y 纠偏时，可复用原投影数学。
4. 第一次留原取放流程，只测空车靠近和离站。通过 `DISC_RETURN_POSITION` 或 `DISC_Y_OFFSET` 判定用了哪条分支，再实测第一段是否回到计划线。随后接一件抓取，确认状态从观察、对齐、抓取到离站都顺利。

**验证。** 做一次「导航到新圆盘 → 对齐 → 抓一件 → 离站」，看 `DISC_HEADING`、离站位置和第一段路线。若允许两个来向，分别试首段的补偿分支。

[圆盘配置](../src/robot_runtime.h) · [圆盘航向误差](../src/robot_runtime.cpp) · [离站补偿](../src/motion_executor.cpp) · [三位置抓取](../src/arm_motion.cpp)

### 05 禁行一段路

**定位与修改。** 先在机载端把禁行边从路网或允许边表中移除，并重算所有涉及该边的任务路线。双向禁行时两个方向都处理。若裁判要求固件也拒收，沿 `serialRoute()` 查提交链，在正式路线校验中按相邻节点对检查该边。全局搜 `ROUTE_REQUEST`，确认改的是实际生成或回传路径的一端。

**用具体报文练习。** 假设双向禁行 `12↔13`。项目协议里的基线路径为 `{CMD,NAV,ROUTE,2,7,12,13,14}`。机载端把 `12→13` 和 `13→12` 从允许边表移除，重算任意经过该边的路线。一条在现有网格上相邻的绕行示例为 `2,7,12,17,16,13,14`，执行前按实际障碍验证。

**固件加拒收点。** 如题目要求主控也检查，在 `serial_commands.inc` 搜 `static void serialRoute`。放在把各字段读入 `path[]` 的循环之后、`if(serialDebugMode)` 之前，可以同时覆盖 Debug 和 Release。示例中的拒绝原因沿用既有 `RANGE`。

```cpp
for (size_t i = 1; i < count; ++i) {
    const uint8_t a = path[i - 1];
    const uint8_t b = path[i];
    if ((a == 12 && b == 13) || (a == 13 && b == 12)) {
        serialError(frame, "RANGE");
        return;
    }
}
```

**三次验收。** 收到 `ROUTE_REQUEST,2,14` 后发送含禁行边的报文，应回 `{RSP,NAV,ROUTE,ERR,RANGE}`，且没有该路线的 `ROUTE_RUNNING`。窗口内回传合规路径，期待 `ACK,7`，随后 `ROUTE_PATH` 带七个节点。再测一次反向含禁行边的路径。Release 回传需要落在每次请求的 2 秒窗口内，窗口外的 `NOT_WAITING` 代表时序问题。

**验证。** 手动构造一条含禁行边的路径，再给一条绕行路径。前者应按要求拒绝，后者要满足起终点正确、节点相邻，而且车体运动包络绕开禁区。

[路线收发协议](serial_protocol.md) · [串口路由入口](../src/serial_commands.inc) · [固件路径校验](../src/continuous_trial.cpp)

### 06 正式节点导航增加平移限速

**定位与修改。** 在 `robot_parameters.json` 搜 `底盘/节点导航速度(mm/s)` 和 `底盘/节点导航中间路段速度(mm/s)`。例如两类路段都限 `600 mm/s`，把现有值中超过上限的项降至 `600`。如果赛题要求运行时也不得配置超上限，在正式导航消费速度的位置统一限幅。若规则还覆盖扫码移动或视觉对齐，继续查各自的 RPM 设置并按实车换算验证。

**把两个独立参数都改到题面上限以内。** 在 `robot_parameters.json` 分别搜索以下两个完整名称，只修改它们各自的 `"value"`。

| 参数名 | 仓库示例值 | 规则上限 600 mm/s 时填写 |
| --- | --- | --- |
| `底盘/节点导航速度(mm/s)` | 3000.0 | 600.0 |
| `底盘/节点导航中间路段速度(mm/s)` | 3000.0 | 600.0 |

如果允许临时调试，在 Debug 模式停视觉、等运动空闲，参数页「一键读取 → 修改并发送 → 回读确认」。正式重启仍保留时，编辑 JSON 后构建并烧录。配置在**下一条路线启动**时读取。

**题面要求上限不可被调参绕过时。** 在 `continuous_trial.cpp` 的 `H30PrepareNodePath()` 搜 `cfg.maxRpm`。在 `Config cfg;` 这一行之前加 `speedMmS = fminf(speedMmS, 600.0f);`，使轮速换算和后面的 `formalSettings.speedMmS` 使用同一限幅值。再把 `formalSettings.middleSpeedMmS` 的赋值右侧换成 `fminf(NODE_PATH_MIDDLE_SPEED_MM_S, 600.0f)`。这两处共同覆盖正式导航的外圈和中间路段。

**分两段测。** 日志分别查 `Profile`、`Middle profile`。实测时在有足够直线和安全余量的路段，使用距离和时间检查速度，短段平均值可低于上限。分别覆盖外圈和中间直线。若赛题按实测瞬时速度判分，用实测反馈或测速工具核对，平均路段速度只能做辅助。

**验证。** 分别测外圈直行和中间路段，核对写入值和实测距离/用时。若也要求转向限速，分别验证角速度。

[速度参数](../robot_parameters.json) · [正式导航配置](../src/continuous_trial.cpp) · [底盘控制](../src/node_position_control.cpp)

## 任务输入和分配

### 07 裁判允许口头公布任务码，改为手动录入

**定位与修改。** 搜 `TASK SET`。固件已有手输入口，可以在正式等待启动时用桌面上位机录入，或在 `READ_TASK` 尚未取得任务码时发送 `{CMD,TASK,SET,156+123+516+231}`。它会调用与扫码相同的解析器。手输默认仍保留去扫码节点的路线。赛题明确取消扫码站时，再把 `SCAN_TASK` 阶段替换成等待有效任务码，并让第一轮到圆盘的路线从车辆实际所在节点出发。

**用现有命令完成。** 在等待开局时，用桌面上位机录入 `156+123+516+231`，或通过串口发送下列一行并带换行。第一段 `156` 是第一轮抓取顺序，第二段 `123` 是第一轮工位顺序。

```text
发送 {CMD,TASK,SET,156+123+516+231}
预期 {RSP,TASK,SET,OK,156+123+516+231}
```

1. 回包为 `ERR,RANGE` 时，对照任务码格式。回包为 `ERR,BUSY` 时，检查是否已确认有效码并开始抓取。
2. 找到 `serial_commands.inc` 的 `frame.is("TASK", "SET", 1)`。接受的阶段是 Debug、正式等待启动、以及尚未取得任务码的 `STATE_READ_TASK`。接受时给 `currentTask` 赋值并置 `taskReceived=true`。
3. 打开 `motion_executor.cpp` 的 `pollTaskCode()`，看同一个 `taskReceived` 怎样让扫码作业使用手输入的码。现有流程仍先完成去扫码点的路线。到站后核对圆盘抓取的第一件与确认过的任务码一致。
4. **如果题面还取消前往扫码点**，在 `CompetitionIO` 增加查询有效任务是否就绪的接口，由 `RuntimeCompetition` 用现有 `taskCodeMux` 读取 `taskReceived`。`SCAN_TASK` 阶段改成轮询该接口，就绪时调用原来的 `io.taskReady()` 并进入 `STATE_GRAB_ROUND1`。同时把 `DISC_NAVIGATE` 第一轮起点从 `map.scan` 换成车辆实际所在的启动节点，并把该节点和经过雷达准备后的位姿余量核对一遍。

**验证。** 确认返回 `RSP,TASK,SET,OK`，然后检查圆盘第一件是否为颜色 `1`。正式抓取中尝试再次提交，应被拒绝为 `BUSY`。

[任务命令](../src/serial_commands.inc) · [扫码阶段](../src/competition_flow.h) · [任务解析](../src/task_code.h)

### 08 任务码分隔符从加号换成短横线

**定位与修改。** 例如新码是 `156-123-516-231`。在 `IsTaskCodeValid()` 修改第 `3 / 7 / 11` 个零基下标处的分隔符。四段仍各三位时，`parseTaskCode()` 的字段下标可以沿用。桌面端还要同步 `validate_task_code()` 的切分、提示和回包校验。现场有其他提交端时，同步它的输入格式。用英文半角 `-`。

**主控和桌面修改各一处。**

1. 打开 `src/task_code.h`，在 `IsTaskCodeValid()` 里只替换这条分隔符条件。`parseTaskCode()` 仍从 `0、4、8、12` 四个起始偏移读取各三位。

```cpp
&& text[3] == '-' && text[7] == '-' && text[11] == '-'
```

2. 在 `upper_computer.py` 搜 `def validate_task_code`，把 `code.split("+")` 改为 `code.split("-")`，提示示例一起换为 `156-123-516-231`。这个校验器既用于发送也用于读回，改完后在实际使用的串口界面各走一次。若使用网页快速命令，其默认示例在 `commands.ts`。
3. 分别从扫码枪和串口送新码。用下表测试。检查结果后再做抓料动作。

| 输入 | 只允许新格式时的预期 |
| --- | --- |
| `156-123-516-231` | 通过 |
| `156+123+516+231` | 拒绝 |
| `156-123-516-23` | 拒绝 |
| `156-113-516-231` | 拒绝，第一轮工位重复 |
| `176-123-716-231` | 拒绝，颜色 7 越界 |

可以先改 `tests/test_task_code.py` 的练习用例，再用 `python -B -m unittest discover -s tests -p test_task_code.py -v` 验证桌面端。本命令应在改规则的练习副本里运行，固件仍需烧录后实测。

**验证。** 用新码测试实际扫码和串口录入。再试错分隔符、少一位、颜色越界和重复工位。若题目要求只接受新格式，旧版加号码应拒绝。

[固件任务码](../src/task_code.h) · [桌面校验](../upper_computer.py) · [任务码测试](../tests/test_task_code.py)

### 09 两轮颜色集合可以不同，并按工位编号叠放

**定位与修改。** 在 `IsTaskCodeValid()` 去掉第二轮三个颜色必须属于第一轮集合的判断，保留题目仍要求的轮内颜色范围和去重检查。同步桌面端 `validate_task_code()`。再改 `TEMP_PLACE` 中按第一轮颜色回查工位的逻辑，让第二轮按裁判指定的规则落位。例如题目说以第四段工位为准，就让 `request.positions` 保留第四段。现场若另有映射表，按题面实现那个映射。

**只放宽两轮之间的限制。** 在 `IsTaskCodeValid()` 找这三条比较并删去，每一条都在检查第二轮某个颜色是否出现于第一轮。保留前后的 `TaskColorsValid(...)` 与 `TaskPositionsValid(...)`。

```cpp
&& (text[8] == text[0] || text[8] == text[1] || text[8] == text[2])
&& (text[9] == text[0] || text[9] == text[1] || text[9] == text[2])
&& (text[10] == text[0] || text[10] == text[1] || text[10] == text[2])
```

然后在 `upper_computer.py` 的 `validate_task_code()` 删除 `if set(parts[0]) != set(parts[2])` 及其抛错行。旧校验中的轮内重复和位置编号检查继续保留。若规则指定新的颜色编号，再查视觉端的类别和圆盘颜色范围，范围修改属于另一项规则。

**让新颜色有确定工位。** 示例规则以第四段作为第二轮暂存位置，可采用第 10 题的删改。准备三组断言：`123+123+456+231` 应通过，`123+123+446+231` 应因第二轮重复颜色拒绝，`123+123+456+221` 应因重复工位拒绝。实车观察颜色 `4→5→6`，对应二层工位 `2→3→1`。

**验证。** 用 `123+123+456+231`，确认第二轮能按颜色 `4、5、6` 抓取，之后依次叠到 `2、3、1`。检查三件都有目标，且符合赛题是否允许异色叠放。

[固件解析](../src/task_code.h) · [桌面校验](../upper_computer.py) · [叠放映射](../src/competition_flow.h)

### 10 两轮颜色仍相同，第二轮暂存改用第四段工位码

**定位与修改。** 在 `TEMP_PLACE` 找到第二轮用第一轮颜色位置重新填 `request.positions[i]` 的双循环。仅取消这次位置覆盖。`io.task(request,secondRound)` 已经填好第四段工位，保留二层高度、第二轮颜色顺序及 `WORK_AREA_LOADED` 视觉模式。

**先算一遍当前代码。** 输入 `156+123+516+231` 时，第一轮颜色和工位配对是 `1→1`、`5→2`、`6→3`。现在会把按颜色 `5、1、6` 抓来的第二批按第一轮映射叠到 `2、1、3`。题面改按第四段时，需要得到 `2、3、1`。

**在源码做最小删改。** 在 `competition_flow.h` 搜 `case Phase::TEMP_PLACE`，找到 `io.task(request,secondRound);`。紧接的 `if(secondRound) { MotionRequest firstRound; ... }` 整段就是重映射，删这一个完整块。保留取任务数据和下面两行。

```cpp
io.task(request, secondRound);
request.layer = secondRound ? 2 : 1;
request.ringCorrection = !secondRound;
```

**看实际放置记录。** 日志 `{EVT,MATERIAL,WORK_PLACE,工位,作业号}` 同时会出现在粗加工和暂存。拿到第二轮暂存作业号后只看该组，顺序应为 `2、3、1`，第一轮暂存仍为 `1、2、3`。确认第二轮的对齐模式仍是 `WORK_AREA_LOADED`，下放高度还是第二层。

**验证。** 用 `156+123+516+231`。第二轮颜色顺序仍为 `5、1、6`，粗加工放置和取回仍用 `231`。暂存工位从当前的 `213` 改为 `231`。

[第二轮落位](../src/competition_flow.h) · [当前任务映射示例](RUN_flow.md)

### 11 三件物料改按 1→2→3 号载物台装盘

**定位与修改。** 搜 `CargoForGrabIndex`。当前表达式 `3-index` 把零基进度 `0、1、2` 映射到 `3、2、1`。改为 `1+index` 即得到新顺序。用 `Shift+F12` 检查圆盘选货位、粗加工取回、批量放料、到站预选料盘和完成进度是否都跟着变。现有机械臂让位逻辑还会判断相邻货位占用，实车需按新顺序检查。

**修改函数表达式。** 在 `disc_material_decision.h` 找原函数，将 `return 3 - index;` 换成下列实现。调用方传入的 `index` 为零基进度。

```cpp
constexpr uint8_t CargoForGrabIndex(int index) { return 1 + index; }
```

1. 按 `Shift+F12` 查引用。至少看 `serial_commands.inc` 的圆盘选盘、`arm_motion.cpp` 批量转运的 `cargos[i]`、`motion_executor.cpp` 作业完成后的货位映射、`competition_flow.h` 到站时预选盘号。它们都应按新顺序计算。
2. 保留 `cargoTrayAngles` 的物理盘号索引，1 号盘仍对应数组第 0 项。`DEMO_PLACE` 自带顺序，实车要使用正式任务或相同配置验证。
3. 在盘上标记 `1、2、3`，按任务放入三种能肉眼区分的物料。逐件观察 `cargoPlatforms` 对应的颜色登记和真正所在盘，确认粗加工取回后重新按此顺序装盘。
4. 用第二件测试一次邻盘已有物料的让位动作。确认预选盘、抓取后登记和下一站卸下的是同一物件。查 `AdvanceDiscGrabProgress`，作业完成且盘号匹配才会增加进度。

**验证。** 分别记录圆盘抓取、粗加工取回和暂存卸货时的台号，确认物料登记和真正的所在货位一致。若只要求改圆盘装盘，需分开设计装盘和下游寻址，避免直接改全局映射。

[货位映射](../src/disc_material_decision.h) · [取放编排](../src/arm_motion.cpp) · [完成进度](../src/motion_executor.cpp)

## 比赛流程与计时

### 12 只跑第一轮，暂存放完即结束

**定位与修改。** 在 `TEMP_PLACE` 的 `motion(request)==1` 分支内，找到第一轮完成后 `next(STATE_GRAB_ROUND2)` 的分支。将一轮赛制的结束接到现有 `STATE_DONE` 和 `io.runDone()`，按现有写法先确认状态已切换成功。若题目要求第一轮放完还要返家，就进入 `STATE_RETURN_HOME`，由已有返家结束分支完成计时。若题面仍给四段码，现有解析可以直接使用。如果只给两段码，单独定义两段格式并同步录入端。

**改动范围。** 假定裁判继续发四段任务码，只执行第一轮。`task_code.h` 继续解析原格式。在 `competition_flow.h` 的 `TEMP_PLACE` 找到调用 `io.clearProgress()` 后的三个分支，把这段结束逻辑替换为下面的代码。保留外层 `if(motion(request)==1)`。这一题中的结束条件是**第一轮暂存成功**。

```cpp
io.clearProgress();
if (HOME_RETURN_ENABLED) {
    next(STATE_RETURN_HOME);
} else {
    next(STATE_DONE);
    if (io.state() == STATE_DONE) io.runDone();
}
```

如果新题还只发 `156+123`，把它当作独立的输入格式变更。主控校验：长度 7、`text[3]=='+'`、校验第一组颜色与第二组工位。解析：仅填 `round1_colors` 和 `round1_pos`，未用的第二轮数组保持零初始化。同步桌面端的 `validate_task_code()` 及任务显示。先完成一轮流程修改，再切换输入格式，便于定位问题。

**验收。** 第三件暂存 `WORK_PLACE` 之后仍要等整组作业完成。返家关闭时随后看到一次 `RUN_RESULT`，并且后续没有 `ROUTE_REQUEST,22,14`。故意让第三件作业失败时，不能触发正常完赛信号。

**验证。** 完成第一轮三件摆放后，确认有一次 `RUN_RESULT`，且没有第二轮去圆盘的路线请求。要求返家时，结果应在归位完成后出现。

[结束分支](../src/competition_flow.h) · [计时及完成处理](../src/main.cpp)

### 13 粗加工放完三件必须等 5 秒再取回

**定位与修改。** 搜 `COARSE_WAIT_RETRIEVE`。在这个阶段把 `due=io.now()+10` 的间隔改为 `5000` 毫秒，沿用 `COARSE_RETRIEVE` 对 `due` 的轮询。这样计时从整组放料完成进入等待阶段开始。如果题面说每件都要独立等待或由裁判确认后才取，计时起点或放行条件要按题面另设。

**用一行改完整组等待。** 在 `competition_flow.h` 搜 `case Phase::COARSE_WAIT_RETRIEVE`，替换原 `due=io.now()+10;`。`due` 表示最早允许开始取回的系统时刻，单位毫秒。

```cpp
due = io.now() + 5000;
```

下一阶段已有 `if(static_cast<int32_t>(io.now()-due)<0)return;`。尚未到时只返回本次 `tick()`，到时才提交 `RETRIEVE_ROUND`。保留这行和等待阶段的 `return`，主状态机可以继续按原周期调度。

**实测时看作业和步骤。** 先在日志里找到粗加工三件放料作业的结束，再找取回作业首次开始或首次取料步骤的开始，确认两者至少相差 5000 ms。`WORK_PICK` 是取料动作后的事件，单拿它计终点会把取料耗时也算入等待。若题面规定从第三件实际松爪那一刻起算，要用松爪所在步骤记录起点，现有修改从整组结束起算。

**验证。** 从最后一件粗加工放料成功到第一件取回开始，用串口时间戳核对间隔至少约 5 秒。确认等待期间串口维护命令仍可用。

[等待和取回](../src/competition_flow.h) · [作业事件](../src/motion_executor.cpp)

### 14 取消粗加工，圆盘抓完直接送暂存

**定位与修改。** 在 `DISC_GRAB_MATERIALS` 的三件成功分支，保留结束识别和清进度。把本轮转入对应的 `STATE_PLACE_TEMP1` 或 `STATE_STACK_TEMP2`，并在确认状态切换后从 `TEMP_NAVIGATE` 起跑。这两个状态默认会进入粗加工取回阶段，需显式改变进入的子阶段。直接路线使用 `path(map.disc,map.temp,...)`，并设置 `request.discDeparture=true`，沿用现有圆盘离站补偿。保留暂存对齐和第一轮放置、第二轮叠放逻辑。

**拆成两个小补丁，全部在 `competition_flow.h`。**

1. 在 `DISC_GRAB_MATERIALS` 的 `progress>=3` 分支里，保留 `io.materialStop()` 与 `io.clearProgress()`，把原先跳到粗加工的 `next(...)` 换成下面三行。`next()` 本身会设置对应主状态的默认子阶段，所以要在它成功后覆盖为 `TEMP_NAVIGATE`。

```cpp
const auto target = secondRound ? STATE_STACK_TEMP2 : STATE_PLACE_TEMP1;
next(target);
if (io.state() == target) phase = Phase::TEMP_NAVIGATE;
```

2. 在 `TEMP_NAVIGATE` 把直接传给 `motion()` 的旧 `path(map.coarse,map.temp,...)` 改成命名请求。把整个 `case` 用花括号包住，以限定新局部变量作用域。照下例保留原来的成功才开启对齐。

```cpp
case Phase::TEMP_NAVIGATE: {
    io.display(secondRound ? "GO TEMP2" : "GO TEMP1");
    auto request = path(map.disc, map.temp, ArmObservation::LOOK, CargoForGrabIndex(0));
    request.discDeparture = true;
    if (motion(request, STATE_ROUTE_FAILED) == 1) {
        beginAlignment(tempVision);
        phase = Phase::TEMP_ALIGN;
    }
    break;
}
```

这段写法适用于**两轮全部跳过粗加工**。若只有其中一轮跳过，需按本轮是否实际经过粗加工，同时决定起点和 `discDeparture`。另外，正常导航中圆盘到粗加工有特定的 `coarseArrivalLook` 展臂时机，直达暂存后要实测 `LOOK` 展臂时是否有空间。

**核对三段路。** 第一轮抓完 `14,22`，第二轮开始 `22,14`，第二轮抓完又是 `14,22`。每轮在暂存区能看到三次 `WORK_PLACE`，中间没有粗加工取回的 `WORK_PICK`。如果首个请求还是 `14,10`，说明圆盘完成分支仍进入了旧的粗加工状态。

**验证。** 看两轮都请求 `14→22`，没有粗加工 `WORK_PLACE` 和 `WORK_PICK`，装着三件物料离开圆盘，到暂存后才卸下。第二轮仍从暂存返回圆盘。

[圆盘完成与暂存路径](../src/competition_flow.h) · [圆盘离站补偿](../src/motion_executor.cpp)

### 15 启用或禁用回家逻辑

当前源码的 `HOME_RETURN_ENABLED` 为 `false`，正式流程在第二轮暂存叠放作业成功后直接完赛。裁判要求完赛前回到启停区时，把它改为 `true`。裁判要求叠放完当场结束时，把它设回 `false`。

**在 VS Code 里修改。**

1. 按 `Ctrl+P` 打开 `src/competition_flow.h`，再按 `Ctrl+F` 搜 `HOME_RETURN_ENABLED`。找到类中的 `static constexpr bool` 定义。另一个搜索结果位于 `TEMP_PLACE` 的成功分支，用来阅读开关如何生效。
2. 在定义处选择下表所需的值，只保留一个定义。`HOME_CORNER_ALIGNMENT_ENABLED` 是另一个开关，用来决定到达返家节点后是否做角点视觉。

| 裁判要求 | `HOME_RETURN_ENABLED` | `HOME_CORNER_ALIGNMENT_ENABLED` | 完赛时机 |
| --- | --- | --- | --- |
| 第二轮叠放完就结束 | `false` | 可保留原值，通常为 `false` | 第二轮暂存作业成功后 |
| 要回启动时选定的启停区 | `true` | `false` | 返航并完成最后补位后 |
| 要回启停区并执行角点视觉对齐 | `true` | `true` | 角点对齐阶段结束，再完成最后补位后 |

例如要返家且沿用当前默认的无角点视觉方案，定义写成这样。

```cpp
static constexpr bool HOME_RETURN_ENABLED = true;
static constexpr bool HOME_CORNER_ALIGNMENT_ENABLED = false;
```

禁用返家时，第一行换成下方这一行。第二个开关只在返家流程内读取。

```cpp
static constexpr bool HOME_RETURN_ENABLED = false;
```

3. 保存后在 PlatformIO 执行 Build，再通过 USB 串口 Upload。比赛电脑没有网络时，按本文开头的 PowerShell 命令操作。这是固件编译期开关，烧录后才生效。`robot_parameters.json` 和串口参数页没有对应项。
4. 查看 `TEMP_PLACE` 的 `if(motion(request)==1)` 分支，确认代码仍在第二轮作业成功后才依据开关选 `STATE_RETURN_HOME` 或 `STATE_DONE`。如果同时采用第 12 题的一轮赛制，则由第一轮暂存成功后的结束分支使用这个开关。

**启用后的执行顺序。**

1. `HOME_PREPARE_ARM` 先收臂，随后 `HOME_NAVIGATE` 从暂存区出发。按当前地图，启停区 1 请求 `{EVT,NAV,ROUTE_REQUEST,22,4}`，启停区 2 请求 `{EVT,NAV,ROUTE_REQUEST,22,0}`。目标是开局锁定的 `homeZone`，到站航向默认 `270°`。做过第 02 题的工区互换时，返家起点跟随新的暂存节点。
2. 路线到站之后，`HOME_PREPARE_VISION` 仍会执行 `HOME_OBSERVE`，其中包括航向校正，启停区 2 还有向车体右侧补位。启用角点视觉时，机载端须能响应 `CORNER` 并回传反馈。若裁判要求视觉必须实际达标，按第 19 题为 `CORNER` 区分达标与超时结果。
3. `HOME_RETURN` 再收臂并做最后的车体坐标补位。当前区 1 为 `(-115,125) mm`，区 2 为 `(140,155) mm`，定义在 `arm_motion.cpp` 的 `case MotionKind::HOME_RETURN`。完成后才进入 `STATE_DONE` 并发 结果为 `DONE` 的 `RUN_RESULT`。题面要求整车入区时，以实车停车位置与边界线为准。

**验证。**

- **设为 `true`。** 在两个启停区分别试。第二轮暂存完成后观察 `RETURN_HOME` 和对应的 `ROUTE_REQUEST`。收到 `ROUTE_DONE` 后还要等实际归位结束，再核对只有一次 结果为 `DONE` 的 `RUN_RESULT`。若没有 `ROUTE_DONE`，先看机载端是否回传了对应起终点的路径。
- **设为 `false`。** 第二轮暂存作业成功后应该直接进入 `DONE` 并发一次 结果为 `DONE` 的 `RUN_RESULT`，没有 `RETURN_HOME` 或暂存到启停区的路线请求。若仍然返家，先核对编译和上传使用的是刚修改的工程及正确的 COM 口。
- 总超时仍按第 16 题的规则处理。开启正常返家会把返航和补位计入本轮耗时，超时停车由原有总定时器负责。

#### 微调角点对齐后的最后一段平移

前往返家节点后，`HOME_OBSERVE` 会先校角，启停区 2 还会执行 `position(p,150,0,0,80)` 右移 **150 mm**，为相机找角点做准备。`HOME_ALIGN` 结束后走 `HOME_RETURN`，先收臂，再执行要微调的补位。角点对齐开关关闭时，流程也会执行这段最终补位。

1. 在 VS Code 按 `Ctrl+P` 打开 `src/arm_motion.cpp`，搜 `case MotionKind::HOME_RETURN`，找到这一行。

```cpp
position(p, r.zone==1 ? -115 : 140, r.zone==1 ? 125 : 155, 0, 200);
```

2. 从**车尾朝车头**看方向，按车辆开始补位时的车身朝向测量。`position(p, x, y, theta, speed)` 的含义如下。

| 参数 | 单位与方向 | 区 1 当前值 | 区 2 当前值 |
| --- | --- | --- | --- |
| `x` | mm，正为右移、负为左移 | `-115` | `140` |
| `y` | mm，正为前移、负为后移 | `125` | `155` |
| `theta` | °，本次相对转角 | `0` | `0` |
| `speed` | RPM，最大轮速 | `200` | `200` |

3. 先跑一次现有归位，标出停车位置和车身方向，再量还差多少。**新值 = 当前值 + 还需要的车身位移**。例如区 1 停下来还需要向车身右侧移 10 mm、再向前 5 mm，`x` 从 `-115` 改为 `-105`，`y` 从 `125` 改为 `130`。区 2 的两项保持原值。只调平移时沿用 `theta=0` 和当前轮速。

```cpp
position(p, r.zone==1 ? -105 : 140, r.zone==1 ? 130 : 155, 0, 200);
```

4. 要先确认方向，可以在 Debug、视觉已停止且运动空闲时，用「底盘 → 麦轮位置移动」，或发 `{CMD,CHASSIS,MOVE_POSITION,10,0,0,80}` 单测一次向右 10 mm。这个指令用来试方向和位移大小，正式返家默认使用上面源码中的补位值。
5. 将实测值填入源码后重新编译并 USB 烧录。从同一区、尽量相同的角点对齐位置再试一次，记录车身边缘距裁判边界线的实际余量。串口 `DONE,ESTIMATED` 和 `POSE GET` 是估算信息，现场用尺或地面标记核对。

如果问题发生在 `CORNER` **启动前**，例如区 2 右移后角点不在画面里，才去看 `HOME_OBSERVE` 里的 `150 mm`。需要修改相机对齐目标、死区或对齐前的横移时，分别核对这些配置。`HOME_RETURN` 的平移用于对齐阶段之后的最终停车位置。

[两段补位](../src/arm_motion.cpp) · [角点对齐与归位顺序](../src/competition_flow.h) · [相对位置移动协议](serial_protocol.md)

[返家开关与流程](../src/competition_flow.h) · [启停区节点](../src/robot_runtime.h) · [返家航向](../src/node_route_heading.h) · [归位动作](../src/arm_motion.cpp) · [完赛计时](../src/main.cpp)

### 16 总限时改为 180 秒，到时立刻停止

**定位与修改。** 搜 `xTimerCreate("HomeTimer"`，把周期示例 `3000000` 毫秒改为 `180000`。计时起点查 `runStart()` 的 `RUN_START`。超时走 `vHomeTimerCallback()`，当前语义是取消动作并停车。如果题目要求限时到达前必须返家，应在剩余时间尚可完成返航时切换流程，另用这个总超时作为到时处理。

**先确定题面计时起点。** 本项目在 `main.cpp` 的 `runStart()` 发 `RUN_START` 并启动总计时器。`SYS RELEASE` 创建计时器。它与正式计时起点之间还包含开局准备。题面若从裁判发令起算，要核对发令与 `RUN_START` 是否同一时刻。

**修改周期。** 在 `serial_commands.inc` 搜 `xTimerCreate("HomeTimer"`，把那一处 `pdMS_TO_TICKS(3000000)` 改为 `pdMS_TO_TICKS(180000)`。保留单次触发的 `pdFALSE` 和现有回调 `vHomeTimerCallback`。同步修正该行旁关于“强制返回”的旧说明。现有回调取消运动、停止视觉、切到 `STATE_TIMEOUT_FAILED`，即原地停止流程。

**先短后长。** 在练习副本临时用例如 `10000` 毫秒测到时分支，保持车辆在可控制的空载试验条件。脚本或人工记录 `RUN_START` 到 `{EVT,NAV,ROUTE_FAILED,TIMEOUT}` 的时间，确认轮组停下后还能用 `{CMD,SYS,HELLO}` 查询。随后恢复 `180000` 并重新烧录。另跑一次正常完成测试，确认 `runDone()` 已停止该计时器。

**验证。** 台架演练可先用较短限时，观察出现 `TASK TIMEOUT`、动作结束、串口仍可查询。再恢复题面时限并整轮计时。另试提前正常完成，确认没有之后又触发超时。

[总定时器创建](../src/serial_commands.inc) · [计时起点](../src/main.cpp) · [超时处理](../src/robot_runtime.cpp)

## 机械臂、视觉和完赛信号

### 17 只改暂存区三个工位的间距

**定位与修改。** 搜 `materialTransferLayout.workArea`。正式粗加工和暂存目前共用三组位姿。若两个区都统一换布置，可直接改 JSON 的 `搬运/工作区1～3`。只改暂存时，为它增加独立的三组位姿，并在 `MotionRequest` 明确区分粗加工和暂存。由 `TEMP_PLACE` 标注暂存，在 `arm_motion.cpp` 的 `transfer()` 中，让转台规划和实际取放步骤都选用同一组区域位姿。需要参数页现场编辑时，再为新位姿注册独立参数名。若视觉工位坐标也变了，同步机载端目标。

**先确认改的是整区位置还是工位相对位置。** 车应停在哪个地图节点由 `robot_runtime.h` 决定。三个物料位距车身多远、转台转多少由 `MaterialStationPose` 决定。一组位姿的顺序是 `{升降高度 mm, 伸出长度 mm, 转台角度 °}`。

**给暂存区单独一组数据。**

1. 在 `material_transfer.h` 的 `MaterialTransferLayout` 末尾增加 `MaterialStationPose tempWorkArea[MATERIAL_STATION_COUNT];`。在 `material_transfer.cpp` 的 `materialTransferLayout` 初始化列表末尾，为这一组追加三条实测位姿。初次验证可复制原 `workArea` 的三组作为基线，再只替换现场量出来的暂存数据。把字段追加在末尾可以保留前面已有聚合初始化的顺序。
2. 在 `motion_types.h` 的 `MotionRequest` 增加 `bool tempWorkArea = false;`。在 `competition_flow.h` 的 `TEMP_PLACE` 创建 `PLACE_ROUND` 请求时置 `request.tempWorkArea = true;`。第一层和第二层的暂存都会走这个分支，粗加工请求用默认值。
3. 在 `arm_motion.cpp` 的 `transfer()`，在第一个 `for` 循环前增加下列选择。把该函数两个循环里的 `materialTransferLayout.workArea[works[i]-1]` 都换成 `workArea[works[i]-1]`。第一个循环决定转台路径，第二个循环生成取放步骤，两次必须使用同一个实际工位。

```cpp
const auto* workArea = r.tempWorkArea
    ? materialTransferLayout.tempWorkArea
    : materialTransferLayout.workArea;
```

4. 如果还需要从参数页调节，在 `runtime_parameters.cpp` 模仿已有的 `registerPose(name, materialTransferLayout.workArea[i])`，为新数组使用不同的前缀，例如 `搬运/暂存工作区1`。对照生成脚本和既有字段名保存 JSON。现场如果只靠烧录实测值，可以先使用源码初始化值。
5. 空载用已标定的观察位姿看三个工位，依次验证 1、2、3 号的转角、伸出和最终下放。`-1` 在部分轴命令中表示保持原位，标定时核对这个特殊值。然后带一件料试放，最后跑正式第一层与第二层。`ARM DEMO` 默认请求没有 `tempWorkArea` 标记，要用正式暂存流程或专门为试动作传标记，才能覆盖新数据。
6. 粗加工回归验收要包含放下和取回。若暂存第一层也改变了物料底座高度，继续检查 `place()` 的 `slowLower` 分支，目前它在悬停后会向 `PLACE_DESCEND` 传高度 `0`。

**验证。** 先逐一测暂存 1、2、3 号工位在车体坐标中的伸出量和转台角度，再对照一次粗加工放置及取回，确认仍使用原位姿。第一层的最终下降还受 `place()` 中的 `slowLower` 分支控制，改工作区高度时一并核对。

[共享位姿](../src/material_transfer.h) · [请求结构](../src/motion_types.h) · [转台计划与取放](../src/arm_motion.cpp) · [参数注册](../src/runtime_parameters.cpp)

### 18 更换物料后，叠两层的增量改为 45 mm

**定位与修改。** 在 `robot_parameters.json` 搜 `搬运/二层高度增量(mm)`，将 `value` 改为 `45`。正式放置序列用 `destination.high + (layer - 1) * secondLayerOffset` 计算二层目标。按实物测二层释放位置，核对抬升通过高度与夹爪留量，再按题面值烧录。

**改一个 JSON 字段。** 在 `robot_parameters.json` 找现有条目，下列补丁只替换 `"value"` 那一行。这个参数是两层之间的**增量**。

```diff
 "搬运/二层高度增量(mm)": {
-  "value": 60.0,
+  "value": 45.0,
```

算一次目标值。假定计划里的工位基础高度为 `2.5 mm`，二层目标从 `2.5 + 60 = 62.5 mm` 降为 `2.5 + 45 = 47.5 mm`。若实车读取的基础高度不同，就用实车值重新算。`arm_motion.cpp` 的 `transfer()` 传给 `place()` 的是计算后的高度。第一层还有自己的慢降步骤。

**从回读到带料。** 烧录后回读增量，先用已有第二层试动作核对释放位置，再在底座上摆好第一层，试第二层单件释放。观察夹爪完全张开后的落差、物料是否顶住，以及抬起后是否带动物料。最后跑三件二层，确认三个台位都适用同一增量。若其中一位不合适，先复核该工位的基础高度和停靠姿态。

**验证。** 参数回读为 `45`，单件二层放置时确认夹爪能释放且物料无明显下落或顶住。再试第一层，确认改层高增量没有改变首层动作表现。

[烧录参数](../robot_parameters.json) · [放料高度公式](../src/arm_motion.cpp) · [实车参数说明](robot_parameters.md)

### 19 必须视觉达标才允许放料

**定位与修改。** 先问清题面指车体工位对齐、机械臂圆环纠偏，还是两者都要。以车体对齐为例，搜 `TickVisualAlignment()` 的 `TIMEOUT_CONTINUE`，当前超时会关闭纠偏并置为 `DONE`。对受规则约束的模式，应把真实达标和超时分开回传。再核对 `CompetitionFlow::aligned()`，当前会话重开后仍空闲也能继续，这条分支同样要遵守规则。超时后的处理用题目允许的重试或本轮失败方式，并保留串口维护。如果圆环也有同样要求，另查 `MotionHardware::correctRing()` 的 `RING_TIMEOUT` 分支。

**示例题限定粗加工和暂存的车体对齐。** 假定裁判允许对齐超时后判本轮失败。先改运行层结果，再改比赛层放行条件。

1. 打开 `robot_runtime.cpp` 的 `TickVisualAlignment()`。函数里已有 `workArea`，覆盖 `WORK_AREA` 和 `WORK_AREA_LOADED`。在超时分支，保留停车、PID 重置、关闭会话和灯光更新。替换原来的状态赋值和结果日志为下列语义，真实达标分支仍产生 `ALIGNED`。

```cpp
autoAlignmentState = workArea ? AutoAlignmentState::FAILED
                              : AutoAlignmentState::DONE;
updateVisionLight();
reportResult(workArea ? "TIMEOUT_FAILED" : "TIMEOUT_CONTINUE");
```

2. 在 `competition_flow.h` 搜 `bool aligned(VisionStartMode mode)`。在 `IDLE` 分支的首次重开逻辑之后、`ALIGN UNAVAILABLE - CONTINUE` 之前插入下列判断。这使两种工位模式在重开后仍空闲时也无法放行。`FAILED` 已经会进入原函数尾部的 `io.fail(STATE_ALIGN_FAILED)`。

```cpp
if (mode == VisionStartMode::WORK_AREA || mode == VisionStartMode::WORK_AREA_LOADED) {
    io.fail(STATE_ALIGN_FAILED);
    return false;
}
```

3. 订阅串口，按下表逐行试。题面若只考暂存而粗加工沿用现状，当前这两个视觉模式由多个阶段共用。此时把策略绑定在业务阶段或随对齐请求明确传递，避免按模式一刀切。
4. 如果题面还要求**机械臂圆环**达标，先确认对应放料请求打开 `ringCorrection`，同时 `圆环纠偏/超时(ms)` 为正数。到 `MotionHardware::correctRing()` 的超时分支让当前动作返回明确错误，并先按现有次序停止圆环视觉，防止序列继续发出后面的下探。二层当前不走第一层专用圆环分支，若规则覆盖二层，需要为该分支另行设计观察及下放动作。

| 测试输入 | 期望串口和动作 |
| --- | --- |
| 连续足够数量的合格帧 | `ALIGN_RESULT` 的结果是 `ALIGNED`，才进入工位放料 |
| 有帧，但误差一直超出死区 | 受约束工位出现 `TIMEOUT_FAILED`，停止纠偏，无后续放料 |
| 从启动起没有帧 | 超时行的有帧字段为 `0`，三项误差为 `NA`，无后续放料 |
| 会话意外空闲且重开后仍空闲 | 进入原有的对齐失败处理，串口维护查询仍可用 |

**验证。** 分别模拟合格反馈、持续不达标、完全无反馈。只有达标的场景允许进入放料。超时后的动作、日志和恢复方式按题面验收。

[底盘视觉结果](../src/robot_runtime.cpp) · [比赛对齐放行](../src/competition_flow.h) · [圆环纠偏结果](../src/motion_hardware.cpp)

### 20 裁判要求完赛时闪两次等长灯

**定位与修改。** 搜 `RuntimeCompetition::runDone()` 或 `LedPwm_PlayCompletionCue`。当前完赛提示使用三次闪烁且中间较长。题面要求两次等长时，可在 `LedPwm_PlayCompletionCue()` 里改用 `startCue(2,false)`。现有提示会同步蜂鸣器。

**替换完赛专用函数中的参数。** 在 `led_pwm.cpp` 搜 `void LedPwm_PlayCompletionCue()`，把 `startCue(3,true)` 改为下列调用。其他地方的 `LedPwm_PlayCue(n)` 是通用提示。

```cpp
void LedPwm_PlayCompletionCue() {
    startCue(2, false);
}
```

`2` 指脉冲数，`false` 表示等长。`LedPwm_UpdateFaultBlink()` 每次更新按 `cuePhaseMs` 计算相位，当前每相位 67 ms，等长模式的亮相位是 `0、2`，中间的 `1` 是暗相位。现有提示函数还会同步蜂鸣器。若裁判只准亮灯，在提示状态中为完赛调用显式关闭提示音，并让 `startCue()` 和周期更新里的两处 `BatteryMonitorSetCue()` 一起使用这一标记。正常低电压告警仍由电池监测逻辑管理。

**验收可用手机慢动作录制。** 从最后一次正常放料或归位结束开始录，包括先前的 `RUN_RESULT`。确认两次亮脉冲的长度一致，结束后恢复原亮度。分别再试开局、航向标定和运行失败，核对提示序列是否仍符合这些阶段的原行为和赛题要求。

**验证。** 启用返家时等归位完成，未启用返家时等最后一组叠放完成。检查 `RUN_RESULT` 只出现一次，灯正好闪两次。校准、开局等其他提示保持原时序。另试失败终态，不能显示完赛灯。

[完赛入口](../src/main.cpp) · [闪灯实现](../src/led_pwm.cpp) · [蜂鸣控制](../src/battery_monitor.cpp)

## 夹爪、物料形状和夹持专项

第 21～31 题以物料变宽、变长、易滑或易压坏为演练条件。示例数值用于说明修改方法。现场尺寸变更后，先确认物料与夹爪、圆盘、车体和料盘的实际间隙，再给出动作目标。

**先确认这次走的是哪条抓取入口。** 圆盘连续识别用 `VISION DISC_POSITIONS`，把物理位置 1～3 传给 `BuildArmProgram()` 里的 `positionGrab` 分支。该分支的抓点、开合角和前两步等待时间目前直接写在 `arm_motion.cpp`。旧版 `GRAB` / `FORCE_GRAB` 使用固定抓点，读取的是另一组变量。工作区和载物台的取料则走通用 `pick()`。搜索到同名 JSON 参数时，可以用 `F12` 或全局搜索追到实际读取位置。

| 想调的量 | 优先查的位置 |
| --- | --- |
| 位置 1～3 号抓点、圆盘张口及夹紧 | `arm_motion.cpp` 的 `if(positionGrab)` |
| 位置抓取三步共用的速度 | `disc_position_grab.cpp` 的 `armSpeed` |
| 通用工作区或载物台开合、步骤速度 | `robot_parameters.json`；参数绑定在 `runtime_parameters.cpp` |
| 伸缩和升降步进电机的加减速 | `Step::lengthAcceleration`、`MoveArmInternal()`；它们使用 EMM V5 档位 |
| 夹爪舵机的速度与斜坡 | `MoveArmInternal()` 和 `Servo_SetAngleMTurnByInterval()`；该接口用毫秒描述斜坡 |

**单件验证的公共做法。**

1. 在 Debug 下停在正确的圆盘对齐位置，让圆盘静止。先空载确认方向和近料路径，有料测试前检查待用货位实际为空，程序记录也为空。轴位置命令使用绝对机械臂位姿，未校准时先查现有原点和标定。
2. 只试夹爪，可用 `{CMD,ARM,MOVE2,-1,80,150}`：第一个 `-1` 保持转台，`80` 为张口目标角，`150` 为本次动作速度输入。试升降或伸缩用 `ARM MOVE1` 的高度、伸出和速度参数组合。先单轴、后组合；等动作结束并目测位置后再发下一次。用 `POSE GET` 读到的夹爪角可以辅助判断，实际夹持仍靠试件确认。
3. 验证**指定物理位置**时，可在确认试件真处于可抓位置、视野和动作空间就绪后，用 Debug 依次输入 `{CMD,TASK,SET,156+123+516+231}`、`{CMD,VISION,START_REQUEST,DISC_MATERIAL}`，然后单独发送 `{CMD,VISION,DISC_POSITIONS,2,1}` 测位置 2 的颜色 1。测试位置 1 或 3 时替换位置字段。此测试从任务首色开始，当前默认首件装入载物台 3。若接着测下一件，先按实际流程卸料并核对货位登记，避免上次物料还在盘上。手动发候选时让机载端停止自动回传候选，避免同时提交两路抓取。
4. 看 `DISC_POSITIONS,SELECTED` 是否选中了预期的物理位置和货位，再等对应的 `DONE` 或 `FAILED`。软件正常结束后，仍要看物料有没有抓牢、提起时有没有擦碰、是否放进目标料盘。点动 `FORCE_GRAB` 会走旧固定抓点，验证三个位置时应使用 `DISC_POSITIONS`。
5. 源码常量类方案需离线构建并 USB 烧录。已有运行参数可在 Debug、视觉关闭且动作空闲时临时发送、回读；确认后保存到 `robot_parameters.json` 并烧录。保存前核对修改的参数名确实被目标分支读取。

[串口协议与位置抓取](serial_protocol.md) · [参数修改说明](robot_parameters.md) · [动作编排](../src/arm_motion.cpp)

### 21 圆盘某个位置抓偏了，只改该位置的抓点

**题面示例。** 换成长物料后，位置 3 要把夹爪往回收 10 mm，位置 1 和位置 2 继续沿用原抓点。

**定位。** 打开 `src/arm_motion.cpp`，搜 `if(positionGrab)` 或 `DISC_APPROACH`。现有抓点以机械臂自身坐标记录。

| 视觉上报的物理位置 | 抓料高度 mm | 伸出 mm | 转台目标 ° |
| --- | ---: | ---: | ---: |
| 1 | 80 | 150 | 71 |
| 2 | 80 | 0 | 90 |
| 3 | 80 | 150 | 102 |

**照着改。** 把原有同时声明 `length` 和 `angle` 的那一行拆开。下面只改位置 3 的伸出，原来两个 `arm(...)` 里传入的 `length` 会同时更新预抓和夹紧位置。

```cpp
const float length = r.position == 1 ? 150.0f
                   : r.position == 2 ?   0.0f : 140.0f;
const float angle = r.position == 1 ? 71.0f
                  : r.position == 2 ? 90.0f : 102.0f;
```

如果现场只给位置 3 垫高，可同样定义一个按 `r.position` 选的 `height`，替换 `DISC_APPROACH` 和 `DISC_GRIP` 中的第一个 `80`。第二个 `80` 是预抓张爪的角度。若是物料方位变化，先核对转台角和伸出两者，转台一转，夹爪也会绕臂轴扫过一段空间。

**验证。** 先在较高位置空载试转向和伸出，再用单件候选 `{CMD,VISION,DISC_POSITIONS,3,1}` 验证完整抓放。用相同方式复查位置 1 和 2。若日志里的 `SELECTED` 就选错了位置，先检查机载端的物理位置编号。

[抓点和步骤](../src/arm_motion.cpp) · [视觉候选解析](../src/serial_commands.inc)

### 22 换成更宽的物料，增加预抓张口

**定位。** 在 `DISC_APPROACH` 找 `arm(p,"DISC_APPROACH",80,length,angle,80,armSpeed,350)`。前面的 `80` 是升降高度，后面的 `80` 才是张爪角。通用抓放还分 `搬运/夹爪张开(deg)`、`搬运/载物台/夹爪张开(deg)` 等。对应工作区抓前/松手、料盘抓前/松手。旧固定圆盘抓取另读 `搬运/圆盘预抓/张爪(deg)`。

**照着改。** 例如实测只需要位置 3 多开 5°，在原 `DISC_APPROACH` 前定义角度，替换该步的张爪实参。

```cpp
const float openAngle = r.position == 3 ? 85.0f : 80.0f;
arm(p, "DISC_APPROACH", 80, length, angle, openAngle, armSpeed, 350);
```

若所有物料和三个位置都需要更大的张口，直接调同一个预抓角即可。当前源码对夹爪张开设有 91° 软件上限。先用卡尺量所需开口和夹爪实际开口，若已到上限仍不能包住物料，重新看夹爪形状或抓取方向。

**验证。** 空载试 80° 和 85° 比较开口，再试带料接近，检查张开时会不会碰相邻件。抓取成功后也观察料盘放料能否顺利退出。若只是装盘后爪尖钩料，调载物台的张开角，位置预抓角可以保持。

[开合参数注册](../src/runtime_parameters.cpp) · [夹爪角度上限](../src/chassis.h)

### 23 易压坏或易滑的物料，重新标定夹紧角

**定位。** 搜 `DISC_GRIP`。三个位置专用分支当前直接用 `-8°`。旧圆盘和通用 `PICK_GRIP` 则读 `materialTransferLayout.clawClosedAngle`，对应 `搬运/夹爪夹紧(deg)`。当前仓库 JSON 的通用烧录默认值为 `-9°`，它与位置分支的 `-8°` 分开。

**操作。**

1. 把夹爪放在试件可控的位置，先以小幅改变目标角，观察夹爪在接触物料时的位置。举例，若验证该机器上从 `-8°` 向 `-6°` 能减小压紧量，可在 Debug 用 `{CMD,ARM,MOVE2,-1,-6,150}` 试这一个角。角度反映目标开合，夹持压力还受夹爪结构与物料形状影响。
2. 正式三位置方案，把 `DISC_GRIP` 中的 `-8` 改为实测的目标角，例如 `-6.0f`。只对某位置生效时，先按 `r.position` 得到 `closeAngle`，再传给该步。`-1` 在 `MoveArm` 中表示保持该轴，不能用来表示负一度夹紧。
3. 若题目要求工作区取回也用同一夹紧角，把相应 JSON 项的 `value` 改为验证值并回读。通用参数会影响所有读取 `clawClosedAngle` 的步骤，改完还需复查粗加工取回和其他使用通用夹爪的动作。

**验证。** 静止夹住后先小幅抬升，再试缩臂和转运，观察滑落、偏转、压痕。角度读取能检查是否到了指令附近，不能单独证明已经夹住。若驱动达到角度时物料仍在滑，先复核接触点和夹爪开合方向。

[圆盘与通用夹紧](../src/arm_motion.cpp) · [夹爪指令换算](../src/chassis.cpp)

### 24 长物料会擦圆盘，把提起和收臂拆成两步

**题面示例。** 物料伸出较长，边抬边缩会卡圆盘挡边，要求先竖直提离，再水平收臂。

**定位。** 在 `arm_motion.cpp` 的 `positionGrab` 内，原 `DISC_LIFT` 同时设置了升降 `materialTransferLayout.approachHeight` 和伸出 `0`。这一步紧跟 `DISC_GRIP`，后面才转向料盘。

**照着改。** 只替换该分支的 `DISC_LIFT` 调用，把下列两步放在原 `lengthAcceleration` 循环之前。这里的 `200` 和 `300` 是示例等待毫秒数，按实测行程填写。

```cpp
arm(p, "DISC_LIFT", materialTransferLayout.approachHeight,
    -1, -1, -1, armSpeed, 200);
arm(p, "DISC_RETRACT", -1,
    0, -1, -1, armSpeed, 300);
```

第一步只改高度，伸缩、转台和夹爪保持。第二步保持高度和夹紧状态，把伸缩轴收回 `0 mm`。收臂完成后沿用已有料盘放置。原本的 `lengthAcceleration` 循环仍可覆盖新加的收臂步骤。若后续还要改放料顺序，保留末尾已有的货位登记，让程序在抓放序列完成后才记账。

**验证。** 用短行程空载看两个动作的先后，再带长试件测试。串口步骤应按 `DISC_GRIP → DISC_LIFT → DISC_RETRACT → PLACE_ROTATE` 出现，实际观察物料底部已越过挡边才开始收臂。定时模式用 `waitMs` 决定切换，先确认高度能在所给时间内达到。修改应覆盖题目要求的位置。

[圆盘抓放步骤](../src/arm_motion.cpp) · [序列推进方式](../src/motion_sequence.h)

### 25 夹爪闭合太猛，只降夹紧步骤的速度

**先区分参数。** `disc_position_grab.cpp` 的 `armSpeed` 当前为 `500`，位置 1～3 的预抓、夹紧、提起都用它。把 `500` 改成 `350` 会同时降低这三步共享的动作速度。题面若只要求夹爪合拢变慢，定位到 `DISC_GRIP` 单独修改。

**做法 A，仅调整夹紧一步的速度。** 如果实测 `DISC_APPROACH` 完成时三轴已经到目标，可以让 `DISC_GRIP` 只发夹爪命令。下面保持原夹紧角和等待时间，改用已有 `materialMotionSteps.discGripSpeed`，它对应 JSON 的 `搬运/速度/圆盘夹紧`，仓库示例值为 `300`。

```cpp
arm(p, "DISC_GRIP", -1, -1, -1, -8.0f,
    materialMotionSteps.discGripSpeed, 400, false, ARM_GRIP_HOLD_MS);
```

若工况还需要在夹紧步重新给三轴位置，则保留原 `80,length,angle`，只替换速度实参。这样同一步内仍发出的升降和伸缩指令也会使用新速度。通用工作区抓取用的是另一项 `搬运/速度/取料夹紧`。

**做法 B，先查清是哪一轴太快。** `MoveArmInternal()` 中，普通升降的目标电机 RPM 由 `speed×15` 换算，伸缩由 `speed×6` 换算，底层以协议字段上限截顶。夹爪依据实际剩余角度和 `speed` 计算执行时间，换算后的非零行程时间最少 `40 ms`。机械臂转台的默认转速在 `turret_motor.h` 的 `TURRET_MOTOR_SPEED_DEG_S` / `TURRET_MOTOR_POSITION_RPM`，改它会影响使用默认值的转台动作。

**验证。** 用同一张口初始角录制合拢过程，比较原速度和新速度的实际耗时及碰料表现。确认闭合到位后才提起。若更改后仍过快或变慢后抬升提前，分别回查本步速度换算与第 28 题的等待策略。

[位置抓取速度变量](../src/disc_position_grab.cpp) · [逐步速度](../src/arm_motion.cpp) · [轴速与舵机行程换算](../src/chassis.cpp) · [转台默认速度](../src/turret_motor.h)

### 26 又长又易滑，伸缩轴启停时要更平缓

**定位。** 在 `arm_motion.cpp` 的圆盘 `positionGrab` 分支，找循环末尾 `p.steps[step].lengthAcceleration=245`。这个值传给伸缩轴 6 号电机的 EMM V5 位置模式。按本项目模型，档位越接近 `255`，斜坡越陡。相同转速下，把 `245` 调低会拉长加减速过程。

**只改带料收臂。** 若保持现有三步动作，在原循环后面追加下面一行。此时最后追加的还是 `DISC_LIFT`，该步包含收臂。若按第 24 题已拆开，则令这一行紧跟 `DISC_RETRACT`，或在原循环之后确认索引仍指到该步。

```cpp
p.steps[p.count - 1].lengthAcceleration = 225;
```

该改法只覆盖最后一条伸缩动作。`DISC_APPROACH`、`DISC_GRIP` 仍用 `245`。对应项目模型中，`245` 约为每增加 1 RPM 用 `0.55 ms`，`225` 约为 `1.55 ms`。这些是档位换算值，实际动作取决于速度、距离和负载。`MoveArmPlanned()` 目前会用默认伸缩档位，其他规划动作改档前要先核对是否走了这个入口。

**单轴先试。** Debug 的 `{CMD,EMM,AXIS_TRIAL,轴号,绝对目标mm,转速RPM,斜坡ms,采样ms,记录时长ms}` 能记录轴位置与转速。轴 6 是伸缩，轴 5 是升降。比如相同的 `200 RPM` 下，斜坡填 `110 ms` 预计得到 `245` 档，`310 ms` 预计得到 `225` 档；目标位置用现场确认可达的小行程。确认 `AXIS,COMMAND` 报出的最后一个字段与预期一致。在相同起点比较数据，再试带长件收臂。

**验证。** 看伸缩初末端的抖动、物料滑移和收臂完成位置。斜坡变缓后，定时模式可能需重新标定收臂步骤的等待值。升降轴普通档位在 `chassis.cpp` 的 `MoveArmInternal()` 中为 `252`。若要只调首层末段慢降，另看 `arm_motion.cpp` 中 `PLACE_DESCEND` 后的 `slowLowerAcceleration=200`。转台则有独立 `turretAcceleration`，与伸缩档位分开。

[步进档位指定](../src/arm_motion.cpp) · [档位与时间模型](../src/position_move_timing.h) · [单轴试验](../src/arm_axis_trial.cpp) · [分轴下发](../src/chassis.cpp)

### 27 夹爪开合要软起停，给舵机分别设加速和减速时间

**题面示例。** 软壳或圆弧形物料在夹爪末段减速不够时容易弹开。先试第 25 题的速度参数。若题目明确要求分别控制两个斜坡，再按本题改发令方式。

**定位。** `chassis.cpp` 的 `MoveArmInternal()` 会读夹爪实际角度，算出最近的等效多圈角 `nearestAngle` 及总行程时间 `gripperIntervalMs`，再调用 `Servo_SetAngleMTurn()`。`servo.h` 另有 `Servo_SetAngleMTurnByInterval()`，可以指定总运动时间、加速时间和减速时间，单位都是毫秒。

**进阶改法，仅用于圆盘位置夹紧。**

1. 在 `motion_sequence.h` 的 `Step` 增加 `bool clawRamp = false;`。在 `arm_motion.cpp` 的位置抓取 `DISC_GRIP` 调用后加 `p.steps[p.count-1].clawRamp = true;`，明确只为这一步启用斜坡。
2. 在 `chassis.h` 和 `chassis.cpp` 的普通 `MoveArm()` 参数末尾加 `bool clawRamp`，默认值 `false` 只放在头文件声明。普通包装函数把它继续传给 `MoveArmInternal()` 的末尾参数。`MoveArmInternal()` 可以给末尾参数默认 `false`，使走 `MoveArmPlanned()` 的步骤继续使用原逻辑。在 `MotionHardware::issue()` 的普通 `MoveArm(...)` 调用最后传入 `s.clawRamp`。当前圆盘位置夹紧走的就是这个普通分支。
3. 在 `MoveArmInternal()` 里，把原来的夹爪下发行替换为下方的示例。保留实测角、最近多圈角和现有功率保护配置。`140` 是总时间，`40` 加速、`60` 减速，中段剩 `40 ms`。项目接口注释要求加速和减速各不少于 `20 ms`，总时间须容纳两个斜坡。

```cpp
if (clawRamp && travelDeg > 0.0f) {
    Servo_SetAngleMTurnByInterval(1, nearestAngle, 140, 40, 60, 0);
} else {
    Servo_SetAngleMTurn(1, nearestAngle, gripperIntervalMs, 0);
}
```

该示例的总时间固定为 `140 ms`，仅影响有标记的闭合步骤。若题面还要求按角度差和输入速度改变总时间，沿用原 `gripperIntervalMs` 计算，并为其选择相容的两个斜坡。在按固定 `waitMs` 切步的模式下，用实际闭合过程核对 `DISC_GRIP` 的等待足够。

**验证。** 先无料对比起步和收尾，再用软物料测试接触和提起。来回做一次旧入口和通用工作区取料，确保这些动作未启用新斜坡。观察物料的压痕、偏转和弹出；夹爪达到目标角只是软件反馈的一部分。

如果练习副本要跑原生测试，用 `rg -n '^bool MoveArm\(' tests` 找测试替身，给其函数定义补上新增的末尾参数，使之与头文件签名一致。

[步骤字段](../src/motion_sequence.h) · [硬件发令](../src/motion_hardware.cpp) · [夹爪控制](../src/chassis.cpp) · [分段斜坡接口](../src/servo.h)

### 28 夹紧后马上抬起会掉料，延长合拢后的保持

**定位。** 同一个 `DISC_GRIP` 调用中有两个时间参数。位置 1～3 目前传 `wait=400` 和 `hold=ARM_GRIP_HOLD_MS`。在 `robot_parameters.json` 搜 `运动执行/机械臂反馈完成`，当前烧录默认值为 `0`。

| 模式 | 正式位置抓取的生效项 | 开始计时点 |
| --- | --- | --- |
| 计时模式，反馈开关为 `0` | 位置分支 `DISC_GRIP` 里的 `400 ms` 改成实测时间 | 发出该步命令后 |
| 已启用并验证反馈的模式 | `运动执行/反馈后夹持保持(ms)` 对应 `ARM_GRIP_HOLD_MS` | 读回达到该步的完成条件后 |

**照着改。** 假定默认计时模式，试验后发现需要将该步从 `400` 延长至 `550 ms`，在 `arm_motion.cpp` 的位置专用分支替换这一个实参。

```cpp
arm(p, "DISC_GRIP", 80, length, angle, -8, armSpeed,
    550, false, ARM_GRIP_HOLD_MS);
```

如果已验证实车反馈，用参数页或 JSON 调 `运动执行/反馈后夹持保持(ms)`。这个保持参数也会被其他引用 `ARM_GRIP_HOLD_MS` 的夹紧步骤使用。只想改位置专用抓取时，在该步单独传入实测保持值。`动作组延时/圆盘/夹紧(ms)` 虽然在注册表中存在，这段位置抓取没有读取它。

**验证。** 在串口时间线标出 `DISC_GRIP` 开始、结束与 `DISC_LIFT` 开始，在视频上看物料是否已经稳定。默认计时模式的 `550 ms` 包含实际合拢过程。若反馈模式下宽物料挡住夹爪、导致读回角始终达不到目标，应复核夹紧目标及真实夹持判据，先查实物和反馈。

[时间字段](../src/arm_motion.cpp) · [反馈和定时的选择](../src/motion_hardware.cpp) · [保持计时](../src/motion_sequence.h) · [参数注册](../src/runtime_parameters.cpp)

### 29 物料更高，调整装入料盘时的释放位置

**先判定碰撞在哪。** 有料时擦到料盘口，先看盘角是否对准。爪子下到正确盘位却压住物料，先量释放高度。用同一个货位做测试，便于区分两个问题。

1. **料盘角偏了。** 在 `robot_parameters.json` 查 `搬运/旋转料盘1/角度(deg)` 到 `搬运/旋转料盘3/角度(deg)`。修改实际对偏的货位，不改货位编号或任务映射。料盘是 ID 3 舵机，机械臂转台是另一轴。新角度应让目标盘中心对准公共的待操作位。可先在 Debug 空盘试转，查看 `TRAY ROTATE` 协议。
2. **抓取位置也需要变。** 查 `搬运/待操作区/高度(mm)`、`伸出(mm)` 和 `转台(deg)`。这三项定义所有盘位对准时公用的机械臂取放姿态。改高度后，从盘中取料的下探也会受影响。
3. **仅在装盘松爪前抬高。** 搜 `CARGO_TRAY_RELEASE_OFFSET_MM`，当前为 `3.0f`，`place()` 在 `destinationCargo` 为真时计算 `height + CARGO_TRAY_RELEASE_OFFSET_MM`。例如实测需要再高 `2 mm`，在 `material_transfer.h` 将常量改为 `5.0f`。如果基准高度是 `125 mm`，实际目标从 `128` 变为 `130 mm`。这个常量由多个装盘入口共用。题面只要求改一个货位时，应在目标货位的下放步骤单独传入偏移。

**验证。** 先空盘逐一找正，再带一件新物料落盘。观察放下时底面是否有足够支撑、松爪时是否钩住、物料是否落入盘内。最后从同一货位取回一件，确认新高度和角度未影响下一次夹取。

[参数文件](../robot_parameters.json) · [装盘偏移常量](../src/material_transfer.h) · [最终下放计算](../src/arm_motion.cpp) · [料盘操作协议](serial_protocol.md)

### 30 扁件放下容易被带歪，调整首层的分段松爪

**定位。** 在 `arm_motion.cpp` 搜 `PLACE_RELEASE_ZERO`。工位首层的 `slowLower` 分支先下放，然后把夹爪发到中间角 `0°`，等待 `300 ms`，随后 `PLACE_RELEASE` 才完全张开。首层粗加工和暂存会走这一分支。第二层和一般的装盘步骤用各自的松爪路径。

**按现象改一个量。**

- 中间角开得太少，爪子依然带着物料移动。先单测中间角。例子中若 `5°` 能释放接触而且物料仍受底面支撑，可把 `PLACE_RELEASE_ZERO` 的目标从 `0` 改为 `5.0f`。
- 中间角合适，但物料还在晃就全开。按视频测稳定时间，改同一行末尾 `300` 毫秒。该步后面 `timed=true`，仍按这个时间推进。
- 到最后全开时才挂住物料。查 `PLACE_RELEASE`，工位张开读 `搬运/夹爪张开(deg)`，盘位张开读 `搬运/载物台/夹爪张开(deg)`。步骤速度对应 `搬运/速度/放料松爪`。只在首层改变速度时，可在首层步骤单独传入速度。

示例仅换中间角、保留原等待。它是角度目标，开合方向用实车确认。

```cpp
arm(p, "PLACE_RELEASE_ZERO", -1, -1, -1,
    5.0f, materialMotionSteps.placeReleaseSpeed, 300);
p.steps[p.count - 1].timed = true;
```

**验证。** 在同一工位用地面或底座标记对比放置位置。跟踪 `PLACE_DESCEND → PLACE_RELEASE_ZERO → PLACE_RELEASE → PLACE_LIFT`，确认中间角出现时物料已由工位承托，最终不被爪尖带偏。再试一次二层和料盘释放，核对它们仍遵循各自流程。

[首层松爪和抬升](../src/arm_motion.cpp) · [松爪角度与速度参数](../src/runtime_parameters.cpp)

### 31 第三件物料必须夹在手上带到工区

**练习按这个口径。** 每轮按任务顺序抓取。第 1、2 件照常放进载物台 3、2，第 3 件抓起来后一直由夹爪携带，从圆盘去粗加工区，途中不装盘。允许到达后先放手上这件。若裁判还要求从粗加工去暂存也这样运，见本题末尾的扩展。

**先问清到站的落位顺序。** 一只夹爪夹着第 3 件时，不能再去料盘夹第 1 或第 2 件。允许自由放置时，顺序采用 **3 → 1 → 2**，三件仍各放到任务码指定工位。若裁判要求 **1 → 2 → 3** 并且禁止暂放，需要裁判认可的其他交接方式或独立持料机构才能同时满足。保留“第三件”的任务索引，避免把第 3 件送去第 1 件的工位。

| 时刻 | 载物台 3 | 载物台 2 | 载物台 1 | 夹爪 |
| --- | --- | --- | --- | --- |
| 前两件抓放完成 | 第 1 件 | 第 2 件 | 空 | 空 |
| 第 3 件提起并确认作业成功 | 第 1 件 | 第 2 件 | 空 | 第 3 件 |
| 到粗加工并先放手上物料 | 第 1 件 | 第 2 件 | 空 | 空 |
| 另两件完成放料 | 空 | 空 | 空 | 空 |

#### 第一步 在选料入口标记第 3 件由夹爪携带

1. 在 `src/serial_commands.inc` 搜 `serialDiscPositions()`。函数里的 `progress` 是本轮已完成件数，`progress==2` 表示正要抓第 3 件。`candidate.position` 是相机上报的物理位置。第 3 件可以在圆盘 1、2、3 任一个位置。
2. 在 `src/motion_types.h` 的 `MotionRequest` 末尾增加 `bool keepInHand = false;`。仅在本题启用、正式抓取状态且 `progress==2` 时置 `true`。在 `serialDiscPositions()` 的料盘占用检查前得出这个标记，在 `serialQueueDiscGrab()` 创建请求时传入。该分支不使用料盘。可用 `r.cargo=0` 表示这一点，但同一条修改里要让它绕过所有 `cargo-1` 索引、目标货位检查和选盘调用。
3. 正式流程用 `DISC_POSITIONS` 时，机载端只发该类候选。如果车上仍启用旧 `GRAB` / `COLOR` 自动入口，也要用同一条第 3 件规则，避免由旧入口误装盘。手动 `FORCE_GRAB` 搬运另有既定用途。
4. 机载端如果继续消费第三件的选中和完成消息，同步表现为“夹爪持有”。例如新增专用事件 `DISC_HELD_DONE`，带位置和颜色，并同步解析。`cargo=0` 只是内部示例，不能继续沿用“已放到 1～3 号货位”的普通回复含义。

#### 第二步 第 3 件抓起后结束抓取作业 带着它驶出圆盘

1. 在 `src/arm_motion.cpp` 的 `BuildArmProgram()` 里找到 `case MotionKind::DISC_GRAB` 和 `if(positionGrab)`。保留 `DISC_APPROACH → DISC_GRIP → DISC_LIFT`，只让 `keepInHand` 跳过料盘占用检查、前置 `prepareTray()`、后续装盘 `place()` 和 `CARGO_COMMIT`。在循环内可于抓取步骤追加完成后直接 `continue`，跳过装盘部分并走函数末尾的统一处理。现有 `HELD_DISC` 会把手上的物料继续放进载物台，不能用它充当“夹着走”的抓取作业。
2. 末步姿态应适合行车。沿用已标定的抬起和收臂动作作为起点，夹爪目标保持 `-1`，也就是不重发开合指令。长件如果需要先离盘再收臂，用第 24 题的拆分法。空载检查底盘转向时的机械臂位置，再用带料试件检查外伸和视野。
3. 在运动执行器记录独立的手持物料，例如保存 `heldColor`，本轮第三件确认成功后才写入。`roundProgress` 仍从 `2` 变为 `3`，让 `DISC_GRAB_MATERIALS` 按原条件进入去粗加工的阶段。`AdvanceDiscGrabProgress()` 现在通过货位号核对进度。手持分支应用独立的成功条件，即确认这是本轮第 3 件且其作业完成。把手持记录更新放在进度变化之前，停止后确认动作与物料实际状态再恢复。
4. 不给手持物料写 `CARGO_COMMIT`。现有实现会用 `s.cargo-1` 索引料盘数组。第 3 件未装盘时，载物台 1 在软件和实物上都应为空。

#### 第三步 到粗加工前后都不主动松爪

1. 在 `src/competition_flow.h` 的 `COARSE_NAVIGATE`，手持分支用 `path(map.disc, map.coarse, ArmObservation::NONE, 0)`。保留 `request.discDeparture=true`，继续做圆盘离站补偿。省去途中自动普通 `LOOK` 和到站预选盘。现有普通 `LOOK` 会给夹爪发 `60°`、`80°` 目标，会松开物料。
2. 如果工作区相机需要观察姿势，新增一个保持夹爪的姿态，例如 `LOOK_HOLD`。在 `observation()` 中参考 `LOOK` 的升降、伸缩、转台目标，只把每一步的夹爪目标改为 `-1`。先试带料是否挡住相机或与车体干涉。必要时按实测选择能看清工位的持料姿势。
3. 到站后在 `COARSE_NAVIGATE` 和 `COARSE_ALIGN` 之间新增准备阶段，用现有 `motion(pose(...))` 提交 `LOOK_HOLD`。作业成功后再通过 `beginAlignment(VisionStartMode::WORK_AREA)` 开始对齐。正式流程这个入口只启动视觉。串口手动 `VISION START_REQUEST WORK_AREA` 当前会先排普通 `LOOK`，夹着物料调试时用专门的持料流程。

[物料入队](../src/serial_commands.inc) · [抓取、观察和放置编排](../src/arm_motion.cpp) · [作业完成和进度](../src/motion_executor.cpp) · [正式比赛阶段](../src/competition_flow.h)

#### 第四步 到站先放手上这件 再做前两件的料盘搬运

**手持放料单独做一个作业。** 在 `MotionKind` 增加 `PLACE_HELD`，在 `CompetitionFlow` 增加 `COARSE_PLACE_HELD` 阶段。对齐阶段成功后进入这个阶段。先用 `io.task(request, secondRound)` 填本轮任务，再设置关键字段如下。第三件的工位使用零基数组的第 2 项。

```cpp
request.kind = MotionKind::PLACE_HELD;
request.work = request.positions[2];
request.color = request.colors[2];
request.layer = 1;
request.ringCorrection = true;
request.automatic = true;
```

在 `arm_motion.cpp` 的新动作分支用上述 `work` 选 `materialTransferLayout.workArea[work-1]`，回读转台实测角并用现有 `plan(...)` 规划去该工位的角度。取料已经完成，此处只调用已有 `place()`，跳过 `pick()`。以现有“先收臂、后转向”的持料姿态为前提，核心调用可写成下面这样。`workPose` 和 `selected` 分别是已选中的工位位姿和本次规划结果。

```cpp
const float height = workPose.high
    + (r.layer - 1) * materialTransferLayout.secondLayerOffset;
place(p, workPose, height, commonMaterialDelays, true,
      &selected.anglesDeg[0], r.layer == 1, r.work,
      0, 0, r.ringCorrection);
```

调用中的两个 `0` 分别表示无来源料盘、无目标料盘。首层仍沿用项目的分段松爪及圆环设定。原先从料盘放置所用的动作组延时没有可直接套用的盘号，这个例子以公共延时为基线再按持料实测调整。可以沿用 `WORK_PLACE` 和工位编号记录完成事件，手持作业完成时更新 `heldColor` 为空。工作任务应在向主状态机发布该作业的 `DONE` 前更新这项记录。取消或动作中断后，先确认第三件实际在夹爪还是已在工位，再按既有恢复流程操作。

**前两件继续从料盘放。** `COARSE_PLACE_HELD` 收到成功结果后再转入现有 `COARSE_PLACE`，用一个新作业处理前两件。推荐为 `MotionRequest` 增加 `uint8_t materialCount = 3;`，仅手持方案下的这次 `PLACE_ROUND` 设为 `2`。然后在 `transfer()` 的两个循环共用这个数量，原来的 `single` 和其他动作仍用各自数量。数组仍使用 `request.positions[0]`、`request.positions[1]`，并沿用 `CargoForGrabIndex(0/1)` 从料盘 3、2 搬运。这样第二个作业会从手持放料结束后的转台实测角重新规划。两批都成功后才进入粗加工取回，现有 `RETRIEVE_ROUND` 可保持三件全取回载物台。

#### 用真实任务码验收

以 `156+123+516+231` 为例。第一轮第 3 件为颜色 `6`、粗加工目标工位 `3`，第二轮第 3 件也是颜色 `6`、粗加工目标工位 `1`。

| 验证场景 | 应看到的结果 |
| --- | --- |
| 第三件分别出现在圆盘位置 1、2、3 | 只由任务下标决定是否手持。每个位置都用自己的标定抓点 |
| 第三件夹紧并抬升完成 | 抓取进度到 `3`，料盘 1 为空，第 3 件仍在爪里，出现去粗加工的路线请求 |
| 离盘、直行、转向、对齐 | 夹爪保持闭合。实车观察伸出范围、滑落和相机遮挡 |
| 粗加工放料 | 第一轮工位日志应为 **3 → 1 → 2**，第二轮为 **1 → 2 → 3**。按各轮任务的工位映射核对物料 |
| 第三件没抓牢或持料中途停止 | 停止后核对夹爪与料盘实物，故障或取消不算抓放成功。避免再次发普通开爪观察姿态 |
| 正常跑完两轮 | 每轮各放三件。粗加工取回和后段暂存仍按题目要求执行，上一轮的手持记录不会带入下一轮 |

练习顺序建议用程序或桩测试先验证第三件的分流和两个放料请求。实车先做静止夹取和持料姿态，再做短距离、转向、到站单件放置，最后跑完整流程。角度或作业完成回报只能辅助判断，携带过程要实看物料有无滑落。

#### 如果粗加工到暂存也要求第 3 件手持

在 `COARSE_RETRIEVE` 中，前两件照常装入载物台 3、2，第 3 件改为工作区夹取后直接结束在持料姿态。让 `TEMP_NAVIGATE` 同样避开普通 `LOOK`，到站用 `LOOK_HOLD` 后再走原暂存视觉模式。暂存先放手上物料，再从盘里搬两件。

计算第三件的暂存目标时，先应用 `TEMP_PLACE` 原有规则：第一轮使用本轮工位码，第二轮同时复用按第一轮颜色位置匹配的映射和二层高度。持料放置用映射完成后的 `positions[2]`。第二轮照旧采用 `WORK_AREA_LOADED`，手持传输不改变配色和叠层规则。若题面明确要改变这些规则，再分别用第 09、10、18 题核对。

[放料步骤及高度](../src/arm_motion.cpp) · [作业类型与请求字段](../src/motion_types.h) · [默认抓取货位映射](../src/disc_material_decision.h) · [流程日志](RUN_flow.md)

## 物料专项进阶与其他临场规则

本节第 32～51 题承接前面的离线演练。代码片段用于练习副本。涉及实体夹爪、相机位置或工位布局的题目，先以现场尺寸及裁判的允许条件确定数值。

### 32 裁判封掉圆盘一个取料位置

**题面。** 圆盘位置 2 禁止下爪，即使该处物料颜色匹配也要等它转到其他位置。

**怎么改。** 打开 `disc_material_decision.h` 搜 `SelectDiscPosition()`。保留候选编号、颜色、重复位置检查。在候选校验完成后、比较颜色和排名前插入 `if (candidates[i].position == 2) continue;`。若机载端也能过滤，在组帧前同步排除位置 2。确认旧 `GRAB` 入口是否也用于正式取料，旧入口没有物理位置字段，现场应使用能判断位置的入口。

**怎么验。** 当前需求色为 1 时，`DISC_POSITIONS,2,1` 不应产生 `SELECTED`。再送 `DISC_POSITIONS,2,1,3,1`，应只选 3。颜色不匹配的候选仍不能抓。禁用位置的帧不增加进度，允许位出现后才继续。

[候选选择](../src/disc_material_decision.h) · [串口入口](../src/serial_commands.inc)

### 33 同色同时可抓时，优先选更稳定的位置

**题面。** 要求优先抓位置 2，其次 1，最后 3，仍逐件按任务颜色。

**怎么改。** `SelectDiscPosition()` 当前用位置编号较小者优先。在候选已验证是 1～3 的前提下，定义 `constexpr uint8_t rank[] = {0, 1, 0, 2};`。把比较位置编号的条件换成 `rank[candidates[i].position] < rank[candidates[selected].position]`，保留 `selected < 0 || ...` 的短路判断。颜色匹配条件保持原样。如果和第 32 题同用，先排掉禁用位置再排序。

**怎么验。** 当前色为 1，同时上报 `1,1,2,1,3,1` 应选 2。调换帧中三组顺序，仍选 2。然后把位置 2 改为其他颜色，应选 1。每个测试只让一个发报端提交候选。

[排序逻辑](../src/disc_material_decision.h)

### 34 抓取改为先从上方对准 再竖直下探

**题面。** 新底座有挡边，夹爪不能在抓取高度横向伸进去。第 24 题调整的是抓完离开，本题调整的是抓前进场。

**怎么改。** 在 `arm_motion.cpp` 的 `if(positionGrab)` 将原 `DISC_APPROACH` 拆为两个步骤。先以已验证的上方安全高度发 `arm(p,"DISC_ABOVE",safeH,length,angle,openAngle,approachSpeed,aboveMs);`，再发 `arm(p,"DISC_DESCEND",grabH,-1,-1,-1,descendSpeed,descendMs);`。示例变量由本工位实测值填入。第二步的三个 `-1` 保持伸缩、转台、夹爪状态。保留后面的 `DISC_GRIP`；若它仍重复发其他轴目标，要核对使用的是同一抓点。保持已有伸缩档位赋值覆盖进场步骤。

**怎么验。** 用空载慢动作确认横移在挡边以上完成，再放一件试抓。时间线上应为 `DISC_ABOVE → DISC_DESCEND → DISC_GRIP`。分别从三个物理位置验证，特别留意下降前夹爪与相邻件的间距。

[位置抓取步骤](../src/arm_motion.cpp)

### 35 不规则物料先轻夹找正 再夹紧

**题面。** 长棱柱接触时容易偏斜，要求先轻合到中间角，稳定后再闭到最终标定角。

**怎么改。** 在位置分支原 `DISC_GRIP` 前新增 `DISC_PRE_CLOSE`，以 `arm(p,"DISC_PRE_CLOSE",-1,-1,-1,preAngle,closeSpeed,preMs);` 只控制夹爪。保留 `DISC_GRIP` 作为最终夹紧，后面再抬升。`preAngle` 由实际接触点测出，不能填 `-1`；`preMs` 在计时模式决定这一步间隔。若启用反馈，核对中间角能否实际到达。`motion_executor.cpp` 的失败恢复会识别 `DISC_GRIP` 作为已进入夹取的边界，如果中间步已经接触物料，也把 `DISC_PRE_CLOSE` 纳入该边界判断。

**怎么验。** 装试件前单测两个角度，再看物料有没有在第一次合拢时被推走。验收顺序是中间角、最终夹紧、上提。测试在中间步后停止，确认恢复时不会把已接触的物料当作从未夹取。

[轴动作编排](../src/arm_motion.cpp) · [动作失败与接续](../src/motion_executor.cpp)

### 36 圆盘取料时加一个低高度试提

**题面。** 形状不规则，要求夹住后先提离圆盘一点，确认无卡住再继续大行程。

**怎么改。** 在位置分支 `DISC_GRIP` 后、`DISC_LIFT` 前加 `DISC_PROOF_LIFT`，高度用 `grabH + proofDeltaMm`，其他轴用 `-1`。例如抓点 80 mm，裁判要求试提 5 mm，则目标为 85 mm，高度仍是绝对值。按低速小行程实测决定步进完成条件，之后继续原提起和后续装盘。若题面要求自动判断抓牢，需要第 38 题的真实持料输入。

**怎么验。** 用刻度和视频观察试提期间伸缩轴没有动作，物料确实离开盘面，后续再提到安全高度。另试空夹，检查记录里不会把试提高度动作当作传感器确认的持料结果。

[位置序列](../src/arm_motion.cpp) · [步骤到位判断](../src/motion_hardware.cpp)

### 37 同一圆盘上有两种外形 按型号用不同夹持配方

**题面。** 圆柱抓得住，方块需要更宽张口及不同抓点。裁判在开局前给出本轮物料型号。

**怎么改。** 在 `MotionRequest` 加一个型号字段，默认沿用现有外形。由开局配置或获准使用的型号输入填入，并在 `serialQueueDiscGrab()` 提交时把型号复制到作业。`arm_motion.cpp` 的位置分支先按 `r.position` 选择基准 `height/length/angle`，再按型号给偏移、张口和夹紧角。颜色继续用于任务匹配；只有裁判明确指定颜色与外形一一对应时才用颜色推导型号。若型号逐件变化，字段来源也必须逐件对应。

**怎么验。** 同一个物理位置，用两种合法型号分别试抓。日志至少能区分型号与位置，动作应使用各自配方。完成一件后复查旧类型和下一件的参数没有被全局修改。

[抓取请求](../src/motion_types.h) · [入队处](../src/serial_commands.inc) · [动作生成](../src/arm_motion.cpp)

### 38 赛题要求传感器确认夹住后才许转运

**前提。** 车辆需要实际可用的持料检测，例如已装并验证的接触开关。项目现有夹爪目标位置反馈不能单独判断物料是否夹住。

**怎么改。** 在 `motion_sequence.h` 加一个专用检测步骤，例如 `GRIP_CHECK`。在 `arm_motion.cpp` 于夹紧后或第 36 题试提后加入。在 `MotionHardware::feedback()` 让此步走反馈，在 `ready()` 读取该作业开始后的新检测值，用 `done` 表示检测通过。按裁判规定使用允许的确认窗口和失败处理，保持已有串口维护。检测步骤必须在转运及 `CARGO_COMMIT` 之前。接触点已发生后，复核 `motion_executor.cpp` 的补抓边界。

**怎么验。** 分别用空爪、正确夹持和提起时滑脱三种情况测。未夹住应该停留在允许的检查流程或结束该作业，不能记录货位完成。给一条上个动作残留的旧信号，应仍需本次新观测。当前无传感器时，先验证裁判是否允许人工目检确认。

[步骤类型](../src/motion_sequence.h) · [硬件反馈](../src/motion_hardware.cpp) · [完成及恢复](../src/motion_executor.cpp)

### 39 偏心重物从圆盘转向料盘时减小转台冲击

**题面。** 直线伸缩合格，夹住后机械臂转向料盘时容易甩料。只调带料的转台动作。

**怎么改。** 在 `arm_motion.cpp` 找 `place()` 设置 `cargoPlaceTurn` 的位置和 `BuildArmProgram()` 最后的 `step.turretAcceleration` 赋值。以位置抓取为例，先保留最终赋值，在它后面仅当 `r.kind==MotionKind::DISC_GRAB && step.cargoPlaceTurn` 时把该步改为已标定的较缓档，例如从当前 `160` 试到 `135`。普通转台目标速度来自 `turret_motor.h` 的 `TURRET_MOTOR_POSITION_RPM`。如果裁判还要求只降低这一步的峰值转速，应给该步骤传独立 RPM，核对普通 `MoveArm()` 和 `MoveArmPlanned()` 的发令路径，避免改全局速度。

**怎么验。** 对比同角度、同负载的转向，观察物料有没有绕夹爪滑动。检查预抓转台和工作区取料等无关动作仍用原档。最后确认变慢后能在原动作时序内转到料盘。

[转向步骤](../src/arm_motion.cpp) · [转台档位](../src/turret_motor.h) · [运动下发](../src/chassis.cpp)

### 40 装有高件的旋转料盘必须慢速选盘

**题面。** 三个小料盘转位时物料会倾倒，要求自动选盘速度降为 180°/s，并延长启停斜坡。

**怎么改。** 在 `servo.cpp` 搜 `Servo_MoveCargoTray()`。此命令的 `speed` 单位为 `0.1°/s`。将 `3000` 改为 `1800`，示例 `rampMs` 从 `30` 改为 `80`，同一个变量填入加速和减速字段。它也用于部分归零和预选盘，若题面只要求有料时减速，应传入档位而非统一改常量。另查 `motion_hardware.cpp`：手动 `TRAY_ROTATE` 和让位分支走 `Servo_SetAngleMTurn()`，要覆盖哪些入口取决于题面。

**怎么验。** 用相同角度差对比空盘和满盘时间，观察急停时物料有没有晃倒。跑一件完整抓放，确认盘到位后再下放；手动旋转和自动选盘分别验证，以免测到不同接口。

[自动选盘协议](../src/servo.cpp) · [动作类型的调用区别](../src/motion_hardware.cpp)

### 41 大件会碰旁边的料盘 重新标定让位角

**题面。** 取 3 号盘的物料时，邻近 2 号盘已有高件，旧让位角空间不足。

**怎么改。** 在 `material_transfer.h` 搜 `CargoTrayNeedsClearance`、`CargoTrayClearanceAngle`。它按来源盘相邻的小一号盘是否有料决定是否加 `CARGO_TRAY_CLEARANCE_DEG`。从实物间隙确定需要的角度，保持每个盘基本对准角不动。另核对 `CargoTrayNeedsWork3Limit` 与 `WORK_POSITION_3_TRAY_TARGET_DEG`：当 1 号盘有料，去工作区 3 号位时可能在下放前触发另一次料盘移动。让位和限位要一起测。

**怎么验。** 用相同来源货位，分别测试邻盘空、邻盘有料，以及目的地为工位 3 的组合。观察让位后是否仍碰到车身、夹爪或其他物料。用地面标记或实测回读记录各阶段盘角，不以单次成功就认为所有组合可用。

[让位和工位 3 限制](../src/material_transfer.h) · [料盘发令](../src/motion_hardware.cpp)

### 42 只有一个工位不允许开启圆环纠偏

**题面。** 粗加工 2 号位没有可识别的标记，要求在那里按标定放料，其他工位仍用圆环纠偏。

**怎么改。** 在 `arm_motion.cpp` 的 `transfer()` 找向 `place()` 传 `!retrieve&&r.ringCorrection` 的位置。若规则适用于所有工区的 2 号位，加上 `works[i]!=2`。若只针对粗加工，先用请求字段区分粗加工与暂存，再按区域和编号组合判断。`place()` 用同一布尔值决定悬停是否等反馈以及是否开启纠偏。若题面仍要求该位用反馈确认悬停，把悬停到位与开启圆环相机拆成两个条件。

**怎么验。** 用任务码安排工位 `2→1→3`。对 2 号位无 `RING_TARGET` 和相机启动请求，1、3 号位仍正常纠偏。确认 2 号位的到位要求符合题面，且相机未启动时也能经过预放位置。

[工位与圆环步骤](../src/arm_motion.cpp) · [圆环事件](../src/motion_hardware.cpp)

### 43 裁判提高圆环精度 要求连续 4 帧达标

**题面。** 物料底座变窄，圆环纠偏要用更严格的像素误差，并要求 4 帧连续达标。

**怎么改。** 在参数目录找 `圆环纠偏/连续达标帧数`，改为 `4`。按相机标定把 `X死区(px)` 和 `Y死区(px)` 填为题面给定值。`ring_correction.h` 当前比较是 `fabs(error) < limit`，因此误差恰好等于阈值不会计入。如规则明确允许等于阈值，再调整这个比较式。先测当前相机帧率，核对 `圆环纠偏/超时(ms)` 能否容纳足够帧。若题面要求超时也不能下放，连同第 19 题的圆环超时分支处理。

**怎么验。** 用本次圆环会话的 token 按顺序送 3 帧合格、1 帧越界、再 4 帧合格。第 4 帧越界要清掉先前计数，最后一组收齐才应成功通过。再用旧 token 发送一帧，确认上一会话的输入不生效。

[圆环计数与会话](../src/ring_correction.h) · [参数](../src/runtime_parameters.cpp) · [串口格式](serial_protocol.md)

### 44 抓取顺序照旧 粗加工必须按工位 1、2、3 摆放

**题面。** 扫码颜色仍规定圆盘抓取顺序，粗加工放置按工位编号升序。

**怎么改。** 在 `COARSE_PLACE` 创建请求时给本轮设置一个“按工位升序”选项。`transfer()` 对本次放料建立由**任务下标**组成的顺序表，按 `r.positions[index]` 排序。构建转台计划和生成步骤的两个循环共用它。每件仍用 `CargoForGrabIndex(index)` 取所在料盘、`r.positions[index]` 定工位。不要只排序 `positions`。自动恢复事件在 `motion_executor.cpp` 用已完成数推算盘号和工位，也要用相同的排序结果定位下一件。

**怎么验。** 第一轮用位置段 `231`。盘 3、2、1 分别存原任务第 1、2、3 件，按新流程应依次从盘 **1、3、2** 卸到工位 **1、2、3**。三种颜色逐一核对。取回阶段保留原任务映射，还要试一次中断后继续是否指向正确货位。

[放料与路径计划](../src/arm_motion.cpp) · [接续定位](../src/motion_executor.cpp)

### 45 粗加工改为每放一件就加工并取回

**题面。** 工区一次只能加工一件，要按「放第 1 件→等待→取回→放第 2 件」串行执行。

**怎么改。** 在 `competition_flow.h` 给粗加工流程增加当前件索引和单件放置、等待、取回阶段。每次用本轮 `colors[i]`、`positions[i]` 与 `CargoForGrabIndex(i)` 创建 `TRANSFER`；它成功后记录 `due`，到时才用 `RETRIEVE` 从同一工位取回同一盘。每次取回成功后递增索引。三件全部取回后，再按暂存主状态进入 `TEMP_NAVIGATE`；这两个暂存状态原本会从粗加工取回开始，改走此方案时要选对进入的子阶段。单件首层仍要求圆环时，在单件请求传对应配置。

**怎么验。** 每次 `WORK_PLACE` 后同工位有一个 `WORK_PICK`，两者之间满足题面等待。检查上一件失败时索引不跳，和最后一次取回前不会发去暂存的路径。核对所有料盘最终颜色。

[流程与等待](../src/competition_flow.h) · [单件取放请求](../src/arm_motion.cpp)

### 46 本轮只搬前两件 禁止再抓第三件

**题面。** 任务条码仍发三件，裁判指定第三项本轮忽略。若新码只有两位，还需按第 08 题同步改格式。

**怎么改。** 在本轮业务请求中记录件数 `2`，让 `DISC_GRAB_MATERIALS` 达到两件就结束识别，`serialDiscPositions()` 和抓取完成事件也按本轮件数允许或终止新候选。`PLACE_ROUND`、`RETRIEVE_ROUND`、暂存放料的两个规划循环都要只处理有效的前两项。第二轮若仍按首轮配色叠放，只在实际执行的两件和首轮已摆下的物料里匹配，先确认题面允许的两轮颜色组合。三个物理料盘和工位编号范围可保留。

**怎么验。** 两次完成后出现去粗加工的请求，后续新候选不会追加第三件。第三个料盘仍空，粗加工与暂存各只发生两次放置。回归测一次设置回三件的正常流程。

[比赛进度](../src/competition_flow.h) · [串口候选](../src/serial_commands.inc) · [组动作循环](../src/arm_motion.cpp)

### 47 每摆一件都要等裁判确认再摆下一件

**题面。** 三件的顺序和位置不变，裁判用获准的按钮或串口信号逐件放行。

**怎么改。** 在 `COARSE_PLACE` 将整批 `PLACE_ROUND` 拆成由 `i` 驱动的单件 `TRANSFER`。第 1 件正常提交，成功后进入等待裁判阶段。接收端只在该阶段消费一次与当前轮次、件号匹配的新确认，再递增 `i` 并提交下一件。使用项目现有 `motion(request)` 的同一作业轮询模式。确认可通过已有面板输入的独立逻辑或新增获准的串口动作接入。保持最后一件成功后才进入原取回阶段。

**怎么验。** 未确认时停在当前阶段。连续发两次同一件的确认，最多进入一次下一件。发上一轮、下一件以及重启前残留的确认，验证它们不能提前放行。单件失败时沿用失败分支，不按裁判确认推进。

[状态机及作业轮询](../src/competition_flow.h) · [串口分发](../src/serial_commands.inc)

### 48 完赛要停在指定展示点 无需执行启停区角点归位

**题面。** 最后必须到裁判给定的节点停车展示，不能走返家补位。

**怎么改。** 在 `competition_flow.h` 最后一轮放料成功分支，接入展示停车阶段。先用 `ArmObservation::START` 收臂，再从最后停靠的暂存节点提交 `path(map.temp, showNode, ArmObservation::NONE)`，需要固定朝向时设置 `request.theta`。用可表达展示阶段的主状态和子阶段，接到现有 `motion(...)`。如果复用 `STATE_RETURN_HOME` 作为显示大阶段，就在切换后明确设置展示子阶段，跳过 `HOME_OBSERVE` 与 `HOME_RETURN` 的启停区专用动作。展示路径完成后才进 `STATE_DONE` 并调用 `io.runDone()`。

**怎么验。** 确认最后一次 `ROUTE_REQUEST` 的起点为实际暂存点、终点为裁判节点。检查最终朝向、停靠范围和计时结果，且没有旧角点启动或最后的启停区平移。

[流程编排](../src/competition_flow.h) · [终点航向选择](../src/motion_executor.cpp)

### 49 指定某条路线必须经过检查点

**题面。** 圆盘去粗加工要经过裁判给的节点，机载规划仍需绕开障碍。

**怎么改。** 机载端收到该起终点的 `ROUTE_REQUEST` 后，先求起点到检查点，再求检查点到终点，把接合处重复节点删掉。多检查点则按题面依次拼接。检查最终路由不超过 `MAX_NODE_PATH_LENGTH` 且相邻性成立。固件也要验证时，在 `motion_executor.cpp` 正式路径起终点校验后、开始圆盘离站补偿前，仅针对题目指定的业务路段检查是否经过节点，并沿用正式路径重请求的办法。若规则要求**在检查点停一下**，用两次导航作业和中间阶段实现，节点出现在数组里并不保证停车，共线节点可能合并。

**怎么验。** 构造一条起终点正确却漏点的回复，再构造合规回复。查日志 `ROUTE_PATH` 的实际节点顺序。若题面要求停留，再用视频和时间戳验证停留。

[导航作业与校验](../src/motion_executor.cpp) · [相邻节点规则](../src/continuous_trial.cpp) · [路由报文](serial_protocol.md)

### 50 扫码只准搜索一批 找不到就原地等待

**题面。** 扫码可以摆动搜索一批，禁止无期限重复摆扫。若允许裁判补码则原地等待。

**怎么改。** 打开 `motion_executor.cpp`，找 `scanAtNode()` 中 `scanTurn==stepsPerRound*maxRounds` 的分支。现有正式模式通过 `scanTurn=0` 重开一批。只给本题的正式扫描请求启用单批规则后，让用尽批次的分支复用 `stage=SCAN_WAIT_CODE`、`phase("SCAN_WAIT_CODE")` 及对应事件。已有等待分支继续调用 `pollTaskCode()`。题面要求的摆动轮数由 `maxRounds` 决定，一轮有四段。是否允许手输按第 07 题的授权口径，禁止手输时应同步限制 `TASK SET` 在该阶段的入口。

**怎么验。** 遮挡二维码，先看一批摆扫结束，再确认没有 `SCAN_RETRY,RESWEEP` 且料盘保持停止。补有效码后应归零并继续，无效码仍停留。正常一开始就扫码成功的情况也要回归。

[扫码状态与重试](../src/motion_executor.cpp) · [任务录入](../src/serial_commands.inc)

### 51 第二轮要送到独立的暂存 B 区 并按空工区单层摆放

**题面。** 第一轮留在原暂存区 A，第二轮去另一处空区 B，并用第二轮工位码摆第一层。

**怎么改。** 在地图配置加入 B 区节点和航向。`TEMP_NAVIGATE` 用 `secondRound` 选择 A 或 B，B 区导航请求显式设 `theta`；第一轮结束回圆盘仍从 A 出发。B 区为空，第二轮使用 `WORK_AREA`。在 `TEMP_PLACE` 的 B 区分支保留第四段工位，跳过按第一轮颜色覆盖位置的循环，并设置 `request.layer=1`。按题面选择圆环纠偏。若 B 区相对车体的工位布局不同，按第 17 题传独立位姿。启用返家时，`HOME_NAVIGATE` 起点也要用车辆最后所在的 B 区。

**怎么验。** 用 `156+123+516+231` 验证 A 区摆 `123`、B 区摆 `231`，都为首层。核对导航从粗加工到对应暂存区，且 B 区启用空工区对齐。若随后返家，首个回程节点必须是 B。

[工区导航、摆放与返家](../src/competition_flow.h) · [终点航向](../src/motion_executor.cpp) · [机械臂位姿](../src/arm_motion.cpp)

### 52 修改旋转料盘的取放料位置

**题面。** 换了料盘、内衬或物料后，机械臂要在新的位置从料盘取料，并把料放回料盘。先分清是某个盘没有转到窗口，还是三个盘对准窗口后机械臂都抓偏。旋转料盘是 **ID 3 舵机**，机械臂转台是 **EMM 地址 7**。两者各有自己的角度。

#### 先选要改的参数

以下为当前仓库 `robot_parameters.json` 的烧录默认值，连接实车后先读回核对。

| 现场现象 | 在 JSON 搜索 | 当前默认值与影响 |
| --- | --- | --- |
| 只有某一号盘转到窗口时偏左或偏右 | `搬运/旋转料盘1/角度(deg)`、`2`、`3` 对应条目 | `115°、180°、245°`，分别是三个料盘的绝对多圈角度 |
| 三个盘已对准，但夹爪伸得太远或不够 | `搬运/待操作区/伸出(mm)` | `37 mm`，三个盘共用 |
| 三个盘已对准，但机械臂转台偏向一侧 | `搬运/待操作区/转台(deg)` | 约 `-14.3°`，三个盘共用 |
| 取料时爪子下探太深或太浅 | `搬运/待操作区/高度(mm)` | `125 mm`，同时是装盘释放高度的基准 |
| 取料正常，但装盘松爪时单独要高一点 | `CARGO_TRAY_RELEASE_OFFSET_MM` | 在 `material_transfer.h` 默认 `3 mm`，装盘释放目标为 `基准高度 + 偏移` |
| 位置正确，但张开时爪尖钩盘或带起物料 | `搬运/载物台/夹爪张开(deg)` | `30°`，用于盘位抓取前张开及装盘松爪 |

例如，2 号盘从 `180°` 试到 `182°` 后能对正，且三个盘实际都需要多伸 `2 mm`，就分别把 `搬运/旋转料盘2/角度(deg)` 的 `value` 改为 `182`，把 `搬运/待操作区/伸出(mm)` 的 `value` 改为 `39`。如果**只想降低取料高度，保持装盘释放高度**，按后文算偏移。

#### 调试与保存步骤

1. **录原值。** 在 Debug 模式，停止视觉对齐并等待机械动作空闲。用参数页「一键读取」记下三个盘角与共用位姿。检查料盘内为空，机械臂收在可以安全旋转料盘的位置。除非重新做了物理零点标定，否则沿用已有料盘零点。
2. **先对盘，后对爪。** 使用「料盘旋转」控制，也可在 Debug 用 `{CMD,TRAY,ROTATE,182,1000,0}` 试 2 号盘的示例目标。参数依次是绝对角度、时长 ms、功率 mW，功率 `0` 沿用默认。看到 `TRAY,ROTATE,DONE` 后检查盘中心实际是否在窗口里。对三个盘分别标定，一个盘的偏差只改该盘的角度。
3. **再量共用位姿。** 把盘转到已标定的窗口位置。机械臂先保留高度余量，单独试转台角及伸出，确认爪尖对准后再逐步试下降。Debug 下 `ARM MOVE1` 的参数是 `高度、伸出、速度`，`ARM MOVE2` 是 `转台、夹爪、速度`。`-1` 让该轴保持。单轴命令的 `ISSUED` 表示指令发出，等实物停止并量到目标后再试下一步。
4. **单件双向验证。** 若用粗加工工位 1 和料盘 2 验证，车应已停在正确工区。先让工位 1 有试件、盘 2 为空，发 `{CMD,ARM,RETRIEVE,1,2}`，从工位 1 取回并放进盘 2。放稳后发 `{CMD,ARM,TRANSFER,2,1}`，从盘 2 取回并放回工位 1。观察装盘松爪时有没有钩料、取盘夹紧时是否夹正。调试取回会按内置颜色登记，仅用作单件几何验收。
5. **保存并回读。** 已有数值可先在上位机临时发送并回读，再保存为烧录默认值。直接改 `robot_parameters.json` 时只改目标项的 `value`，保存后需重新 Build 和通过 USB Upload。重启后再次读取参数确认生效。仅写 RAM 的调试值会在重启后恢复。

#### 取料和放料要分开调整时

- **只改装盘释放高度。** 固定 `待操作区/高度`，在 `src/material_transfer.h` 找 `CARGO_TRAY_RELEASE_OFFSET_MM`。例如基准 `125 mm`、旧释放目标 `128 mm`，实测需要 `130 mm`，将偏移由 `3.0f` 改为 `5.0f`。这个常量影响采用该计算的装盘入口，改后需烧录。
- **只改取料高度。** 示例从 `125` 降到 `123 mm`，同时要保持原来的释放目标 `128 mm`，则把共用高度改成 `123`、释放偏移改为 `5.0f`。如果只在一个入口调整，应在该入口分开传取料高度和装盘释放高度，其他入口继续沿用原设定。
- **只有某一个盘需要不同的机械臂位姿。** 现有 `materialTransferLayout.cargo` 是共用的一组。先确认改盘角无法解决偏差，再考虑按盘号增加独立位姿。在 `arm_motion.cpp` 的 `transfer()` 中，转台规划和实际生成动作两个循环都用同一盘的位姿。圆盘抓取后直接装盘的 `DISC_GRAB` 也要取对应盘位。`place()` 当前用 `&pose==&materialTransferLayout.cargo` 选择料盘专用张爪角，换成独立盘位对象时，应改为按 `destinationCargo` 判断，避免误用工位张爪角。

**验收。** 用同一件标准试件对三个货位分别做一轮装盘及取盘。分别记录盘角、取料高度、释放高度以及夹爪是否碰边。最后再测一组邻盘有料和工作区 3 号位的组合，复核第 41 题里的让位逻辑。若个别盘角度回读已到目标但实物仍偏，优先核对该盘的机械位置和标定角。

[配置参数](../robot_parameters.json) · [参数绑定](../src/runtime_parameters.cpp) · [动作编排](../src/arm_motion.cpp) · [装盘释放偏移](../src/material_transfer.h) · [调试命令](serial_protocol.md)

## 现场先做哪一步

- 题目落在**现有功能**，例如只指定 2 号区或允许手输任务码，先用现有面板或串口功能试通。
- 题目落在**速度、层高、工位位姿**，先定位 JSON 参数和它的消费位置，确定同一参数会影响哪些环节。
- 题目落在**区域互换、删减工序或改变放行条件**，先在纸面写出新的路线、阶段和完成事件，再去改对应的配置和状态机。
- 题目落在**任务码、视觉或导航协议**，验证固件和机载端使用同一份规则。离线测试用本地串口日志留证，别只看界面是否显示已发送。
