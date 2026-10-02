# 小车运行流程

本文规定小车从上电准备、任务码获取、两批物料搬运到返回启停区的动作流程。

## 流程概览

上电与启停区选择 → 扫描与任务码获取 → 第一批物料搬运 → 第二批物料搬运 → 返回初始启停区与复位。

每批物料均按“圆盘区取料 → 粗加工区放料并重新取料 → 暂存区放料”的顺序搬运。

## 视觉功能速查

视觉功能以 **加粗** 标注；串口事件、指令和函数调用以行内代码标注。

| 视觉功能 | 启动事件 | 结束事件 | 使用步骤 |
| --- | --- | --- | --- |
| **圆盘定位视觉** | `{EVT,VISION,START_REQUEST,DISC}` | `{EVT,VISION,STOP_REQUEST,DISC}` | 7、14 |
| **圆盘物料识别视觉** | `{EVT,VISION,START_REQUEST,DISC_MATERIAL}` | `{EVT,VISION,STOP_REQUEST,DISC_MATERIAL}` | 8、15 |
| **暂存区/粗加工区定位视觉** | `{EVT,VISION,START_REQUEST,WORK_AREA}` | `{EVT,VISION,STOP_REQUEST,WORK_AREA}` | 9、12、16、19 |
| **暂存区/粗加工区带物料视觉** | `{EVT,VISION,START_REQUEST,WORK_AREA_LOADED}` | `{EVT,VISION,STOP_REQUEST,WORK_AREA_LOADED}` | 11、18 |
| **角点视觉识别** | `{EVT,VISION,START_REQUEST,CORNER}` | `{EVT,VISION,STOP_REQUEST,CORNER}` | 22 |

上位机“视觉”页可手动触发以上五种功能：发送 `{CMD,VISION,START_REQUEST,模式}`，小车先回复 `{RSP,VISION,START_REQUEST,ACK,模式}`，再发出表中的启动事件。机载电脑视觉程序收到事件后开启对应功能；接令及请求事件均不代表视觉已启动或已完成。定位功能需要另行开启对齐并回传视觉数据。

每种功能的“结束”按钮发送 `{CMD,VISION,STOP_REQUEST,模式}`，小车先回复 `{RSP,VISION,STOP_REQUEST,ACK,模式}`，再发出表中的结束事件；固件可调用 `requestVisionStop(VisionStartMode::模式)`。机载电脑收到事件后结束对应识别和反馈。结束请求不代表任务成功，也不证明相机已经停止；连续 PID 对齐需另行通过 `{CMD,VISION,ALIGN_STOP}` 停车并清零。

目标流程中的结束时机如下（首轮圆盘对齐切换已接入，其余阶段仍待接入）：

- 圆盘定位 `DISC`：步骤 8、15 对齐完成后结束，再开启 `DISC_MATERIAL`。
- 圆盘物料 `DISC_MATERIAL`：步骤 8、15 完成该批三件物料抓取后结束，再离开圆盘区。
- 工位定位 `WORK_AREA`：步骤 10、13、17、20 完成放料／码放后结束，再切换视觉或离开工位。
- 带物料视觉 `WORK_AREA_LOADED`：步骤 11、18 完成三件物料取回后结束，再离开粗加工区。
- 角点识别 `CORNER`：步骤 22 取得有效到达确认后结束，再进入复位步骤；不能用结束请求代替到达确认。

以下为目标运行流程。当前固件在首轮圆盘路径结束后自动开启单次 PID 对齐并请求 `DISC`；机载电脑直接发送 `ALIGN_DATA`，无需再发 `ALIGN_START`。对齐成功后锁定停车、请求结束 `DISC`，再请求 `DISC_MATERIAL`，每次切换只执行一次。等待首帧及整个对齐最多 15 秒；运动反馈断流 300 ms、总对齐超时或手动启停打断时，流程停止并进入对齐故障状态，不开启物料识别。手动 `ALIGN_START` 会取消本次自动等待。

首轮物料识别启动后，旋转圆盘的逐件取料时机、抓取完成计数仍待接入；不会直接调用不等待视觉的 `LoadRoundFromDisc`。其余视觉切换、回家导航与角点结果判定仍需接入对应业务阶段；五种功能均已支持上位机手动请求。

## 一、上电准备与任务码获取

1. 开局，小车上电，机械臂复位；若不复位，则不满足规则的尺寸要求。
2. 抽签后，小车被放置任一启停区。
3. 在机载电脑的屏幕上点击选择启停区，小车通过串口接收。
4. 得到启停区信息后，小车稍作位移，并将雷达（机械臂）摆放到指定位置进行扫描（机载电脑处理雷达信息）。
5. 扫描完成后（接收到 `start` 指令），小车根据所在区域继续向前行驶，若未接收到扫码信息，则重试三次。
6. 接收到任务码后，发送给机载电脑进行显示。

## 二、第一批物料搬运

### 圆盘区取料

7. 请求扫码区到圆盘区的路径节点，移动完成后发送 `{EVT,VISION,START_REQUEST,DISC}`，开启 **圆盘定位视觉**。
8. 视觉对齐完成并停车后，先发送 `{EVT,VISION,STOP_REQUEST,DISC}`，再发送 `{EVT,VISION,START_REQUEST,DISC_MATERIAL}`，开启 **圆盘物料识别视觉**，再抓取物料到载物台（逐件抓取实现待定）。三件抓取完成后请求结束 `DISC_MATERIAL`，再进入粗加工阶段。

### 粗加工区放料与取料

9. 请求从圆盘区到粗加工区路径并移动，移动完成后发送 `{EVT,VISION,START_REQUEST,WORK_AREA}`，开启 **暂存区/粗加工区定位视觉**。
10. 视觉对齐完成后，在粗加工区按第一批位置码放第一层： `PlaceTaskCargoToWorkArea(currentTask.round1_pos, 1)`。
11. 放料完毕后发送 `{EVT,VISION,START_REQUEST,WORK_AREA_LOADED}`，开启 **暂存区/粗加工区带物料视觉**，再重新抓取到载物台。

### 暂存区放料

12. 请求从粗加工区到暂存区路径并移动，移动完成后发送 `{EVT,VISION,START_REQUEST,WORK_AREA}`，开启 **暂存区/粗加工区定位视觉**。
13. 视觉对齐完成后，在暂存区按第一批位置码放第一层： `PlaceTaskCargoToWorkArea(currentTask.round1_pos, 1)`。

## 三、第二批物料搬运

### 圆盘区取料

14. 请求从暂存区到圆盘区路径并移动，移动完成后发送 `{EVT,VISION,START_REQUEST,DISC}`，开启 **圆盘定位视觉**。
15. 视觉对齐完成后发送 `{EVT,VISION,START_REQUEST,DISC_MATERIAL}`，开启 **圆盘物料识别视觉**，再抓取物料到载物台（具体实现待定）。

### 粗加工区放料与取料

16. 请求从圆盘区到粗加工区路径并移动，移动完成后发送 `{EVT,VISION,START_REQUEST,WORK_AREA}`，开启 **暂存区/粗加工区定位视觉**。
17. 视觉对齐完成后，在粗加工区按第二批位置码放第一层： `PlaceTaskCargoToWorkArea(currentTask.round2_pos, 1)`。
18. 放料完毕后发送 `{EVT,VISION,START_REQUEST,WORK_AREA_LOADED}`，开启 **暂存区/粗加工区带物料视觉**，再重新抓取到载物台。

### 暂存区放料

19. 请求从粗加工区到暂存区路径并移动，移动完成后发送 `{EVT,VISION,START_REQUEST,WORK_AREA}`，开启 **暂存区/粗加工区定位视觉**。
20. 视觉对齐完成后，在暂存区按第二批位置码放第二层： `PlaceTaskCargoToWorkArea(currentTask.round2_pos, 2)`。

## 四、返回与复位

21. 放料任务完成，请求回家路径（初始启停区）。
22. 回家路径执行结束后，发送 `{EVT,VISION,START_REQUEST,CORNER}`（固件接口：`requestVisionStart(VisionStartMode::CORNER)`），请求开启 **角点视觉识别**。由机载电脑识别角点并确认初始启停区位置；路径开环结束和请求事件本身均不能确认实际到达。角点结果回传与固件到达判定待接入。
23. 任务完成，复位机械臂。
