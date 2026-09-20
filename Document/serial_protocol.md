# 机载电脑与 ESP32 串口通信协议

本文档定义 Jetson/上位机与 ESP32 主控之间的 Serial0 文本协议。除人工可读的调试日志外，机器可读消息均使用本协议。

## 1. 基础帧格式

```text
{魔术字:参数1,参数2,...}\n
```

- 编码：ASCII（UTF-8 的 ASCII 子集）。
- 串口：115200 bit/s，8N1。
- 帧头、帧尾：`{`、`}`。
- 帧终止符：换行 `\n`；接收端也兼容 `\r\n`。
- 魔术字区分大小写，必须使用本文规定的拼写。
- 数值使用十进制，可带负号和小数点，不允许 `NaN`、`inf` 或缺失字段。
- 一行只允许一帧；花括号外不得附加其他字符。

无参数命令省略冒号，例如：

```text
{ready}
{start}
{ok}
{help}
```

## 2. 已实现：上位机发送给 ESP32

### 2.1 视觉闭环对齐

```text
{ALIGN:angle,x,y}
```

示例：

```text
{ALIGN:0,-120,-121}
```

字段：

| 字段 | 单位 | 含义 |
|---|---:|---|
| `angle` | ° | 上位机检测到的角度偏差 |
| `x` | 视觉单位 | 上位机检测到的 X 偏差 |
| `y` | 视觉单位 | 上位机检测到的 Y 偏差 |

上位机建议以 5 Hz 连续发送。ESP32 始终只保留最新帧，并采用分阶段校准：

1. `abs(angle) >= 0.5°` 时只校准角度，忽略该帧 X/Y。
2. `abs(angle) < 0.5°` 时只校准 X/Y。
3. `abs(angle) < 0.5°` 且 `abs(x) < 3`、`abs(y) < 3` 时判定对齐成功。

对齐成功后 ESP32 返回：

```text
{ALIGN:OK}
```

### 2.2 下发完整节点路径

```text
{way:n1-n2-...-nN}
```

示例：

```text
{way:1-2-3-6}
```

- 节点范围：`1`～`9`。
- 节点数：`2`～`16`。
- 相邻节点必须在 3×3 节点图中上下或左右相邻。
- ESP32 接受后返回 `{way:OK,N}`，其中 `N` 为节点数；格式或路径非法时返回 `{way:ERR}`。

### 2.3 设置启停区

```text
{StartZone:1}
{StartZone:2}
```

成功时 ESP32 返回：

```text
{StartZone:OK,1}
{StartZone:OK,2}
```

### 2.4 运行状态命令

```text
{ready}
{start}
{color:N}
{ok}
```

| 帧 | 状态 | 用途 |
|---|---|---|
| `{ready}` | 已实现 | 机载电脑报告通信与业务程序已就绪 |
| `{start}` | 已实现 | 请求主控启动业务状态机 |
| `{color:N}` | 已实现 | 报告识别到颜色编号 `N` |
| `{ok}` | 已实现 | 报告当前视觉动作已经确认到位 |

对应确认帧为 `{ready:OK}`、`{start:OK}`、`{color:OK}`、`{ok:ACK}`。

## 3. 已实现：ESP32 发送给上位机

### 3.1 请求规划路径

```text
{way:start-end?}
```

示例：

```text
{way:1-6?}
```

含义：ESP32 请求上位机规划从节点 1 到节点 6 的路径。上位机应返回包含首尾节点的完整路径，例如：

```text
{way:1-2-3-6}
```

问号只允许出现在 ESP32 发出的路径请求中，不得出现在上位机返回的可执行路径中。

### 3.2 通用错误

```text
{ERR:FRAME}
{ERR:UNKNOWN_FRAME}
```

- `FRAME`：缺少完整花括号或帧结构非法。
- `UNKNOWN_FRAME`：结构完整，但魔术字不受支持。

## 4. Debug 模式命令

以下命令用于仓库自带的 `upper_computer.py` 调试界面，也统一采用花括号帧：

```text
{GOTOpose:x_mm,y_mm,theta_deg}
{SetPose:x_mm,y_mm,theta_deg}
{Movepose:direction,speed,stop}
{MoveArm_1:height_mm,length_mm,speed}
{MoveArm_2:turret_deg,pawl_deg,speed}
{SERVO:id,angle_deg}
{En_C:enable}
{help}
```

`Movepose` 的 `direction`：`0` 前进、`1` 后退、`2` 左移、`3` 右移；`stop` 为 `0` 启动、`1` 停止。

Debug 命令接收后的机器确认帧使用以下形式：

```text
{GOTOpose:ACK,x_mm,y_mm,theta_deg}
{SetPose:ACK,x_mm,y_mm,theta_deg}
{Movepose:ACK,direction,speed,stop}
{MoveArm_1:ACK,height_mm,length_mm,speed}
{MoveArm_2:ACK,turret_deg,pawl_deg,speed}
{SERVO:ACK,id,angle_deg}
{En_C:ACK,enable}
```

`ACK` 仅表示 ESP32 已接收并开始处理命令，不代表车辆或机械臂已经物理到位。

## 5. 预留协议

以下帧用于后续业务扩展，目前不得依赖 ESP32 已经执行：

```text
{Task:156+123+516+231}
{Grab:color,position}
{Place:area,position,layer}
{Obstacle:x,y,radius}
{Pose:x_mm,y_mm,theta_deg}
{Heartbeat:sequence}
```

建议的完成/失败回复形式：

```text
{Task:OK}
{Grab:OK}
{Place:OK}
{ERR:魔术字,错误码}
```

新增协议时必须同时更新 ESP32 解析逻辑、上位机发送逻辑和本文档，禁止恢复无花括号的裸字符串格式。
