# 安全混合路径独立试跑

仅替换 Debug“连续试跑”的规划与执行，不改变比赛自动导航。窄路保持车身沿道路，不再把起终航向插值到整条路线；外围可行时向外绕角。位置仍按下发轮速与时间估算，H30闭环只确认航向，完成事件保留 `ESTIMATED`。

## 加速度能否确认直线完成

H30协议包含三轴加速度（0x10）、角速度（0x20）和姿态（0x40）。当前固件发布姿态，未用加速度作到位判断。静止和匀速时运动加速度都可接近0；检测到减速也不能证明走够目标距离。原始加速度包含重力，扣重力误差、振动、零偏会让积分速度/位移漂移。因此直线仍用脉冲标定和实际限幅轮速积分估算，不能把“加速度小”当成到位。若要确认真实X/Y，需要驱动器位置反馈/编码器，并结合视觉等外部定位修正打滑误差。

## 简化上位机操作

1. 同时更新固件与工程上位机，连接后完整回读参数地图；只读回读不需要开关视觉。
2. 输入相邻节点路径、最大轮速（初试80RPM）、转弯策略。无需逐节点填写半径。
3. 起始航向可选“当前已标定朝向”（取主界面估计值，并非自动读取当前H30），也可直接填写地图绝对角度。0°朝地图+X、90°朝+Y。终点默认“沿末段自动”，或手动指定。
4. **先把实车中心摆到首节点、核对实车朝向**，再点“已摆到首节点：校准起点”。这只发送POSE SET并建立H30地图参考，不会把车开到起点。确认日志POSE SET和IMU REFERENCE成功后再启动。
5. 预览：黄色为已知禁入区，蓝线为中心轨迹，绿色为稀疏车身轮廓，红点为停转/终点确认点。预览失败会清空旧图，避免误用上一条轨迹。
6. 空车、机械臂收拢，从80RPM低速试跑；随时可停车/取消。完成后界面同步估算位姿，采样可导出CSV。

高级参数默认折叠，包含加速度、航向Kp、角速度上限、横移/前进脉冲每毫米、优先外绕量60mm和规划余量20mm。仅作用于本次独立试跑，不写入比赛默认参数。安全余量10～40mm是规划条件，不是实车精度保证；不要靠减小余量强行通过。

## 两种策略

- **自动混合（1）**：中心窄路直行，转向必须先减速到零、在有完整车身转动空间的节点原地转，再继续直行。外围四角尝试向外鼓出的三次曲线，而非向黄色区域内切；不可行则退回停转，停转也不安全就拒绝整条路线。
- **全程停转（2）**：所有道路拐角停转，不生成移动转向曲线。直线中间节点不停车。

例如默认地图 `11-12-17` 在12处停转；`14-15-24-23-22`、起90°终180°在24附近外绕。外绕几何与车头角度分别连续规划，仍可能短暂横移；它不是通过主动打滑实现的漂移。车头自动选择沿道路正向/反向中转角较小的方向，平局选择正向，因此允许倒退。原路返回可保持车头不变，但仍停稳确认后换向。

外围外绕在每个候选外绕量下同时校验正向和倒退出弯朝向，优先保留可行外绕；不会因为先选中的朝向碰撞就立即停转。出弯朝向传递到后续路段，“沿末段自动”也使用最终选中的朝向。例如 `10-19-20-21-22` 起始180.855°时，20处可从270°转到180°倒退出弯，而非因270°转0°不可行退回停转。仍保留原安全余量与完整车身碰撞检查。

外绕同时限制姿态角加速度：90°/s²变化率预算中，几何曲率变化使用45、弯中平移加速使用30，余下15留给纠偏。提高最大轮速并不取消弯中限速，避免速度前馈增长快于实际转向能力、继而触发航向保护。截图路径300RPM、平移加速度500、总角速度90的参数已纳入无滑移控制器回归，不代表实车300RPM已验收。

## 几何与控制条件

- 共享 `src/field_geometry.def` 定义2400mm场地、300mm方形车体和四块黄色区：X/Y分别为550～1000、1400～1850mm的四种组合。主地图、预览、固件一致；上传校验几何签名，版本不一致不能启动。
- 使用完整旋转矩形碰撞检查（SAT），不仅检查车体中心/四角；边界和黄色区都包含规划余量。移动按≤2mm、姿态约≤1°密集采样，原地转向检查整个角度扫掠。未建模工位、突出机械臂、物料及临时障碍仍需人工检查。
- 300mm方形车体45°投影宽约424mm，400mm直路上不能随意原地转；只能在经验证有空间的路口转。
- 直线恒定道路航向；外绕段用五次平滑插值分配该拐角航向。轮速、曲率、横向加速度、角速度及下一停转点的制动距离共同限速。转向采用角速度前馈加H30航向P纠偏。
- 原地转向完成：航向误差≤1°，由连续新姿态样本计算的角速度≤3°/s，持续至少120ms。重复样本不增加完成计数，**不是到达预计等待时间就宣布旋转完成**。原地转向角速度≤min(45,配置总上限)°/s。
- 控制周期40ms，四轮先装载速度、再同步提交；包含组间发送等待，积分使用限幅和整数化后的实际命令。运行时对估算XY及实测航向再次检查车身碰撞，但这不能发现未被定位测到的实际滑移。
- 直线和外绕的末端距离仍是估算，不使用加速度假装确认X/Y。转向超时上限为 `2×转角/min(45,总角速)+3秒`；到位后无需耗尽该上限。

## 启动与停车

启动要求：Debug、完整配置及策略、邻接节点、坐标和几何版本一致、内部估算距首节点≤25mm、H30参考有效、样本年龄≤100ms、实际航向距起始航向≤5°。25mm只检查内部估算，不代表实车摆放误差已测量。

运行中H30失效/过期、地图参考改变、控制周期>120ms、移动时航向误差>5°持续200ms、估算车身越界/碰障碍、转向超时、用户停车或OTA都会终止并停车。全程限时为计划时长2倍+5秒。STATUS心跳每500ms续约，固件1500ms无心跳停车；上位机2秒无状态也停止续约并请求停车。

失败或主动停车后必须重新确认实车位置并POSE SET校准，再使用新编号上传。STOP ACK仅表示接令，需等RESULT/终态。软件超时不是硬件急停和停距保证。

## 协议

115200 8N1，v2，每帧换行。id=1～9999999。逐帧ACK后继续：

```text
{CMD,NAV,TRIAL_BEGIN,id,count,start,end,rpm,accel,lateral_accel,lateral_ppm,forward_ppm,kp,max_omega,correction}
{CMD,NAV,TRIAL_POLICY,id,mode,outset,margin,geometry_signature}
{CMD,NAV,TRIAL_NODE,id,index,node,0,expected_x,expected_y}
...完整count个NODE，index从0开始...
{CMD,NAV,TRIAL_START,id}
```

POLICY的mode=1/2，outset=0～100mm为优先外绕量，规划会尝试其他候选以满足轮廓约束；margin=10～40mm。当前geometry_signature=1831827565，由共享文件计算。未上传策略拒绝 `POLICY_REQUIRED`，防止旧上位机按旧预览启动新路线。NODE保留半径字段以维持帧结构，新策略忽略该字段，上位机固定传0。坐标差异>0.05mm、上传后地图改变均拒绝；START冻结旋转标定。

```text
{RSP,NAV,TRIAL_BEGIN,ACK,id}
{RSP,NAV,TRIAL_POLICY,ACK,id}
{RSP,NAV,TRIAL_NODE,ACK,id,index}
{RSP,NAV,TRIAL_START,ACK,id}
{CMD,NAV,TRIAL_STATUS,id}
{RSP,NAV,TRIAL_STATUS,OK,id,state,reason,s,total,estimated_x,estimated_y,heading}
{CMD,NAV,TRIAL_STOP,id}
{RSP,NAV,TRIAL_STOP,ACK,id}
{EVT,NAV,TRIAL_SAMPLE,id,s,total,reference,heading,error,rpm1,rpm2,rpm3,rpm4,cycle_ms}
{EVT,NAV,TRIAL_RESULT,id,DONE,ESTIMATED,estimated_x,estimated_y,heading}
{EVT,NAV,TRIAL_RESULT,id,FAILED,reason,estimated_x,estimated_y,heading}
```

状态IDLE/RUNNING/STOPPING/DONE/FAILED。语义错误 `{RSP,NAV,TRIAL_动作,ERR,id,reason}`；格式/范围/MODE/BUSY沿用原错误帧。新增几何/策略原因：POLICY_REQUIRED、GEOMETRY_MISMATCH、POLICY_RANGE、OBSTACLE、INITIAL_TURN_SPACE、CORNER_TURN_SPACE、FINAL_TURN_SPACE、TURN_TIMEOUT。其他包括START_POSITION、START_HEADING、RECALIBRATE、MAP_MISMATCH、MAP_CHANGED、NOT_ADJACENT。

## 验证

```powershell
python -m unittest discover -s tests -p 'test_hybrid_route.py'
python -m unittest discover -s tests -p 'test_continuous_trial.py'
python -m unittest discover -s tests -p 'test_parameter_panel.py'
& 'C:\Users\LZF\.platformio\penv\Scripts\platformio.exe' run
```

主机C++仿真执行实际continuous_route.cpp和hybrid_route.cpp，覆盖外围外绕、中心停转、全停转、原路返回、禁止窄路原地转、碰撞、样本稳定判定，并与Python预览和几何签名比较。使用CONTINUOUS_TEST_CXX，或忽略目录.pio/host-test-tools中的Zig；没有主机编译器时对应测试跳过。

本次没有烧录或驱动实车。实车仍需低速重复验证路径、载物外廓、实际位置误差、H30掉线及停车行为；仿真不能证明真实抓地和停距。没有通过前不要直接替换比赛路线或提高转速。
