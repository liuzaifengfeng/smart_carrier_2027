"""连续试跑的纯轨迹预览、分帧协议和 Tk 面板。坐标/圆弧规则与固件一致。"""
from __future__ import annotations

import math
import secrets
import time
from dataclasses import dataclass
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from field_geometry import OBSTACLES


def angle_delta(start: float, end: float) -> float:
    value = (end - start) % 360
    return value - 360 if value > 180 else value


@dataclass(frozen=True)
class TrialConfig:
    start: float = 180
    end: float = 270
    rpm: float = 80
    accel: float = 200
    lateral_accel: float = 300
    lateral_ppm: float = 13.3
    forward_ppm: float = 13.6
    kp: float = 2
    omega: float = 45
    correction: float = 20
    policy: int = 0
    outset: float = 60
    margin: float = 20

    def validate(self):
        limits = ((self.start, 0, 359.999), (self.end, 0, 359.999), (self.rpm, 1, 5000),
                  (self.accel, 1, 2000), (self.lateral_accel, 1, 3000),
                  (self.lateral_ppm, .01, 1000), (self.forward_ppm, .01, 1000),
                  (self.kp, .1, 10), (self.omega, 5, 90), (self.correction, 1, 89))
        names = ("起始航向", "终点航向", "最大轮速", "平移加速度", "横向加速度", "横移脉冲/mm",
                 "前进脉冲/mm", "航向Kp", "总角速度", "纠偏角速度")
        for name, (x, lo, hi) in zip(names, limits):
            if not math.isfinite(x) or not lo <= x <= hi:
                raise ValueError(f"{name}超出范围：应为 {lo}～{hi}")
        if self.policy not in (0, 1, 2) or not 0 <= self.outset <= 100 or not 10 <= self.margin <= 40:
            raise ValueError("策略/外绕量/安全余量无效（外绕0～100mm，余量10～40mm）")
        if self.correction >= self.omega:
            raise ValueError("纠偏角速度上限必须小于总角速度上限")
        if self.rpm != int(self.rpm):
            raise ValueError("最大轮速必须为整数RPM")


@dataclass(frozen=True)
class Segment:
    start: tuple[float, float]
    end: tuple[float, float]
    length: float
    offset: float
    center: tuple[float, float] | None = None
    radius: float = 0
    angle: float = 0
    sweep: float = 0

    def at(self, distance):
        t = max(0, min(1, (distance - self.offset) / self.length))
        if self.center is None:
            return tuple(a + (b - a) * t for a, b in zip(self.start, self.end))
        a = self.angle + self.sweep * t
        return self.center[0] + self.radius * math.cos(a), self.center[1] + self.radius * math.sin(a)


def adjacent(a, b):
    ar, ac = divmod(a, 5)
    br, bc = divmod(b, 5)
    if ar % 2:
        ac = 4 - ac
    if br % 2:
        bc = 4 - bc
    return abs(ar - br) + abs(ac - bc) == 1


class TrialPlan:
    def __init__(self, nodes, radii, positions, config: TrialConfig):
        config.validate()
        if not 2 <= len(nodes) <= 25 or len(radii) != len(nodes):
            raise ValueError("需要 2～25 个节点及对应半径")
        if any(not isinstance(n, int) or not 0 <= n < 25 for n in nodes):
            raise ValueError("节点编号应为 0～24")
        if any(not adjacent(a, b) for a, b in zip(nodes, nodes[1:])):
            raise ValueError("相邻输入节点必须在网格上相邻")
        if any(not math.isfinite(r) or not 0 <= r <= 1000 for r in radii):
            raise ValueError("半径范围为 0～1000 mm")
        if radii[0] or radii[-1]:
            raise ValueError("起终点半径必须为0")
        self.nodes, self.radii, self.config = tuple(nodes), tuple(radii), config
        self.positions = dict(positions)
        self.segments = []
        self.length = 0.
        points = [tuple(map(float, positions[n])) for n in nodes]
        if any(not math.isfinite(v) for p in points for v in p):
            raise ValueError("地图坐标必须为有限数")
        directions, distances = [], []
        for a, b in zip(points, points[1:]):
            d = math.dist(a, b)
            if d < 20:
                raise ValueError("节点间距不足20mm")
            distances.append(d)
            directions.append(((b[0]-a[0])/d, (b[1]-a[1])/d))
        trims = [0.] * len(nodes)
        for i in range(1, len(nodes)-1):
            a, b = directions[i-1], directions[i]
            dot = a[0]*b[0]+a[1]*b[1]
            if dot > .9999:
                continue
            if abs(dot) > .001:
                raise ValueError("第一版仅支持直线及90°拐角，不支持掉头")
            if radii[i] < 20:
                raise ValueError(f"节点{nodes[i]}未允许切弯，或半径小于20mm")
            trims[i] = radii[i]
        if any(trims[i]+trims[i+1] > d-20 for i, d in enumerate(distances)):
            raise ValueError("圆弧互相重叠或剩余直线不足20mm，请减小半径")

        def line(a, b):
            d = math.dist(a, b)
            if d > .001:
                self.segments.append(Segment(a, b, d, self.length))
                self.length += d

        cursor = points[0]
        for i in range(1, len(nodes)):
            p, incoming, r = points[i], directions[i-1], trims[i]
            entry = (p[0]-incoming[0]*r, p[1]-incoming[1]*r)
            line(cursor, entry)
            if r:
                outgoing = directions[i]
                end = (p[0]+outgoing[0]*r, p[1]+outgoing[1]*r)
                center = (entry[0]+outgoing[0]*r, entry[1]+outgoing[1]*r)
                sweep = math.copysign(math.pi/2, incoming[0]*outgoing[1]-incoming[1]*outgoing[0])
                self.segments.append(Segment(entry, end, r*math.pi/2, self.length,
                                             center, r, math.atan2(entry[1]-center[1], entry[0]-center[0]), sweep))
                self.length += r*math.pi/2
                cursor = end
            else:
                cursor = p
        if math.ceil(self.length/5)+1 > 4096:
            raise ValueError("路径长度超出固件速度规划容量")
        delta = abs(angle_delta(config.start, config.end))
        step = min(5, min((r for r in trims if r), default=100000)*math.pi/180,
                   self.length/(delta*1.875) if delta else 5)
        count = math.ceil(self.length/step)
        if count > 200000:
            raise ValueError("轮廓检查采样过多")
        self.preview = []
        for i in range(count+1):
            s = self.length*i/count
            x, y = self.at(s)
            heading = self.heading(s)
            corners = self.corners(x, y, heading)
            if any(not 20 <= px <= 2380 or not 20 <= py <= 2380 for px, py in corners):
                raise ValueError(f"轨迹约{s:.0f}mm处车体轮廓越界或边界余量不足20mm")
            self.preview.append((s, x, y, heading))

    def at(self, s):
        for segment in self.segments:
            if s <= segment.offset+segment.length:
                return segment.at(s)
        return self.segments[-1].end

    def heading(self, s):
        u = max(0, min(1, s/self.length))
        return (self.config.start+angle_delta(self.config.start, self.config.end)*u**3*(10-15*u+6*u*u)) % 360

    @staticmethod
    def corners(x, y, heading):
        a = math.radians(heading)
        return [(x+rx*math.sin(a)+fy*math.cos(a), y-rx*math.cos(a)+fy*math.sin(a))
                for rx in (-150, 150) for fy in (-150, 150)]

    def commands(self, job_id, build_frame):
        c = self.config
        fmt = lambda x: f"{x:.6g}"
        frames = [build_frame("CMD", "NAV", "TRIAL_BEGIN", job_id, len(self.nodes),
                              *(fmt(v) for v in (c.start, c.end, c.rpm, c.accel, c.lateral_accel,
                                                c.lateral_ppm, c.forward_ppm, c.kp, c.omega, c.correction)))]
        frames.extend(build_frame("CMD", "NAV", "TRIAL_NODE", job_id, i, n, fmt(r),
                                  *(fmt(v) for v in self.positions[n]))
                      for i, (n, r) in enumerate(zip(self.nodes, self.radii)))
        frames.append(build_frame("CMD", "NAV", "TRIAL_START", job_id))
        return frames


class ContinuousTrialPanel(ttk.Frame):
    def __init__(self, parent, send, log, build_frame, parse_frame, nodes_provider,
                 heading_provider=None, calibrate=None, on_result=None):
        super().__init__(parent, padding=10)
        self.send, self.log, self.build_frame, self.parse_frame = send, log, build_frame, parse_frame
        self.nodes_provider = nodes_provider
        self.heading_provider = heading_provider or (lambda: 180.)
        self.calibrate_callback, self.result_callback = calibrate, on_result
        self.job_id = 0
        self.phase = "idle"
        self.queue = []
        self.pending = None
        self.ack_timer = self.heartbeat_timer = None
        self.variables = {}
        self.plan = None
        self.last_rx = time.monotonic()
        self.telemetry = []
        self.formal_running = False
        self.motion_stage = ""
        ttk.Label(self, text="安全混合路径 · Debug 独立试跑", font=("Microsoft YaHei UI", 12, "bold")).pack(anchor="w")
        form = ttk.Frame(self)
        form.pack(fill=tk.X)
        fields = (("path", "节点路径", "11-12-17"), ("rpm", "最大轮速(RPM)", "80"),
                  ("policy", "转弯策略", "全程停转"),
                  ("start", "起始地图航向(°)", "当前已标定朝向"), ("end", "终点地图航向(°)", "沿末段自动"))
        choices = {"policy": ("全程停转", "自动混合"),
                   "start": ("当前已标定朝向", "0", "90", "180", "270"),
                   "end": ("沿末段自动", "0", "90", "180", "270")}
        for row, (name, label, default) in enumerate(fields):
            self.variables[name] = tk.StringVar(value=default)
            ttk.Label(form, text=label).grid(row=row, column=0, sticky="w", padx=3)
            widget = (ttk.Combobox(form, textvariable=self.variables[name], values=choices[name], width=19)
                      if name in choices else ttk.Entry(form, textvariable=self.variables[name], width=23))
            widget.grid(row=row, column=1, sticky="ew")
        self.advanced = ttk.Frame(self)
        self.show_advanced = tk.BooleanVar(value=False)
        ttk.Checkbutton(self, text="展开高级参数（通常不需修改）", variable=self.show_advanced,
                        command=self._toggle_advanced).pack(anchor="w")
        fields = (("accel", "平移加速度(mm/s²)", "200"),
                  ("lateral_accel", "横向加速度上限(mm/s²)", "300"),
                  ("lateral_ppm", "独立横移脉冲/mm", "13.3"), ("forward_ppm", "独立前进脉冲/mm", "13.6"),
                  ("kp", "航向Kp(1/s)", "2"), ("omega", "总角速度上限(°/s)", "45"),
                  ("correction", "纠偏角速度上限(°/s)", "20"),
                  ("outset", "优先外绕量(mm)", "60"), ("margin", "规划安全余量(mm)", "20"))
        for i, (name, label, default) in enumerate(fields):
            group, row = divmod(i, 5)
            self.variables[name] = tk.StringVar(value=default)
            ttk.Label(self.advanced, text=label).grid(row=row, column=group*2, sticky="w", padx=3)
            ttk.Entry(self.advanced, textvariable=self.variables[name], width=10).grid(row=row, column=group*2+1, sticky="ew")
        self.calibration_button = ttk.Button(self, text="已摆到首节点：校准起点", command=self.calibrate_start)
        self.calibration_button.pack(anchor="w", pady=3)
        if calibrate is None:
            self.calibration_button.configure(state=tk.DISABLED)
        buttons = ttk.Frame(self)
        buttons.pack(fill=tk.X, pady=6)
        ttk.Button(buttons, text="预览轨迹", command=self.preview).pack(side=tk.LEFT)
        ttk.Button(buttons, text="上传并试跑", command=self.start).pack(side=tk.LEFT, padx=6)
        ttk.Button(buttons, text="停车 / 取消", command=self.stop).pack(side=tk.LEFT)
        ttk.Button(buttons, text="导出本次采样CSV", command=self.export_csv).pack(side=tk.LEFT, padx=6)
        self.status = tk.StringVar(value="先回读参数地图，按实车位置校准起点，再预览；从80RPM低速验证")
        ttk.Label(self, textvariable=self.status, wraplength=580).pack(fill=tk.X)
        self.canvas = tk.Canvas(self, width=500, height=390, background="#f7fafc")
        self.canvas.pack(fill=tk.BOTH, expand=True, pady=5)
        ttk.Label(self, text="预览按地图X向右、Y向上；蓝线为轨迹，绿色框为车体。\n"
                  "黄色为已知禁入区；自动混合：中心窄路停转，外围角点尝试向外绕，空间不足则停转或拒绝。\n"
                  "红点为停稳确认点。零指令后按H30角速/加速度安静窗口接续；直线PI纠偏及转速阻尼，距离仍为估算。\n"
                  "状态心跳每500ms发送，失联1500ms停车；运行中其他运动指令被固件拒绝。",
                  wraplength=580).pack(fill=tk.X)

    def _toggle_advanced(self):
        if self.show_advanced.get():
            self.advanced.pack(fill=tk.X, before=self.calibration_button)
        else:
            self.advanced.pack_forget()

    def calibrate_start(self):
        if self.phase in ("uploading", "starting", "running", "stopping"):
            self.status.set("请先停车 / 取消，再校准起点")
            return
        try:
            plan = self.read_plan()
            if messagebox.askokcancel("实际位置确认", f"请确认实车中心已经摆在节点{plan.nodes[0]}，车头地图航向为{plan.config.start:g}°。\n此操作仅标定，不会移动车辆。", parent=self):
                self.calibrate_callback(*plan.positions[plan.nodes[0]], plan.config.start)
                self.status.set("已发送起点校准，请确认日志中POSE SET及地图航向成功后再试跑")
        except (ValueError, RuntimeError, OSError, KeyError) as exc:
            self.status.set(str(exc))

    def read_plan(self):
        from hybrid_plan import HybridPlan
        positions = self.nodes_provider()  # 必须来自已回读的固件参数
        raw = self.variables["path"].get().strip().replace(",", "-")
        nodes = [int(v.strip()) for v in raw.split("-")]
        start = self.variables["start"].get().strip()
        start = float(self.heading_provider()) if start == "当前已标定朝向" else float(start)
        end = self.variables["end"].get().strip()
        auto_end = end == "沿末段自动"
        end = start if auto_end else float(end)
        policy = self.variables["policy"].get()
        if policy not in ("自动混合", "全程停转"):
            raise ValueError("转弯策略请选择自动混合或全程停转")
        config = TrialConfig(start=start, end=end, policy=1 if policy == "自动混合" else 2,
                             **{key: float(v.get()) for key, v in self.variables.items() if key not in ("path", "start", "end", "policy")})
        return HybridPlan(nodes, positions, config, auto_end=auto_end)

    def preview(self):
        try:
            self.plan = self.read_plan()
            self.draw(self.plan)
            turns = sum(p.kind == "ROTATE" for p in self.plan.segments[:-1])
            if abs(angle_delta(self.plan.segments[-1].h0, self.plan.segments[-1].h1)) > .05:
                turns += 1
            curves = sum(p.kind == "CUBIC" for p in self.plan.segments)
            self.status.set(f"预览通过：{self.plan.length:.1f}mm，停转{turns}处、外绕{curves}处；终点{self.plan.config.end:g}°。仍需检查实际障碍和定位误差")
            return self.plan
        except (ValueError, RuntimeError, KeyError) as exc:
            self.plan = None
            self.canvas.delete("all")
            self.status.set(str(exc))
            return None

    def draw(self, plan):
        c = self.canvas
        c.delete("all")
        w, h = max(400, c.winfo_width()), max(320, c.winfo_height())
        scale = min((w-30)/2400, (h-30)/2400)
        xy = lambda x, y: (15+x*scale, 15+(2400-y)*scale)
        c.create_rectangle(*xy(0, 2400), *xy(2400, 0), outline="#8091a5")
        for left, right, bottom, top in OBSTACLES:
            c.create_rectangle(*xy(left, top), *xy(right, bottom), fill="#fff5c2", outline="#e6d798")
        for i, p in self.nodes_provider().items():
            x, y = xy(*p)
            c.create_text(x, y, text=str(i), fill="#8593a3")
        flat = [v for _, x, y, _ in plan.preview for v in xy(x, y)]
        c.create_line(*flat, fill="#1976d2", width=3)
        for phase in plan.segments:
            if phase.kind == "ROTATE":
                px, py = xy(*phase.start)
                c.create_oval(px-4, py-4, px+4, py+4, fill="#d45441", outline="")
        stride = max(1, len(plan.preview)//12)
        for _, x, y, a in plan.preview[::stride]+[plan.preview[-1]]:
            corners = plan.corners(x, y, a)
            ordered = [corners[i] for i in (0, 1, 3, 2)]
            c.create_polygon(*(v for p in ordered for v in xy(*p)), fill="", outline="#1b8d72")
            angle = math.radians(a)
            c.create_line(*xy(x, y), *xy(x+150*math.cos(angle), y+150*math.sin(angle)),
                          fill="#1b8d72", arrow=tk.LAST)

    def start(self):
        if self.phase in ("uploading", "starting", "running", "stopping"):
            self.status.set("已有试跑或上传在进行，请先停车 / 取消")
            return
        plan = self.preview()
        if plan is None:
            return
        if not messagebox.askokcancel("确认试跑区域", "已按实车朝向标定地图航向、摆到首节点，且预览中车体扫过区域无障碍？", parent=self):
            return
        self.job_id = secrets.randbelow(9999999)+1
        self.telemetry.clear()
        self.queue = plan.commands(self.job_id, self.build_frame)
        self.phase = "uploading"
        self._next()

    def _send(self, line):
        self.send(line)
        self.log("TX", line)

    def _next(self):
        if not self.queue:
            return
        self.pending = self.queue.pop(0)
        action = self.pending.split(",")[2]
        try:
            self._send(self.pending)
        except (RuntimeError, OSError) as exc:
            self.disconnected(str(exc))
            return
        if action == "TRIAL_START":
            self.phase = "starting"
            self.last_rx = time.monotonic()
            self._heartbeat()
        self.ack_timer = self.after(2000, self._ack_timeout)
        self.status.set(f"等待固件 {action} 确认，试跑编号 {self.job_id}")

    def _cancel(self, name):
        timer = getattr(self, name)
        if timer is not None:
            self.after_cancel(timer)
            setattr(self, name, None)

    def _ack_timeout(self):
        self.ack_timer = None
        # START 不自动重发，避免重复执行；请求停车并等待状态确认。
        self.status.set("配置或启动确认超时，已请求停车；请查询日志并重新标定位姿")
        self.stop()

    def _heartbeat(self):
        self.heartbeat_timer = None
        if self.phase not in ("starting", "running", "stopping"):
            return
        if time.monotonic()-self.last_rx > 2:
            self.shutdown()
            self.status.set("超过2秒未收到试跑状态，已请求停车并停止心跳；重新确认位姿后再试")
            return
        try:
            self._send(self.build_frame("CMD", "NAV", "TRIAL_STATUS", self.job_id))
        except (RuntimeError, OSError) as exc:
            self.disconnected(str(exc))
            return
        self.heartbeat_timer = self.after(500, self._heartbeat)

    def stop(self):
        if self.formal_running:
            try:
                self._send(self.build_frame("CMD", "NAV", "STOP"))
                self.status.set("已请求正式节点导航停车，等待结果")
            except (RuntimeError, OSError) as exc:
                self.status.set(str(exc))
            return
        self.queue.clear()
        self._cancel("ack_timer")
        if self.phase in ("starting", "running", "stopping"):
            self.phase = "stopping"
            try:
                self._send(self.build_frame("CMD", "NAV", "TRIAL_STOP", self.job_id))
            except (RuntimeError, OSError) as exc:
                self.disconnected(str(exc))
                return
            self.status.set("已请求停车，等待固件确认停止")
        else:
            self.phase = "idle"
            self.status.set("已取消上传；尚未启动试跑")
        self.pending = None

    def receive(self, text):
        f = self.parse_frame(text)
        if not f or len(f) < 5 or f[1] != "NAV":
            if f and f[:3] == ("EVT", "NAV", "ROUTE_RUNNING"):
                self.formal_running = True
                self.status.set("正式节点导航运行中：H30实时纠偏、路口停稳后接续")
            elif f and f[:3] in (("EVT", "NAV", "ROUTE_FAILED"), ("EVT", "NAV", "ROUTE_DONE")):
                self.formal_running = False
                self.status.set("正式节点导航："+" / ".join(f[2:]))
            return
        if f[0] == "EVT" and f[2] in ("TRIAL_PHASE", "ROUTE_PHASE") and len(f) == 5:
            if f[2] == "TRIAL_PHASE" and f[3] != str(self.job_id):
                return
            names = {"HEADING_HOLD": "运动中航向纠偏", "SETTLE_BEFORE_TURN": "等待车体安静",
                     "TURN": "转向闭环", "FINAL_HEADING": "终点航向闭环", "SETTLE_AFTER_TURN": "转向后停稳确认"}
            names.update(POSITION_STRAIGHT="位置直行：H30辅助纠偏", POSITION_TURN="位置转向：辅助修正并确认停稳", POSITION_RECOVER="末端残差：低速自动修正", POSITION_SETTLE="交接停稳：自动重新确认")
            names.update(OUTER_DRIFT="外侧路径：连续移动", OUTER_CURVE="外侧转角：外绕连续过弯")
            self.motion_stage = names.get(f[4], f[4])
            self.status.set(self.motion_stage)
            return
        if f[0] == "EVT" and f[2] == "ROUTE_RESULT" and len(f) == 9:
            self.formal_running = False
            self.status.set(f"正式节点导航 {f[4]} / {f[5]}；位置估算X={f[6]} Y={f[7]}，航向={f[8]}°")
            callback = self.result_callback
            if callback:
                callback(*(float(v) for v in f[6:9]))
            return
        if f[0] == "EVT" and f[2] in ("TRIAL_SAMPLE", "TRIAL_RESULT"):
            if f[3] != str(self.job_id):
                return
            self.last_rx = time.monotonic()
            if f[2] == "TRIAL_SAMPLE" and len(f) == 14:
                self.telemetry.append((f"{time.monotonic():.3f}", *f[3:]))
                if len(self.telemetry)>10000:
                    self.telemetry.pop(0)
                self.status.set(f"行程 {f[4]}/{f[5]}mm；参考 {f[6]}°，实测 {f[7]}°，误差 {f[8]}°；周期 {f[13]}ms")
            elif f[2] == "TRIAL_RESULT" and len(f) == 9:
                self._terminal(f[4], f"{f[4]} / {f[5]}：估算 X={f[6]} Y={f[7]}，实测航向={f[8]}°")
                callback = self.result_callback
                if callback:
                    callback(*(float(v) for v in f[6:9]))
            return
        if f[0] != "RSP" or not f[2].startswith("TRIAL_"):
            return
        if f[4] != str(self.job_id):
            if f[3] == "ERR" and len(f) == 5 and self.pending is not None and f[2] == self.pending.split(",")[2]:
                self.stop()
                self.status.set("试跑命令被拒绝："+f[4])
            return
        self.last_rx = time.monotonic()
        if f[3] == "ERR":
            reason = "试跑被拒绝："+",".join(f[5:])
            if "START_POSITION" in f[5:]:
                reason += "；估算位置距首节点超过25mm，请实际摆正后使用起点校准"
            if self.phase in ("starting", "running", "stopping"):
                try:
                    self._send(self.build_frame("CMD", "NAV", "TRIAL_STOP", self.job_id))
                except (RuntimeError, OSError) as exc:
                    reason += f"；停车请求失败：{exc}"
            self._terminal("FAILED", reason)
            return
        if f[2] == "TRIAL_STATUS" and f[3] == "OK" and len(f) == 12:
            if f[5] in ("DONE", "FAILED"):
                self._terminal(f[5], f"{f[5]} / {f[6]}：估算X={f[9]} Y={f[10]}，航向={f[11]}°")
            return
        if self.pending is None or f[3] != "ACK":
            return
        sent = self.pending.rstrip("}").split(",")
        if f[2] != sent[2] or (f[2] == "TRIAL_NODE" and (len(f) != 6 or f[5] != sent[4])):
            return
        self._cancel("ack_timer")
        self.pending = None
        if f[2] == "TRIAL_START":
            self.phase = "running"
            self.status.set("连续试跑中；位置为估算值，可随时请求停车")
        else:
            self._next()

    def _terminal(self, state, message):
        self.phase = state.lower()
        self.queue.clear()
        self.pending = None
        self._cancel("ack_timer")
        self._cancel("heartbeat_timer")
        self.status.set(message)

    def disconnected(self, reason="连接断开"):
        self._terminal("FAILED", reason+"；固件失去心跳后停车，重新试跑前确认位姿")

    def shutdown(self):
        if self.phase in ("starting", "running", "stopping"):
            try:
                self._send(self.build_frame("CMD", "NAV", "TRIAL_STOP", self.job_id))
            except (RuntimeError, OSError) as exc:
                self.disconnected(f"停车请求失败：{exc}")
                return
        self.disconnected()

    def export_csv(self):
        import csv
        path = filedialog.asksaveasfilename(parent=self, defaultextension=".csv",
                                          initialfile=f"trial_{self.job_id}.csv", filetypes=[("CSV", "*.csv")])
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8-sig", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(("host_monotonic_s", "id", "s_mm", "total_mm", "reference_deg", "heading_deg",
                                 "error_deg", "rpm1", "rpm2", "rpm3", "rpm4", "cycle_ms"))
                writer.writerows(self.telemetry)
        except OSError as exc:
            self.status.set(f"导出失败：{exc}")
