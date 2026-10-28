"""转盘半自动调参：固定参数往返试验、原始数据与指标。"""
import csv
import math
from dataclasses import dataclass, field
import tkinter as tk
from tkinter import ttk, filedialog, messagebox


@dataclass
class TurnRecord:
    trace_id: str
    start_ms: int
    target: float
    start: float
    profile: dict
    samples: list = field(default_factory=list)
    result: str = ""

    def metrics(self, tolerance=1.0):
        if not self.samples:
            return None
        sign = 1 if self.target >= self.start else -1
        overshoot = max(0, max(sign * (s[1] - self.target) for s in self.samples))
        last_bad = max((i for i, s in enumerate(self.samples) if abs(s[1] - self.target) > tolerance), default=-1)
        stable = self.samples[last_bad + 1][0] if last_bad + 1 < len(self.samples) else None
        # 至少观察0.5秒的连续带内数据，才报告稳定时间。
        if stable is not None and self.samples[-1][0] - stable < 500:
            stable = None
        tail = [s[1] for s in self.samples if s[0] >= self.samples[-1][0] - 500]
        intervals = [b[0] - a[0] for a, b in zip(self.samples, self.samples[1:])]
        return dict(overshoot=overshoot, settling_ms=stable,
                    final_error=self.samples[-1][1] - self.target,
                    tail_p2p=max(tail) - min(tail), samples=len(self.samples),
                    mean_interval=sum(intervals) / len(intervals) if intervals else 0,
                    max_interval=max(intervals, default=0))


def trial_settings(values):
    a, b, speed = (float(values[k]) for k in ("a", "b", "speed"))
    ramp, period, duration, cycles = (int(values[k]) for k in ("ramp", "period", "duration", "cycles"))
    if not all(math.isfinite(v) for v in (a, b, speed)) or not all(-150 <= v <= 360 for v in (a, b)):
        raise ValueError("角度须在 -150～360° 内且数值有限")
    if not 1 <= speed <= 65535 * 6 or not 0 <= ramp <= 65535:
        raise ValueError("速度或加减速时间超出协议表示范围")
    if not 1 <= period <= 0xFFFFFFFF or not 1 <= duration <= 0xFFFFFFFF or cycles < 1:
        raise ValueError("采样周期、观察时长和往返次数须为正整数")
    return dict(a=a, b=b, speed=speed, ramp=ramp, period=period, duration=duration, cycles=cycles)


class TurretTuningPanel(ttk.LabelFrame):
    def __init__(self, parent, send, log, pid_values, motor):
        super().__init__(parent, text="7 号转盘 · 半自动 PID 试验", padding=8)
        self.send, self.log, self.pid_values, self.motor = send, log, pid_values, motor
        self.active = False
        self.phase = "idle"
        self.records = []
        self.current = None
        self.pending = []
        self.settings = {}
        self.variables = {}
        for row, (key, label, default) in enumerate((
            ("a", "角度 A（°）", "0"), ("b", "角度 B（°）", "90"),
            ("speed", "速度（°/s）", "250"), ("ramp", "加减速时间（ms）", "100"),
            ("period", "请求采样周期（ms）", "20"), ("duration", "每段观察时长（ms）", "3000"),
            ("cycles", "往返次数（A→B→A）", "2"))):
            variable = self.variables[key] = tk.StringVar(value=default)
            ttk.Label(self, text=label).grid(row=row, column=0, sticky="w")
            ttk.Entry(self, textvariable=variable, width=12).grid(row=row, column=1, sticky="w", pady=2)
        self.status = tk.StringVar(value="先读取7号PID或输入参数，再开始试验。")
        buttons = ttk.Frame(self); buttons.grid(row=7, column=0, columnspan=2, sticky="w", pady=6)
        self.start_button = ttk.Button(buttons, text="写入 RAM 并开始试验", command=self.start_trial)
        self.start_button.pack(side="left")
        ttk.Button(buttons, text="停止试验", command=self.stop_trial).pack(side="left", padx=4)
        ttk.Button(buttons, text="导出 CSV", command=self.export_csv).pack(side="left")
        ttk.Label(self, textvariable=self.status, wraplength=440).grid(row=8, column=0, columnspan=2, sticky="w")
        self.canvas = tk.Canvas(self, width=430, height=210, background="white", highlightthickness=1)
        self.canvas.grid(row=9, column=0, columnspan=2, sticky="ew", pady=6)
        self.canvas.bind("<Configure>", lambda _: self.draw())
        view = ttk.Frame(self)
        view.grid(row=10, column=0, columnspan=2, sticky="w")
        self.view_mode = tk.StringVar(value="角度")
        chooser = ttk.Combobox(view, textvariable=self.view_mode, values=("角度", "位置误差", "转速"), state="readonly", width=10)
        chooser.pack(side="left")
        chooser.bind("<<ComboboxSelected>>", self.select_record)
        self.tail_only = tk.BooleanVar(value=False)
        ttk.Checkbutton(view, text="放大末 0.5 秒", variable=self.tail_only, command=self.select_record).pack(side="left", padx=8)
        self.table = ttk.Treeview(self, columns=("pid", "target", "over", "settle", "jitter", "error"), show="headings", height=5)
        for col, title in zip(self.table["columns"], ("Kp/Ki/Kd", "目标°", "过冲°", "稳定ms", "末0.5s峰峰°", "末误差°")):
            self.table.heading(col, text=title); self.table.column(col, width=135 if col == "pid" else 75, stretch=True)
        self.table.grid(row=11, column=0, columnspan=2, sticky="ew")
        self.table.bind("<<TreeviewSelect>>", self.select_record)
        ttk.Label(self, wraplength=440, text="开始后先移动到 A 并记录，再按 A→B→A 往返。每段从发令起记录完整观察时长，不因到位提前结束；时长需覆盖运动和静止观察。蓝线为实测值，橙线为目标。稳定时间按±1°且持续到记录结束（至少0.5秒）计算。末段峰峰值不一定代表静止抖动，须结合曲线。采样周期为请求值，实际间隔随通信耗时变化；CSV保留原始编码器、累计计数、转速、标志和查询耗时。").grid(row=12, column=0, columnspan=2, sticky="w", pady=6)
        self.columnconfigure(1, weight=1)

    def transmit(self, line):
        self.send(line); self.log("TX", line)

    def start_trial(self):
        if self.active:
            return
        try:
            if self.motor.get() != "7":
                raise ValueError("请先选择7号电机并读取PID，再开始转盘试验")
            self.settings = trial_settings({k: v.get() for k, v in self.variables.items()})
            from emm_panel import build_emm_command
            line = build_emm_command("PID_SET", 7, *(v.get() for v in self.pid_values), 0)
            self.settings["pid"] = tuple(int(v.get()) for v in self.pid_values)
            s = self.settings
            self.pending = [s["a"]] + [v for _ in range(s["cycles"]) for v in (s["b"], s["a"])]
            self.active = True; self.phase = "pid"
            self.start_button.configure(state="disabled")
            self.transmit(line)
            self.status.set("等待7号PID写入回读确认；随后开始运动。")
        except (ValueError, RuntimeError, OSError) as exc:
            self.disconnected(str(exc))
            messagebox.showerror("试验未启动", str(exc), parent=self)

    def next_turn(self):
        if not self.pending:
            self.disconnected("本组试验完成，可修改 PID 后比较下一组。")
            return
        self.phase = "command"
        self.current = None
        s = self.settings
        target = self.pending.pop(0)
        try:
            self.transmit(f"{{CMD,EMM,TURRET_TRIAL,{target},{s['speed']},{s['ramp']},{s['period']},{s['duration']}}}")
            self.status.set(f"试验目标 {target}°；剩余 {len(self.pending)} 段")
        except (RuntimeError, OSError) as exc:
            self.disconnected(str(exc))

    def stop_trial(self):
        was_active = self.active
        self.disconnected("已请求停止试验，已有数据保留。")
        if was_active:
            try:
                self.transmit("{CMD,NAV,STOP}")
            except (RuntimeError, OSError) as exc:
                self.status.set(str(exc))

    def disconnected(self, reason="连接或模式改变，试验序列已取消"):
        self.active = False; self.phase = "idle"; self.pending = []
        self.start_button.configure(state="normal")
        self.status.set(reason)

    def receive(self, f):
        if not self.active:
            return
        if f[:2] == ("RSP", "EMM") and f[3] == "ERR":
            self.disconnected("试验失败：" + f[4]); return
        if self.phase == "pid" and f[:4] == ("RSP", "EMM", "PID_SET", "READBACK") and f[4] == "7":
            actual = tuple(map(int, f[5:8]))
            if actual != self.settings["pid"]:
                self.disconnected("PID回读与本组参数不一致"); return
            self.next_turn()
        elif f[:3] == ("EVT", "TURRET", "COMMAND") and self.phase == "command":
            self.current = TurnRecord(f[3], int(f[4]), float(f[5]), float(f[6]), dict(self.settings, actual_rpm=int(f[7]), acceleration=int(f[8])))
            self.records.append(self.current)
            self.phase = "samples"
        elif f[:3] == ("EVT", "TURRET", "SAMPLE") and self.current and f[3] == self.current.trace_id:
            # elapsed, angle, rpm, flags, encoder, position, query_ms
            elapsed = (int(f[4]) - self.current.start_ms) & 0xFFFFFFFF
            self.current.samples.append((elapsed, float(f[6]), int(f[7]), int(f[8]),
                                         int(f[10]), int(f[11]), int(f[12])))
            self.draw()
        elif f[:3] == ("EVT", "TURRET", "RESULT") and self.current and f[3] == self.current.trace_id:
            self.current.result = f[5]
            self.add_result(self.current)
        elif f[:3] == ("EVT", "EMM", "TURRET_TRIAL"):
            if f[3] == "DONE":
                self.next_turn()
            else:
                self.disconnected("试验中断：" + f[3])

    def add_result(self, record):
        m = record.metrics()
        if m:
            self.table.insert("", "end", iid=str(len(self.records)-1), values=("/".join(map(str, record.profile["pid"])), record.target,
                f"{m['overshoot']:.3f}", "未稳定" if m['settling_ms'] is None else m['settling_ms'],
                f"{m['tail_p2p']:.3f}", f"{m['final_error']:.3f}"))
            self.status.set(f"采样 {m['samples']} 点，平均间隔 {m['mean_interval']:.1f}ms，最大 {m['max_interval']}ms")

    def select_record(self, _=None):
        selected = self.table.selection()
        if selected:
            self.draw(self.records[int(selected[0])])
        else:
            self.draw()

    def draw(self, record=None):
        record = record or self.current
        c = self.canvas; c.delete("all")
        if not record or not record.samples:
            return
        w, h = max(c.winfo_width(), 300), max(c.winfo_height(), 180)
        samples = record.samples
        if self.tail_only.get():
            samples = [s for s in samples if s[0] >= samples[-1][0] - 500]
        mode = self.view_mode.get()
        measurement = lambda s: s[2] if mode == "转速" else s[1] - record.target if mode == "位置误差" else s[1]
        target = record.target if mode == "角度" else 0
        values = [measurement(s) for s in samples] + [target]
        span = max(values) - min(values)
        margin = max(span * 0.05, 0.01)
        low, high = min(values) - margin, max(values) + margin
        start_ms = samples[0][0] if self.tail_only.get() else 0
        end_ms = samples[-1][0]
        duration = max(end_ms - start_ms, 1)
        x = lambda t: 45 + (t - start_ms) / duration * (w - 60)
        y = lambda a: h - 25 - (a - low) / (high - low) * (h - 70)
        unit = "RPM" if mode == "转速" else "°"
        c.create_text(4, 4, anchor="nw", text=f"{mode} {low:.3f}～{high:.3f}{unit}   PID {record.profile['pid']}")
        m = record.metrics()
        c.create_text(4, 20, anchor="nw", text=f"{m['samples']}点；间隔均值 {m['mean_interval']:.1f}ms / 最大 {m['max_interval']}ms")
        c.create_line(45, y(target), w-15, y(target), fill="#d97706", dash=(5, 3))
        points = [v for s in samples for v in (x(s[0]), y(measurement(s)))]
        if len(points) >= 4:
            c.create_line(*points, fill="#2563eb", width=2)
        c.create_text(w-10, h-8, anchor="e", text=f"{start_ms}～{end_ms} ms")

    def export_csv(self):
        path = filedialog.asksaveasfilename(parent=self, defaultextension=".csv", filetypes=[("CSV", "*.csv")])
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8-sig") as stream:
                writer = csv.writer(stream)
                writer.writerow(("record", "trace", "kp", "ki", "kd", "speed_deg_s", "ramp_ms", "requested_period_ms", "duration_ms", "actual_command_rpm", "acceleration", "target_deg", "start_deg", "result", "elapsed_ms", "angle_deg", "rpm", "flags", "encoder", "position_counts", "query_ms"))
                for i, r in enumerate(self.records):
                    p = r.profile
                    for sample in r.samples:
                        writer.writerow((i, r.trace_id, *p["pid"], p["speed"], p["ramp"], p["period"], p["duration"], p["actual_rpm"], p["acceleration"], r.target, r.start, r.result, *sample))
            self.status.set(f"已导出 {len(self.records)} 段数据：{path}")
        except OSError as exc:
            messagebox.showerror("导出失败", str(exc), parent=self)
