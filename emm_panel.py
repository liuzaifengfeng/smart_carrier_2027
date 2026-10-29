"""EMM V5 驱动器参数调试页。"""
import tkinter as tk
from decimal import Decimal, InvalidOperation
from tkinter import ttk, messagebox
from turret_tuning import TurretTuningPanel

DRIVE_FIELDS = ("最大输出电压（mV）", "开环工作电流（mA）", "堵转最大电流（mA）",
                "堵转检测转速（RPM）", "堵转检测电流（mA）", "堵转检测时间（ms）", "位置到达窗口（°）")


def window_to_raw(value):
    try:
        raw = Decimal(str(value).strip()) * 10
        if not raw.is_finite() or raw != raw.to_integral_value() or not 0 <= raw <= 65535:
            raise ValueError("位置到达窗口须为0～6553.5°，步进0.1°")
        return int(raw)
    except InvalidOperation as exc:
        raise ValueError("位置到达窗口须填写数字，步进0.1°") from exc


def build_emm_command(action, motor, *values):
    limits = {"PID_GET": (), "PID_SET": (0xFFFFFFFF,) * 3 + (1,),
              "DRIVE_GET": (), "DRIVE_SET": (65535,) * 7 + (1,),
              "STATE_GET": (), "MODE_SET": (3, 1), "ENABLE": (1,), "CLEAR_STALL": ()}
    if action == "DRIVE_SET" and len(values) == 8:
        values = (*values[:6], window_to_raw(values[6]), values[7])
    numbers = [int(str(v).strip()) for v in (motor, *values)]
    if action not in limits or len(values) != len(limits[action]):
        raise ValueError("EMM 命令参数数量错误")
    if not 1 <= numbers[0] <= 7:
        raise ValueError("电机地址为 1～7")
    if any(not 0 <= v <= maximum for v, maximum in zip(numbers[1:], limits[action])):
        raise ValueError("PID 为 uint32，驱动字段为 uint16 整数；保存/使能为0或1，模式为0～3")
    return "{CMD,EMM," + action + "," + ",".join(map(str, numbers)) + "}"


class EmmPanel(ttk.Frame):
    lift_tuning = extension_tuning = None

    def __init__(self, parent, send, log, parse):
        super().__init__(parent, padding=12)
        self.send, self.log, self.parse = send, log, parse
        self.motor = tk.StringVar(value="1")
        self.gains = [tk.StringVar() for _ in range(3)]
        self.save = tk.IntVar(value=0)
        self.mode = tk.StringVar(value="2：闭环脉冲输入")
        self.status = tk.StringVar(value="请选择电机并读取 PID。")
        self.feedback = tk.StringVar(value="尚无电机状态")
        ttk.Label(self, text="张大头 EMM V5 参数", font=("Microsoft YaHei UI", 14, "bold")).pack(anchor="w")
        row = ttk.Frame(self); row.pack(fill="x", pady=12)
        ttk.Label(row, text="电机地址：").pack(side="left")
        ttk.Combobox(row, textvariable=self.motor, values=tuple(map(str, range(1, 8))), state="readonly", width=5).pack(side="left")
        self.motor.trace_add("write", self.changed_motor)
        ttk.Label(self, text="1～4：底盘　5：升降　6：伸出　7：转盘").pack(anchor="w")
        group = ttk.LabelFrame(self, text="驱动器位置环 PID（原始整数）", padding=10)
        group.pack(fill="x", pady=12)
        for name, variable in zip(("Kp", "Ki", "Kd"), self.gains):
            row = ttk.Frame(group); row.pack(fill="x", pady=3)
            ttk.Label(row, text=name, width=6).pack(side="left")
            ttk.Entry(row, textvariable=variable, width=18).pack(side="left")
        ttk.Checkbutton(group, text="写入驱动器 Flash（掉电保存）", variable=self.save).pack(anchor="w", pady=6)
        row = ttk.Frame(group); row.pack(fill="x")
        ttk.Button(row, text="读取 PID", command=lambda: self.command("PID_GET")).pack(side="left")
        ttk.Button(row, text="写入并回读", command=lambda: self.command("PID_SET", *(v.get() for v in self.gains), self.save.get())).pack(side="left", padx=8)
        group = ttk.LabelFrame(self, text="驱动参数（截图红圈项目）", padding=10)
        group.pack(fill="x", pady=(0, 12))
        self.drive_values = [tk.StringVar() for _ in DRIVE_FIELDS]
        self.drive_save = tk.IntVar(value=0)
        self.drive_status = tk.StringVar(value="尚未读取该电机驱动配置")
        for label, variable in zip(DRIVE_FIELDS, self.drive_values):
            row = ttk.Frame(group); row.pack(fill="x", pady=2)
            ttk.Label(row, text=label, width=25).pack(side="left")
            ttk.Entry(row, textvariable=variable, width=14).pack(side="left")
        ttk.Checkbutton(group, text="驱动参数写入 Flash（掉电保存）", variable=self.drive_save).pack(anchor="w", pady=5)
        row = ttk.Frame(group); row.pack(fill="x")
        ttk.Button(row, text="读取驱动参数", command=lambda: self.command("DRIVE_GET")).pack(side="left")
        ttk.Button(row, text="写入并回读驱动参数", command=lambda: self.command("DRIVE_SET", *(v.get() for v in self.drive_values), self.drive_save.get())).pack(side="left", padx=8)
        ttk.Label(group, textvariable=self.drive_status, wraplength=440).pack(anchor="w", pady=5)
        ttk.Label(group, wraplength=440, text="窗口按0.1°设置。写入前读取完整配置，仅替换这7项；其余配置沿用电机实读值。开环工作电流用于开环模式。到达窗口只影响驱动器到位判定，调参曲线的±1°统计标准不变。数值输入范围按协议表示，具体可用范围由驱动器型号决定。").pack(anchor="w")
        group = ttk.LabelFrame(self, text="其他驱动参数与状态", padding=10)
        group.pack(fill="x")
        ttk.Combobox(group, textvariable=self.mode, state="readonly", width=30,
                     values=("0：关闭脉冲输入", "1：开环脉冲输入", "2：闭环脉冲输入", "3：En/Dir 复用限位/到位")).pack(anchor="w")
        ttk.Button(group, text="应用脉冲输入模式", command=lambda: self.command("MODE_SET", self.mode.get()[0], self.save.get())).pack(anchor="w", pady=6)
        row = ttk.Frame(group); row.pack(fill="x", pady=6)
        for label, action, args in (("使能", "ENABLE", (1,)), ("失能", "ENABLE", (0,)), ("解除堵转保护", "CLEAR_STALL", ()), ("读取状态", "STATE_GET", ())):
            ttk.Button(row, text=label, command=lambda a=action, v=args: self.command(a, *v)).pack(side="left", padx=(0, 4))
        ttk.Label(group, textvariable=self.feedback, wraplength=440).pack(anchor="w")
        ttk.Label(self, textvariable=self.status, wraplength=440).pack(anchor="w", pady=12)
        ttk.Label(self, wraplength=440, text="需进入 Debug 模式并在动作空闲时操作。默认仅修改 RAM；勾选保存同时用于 PID 和脉冲输入模式。\n转盘运动直接使用驱动器当前 PID，不再自动放大 Kd。\n模式设置收到驱动器 ACK 后显示成功；PID 显示实际回读值。原始应答同时记录在日志中。").pack(anchor="w")
        self.trial_tabs = ttk.Notebook(self)
        self.trial_tabs.pack(fill="both", expand=True, pady=12)
        for axis, name, attr in ((5, "升降", "lift_tuning"), (6, "伸缩", "extension_tuning"), (7, "转盘", "tuning")):
            page = ttk.Frame(self.trial_tabs)
            self.trial_tabs.add(page, text=f"{axis}号{name}测试")
            panel = TurretTuningPanel(page, send, log, self.gains, self.motor, axis, self.can_start_trial)
            setattr(self, attr, panel)
            panel.pack(fill="both", expand=True)

    def tuning_panels(self):
        return tuple(p for p in (self.tuning, self.lift_tuning, self.extension_tuning) if p is not None)

    def can_start_trial(self):
        return not any(panel.active for panel in self.tuning_panels())

    def disconnected(self, reason="连接或模式改变，试验序列已取消"):
        for panel in self.tuning_panels():
            panel.disconnected(reason)

    def changed_motor(self, *_):
        for panel in self.tuning_panels():
            if panel.active:
                panel.stop_trial()
        for variable in self.gains:
            variable.set("")
        for variable in self.drive_values:
            variable.set("")
        self.drive_status.set("尚未读取该电机驱动配置")
        self.feedback.set("尚无该电机状态")
        self.status.set("请选择读取 PID，或输入新的参数。")

    def command(self, action, *values):
        try:
            line = build_emm_command(action, self.motor.get(), *values)
            self.send(line)
        except (ValueError, RuntimeError, OSError) as exc:
            messagebox.showerror("EMM 命令未发送", str(exc), parent=self)
            return
        self.log("TX", line)
        self.status.set(f"电机 {self.motor.get()}：{action} 已发送，等待应答。")
        if action in ("DRIVE_GET", "DRIVE_SET"):
            self.drive_status.set("等待驱动器实际回读……")

    def receive(self, text):
        fields = self.parse(text)
        if fields:
            for panel in self.tuning_panels():
                panel.receive(fields)
        if not fields or fields[:2] != ("RSP", "EMM"):
            return
        # 试验应答由调参模块处理；其ACK不带电机地址字段。
        if fields[2] in ("TURRET_TRIAL", "AXIS_TRIAL"):
            return
        if fields[3] == "ERR":
            self.status.set(f"操作失败：{fields[4]}（MODE 需 Debug，BUSY 表示动作执行中）")
            if fields[2] in ("DRIVE_GET", "DRIVE_SET"):
                self.drive_status.set(f"驱动参数操作失败：{fields[4]}")
            return
        if fields[4] != self.motor.get():
            return
        if fields[2] in ("DRIVE_GET", "DRIVE_SET") and len(fields) == 12:
            values = list(fields[5:])
            values[6] = f"{int(values[6])/10:.1f}"
            for variable, value in zip(self.drive_values, values):
                variable.set(value)
            note = "与请求不一致，已显示实际值" if fields[3] == "READBACK_MISMATCH" else "回读完成"
            self.drive_status.set(f"电机 {fields[4]} {note}：" + " / ".join(values))
            self.status.set(f"驱动参数：{note}")
        elif fields[2] in ("PID_GET", "PID_SET") and len(fields) == 8:
            for variable, value in zip(self.gains, fields[5:]):
                variable.set(value)
            self.status.set(f"电机 {fields[4]} PID 实际回读：" + " / ".join(fields[5:]))
        elif fields[2] == "STATE_GET" and len(fields) == 9:
            flags = int(fields[8])
            self.feedback.set(f"单圈编码器 {fields[5]}；累计位置 {fields[6]}（65536/圈）；转速 {fields[7]} RPM\n使能 {bool(flags & 1)}；到位 {bool(flags & 2)}；堵转 {bool(flags & 4)}；堵转保护 {bool(flags & 8)}")
            self.status.set("状态读取完成")
        elif fields[3] == "ACK":
            self.status.set(f"电机 {fields[4]}：{fields[2]} 驱动器已确认")
