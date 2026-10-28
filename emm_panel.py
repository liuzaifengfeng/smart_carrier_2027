"""EMM V5 驱动器参数调试页。"""
import tkinter as tk
from tkinter import ttk, messagebox


def build_emm_command(action, motor, *values):
    limits = {"PID_GET": (), "PID_SET": (0xFFFFFFFF,) * 3 + (1,),
              "STATE_GET": (), "MODE_SET": (3, 1), "ENABLE": (1,), "CLEAR_STALL": ()}
    numbers = [int(str(v).strip()) for v in (motor, *values)]
    if action not in limits or len(values) != len(limits[action]):
        raise ValueError("EMM 命令参数数量错误")
    if not 1 <= numbers[0] <= 7:
        raise ValueError("电机地址为 1～7")
    if any(not 0 <= v <= maximum for v, maximum in zip(numbers[1:], limits[action])):
        raise ValueError("PID 为 uint32 整数，保存/使能为 0 或 1，模式为 0～3")
    return "{CMD,EMM," + action + "," + ",".join(map(str, numbers)) + "}"


class EmmPanel(ttk.Frame):
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
        ttk.Label(self, wraplength=440, text="需进入 Debug 模式并在动作空闲时操作。默认仅修改 RAM；勾选保存同时用于 PID 和脉冲输入模式。\n7 号转盘启动时会按固件保存的基准重新配置 PID；此处保存不会更新该基准。\n模式设置收到驱动器 ACK 后显示成功；PID 显示实际回读值。原始应答同时记录在日志中。").pack(anchor="w")

    def changed_motor(self, *_):
        for variable in self.gains:
            variable.set("")
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

    def receive(self, text):
        fields = self.parse(text)
        if not fields or fields[:2] != ("RSP", "EMM"):
            return
        if fields[3] == "ERR":
            self.status.set(f"操作失败：{fields[4]}（MODE 需 Debug，BUSY 表示动作执行中）")
            return
        if fields[4] != self.motor.get():
            return
        if fields[2] in ("PID_GET", "PID_SET") and len(fields) == 8:
            for variable, value in zip(self.gains, fields[5:]):
                variable.set(value)
            self.status.set(f"电机 {fields[4]} PID 实际回读：" + " / ".join(fields[5:]))
        elif fields[2] == "STATE_GET" and len(fields) == 9:
            flags = int(fields[8])
            self.feedback.set(f"单圈编码器 {fields[5]}；累计位置 {fields[6]}（65536/圈）；转速 {fields[7]} RPM\n使能 {bool(flags & 1)}；到位 {bool(flags & 2)}；堵转 {bool(flags & 4)}；堵转保护 {bool(flags & 8)}")
            self.status.set("状态读取完成")
        elif fields[3] == "ACK":
            self.status.set(f"电机 {fields[4]}：{fields[2]} 驱动器已确认")
