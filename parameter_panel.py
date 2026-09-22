"""运行参数页：固件提供参数目录，按确认帧逐项发送，最后统一提交到 RAM。"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import secrets
import tkinter as tk
from tkinter import ttk, messagebox


@dataclass
class Parameter:
    index: int
    name: str
    value: str
    minimum: float
    maximum: float
    integer: bool

    def validate(self, text: str) -> str:
        try:
            value = float(text)
        except ValueError as exc:
            raise ValueError("请输入数字") from exc
        if not math.isfinite(value) or not self.minimum <= value <= self.maximum:
            raise ValueError(f"{self.name}：范围为 {self.minimum:g}～{self.maximum:g}")
        if self.integer and not value.is_integer():
            raise ValueError(f"{self.name}：必须为整数")
        return str(int(value)) if self.integer else format(value, ".9g")


def parse_parameter(fields: list[str]) -> Parameter:
    if len(fields) != 8 or fields[0] != "VALUE":
        raise ValueError("参数帧格式错误")
    index = int(fields[2])
    minimum, maximum = float(fields[5]), float(fields[6])
    if (index < 0 or not fields[3] or fields[7] not in ("0", "1")
            or not math.isfinite(minimum) or not math.isfinite(maximum)
            or minimum > maximum):
        raise ValueError("参数描述错误")
    # 允许显示固件中的 NaN（尚未标定）；发送时只允许有限数值。
    float(fields[4])
    return Parameter(index, fields[3], fields[4], minimum, maximum, fields[7] == "1")


class ParameterPanel(ttk.Frame):
    def __init__(self, parent, send_line, log):
        super().__init__(parent, padding=8)
        self.send_line = send_line
        self.log = log
        self.parameters: dict[int, Parameter] = {}
        self.edits: dict[int, str] = {}
        self.received: dict[int, Parameter] = {}
        self.sequence = secrets.randbelow(1000000000) + 1
        self.mode = ""
        self.ready = False
        self.commands = deque()
        self.expected = ""
        self.timer = None
        self.selected = None
        self.verifying = False
        ttk.Label(self, text="运行参数", font=("Microsoft YaHei UI", 14, "bold")).pack(anchor="w")
        ttk.Label(self, text="发送后保存在小车内存，重启恢复默认值。\n读写前停止视觉对齐；写入需调试模式。", justify=tk.LEFT).pack(anchor="w", pady=5)
        toolbar = ttk.Frame(self)
        toolbar.pack(fill=tk.X)
        self.read_button = ttk.Button(toolbar, text="一键读取", command=self.read)
        self.read_button.pack(side=tk.LEFT)
        self.send_button = ttk.Button(toolbar, text="发送修改", command=self.send)
        self.send_button.pack(side=tk.LEFT, padx=5)
        self.revert_button = ttk.Button(toolbar, text="撤销编辑", command=self.revert)
        self.revert_button.pack(side=tk.LEFT)
        body = ttk.Frame(self)
        body.pack(fill=tk.BOTH, expand=True, pady=8)
        body.rowconfigure(0, weight=1)
        body.columnconfigure(0, weight=1)
        self.tree = ttk.Treeview(body, columns=("name", "value", "edit"), show="headings", height=14, selectmode="browse")
        for key, title, width in (("name", "参数（含单位）", 235), ("value", "当前值", 80), ("edit", "待发送", 80)):
            self.tree.heading(key, text=title)
            self.tree.column(key, width=width, minwidth=60, stretch=(key == "name"))
        self.tree.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(body, orient=tk.VERTICAL, command=self.tree.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        horizontal = ttk.Scrollbar(body, orient=tk.HORIZONTAL, command=self.tree.xview)
        horizontal.grid(row=1, column=0, sticky="ew")
        self.tree.configure(yscrollcommand=scroll.set, xscrollcommand=horizontal.set)
        self.tree.tag_configure("dirty", foreground="#b05b00")
        self.tree.bind("<<TreeviewSelect>>", self.select)
        self.tree.bind("<Double-1>", lambda _event: self.entry.focus_set())
        self.label = tk.StringVar(value="选择参数后在下方修改；数组编号从 1 开始。")
        ttk.Label(self, textvariable=self.label, wraplength=430).pack(anchor="w")
        editor = ttk.Frame(self)
        editor.pack(fill=tk.X, pady=6)
        self.value = tk.StringVar()
        self.entry = ttk.Entry(editor, textvariable=self.value, width=20)
        self.entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.entry.bind("<Return>", lambda _event: self.edit())
        self.edit_button = ttk.Button(editor, text="修改选中项", command=self.edit)
        self.edit_button.pack(side=tk.LEFT, padx=5)
        self.status = tk.StringVar(value="连接串口后点击“一键读取”。")
        ttk.Label(self, textvariable=self.status, wraplength=430).pack(anchor="w", pady=5)
        self.update_controls()

    def update_controls(self):
        idle = not self.mode
        self.read_button.configure(state=tk.NORMAL if idle else tk.DISABLED)
        for widget in (self.send_button, self.edit_button, self.revert_button, self.entry):
            widget.configure(state=tk.NORMAL if idle and self.ready else tk.DISABLED)

    def cancel_timer(self):
        if self.timer is not None:
            self.after_cancel(self.timer)
            self.timer = None

    def fail(self, message):
        self.cancel_timer()
        self.mode = ""
        self.ready = False
        self.commands.clear()
        self.status.set(message + "；请重新读取确认小车当前值。")
        self.update_controls()

    def disconnected(self):
        self.fail("连接已变化，旧参数不能继续发送")

    def transmit(self, line):
        try:
            self.send_line(line)
        except (RuntimeError, OSError) as exc:
            self.fail(str(exc))
            return False
        self.log("TX", line)
        self.cancel_timer()
        self.timer = self.after(8000, lambda: self.fail("等待参数回复超时（固件需支持 CFG 协议）"))
        return True

    def read(self, verify=False):
        if self.mode:
            return
        if not verify and self.edits:
            if not messagebox.askyesno("重新读取", "重新读取会丢弃未发送的编辑，是否继续？", parent=self):
                return
        self.sequence += 1
        self.mode = "read"
        self.ready = False
        self.verifying = verify
        self.received = {}
        self.status.set("正在回读生效值…" if verify else "正在读取小车参数…")
        self.update_controls()
        self.transmit(f"{{CFG:GET,{self.sequence}}}")

    def select(self, _event=None):
        selection = self.tree.selection()
        if not selection:
            return
        self.selected = int(selection[0])
        p = self.parameters[self.selected]
        self.value.set(self.edits.get(p.index, p.value))
        kind = "整数" if p.integer else "数值"
        self.label.set(f"{p.name}\n允许范围：{p.minimum:g}～{p.maximum:g}（{kind}）")

    def edit(self):
        if not self.ready or self.mode or self.selected is None:
            return True
        p = self.parameters[self.selected]
        # 未标定值原样显示时不作为一次编辑。
        if self.value.get().strip() == p.value:
            self.edits.pop(p.index, None)
        else:
            try:
                value = p.validate(self.value.get())
            except ValueError as exc:
                messagebox.showerror("参数无效", str(exc), parent=self)
                return False
            if float(value) == float(p.value):
                self.edits.pop(p.index, None)
            else:
                self.edits[p.index] = value
        self.tree.item(str(p.index), values=(p.name, p.value, self.edits.get(p.index, "")), tags=("dirty",) if p.index in self.edits else ())
        self.status.set(f"已修改 {len(self.edits)} 项，点击“发送修改”后生效。")
        return True

    def revert(self):
        self.edits.clear()
        for p in self.parameters.values():
            self.tree.item(str(p.index), values=(p.name, p.value, ""), tags=())
        self.select()
        self.status.set("已撤销本地编辑。")

    def send(self):
        if self.mode or not self.ready or not self.edit():
            return
        if not self.edits:
            self.status.set("没有需要发送的修改。")
            return
        self.sequence += 1
        self.mode = "write"
        self.commands = deque([("BEGIN", "BEGIN")])
        self.commands.extend((f"SET,{i},{v}", f"SET,{i}") for i, v in self.edits.items())
        self.commands.append(("COMMIT", "COMMIT"))
        self.status.set(f"正在发送 {len(self.edits)} 项修改…")
        self.update_controls()
        self.send_next()

    def send_next(self):
        command, self.expected = self.commands.popleft()
        operation, *args = command.split(",")
        suffix = "," + ",".join(args) if args else ""
        self.transmit(f"{{CFG:{operation},{self.sequence}{suffix}}}")

    def receive(self, text):
        if not self.mode or not text.startswith("{CFG:") or not text.endswith("}"):
            return
        fields = text[5:-1].split(",")
        try:
            if len(fields) < 2 or int(fields[1]) != self.sequence:
                return  # 忽略旧连接、旧请求的迟到回复。
            if fields[0] == "ERR":
                reason = {"BUSY_OR_MODE": "请切换到调试模式并停止视觉对齐", "RANGE": "固件拒绝了越界参数", "TRANSACTION": "参数事务已失效"}.get(fields[2], fields[2])
                self.fail("发送失败：" + reason)
            elif self.mode == "read" and fields[0] == "VALUE":
                p = parse_parameter(fields)
                if p.index in self.received:
                    raise ValueError("重复参数")
                self.received[p.index] = p
            elif self.mode == "read" and fields[0] == "END":
                count = int(fields[2])
                if len(fields) != 3 or count <= 0 or set(self.received) != set(range(count)):
                    raise ValueError("读取不完整")
                if self.verifying:
                    for index, expected in self.edits.items():
                        if index not in self.received or not math.isclose(float(self.received[index].value), float(expected), rel_tol=1e-6, abs_tol=1e-8):
                            raise ValueError("回读值与发送值不一致")
                self.parameters = self.received
                self.edits.clear()
                self.tree.delete(*self.tree.get_children())
                self.selected = None
                self.value.set("")
                for p in self.parameters.values():
                    self.tree.insert("", tk.END, iid=str(p.index), values=(p.name, p.value, ""))
                self.cancel_timer()
                self.mode = ""
                self.ready = True
                self.status.set(f"修改已生效并回读确认，共 {count} 项。重启恢复默认值。" if self.verifying else f"已读取 {count} 项参数，选择一项开始编辑。")
                self.update_controls()
            elif self.mode == "write" and fields[0] == "OK" and ",".join(fields[2:]) == self.expected:
                self.cancel_timer()
                if self.commands:
                    self.send_next()
                else:
                    self.mode = ""
                    self.read(verify=True)
        except (ValueError, IndexError) as exc:
            self.fail("参数回复异常：" + str(exc))
