"""智能搬运车调试上位机：有线串口、场地地图与机械臂姿态预览。

坐标系采用比赛场地图示：右下角为原点，X 轴向上，Y 轴向左，单位 mm。
角度暂定为 0 度朝 X 正方向，正角由 X 正方向转向 Y 正方向。
"""

from __future__ import annotations

import math
import queue
import sys
import threading
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, scrolledtext, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


FIELD_SIZE_MM = 2400.0
ROBOT_SIZE_MM = 300.0
ROBOT_HALF_MM = ROBOT_SIZE_MM / 2.0
MAX_RELATIVE_MOVE_MM = FIELD_SIZE_MM * math.sqrt(2.0)
FIELD_EDGE_EPSILON_MM = 1e-6


@dataclass(frozen=True)
class Pose:
    """场地坐标系中的机器人中心位姿。"""

    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class ArmPose:
    """机械臂四个可控轴的估计姿态。"""

    high: float
    length: float
    turret_angle: float
    pawl_angle: float


START_POSES = {
    "启停区1": Pose(2250.0, 150.0, 180.0),
    "启停区2": Pose(150.0, 150.0, 0.0),
}


def next_start_zone(current: str) -> str:
    """返回另一个启停区名称。"""
    if current == "启停区1":
        return "启停区2"
    return "启停区1"


def merge_arm_pose(current: ArmPose, requested: ArmPose) -> ArmPose:
    """按固件约定合并机械臂命令，-1 表示保留当前轴。"""
    return ArmPose(
        high=current.high if requested.high == -1.0 else requested.high,
        length=current.length if requested.length == -1.0 else requested.length,
        turret_angle=(
            current.turret_angle
            if requested.turret_angle == -1.0
            else requested.turret_angle
        ),
        pawl_angle=(
            current.pawl_angle if requested.pawl_angle == -1.0 else requested.pawl_angle
        ),
    )


def parse_arm_request(raw_values: list[str]) -> ArmPose:
    """解析四个机械臂轴输入；-1 沿用固件的“不操作该轴”语义。"""
    if len(raw_values) != 4:
        raise ValueError("机械臂姿态参数数量错误")
    try:
        values = [float(value.strip()) for value in raw_values]
    except ValueError as exc:
        raise ValueError("机械臂姿态参数必须填写数字") from exc
    if not all(math.isfinite(value) for value in values):
        raise ValueError("机械臂姿态参数不能包含无穷大或 NaN")

    limits = (
        (0.0, 200.0, "高度"),
        (0.0, 170.0, "伸出长度"),
        (-360.0, 360.0, "转台角度"),
        (-360.0, 360.0, "夹爪角度"),
    )
    for value, (minimum, maximum, label) in zip(values, limits):
        if value != -1.0 and not minimum <= value <= maximum:
            raise ValueError(f"{label}必须在 {minimum:g}~{maximum:g} 之间，或填 -1")
    return ArmPose(*values)


def world_to_normalized(x: float, y: float) -> tuple[float, float]:
    """将场地坐标转换为左上原点的 0~1 画布坐标。"""
    return 1.0 - y / FIELD_SIZE_MM, 1.0 - x / FIELD_SIZE_MM


def robot_corners(pose: Pose) -> list[tuple[float, float]]:
    """计算旋转后 300×300 mm 车体的四个场地坐标顶点。"""
    angle = math.radians(pose.theta)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    corners: list[tuple[float, float]] = []
    for forward, lateral in (
        (ROBOT_HALF_MM, ROBOT_HALF_MM),
        (ROBOT_HALF_MM, -ROBOT_HALF_MM),
        (-ROBOT_HALF_MM, -ROBOT_HALF_MM),
        (-ROBOT_HALF_MM, ROBOT_HALF_MM),
    ):
        x = pose.x + forward * cos_a - lateral * sin_a
        y = pose.y + forward * sin_a + lateral * cos_a
        corners.append((x, y))
    return corners


def pose_fits_field(pose: Pose) -> bool:
    """仅检查完整车体是否仍位于 2400×2400 mm 场地内。"""
    return all(
        -FIELD_EDGE_EPSILON_MM <= x <= FIELD_SIZE_MM + FIELD_EDGE_EPSILON_MM
        and -FIELD_EDGE_EPSILON_MM <= y <= FIELD_SIZE_MM + FIELD_EDGE_EPSILON_MM
        for x, y in robot_corners(pose)
    )


def shortest_angle_delta(current_deg: float, target_deg: float) -> float:
    """返回从当前角度转到目标角度的最短有符号角度。"""
    return (target_deg - current_deg + 180.0) % 360.0 - 180.0


def target_to_relative_move(current: Pose, target: Pose) -> Pose:
    """把场地绝对目标转换为 GotoPose 使用的车体相对位移。"""
    world_dx = target.x - current.x
    world_dy = target.y - current.y
    angle = math.radians(current.theta)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return Pose(
        x=world_dx * cos_a + world_dy * sin_a,
        y=-world_dx * sin_a + world_dy * cos_a,
        theta=shortest_angle_delta(current.theta, target.theta),
    )


def parse_gotopose_echo(text: str) -> Pose | None:
    """解析固件 ``GOTOpose %.0f, %.0f, %.0f`` 格式的接令回显。"""
    prefix = "GOTOpose "
    if not text.startswith(prefix):
        return None
    parts = text[len(prefix):].replace(",", " ").split()
    if len(parts) != 3:
        return None
    try:
        values = tuple(float(part) for part in parts)
    except ValueError:
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    return Pose(*values)


def gotopose_echo_matches(text: str, expected: Pose) -> bool:
    """考虑固件只回显整数，判断回显是否对应待确认命令。"""
    echoed = parse_gotopose_echo(text)
    if echoed is None:
        return False
    return all(
        abs(actual - wanted) <= 0.500001
        for actual, wanted in zip(
            (echoed.x, echoed.y, echoed.theta),
            (expected.x, expected.y, expected.theta),
        )
    )


COMMAND_FIELDS = {
    "GOTOpose": (("X 增量", "0"), ("Y 增量", "0"), ("θ 增量", "0")),
    "Movepose": (("前进 0/1", "1"), ("速度", "80"), ("停止 0/1", "0")),
    "En_C": (("使能 0/1", "1"),),
    "MoveArm_1": (("高度", "-1"), ("伸出长度", "-1"), ("速度", "80")),
    "MoveArm_2": (("转台角度", "-1"), ("夹爪角度", "-1"), ("速度", "80")),
    "SERVO": (("舵机 ID", "1"), ("角度", "0")),
}


def build_debug_command(command: str, raw_values: list[str]) -> str:
    """校验界面参数并生成 ESP32 当前支持的调试命令。"""
    if command == "help":
        return "help"
    fields = COMMAND_FIELDS.get(command)
    if fields is None or len(raw_values) != len(fields):
        raise ValueError("未知命令或参数数量错误")
    try:
        values = [float(value.strip()) for value in raw_values]
    except ValueError as exc:
        raise ValueError("命令参数必须填写数字") from exc
    if not all(math.isfinite(value) for value in values):
        raise ValueError("命令参数不能包含无穷大或 NaN")

    def require_range(index: int, minimum: float, maximum: float, label: str) -> None:
        if not minimum <= values[index] <= maximum:
            raise ValueError(f"{label}必须在 {minimum:g}~{maximum:g} 之间")

    def require_binary(index: int, label: str) -> None:
        if values[index] not in (0.0, 1.0):
            raise ValueError(f"{label}只能填写 0 或 1")

    def require_range_or_skip(index: int, minimum: float, maximum: float, label: str) -> None:
        if values[index] != -1.0:
            require_range(index, minimum, maximum, f"{label}（或填 -1 跳过）")

    integer_indexes: set[int] = set()
    if command == "GOTOpose":
        require_range(0, -MAX_RELATIVE_MOVE_MM, MAX_RELATIVE_MOVE_MM, "X 增量")
        require_range(1, -MAX_RELATIVE_MOVE_MM, MAX_RELATIVE_MOVE_MM, "Y 增量")
        require_range(2, -360.0, 360.0, "θ 增量")
    elif command == "Movepose":
        require_binary(0, "前进参数")
        require_range(1, 0.0, 1000.0, "速度")
        require_binary(2, "停止参数")
        integer_indexes = {0, 2}
    elif command == "En_C":
        require_binary(0, "使能参数")
        integer_indexes = {0}
    elif command == "MoveArm_1":
        require_range_or_skip(0, 0.0, 200.0, "高度")
        require_range_or_skip(1, 0.0, 170.0, "伸出长度")
        require_range(2, 1.0, 1000.0, "速度")
    elif command == "MoveArm_2":
        require_range_or_skip(0, -360.0, 360.0, "转台角度")
        require_range_or_skip(1, -360.0, 360.0, "夹爪角度")
        require_range(2, 1.0, 1000.0, "速度")
    elif command == "SERVO":
        require_range(0, 0.0, 254.0, "舵机 ID")
        require_range(1, -135.0, 135.0, "舵机角度")
        if not values[0].is_integer():
            raise ValueError("舵机 ID 必须是整数")
        integer_indexes = {0}

    tokens = [str(int(value)) if index in integer_indexes else f"{value:g}"
              for index, value in enumerate(values)]
    return " ".join((command, *tokens))


class SerialLink:
    """通过后台线程读取 pyserial，所有界面更新交给 Tk 主线程。"""

    def __init__(self, events: queue.Queue[tuple[str, str]]) -> None:
        self.events = events
        self._port = None
        self._reader: threading.Thread | None = None
        self._stop = threading.Event()

    @property
    def is_open(self) -> bool:
        return bool(self._port is not None and self._port.is_open)

    @staticmethod
    def available_ports() -> list[str]:
        if list_ports is None:
            return []
        return [info.device for info in list_ports.comports()]

    def connect(self, port_name: str, baudrate: int = 115200) -> None:
        if serial is None:
            raise RuntimeError("未安装 pyserial，请运行：python -m pip install pyserial")
        self.disconnect()
        port = serial.Serial()
        port.port = port_name
        port.baudrate = baudrate
        port.bytesize = serial.EIGHTBITS
        port.parity = serial.PARITY_NONE
        port.stopbits = serial.STOPBITS_ONE
        port.timeout = 0.1
        port.write_timeout = 0.5
        # 尽量避免打开串口时由 DTR/RTS 触发开发板复位。
        port.dtr = False
        port.rts = False
        port.open()
        self._port = port
        self._stop.clear()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def disconnect(self) -> None:
        self._stop.set()
        port = self._port
        self._port = None
        if port is not None:
            try:
                port.close()
            except Exception:
                pass
        reader = self._reader
        self._reader = None
        if reader is not None and reader is not threading.current_thread():
            reader.join(timeout=0.4)

    def send_line(self, command: str) -> None:
        if not self.is_open:
            raise RuntimeError("串口尚未连接")
        assert self._port is not None
        self._port.write((command + "\n").encode("utf-8"))

    def _read_loop(self) -> None:
        try:
            while not self._stop.is_set() and self._port is not None:
                data = self._port.readline()
                if data:
                    text = data.decode("utf-8", errors="replace").rstrip("\r\n")
                    self.events.put(("RX", text))
        except Exception as exc:
            if not self._stop.is_set():
                self.events.put(("ERROR", f"串口读取失败：{exc}"))
        finally:
            self._stop.set()


class FieldCanvas(tk.Canvas):
    """按窗口大小等比例绘制场地和机器人。"""

    def __init__(self, master: tk.Misc) -> None:
        super().__init__(master, background="#eef1f4", highlightthickness=0)
        self.current_pose = START_POSES["启停区2"]
        self.target_pose = Pose(600.0, 600.0, 0.0)
        self.target_visible = False
        self.scale = 1.0
        self.field_left = 0.0
        self.field_top = 0.0
        self.bind("<Configure>", self._on_resize)

    def _on_resize(self, _event: tk.Event) -> None:
        self.redraw()

    def set_current_pose(self, pose: Pose) -> None:
        self.current_pose = pose
        self.redraw()

    def set_start_pose(self, pose: Pose) -> None:
        """切换启停区时重置当前估计，并清除旧目标轮廓。"""
        self.current_pose = pose
        self.target_pose = pose
        self.target_visible = False
        self.redraw()

    def set_target_pose(self, pose: Pose) -> None:
        self.target_pose = pose
        self.target_visible = True
        self.redraw()

    def commit_target_as_estimate(self, pose: Pose) -> None:
        """把已接令的目标记为新的开环估计位置。"""
        self.current_pose = pose
        self.target_pose = pose
        self.target_visible = False
        self.redraw()

    def world_to_canvas(self, x: float, y: float) -> tuple[float, float]:
        u, v = world_to_normalized(x, y)
        return (
            self.field_left + u * FIELD_SIZE_MM * self.scale,
            self.field_top + v * FIELD_SIZE_MM * self.scale,
        )

    def _world_rect(self, x_min: float, x_max: float, y_min: float,
                    y_max: float) -> tuple[float, float, float, float]:
        points = (
            self.world_to_canvas(x_min, y_min),
            self.world_to_canvas(x_max, y_max),
        )
        xs = (points[0][0], points[1][0])
        ys = (points[0][1], points[1][1])
        return min(xs), min(ys), max(xs), max(ys)

    def _draw_zone(self, x_min: float, x_max: float, y_min: float, y_max: float,
                   *, fill: str, outline: str = "#707780", width: int = 1) -> None:
        self.create_rectangle(
            *self._world_rect(x_min, x_max, y_min, y_max),
            fill=fill,
            outline=outline,
            width=width,
        )

    def _draw_ring(self, x: float, y: float, radius: float = 38.0) -> None:
        left, top = self.world_to_canvas(x + radius, y + radius)
        right, bottom = self.world_to_canvas(x - radius, y - radius)
        self.create_oval(left, top, right, bottom, fill="#ffffff", width=2)
        inner = radius * 0.42
        left, top = self.world_to_canvas(x + inner, y + inner)
        right, bottom = self.world_to_canvas(x - inner, y - inner)
        self.create_oval(left, top, right, bottom, outline="#60666d")

    def _draw_robot(self, pose: Pose, *, target: bool) -> None:
        canvas_points: list[float] = []
        for x, y in robot_corners(pose):
            px, py = self.world_to_canvas(x, y)
            canvas_points.extend((px, py))

        if target:
            self.create_polygon(
                *canvas_points,
                fill="",
                outline="#e67e22",
                width=3,
                dash=(7, 4),
            )
            color = "#e67e22"
            label = "目标"
        else:
            self.create_polygon(
                *canvas_points,
                fill="#4f9dd9",
                outline="#145a86",
                width=3,
            )
            color = "#0b3c5d"
            label = "估计位置"

        angle = math.radians(pose.theta)
        arrow_x = pose.x + ROBOT_SIZE_MM * 0.68 * math.cos(angle)
        arrow_y = pose.y + ROBOT_SIZE_MM * 0.68 * math.sin(angle)
        center_px = self.world_to_canvas(pose.x, pose.y)
        arrow_px = self.world_to_canvas(arrow_x, arrow_y)
        self.create_line(
            *center_px,
            *arrow_px,
            fill=color,
            width=4,
            arrow=tk.LAST,
            arrowshape=(12, 14, 5),
        )
        self.create_text(
            center_px[0],
            center_px[1],
            text=label,
            fill="#ffffff" if not target else color,
            font=("Microsoft YaHei UI", 9, "bold"),
        )

    def _draw_rulers(self, field_right: float, field_bottom: float) -> None:
        """按右下角原点绘制 X/Y 毫米刻度。"""
        for value in range(0, int(FIELD_SIZE_MM) + 1, 300):
            major = value % 600 == 0
            tick_length = 8 if major else 5

            tick_x, _ = self.world_to_canvas(0.0, float(value))
            self.create_line(
                tick_x,
                field_bottom,
                tick_x,
                field_bottom + tick_length,
                fill="#384047",
            )
            if major:
                self.create_text(
                    tick_x,
                    field_bottom + 17,
                    text=str(value),
                    fill="#343a40",
                    font=("Microsoft YaHei UI", 8),
                )

            _, tick_y = self.world_to_canvas(float(value), 0.0)
            self.create_line(
                field_right,
                tick_y,
                field_right + tick_length,
                tick_y,
                fill="#384047",
            )
            if major:
                self.create_text(
                    field_right + 12,
                    tick_y,
                    text=str(value),
                    anchor="w",
                    fill="#343a40",
                    font=("Microsoft YaHei UI", 8),
                )

        field_bottom_center = (self.field_left + field_right) / 2.0
        field_right_center = (self.field_top + field_bottom) / 2.0
        self.create_text(
            field_bottom_center,
            field_bottom + 40,
            text="Y 坐标 / mm（向左增大）",
            fill="#343a40",
            font=("Microsoft YaHei UI", 9),
        )
        self.create_text(
            field_right + 54,
            field_right_center,
            text="X 坐标 / mm（向上增大）",
            angle=90,
            fill="#343a40",
            font=("Microsoft YaHei UI", 9),
        )
        self.create_text(
            self.field_left,
            self.field_top - 28,
            text="场地总尺寸：2400 × 2400 mm    小车：300 × 300 mm",
            anchor="w",
            fill="#343a40",
            font=("Microsoft YaHei UI", 9, "bold"),
        )

    def redraw(self) -> None:
        width = max(self.winfo_width(), 300)
        height = max(self.winfo_height(), 300)
        margin = 72.0
        self.scale = max(min((width - 2 * margin) / FIELD_SIZE_MM,
                             (height - 2 * margin) / FIELD_SIZE_MM), 0.05)
        side = FIELD_SIZE_MM * self.scale
        self.field_left = (width - side) / 2.0
        self.field_top = (height - side) / 2.0
        field_right = self.field_left + side
        field_bottom = self.field_top + side

        self.delete("all")
        self.create_rectangle(
            self.field_left,
            self.field_top,
            field_right,
            field_bottom,
            fill="#d8dadd",
            outline="#384047",
            width=2,
        )

        # 中央四块淡黄色区域；中间保留 400 mm 十字通道。
        for x_min, x_max in ((550.0, 1000.0), (1400.0, 1850.0)):
            for y_min, y_max in ((550.0, 1000.0), (1400.0, 1850.0)):
                self._draw_zone(x_min, x_max, y_min, y_max, fill="#fffde2", outline="")

        center_start = self.world_to_canvas(0.0, 1200.0)
        center_end = self.world_to_canvas(2400.0, 1200.0)
        self.create_line(*center_start, *center_end, fill="#747b82", dash=(8, 8))
        center_start = self.world_to_canvas(1200.0, 0.0)
        center_end = self.world_to_canvas(1200.0, 2400.0)
        self.create_line(*center_start, *center_end, fill="#747b82", dash=(8, 8))

        self._draw_zone(2100.0, 2400.0, 0.0, 300.0, fill="#1859d1", outline="#1859d1")
        self._draw_zone(0.0, 300.0, 0.0, 300.0, fill="#1859d1", outline="#1859d1")
        self._draw_zone(910.0, 1490.0, 2250.0, 2400.0, fill="#cfd2d5")
        self._draw_zone(0.0, 150.0, 910.0, 1490.0, fill="#cfd2d5")

        for x in (1010.0, 1200.0, 1390.0):
            self._draw_ring(x, 2325.0)
        for y in (1010.0, 1200.0, 1390.0):
            self._draw_ring(75.0, y)

        raw_center = self.world_to_canvas(2400.0, 1200.0)
        raw_radius = 150.0 * self.scale
        self.create_oval(
            raw_center[0] - raw_radius,
            raw_center[1] - raw_radius,
            raw_center[0] + raw_radius,
            raw_center[1] + raw_radius,
            fill="#f4f4f4",
            outline="#60666d",
            width=2,
        )

        qr_top = self.world_to_canvas(1300.0, 0.0)
        qr_bottom = self.world_to_canvas(1100.0, 0.0)
        self.create_line(*qr_top, *qr_bottom, fill="#1f252a", width=6)

        labels = (
            (2250.0, 150.0, "启停区1"),
            (150.0, 150.0, "启停区2"),
            (2350.0, 1500.0, "原料区"),
            (1200.0, 2180.0, "暂存区"),
            (210.0, 1200.0, "粗加工区"),
            (1200.0, 80.0, "二维码板"),
        )
        for x, y, text in labels:
            px, py = self.world_to_canvas(x, y)
            self.create_text(px, py, text=text, font=("Microsoft YaHei UI", 10, "bold"))

        # 在右下角画出用户指定的坐标轴。
        axis_length = min(92.0, side * 0.16)
        self.create_line(field_right, field_bottom, field_right, field_bottom - axis_length,
                         fill="#d9342b", width=3, arrow=tk.LAST)
        self.create_line(field_right, field_bottom, field_right - axis_length, field_bottom,
                         fill="#d9342b", width=3, arrow=tk.LAST)
        self.create_text(field_right - 18, field_bottom - axis_length, text="+X", fill="#d9342b")
        self.create_text(field_right - axis_length, field_bottom - 14, text="+Y", fill="#d9342b")
        self._draw_rulers(field_right, field_bottom)

        if self.target_visible:
            self._draw_robot(self.target_pose, target=True)
        self._draw_robot(self.current_pose, target=False)


class ArmCanvas(tk.Canvas):
    """机械臂轻量示意图：侧视升降/伸出，俯视转台/夹爪。"""

    # 独立 Canvas 重绘结构参考 MIT 项目：
    # https://github.com/NuclearVenom/Robot-Arm-Simulator-2D
    def __init__(self, master: tk.Misc) -> None:
        super().__init__(
            master,
            height=270,
            background="#f7f8fa",
            highlightthickness=1,
            highlightbackground="#b8bec5",
        )
        self.pose = ArmPose(0.0, 0.0, 0.0, 0.0)
        self.bind("<Configure>", self._on_resize)

    def _on_resize(self, _event: tk.Event) -> None:
        self.redraw()

    def set_pose(self, pose: ArmPose) -> None:
        self.pose = pose
        self.redraw()

    @staticmethod
    def _ratio(value: float, maximum: float) -> float:
        return max(0.0, min(value / maximum, 1.0))

    def _draw_side_view(self, width: float, split_y: float) -> None:
        mast_x = width * 0.24
        mast_top = 31.0
        mast_bottom = split_y - 18.0
        carriage_y = mast_bottom - self._ratio(self.pose.high, 200.0) * (
            mast_bottom - mast_top
        )
        boom_start = mast_x + 14.0
        boom_length = 38.0 + self._ratio(self.pose.length, 170.0) * max(
            width - boom_start - 48.0, 30.0
        )
        boom_end = boom_start + boom_length

        self.create_text(
            10,
            9,
            text="侧视：升降 / 伸出",
            anchor="nw",
            fill="#263746",
            font=("Microsoft YaHei UI", 9, "bold"),
        )
        self.create_line(18, mast_bottom + 10, width - 18, mast_bottom + 10, fill="#7b858e")
        self.create_rectangle(
            mast_x - 10,
            mast_top,
            mast_x + 10,
            mast_bottom + 10,
            fill="#c7ccd1",
            outline="#56616b",
            width=2,
        )
        self.create_rectangle(
            mast_x - 15,
            carriage_y - 10,
            mast_x + 18,
            carriage_y + 10,
            fill="#4f9dd9",
            outline="#145a86",
            width=2,
        )
        self.create_rectangle(
            boom_start,
            carriage_y - 6,
            boom_end,
            carriage_y + 6,
            fill="#99a3ad",
            outline="#4c5862",
        )
        pawl_angle = math.radians(self.pose.pawl_angle)
        pawl_length = 18.0
        pawl_dx = pawl_length * math.cos(pawl_angle)
        pawl_dy = pawl_length * math.sin(pawl_angle)
        self.create_line(
            boom_end,
            carriage_y,
            boom_end + pawl_dx,
            carriage_y - pawl_dy,
            fill="#d3543c",
            width=5,
        )
        self.create_text(
            width - 10,
            9,
            text=f"H {self.pose.high:.0f} mm   L {self.pose.length:.0f} mm",
            anchor="ne",
            fill="#44515c",
            font=("Microsoft YaHei UI", 8),
        )

    def _draw_top_view(self, width: float, height: float, split_y: float) -> None:
        center_x = width / 2.0
        center_y = split_y + (height - split_y) * 0.56
        reach = 28.0 + self._ratio(self.pose.length, 170.0) * min(
            width * 0.22, (height - split_y) * 0.28
        )
        turret_angle = math.radians(self.pose.turret_angle)
        end_x = center_x + reach * math.cos(turret_angle)
        end_y = center_y - reach * math.sin(turret_angle)

        self.create_line(8, split_y, width - 8, split_y, fill="#c4c9ce", dash=(5, 4))
        self.create_text(
            10,
            split_y + 7,
            text="俯视：转台 / 夹爪",
            anchor="nw",
            fill="#263746",
            font=("Microsoft YaHei UI", 9, "bold"),
        )
        self.create_line(center_x - 58, center_y, center_x + 58, center_y,
                         fill="#d7dbe0", dash=(3, 4))
        self.create_line(center_x, center_y - 43, center_x, center_y + 43,
                         fill="#d7dbe0", dash=(3, 4))
        self.create_rectangle(
            center_x - 17,
            center_y - 17,
            center_x + 17,
            center_y + 17,
            fill="#c7ccd1",
            outline="#56616b",
            width=2,
        )
        self.create_line(
            center_x,
            center_y,
            end_x,
            end_y,
            fill="#4f9dd9",
            width=12,
        )
        gripper_angle = turret_angle + math.radians(self.pose.pawl_angle)
        jaw_dx = 14.0 * math.cos(gripper_angle)
        jaw_dy = 14.0 * math.sin(gripper_angle)
        self.create_line(
            end_x - jaw_dx,
            end_y + jaw_dy,
            end_x + jaw_dx,
            end_y - jaw_dy,
            fill="#d3543c",
            width=5,
        )
        self.create_text(
            width - 10,
            split_y + 7,
            text=f"转台 {self.pose.turret_angle:.0f}°   夹爪 {self.pose.pawl_angle:.0f}°",
            anchor="ne",
            fill="#44515c",
            font=("Microsoft YaHei UI", 8),
        )

    def redraw(self) -> None:
        width = float(max(self.winfo_width(), 280))
        height = float(max(self.winfo_height(), 250))
        split_y = height * 0.55
        self.delete("all")
        self._draw_side_view(width, split_y)
        self._draw_top_view(width, height, split_y)
        self.create_text(
            width - 9,
            height - 7,
            text="姿态示意 · 暂不等比例",
            anchor="se",
            fill="#7a838b",
            font=("Microsoft YaHei UI", 8),
        )


class UpperComputerApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        root.title("智能搬运车调试上位机 - 有线串口")
        root.geometry("1180x780")
        root.minsize(940, 650)
        self.serial_events: queue.Queue[tuple[str, str]] = queue.Queue()
        self.serial_link = SerialLink(self.serial_events)
        self.command_vars: dict[str, list[tk.StringVar]] = {}
        self.arm_estimate = ArmPose(0.0, 0.0, 0.0, 0.0)
        self.arm_vars: dict[str, tk.StringVar] = {}
        self.pending_target: Pose | None = None
        self.pending_relative_move: Pose | None = None

        container = ttk.Frame(root, padding=10)
        container.pack(fill=tk.BOTH, expand=True)
        container.columnconfigure(0, weight=1)
        container.columnconfigure(1, weight=0)
        container.rowconfigure(1, weight=1)

        connection = ttk.LabelFrame(container, text="USB 串口连接", padding=8)
        connection.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8))
        connection.columnconfigure(1, weight=1)
        ttk.Label(connection, text="端口：").grid(row=0, column=0, padx=(0, 4))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, width=18)
        self.port_combo.grid(row=0, column=1, sticky="w")
        ttk.Button(connection, text="刷新", command=self.refresh_ports).grid(
            row=0, column=2, padx=5
        )
        self.connect_button = ttk.Button(connection, text="连接", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=3, padx=5)
        self.connection_var = tk.StringVar(value="未连接 · 115200 8N1")
        ttk.Label(connection, textvariable=self.connection_var).grid(
            row=0, column=4, padx=(12, 0), sticky="e"
        )

        self.field = FieldCanvas(container)
        self.field.grid(row=1, column=0, sticky="nsew", padx=(0, 12))

        notebook = ttk.Notebook(container, width=330)
        notebook.grid(row=1, column=1, sticky="ns")
        pose_tab = ttk.Frame(notebook, padding=12)
        chassis_tab = ttk.Frame(notebook, padding=8)
        arm_tab = ttk.Frame(notebook, padding=8)
        log_tab = ttk.Frame(notebook, padding=8)
        notebook.add(pose_tab, text="姿态")
        notebook.add(chassis_tab, text="底盘")
        notebook.add(arm_tab, text="机械臂")
        notebook.add(log_tab, text="日志")

        ttk.Label(pose_tab, text="姿态调试", font=("Microsoft YaHei UI", 16, "bold")).pack(
            anchor="w", pady=(0, 6)
        )
        ttk.Label(
            pose_tab,
            text="原点：右下角\n+X：向上    +Y：向左\n单位：mm / °",
            justify=tk.LEFT,
        ).pack(anchor="w", pady=(0, 8))

        start_group = ttk.LabelFrame(pose_tab, text="启动位置", padding=8)
        start_group.pack(fill=tk.X, pady=(0, 10))
        self.start_zone_name = "启停区2"
        self.start_zone_var = tk.StringVar(value="当前选择：启停区2")
        ttk.Label(start_group, textvariable=self.start_zone_var).pack(anchor="w")
        self.start_zone_button = ttk.Button(
            start_group,
            text="切换到启停区1",
            command=self.switch_start_zone,
        )
        self.start_zone_button.pack(fill=tk.X, pady=(6, 0))

        self.current_vars = self._pose_editor(pose_tab, "当前估计姿态", (150.0, 150.0, 0.0))
        ttk.Button(pose_tab, text="更新估计位置", command=self.update_current).pack(
            fill=tk.X, pady=(5, 14)
        )

        self.target_vars = self._pose_editor(pose_tab, "目标姿态预览", (600.0, 600.0, 0.0))
        ttk.Button(pose_tab, text="在地图上预览目标", command=self.preview_target).pack(
            fill=tk.X, pady=(5, 5)
        )
        ttk.Button(
            pose_tab,
            text="发送目标姿态（GOTOpose）",
            command=self.move_to_target,
        ).pack(
            fill=tk.X, pady=(0, 14)
        )

        ttk.Separator(pose_tab).pack(fill=tk.X, pady=8)
        ttk.Label(
            pose_tab,
            text=(
                "地图显示的是开环估计位置，并非传感器实测位置。\n"
                "首次请用小距离核对车体 X/Y 与场地轴方向。"
            ),
            foreground="#59636e",
            justify=tk.LEFT,
            wraplength=275,
        ).pack(anchor="w", pady=8)

        self.status_var = tk.StringVar(value="就绪：当前估计位置位于启停区2中心")
        ttk.Label(
            pose_tab,
            textvariable=self.status_var,
            foreground="#145a86",
            justify=tk.LEFT,
            wraplength=275,
        ).pack(anchor="w", pady=(16, 0))

        for command in ("GOTOpose", "Movepose", "En_C"):
            self._command_group(chassis_tab, command)
        ttk.Button(chassis_tab, text="向 ESP32 请求 help", command=lambda: self.send_command("help")).pack(
            fill=tk.X, pady=6
        )
        ttk.Label(
            chassis_tab,
            text="“En_C 0”仅为软件失能，不等同于物理急停。",
            foreground="#b03a2e",
            wraplength=285,
        ).pack(anchor="w", pady=5)

        ttk.Label(
            arm_tab,
            text="机械臂姿态预览",
            font=("Microsoft YaHei UI", 12, "bold"),
        ).pack(anchor="w", pady=(0, 5))
        self.arm_preview = ArmCanvas(arm_tab)
        self.arm_preview.pack(fill=tk.X, pady=(0, 8))

        arm_controls = ttk.LabelFrame(
            arm_tab,
            text="机械臂参数（-1 表示不操作该轴）",
            padding=7,
        )
        arm_controls.pack(fill=tk.X)
        arm_fields = (
            ("high", "高度", "-1", "mm"),
            ("length", "伸出长度", "-1", "mm"),
            ("turret_angle", "转台角度", "-1", "°"),
            ("pawl_angle", "夹爪角度", "-1", "°"),
            ("speed", "速度", "80", "mm/s"),
        )
        for row, (key, label, default, unit) in enumerate(arm_fields):
            variable = tk.StringVar(value=default)
            self.arm_vars[key] = variable
            ttk.Label(arm_controls, text=f"{label}：").grid(
                row=row, column=0, sticky="w", pady=2
            )
            ttk.Entry(arm_controls, textvariable=variable, width=13).grid(
                row=row, column=1, sticky="ew", pady=2
            )
            ttk.Label(arm_controls, text=unit).grid(
                row=row, column=2, sticky="w", padx=(5, 0)
            )
        arm_controls.columnconfigure(1, weight=1)
        ttk.Button(
            arm_controls,
            text="发送升降 / 伸出（MoveArm_1）",
            command=lambda: self.send_arm_command("MoveArm_1"),
        ).grid(row=5, column=0, columnspan=3, sticky="ew", pady=(7, 3))
        ttk.Button(
            arm_controls,
            text="发送转台 / 夹爪（MoveArm_2）",
            command=lambda: self.send_arm_command("MoveArm_2"),
        ).grid(row=6, column=0, columnspan=3, sticky="ew", pady=3)

        self.arm_status_var = tk.StringVar(
            value="输入参数会实时预览；-1 保留上次已发送的估计值。"
        )
        ttk.Label(
            arm_tab,
            textvariable=self.arm_status_var,
            foreground="#59636e",
            justify=tk.LEFT,
            wraplength=285,
        ).pack(anchor="w", pady=(7, 0))
        for variable in self.arm_vars.values():
            variable.trace_add("write", self.preview_arm_pose)
        self.preview_arm_pose()

        log_tab.rowconfigure(0, weight=1)
        log_tab.columnconfigure(0, weight=1)
        self.log_text = scrolledtext.ScrolledText(
            log_tab, width=38, height=28, state=tk.DISABLED, font=("Consolas", 9)
        )
        self.log_text.grid(row=0, column=0, columnspan=2, sticky="nsew")
        self.raw_command_var = tk.StringVar()
        ttk.Entry(log_tab, textvariable=self.raw_command_var).grid(
            row=1, column=0, sticky="ew", pady=(8, 0)
        )
        ttk.Button(log_tab, text="发送原始命令", command=self.send_raw_command).grid(
            row=1, column=1, padx=(5, 0), pady=(8, 0)
        )

        self.refresh_ports()
        self.root.after(60, self.poll_serial_events)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    @staticmethod
    def _pose_editor(
        parent: ttk.Frame, title: str, defaults: tuple[float, float, float]
    ) -> tuple[tk.StringVar, tk.StringVar, tk.StringVar]:
        group = ttk.LabelFrame(parent, text=title, padding=10)
        group.pack(fill=tk.X, pady=(0, 4))
        variables = tuple(tk.StringVar(value=f"{value:g}") for value in defaults)
        for row, (name, variable) in enumerate(zip(("X", "Y", "θ"), variables)):
            ttk.Label(group, text=f"{name}：", width=4).grid(row=row, column=0, sticky="w", pady=3)
            ttk.Entry(group, textvariable=variable, width=18).grid(
                row=row, column=1, sticky="ew", pady=3
            )
            ttk.Label(group, text="mm" if name != "θ" else "°").grid(
                row=row, column=2, sticky="w", padx=(5, 0)
            )
        group.columnconfigure(1, weight=1)
        return variables  # type: ignore[return-value]

    def _command_group(self, parent: ttk.Frame, command: str) -> None:
        group = ttk.LabelFrame(parent, text=command, padding=7)
        group.pack(fill=tk.X, pady=(0, 6))
        variables: list[tk.StringVar] = []
        for row, (label, default) in enumerate(COMMAND_FIELDS[command]):
            variable = tk.StringVar(value=default)
            variables.append(variable)
            ttk.Label(group, text=f"{label}：").grid(row=row, column=0, sticky="w", pady=2)
            ttk.Entry(group, textvariable=variable, width=13).grid(
                row=row, column=1, sticky="ew", pady=2
            )
        group.columnconfigure(1, weight=1)
        ttk.Button(group, text="发送", command=lambda name=command: self.send_command(name)).grid(
            row=len(variables), column=0, columnspan=2, sticky="ew", pady=(5, 0)
        )
        self.command_vars[command] = variables

    @staticmethod
    def _read_pose(variables: tuple[tk.StringVar, tk.StringVar, tk.StringVar]) -> Pose:
        try:
            values = tuple(float(variable.get().strip()) for variable in variables)
        except ValueError as exc:
            raise ValueError("X、Y、θ 必须填写数字") from exc
        if not all(math.isfinite(value) for value in values):
            raise ValueError("姿态不能包含无穷大或 NaN")
        pose = Pose(*values)
        if not pose_fits_field(pose):
            raise ValueError("该姿态会使 300×300 mm 车体超出场地边界")
        return pose

    def switch_start_zone(self) -> None:
        zone_name = next_start_zone(self.start_zone_name)
        pose = START_POSES[zone_name]
        if self.pending_target is not None:
            self.append_log("WARN", "切换启停区，已取消等待中的目标回显")
            self._clear_pending_target()
        self.start_zone_name = zone_name
        self.start_zone_var.set(f"当前选择：{zone_name}")
        self.start_zone_button.configure(text=f"切换到{next_start_zone(zone_name)}")
        for variable, value in zip(self.current_vars, (pose.x, pose.y, pose.theta)):
            variable.set(f"{value:g}")
        self.field.set_start_pose(pose)
        self.status_var.set(
            f"已切换到{zone_name}：X={pose.x:.0f} mm，Y={pose.y:.0f} mm，"
            f"θ={pose.theta:.0f}°"
        )

    def update_current(self) -> None:
        try:
            pose = self._read_pose(self.current_vars)
        except ValueError as exc:
            messagebox.showerror("姿态无效", str(exc), parent=self.root)
            return
        if self.pending_target is not None:
            self.append_log("WARN", "手动更新估计位置，已取消等待中的目标回显")
            self._clear_pending_target()
        self.field.set_current_pose(pose)
        self.status_var.set(
            f"估计位置：X={pose.x:.1f} mm，Y={pose.y:.1f} mm，θ={pose.theta % 360:.1f}°"
        )

    def preview_target(self) -> None:
        try:
            pose = self._read_pose(self.target_vars)
        except ValueError as exc:
            messagebox.showerror("目标无效", str(exc), parent=self.root)
            return
        self.field.set_target_pose(pose)
        self.status_var.set(
            f"目标预览：X={pose.x:.1f} mm，Y={pose.y:.1f} mm，θ={pose.theta % 360:.1f}°"
        )

    def move_to_target(self) -> None:
        if self.pending_target is not None:
            messagebox.showerror(
                "目标未发送",
                "上一条目标仍在等待 ESP32 回显；可断开串口或手动更新估计位置后重试。",
                parent=self.root,
            )
            return
        try:
            target = self._read_pose(self.target_vars)
            relative = target_to_relative_move(self.field.current_pose, target)
            line = build_debug_command(
                "GOTOpose",
                [str(relative.x), str(relative.y), str(relative.theta)],
            )
            self.serial_link.send_line(line)
        except (ValueError, RuntimeError, OSError) as exc:
            messagebox.showerror("目标未发送", str(exc), parent=self.root)
            return
        self.pending_target = target
        self.pending_relative_move = relative
        self.field.set_target_pose(target)
        self.append_log("TX", line)
        self.status_var.set(
            "已发送相对 GOTOpose，等待 ESP32 接令回显；回显不代表车辆真实到位。"
        )

    def _clear_pending_target(self) -> None:
        self.pending_target = None
        self.pending_relative_move = None

    def _accept_target_echo(self, text: str) -> None:
        target = self.pending_target
        relative = self.pending_relative_move
        if target is None or relative is None or parse_gotopose_echo(text) is None:
            return
        if not gotopose_echo_matches(text, relative):
            self.append_log("WARN", "收到其他 GOTOpose 回显，未更新目标估计位置")
            return
        self.field.commit_target_as_estimate(target)
        for variable, value in zip(self.current_vars, (target.x, target.y, target.theta)):
            variable.set(f"{value:g}")
        self._clear_pending_target()
        self.status_var.set(
            f"估计位置已更新：X={target.x:.1f} mm，Y={target.y:.1f} mm，"
            f"θ={target.theta % 360:.1f}°；仅确认已接令，未确认真实到位。"
        )

    def refresh_ports(self) -> None:
        ports = self.serial_link.available_ports()
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        if serial is None:
            self.connection_var.set("未安装 pyserial")

    def toggle_connection(self) -> None:
        if self.serial_link.is_open:
            self.serial_link.disconnect()
            if self.pending_target is not None:
                self._clear_pending_target()
                self.status_var.set("串口已断开；等待中的目标估计未更新。")
            self.connect_button.configure(text="连接")
            self.connection_var.set("未连接 · 115200 8N1")
            self.append_log("INFO", "串口已断开")
            return
        port_name = self.port_var.get().strip()
        if not port_name:
            messagebox.showerror("无法连接", "请先选择或输入 COM 端口", parent=self.root)
            return
        try:
            self.serial_link.connect(port_name)
        except Exception as exc:
            messagebox.showerror("串口连接失败", str(exc), parent=self.root)
            self.append_log("ERROR", str(exc))
            return
        self.connect_button.configure(text="断开")
        self.connection_var.set(f"已连接 {port_name} · 115200 8N1")
        self.append_log("INFO", f"已连接 {port_name}；等待 ESP32 Debug mode 输出")

    def send_command(self, command: str) -> None:
        if command == "GOTOpose" and self.pending_target is not None:
            messagebox.showerror(
                "命令未发送",
                "姿态页目标仍在等待回显，请勿同时发送另一条 GOTOpose。",
                parent=self.root,
            )
            return
        variables = self.command_vars.get(command, [])
        try:
            line = build_debug_command(command, [variable.get() for variable in variables])
            self.serial_link.send_line(line)
        except (ValueError, RuntimeError, OSError) as exc:
            messagebox.showerror("命令未发送", str(exc), parent=self.root)
            return
        self.append_log("TX", line)

    def _arm_request(self) -> ArmPose:
        return parse_arm_request(
            [
                self.arm_vars["high"].get(),
                self.arm_vars["length"].get(),
                self.arm_vars["turret_angle"].get(),
                self.arm_vars["pawl_angle"].get(),
            ]
        )

    def preview_arm_pose(self, *_trace_args: str) -> None:
        """使用全部四轴输入预览；尚未发送的值不会写入开环估计。"""
        try:
            requested = self._arm_request()
        except ValueError as exc:
            self.arm_status_var.set(f"预览暂停：{exc}")
            return
        preview = merge_arm_pose(self.arm_estimate, requested)
        self.arm_preview.set_pose(preview)
        self.arm_status_var.set(
            f"输入预览：H={preview.high:g} mm，L={preview.length:g} mm，"
            f"转台={preview.turret_angle:g}°，夹爪={preview.pawl_angle:g}°"
        )

    def send_arm_command(self, command: str) -> None:
        """分别发送固件当前能解析的两条三参数机械臂命令。"""
        if command == "MoveArm_1":
            raw_values = [
                self.arm_vars["high"].get(),
                self.arm_vars["length"].get(),
                self.arm_vars["speed"].get(),
            ]
            request_values = [raw_values[0], raw_values[1], "-1", "-1"]
            description = "升降 / 伸出"
        elif command == "MoveArm_2":
            raw_values = [
                self.arm_vars["turret_angle"].get(),
                self.arm_vars["pawl_angle"].get(),
                self.arm_vars["speed"].get(),
            ]
            request_values = ["-1", "-1", raw_values[0], raw_values[1]]
            description = "转台 / 夹爪"
        else:
            raise ValueError(f"不支持的机械臂命令：{command}")

        try:
            requested = parse_arm_request(request_values)
            line = build_debug_command(command, raw_values)
            self.serial_link.send_line(line)
        except (ValueError, RuntimeError, OSError) as exc:
            messagebox.showerror("命令未发送", str(exc), parent=self.root)
            return
        self.arm_estimate = merge_arm_pose(self.arm_estimate, requested)
        self.append_log("TX", line)
        self.preview_arm_pose()
        self.arm_status_var.set(
            f"{description}命令已发送；图中是开环估计，不表示机械臂已经到位。"
        )

    def send_raw_command(self) -> None:
        line = self.raw_command_var.get().strip()
        if not line:
            return
        if self.pending_target is not None and line.split(maxsplit=1)[0] == "GOTOpose":
            messagebox.showerror(
                "命令未发送",
                "姿态页目标仍在等待回显，请勿同时发送另一条 GOTOpose。",
                parent=self.root,
            )
            return
        if "\n" in line or "\r" in line or len(line.encode("utf-8")) > 90:
            messagebox.showerror("命令未发送", "原始命令不能换行，长度不能超过 90 字节", parent=self.root)
            return
        try:
            self.serial_link.send_line(line)
        except (RuntimeError, OSError) as exc:
            messagebox.showerror("命令未发送", str(exc), parent=self.root)
            return
        self.append_log("TX", line)
        self.raw_command_var.set("")

    def append_log(self, kind: str, text: str) -> None:
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{kind}] {text}\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def poll_serial_events(self) -> None:
        while True:
            try:
                kind, text = self.serial_events.get_nowait()
            except queue.Empty:
                break
            self.append_log(kind, text)
            if kind == "RX":
                self._accept_target_echo(text)
            if kind == "ERROR":
                self.serial_link.disconnect()
                if self.pending_target is not None:
                    self._clear_pending_target()
                    self.status_var.set("串口异常；等待中的目标估计未更新。")
                self.connect_button.configure(text="连接")
                self.connection_var.set("串口异常断开")
        self.root.after(60, self.poll_serial_events)

    def on_close(self) -> None:
        self.serial_link.disconnect()
        self.root.destroy()


def run_self_test() -> None:
    assert world_to_normalized(0.0, 0.0) == (1.0, 1.0)
    assert world_to_normalized(2400.0, 2400.0) == (0.0, 0.0)
    assert START_POSES["启停区1"] == Pose(2250.0, 150.0, 180.0)
    assert START_POSES["启停区2"] == Pose(150.0, 150.0, 0.0)
    assert next_start_zone("启停区1") == "启停区2"
    assert next_start_zone("启停区2") == "启停区1"
    assert all(pose_fits_field(pose) for pose in START_POSES.values())
    arm = ArmPose(50.0, 80.0, 30.0, 20.0)
    request = ArmPose(-1.0, 120.0, -1.0, 45.0)
    assert merge_arm_pose(arm, request) == ArmPose(50.0, 120.0, 30.0, 45.0)
    assert parse_arm_request(["-1", "120", "-1", "45"]) == request
    assert ArmCanvas._ratio(-10.0, 200.0) == 0.0
    assert ArmCanvas._ratio(100.0, 200.0) == 0.5
    assert ArmCanvas._ratio(250.0, 200.0) == 1.0
    assert pose_fits_field(Pose(150.0, 150.0, 0.0))
    assert not pose_fits_field(Pose(100.0, 150.0, 0.0))
    corners = robot_corners(Pose(1200.0, 1200.0, 0.0))
    assert math.dist(corners[0], corners[1]) == ROBOT_SIZE_MM
    assert math.dist(corners[1], corners[2]) == ROBOT_SIZE_MM
    assert shortest_angle_delta(350.0, 10.0) == 20.0
    assert shortest_angle_delta(10.0, 350.0) == -20.0
    relative = target_to_relative_move(Pose(150.0, 150.0, 0.0), Pose(600.0, 400.0, 90.0))
    assert relative == Pose(450.0, 250.0, 90.0)
    rotated = target_to_relative_move(Pose(1000.0, 1000.0, 90.0), Pose(1100.0, 1200.0, 0.0))
    assert math.isclose(rotated.x, 200.0, abs_tol=1e-9)
    assert math.isclose(rotated.y, -100.0, abs_tol=1e-9)
    assert rotated.theta == -90.0
    diagonal = target_to_relative_move(
        Pose(300.0, 300.0, 45.0), Pose(2100.0, 2100.0, 45.0)
    )
    assert build_debug_command(
        "GOTOpose", [str(diagonal.x), str(diagonal.y), str(diagonal.theta)]
    ).startswith("GOTOpose 2545.58 ")
    assert parse_gotopose_echo("GOTOpose 450, 250, 90") == Pose(450.0, 250.0, 90.0)
    assert parse_gotopose_echo("other 450, 250, 90") is None
    assert gotopose_echo_matches("GOTOpose 450, 250, 90", Pose(450.4, 249.6, 90.0))
    assert not gotopose_echo_matches("GOTOpose 451, 250, 90", Pose(450.4, 249.6, 90.0))
    assert build_debug_command("GOTOpose", ["100", "-20.5", "90"]) == "GOTOpose 100 -20.5 90"
    assert build_debug_command("Movepose", ["1", "80", "0"]) == "Movepose 1 80 0"
    assert build_debug_command("MoveArm_1", ["-1", "100", "80"]) == "MoveArm_1 -1 100 80"
    assert build_debug_command("MoveArm_2", ["30", "-1", "80"]) == "MoveArm_2 30 -1 80"
    assert build_debug_command("SERVO", ["2", "-45"]) == "SERVO 2 -45"
    assert build_debug_command("help", []) == "help"
    try:
        build_debug_command("En_C", ["2"])
    except ValueError:
        pass
    else:
        raise AssertionError("En_C 非法参数未被拒绝")
    try:
        parse_arm_request(["201", "0", "0", "0"])
    except ValueError:
        pass
    else:
        raise AssertionError("机械臂高度非法参数未被拒绝")
    print("self-test passed")


def main() -> None:
    root = tk.Tk()
    UpperComputerApp(root)
    root.mainloop()


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        run_self_test()
    else:
        main()
