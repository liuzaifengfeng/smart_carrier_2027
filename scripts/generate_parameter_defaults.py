"""把 JSON 数值变为固件启动默认值；可独立运行以预检参数文件。"""
import json
import math
from pathlib import Path
import sys

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))
from parameter_config import load_config


def supported_names():
    names = set()
    for pose in ("圆盘", *(f"载物台{i}" for i in range(1, 4)),
                 *(f"工作区{i}" for i in range(1, 4))):
        names.update(f"搬运/{pose}/{field}" for field in
                     ("高度(mm)", "伸出(mm)", "转台(deg)"))
    names.update("搬运/" + field for field in
                 ("安全高度(mm)", "二层高度增量(mm)", "速度",
                  "夹爪张开(deg)", "夹爪夹紧(deg)", "等待时间(ms)"))
    names.update(f"搬运/圆盘预抓/{field}" for field in
                 ("高度(mm)", "伸出(mm)", "转台(deg)", "张爪(deg)"))
    names.add("搬运/圆盘提起/高度(mm)")
    names.update("搬运/速度/" + step for step in
                 ("圆盘预抓", "圆盘夹紧", "圆盘提起", "移至工位平移", "移至工位转向",
                  "取料下降", "取料夹紧", "取料提起", "放料下降", "放料松爪", "放料提起"))
    names.update(f"PID/{axis}/{field}" for axis in ("视觉X", "视觉Y", "角度")
                 for field in ("Kp", "Ki", "Kd", "积分限幅"))
    names.update(f"PID/CORNER/{axis}/{field}" for axis in ("视觉X", "视觉Y", "角度")
                 for field in ("Kp", "Ki", "Kd", "积分限幅"))
    names.update(f"对齐/{mode}/{field}" for mode in
                 ("DISC", "DISC_MATERIAL", "WORK_AREA", "WORK_AREA_LOADED", "CORNER")
                 for field in ("X死区", "Y死区", "角度死区(deg)"))
    names.add("对齐/最大轮速(RPM)")
    names.add("底盘/停车稳定时间(ms)")
    names.add("底盘/节点导航平移加速度(mm/s²)")
    names.add("底盘/节点导航中间路段速度(mm/s)")
    names.add("底盘/节点导航中间路段平移加速度(mm/s²)")
    names.add("底盘/节点导航转向速度(°/s)")
    names.add("底盘/节点导航转向加速度(°/s²)")
    names.add("底盘/节点导航独立纠偏死区(°)")
    names.update("底盘/" + field for field in
                 ("节点导航速度(mm/s)", "扫码接近轮速(RPM)", "扫码往返轮速(RPM)"))
    names.update("标定/" + field for field in
                 ("X脉冲每毫米", "Y脉冲每毫米", "旋转脉冲每度",
                  "升降脉冲每毫米", "伸缩脉冲每毫米"))
    names.update("运动执行/" + field for field in
                 ("look末段提前量(mm)", "look2末段提前量(mm)", "机械臂反馈完成",
                  "反馈后夹持保持(ms)", "反馈后释放保持(ms)"))
    return names


def generate(config_path, output_path):
    data = load_config(config_path)
    allowed = supported_names()
    entries = []
    for group in data["groups"].values():
        for name, entry in group["parameters"].items():
            if name not in allowed:
                raise ValueError(f"固件不支持参数：{name}")
            value = entry["value"]
            if value is None:
                continue  # 未标定项保留固件源码值。
            minimum, maximum = entry.get("minimum"), entry.get("maximum")
            integer = entry.get("integer")
            if (type(minimum) not in (int, float) or type(maximum) not in (int, float)
                    or not math.isfinite(minimum) or not math.isfinite(maximum)
                    or minimum > maximum or type(integer) is not bool
                    or not minimum <= value <= maximum
                    or (integer and (int(value) != value or value < 0 or value > 4294967295))):
                raise ValueError(f"参数超出范围或类型不正确：{name}")
            entries.append((name, value))
    if not entries:
        raise ValueError("JSON 没有可编译的参数")
    rows = ["// Generated from robot_parameters.json; do not edit.\n",
            "#pragma once\n#include <stddef.h>\n",
            "struct CompiledParameterDefault { const char *name; double value; };\n",
            "static const CompiledParameterDefault COMPILED_PARAMETER_DEFAULTS[] = {\n"]
    for name, value in entries:
        rows.append(f"    {{{json.dumps(name, ensure_ascii=False)}, {format(value, '.17g')}}},\n")
    rows.extend(["};\n", "static constexpr size_t COMPILED_PARAMETER_DEFAULT_COUNT = ",
                 f"{len(entries)};\n"])
    contents = "".join(rows)
    output_path = Path(output_path)
    if not output_path.exists() or output_path.read_text(encoding="utf-8") != contents:
        output_path.write_text(contents, encoding="utf-8")
    return len(entries)


if __name__ == "__main__":
    try:
        count = generate(PROJECT_DIR / "robot_parameters.json",
                         PROJECT_DIR / "src" / "generated_parameter_defaults.h")
    except (OSError, ValueError) as exc:
        raise SystemExit(f"参数文件错误，停止编译：{exc}") from exc
    print(f"Generated {count} compiled parameter defaults")
