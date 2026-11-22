"""JSON 参数文件：按固件参数名称匹配，文件本身不触发串口或机械动作。"""
import json
import math
from pathlib import Path
from action_group_config import DELAY_PREFIX

DEFAULT_CONFIG_PATH = Path(__file__).resolve().with_name("robot_parameters.json")
OUTER_ACCEL_LEVEL = "底盘/节点导航外圈加速度档位(EMM V5)"
LEGACY_OUTER_ACCEL = "底盘/节点导航平移加速度(mm/s²)"
MIDDLE_ACCEL_LEVEL = "底盘/节点导航中间路段加速度档位(EMM V5)"
LEGACY_MIDDLE_ACCEL = "底盘/节点导航中间路段平移加速度(mm/s²)"
Y_PULSES_PER_MM = "标定/Y脉冲每毫米"


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"JSON 出现重复名称：{key}")
        result[key] = value
    return result


def load_config(path):
    with open(path, encoding="utf-8-sig") as stream:
        data = json.load(stream, object_pairs_hook=_unique_object)
    if not isinstance(data, dict) or data.get("schema_version") != 1:
        raise ValueError("不支持的参数文件版本，需要 schema_version = 1")
    groups = data.get("groups")
    if not isinstance(groups, dict) or not groups:
        raise ValueError("参数文件缺少 groups")
    names = set()
    for group in groups.values():
        if not isinstance(group, dict) or not isinstance(group.get("parameters"), dict):
            raise ValueError("每组必须包含 parameters 对象")
        for name, entry in group["parameters"].items():
            if name in names:
                raise ValueError(f"参数重复：{name}")
            names.add(name)
            if not isinstance(entry, dict) or "value" not in entry:
                raise ValueError(f"{name} 缺少 value")
            value = entry["value"]
            if value is not None and (type(value) not in (int, float) or not math.isfinite(value)):
                raise ValueError(f"{name} 的 value 必须为有限数字或 null")
    return data


def parameter_entries(data, default_y_pulses=13.6, *, use_outer_level=True, use_middle_level=True):
    entries = {name: entry for group in data["groups"].values()
               for name, entry in group["parameters"].items()}
    for use_level, old_name, level_name in (
        (use_outer_level, LEGACY_OUTER_ACCEL, OUTER_ACCEL_LEVEL),
        (use_middle_level, LEGACY_MIDDLE_ACCEL, MIDDLE_ACCEL_LEVEL),
    ):
        if not use_level or old_name not in entries:
            continue
        old = entries.pop(old_name)["value"]
        if level_name not in entries:
            level = None
            if old is not None:
                pulses = entries.get(Y_PULSES_PER_MM, {}).get("value")
                pulses = default_y_pulses if pulses is None else pulses
                if not 1 <= old <= 4000 or pulses <= 0:
                    raise ValueError(f"无法换算参数：{old_name}")
                level = max(1, min(255, math.floor(256.5 - 20000 * 3200 / (old * pulses * 60))))
            entries[level_name] = {"value": level, "integer": True, "minimum": 1, "maximum": 255}
    return entries


def config_edits(data, parameters):
    """全量校验后返回编辑；范围以连接设备的固件回读值为准。"""
    by_name = {p.name: p for p in parameters.values()}
    obsolete_poses = {
        f"搬运/载物台{i}/{field}" for i in range(1, 4)
        for field in ("高度(mm)", "伸出(mm)", "转台(deg)")
    } if "搬运/待操作区/高度(mm)" in by_name else set()
    edits = {}
    y_pulses = float(by_name[Y_PULSES_PER_MM].value) if Y_PULSES_PER_MM in by_name else 13.6
    entries = parameter_entries(data, y_pulses, use_outer_level=OUTER_ACCEL_LEVEL in by_name,
                                use_middle_level=MIDDLE_ACCEL_LEVEL in by_name)
    for name, entry in entries.items():
        # 旧 JSON 中的地图坐标不再导入，节点默认坐标由固件源码维护。
        if name.startswith("地图/") or name in obsolete_poses:
            continue
        if name not in by_name:
            raise ValueError(f"当前固件不支持参数：{name}")
        value = entry["value"]
        if value is None:
            continue
        p = by_name[name]
        normalized = p.validate(str(value))
        if float(normalized) != float(p.value):
            edits[p.index] = normalized
    return edits


def export_config(path, parameters, edits):
    template = load_config(DEFAULT_CONFIG_PATH)
    descriptions = {
        name: entry for group in template["groups"].values()
        for name, entry in group["parameters"].items()
    }
    groups = {}
    for p in parameters.values():
        # 动作组延时单独写回 C++；地图坐标也由源码维护，不导出到 JSON。
        if p.name.startswith((DELAY_PREFIX, "地图/")):
            continue
        category = p.name.split("/", 1)[0]
        group = groups.setdefault(category, {
            "description": template["groups"].get(category, {}).get("description", "固件运行参数"),
            "parameters": {},
        })
        entry = dict(descriptions.get(p.name, {}))
        value = float(edits.get(p.index, p.value))
        entry.update(value=(int(value) if p.integer else value) if math.isfinite(value) else None,
                     minimum=p.minimum, maximum=p.maximum, integer=p.integer)
        group["parameters"][p.name] = entry
    template["groups"] = groups
    template["description"] = "导出自上位机：包含待发送编辑；null 表示未标定，导入时跳过。"
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(template, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
