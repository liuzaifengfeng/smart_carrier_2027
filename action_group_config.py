"""动作组延时以 C++ 为默认值来源；只解析和改写标记区域内的数字。"""
from decimal import Decimal, InvalidOperation
import os
from pathlib import Path
import re
import tempfile

DEFAULT_SOURCE_PATH = Path(__file__).resolve().parent / "src" / "material_transfer.cpp"
DELAY_PREFIX = "动作组延时/"
TRANSFER_STEPS = ("取料平移", "取料稳定", "取料转向", "取料下降", "取料夹紧", "取料抬升",
                  "放料平移", "放料稳定", "放料转向", "放料下降", "放料松爪", "放料抬升")
FORWARD_TRANSFER_GROUPS = tuple(f"载物台{c}到工作区{w}" for c in range(1, 4) for w in range(1, 4))
REVERSE_TRANSFER_GROUPS = tuple(f"工作区{w}到载物台{c}" for w in range(1, 4) for c in range(1, 4))
TRANSFER_GROUPS = ("公共", *FORWARD_TRANSFER_GROUPS, *REVERSE_TRANSFER_GROUPS)
AUXILIARY_STEPS = ("圆盘/预抓", "圆盘/夹紧", "圆盘/抬升", "扫码准备1/抬升",
                   "扫码准备1/转向", "扫码准备2/抬升", "扫码准备2/转向")
_LEGACY_DELAY_NAMES = tuple(f"{DELAY_PREFIX}{group}/{step}(ms)"
                          for group in ("公共", *FORWARD_TRANSFER_GROUPS) for step in TRANSFER_STEPS) + tuple(
                              f"{DELAY_PREFIX}{step}(ms)" for step in AUXILIARY_STEPS)
_REVERSE_DELAY_NAMES = tuple(f"{DELAY_PREFIX}{group}/{step}(ms)"
                           for group in REVERSE_TRANSFER_GROUPS for step in TRANSFER_STEPS)
DELAY_NAMES = _LEGACY_DELAY_NAMES + _REVERSE_DELAY_NAMES


def _numbers(count):
    return r"\s*,\s*".join([r"([0-9]+)"] * count)


_APPROACH = r"\{\s*" + _numbers(3) + r"\s*\}"
_TRANSFER = (r"\{\s*" + _APPROACH + r"\s*,\s*" + _numbers(3) + r"\s*,\s*"
             + _APPROACH + r"\s*,\s*" + _numbers(3) + r"\s*\}")
_STATION = r"\{\s*" + r"\s*,\s*".join([_TRANSFER] * 3) + r"\s*,?\s*\}"
_LAYOUT = re.compile(
    r"\s*MaterialTransferDelays\s+commonMaterialDelays\s*=\s*" + _TRANSFER
    + r"\s*;\s*MaterialTransferDelays\s+cargoToWorkAreaDelays\s*"
    + r"\[\s*MATERIAL_STATION_COUNT\s*\]\s*\[\s*MATERIAL_STATION_COUNT\s*\]\s*=\s*\{\s*"
    + r"\s*,\s*".join([_STATION] * 3) + r"\s*,?\s*\}\s*;\s*"
    + r"MaterialAuxiliaryDelays\s+materialAuxiliaryDelays\s*=\s*\{\s*"
    + _numbers(7) + r"\s*\}\s*;\s*")
_REVERSE_LAYOUT = re.compile(
    r"\s*MaterialTransferDelays\s+workAreaToCargoDelays\s*"
    + r"\[\s*MATERIAL_STATION_COUNT\s*\]\s*\[\s*MATERIAL_STATION_COUNT\s*\]\s*=\s*\{\s*"
    + r"\s*,\s*".join([_STATION] * 3) + r"\s*,?\s*\}\s*;\s*")


def _parse_source(original):
    text = original.decode("utf-8")
    entries = {}
    for marker, layout, names in (
            ("ACTION_GROUP_DELAYS", _LAYOUT, _LEGACY_DELAY_NAMES),
            ("REVERSE_ACTION_GROUP_DELAYS", _REVERSE_LAYOUT, _REVERSE_DELAY_NAMES)):
        markers = [list(re.finditer(r"(?m)^// " + marker + "_" + kind + r"\s*$", text))
                   for kind in ("BEGIN", "END")]
        if any(len(matches) != 1 for matches in markers) or markers[0][0].end() >= markers[1][0].start():
            raise ValueError("C++ 延时区域标记缺失、重复或顺序错误，未修改文件")
        start, end = markers[0][0].end(), markers[1][0].start()
        # 用等长空格屏蔽注释，保证数字位置仍对应原文；不接受任意 C++ 表达式。
        body = re.sub(r"//[^\r\n]*|/\*.*?\*/", lambda match: " " * len(match[0]), text[start:end], flags=re.S)
        match = layout.fullmatch(body)
        if match is None:
            raise ValueError("C++ 延时配置结构已改变，请保留公共、正反向各9组及辅助延时表，未修改文件")
        for index, name in enumerate(names, 1):
            value = int(match[index])
            if not 0 <= value <= 60000:
                raise ValueError(f"C++ 延时超出 0～60000 ms：{name}")
            entries[name] = (start + match.start(index), start + match.end(index), value)
    return text, entries


def load_action_group_delays(path=DEFAULT_SOURCE_PATH):
    """读取本地 C++ 默认延时；不会发送串口命令。"""
    _, entries = _parse_source(Path(path).read_bytes())
    return {name: entry[2] for name, entry in entries.items()}


def save_action_group_delays(values, path=DEFAULT_SOURCE_PATH):
    """保存指定延时，支持单组或全部；保留注释、格式和其余组，返回保存项数。"""
    path = Path(path)
    original = path.read_bytes()
    text, entries = _parse_source(original)
    replacements = []
    for name, value in values.items():
        if name not in entries:
            raise ValueError(f"未知动作组延时：{name}")
        try:
            number = Decimal(str(value))
        except InvalidOperation as exc:
            raise ValueError(f"延时必须是 0～60000 的整数：{name}") from exc
        if not number.is_finite() or not 0 <= number <= 60000 or number != number.to_integral_value():
            raise ValueError(f"延时必须是 0～60000 的整数：{name}")
        start, end, _ = entries[name]
        replacements.append((start, end, str(int(number))))
    if not replacements:
        raise ValueError("没有可保存的动作组延时")
    for start, end, value in sorted(replacements, reverse=True):
        text = text[:start] + value + text[end:]
    updated = text.encode("utf-8")
    if updated == original:
        return len(replacements)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=".delays-", suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(updated)
        if path.read_bytes() != original:
            raise ValueError("保存期间 C++ 文件已被其他操作修改，请重新读取后保存")
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return len(replacements)
