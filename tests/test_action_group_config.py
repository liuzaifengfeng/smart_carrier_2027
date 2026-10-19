"""验证动作组延时只写回指定数字，并与 JSON 默认参数保持独立。"""
from pathlib import Path
import re
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from action_group_config import (DEFAULT_SOURCE_PATH, DELAY_NAMES, DELAY_PREFIX,
                                TRANSFER_STEPS, load_action_group_delays, save_action_group_delays)
from parameter_config import export_config, load_config
from scripts.generate_parameter_defaults import generate


class ActionGroupConfigTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.source = Path(self.folder.name) / "material_transfer.cpp"
        self.original = DEFAULT_SOURCE_PATH.read_bytes()
        self.source.write_bytes(self.original)

    def test_names_match_firmware_and_defaults(self):
        values = load_action_group_delays(self.source)
        self.assertEqual(len(values), 235)
        self.assertEqual(len(set(DELAY_NAMES)), 235)
        runtime = DEFAULT_SOURCE_PATH.with_name("runtime_parameters.cpp").read_text(encoding="utf-8")
        steps = re.search(r"steps\[\] = \{(.*?)\};", runtime, re.S)[1]
        self.assertEqual(tuple(re.findall(r'"([^"]+)"', steps)), TRANSFER_STEPS)
        auxiliary = re.findall(r'RegisterRuntimeUInt\("(动作组延时/[^\"]+\(ms\))"', runtime)
        self.assertEqual(tuple(auxiliary), DELAY_NAMES[120:127])
        for c in range(1, 4):
            for w in range(1, 4):
                row = next(line for line in self.original.splitlines() if f"// {c} -> {w}".encode() in line)
                numbers = [int(v) for v in re.findall(rb"\d+", row.split(b"//")[0])]
                self.assertEqual([values[f"{DELAY_PREFIX}载物台{c}到工作区{w}/{s}(ms)"] for s in TRANSFER_STEPS],
                                 numbers)
                marker = f"// 工作区{w} -> 载物台{c}".encode()
                reverse = next(line for line in self.original.splitlines() if marker in line)
                numbers = [int(v) for v in re.findall(rb"\d+", reverse.split(b"//")[0])]
                self.assertEqual([values[f"{DELAY_PREFIX}工作区{w}到载物台{c}/{s}(ms)"] for s in TRANSFER_STEPS],
                                 numbers)

    def test_save_one_group_preserves_all_other_bytes_and_crlf_bom(self):
        original = b"\xef\xbb\xbf" + self.original.decode("utf-8").replace("\r\n", "\n").replace("\n", "\r\n").encode("utf-8")
        for group, marker in (("载物台2到工作区3", "// 2 -> 3"), ("工作区2到载物台3", "// 工作区2 -> 载物台3")):
            self.source.write_bytes(original)
            before = load_action_group_delays(self.source)
            edits = {f"{DELAY_PREFIX}{group}/{step}(ms)": str(i * 57) for i, step in enumerate(TRANSFER_STEPS)}
            self.assertEqual(save_action_group_delays(edits, self.source), 12)
            self.assertEqual(load_action_group_delays(self.source), before | {name: int(v) for name, v in edits.items()})
            updated = self.source.read_bytes()
            old_line = next(line for line in original.splitlines(keepends=True) if marker.encode() in line)
            new_line = next(line for line in updated.splitlines(keepends=True) if marker.encode() in line)
            self.assertEqual(updated, original.replace(old_line, new_line))
            self.assertEqual([int(v) for v in re.findall(rb"\d+", new_line.split(b"//")[0])], list(range(0, 12 * 57, 57)))

    def test_save_all_and_unchanged_save(self):
        values = {name: i * 251 for i, name in enumerate(DELAY_NAMES)}
        self.assertEqual(save_action_group_delays(values, self.source), 235)
        self.assertEqual(load_action_group_delays(self.source), values)
        for w in range(1, 4):
            for c in range(1, 4):
                row = next(line for line in self.source.read_bytes().splitlines()
                           if f"// 工作区{w} -> 载物台{c}".encode() in line)
                self.assertEqual([int(v) for v in re.findall(rb"\d+", row.split(b"//")[0])],
                                 [values[f"{DELAY_PREFIX}工作区{w}到载物台{c}/{s}(ms)"] for s in TRANSFER_STEPS])
        with patch("action_group_config.tempfile.NamedTemporaryFile", side_effect=AssertionError("不应重复写入")):
            self.assertEqual(save_action_group_delays(values, self.source), 235)

    def test_invalid_values_do_not_partially_write(self):
        for value in (-1, 60001, 0.5, "60000.00000000001", "nan", "inf", True, None, []):
            with self.subTest(value=value), self.assertRaises(ValueError):
                save_action_group_delays({DELAY_NAMES[0]: 100, DELAY_NAMES[1]: value}, self.source)
            self.assertEqual(self.source.read_bytes(), self.original)
        for edits in ({}, {"未知延时": 100}):
            with self.assertRaises(ValueError):
                save_action_group_delays(edits, self.source)
        self.assertEqual(self.source.read_bytes(), self.original)

    def test_changed_source_layout_is_rejected_without_write(self):
        begin, end = b"// ACTION_GROUP_DELAYS_BEGIN", b"// ACTION_GROUP_DELAYS_END"
        row = next(line for line in self.original.splitlines(keepends=True) if b"// 1 -> 2" in line)
        number = re.search(rb"commonMaterialDelays\s*=\s*\{\s*\{\s*(\d+)", self.original)
        start, stop = number.span(1)
        variants = (self.original.replace(begin, b"// missing"),
                    self.original.replace(end, begin + b"\n" + end),
                    self.original.replace(row, b""),
                    self.original[:stop] + b" + 1" + self.original[stop:],
                    self.original[:start] + b"60001" + self.original[stop:],
                    self.original.replace(end, b"int unexpected = 5;\n" + end))
        reverse_begin, reverse_end = b"// REVERSE_ACTION_GROUP_DELAYS_BEGIN", b"// REVERSE_ACTION_GROUP_DELAYS_END"
        reverse_row = next(line for line in self.original.splitlines(keepends=True)
                           if "// 工作区1 -> 载物台2".encode() in line)
        reverse_number = re.search(rb"workAreaToCargoDelays[^=]*=\s*\{\s*\{\s*\{\s*\{\s*(\d+)", self.original)
        start, stop = reverse_number.span(1)
        variants += (self.original.replace(reverse_begin, b"// missing"),
                     self.original.replace(reverse_end, reverse_begin + b"\n" + reverse_end),
                     self.original.replace(reverse_row, b""),
                     self.original[:stop] + b" + 1" + self.original[stop:],
                     self.original[:start] + b"60001" + self.original[stop:],
                     self.original.replace(reverse_end, b"int unexpected = 5;\n" + reverse_end))
        for original in variants:
            self.source.write_bytes(original)
            with self.assertRaises(ValueError):
                save_action_group_delays({DELAY_NAMES[0]: 100}, self.source)
            self.assertEqual(self.source.read_bytes(), original)

    def test_replace_failure_and_concurrent_edit(self):
        with patch("action_group_config.os.replace", side_effect=OSError("写入失败")), self.assertRaises(OSError):
            save_action_group_delays({DELAY_NAMES[0]: 100}, self.source)
        self.assertEqual(self.source.read_bytes(), self.original)
        self.assertEqual(list(Path(self.folder.name).iterdir()), [self.source])
        read_bytes = Path.read_bytes
        calls = 0

        def concurrent_read(path):
            nonlocal calls
            calls += 1
            if calls == 2:
                path.write_bytes(self.original + b"\n// external edit\n")
            return read_bytes(path)

        with patch("action_group_config.Path.read_bytes", concurrent_read), self.assertRaises(ValueError):
            save_action_group_delays({DELAY_NAMES[0]: 100}, self.source)
        self.assertEqual(self.source.read_bytes(), self.original + b"\n// external edit\n")
        self.assertEqual(list(Path(self.folder.name).iterdir()), [self.source])

    def test_json_export_excludes_delays_and_still_compiles(self):
        parameters = {0: SimpleNamespace(index=0, name="搬运/速度", value="250", minimum=1, maximum=300, integer=False)}
        for index, name in enumerate(DELAY_NAMES, 1):
            parameters[index] = SimpleNamespace(index=index, name=name, value="680", minimum=0, maximum=60000, integer=True)
        path, header = Path(self.folder.name) / "robot.json", Path(self.folder.name) / "defaults.h"
        export_config(path, parameters, {0: "270", 1: "123"})
        data = load_config(path)
        self.assertEqual(set(data["groups"]), {"搬运"})
        self.assertEqual(data["groups"]["搬运"]["parameters"]["搬运/速度"]["value"], 270)
        self.assertEqual(generate(path, header), 1)
        self.assertNotIn(DELAY_PREFIX, header.read_text(encoding="utf-8"))
        self.assertEqual(self.source.read_bytes(), self.original)
