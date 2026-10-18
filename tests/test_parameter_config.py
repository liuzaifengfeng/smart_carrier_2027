"""参数文件的名称映射、边界校验及导入/导出，不连接硬件。"""
import json
import tkinter as tk
import unittest
import uuid
from contextlib import contextmanager
from pathlib import Path
from unittest.mock import patch

from parameter_config import DEFAULT_CONFIG_PATH, config_edits, export_config, load_config
from scripts.generate_parameter_defaults import generate
from upper_computer import Parameter, ParameterPanel


@contextmanager
def temporary_folder():
    # Python 3.13 的 Windows 临时目录私有 ACL 与受限执行环境不兼容。
    folder = DEFAULT_CONFIG_PATH.parent / ('.parameter-test-' + uuid.uuid4().hex)
    folder.mkdir()
    try:
        yield folder
    finally:
        for path in folder.iterdir():
            path.unlink()
        folder.rmdir()


class ConfigTests(unittest.TestCase):
    def setUp(self):
        self.data = load_config(DEFAULT_CONFIG_PATH)
        self.parameters = {}
        for group in self.data['groups'].values():
            for name, entry in group['parameters'].items():
                index = len(self.parameters)
                self.parameters[index] = Parameter(index, name, str(entry['value']),
                    entry['minimum'], entry['maximum'], entry['integer'])

    def test_defaults_and_name_mapping(self):
        self.assertEqual(len(self.parameters), 133)
        self.assertEqual(config_edits(self.data, self.parameters), {})
        chassis = self.data['groups']['底盘']['parameters']
        for name, value in (('节点导航速度(mm/s)', 1200), ('节点导航平移加速度(mm/s²)', 1600),
                            ('节点导航转向速度(°/s)', 150), ('节点导航转向加速度(°/s²)', 360)):
            self.assertEqual(chassis['底盘/' + name]['value'], value)
        chassis = self.data['groups']['底盘']['parameters']
        for name, value in (('节点导航速度(mm/s)', 1200), ('节点导航平移加速度(mm/s²)', 1600),
                            ('节点导航转向速度(°/s)', 150), ('节点导航转向加速度(°/s²)', 360)):
            self.assertEqual(chassis['底盘/' + name]['value'], value)
        p = next(p for p in self.parameters.values() if p.name == 'PID/视觉X/Kp')
        p.index = 999
        self.data['groups']['PID']['parameters'][p.name]['value'] = 0.02
        self.assertEqual(config_edits(self.data, self.parameters), {999: '0.02'})

    def test_node_acceleration_exports_and_compiles_as_default(self):
        name = '底盘/节点导航平移加速度(mm/s²)'
        p = next(p for p in self.parameters.values() if p.name == name)
        with temporary_folder() as folder:
            source, header = folder/'settings.json', folder/'defaults.h'
            export_config(source, self.parameters, {p.index: '450.5'})
            saved = load_config(source)
            self.assertEqual(saved['groups']['底盘']['parameters'][name]['value'], 450.5)
            self.assertEqual(generate(source, header), 133)
            self.assertIn(f'"{name}", 450.5', header.read_text(encoding='utf-8'))

    def test_device_range_wins_and_unknown_name_rejected(self):
        p = self.parameters[0]
        p.maximum = 100
        with self.assertRaises(ValueError):
            config_edits(self.data, self.parameters)
        with self.assertRaises(ValueError):
            config_edits(self.data, {})

    def test_invalid_json_values_and_duplicates(self):
        with temporary_folder() as folder:
            path = Path(folder) / 'bad.json'
            for value in ['true', '"123"', 'NaN', 'Infinity', '[]']:
                path.write_text('{"schema_version":1,"groups":{"a":{"parameters":{"x":{"value":'+value+'}}}}}', encoding='utf-8')
                with self.subTest(value=value), self.assertRaises(ValueError):
                    load_config(path)
            path.write_text('{"schema_version":1,"schema_version":1}', encoding='utf-8')
            with self.assertRaises(ValueError):
                load_config(path)

    def test_export_round_trip_includes_edits_and_null(self):
        self.parameters[1].value = 'nan'
        with temporary_folder() as folder:
            path = Path(folder) / 'saved.json'
            export_config(path, self.parameters, {0:'145'})
            saved = load_config(path)
            self.assertEqual(config_edits(saved, self.parameters), {0:'145'})
            self.assertIsNone(saved['groups']['搬运']['parameters'][self.parameters[1].name]['value'])

    def test_compile_uses_json_value_and_rejects_invalid_number(self):
        name = '搬运/等待时间(ms)'
        entry = self.data['groups']['搬运']['parameters'][name]
        with temporary_folder() as folder:
            source, header = folder / 'robot.json', folder / 'generated.h'
            entry['value'] = 1500
            source.write_text(json.dumps(self.data, ensure_ascii=False), encoding='utf-8')
            self.assertEqual(generate(source, header), 133)
            self.assertIn('"搬运/等待时间(ms)", 1500', header.read_text(encoding='utf-8'))
            entry['value'] = 60001
            source.write_text(json.dumps(self.data, ensure_ascii=False), encoding='utf-8')
            with self.assertRaises(ValueError):
                generate(source, header)
            self.assertIn('"搬运/等待时间(ms)", 1500', header.read_text(encoding='utf-8'))


class JsonPanelTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.sent = []
        self.panel = ParameterPanel(self.root, self.sent.append, lambda *_: None)

    def tearDown(self):
        self.panel.cancel_timer()
        self.root.destroy()

    def test_import_stages_without_sending_and_refreshes_selection(self):
        self.panel.read()
        seq = self.panel.sequence
        self.panel.receive(f'{{RSP,CFG,GET,VALUE,{seq},0,搬运/高度,100,0,175,0}}')
        self.panel.receive(f'{{RSP,CFG,GET,VALUE,{seq},1,搬运/等待,1000,1,60000,1}}')
        self.panel.receive(f'{{RSP,CFG,GET,END,{seq},2}}')
        self.panel.tree.selection_set('0')
        self.panel.select()
        data = {'schema_version':1, 'groups':{'搬运':{'parameters':{
            '搬运/高度':{'value':120}, '搬运/等待':{'value':500}}}}}
        with temporary_folder() as folder:
            path = Path(folder) / 'config.json'
            path.write_text(json.dumps(data), encoding='utf-8')
            previous = list(self.sent)
            with patch('upper_computer.filedialog.askopenfilename', return_value=str(path)):
                self.panel.import_json()
            self.assertEqual(self.sent, previous)
            self.assertEqual(self.panel.edits, {0:'120',1:'500'})
            self.assertEqual(self.panel.value.get(), '120')
            data['groups']['搬运']['parameters']['搬运/等待']['value'] = 0
            path.write_text(json.dumps(data), encoding='utf-8')
            with patch('upper_computer.filedialog.askopenfilename', return_value=str(path)), patch('upper_computer.messagebox.showerror'):
                self.panel.import_json()
            self.assertEqual(self.panel.edits, {0:'120',1:'500'})
            self.panel.send()
            self.assertIn('CFG,BEGIN', self.sent[-1])


if __name__ == '__main__':
    unittest.main()
