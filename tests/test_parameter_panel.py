"""无硬件测试：校验参数边界、串口确认顺序、失败中止和回读确认。"""
import tkinter as tk
import unittest
from unittest.mock import patch

from upper_computer import (Parameter, ParameterPanel, parse_parameter,
                            ActionGroupPanel, parameter_matches_stage, build_parameter_test_command,
                            UpperComputerApp)
from continuous_trial import TrialPlan, TrialConfig
from action_group_config import DELAY_NAMES, DELAY_PREFIX, TRANSFER_GROUPS, load_action_group_delays


class ValidationTests(unittest.TestCase):
    def test_stage_action_uses_tuned_value(self):
        values = {"搬运/圆盘/高度(mm)": 145, "搬运/速度": 250}
        self.assertEqual(build_parameter_test_command("试选中机械臂轴", "圆盘抓取（两轮）",
            "搬运/圆盘/高度(mm)", values), "{CMD,ARM,MOVE1,145,-1,250}")
        self.assertEqual(build_parameter_test_command("三件码垛第二层", "暂存第二层", None, values),
                         "{CMD,ARM,DEMO2}")
        self.assertEqual(build_parameter_test_command("开启视觉定位", "暂存第二层", None, values),
                         "{CMD,VISION,START_REQUEST,WORK_AREA_LOADED}")
        self.assertEqual(build_parameter_test_command("圆盘抓取一件", "圆盘抓取（两轮)", None,
                         values, color=2, cargo=3), "{CMD,VISION,FORCE_GRAB,2,3}")

    def test_competition_stages_show_shared_and_specific_parameters(self):
        work = "搬运/工作区1/高度(mm)"
        self.assertTrue(parameter_matches_stage(work, "粗加工（两轮）"))
        self.assertTrue(parameter_matches_stage(work, "暂存第一层"))
        self.assertTrue(parameter_matches_stage(work, "暂存第二层"))
        self.assertTrue(parameter_matches_stage("搬运/二层高度增量(mm)", "暂存第二层"))
        self.assertFalse(parameter_matches_stage("搬运/二层高度增量(mm)", "暂存第一层"))
        self.assertTrue(parameter_matches_stage("对齐/DISC/X死区", "圆盘抓取（两轮）"))
        self.assertFalse(parameter_matches_stage("对齐/CORNER/X死区", "圆盘抓取（两轮）"))
        self.assertTrue(parameter_matches_stage("搬运/圆盘预抓/高度(mm)", "圆盘抓取（两轮）"))
        self.assertTrue(parameter_matches_stage("搬运/速度/圆盘夹紧", "圆盘抓取（两轮）"))
        self.assertTrue(parameter_matches_stage("搬运/速度/放料下降", "粗加工（两轮）"))
        self.assertTrue(parameter_matches_stage("底盘/扫码往返轮速(RPM)", "开局与扫码"))
        self.assertTrue(parameter_matches_stage("底盘/节点导航速度(mm/s)", "圆盘抓取（两轮）"))
        for stage in ("底盘导航与标定", "圆盘抓取（两轮）", "粗加工（两轮）", "暂存第一层", "暂存第二层", "返航归位"):
            self.assertTrue(parameter_matches_stage("底盘/节点导航平移加速度(mm/s²)", stage))
            self.assertTrue(parameter_matches_stage("底盘/节点导航速度(mm/s)", stage))
            self.assertTrue(parameter_matches_stage("底盘/节点导航转向速度(°/s)", stage))
            self.assertTrue(parameter_matches_stage("底盘/节点导航转向加速度(°/s²)", stage))
            self.assertTrue(parameter_matches_stage("底盘/节点导航独立纠偏死区(°)", stage))
            self.assertTrue(parameter_matches_stage("底盘/节点导航转向速度(°/s)", stage))
            self.assertTrue(parameter_matches_stage("底盘/节点导航转向加速度(°/s²)", stage))

    def test_float_limits_and_nonfinite(self):
        p = Parameter(0, "高度", "100", 0, 175, False)
        self.assertEqual(p.validate("175"), "175")
        for value in ("176", "-1", "nan", "inf", "abc"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                p.validate(value)

    def test_integer_and_uncalibrated_read(self):
        p = Parameter(0, "等待", "1000", 1, 60000, True)
        self.assertEqual(p.validate("1.0"), "1")
        with self.assertRaises(ValueError):
            p.validate("1.5")
        p = parse_parameter(["VALUE", "1", "0", "高度", "nan", "0", "175", "0"])
        self.assertEqual(p.value, "nan")
        with self.assertRaises(ValueError):
            parse_parameter(["VALUE", "1", "0", "高度", "1", "175", "0", "0"])


class PanelTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.sent = []
        self.panel = ParameterPanel(self.root, self.sent.append, lambda *_: None)

    def tearDown(self):
        self.panel.cancel_timer()
        self.root.destroy()

    def feed_values(self, values=("100", "1000")):
        seq = self.panel.sequence
        self.panel.receive(f"{{RSP,CFG,GET,VALUE,{seq},0,搬运/高度,{values[0]},0,175,0}}")
        self.panel.receive(f"{{RSP,CFG,GET,VALUE,{seq},1,搬运/等待,{values[1]},1,60000,1}}")
        self.panel.receive(f"{{RSP,CFG,GET,END,{seq},2}}")

    def read(self):
        self.panel.read()
        self.feed_values()
        self.assertTrue(self.panel.ready)

    def start_write(self):
        self.read()
        self.panel.edits = {0: "120", 1: "500"}
        self.panel.send()
        return self.panel.sequence

    def test_read_requires_complete_directory(self):
        self.panel.read()
        seq = self.panel.sequence
        self.panel.receive(f"{{RSP,CFG,GET,VALUE,{seq},0,高度,100,0,175,0}}")
        self.panel.receive(f"{{RSP,CFG,GET,END,{seq},2}}")
        self.assertFalse(self.panel.ready)
        self.assertIn("不完整", self.panel.status.get())

    def test_node_motion_parameters_write_and_readback(self):
        def feed(speed, accel):
            seq = self.panel.sequence
            self.panel.receive(f"{{RSP,CFG,GET,VALUE,{seq},0,底盘/节点导航速度(mm/s),{speed},10,1600,0}}")
            self.panel.receive(f"{{RSP,CFG,GET,VALUE,{seq},1,底盘/节点导航平移加速度(mm/s²),{accel},1,4000,0}}")
            self.panel.receive(f"{{RSP,CFG,GET,END,{seq},2}}")
        self.panel.read();feed(1200, 1600)
        self.assertTrue(self.panel.ready)
        self.assertEqual(self.panel.parameters[1].validate('450.5'), '450.5')
        self.assertEqual(self.panel.parameters[0].validate('1200'), '1200')
        self.assertEqual(self.panel.parameters[1].validate('4000'), '4000')
        for value in ('0', '4001', 'nan', 'inf'):
            with self.assertRaises(ValueError):
                self.panel.parameters[1].validate(value)
        self.panel.edits = {0: '300', 1: '450.5'}
        self.panel.send();seq = self.panel.sequence
        self.panel.receive(f'{{RSP,CFG,BEGIN,OK,{seq}}}')
        self.assertEqual(self.sent[-1], f'{{CMD,CFG,SET,{seq},0,300}}')
        self.panel.receive(f'{{RSP,CFG,SET,OK,{seq},0}}')
        self.assertEqual(self.sent[-1], f'{{CMD,CFG,SET,{seq},1,450.5}}')
        self.panel.receive(f'{{RSP,CFG,SET,OK,{seq},1}}')
        self.panel.receive(f'{{RSP,CFG,COMMIT,OK,{seq}}}')
        feed(300, 450.5)
        self.assertTrue(self.panel.ready)
        self.assertEqual(self.panel.edits, {})
        self.assertIn('回读确认', self.panel.status.get())

    def test_full_map_readback_can_preview_trial(self):
        self.panel.read()
        seq = self.panel.sequence
        expected = {}
        for node in range(25):
            row, column = divmod(node, 5)
            if row % 2:
                column = 4 - column
            expected[node] = (240 + column * 480, 240 + row * 480)
            for offset, axis in enumerate(("X", "Y")):
                index = node * 2 + offset
                self.panel.receive(f"{{RSP,CFG,GET,VALUE,{seq},{index},地图/节点{node}/{axis}(mm),{expected[node][offset]},0,2400,0}}")
        self.panel.receive(f"{{RSP,CFG,GET,END,{seq},50}}")
        self.assertTrue(self.panel.ready)
        app = UpperComputerApp.__new__(UpperComputerApp)
        app.parameter_panel = self.panel
        actual = app._trial_node_positions()
        self.assertEqual(actual, expected)
        plan = TrialPlan((0, 1, 8), (0, 120, 0), actual, TrialConfig())
        self.assertGreater(plan.length, 0)

    def test_batch_waits_for_each_ack_and_verifies(self):
        seq = self.start_write()
        self.assertEqual(self.sent[-1], f"{{CMD,CFG,BEGIN,{seq}}}")
        self.panel.receive(f"{{RSP,CFG,BEGIN,OK,{seq - 1}}}")
        self.assertEqual(self.sent[-1], f"{{CMD,CFG,BEGIN,{seq}}}")
        self.panel.receive(f"{{RSP,CFG,BEGIN,OK,{seq}}}")
        self.assertEqual(self.sent[-1], f"{{CMD,CFG,SET,{seq},0,120}}")
        self.panel.receive(f"{{RSP,CFG,SET,OK,{seq},1}}")  # 错序 ACK 不能推进队列。
        self.assertEqual(self.sent[-1], f"{{CMD,CFG,SET,{seq},0,120}}")
        self.panel.receive(f"{{RSP,CFG,SET,OK,{seq},0}}")
        self.assertEqual(self.sent[-1], f"{{CMD,CFG,SET,{seq},1,500}}")
        self.panel.receive(f"{{RSP,CFG,SET,OK,{seq},1}}")
        self.assertEqual(self.sent[-1], f"{{CMD,CFG,COMMIT,{seq}}}")
        self.panel.receive(f"{{RSP,CFG,COMMIT,OK,{seq}}}")
        self.assertEqual(self.sent[-1], f"{{CMD,CFG,GET,{seq + 1}}}")
        self.feed_values(("120", "500"))
        self.assertTrue(self.panel.ready)
        self.assertIn("回读确认", self.panel.status.get())
        self.assertFalse(self.panel.edits)

    def test_error_stops_without_commit(self):
        seq = self.start_write()
        self.panel.receive(f"{{RSP,CFG,BEGIN,OK,{seq}}}")
        self.panel.receive(f"{{RSP,CFG,SET,ERR,{seq},RANGE}}")
        self.assertFalse(any("COMMIT" in frame for frame in self.sent))
        self.assertFalse(self.panel.ready)
        self.assertFalse(self.panel.commands)

    def test_disconnect_ignores_late_ack(self):
        seq = self.start_write()
        previous = list(self.sent)
        self.panel.disconnected()
        self.panel.receive(f"{{RSP,CFG,BEGIN,OK,{seq}}}")
        self.assertEqual(previous, self.sent)
        self.assertFalse(self.panel.ready)

    def test_malformed_error_reply_stops_batch(self):
        seq = self.start_write()
        self.panel.receive(f"{{RSP,CFG,SET,ERR,{seq}}}")
        self.assertFalse(self.panel.ready)
        self.assertFalse(self.panel.commands)

    def test_readback_mismatch_not_reported_as_success(self):
        self.read()
        self.panel.edits = {0: "120"}
        self.panel.read(verify=True)
        self.feed_values()
        self.assertFalse(self.panel.ready)
        self.assertIn("不一致", self.panel.status.get())

    def test_nan_can_be_calibrated(self):
        self.panel.read()
        self.feed_values(("nan", "1000"))
        self.panel.tree.selection_set("0")
        self.panel.select()
        self.panel.value.set("120")
        self.assertTrue(self.panel.edit())
        self.assertEqual(self.panel.edits[0], "120")

    def test_run_waits_for_verified_parameters(self):
        called = []
        self.panel.run_main = lambda: called.append("release")
        self.read()
        self.panel.tree.selection_set("0")
        self.panel.select()
        self.panel.value.set("120")
        self.panel.send_and_run()
        seq = self.panel.sequence
        self.assertEqual(called, [])
        self.panel.receive(f"{{RSP,CFG,BEGIN,OK,{seq}}}")
        self.panel.receive(f"{{RSP,CFG,SET,OK,{seq},0}}")
        self.panel.receive(f"{{RSP,CFG,COMMIT,OK,{seq}}}")
        self.assertEqual(called, [])
        self.feed_values(("120", "1000"))
        self.assertEqual(called, ["release"])

    def test_failed_write_does_not_run(self):
        called = []
        self.panel.run_main = lambda: called.append("release")
        self.read()
        self.panel.tree.selection_set("0")
        self.panel.select()
        self.panel.value.set("120")
        self.panel.send_and_run()
        seq = self.panel.sequence
        self.panel.receive(f"{{RSP,CFG,BEGIN,OK,{seq}}}")
        self.panel.receive(f"{{RSP,CFG,SET,ERR,{seq},RANGE}}")
        self.assertEqual(called, [])

    def test_save_defaults_requires_confirmed_values(self):
        self.read()
        self.panel.tree.selection_set("0")
        self.panel.select()
        self.panel.value.set("120")
        with patch("upper_computer.export_config") as export:
            self.panel.save_defaults()
            export.assert_not_called()
            self.assertIn("待发送", self.panel.status.get())
            self.panel.edits.clear()
            self.panel.value.set("100")
            self.panel.save_defaults()
            export.assert_called_once()

    def test_test_action_waits_for_parameter_readback(self):
        self.read()
        self.panel.tree.selection_set("0")
        self.panel.select()
        # 只保留本测试固件回读中的高度与速度两个参数。
        self.panel.parameters[0].name = "搬运/圆盘/高度(mm)"
        self.panel.parameters[1].name = "搬运/速度"
        self.panel.value.set("120")
        self.panel.test_action.set("试选中机械臂轴")
        self.panel.apply_and_test()
        seq = self.panel.sequence
        self.assertNotIn("ARM,MOVE1", " ".join(self.sent))
        self.panel.receive(f"{{RSP,CFG,BEGIN,OK,{seq}}}")
        self.panel.receive(f"{{RSP,CFG,SET,OK,{seq},0}}")
        self.panel.receive(f"{{RSP,CFG,COMMIT,OK,{seq}}}")
        self.assertNotIn("ARM,MOVE1", " ".join(self.sent))
        self.panel.receive(f"{{RSP,CFG,GET,VALUE,{seq+1},0,搬运/圆盘/高度(mm),120,0,175,0}}")
        self.panel.receive(f"{{RSP,CFG,GET,VALUE,{seq+1},1,搬运/速度,1000,1,1000,1}}")
        self.panel.receive(f"{{RSP,CFG,GET,END,{seq+1},2}}")
        self.assertEqual(self.sent[-1], "{CMD,ARM,MOVE1,120,-1,1000}")
        self.assertIsNotNone(self.panel.test_running)
        self.panel.receive_test_result("{EVT,ARM,MOVE1,ISSUED}")
        self.assertIsNone(self.panel.test_running)

    def test_failed_parameter_write_does_not_test_motion(self):
        self.read()
        self.panel.parameters[0].name = "搬运/圆盘/高度(mm)"
        self.panel.parameters[1].name = "搬运/速度"
        self.panel.tree.selection_set("0")
        self.panel.select()
        self.panel.value.set("120")
        self.panel.test_action.set("试选中机械臂轴")
        self.panel.apply_and_test()
        seq = self.panel.sequence
        self.panel.receive(f"{{RSP,CFG,BEGIN,OK,{seq}}}")
        self.panel.receive(f"{{RSP,CFG,SET,ERR,{seq},RANGE}}")
        self.assertNotIn("ARM,MOVE1", " ".join(self.sent))


class ActionGroupPanelTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.sent = []
        self.controller = ParameterPanel(self.root, self.sent.append, lambda *_: None)
        self.panel = ActionGroupPanel(self.root, self.controller)
        self.defaults = load_action_group_delays()

    def tearDown(self):
        self.controller.cancel_timer()
        self.root.destroy()

    def feed(self, edits=None, old_firmware=False):
        seq = self.controller.sequence
        self.controller.receive(f"{{RSP,CFG,GET,VALUE,{seq},0,搬运/速度,300,1,300,0}}")
        values = {} if old_firmware else self.defaults | (edits or {})
        for index, (name, value) in enumerate(values.items(), 1):
            self.controller.receive(f"{{RSP,CFG,GET,VALUE,{seq},{index},{name},{value},0,60000,1}}")
        self.controller.receive(f"{{RSP,CFG,GET,END,{seq},{len(values)+1}}}")
        self.root.update()

    def read(self):
        self.controller.read()
        self.feed()

    def choose(self, group):
        self.panel.group.set(group)
        self.panel.change_group()
        self.root.update()

    def enter(self, step, value):
        name = f"{DELAY_PREFIX}{self.panel.group.get()}/{step}(ms)"
        self.panel.tree.selection_set(name)
        self.root.update()
        self.panel.value.set(str(value))
        return next(p for p in self.controller.parameters.values() if p.name == name)

    def confirm_write(self):
        while self.controller.mode == "write":
            seq = self.controller.sequence
            action, *args = self.controller.expected.split(",")
            suffix = "," + args[0] if args else ""
            self.controller.receive(f"{{RSP,CFG,{action},OK,{seq}{suffix}}}")

    def test_new_motion_labels_preserve_parameter_keys_and_other_groups(self):
        labels = {"取料抬升": "抬升后等待（缩回前）", "放料平移": "缩回后等待",
                  "放料稳定": "转向前稳定", "放料下降": "伸出并下降"}
        for group in TRANSFER_GROUPS[1:]:
            self.choose(group)
            self.assertEqual(len(self.panel.tree.get_children()), 12)
            for old, new in labels.items():
                name = f"{DELAY_PREFIX}{group}/{old}(ms)"
                self.assertEqual(self.panel.tree.set(name, "step"), new)
                self.assertEqual(self.panel.tree.set(name, "source"), str(self.defaults[name]))
            self.assertIn("缩回到 0", self.panel.note.get())
            self.assertIn("抬升 → 等待 → 缩回到 0", self.panel.note.get())
        for group in ("公共", "圆盘", "扫码准备1", "扫码准备2"):
            self.choose(group)
            for name in self.panel.tree.get_children():
                self.assertEqual(self.panel.tree.set(name, "step"), name.rsplit("/", 1)[-1].removesuffix("(ms)"))
        self.assertEqual(load_action_group_delays(), self.defaults)

    def test_routes_are_independent_and_group_switch_preserves_edits(self):
        self.read()
        first = self.enter("放料下降", 100)
        self.choose("载物台1到工作区2")
        self.assertEqual(self.controller.edits[first.index], "100")
        second = self.enter("放料下降", 200)
        self.assertTrue(self.panel.edit())
        self.assertEqual(self.controller.edits, {first.index: "100", second.index: "200"})
        self.choose("载物台1到工作区1")
        self.assertEqual(self.panel.tree.set(first.name, "edit"), "100")

    def test_trial_waits_for_readback_and_matching_result(self):
        self.read()
        self.choose("载物台2到工作区3")
        p = self.enter("放料下降", 100)
        self.panel.trial()
        self.assertEqual(self.controller.mode, "write")
        self.assertFalse(any("ARM,TRANSFER" in line for line in self.sent))
        self.confirm_write()
        self.assertEqual(self.controller.mode, "read")
        self.assertFalse(any("ARM,TRANSFER" in line for line in self.sent))
        self.feed({p.name: 100})
        self.assertEqual(self.sent[-1], "{CMD,ARM,TRANSFER,2,3}")
        self.assertEqual(str(self.panel.save_button["state"]), "disabled")
        self.controller.receive_test_result("{EVT,ARM,TRANSFER,DONE,1,1}")
        self.assertIsNotNone(self.controller.test_running)
        self.controller.receive_test_result("{EVT,ARM,TRANSFER,DONE,2,3}")
        self.assertIsNone(self.controller.test_running)
        self.assertEqual(str(self.panel.save_button["state"]), "normal")

    def test_readback_failure_prevents_trial_and_saving(self):
        self.read()
        self.enter("放料下降", 100)
        self.panel.trial()
        self.confirm_write()
        self.feed()  # 小车仍返回旧值。
        self.assertFalse(self.controller.ready)
        self.assertFalse(any("ARM,TRANSFER" in line for line in self.sent))
        with patch("upper_computer.save_action_group_delays") as save:
            self.panel.save_to_cpp()
            save.assert_not_called()

    def test_save_requires_confirmed_values_and_saves_selected_or_all(self):
        self.read()
        p = self.enter("放料下降", 100)
        with patch("upper_computer.save_action_group_delays", return_value=12) as save:
            self.panel.save_to_cpp()
            save.assert_not_called()
            self.panel.send()
            self.confirm_write()
            self.feed({p.name: 100})
            self.panel.save_to_cpp()
            selected = save.call_args.args[0]
            self.assertEqual(len(selected), 12)
            self.assertEqual(selected[p.name], "100")
            self.assertTrue(all("载物台1到工作区1/" in name for name in selected))
            self.panel.save_to_cpp(True)
            self.assertEqual(set(save.call_args.args[0]), set(DELAY_NAMES))

    def test_hidden_parameter_editor_cannot_restore_old_delay(self):
        self.read()
        p = self.enter("放料下降", 100)
        self.controller.tree.selection_set(str(p.index))
        self.controller.select()
        self.panel.send()
        self.controller.receive(f"{{RSP,CFG,BEGIN,OK,{self.controller.sequence}}}")
        self.assertEqual(self.sent[-1], f"{{CMD,CFG,SET,{self.controller.sequence},{p.index},100}}")

    def test_invalid_wait_and_common_group_cannot_trial(self):
        self.read()
        for value in (-1, 60001, 0.5, "nan"):
            self.enter("放料下降", value)
            with patch("upper_computer.messagebox.showerror") as error:
                self.assertFalse(self.panel.edit())
                error.assert_called_once()
        self.enter("放料下降", 0)
        self.assertTrue(self.panel.edit())
        self.choose("公共")
        self.assertEqual(str(self.panel.trial_button["state"]), "disabled")
        previous = list(self.sent)
        self.panel.trial()
        self.assertEqual(self.sent, previous)

    def test_old_firmware_and_disconnect_disable_writes(self):
        self.controller.read()
        self.feed(old_firmware=True)
        self.assertEqual(str(self.panel.trial_button["state"]), "disabled")
        self.assertEqual(str(self.panel.save_button["state"]), "disabled")
        self.assertIn("新版固件", self.panel.note.get())
        self.controller.disconnected()
        with patch("upper_computer.save_action_group_delays") as save:
            self.panel.save_to_cpp(True)
            save.assert_not_called()


if __name__ == "__main__":
    unittest.main()
