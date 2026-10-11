"""无硬件测试：校验参数边界、串口确认顺序、失败中止和回读确认。"""
import tkinter as tk
import unittest
from unittest.mock import patch

from upper_computer import (Parameter, ParameterPanel, parse_parameter,
                            parameter_matches_stage, build_parameter_test_command)


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
        self.assertTrue(parameter_matches_stage("底盘/节点导航轮速(RPM)", "圆盘抓取（两轮）"))

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


if __name__ == "__main__":
    unittest.main()
