"""无硬件测试：校验参数边界、串口确认顺序、失败中止和回读确认。"""
import tkinter as tk
import unittest

from parameter_panel import Parameter, ParameterPanel, parse_parameter


class ValidationTests(unittest.TestCase):
    def test_float_limits_and_nonfinite(self):
        p = Parameter(0, "高度", "100", 0, 160, False)
        self.assertEqual(p.validate("160"), "160")
        for value in ("161", "-1", "nan", "inf", "abc"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                p.validate(value)

    def test_integer_and_uncalibrated_read(self):
        p = Parameter(0, "等待", "1000", 1, 60000, True)
        self.assertEqual(p.validate("1.0"), "1")
        with self.assertRaises(ValueError):
            p.validate("1.5")
        p = parse_parameter(["VALUE", "1", "0", "高度", "nan", "0", "160", "0"])
        self.assertEqual(p.value, "nan")
        with self.assertRaises(ValueError):
            parse_parameter(["VALUE", "1", "0", "高度", "1", "160", "0", "0"])


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
        self.panel.receive(f"{{CFG:VALUE,{seq},0,搬运/高度,{values[0]},0,160,0}}")
        self.panel.receive(f"{{CFG:VALUE,{seq},1,搬运/等待,{values[1]},1,60000,1}}")
        self.panel.receive(f"{{CFG:END,{seq},2}}")

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
        self.panel.receive(f"{{CFG:VALUE,{seq},0,高度,100,0,160,0}}")
        self.panel.receive(f"{{CFG:END,{seq},2}}")
        self.assertFalse(self.panel.ready)
        self.assertIn("不完整", self.panel.status.get())

    def test_batch_waits_for_each_ack_and_verifies(self):
        seq = self.start_write()
        self.assertEqual(self.sent[-1], f"{{CFG:BEGIN,{seq}}}")
        self.panel.receive(f"{{CFG:OK,{seq - 1},BEGIN}}")
        self.assertEqual(self.sent[-1], f"{{CFG:BEGIN,{seq}}}")
        self.panel.receive(f"{{CFG:OK,{seq},BEGIN}}")
        self.assertEqual(self.sent[-1], f"{{CFG:SET,{seq},0,120}}")
        self.panel.receive(f"{{CFG:OK,{seq},SET,1}}")  # 错序 ACK 不能推进队列。
        self.assertEqual(self.sent[-1], f"{{CFG:SET,{seq},0,120}}")
        self.panel.receive(f"{{CFG:OK,{seq},SET,0}}")
        self.assertEqual(self.sent[-1], f"{{CFG:SET,{seq},1,500}}")
        self.panel.receive(f"{{CFG:OK,{seq},SET,1}}")
        self.assertEqual(self.sent[-1], f"{{CFG:COMMIT,{seq}}}")
        self.panel.receive(f"{{CFG:OK,{seq},COMMIT}}")
        self.assertEqual(self.sent[-1], f"{{CFG:GET,{seq + 1}}}")
        self.feed_values(("120", "500"))
        self.assertTrue(self.panel.ready)
        self.assertIn("回读确认", self.panel.status.get())
        self.assertFalse(self.panel.edits)

    def test_error_stops_without_commit(self):
        seq = self.start_write()
        self.panel.receive(f"{{CFG:OK,{seq},BEGIN}}")
        self.panel.receive(f"{{CFG:ERR,{seq},RANGE}}")
        self.assertFalse(any("COMMIT" in frame for frame in self.sent))
        self.assertFalse(self.panel.ready)
        self.assertFalse(self.panel.commands)

    def test_disconnect_ignores_late_ack(self):
        seq = self.start_write()
        previous = list(self.sent)
        self.panel.disconnected()
        self.panel.receive(f"{{CFG:OK,{seq},BEGIN}}")
        self.assertEqual(previous, self.sent)
        self.assertFalse(self.panel.ready)

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


if __name__ == "__main__":
    unittest.main()
