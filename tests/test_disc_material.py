"""圆盘颜色/抓取协议及上位机结果区分，无需硬件。"""
import unittest
from unittest.mock import Mock

from upper_computer import (
    build_disc_color_command, build_disc_grab_command,
    parse_disc_material_response, UpperComputerApp,
    build_force_disc_grab_command,
)


class DiscMaterialTests(unittest.TestCase):
    def test_color_and_grab_packets(self):
        for color in range(1, 7):
            self.assertEqual(build_disc_color_command(color), f"{{CMD,VISION,COLOR,{color}}}")
            for cargo in range(1, 4):
                self.assertEqual(build_disc_grab_command(cargo, color),
                                 f"{{CMD,VISION,GRAB,{color},{cargo}}}")
        for cargo in range(1, 4):
            self.assertEqual(build_disc_grab_command(cargo), f"{{CMD,VISION,GRAB,{cargo}}}")

    def test_invalid_color_and_cargo(self):
        for color in (0, 7, -1, 1.5, float("nan"), "1", True):
            with self.subTest(color=color), self.assertRaises(ValueError):
                build_disc_color_command(color)
            with self.subTest(color=color), self.assertRaises(ValueError):
                build_disc_grab_command(1, color)
        for cargo in (0, 4, -1, 1.5, "1", True):
            with self.subTest(cargo=cargo), self.assertRaises(ValueError):
                build_disc_grab_command(cargo, 1)

    def test_ack_and_terminal_results(self):
        for status, kind in (("ACK", "RSP"), ("DONE", "EVT"), ("FAILED", "EVT")):
            self.assertEqual(parse_disc_material_response(f"{{{kind},VISION,GRAB,{status},6,3}}"),
                             ("GRAB", status, 6, 3, None))
        self.assertEqual(parse_disc_material_response("{RSP,VISION,COLOR,OK}"),
                         ("COLOR", "OK", None, None, None))
        for reason in ("NOT_ACTIVE", "ALIGN_ACTIVE", "NO_COLOR", "ORDER", "COLOR_MISMATCH", "OCCUPIED"):
            self.assertEqual(parse_disc_material_response(f"{{RSP,VISION,GRAB,ERR,{reason}}}")[-1], reason)

    def test_invalid_results_are_not_completion(self):
        for line in ("{EVT,VISION,GRAB,DONE}", "{RSP,VISION,GRAB,DONE,1,1}",
                     "{EVT,VISION,GRAB,ACK,1,1}", "{EVT,VISION,GRAB,DONE,0,1}",
                     "{EVT,VISION,GRAB,DONE,1,4}", "{EVT,VISION,GRAB,DONE,1.0,1}",
                     "{EVT,VISION,GRAB,DONE,1,1,extra}", "{EVT,VISION,COLOR,OK}"):
            with self.subTest(line=line):
                self.assertIsNone(parse_disc_material_response(line))

    def make_app(self):
        app = UpperComputerApp.__new__(UpperComputerApp)
        app.disc_color_var = Mock()
        app.disc_color_var.get.return_value = "6 浅蓝色"
        app.disc_cargo_var = Mock()
        app.disc_cargo_var.get.return_value = "3"
        app.serial_link = Mock()
        app.append_log = Mock()
        app.disc_material_status = Mock()
        return app

    def test_ui_sends_atomic_grab_and_connection_failure(self):
        app = self.make_app()
        app.send_disc_material(False)
        app.serial_link.send_line.assert_called_with("{CMD,VISION,COLOR,6}")
        app.send_disc_material(True)
        app.serial_link.send_line.assert_called_with("{CMD,VISION,GRAB,6,3}")
        app.serial_link.send_line.side_effect = RuntimeError("disconnected")
        app.send_disc_material(True)
        self.assertIn("未发送", app.disc_material_status.set.call_args.args[0])

    def test_ui_never_treats_ack_as_grab_success(self):
        app = self.make_app()
        for line, expected in (("{RSP,VISION,GRAB,ACK,1,1}", "等待动作结果"),
                               ("{EVT,VISION,GRAB,DONE,1,1}", "现场确认"),
                               ("{EVT,VISION,GRAB,FAILED,1,1}", "失败"),
                               ("{RSP,VISION,GRAB,ERR,NO_COLOR}", "被拒绝")):
            self.assertTrue(app._accept_disc_material_response(line))
            self.assertIn(expected, app.disc_material_status.set.call_args.args[0])
        self.assertFalse(app._accept_disc_material_response("{EVT,VISION,GRAB,DONE}"))

    def test_color_decision_and_grab_return_are_not_completion(self):
        app = self.make_app()
        for line, expected in (("{EVT,VISION,COLOR,GRAB,1,1}", "抓取函数"),
                               ("{EVT,VISION,COLOR,SKIP,5,1}", "不抓取"),
                               ("{EVT,VISION,COLOR,SKIP,1,0}", "不抓取"),
                               ("{EVT,VISION,GRAB,REQUESTED,1,1}", "不代表成功")):
            self.assertTrue(app._accept_disc_material_response(line))
            self.assertIn(expected, app.disc_material_status.set.call_args.args[0])
        for line in ("{EVT,VISION,COLOR,GRAB,1,0}", "{RSP,VISION,COLOR,SKIP,1,1}",
                     "{EVT,VISION,COLOR,SKIP,7,1}", "{EVT,VISION,GRAB,REQUESTED,1,0}"):
            self.assertIsNone(parse_disc_material_response(line))

    def test_force_grab_explicit_parameters_and_ranges(self):
        self.assertEqual(build_force_disc_grab_command(1, 1), "{CMD,VISION,FORCE_GRAB,1,1}")
        for color, cargo in ((0, 1), (7, 1), (1, 0), (1, 4), (True, 1), (1, "1")):
            with self.subTest(color=color, cargo=cargo), self.assertRaises(ValueError):
                build_force_disc_grab_command(color, cargo)

    def test_color_block_reasons_and_mismatch_are_distinct(self):
        app = self.make_app()
        for reason, expected in (("NO_TASK", "任务码"), ("NOT_WAITING", "阶段"),
                                 ("ROUND_COMPLETE", "进度"), ("ALIGN_ACTIVE", "对齐"),
                                 ("NOT_READY", "定位"), ("OCCUPIED", "已有物料")):
            line = f"{{EVT,VISION,COLOR,BLOCKED,2,1,{reason}}}"
            with self.subTest(reason=reason):
                self.assertEqual(parse_disc_material_response(line),
                                 ("COLOR", "BLOCKED", 2, 1, reason))
                self.assertTrue(app._accept_disc_material_response(line))
                self.assertIn(expected, app.disc_material_status.set.call_args.args[0])
        self.assertTrue(app._accept_disc_material_response("{EVT,VISION,COLOR,SKIP,3,1,COLOR_MISMATCH}"))
        self.assertIn("不匹配", app.disc_material_status.set.call_args.args[0])
        self.assertEqual(parse_disc_material_response("{EVT,VISION,COLOR,BLOCKED,2,0,ROUND_COMPLETE}"),
                         ("COLOR", "BLOCKED", 2, 0, "ROUND_COMPLETE"))

    def test_invalid_color_decision_reasons_are_rejected(self):
        for line in ("{EVT,VISION,COLOR,SKIP,2,1,OCCUPIED}",
                     "{EVT,VISION,COLOR,BLOCKED,2,1,COLOR_MISMATCH}",
                     "{RSP,VISION,COLOR,BLOCKED,2,1,OCCUPIED}",
                     "{EVT,VISION,COLOR,BLOCKED,2,1,UNKNOWN}",
                     "{EVT,VISION,COLOR,BLOCKED,7,1,OCCUPIED}",
                     "{EVT,VISION,COLOR,BLOCKED,2,4,OCCUPIED}"):
            with self.subTest(line=line):
                self.assertIsNone(parse_disc_material_response(line))

    def test_force_button_and_return_do_not_claim_success(self):
        app = self.make_app()
        app.send_force_disc_grab()
        app.serial_link.send_line.assert_called_with("{CMD,VISION,FORCE_GRAB,6,3}")
        for line, expected in (("{RSP,VISION,FORCE_GRAB,ACK,6,3}", "等待函数调用返回"),
                               ("{EVT,VISION,FORCE_GRAB,REQUESTED,6,3}", "结果请查看物料日志"),
                               ("{RSP,VISION,FORCE_GRAB,ERR,RANGE}", "被拒绝")):
            self.assertTrue(app._accept_disc_material_response(line))
            self.assertIn(expected, app.disc_material_status.set.call_args.args[0])
        for line in ("{EVT,VISION,FORCE_GRAB,DONE,6,3}", "{EVT,VISION,FORCE_GRAB,REQUESTED,6,0}"):
            self.assertIsNone(parse_disc_material_response(line))
        app.serial_link.send_line.side_effect = RuntimeError("disconnected")
        app.send_force_disc_grab()
        self.assertIn("未发送", app.disc_material_status.set.call_args.args[0])

    def test_force_occupied_warning_remains_after_function_return(self):
        app = self.make_app()
        warning = "{EVT,VISION,FORCE_GRAB,WARN,OCCUPIED,2,3}"
        self.assertEqual(parse_disc_material_response(warning),
                         ("FORCE_GRAB", "WARN", 2, 3, "OCCUPIED"))
        self.assertTrue(app._accept_disc_material_response(warning))
        self.assertIn("忽略占用并继续", app.disc_material_status.set.call_args.args[0])
        app._accept_disc_material_response("{EVT,VISION,FORCE_GRAB,REQUESTED,6,3}")
        message = app.disc_material_status.set.call_args.args[0]
        self.assertIn("警告", message)
        self.assertIn("颜色 2", message)
        self.assertIn("颜色 6", message)
        app.send_force_disc_grab()
        self.assertEqual(app.force_disc_grab_warning, "")

    def test_invalid_occupied_warnings_are_rejected(self):
        for line in ("{EVT,VISION,FORCE_GRAB,WARN,OCCUPIED,2,0}",
                     "{EVT,VISION,FORCE_GRAB,WARN,OCCUPIED,7,3}",
                     "{RSP,VISION,FORCE_GRAB,WARN,OCCUPIED,2,3}",
                     "{EVT,VISION,FORCE_GRAB,WARN,OCCUPIED,2}",
                     "{EVT,VISION,FORCE_GRAB,WARN,OTHER,2,3}"):
            with self.subTest(line=line):
                self.assertIsNone(parse_disc_material_response(line))
