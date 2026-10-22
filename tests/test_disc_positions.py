"""位置快照的封包、界面回显及固件实际处理函数，无需硬件。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import Mock

from upper_computer import (build_disc_positions_command,
                            parse_disc_positions_response, UpperComputerApp)

ROOT = Path(__file__).resolve().parents[1]


class DiscPositionsTests(unittest.TestCase):
    def test_each_completed_grab_restarts_position_recognition(self):
        from native_support import build_native, run_native
        run_native(build_native("executor", ["tests/motion_executor_native.cpp"]), "rounds")

    def test_look3_requires_measured_turret_within_five_degrees(self):
        from native_support import run_native
        from test_motion_hardware import build_motion_hardware
        run_native(build_motion_hardware(), "look3")

    def test_build_and_reject_invalid_candidates(self):
        self.assertEqual(build_disc_positions_command([(2, 1)]),
                         "{CMD,VISION,DISC_POSITIONS,2,1}")
        self.assertEqual(build_disc_positions_command([(1, 5), (2, 1), (3, 6)]),
                         "{CMD,VISION,DISC_POSITIONS,1,5,2,1,3,6}")
        for candidates in ([], [(1, 1)] * 4, [(1, 1), (1, 2)], [(0, 1)],
                           [(4, 1)], [(True, 1)], [(1, True)], [(1, 0)],
                           [(1, 7)], [(1, "2")], [(1.5, 1)]):
            with self.subTest(candidates=candidates), self.assertRaises(ValueError):
                build_disc_positions_command(candidates)

    def test_replies_and_ui_report_real_completion(self):
        app = UpperComputerApp.__new__(UpperComputerApp)
        app.disc_material_status = Mock()
        app.serial_link = Mock()
        app.append_log = Mock()
        app.disc_position_vars = [Mock(), Mock(), Mock()]
        for variable, value in zip(app.disc_position_vars, ("1 红色", "不报送", "6 浅蓝色")):
            variable.get.return_value = value
        app.send_disc_positions()
        app.serial_link.send_line.assert_called_once_with("{CMD,VISION,DISC_POSITIONS,1,1,3,6}")
        for line, message in (
            ("{RSP,VISION,DISC_POSITIONS,ACK,2}", "等待选择"),
            ("{EVT,VISION,DISC_POSITIONS,SELECTED,3,6,1}", "圆盘位置 3"),
            ("{EVT,VISION,DISC_POSITIONS,DONE,3,6,1}", "抓放完成"),
            ("{EVT,VISION,DISC_POSITIONS,FAILED,3,6,1}", "抓放失败"),
            ("{EVT,VISION,DISC_POSITIONS,SKIP,1,1,COLOR_MISMATCH}", "等待新位置"),
            ("{RSP,VISION,DISC_POSITIONS,ERR,NOT_READY}", "被拒绝"),
        ):
            self.assertTrue(app._accept_disc_material_response(line))
            self.assertIn(message, app.disc_material_status.set.call_args.args[0])
        for line in ("{RSP,VISION,DISC_POSITIONS,DONE,3,6,1}",
                     "{EVT,VISION,DISC_POSITIONS,DONE,0,6,1}",
                     "{EVT,VISION,DISC_POSITIONS,DONE,3,7,1}",
                     "{EVT,VISION,DISC_POSITIONS,DONE,3,6,4}",
                     "{RSP,VISION,DISC_POSITIONS,ACK,4}",
                     "{EVT,VISION,DISC_POSITIONS,SELECTED,3,6}"):
            self.assertIsNone(parse_disc_positions_response(line))
        app.serial_link.send_line.side_effect = RuntimeError("disconnected")
        app.send_disc_positions()
        self.assertIn("未发送", app.disc_material_status.set.call_args.args[0])

    def test_actual_firmware_handler_and_position_actions(self):
        from native_support import run_native
        from test_serial_motion import build_serial_motion
        from test_arm_motion import build_arm_motion
        run_native(build_serial_motion(), "positions")
        run_native(build_arm_motion(), "discs")


if __name__ == "__main__":
    unittest.main()
