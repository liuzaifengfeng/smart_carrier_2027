"""独立视觉死区的串口及上位机行为测试。"""
import unittest
from unittest.mock import Mock

from upper_computer import (
    build_alignment_control_command, VISION_START_REQUEST_LABELS, UpperComputerApp,
)


class VisionAlignmentTests(unittest.TestCase):
    def test_busy_response_explains_automatic_state_is_preserved(self):
        app = UpperComputerApp.__new__(UpperComputerApp)
        app.vision_request_var = Mock()
        self.assertTrue(app._accept_vision_start_request("{RSP,VISION,ALIGN_START,ERR,BUSY}"))
        message = app.vision_request_var.set.call_args.args[0]
        self.assertIn("被拒绝", message)
        self.assertIn("未改变对齐状态", message)

    def test_explicit_mode_and_legacy_commands(self):
        self.assertEqual(build_alignment_control_command("START"), "{CMD,VISION,ALIGN_START}")
        self.assertEqual(build_alignment_control_command("stop"), "{CMD,VISION,ALIGN_STOP}")
        for mode in VISION_START_REQUEST_LABELS:
            self.assertEqual(build_alignment_control_command("start", mode),
                             f"{{CMD,VISION,ALIGN_START,{mode}}}")
        for action, mode in (("STOP", "DISC"), ("START", "NONE"), ("START", ""), ("OTHER", None)):
            with self.subTest(action=action, mode=mode), self.assertRaises(ValueError):
                build_alignment_control_command(action, mode)

    def test_ui_uses_selected_deadzone_and_stop_has_no_mode(self):
        app = UpperComputerApp.__new__(UpperComputerApp)
        app.alignment_mode_var = Mock()
        app.serial_link = Mock()
        app.append_log = Mock()
        app.status_var = Mock()
        for mode in VISION_START_REQUEST_LABELS:
            app.alignment_mode_var.get.return_value = mode
            app.send_alignment_control("START")
            app.serial_link.send_line.assert_called_with(f"{{CMD,VISION,ALIGN_START,{mode}}}")
        app.send_alignment_control("STOP")
        app.serial_link.send_line.assert_called_with("{CMD,VISION,ALIGN_STOP}")
