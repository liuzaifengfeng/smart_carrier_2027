"""无硬件验证键盘微调的启动、停止和模式切换。"""

import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from upper_computer import UpperComputerApp, build_debug_command


class KeyboardControlTests(unittest.TestCase):
    def setUp(self):
        self.app = UpperComputerApp.__new__(UpperComputerApp)
        self.app.serial_link = Mock(is_open=True)
        self.app.append_log = Mock()
        self.app.keyboard_drive_status_var = Mock()
        self.app.command_vars = {"Movepose": [Mock(), Mock(), Mock()]}
        self.app.command_vars["Movepose"][1].get.return_value = "80"
        self.app.pressed_drive_keys = []
        self.app.pressed_rotation_keys = set()
        self.app.active_drive_key = None
        self.app.pending_target = None

    def press(self, key, shift=False):
        self.app._on_keyboard_key_press(SimpleNamespace(
            keysym=key, state=1 if shift else 0, widget=None))

    def release(self, key):
        self.app._on_keyboard_key_release(SimpleNamespace(keysym=key))

    def lines(self):
        return [call.args[0] for call in self.app.serial_link.send_line.call_args_list]

    def test_both_fine_directions_and_repeat(self):
        for key, direction in (("Q", 4), ("E", 5)):
            with self.subTest(key=key):
                self.app.serial_link.send_line.reset_mock()
                self.press(key, shift=True)
                self.press(key, shift=True)
                self.release(key)
                self.assertEqual(self.lines(), [
                    f"{{CMD,CHASSIS,MOVE,{direction},80,0}}",
                    f"{{CMD,CHASSIS,MOVE,{direction},80,1}}"])
                self.assertIsNone(self.app.active_drive_key)

    def test_plain_rotation_stays_90_degrees(self):
        self.press("q")
        self.press("q")
        self.release("q")
        self.press("e")
        self.release("e")
        self.assertEqual(self.lines(), ["{CMD,POSE,GOTO_REL,0,0,90}",
                                       "{CMD,POSE,GOTO_REL,0,0,-90}"])

    def test_shift_release_stops_without_starting_90_degree_turn(self):
        for shift in ("Shift_L", "Shift_R"):
            with self.subTest(shift=shift):
                self.app.serial_link.send_line.reset_mock()
                self.press("q", shift=True)
                self.release(shift)
                self.press("q")  # 松开 Shift 后系统连发 Q。
                self.release("q")
                self.assertEqual(self.lines(), ["{CMD,CHASSIS,MOVE,4,80,0}",
                                               "{CMD,CHASSIS,MOVE,4,80,1}"])

    def test_restore_held_translation(self):
        self.press("w")
        self.press("e", shift=True)
        self.release("e")
        self.release("w")
        self.assertEqual(self.lines(), ["{CMD,CHASSIS,MOVE,0,80,0}",
                                       "{CMD,CHASSIS,MOVE,0,80,1}",
                                       "{CMD,CHASSIS,MOVE,5,80,0}",
                                       "{CMD,CHASSIS,MOVE,5,80,1}",
                                       "{CMD,CHASSIS,MOVE,0,80,0}",
                                       "{CMD,CHASSIS,MOVE,0,80,1}"])

    def test_shift_release_clears_both_rotation_keys(self):
        self.press("q", shift=True)
        self.press("e", shift=True)
        self.app.serial_link.send_line.reset_mock()
        self.release("Shift_L")
        self.assertEqual(self.lines(), ["{CMD,CHASSIS,MOVE,5,80,1}"])
        self.assertEqual(self.app.pressed_drive_keys, [])

    def test_focus_loss_stops(self):
        self.app.root = Mock()
        self.app.root.focus_displayof.return_value = None
        self.press("q", shift=True)
        self.app._stop_keyboard_drive_if_unfocused()
        self.assertEqual(self.lines()[-1], "{CMD,CHASSIS,MOVE,4,80,1}")
        self.assertEqual(self.app.pressed_rotation_keys, set())

    def test_send_failure_does_not_leave_active_motion(self):
        self.app.serial_link.send_line.side_effect = RuntimeError("disconnected")
        self.press("e", shift=True)
        self.assertIsNone(self.app.active_drive_key)
        self.assertEqual(self.app.pressed_drive_keys, [])
        self.release("e")

    def test_direction_range_rejects_unknown_values(self):
        for direction in ("-1", "6", "4.5"):
            with self.subTest(direction=direction), self.assertRaises(ValueError):
                build_debug_command("Movepose", [direction, "80", "0"])


if __name__ == "__main__":
    unittest.main()
