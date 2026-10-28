"""EMM 参数封包和实际回读显示。"""
import unittest
from unittest.mock import Mock
from emm_panel import EmmPanel, build_emm_command
from upper_computer import parse_frame


class EmmPanelTests(unittest.TestCase):
    def test_drive_commands_and_window_units(self):
        self.assertEqual(build_emm_command("DRIVE_GET", 7), "{CMD,EMM,DRIVE_GET,7}")
        self.assertEqual(build_emm_command("DRIVE_SET", 7, 5000, 1000, 2400, 8, 2200, 2000, "0.8", 0),
                         "{CMD,EMM,DRIVE_SET,7,5000,1000,2400,8,2200,2000,8,0}")
        for window in ("0.08", "NaN", "inf", "-0.1", "6553.6"):
            with self.assertRaises(ValueError):
                build_emm_command("DRIVE_SET", 7, 5000, 1000, 2400, 8, 2200, 2000, window, 0)

    def test_drive_readback_matches_selected_motor(self):
        panel = EmmPanel.__new__(EmmPanel)
        panel.parse = parse_frame; panel.tuning = Mock()
        panel.motor = Mock(); panel.motor.get.return_value = "7"
        panel.drive_values = [Mock() for _ in range(7)]
        panel.drive_status = Mock()
        panel.status = Mock()
        panel.receive("{RSP,EMM,DRIVE_GET,READBACK,1,5000,1000,2400,8,2200,2000,8}")
        panel.drive_values[0].set.assert_not_called()
        panel.receive("{RSP,EMM,DRIVE_SET,READBACK,7,5000,1000,2400,8,2200,2000,8}")
        for variable, value in zip(panel.drive_values, ("5000", "1000", "2400", "8", "2200", "2000", "0.8")):
            variable.set.assert_called_once_with(value)

    def test_drive_mismatch_displays_actual_values(self):
        panel = EmmPanel.__new__(EmmPanel)
        panel.parse = parse_frame; panel.tuning = Mock()
        panel.motor = Mock(); panel.motor.get.return_value = "7"
        panel.drive_values = [Mock() for _ in range(7)]
        panel.drive_status = Mock(); panel.status = Mock()
        panel.receive("{RSP,EMM,DRIVE_SET,READBACK_MISMATCH,7,5000,4000,4900,8,5000,2000,8}")
        panel.drive_values[0].set.assert_called_once_with("5000")
        panel.drive_values[4].set.assert_called_once_with("5000")
        self.assertIn("与请求不一致", panel.drive_status.set.call_args.args[0])

    def test_trial_ack_has_no_motor_id(self):
        panel = EmmPanel.__new__(EmmPanel)
        panel.parse = parse_frame
        panel.tuning = Mock()
        panel.status = Mock()
        # 实机返回四字段ACK，不能进入要求第5字段为地址的通用处理。
        panel.receive("{RSP,EMM,TURRET_TRIAL,ACK}")
        panel.tuning.receive.assert_called_once_with(("RSP", "EMM", "TURRET_TRIAL", "ACK"))
        panel.status.set.assert_not_called()

    def test_commands(self):
        self.assertEqual(build_emm_command("PID_SET", "7", "4294967295", "0", "62000", 1),
                         "{CMD,EMM,PID_SET,7,4294967295,0,62000,1}")
        self.assertEqual(build_emm_command("MODE_SET", 5, 3, 0), "{CMD,EMM,MODE_SET,5,3,0}")
        for args in (("PID_GET", 0), ("PID_SET", 1, -1, 0, 0, 0),
                     ("PID_SET", 1, 4294967296, 0, 0, 0), ("PID_GET", 1, 2),
                     ("ENABLE", 1, "1.5")):
            with self.assertRaises(ValueError):
                build_emm_command(*args)

    def test_readback_matches_motor(self):
        panel = EmmPanel.__new__(EmmPanel)
        panel.parse = parse_frame
        panel.tuning = Mock()
        panel.motor = Mock(); panel.motor.get.return_value = "7"
        panel.gains = [Mock() for _ in range(3)]
        panel.status = Mock()
        panel.receive("{RSP,EMM,PID_SET,READBACK,5,10,20,30}")
        for value in panel.gains:
            value.set.assert_not_called()
        panel.receive("{RSP,EMM,PID_SET,READBACK,7,10,20,30}")
        for variable, value in zip(panel.gains, ("10", "20", "30")):
            variable.set.assert_called_once_with(value)
        panel.receive("{RSP,EMM,PID_SET,ERR,MOTOR_FEEDBACK_TIMEOUT}")
        self.assertIn("MOTOR_FEEDBACK_TIMEOUT", panel.status.set.call_args.args[0])


if __name__ == "__main__":
    unittest.main()
