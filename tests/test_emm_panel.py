"""EMM 参数封包和实际回读显示。"""
import unittest
from unittest.mock import Mock
from emm_panel import EmmPanel, build_emm_command
from upper_computer import parse_frame


class EmmPanelTests(unittest.TestCase):
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
