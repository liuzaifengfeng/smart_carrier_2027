import unittest
import csv
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import Mock, patch
from turret_tuning import TurnRecord, TurretTuningPanel, trial_settings
from upper_computer import parse_frame
from emm_panel import EmmPanel


class TuningTests(unittest.TestCase):
    def test_metrics_include_overshoot_and_full_observation(self):
        record = TurnRecord("1", 1000, 90, 0, {})
        record.samples = [(t, angle, 0, 3, 0, 0, 8) for t, angle in
                          ((0, 0), (100, 100), (200, 89), (400, 90.5), (800, 89.5), (1000, 90))]
        m = record.metrics()
        self.assertEqual(m["overshoot"], 10)
        self.assertEqual(m["settling_ms"], 200)
        self.assertEqual(m["tail_p2p"], 0.5)
        record.samples.append((1100, 92, 0, 3, 0, 0, 8))
        self.assertIsNone(record.metrics()["settling_ms"])
        record.start, record.target = 180, 90
        self.assertEqual(record.metrics()["overshoot"], 90)

    def make_panel(self):
        panel = TurretTuningPanel.__new__(TurretTuningPanel)
        panel.motor = Mock(); panel.motor.get.return_value = "7"
        panel.variables = {key: Mock() for key in ("a", "b", "speed", "ramp", "period", "duration", "cycles")}
        for key, value in zip(panel.variables, (0, 90, 250, 100, 20, 3000, 1)):
            panel.variables[key].get.return_value = str(value)
        panel.pid_values = [Mock() for _ in range(3)]
        for var, value in zip(panel.pid_values, (26000, 10, 39000)):
            var.get.return_value = str(value)
        panel.active = False; panel.records = []; panel.current = None
        panel.status = Mock(); panel.start_button = Mock(); panel.send = Mock(); panel.log = Mock()
        panel.draw = Mock(); panel.table = Mock()
        return panel

    def test_pid_verification_then_turns_then_done(self):
        panel = self.make_panel()
        emm = EmmPanel.__new__(EmmPanel)
        emm.parse = parse_frame; emm.tuning = panel
        emm.motor = panel.motor; emm.gains = panel.pid_values; emm.status = Mock()
        panel.start_trial()
        panel.send.assert_called_once_with("{CMD,EMM,PID_SET,7,26000,10,39000,0}")
        emm.receive("{RSP,EMM,PID_SET,READBACK,7,26000,10,39000}")
        self.assertEqual(panel.send.call_args.args[0], "{CMD,EMM,TURRET_TRIAL,0.0,250.0,100,20,3000}")
        for i, target in enumerate((0, 90, 0), 1):
            emm.receive("{RSP,EMM,TURRET_TRIAL,ACK}")
            emm.receive(f"{{EVT,TURRET,COMMAND,{i},1000,{target},0,42,208}}")
            emm.receive(f"{{EVT,TURRET,SAMPLE,{i},1020,{target},{target},0,3,0,123,-456,8}}")
            self.assertEqual(panel.current.samples[0][-3:], (123, -456, 8))
            emm.receive(f"{{EVT,TURRET,RESULT,{i},3000,OBSERVED,{target},{target},0}}")
            emm.receive("{EVT,EMM,TURRET_TRIAL,DONE}")
        self.assertFalse(panel.active)
        self.assertEqual(len(panel.records), 3)
        self.assertEqual(panel.send.call_count, 4)
        with TemporaryDirectory() as folder:
            path = Path(folder) / "trial.csv"
            with patch("turret_tuning.filedialog.asksaveasfilename", return_value=str(path)):
                panel.export_csv()
            with path.open(encoding="utf-8-sig", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 3)
            self.assertEqual(rows[0]["actual_command_rpm"], "42")
            self.assertEqual(rows[0]["position_counts"], "-456")
            self.assertEqual(rows[0]["kd"], "39000")


    def test_error_and_stop_do_not_launch_more_turns(self):
        panel = self.make_panel(); panel.start_trial()
        panel.receive(parse_frame("{RSP,EMM,PID_SET,ERR,MOTOR_FEEDBACK_TIMEOUT}"))
        self.assertFalse(panel.active); self.assertEqual(panel.send.call_count, 1)
        panel.start_trial(); panel.stop_trial()
        self.assertEqual(panel.send.call_args.args[0], "{CMD,NAV,STOP}")
        before = panel.send.call_count
        panel.receive(parse_frame("{RSP,EMM,PID_SET,READBACK,7,26000,10,39000}"))
        self.assertEqual(panel.send.call_count, before)

    def make_linear_panel(self, motor):
        panel = self.make_panel()
        panel.axis_id = str(motor)
        panel.unit, panel.position_mode = "mm", "位置"
        panel.event_group, panel.trial_command = "AXIS", "AXIS_TRIAL"
        panel.motor.get.return_value = str(motor)
        panel.variables["a"].get.return_value = "80" if motor == 5 else "0"
        panel.variables["b"].get.return_value = "120" if motor == 5 else "50"
        panel.variables["speed"].get.return_value = "500"
        return panel

    def test_linear_limits_and_integer_rpm(self):
        values = dict(a="0", b="175", speed="65535", ramp="65535", period="1", duration="3000", cycles="1")
        self.assertEqual(trial_settings(values, 5)["speed"], 65535)
        self.assertEqual(trial_settings(dict(values, b="170"), 6)["b"], 170)
        for motor, change in ((5, {"b": "175.1"}), (6, {"b": "170.1"}),
                              (5, {"a": "-0.1"}), (6, {"b": "nan"}),
                              (5, {"speed": "500.5"}), (5, {"speed": "65536"}),
                              (5, {"period": "0"}), (6, {"duration": "0"})):
            with self.subTest(motor=motor, change=change), self.assertRaises(ValueError):
                trial_settings({**values, "b": "100", **change}, motor)

    def test_both_linear_trials_verify_own_pid_filter_feedback_and_export_mm(self):
        for motor in (5, 6):
            with self.subTest(motor=motor):
                panel = self.make_linear_panel(motor)
                panel.start_trial()
                self.assertEqual(panel.send.call_args.args[0], f"{{CMD,EMM,PID_SET,{motor},26000,10,39000,0}}")
                other = 6 if motor == 5 else 5
                panel.receive(parse_frame(f"{{RSP,EMM,PID_SET,READBACK,{other},26000,10,39000}}"))
                self.assertEqual(panel.send.call_count, 1)
                panel.receive(parse_frame(f"{{RSP,EMM,PID_SET,READBACK,{motor},26000,10,39000}}"))
                a, b = (80, 120) if motor == 5 else (0, 50)
                self.assertEqual(panel.send.call_args.args[0], f"{{CMD,EMM,AXIS_TRIAL,{motor},{float(a)},500,100,20,3000}}")
                # 其他电机的数据及过早/重复DONE不得跳过本段。
                panel.receive(parse_frame(f"{{EVT,AXIS,COMMAND,{other},1,1000,10,0,500,252}}"))
                panel.receive(parse_frame(f"{{EVT,EMM,AXIS_TRIAL,DONE,{motor}}}"))
                self.assertIsNone(panel.current)
                for trace, target in enumerate((a, b, a), 1):
                    panel.receive(parse_frame(f"{{EVT,AXIS,COMMAND,{motor},{trace},1000,{target},0,500,252}}"))
                    panel.receive(parse_frame(f"{{EVT,AXIS,SAMPLE,{other},{trace},1020,{target},999,1,3,0,123,-456,8}}"))
                    self.assertEqual(panel.current.samples, [])
                    panel.receive(parse_frame(f"{{EVT,AXIS,SAMPLE,{motor},{trace},1020,{target},{target},-42,3,0,123,-456,8}}"))
                    self.assertEqual(panel.current.samples[0], (20, target, -42, 3, 123, -456, 8))
                    panel.receive(parse_frame(f"{{EVT,AXIS,RESULT,{motor},{trace},3000,OBSERVED,{target},{target},0}}"))
                    panel.receive(parse_frame(f"{{EVT,EMM,AXIS_TRIAL,DONE,{other}}}"))
                    panel.receive(parse_frame(f"{{EVT,EMM,AXIS_TRIAL,DONE,{motor}}}"))
                    calls = panel.send.call_count
                    panel.receive(parse_frame(f"{{EVT,EMM,AXIS_TRIAL,DONE,{motor}}}"))
                    self.assertEqual(panel.send.call_count, calls)
                self.assertFalse(panel.active)
                self.assertEqual(len(panel.records), 3)
                self.assertEqual(panel.send.call_count, 4)
                with TemporaryDirectory() as folder:
                    path = Path(folder) / "linear.csv"
                    with patch("turret_tuning.filedialog.asksaveasfilename", return_value=str(path)):
                        panel.export_csv()
                    with path.open(encoding="utf-8-sig", newline="") as stream:
                        rows = list(csv.DictReader(stream))
                    self.assertEqual(len(rows), 3)
                    self.assertEqual(rows[0]["motor"], str(motor))
                    self.assertEqual(rows[0]["position_mm"], str(float(a)))
                    self.assertEqual(rows[0]["speed_rpm"], "500")
                    self.assertEqual(rows[0]["position_counts"], "-456")
                    self.assertNotIn("angle_deg", rows[0])

    def test_linear_stop_disconnect_mismatch_and_errors_do_not_continue(self):
        panel = self.make_linear_panel(5)
        panel.start_trial()
        panel.receive(parse_frame("{RSP,EMM,PID_SET,READBACK,5,26000,10,123}"))
        self.assertFalse(panel.active)
        self.assertEqual(panel.send.call_count, 1)
        panel.start_trial()
        panel.receive(parse_frame("{RSP,EMM,PID_SET,READBACK,5,26000,10,39000}"))
        panel.receive(parse_frame("{RSP,EMM,STATE_GET,ERR,BUSY}"))
        self.assertTrue(panel.active)  # 无关状态查询错误不取消本段记录。
        panel.receive(parse_frame("{EVT,AXIS,COMMAND,5,1,1000,80,0,500,252}"))
        panel.receive(parse_frame("{EVT,AXIS,RESULT,5,1,3000,MOTOR_FEEDBACK_TIMEOUT,80,0,0}"))
        calls = panel.send.call_count
        panel.receive(parse_frame("{EVT,EMM,AXIS_TRIAL,FAILED,5}"))
        self.assertFalse(panel.active)
        self.assertEqual(panel.send.call_count, calls)
        panel.start_trial(); panel.stop_trial()
        self.assertEqual(panel.send.call_args.args[0], "{CMD,NAV,STOP}")
        panel.start_trial(); panel.disconnected()
        calls = panel.send.call_count
        panel.receive(parse_frame("{RSP,EMM,PID_SET,READBACK,5,26000,10,39000}"))
        self.assertEqual(panel.send.call_count, calls)

    def test_wrong_motor_and_concurrent_trial_cannot_write_pid(self):
        panel = self.make_linear_panel(6)
        panel.motor.get.return_value = "5"
        with patch("turret_tuning.messagebox.showerror"):
            panel.start_trial()
        panel.send.assert_not_called()
        panel.motor.get.return_value = "6"
        panel.can_start = lambda: False
        with patch("turret_tuning.messagebox.showerror"):
            panel.start_trial()
        panel.send.assert_not_called()


if __name__ == "__main__":
    unittest.main()
