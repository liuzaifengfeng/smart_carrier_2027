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


if __name__ == "__main__":
    unittest.main()
