import json
import math
import os
from pathlib import Path
import subprocess
import time
import unittest
from unittest.mock import Mock
from types import SimpleNamespace

from continuous_trial import TrialPlan, TrialConfig, ContinuousTrialPanel, angle_delta
from upper_computer import build_frame, parse_frame, UpperComputerApp, Parameter

ROOT = Path(__file__).resolve().parents[1]


def positions():
    return {i: ((i % 5 if i//5 % 2 == 0 else 4-i % 5)*480+240, i//5*480+240) for i in range(25)}


class PlanTests(unittest.TestCase):
    def test_actual_firmware_dispatch_and_busy_gate(self):
        from native_support import run_native
        from test_serial_motion import build_serial_motion
        run_native(build_serial_motion(), "trial")

    def plan(self, nodes=(0, 1, 8), radii=(0, 120, 0), config=None):
        return TrialPlan(nodes, radii, positions(), config or TrialConfig())

    def test_arc_length_continuity_and_heading(self):
        p = self.plan()
        self.assertAlmostEqual(p.length, 960-240+120*math.pi/2)
        self.assertAlmostEqual(p.heading(0), 180)
        self.assertAlmostEqual(p.heading(p.length), 270)
        for a, b in zip(p.segments, p.segments[1:]):
            self.assertLess(math.dist(a.at(a.offset+a.length), b.at(b.offset)), .00001)
        self.assertEqual(angle_delta(350, 10), 20)
        self.assertEqual(angle_delta(10, 350), -20)
        self.assertEqual(angle_delta(0, 180), 180)

    def test_straight_and_multiple_corners(self):
        self.assertAlmostEqual(self.plan((0, 1, 2), (0, 0, 0)).length, 960)
        self.assertEqual(len(self.plan((0, 1, 8, 7), (0, 120, 120, 0)).segments), 5)

    def test_invalid_geometry_and_config(self):
        for nodes, radii in (((0, 1, 8), (0, 0, 0)), ((0, 1, 0), (0, 120, 0)),
                             ((0, 2), (0, 0)), ((0, 1, 8, 7), (0, 240, 240, 0)),
                             ((0, 1, 8), (1, 120, 0)), ((0, 1, 8), (0, float("nan"), 0))):
            with self.subTest(nodes=nodes, radii=radii), self.assertRaises(ValueError):
                self.plan(nodes, radii)
        for config in (TrialConfig(rpm=0), TrialConfig(start=360), TrialConfig(kp=float("nan")),
                       TrialConfig(omega=20, correction=20), TrialConfig(rpm=80.5)):
            with self.assertRaises(ValueError):
                self.plan(config=config)

    def test_boundary_includes_rotating_body(self):
        coords = positions()
        coords[0] = (150, 150)
        with self.assertRaises(ValueError):
            TrialPlan((0, 1), (0, 0), coords, TrialConfig(start=45, end=45))

    def test_frames_are_short_and_include_preview_coordinates(self):
        frames = self.plan().commands(9999999, build_frame)
        self.assertEqual(len(frames), 5)
        self.assertTrue(all(len(f.encode()) <= 159 for f in frames))
        self.assertEqual(frames[1], "{CMD,NAV,TRIAL_NODE,9999999,0,0,0,240,240}")
        self.assertEqual(frames[-1], "{CMD,NAV,TRIAL_START,9999999}")

    def test_native_controller_and_preview_parity(self):
        zig = ROOT / ".pio/host-test-tools/ziglang/zig.exe"
        compiler = os.environ.get("CONTINUOUS_TEST_CXX")
        if not compiler and not zig.is_file():
            self.skipTest("需要CONTINUOUS_TEST_CXX或.pio/host-test-tools中的Zig进行实际C++仿真")
        executable = ROOT / ".pio/continuous_route_test.exe"
        command = ([compiler] if compiler else [str(zig), "c++"])
        result = subprocess.run(command + ["-std=c++14", "-O1", str(ROOT/"tests/continuous_route_native.cpp"),
                                          str(ROOT/"src/continuous_route.cpp"), str(ROOT/"src/hybrid_route.cpp"), "-o", str(executable)],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        result = subprocess.run([str(executable)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        data = json.loads(result.stdout)
        coords = positions()
        coords[0] = (310, 240)
        p = TrialPlan((0, 1, 8), (0, 120, 0), coords, TrialConfig())
        self.assertAlmostEqual(p.length, data["length"], places=3)
        for i, (x, y, heading) in enumerate(data["samples"]):
            s = p.length*i/10
            px, py = p.at(s)
            self.assertAlmostEqual(x, px, places=3)
            self.assertAlmostEqual(y, py, places=3)
            self.assertAlmostEqual(heading, p.heading(s), places=3)


class PanelProtocolTests(unittest.TestCase):
    def map_app(self):
        app = UpperComputerApp.__new__(UpperComputerApp)
        parameters = {}
        for i, (x, y) in positions().items():
            for axis, value in (("X", x), ("Y", y)):
                index = 130 + len(parameters)
                parameters[index] = Parameter(index, f"地图/节点{i}/{axis}(mm)", str(value), 0, 2400, False)
        app.parameter_panel = SimpleNamespace(ready=True, parameters=parameters)
        return app

    def test_trial_map_uses_integer_indexed_readback(self):
        app = self.map_app()
        self.assertEqual(app._trial_node_positions(), positions())
        # 回读表还可以包含非地图参数，索引不必从0起。
        app.parameter_panel.parameters[999] = Parameter(999, "标定/X脉冲每毫米", "13.3", 0, 100, False)
        self.assertEqual(app._trial_node_positions(), positions())

    def test_trial_map_reports_actual_missing_field(self):
        app = self.map_app()
        missing = app.parameter_panel.parameters.pop(130)
        with self.assertRaisesRegex(RuntimeError, missing.name.replace("(", r"\(").replace(")", r"\)")):
            app._trial_node_positions()
        app.parameter_panel.ready = False
        with self.assertRaisesRegex(RuntimeError, "完整回读"):
            app._trial_node_positions()

    def test_trial_map_rejects_invalid_coordinates(self):
        for value in ("nan", "inf", "-1", "2401", "abc"):
            app = self.map_app()
            app.parameter_panel.parameters[130].value = value
            with self.subTest(value=value), self.assertRaises(RuntimeError):
                app._trial_node_positions()

    def test_real_tk_panel_preview(self):
        import tkinter as tk
        root = tk.Tk()
        root.withdraw()
        try:
            panel = ContinuousTrialPanel(root, Mock(), Mock(), build_frame, parse_frame, positions)
            panel.pack()
            root.update_idletasks()
            plan = panel.preview()
            self.assertIsNotNone(plan)
            self.assertGreater(len(panel.canvas.find_all()), 30)
            panel.destroy()
        finally:
            root.destroy()

    def panel(self):
        p = ContinuousTrialPanel.__new__(ContinuousTrialPanel)
        p.send, p.log, p.status = Mock(), Mock(), Mock()
        p.build_frame, p.parse_frame = build_frame, parse_frame
        p.after, p.after_cancel = Mock(return_value="timer"), Mock()
        p.formal_running = False
        p.result_callback = None
        p.job_id = 42
        p.phase = "uploading"
        p.queue, p.pending = [], None
        p.ack_timer = p.heartbeat_timer = None
        p.last_rx = time.monotonic()
        p.telemetry = []
        return p

    def test_upload_waits_for_matching_ack(self):
        p = self.panel()
        plan = TrialPlan((0, 1, 8), (0, 120, 0), positions(), TrialConfig())
        p.queue = plan.commands(42, build_frame)
        p._next()
        self.assertEqual(p.send.call_count, 1)
        p.receive("{RSP,NAV,TRIAL_BEGIN,ACK,41}")
        self.assertEqual(p.send.call_count, 1)
        p.receive("{RSP,NAV,TRIAL_BEGIN,ACK,42}")
        p.receive("{RSP,NAV,TRIAL_NODE,ACK,42,1}")
        self.assertEqual(p.send.call_count, 2)
        for index in range(3):
            p.receive(f"{{RSP,NAV,TRIAL_NODE,ACK,42,{index}}}")
        self.assertEqual(p.phase, "starting")
        p.receive("{RSP,NAV,TRIAL_START,ACK,42}")
        self.assertEqual(p.phase, "running")
        self.assertTrue(any("TRIAL_STATUS" in call.args[0] for call in p.send.call_args_list))

    def test_start_timeout_never_retries_motion(self):
        p = self.panel()
        p.phase = "starting"
        p._ack_timeout()
        self.assertEqual(p.phase, "stopping")
        p.send.assert_called_once_with("{CMD,NAV,TRIAL_STOP,42}")

    def test_error_is_terminal_without_stop_loop(self):
        p = self.panel()
        p.phase = "running"
        p.receive("{RSP,NAV,TRIAL_STATUS,ERR,42,ID}")
        self.assertEqual(p.phase, "failed")
        self.assertIn("ID", p.status.set.call_args.args[0])
        self.assertEqual(p.send.call_count, 1)

    def test_telemetry_result_and_job_filter(self):
        p = self.panel()
        p.phase = "running"
        p.receive("{EVT,NAV,TRIAL_SAMPLE,42,100,900,180,179,1,20,-20,20,-20,40}")
        self.assertEqual(len(p.telemetry), 1)
        self.assertIn("40ms", p.status.set.call_args.args[0])
        p.receive("{EVT,NAV,TRIAL_RESULT,41,DONE,ESTIMATED,720,720,270}")
        self.assertEqual(p.phase, "running")
        p.receive("{EVT,NAV,TRIAL_RESULT,42,DONE,ESTIMATED,720,720,270}")
        self.assertEqual(p.phase, "done")
        self.assertIsNone(p.heartbeat_timer)

    def test_missing_downlink_stops_heartbeat(self):
        p = self.panel()
        p.phase = "running"
        p.last_rx -= 3
        p._heartbeat()
        self.assertEqual(p.phase, "failed")
        p.send.assert_called_once_with("{CMD,NAV,TRIAL_STOP,42}")

    def test_stop_during_upload_cannot_launch(self):
        p = self.panel()
        p.queue = ["{CMD,NAV,TRIAL_START,42}"]
        p.pending = "{CMD,NAV,TRIAL_BEGIN,42,3}"
        p.stop()
        p.receive("{RSP,NAV,TRIAL_BEGIN,ACK,42}")
        p.send.assert_not_called()
        self.assertEqual(p.phase, "idle")


if __name__ == "__main__":
    unittest.main()
