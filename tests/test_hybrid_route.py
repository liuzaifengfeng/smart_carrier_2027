import json
import math
import os
from pathlib import Path
import subprocess
import unittest

from continuous_trial import TrialConfig, ContinuousTrialPanel
from hybrid_plan import HybridPlan
from field_geometry import check_pose, GEOMETRY_SIGNATURE
from upper_computer import build_frame, parse_frame
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]


def positions():
    xs, ys = [310, 720, 1200, 1680, 2090], [240, 720, 1200, 1680, 2020]
    return {i: (xs[i % 5 if i//5 % 2 == 0 else 4-i % 5], ys[i//5]) for i in range(25)}


class HybridTests(unittest.TestCase):
    def test_stopped_route_uses_final_heading_to_avoid_extra_half_turn(self):
        for start in (270, 269.6, 270.4):
            plan = HybridPlan([10, 19, 20, 21, 22], positions(), TrialConfig(start=start, end=180, policy=2))
            turns = [p for p in plan.segments if p.kind == 'ROTATE' and abs((p.h1-p.h0+180) % 360-180) > 1]
            self.assertEqual(len(turns), 1)
            self.assertEqual((turns[0].h0, turns[0].h1), (270, 180))

    def plan(self, policy=1):
        return HybridPlan([14, 15, 24, 23, 22], positions(), TrialConfig(start=90, end=180, policy=policy))

    def test_outside_bulges_away_and_never_clips(self):
        plan = self.plan()
        self.assertEqual(sum(p.kind == 'CUBIC' for p in plan.segments), 1)
        self.assertGreater(max(x for _, x, _, _ in plan.preview), 2090)
        self.assertGreater(max(y for _, _, y, _ in plan.preview), 2020)
        self.assertTrue(all(check_pose(x, y, h, 20) is None for _, x, y, h in plan.preview))

    def test_center_stops_and_turns_only_at_junction(self):
        p = HybridPlan([11, 12, 17], positions(), TrialConfig(start=0, end=90, policy=1))
        self.assertEqual([s.kind for s in p.segments], ['LINE', 'ROTATE', 'LINE', 'ROTATE'])
        self.assertEqual(p.segments[1].start, (1200, 1200))
        self.assertEqual(p.segments[0].h1, 0)
        self.assertEqual(p.segments[2].h0, 90)
        self.assertIsNotNone(check_pose(720, 1200, 45))
        self.assertIsNone(check_pose(1200, 1200, 45))

    def test_heading_noise_does_not_hide_feasible_outside_curve(self):
        for heading in (179.9, 180, 180.1, 180.855, 181, 270, 90):
            with self.subTest(heading=heading):
                plan = HybridPlan([10, 19, 20, 21, 22], positions(),
                                  TrialConfig(start=heading, end=180, policy=1, rpm=300, accel=500, omega=90))
                curves = [p for p in plan.segments if p.kind == 'CUBIC']
                self.assertEqual(len(curves), 1)
                self.assertTrue(all(check_pose(x, y, h, 20) is None for _, x, y, h in plan.preview))
                if heading == 180.855:
                    self.assertEqual((curves[0].h0, curves[0].h1), (270, 180))
                    self.assertEqual(plan.segments[-2].h0, 180)

    def test_auto_end_uses_heading_selected_by_feasibility(self):
        plan = HybridPlan([10, 19, 20, 21, 22], positions(),
                          TrialConfig(start=180.855, end=180.855, policy=1), auto_end=True)
        self.assertEqual(plan.config.end, 180)
        self.assertEqual((plan.segments[-1].h0, plan.segments[-1].h1), (180, 180))
        self.assertIn(',180,80,', plan.commands(42, build_frame)[0])

    def test_allstop_and_invalid_sweep(self):
        self.assertFalse(any(p.kind == 'CUBIC' for p in self.plan(2).segments))
        with self.assertRaisesRegex(ValueError, '起点原地转向空间不足'):
            HybridPlan([11, 12, 17], positions(), TrialConfig(start=90, end=90, policy=1))
        with self.assertRaisesRegex(ValueError, '禁入区'):
            HybridPlan([0, 1, 8], positions(), TrialConfig(start=0, end=90, policy=1))

    def test_complete_outer_loop_checks_all_four_corners(self):
        nodes = [14, 15, 24, 23, 22, 21, 20, 19, 10, 9, 0, 1, 2, 3, 4, 5, 14]
        plan = HybridPlan(nodes, positions(), TrialConfig(start=90, end=90, policy=1))
        self.assertEqual(sum(p.kind == 'CUBIC' for p in plan.segments), 4)
        self.assertTrue(all(check_pose(x, y, h, 20) is None for _, x, y, h in plan.preview))

    def test_protocol_has_mandatory_geometry_policy(self):
        frames = self.plan().commands(42, build_frame)
        self.assertEqual(frames[1], f'{{CMD,NAV,TRIAL_POLICY,42,1,60,20,{GEOMETRY_SIGNATURE}}}')
        self.assertTrue(all(len(f.encode()) <= 159 for f in frames))

    def test_upload_waits_for_policy_ack_before_nodes(self):
        from test_continuous_trial import PanelProtocolTests
        panel = PanelProtocolTests().panel()
        panel.queue = self.plan().commands(42, build_frame)
        panel._next()
        panel.receive('{RSP,NAV,TRIAL_BEGIN,ACK,42}')
        self.assertIn('TRIAL_POLICY', panel.pending)
        panel.receive('{RSP,NAV,TRIAL_NODE,ACK,42,0}')
        self.assertIn('TRIAL_POLICY', panel.pending)
        panel.receive('{RSP,NAV,TRIAL_POLICY,ACK,42}')
        self.assertIn('TRIAL_NODE,42,0', panel.pending)

    def test_real_panel_simple_defaults_and_clear_stale_preview(self):
        import tkinter as tk
        root = tk.Tk();root.withdraw()
        try:
            panel = ContinuousTrialPanel(root, Mock(), Mock(), build_frame, parse_frame, positions)
            panel.pack();root.update_idletasks()
            self.assertNotIn('radii', panel.variables)
            self.assertFalse(panel.show_advanced.get())
            self.assertEqual(panel.variables['policy'].get(), '全程停转')
            self.assertIsNotNone(panel.preview())
            panel.variables['policy'].set('自动混合')
            panel.variables['path'].set('10-19-20-21-22')
            panel.heading_provider = lambda: 180.855
            plan = panel.preview()
            self.assertIsNotNone(plan)
            self.assertEqual(plan.config.end, 180)
            self.assertIn('外绕1处', panel.status.get())
            panel.variables['path'].set('0-1-8')
            self.assertIsNone(panel.preview())
            self.assertEqual(panel.canvas.find_all(), ())
            panel.destroy()
        finally:
            root.destroy()

    def test_native_simulation_and_preview_parity(self):
        zig = ROOT/'.pio/host-test-tools/ziglang/zig.exe'
        compiler = os.environ.get('CONTINUOUS_TEST_CXX')
        if not compiler and not zig.is_file():
            self.skipTest('需要主机C++编译器')
        command = [compiler] if compiler else [str(zig), 'c++']
        exe = ROOT/'.pio/hybrid_route_test.exe'
        result = subprocess.run(command+['-std=c++14', '-O1', str(ROOT/'tests/hybrid_route_native.cpp'),
                                str(ROOT/'src/continuous_route.cpp'), str(ROOT/'src/hybrid_route.cpp'), '-o', str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(result.stdout)
        self.assertEqual(data['signature'], GEOMETRY_SIGNATURE)
        plan = self.plan()
        self.assertAlmostEqual(data['length'], plan.length, delta=.002)
        for i, (x, y, h) in enumerate(data['samples']):
            s = plan.length*i/10
            px, py = plan.at(s)
            self.assertAlmostEqual(x, px, delta=.002)
            self.assertAlmostEqual(y, py, delta=.002)
            self.assertAlmostEqual(h, plan.heading(s), delta=.002)
        plan = HybridPlan([10, 19, 20, 21, 22], positions(),
                          TrialConfig(start=180.855, end=180, policy=1, rpm=300, accel=500, omega=90))
        regression = data['regression']
        self.assertAlmostEqual(regression['length'], plan.length, delta=.002)
        for i, (x, y, h) in enumerate(regression['samples']):
            s = plan.length*i/10
            px, py = plan.at(s)
            self.assertAlmostEqual(x, px, delta=.002)
            self.assertAlmostEqual(y, py, delta=.002)
            self.assertAlmostEqual(h, plan.heading(s), delta=.002)


if __name__ == '__main__':
    unittest.main()
