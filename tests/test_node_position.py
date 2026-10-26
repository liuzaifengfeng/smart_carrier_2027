from pathlib import Path
import os
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]


class PositionTests(unittest.TestCase):
    def test_fixed_targets_and_feedback_gate(self):
        zig = ROOT/'.pio/host-test-tools/ziglang/zig.exe'
        compiler = os.environ.get('CONTINUOUS_TEST_CXX')
        if not compiler and not zig.is_file():
            self.skipTest('需要主机C++编译器')
        command = [compiler] if compiler else [str(zig), 'c++']
        exe = ROOT/'.pio/node_position_test.exe'
        result = subprocess.run(command+['-std=c++14', '-O1', str(ROOT/'tests/node_position_native.cpp'),
            str(ROOT/'src/node_position_control.cpp'), str(ROOT/'src/continuous_route.cpp'),
            str(ROOT/'src/hybrid_route.cpp'), str(ROOT/'src/scan_motion_control.cpp'),
            '-o', str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_scanning_continuous_entry_and_position_retries(self):
        from native_support import build_native, run_native
        sources = ["tests/node_position_tick_native.cpp", "src/node_position_control.cpp", "src/continuous_route.cpp", "src/hybrid_route.cpp", "src/scan_motion_control.cpp"]
        run_native(build_native("node-tick", sources), "node")

    def test_representative_speed_polling_and_full_arrival_comparison(self):
        from native_support import build_native, run_native
        sources = ["tests/node_position_tick_native.cpp", "src/node_position_control.cpp", "src/continuous_route.cpp", "src/hybrid_route.cpp", "src/scan_motion_control.cpp"]
        run_native(build_native("node-tick", sources), "representative")

    def test_middle_and_outer_profiles_on_collinear_segments_both_directions(self):
        from native_support import build_native, run_native
        sources = ["tests/node_position_tick_native.cpp", "src/node_position_control.cpp", "src/continuous_route.cpp", "src/hybrid_route.cpp", "src/scan_motion_control.cpp"]
        run_native(build_native("node-tick", sources), "middle")

    def test_three_new_stable_imu_frames_after_wheel_stop(self):
        from native_support import build_native, run_native
        sources = ["tests/node_position_tick_native.cpp", "src/node_position_control.cpp", "src/continuous_route.cpp", "src/hybrid_route.cpp", "src/scan_motion_control.cpp"]
        run_native(build_native("node-tick", sources), "stable")

    def test_middle_heading_hold_disturbances_forward_reverse_and_encoder_polarity(self):
        from native_support import build_native, run_native
        sources = ["tests/node_position_tick_native.cpp", "src/node_position_control.cpp", "src/continuous_route.cpp", "src/hybrid_route.cpp", "src/scan_motion_control.cpp"]
        run_native(build_native("node-tick", sources), "hold")
