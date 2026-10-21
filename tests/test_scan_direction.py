"""编译期执行实际扫码移动分支，验证两启停区方向及提前停车。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from upper_computer import Pose, START_POSES


class ScanDirectionTests(unittest.TestCase):
    def test_actual_encoder_pose_after_early_stop(self):
        root = Path(__file__).resolve().parents[1]
        zig = root/'.pio/host-test-tools/ziglang/zig.exe'
        compiler = os.environ.get('CONTINUOUS_TEST_CXX')
        if not compiler and not zig.is_file():
            self.skipTest('需要主机C++编译器')
        command = [compiler] if compiler else [str(zig), 'c++']
        cache = root/'.pio/native_scan_position'
        cache.mkdir(exist_ok=True)
        source = (root/'src/chassis.cpp').read_text(encoding='utf-8')
        actual = source[source.index('bool MoveScanPosition('):source.index('bool MoveScanApproach(')]
        (cache/'scan_position_actual.inc').write_text(actual, encoding='utf-8')
        target = source[source.index('    const double heading = normalizeHeading(currentPose.theta)',
                                     source.index('bool BeginPositionMove(')):]
        target = target[:target.index('    for (uint8_t motor = 1;')]
        (cache/'position_target_actual.inc').write_text(target, encoding='utf-8')
        exe = cache/'test.exe'
        result = subprocess.run(command+['-std=c++14', '-O1', '-I', str(cache),
            str(root/'tests/scan_position_native.cpp'), '-o', str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_start_heading_and_scan_to_left_boundary_turn(self):
        from native_support import run_native
        from test_serial_motion import build_serial_motion
        run_native(build_serial_motion(), "zones")
        self.assertEqual(START_POSES["启停区1"], Pose(2250, 150, 180))
        self.assertEqual(START_POSES["启停区2"], Pose(150, 150, 180))

    def test_scan_motion_for_both_start_zones(self):
        from native_support import build_native, run_native
        sources = ["tests/node_position_tick_native.cpp", "src/node_position_control.cpp", "src/continuous_route.cpp", "src/hybrid_route.cpp", "src/scan_motion_control.cpp"]
        run_native(build_native("node-tick", sources), "scan")


if __name__ == "__main__":
    unittest.main()
