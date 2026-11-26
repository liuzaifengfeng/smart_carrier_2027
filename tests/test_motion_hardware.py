"""编译实际硬件适配逻辑，明确区分指令目标和设备实读反馈。"""
import re
import unittest
from native_support import ROOT, build_native, run_native


def build_motion_hardware():
    cache = ROOT / ".pio/motion-native"
    cache.mkdir(parents=True, exist_ok=True)
    source = (ROOT / "src/motion_hardware.cpp").read_text(encoding="utf-8")
    (cache / "motion_hardware_actual.inc").write_text(re.sub(r"^#include.*\n", "", source, flags=re.M), encoding="utf-8")
    return build_native("hardware", ["tests/motion_hardware_native.cpp"])


class MotionHardwareTests(unittest.TestCase):
    def test_ring_feedback_counts_stable_frames_and_latches_completion(self):
        run_native(build_motion_hardware(), "ring")

    def test_ring_velocity_stops_at_deadline_and_ignores_stale_frames(self):
        run_native(build_motion_hardware(), "ring-velocity")

    def test_tray_requires_measured_multiturn_arrival_and_can_be_stopped(self):
        run_native(build_motion_hardware(), "tray")

    def test_placement_logs_signed_measured_errors_and_read_failures(self):
        run_native(build_motion_hardware(), "placement")

    def test_feedback_requires_actual_position_flags_and_both_servo_angles(self):
        run_native(build_motion_hardware(), "feedback")

    def test_rotation_finishes_on_measured_multiturn_target_only(self):
        run_native(build_motion_hardware(), "turret")

    def test_initial_lift_blocks_rotation_until_measured_height_and_arrival_flag(self):
        run_native(build_motion_hardware(), "lift")
