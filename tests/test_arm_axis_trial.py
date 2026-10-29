"""编译实际直线轴试验模块，验证总线命令、实测单位和完整采样窗口。"""
import re
import unittest
from native_support import ROOT, build_native, run_native


class ArmAxisTrialTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cache = ROOT / ".pio/motion-native"
        cache.mkdir(parents=True, exist_ok=True)
        source = (ROOT / "src/arm_axis_trial.cpp").read_text(encoding="utf-8")
        (cache / "arm_axis_trial_actual.inc").write_text(
            re.sub(r"^#include.*\n", "", source, flags=re.M), encoding="utf-8")
        cls.binary = build_native("arm-axis-trial", ["tests/arm_axis_trial_native.cpp"])

    def test_axis_ids_absolute_pulses_rpm_and_ramp_quantization(self):
        run_native(self.binary, "commands")

    def test_actual_feedback_raw_fields_and_full_observation(self):
        run_native(self.binary, "sampling")

    def test_reverse_overshoot_signed_position_and_cancel(self):
        run_native(self.binary, "reverse")

    def test_limits_invalid_calibration_and_recovery(self):
        run_native(self.binary, "invalid")

    def test_feedback_failure_retries_and_leaves_trial_reusable(self):
        run_native(self.binary, "failures")

    def test_deadline_wraparound_and_calibration_snapshot(self):
        run_native(self.binary, "timing")


if __name__ == "__main__":
    unittest.main()
