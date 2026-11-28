"""实际动作程序：保留九组参数、坐标、取放顺序及完成后登记。"""
import unittest
from native_support import ROOT, build_native, run_native


def build_arm_motion():
    cache = ROOT / ".pio/motion-native"
    cache.mkdir(parents=True, exist_ok=True)
    runtime = (ROOT / "src/runtime_parameters.cpp").read_text(encoding="utf-8")
    helpers = runtime[runtime.index("void registerTransferDelays("):runtime.index("// 所有搬运速度")]
    tail_start = runtime.index("    registerMaterialDelays();")
    tail = runtime[tail_start:runtime.index("\n}", tail_start)]
    (cache / "delay_registry_actual.inc").write_text(helpers + "\nvoid registrationTail(){\n" + tail + "\n}\n", encoding="utf-8")
    return build_native("arm", ["tests/arm_motion_native.cpp", "src/arm_motion.cpp", "src/material_transfer.cpp"])


class ArmMotionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.binary = build_arm_motion()

    def test_black_material_height_offsets(self):
        run_native(self.binary, "black")

    def test_nine_forward_groups_and_three_heights(self):
        run_native(self.binary, "forward")

    def test_nine_reverse_groups_retract_with_lift(self):
        run_native(self.binary, "reverse")

    def test_round_plan_and_failed_preparation_do_not_change_inventory(self):
        run_native(self.binary, "rounds")

    def test_three_disc_positions_keep_calibrated_targets_and_delays(self):
        run_native(self.binary, "discs")

    def test_corner_observation_poses_for_both_zones(self):
        run_native(self.binary, "corner")

    def test_observation_and_return_poses_keep_original_order(self):
        run_native(self.binary, "poses")

    def test_lidar_preparation_uses_only_chassis_translation_and_turn(self):
        run_native(self.binary, "lidar")

    def test_ring_correction_in_demo_and_formal_first_layer_with_timeout_enabled(self):
        run_native(self.binary, "ring")
