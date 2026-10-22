"""实际主流程、动作等待、合流及取消行为的原生运行测试。"""
import unittest
from native_support import build_native, run_native


class MotionExecutorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.binary = build_native("executor", ["tests/motion_executor_native.cpp"])

    def test_arm_wait_allows_chassis_ticks_and_commits_after_completion(self):
        run_native(self.binary, "sequence")

    def test_last_translation_triggers_once_and_both_completion_orders_join(self):
        run_native(self.binary, "join")

    def test_ack_precedes_launch_and_cancel_invalidates_old_generation(self):
        run_native(self.binary, "launch")

    def test_polling_submits_once_and_waits_for_actual_result(self):
        run_native(self.binary, "polling")

    def test_two_complete_rounds_and_both_start_zones(self):
        run_native(self.binary, "rounds")

    def test_failed_actions_do_not_advance_business(self):
        run_native(self.binary, "failures")

    def test_vision_failure_prevents_material_actions(self):
        run_native(self.binary, "vision")

    def test_external_cancel_and_timeout_do_not_resume_old_steps(self):
        run_native(self.binary, "cancel")
