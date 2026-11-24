"""实际主流程、动作等待、合流及取消行为的原生运行测试。"""
import unittest
from native_support import ROOT, build_native, run_native


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

    def test_scan_reads_past_bad_frame_and_sweeps_about_tray_angle(self):
        source = (ROOT / "src/motion_executor.cpp").read_text(encoding="utf-8")
        start = source.index("bool pollTaskCode()")
        end = source.index("bool discReturnArrived()", start)
        cache = ROOT / ".pio/motion-native"
        cache.mkdir(parents=True, exist_ok=True)
        wait_start = source.index("bool takeRequestedPath(")
        wait_end = source.index("\n/**", wait_start)
        runtime = (ROOT / "src/robot_runtime.h").read_text(encoding="utf-8")
        route_definitions = "\n".join(line for line in runtime.splitlines()
                                      if line.startswith(("enum class NodePathState", "constexpr uint32_t ROUTE_REPLY_TIMEOUT_MS")))
        (cache / "scan_navigation_actual.inc").write_text(
            route_definitions + "\n" + source[start:end] + source[wait_start:wait_end], encoding="utf-8")
        run_native(build_native("scan-navigation", ["tests/scan_navigation_native.cpp"]))

    def test_failed_startup_shift_does_not_auto_start(self):
        run_native(self.binary, "startup")

    def test_failed_actions_do_not_advance_business(self):
        run_native(self.binary, "failures")

    def test_vision_failure_prevents_material_actions(self):
        run_native(self.binary, "vision")

    def test_external_cancel_and_timeout_do_not_resume_old_steps(self):
        run_native(self.binary, "cancel")
