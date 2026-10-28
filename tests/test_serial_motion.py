"""实际串口接收分发：ACK 后仍可查询、停止，动作只排队。"""
from pathlib import Path
import re
import unittest
from native_support import ROOT, build_native, run_native


def build_serial_motion():
    cache = ROOT / ".pio/motion-native"
    cache.mkdir(parents=True, exist_ok=True)
    source = (ROOT / "src/serial_commands.inc").read_text(encoding="utf-8")
    source = source[:source.index("void Task_Serial_CMD(")]
    source = re.sub(r"^#include.*\n", "", source, flags=re.M)
    (cache / "serial_motion_actual.inc").write_text(source, encoding="utf-8")
    runtime = (ROOT / "src/robot_runtime.cpp").read_text(encoding="utf-8")
    begin = runtime.index("namespace {\nbool releasePoseCompleted")
    end = runtime.index("// ================= 视觉闭环", begin)
    (cache / "lidar_motion_actual.inc").write_text(runtime[begin:end], encoding="utf-8")
    driver = (ROOT / "src/Emm_V5.cpp").read_text(encoding="utf-8")
    begin = driver.index("void Emm_V5_Init(void)")
    end = driver.index("\n/**", begin)
    (cache / "motor_init_actual.inc").write_text(driver[begin:end], encoding="utf-8")
    return build_native("serial", ["tests/serial_motion_native.cpp"])


class SerialMotionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.binary = build_serial_motion()

    def test_grab_ack_then_queries_and_stop(self):
        run_native(self.binary, "grab")

    def test_position_snapshot_is_not_reused_while_grabbing(self):
        run_native(self.binary, "positions")

    def test_manual_initialization_and_transfers_only_submit(self):
        run_native(self.binary, "maintenance")

    def test_linear_axis_zero_updates_only_measured_axis_and_recovers_after_failure(self):
        run_native(self.binary, "linear-zero")

    def test_lidar_retries_share_job_and_completed_result(self):
        run_native(self.binary, "lidar")

    def test_trial_uses_same_entry_with_existing_protocol(self):
        run_native(self.binary, "trial")

    def test_boot_release_waits_for_motion_idle(self):
        run_native(self.binary, "boot")
