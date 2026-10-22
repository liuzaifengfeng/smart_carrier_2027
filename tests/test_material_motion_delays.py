"""实际动作程序、235项延时地址绑定及串口搬运行为。"""
import unittest
from native_support import run_native
from test_arm_motion import build_arm_motion
from test_serial_motion import build_serial_motion


class MaterialMotionDelayTests(unittest.TestCase):
    def test_actual_transfer_sequences(self):
        binary = build_arm_motion()
        run_native(binary, "forward")
        run_native(binary, "reverse")

    def test_delay_registry(self):
        run_native(build_arm_motion(), "registry")

    def test_single_transfer_command(self):
        run_native(build_serial_motion(), "maintenance")
