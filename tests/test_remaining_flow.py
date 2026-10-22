"""以实际非阻塞执行模块验证原有业务契约。"""
import unittest
from native_support import build_native, run_native


class RemainingFlowTests(unittest.TestCase):
    def test_round_two_and_return_home(self):
        binary = build_native("executor", ["tests/motion_executor_native.cpp"])
        run_native(binary, "rounds")
        run_native(binary, "cancel")
