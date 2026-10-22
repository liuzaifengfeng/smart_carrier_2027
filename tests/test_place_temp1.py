"""以实际非阻塞执行模块验证原有业务契约。"""
import unittest
from native_support import build_native, run_native


class PlaceTemp1Tests(unittest.TestCase):
    def test_actual_state_flow(self):
        run_native(build_native("executor", ["tests/motion_executor_native.cpp"]), "failures")
