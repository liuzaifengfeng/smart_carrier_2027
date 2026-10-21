"""以实际非阻塞执行模块验证原有业务契约。"""
import unittest
from native_support import build_native, run_native


class BootReleaseTests(unittest.TestCase):
    def test_button_and_release_entry(self):
        from test_serial_motion import build_serial_motion
        run_native(build_serial_motion(), "boot")
