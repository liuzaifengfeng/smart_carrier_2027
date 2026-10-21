"""以实际非阻塞执行模块验证原有业务契约。"""
import unittest
from native_support import build_native, run_native


class LidarPoseTests(unittest.TestCase):
    def test_release_retries_do_not_repeat_motion(self):
        from test_serial_motion import build_serial_motion
        run_native(build_serial_motion(), "lidar")
