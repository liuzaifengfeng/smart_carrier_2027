"""编译并运行实际转盘驱动，验证协议、位置单位及正负绝对目标。"""
import re
import unittest
from native_support import ROOT, build_native, run_native


class TurretMotorTests(unittest.TestCase):
    def test_driver(self):
        cache = ROOT / '.pio/motion-native'
        cache.mkdir(parents=True, exist_ok=True)
        source = (ROOT / 'src/turret_motor.cpp').read_text(encoding='utf-8')
        (cache / 'turret_motor_actual.inc').write_text(
            re.sub(r'^#include.*\n', '', source, flags=re.M), encoding='utf-8')
        run_native(build_native('turret', ['tests/turret_motor_native.cpp']))
