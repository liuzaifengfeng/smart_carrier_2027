"""运行实际视觉调度与PID，验证DISC航向来源、旋转符号和完成判定。"""
from pathlib import Path
import unittest
from native_support import build_native, run_native

ROOT = Path(__file__).resolve().parents[1]


class DiscImuAlignmentTests(unittest.TestCase):
    def test_actual_disc_heading_control(self):
        cache = ROOT / '.pio/motion-native'
        cache.mkdir(parents=True, exist_ok=True)
        chassis = (ROOT / 'src/chassis.cpp').read_text(encoding='utf-8')
        parameters = chassis[chassis.index('VisionAlignmentDeadzone ALIGN_DEADZONES[]'):chassis.index('// 根据节点序号读取坐标')]
        start = chassis.index('void ResetDiscAlignmentPid()')
        end = chassis.index('\n/**', chassis.index('bool AlignToVisionContinuous(', start))
        (cache / 'disc_pid_actual.inc').write_text(parameters + chassis[start:end], encoding='utf-8')
        runtime = (ROOT / 'src/robot_runtime.cpp').read_text(encoding='utf-8')
        tick = runtime[runtime.index('void TickVisualAlignment()'):runtime.index('// 通过 Serial0')]
        (cache / 'disc_tick_actual.inc').write_text(tick, encoding='utf-8')
        run_native(build_native('disc_imu_alignment', ['tests/disc_imu_alignment_native.cpp']))


if __name__ == '__main__':
    unittest.main()
