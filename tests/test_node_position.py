from pathlib import Path
import os
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]


class PositionTests(unittest.TestCase):
    def test_fixed_targets_and_feedback_gate(self):
        zig = ROOT/'.pio/host-test-tools/ziglang/zig.exe'
        compiler = os.environ.get('CONTINUOUS_TEST_CXX')
        if not compiler and not zig.is_file():
            self.skipTest('需要主机C++编译器')
        command = [compiler] if compiler else [str(zig), 'c++']
        exe = ROOT/'.pio/node_position_test.exe'
        result = subprocess.run(command+['-std=c++14', '-O1', str(ROOT/'tests/node_position_native.cpp'),
            str(ROOT/'src/node_position_control.cpp'), str(ROOT/'src/continuous_route.cpp'),
            str(ROOT/'src/hybrid_route.cpp'), str(ROOT/'src/scan_motion_control.cpp'),
            '-o', str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_scanning_continuous_entry_and_position_retries(self):
        main = (ROOT/'src/main.cpp').read_text(encoding='utf-8')
        self.assertIn('MoveScanApproach(scanDirection * SCAN_APPROACH_MM,', main)
        self.assertIn('MoveScanPosition(distance, SCAN_SEARCH_SPEED_RPM, pollTaskCode)', main)
        chassis = (ROOT/'src/chassis.cpp').read_text(encoding='utf-8')
        self.assertIn('MovePosition(0,distance,0,speed,shouldStop)', chassis)
        source = (ROOT/'src/node_position_hardware.cpp').read_text(encoding='utf-8')
        self.assertIn('Emm_V5_Vel_Control(', source)  # 仅外圈连续移动块使用速度模式
        engine = (ROOT/'src/node_position_control.cpp').read_text(encoding='utf-8')
        self.assertIn('if(curved)', engine)
        self.assertIn('outerBlock(route,i,end', engine)
        position_action = engine[engine.index('const char *move('):engine.index('const char *align(')]
        self.assertNotIn('io.velocity(', position_action)
        self.assertIn('pulses,true,true', source)
        controller = (ROOT/'src/continuous_trial.cpp').read_text(encoding='utf-8')
        self.assertIn('while(!job.formal&&!failure&&!success)', controller)
        self.assertIn('RunPositionNodeRoute(', controller)
