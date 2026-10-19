from pathlib import Path
import os
import subprocess
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]


class MotionTests(unittest.TestCase):
    def test_actual_formal_navigation_adapter(self):
        zig = ROOT/'.pio/host-test-tools/ziglang/zig.exe'
        compiler = os.environ.get('CONTINUOUS_TEST_CXX')
        if not compiler and not zig.is_file():
            self.skipTest('需要主机C++编译器')
        command = [compiler] if compiler else [str(zig), 'c++']
        cache = ROOT/'.pio/native_h30_formal';cache.mkdir(exist_ok=True)
        source = (ROOT/'src/continuous_trial.cpp').read_text(encoding='utf-8')
        (cache/'h30_formal_actual.inc').write_text(source[source.index('bool H30StartNodePath('):], encoding='utf-8')
        exe = cache/'test.exe'
        result = subprocess.run(command+['-std=c++14', '-O1', '-I', str(cache), str(ROOT/'tests/h30_formal_native.cpp'),
            str(ROOT/'src/continuous_route.cpp'), str(ROOT/'src/hybrid_route.cpp'), '-o', str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_actual_settle_and_heading_controller(self):
        zig = ROOT/'.pio/host-test-tools/ziglang/zig.exe'
        compiler = os.environ.get('CONTINUOUS_TEST_CXX')
        if not compiler and not zig.is_file():
            self.skipTest('需要主机C++编译器')
        command = [compiler] if compiler else [str(zig), 'c++']
        exe = ROOT/'.pio/h30_motion_test.exe'
        result = subprocess.run(command+['-std=c++14', '-O1', str(ROOT/'tests/h30_motion_native.cpp'),
            str(ROOT/'src/continuous_route.cpp'), str(ROOT/'src/hybrid_route.cpp'), '-o', str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_formal_route_stop_and_result_ui(self):
        from test_continuous_trial import PanelProtocolTests
        panel = PanelProtocolTests().panel()
        panel.result_callback = Mock()
        panel.receive('{EVT,NAV,ROUTE_RUNNING}')
        self.assertTrue(panel.formal_running)
        panel.stop()
        panel.send.assert_called_with('{CMD,NAV,STOP}')
        panel.receive('{EVT,NAV,ROUTE_PHASE,0,SETTLE_BEFORE_TURN}')
        self.assertIn('安静', panel.status.set.call_args.args[0])
        panel.receive('{EVT,NAV,ROUTE_RESULT,0,DONE,ESTIMATED,1200,1200,90}')
        self.assertFalse(panel.formal_running)
        panel.result_callback.assert_called_once_with(1200, 1200, 90)

    def test_formal_entry_uses_shared_controller_without_rezero(self):
        chassis = (ROOT/'src/chassis.cpp').read_text(encoding='utf-8')
        actual = chassis[chassis.index('bool MoveNodePath('):]
        actual = actual[:actual.index('bool InitArm_start(')]
        self.assertIn('H30RunNodePath(', actual)
        self.assertNotIn('commandSynchronizedStraight(', actual)
        controller = (ROOT/'src/continuous_trial.cpp').read_text(encoding='utf-8')
        actual = controller[controller.index('bool H30StartNodePath('):]
        self.assertNotIn('CalibrateH30', actual)
        self.assertIn('cfg.policy=3', actual)
        serial = (ROOT/'src/serial_commands.inc').read_text(encoding='utf-8')
        actual = serial[serial.index('static void serialRoute('):serial.index('static void serialRelease(')]
        self.assertIn('H30StartNodePath(', actual)
        self.assertNotIn('executeNodePathWithStatus(', actual)
