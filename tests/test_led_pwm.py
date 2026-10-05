"""LED 协议边界及上位机发送/回读行为，无需硬件。"""
import unittest
from unittest.mock import Mock

from upper_computer import build_led_command, build_led_frequency_command, parse_led_response, UpperComputerApp


class LedTests(unittest.TestCase):
    def test_commands_and_invalid_values(self):
        self.assertEqual(build_led_command(), "{CMD,LED,GET}")
        for value in (0, 1, 50, 99, 100):
            self.assertEqual(build_led_command(value), f"{{CMD,LED,SET,{value}}}")
        for value in (-1, 101, 50.5, float("nan"), "50", True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                build_led_command(value)

    def test_readback_requires_valid_success(self):
        for action in ("SET", "GET"):
            for value in (0, 50, 100):
                self.assertEqual(parse_led_response(f"{{RSP,LED,{action},OK,{value}}}"), (value, None))
        for text in ("{RSP,LED,SET,ERR,RANGE}", "{RSP,LED,SET,ACK,50}",
                     "{RSP,LED,GET,OK,101}", "{RSP,LED,GET,OK,-1}",
                     "{RSP,LED,GET,OK,50.5}", "{RSP,LED,GET,OK,50,extra}",
                     "{RSP,LED,OTHER,OK,50}"):
            self.assertIsNone(parse_led_response(text))

    def test_send_waits_for_readback_and_disconnect_is_reported(self):
        app = UpperComputerApp.__new__(UpperComputerApp)
        app.serial_link = Mock()
        app.led_status = Mock()
        app.append_log = Mock()
        for value in (50, 0, None):
            app.send_led(value)
            app.serial_link.send_line.assert_called_with(build_led_command(value))
            self.assertIn("等待", app.led_status.set.call_args.args[0])
        app._accept_led_response("{RSP,LED,SET,OK,50}")
        self.assertIn("50%", app.led_status.set.call_args.args[0])
        app._accept_led_response("{RSP,LED,SET,ERR,UNKNOWN_ACTION}")
        self.assertIn("拒绝", app.led_status.set.call_args.args[0])
        app.serial_link.send_line.side_effect = RuntimeError("disconnected")
        app.send_led(0)
        self.assertIn("未发送", app.led_status.set.call_args.args[0])

    def test_frequency_range_and_readback(self):
        for hz in (100, 2000, 9000):
            self.assertEqual(build_led_frequency_command(hz), f"{{CMD,LED,FREQ,{hz}}}")
            for action in ("FREQ", "SET", "GET"):
                self.assertEqual(parse_led_response(f"{{RSP,LED,{action},OK,50,{hz}}}"), (50, hz))
        for hz in (0, 99, 9001, -1, 100.5, True, "2000"):
            with self.subTest(hz=hz), self.assertRaises(ValueError):
                build_led_frequency_command(hz)
        for text in ("{RSP,LED,FREQ,OK,50}", "{RSP,LED,GET,OK,50,0}",
                     "{RSP,LED,GET,OK,50,9001}", "{RSP,LED,FREQ,OK,50,2000.5}"):
            self.assertIsNone(parse_led_response(text))

    def test_frequency_button_and_old_firmware(self):
        app = UpperComputerApp.__new__(UpperComputerApp)
        app.serial_link = Mock()
        app.led_status = Mock()
        app.append_log = Mock()
        app.led_frequency = Mock()
        app.led_frequency.get.return_value = "3500"
        app.send_led_frequency()
        app.serial_link.send_line.assert_called_once_with("{CMD,LED,FREQ,3500}")
        app._accept_led_response("{RSP,LED,FREQ,OK,50,3500}")
        self.assertIn("3500 Hz", app.led_status.set.call_args.args[0])
        app._accept_led_response("{RSP,LED,GET,OK,50}")
        self.assertIn("未报告频率", app.led_status.set.call_args.args[0])
        app.serial_link.send_line.reset_mock()
        for raw in ("", "1.5", "-100", "9001", "nan"):
            app.led_frequency.get.return_value = raw
            app.send_led_frequency()
        app.serial_link.send_line.assert_not_called()
