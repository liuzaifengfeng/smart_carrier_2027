"""LED 协议边界及上位机发送/回读行为，无需硬件。"""
import unittest
from unittest.mock import Mock

from upper_computer import build_led_command, parse_led_response, UpperComputerApp


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
                self.assertEqual(parse_led_response(f"{{RSP,LED,{action},OK,{value}}}"), value)
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
