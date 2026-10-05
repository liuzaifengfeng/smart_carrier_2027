import unittest
from unittest.mock import Mock

from upper_computer import build_task_code_command, parse_task_code_response, UpperComputerApp


class TaskCodeTests(unittest.TestCase):
    def test_valid_codes_and_outer_spaces(self):
        for code in ("156+123+516+231", "111+321+666+132"):
            self.assertEqual(build_task_code_command("  " + code + "  "), f"{{CMD,TASK,SET,{code}}}")

    def test_invalid_task_format_and_ranges(self):
        for code in ("", "156+123+516", "156+123+516+231x", "156-123+516+231",
                     "056+123+516+231", "156+123+716+231", "156+112+516+231",
                     "156+123+516+241", "１５６+123+516+231", "156+123+51 6+231"):
            with self.subTest(code=code), self.assertRaises(ValueError):
                build_task_code_command(code)

    def make_app(self):
        app = UpperComputerApp.__new__(UpperComputerApp)
        app.task_code_input = Mock()
        app.task_code_input.get.return_value = "156+123+516+231"
        app.serial_link = Mock()
        app.append_log = Mock()
        app.task_code_var = Mock()
        app.task_code_status = Mock()
        return app

    def test_send_waits_for_confirmed_code(self):
        app = self.make_app()
        app.send_task_code()
        app.serial_link.send_line.assert_called_once_with("{CMD,TASK,SET,156+123+516+231}")
        app.task_code_var.set.assert_not_called()
        self.assertIn("等待", app.task_code_status.set.call_args.args[0])
        self.assertTrue(app._accept_task_code_response("{RSP,TASK,SET,OK,156+123+516+231}"))
        app.task_code_var.set.assert_called_once_with("156+123+516+231")

    def test_rejection_and_invalid_readback_preserve_current_code(self):
        app = self.make_app()
        self.assertTrue(app._accept_task_code_response("{RSP,TASK,SET,ERR,BUSY}"))
        self.assertIn("拒绝", app.task_code_status.set.call_args.args[0])
        for line in ("{RSP,TASK,SET,OK,156+112+516+231}", "{EVT,TASK,SET,OK,156+123+516+231}",
                     "{RSP,TASK,SET,ACK,156+123+516+231}", "{RSP,TASK,SET,OK}"):
            self.assertIsNone(parse_task_code_response(line))
            self.assertFalse(app._accept_task_code_response(line))
        app.task_code_var.set.assert_not_called()

    def test_invalid_input_and_disconnection_do_not_confirm(self):
        app = self.make_app()
        app.task_code_input.get.return_value = "156+112+516+231"
        app.send_task_code()
        app.serial_link.send_line.assert_not_called()
        app.task_code_input.get.return_value = "156+123+516+231"
        app.serial_link.send_line.side_effect = RuntimeError("disconnected")
        app.send_task_code()
        self.assertIn("未发送", app.task_code_status.set.call_args.args[0])
        app.task_code_var.set.assert_not_called()
