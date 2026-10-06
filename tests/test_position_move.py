"""麦轮位置移动的上位机封包、输入校验与执行阶段显示。"""
import unittest
import tkinter as tk
from tkinter import ttk
from unittest.mock import Mock, patch

from upper_computer import UpperComputerApp, build_debug_command


class PositionMoveTests(unittest.TestCase):
    def test_four_arguments_and_pure_translation_or_rotation(self):
        for values, frame in (
            (["200", "100", "30", "80"], "{CMD,CHASSIS,MOVE_POSITION,200,100,30,80}"),
            (["-20.5", "0", "0", "5000"], "{CMD,CHASSIS,MOVE_POSITION,-20.5,0,0,5000}"),
            (["0", "0", "-360", "1"], "{CMD,CHASSIS,MOVE_POSITION,0,0,-360,1}"),
        ):
            with self.subTest(values=values):
                self.assertEqual(build_debug_command("MovePosition", values), frame)

    def test_invalid_input_is_not_sent(self):
        invalid = (["1", "2", "3"], ["1", "2", "3", "80", "0"],
                   ["nan", "0", "0", "80"], ["0", "inf", "0", "80"],
                   ["3395", "0", "0", "80"], ["0", "0", "361", "80"],
                   ["0", "0", "0", "0"], ["0", "0", "0", "5001"],
                   ["0", "0", "0", "inf"], ["1", "0", "360", "80"])
        for values in invalid:
            with self.subTest(values=values), self.assertRaises(ValueError):
                build_debug_command("MovePosition", list(values))

    def app(self):
        app = UpperComputerApp.__new__(UpperComputerApp)
        app.position_move_status = Mock()
        app.pending_target = None
        app.root = Mock()
        app.serial_link = Mock()
        app.append_log = Mock()
        app.command_vars = {"MovePosition": [Mock() for _ in range(4)]}
        for var, value in zip(app.command_vars["MovePosition"], ("200", "100", "30", "80")):
            var.get.return_value = value
        return app

    def test_button_sends_and_waits_for_ack(self):
        app = self.app()
        app.send_command("MovePosition")
        app.serial_link.send_line.assert_called_once_with("{CMD,CHASSIS,MOVE_POSITION,200,100,30,80}")
        self.assertIn("等待", app.position_move_status.set.call_args.args[0])

    def test_embedded_position_controls_send_four_arguments(self):
        root = tk.Tk()
        root.withdraw()
        try:
            app = self.app()
            app.root = root
            app.position_move_status = tk.StringVar(root)
            panel = ttk.Frame(root)
            app._command_group(panel, "MovePosition")
            values = ("200", "100", "30", "80")
            for var, value in zip(app.command_vars["MovePosition"], values):
                var.set(value)

            def descendants(widget):
                for child in widget.winfo_children():
                    yield child
                    yield from descendants(child)

            button = next(widget for widget in descendants(panel)
                          if isinstance(widget, ttk.Button)
                          and widget.cget("text") == "执行同步移动")
            button.invoke()
            app.serial_link.send_line.assert_called_once_with(
                "{CMD,CHASSIS,MOVE_POSITION,200,100,30,80}")
            self.assertEqual(tuple(var.get() for var in app.command_vars["MovePosition"]), values)
        finally:
            root.destroy()

    def test_pending_pose_target_blocks_conflicting_position_move(self):
        app = self.app()
        app.pending_target = object()
        with patch("upper_computer.messagebox.showerror") as error:
            app.send_command("MovePosition")
        error.assert_called_once()
        app.serial_link.send_line.assert_not_called()

    def test_ack_estimated_completion_and_failure_are_distinct(self):
        app = self.app()
        for frame, expected in (
            ("{RSP,CHASSIS,MOVE_POSITION,ACK,200,100,30,80}", "尚未确认"),
            ("{EVT,CHASSIS,MOVE_POSITION,DONE,ESTIMATED}", "现场确认"),
            ("{EVT,CHASSIS,MOVE_POSITION,FAILED}", "失败"),
            ("{RSP,CHASSIS,MOVE_POSITION,ERR,MODE}", "拒绝"),
            ("{RSP,CHASSIS,MOVE_POSITION,ERR,UNKNOWN}", "拒绝"),
        ):
            with self.subTest(frame=frame):
                self.assertTrue(app._accept_position_move_response(frame))
                self.assertIn(expected, app.position_move_status.set.call_args.args[0])
        # 状态回显不直接写地图，ACK 不构成物理位置反馈。
        for frame in ("{CMD,CHASSIS,MOVE_POSITION,1,2,3,80}",
                      "{EVT,CHASSIS,MOVE_POSITION,DONE}",
                      "{RSP,CHASSIS,MOVE_POSITION,ACK,nan,2,3,80}",
                      "{EVT,CHASSIS,MOVE_POSITION,FAILED,EXTRA}"):
            app.position_move_status.reset_mock()
            self.assertFalse(app._accept_position_move_response(frame))
            app.position_move_status.set.assert_not_called()


if __name__ == "__main__":
    unittest.main()
