"""无需硬件的 v2 串口封包与连接门控测试。"""

import queue
import unittest
from unittest.mock import Mock

from upper_computer import (
    SerialLink, UpperComputerApp, build_frame, build_node_path_command,
    parse_display_event, parse_frame,
)


class FrameTests(unittest.TestCase):
    def test_command_and_reply_shapes(self):
        self.assertEqual(build_frame("CMD", "SYS", "HELLO"), "{CMD,SYS,HELLO}")
        self.assertEqual(
            parse_frame("{RSP,SYS,HELLO,OK,2}"),
            ("RSP", "SYS", "HELLO", "OK", "2"),
        )
        self.assertEqual(
            parse_frame("{EVT,NAV,ROUTE_REQUEST,2,14}"),
            ("EVT", "NAV", "ROUTE_REQUEST", "2", "14"),
        )

    def test_bad_shapes_and_too_long_command(self):
        for text in ("{RSP,SYS,}", "{RSP,SYS,HELLO", "noise{RSP,SYS,HELLO}",
                     "{CMD,SYS,START}", "{EVT,NAV,ROUTE_REQUEST,\n2,14}"):
            with self.subTest(text=text):
                self.assertIsNone(parse_frame(text))
        for args in (("",), ("1,2",), ("x\ny",), ("x" * 160,)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                build_frame("CMD", "NAV", "ROUTE", *args)

    def test_full_route_fits_firmware_line(self):
        line = build_node_path_command("-".join(map(str, range(25))))
        self.assertLessEqual(len(line.encode("ascii")), 159)
        self.assertEqual(line.count(","), 27)

    def test_display_event_is_one_safe_content_field(self):
        self.assertEqual(
            parse_display_event("{EVT,DISPLAY,TASK_CODE,156+123+516+231}"),
            ("TASK_CODE", "156+123+516+231"),
        )
        self.assertEqual(
            parse_display_event("{EVT,DISPLAY,DEBUG,等待扫码}"),
            ("DEBUG", "等待扫码"),
        )
        for text in ("{EVT,DISPLAY,TASK_CODE,a,b}", "{EVT,DISPLAY,OTHER,text}",
                     "{EVT,DISPLAY,DEBUG,one\ntwo}", "{RSP,DISPLAY,DEBUG,text}"):
            with self.subTest(text=text):
                self.assertIsNone(parse_display_event(text))


class ConnectionGateTests(unittest.TestCase):
    def test_hello_only_until_version_confirmed(self):
        link = SerialLink(queue.Queue())
        link._port = Mock(is_open=True)
        with self.assertRaisesRegex(RuntimeError, "协议 v2"):
            link.send_line("{CMD,CHASSIS,MOVE,0,80,0}")
        link.send_line("{CMD,SYS,HELLO}")
        link._port.write.assert_called_once_with(b"{CMD,SYS,HELLO}\n")
        link.protocol_ready = True
        link.send_line("{CMD,CHASSIS,MOVE,0,80,0}")
        self.assertEqual(link._port.write.call_count, 2)

    def test_task_code_and_debug_use_separate_labels(self):
        app = UpperComputerApp.__new__(UpperComputerApp)
        app.task_code_var = Mock()
        app.debug_display_var = Mock()
        self.assertTrue(app._accept_display_event("{EVT,DISPLAY,TASK_CODE,156+123+516+231}"))
        self.assertTrue(app._accept_display_event("{EVT,DISPLAY,DEBUG,READ TASK}"))
        app.task_code_var.set.assert_called_once_with("156+123+516+231")
        app.debug_display_var.set.assert_called_once_with("READ TASK")


if __name__ == "__main__":
    unittest.main()
