"""无需硬件的 v2 串口封包与连接门控测试。"""

import queue
import unittest
from unittest.mock import Mock

from upper_computer import (
    SerialLink, UpperComputerApp, build_frame, build_node_path_command,
    build_debug_command, parse_display_event, parse_frame, parse_vision_start_request,
    build_vision_start_request_command,
)


class FrameTests(unittest.TestCase):
    def test_retrieve_demo_command(self):
        self.assertEqual(build_debug_command("MaterialDemo4", []), "{CMD,ARM,DEMO4}")
        with self.assertRaises(ValueError):
            build_debug_command("MaterialDemo4", ["1"])

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

    def test_five_vision_start_requests_have_exact_shape(self):
        for mode in ("DISC", "DISC_MATERIAL", "WORK_AREA", "WORK_AREA_LOADED", "CORNER"):
            frame = f"{{EVT,VISION,START_REQUEST,{mode}}}"
            with self.subTest(mode=mode):
                self.assertEqual(parse_vision_start_request(frame), mode)
                self.assertEqual(build_vision_start_request_command(mode),
                                 f"{{CMD,VISION,START_REQUEST,{mode}}}")
        for mode in ("", "corner", "UNKNOWN", "CORNER,DISC", "CORNER\n"):
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                build_vision_start_request_command(mode)
        for frame in ("{CMD,VISION,START_REQUEST,DISC}",
                      "{EVT,VISION,START_REQUEST,DISC,EXTRA}",
                      "{EVT,VISION,START_REQUEST,UNKNOWN}",
                      "{EVT,VISION,START_REQUEST,DISC"):
            with self.subTest(frame=frame):
                self.assertIsNone(parse_vision_start_request(frame))


class ConnectionGateTests(unittest.TestCase):
    def test_vision_button_waits_for_vehicle_event(self):
        app = UpperComputerApp.__new__(UpperComputerApp)
        app.serial_link = Mock()
        app.append_log = Mock()
        app.vision_request_var = Mock()
        app.send_vision_start_request("CORNER")
        app.serial_link.send_line.assert_called_once_with("{CMD,VISION,START_REQUEST,CORNER}")
        app.vision_request_var.set.assert_called_with("命令已发送：角点视觉识别，等待小车接令")
        self.assertTrue(app._accept_vision_start_request("{RSP,VISION,START_REQUEST,ACK,CORNER}"))
        app.vision_request_var.set.assert_called_with("小车已接令：角点视觉识别，等待请求事件")
        self.assertTrue(app._accept_vision_start_request("{EVT,VISION,START_REQUEST,CORNER}"))
        app.vision_request_var.set.assert_called_with("请求开启：角点视觉识别")
        self.assertTrue(app._accept_vision_start_request("{RSP,VISION,START_REQUEST,ERR,RANGE}"))
        app.vision_request_var.set.assert_called_with("小车拒绝视觉请求：RANGE")
        self.assertFalse(app._accept_vision_start_request("{RSP,VISION,START_REQUEST,ACK,UNKNOWN}"))

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

    def test_vision_request_is_displayed_without_claiming_vision_started(self):
        app = UpperComputerApp.__new__(UpperComputerApp)
        app.vision_request_var = Mock()
        self.assertTrue(app._accept_vision_start_request(
            "{EVT,VISION,START_REQUEST,WORK_AREA_LOADED}"))
        app.vision_request_var.set.assert_called_once_with(
            "请求开启：粗加工区／暂存区带物料定位")
        self.assertFalse(app._accept_vision_start_request(
            "{EVT,VISION,START_REQUEST,UNKNOWN}"))


if __name__ == "__main__":
    unittest.main()
