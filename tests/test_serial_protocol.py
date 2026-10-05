"""无需硬件的 v2 串口封包与连接门控测试。"""

import queue
import unittest
from unittest.mock import Mock

from upper_computer import (
    SerialLink, UpperComputerApp, build_frame, build_node_path_command,
    build_debug_command, parse_display_event, parse_frame, parse_vision_start_request,
    build_vision_start_request_command,
    build_vision_stop_request_command, parse_vision_stop_request,
)


class FrameTests(unittest.TestCase):
    def test_lidar_scan_complete_uses_system_start(self):
        self.assertEqual(build_debug_command("start", []), "{CMD,SYS,START}")
        with self.assertRaises(ValueError):
            build_debug_command("start", ["1"])

    def test_arm_initialization_commands_have_no_parameters(self):
        for name, action in (("InitArm_start", "INIT_START"), ("InitArm_look", "INIT_LOOK")):
            with self.subTest(name=name):
                self.assertEqual(build_debug_command(name, []), f"{{CMD,ARM,{action}}}")
                with self.assertRaises(ValueError):
                    build_debug_command(name, ["1"])

    def test_turret_cable_limits_in_debug_commands(self):
        self.assertEqual(
            build_debug_command("MoveArm_2", ["-180", "-1", "80"]),
            "{CMD,ARM,MOVE2,-180,-1,80}",
        )
        self.assertEqual(
            build_debug_command("MoveArm_2", ["360", "-1", "80"]),
            "{CMD,ARM,MOVE2,360,-1,80}",
        )
        for angle in ("-180.1", "360.1"):
            with self.subTest(angle=angle), self.assertRaises(ValueError):
                build_debug_command("MoveArm_2", [angle, "-1", "80"])
        with self.assertRaises(ValueError):
            build_debug_command("SERVO", ["254", "0"])

    def test_five_stop_requests_and_invalid_modes(self):
        for mode in ("DISC", "DISC_MATERIAL", "WORK_AREA", "WORK_AREA_LOADED", "CORNER"):
            with self.subTest(mode=mode):
                self.assertEqual(build_vision_stop_request_command(mode),
                                 f"{{CMD,VISION,STOP_REQUEST,{mode}}}")
                self.assertEqual(parse_vision_stop_request(f"{{EVT,VISION,STOP_REQUEST,{mode}}}"), mode)
                self.assertIsNone(parse_vision_start_request(f"{{EVT,VISION,STOP_REQUEST,{mode}}}"))
        for mode in ("", "corner", "UNKNOWN", "DISC,EXTRA", "CORNER\n"):
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                build_vision_stop_request_command(mode)
        for text in ("{CMD,VISION,STOP_REQUEST,DISC}", "{EVT,VISION,STOP_REQUEST,UNKNOWN}",
                     "{EVT,VISION,STOP_REQUEST,DISC,EXTRA}", "{EVT,VISION,STOP_REQUEST}",
                     "{EVT,VISION,START_REQUEST,DISC}", "{EVT,VISION,STOP_REQUEST,DISC"):
            with self.subTest(text=text):
                self.assertIsNone(parse_vision_stop_request(text))

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
    def test_lidar_scan_complete_button_sends_system_start(self):
        app = UpperComputerApp.__new__(UpperComputerApp)
        app.command_vars = {}
        app.serial_link = Mock()
        app.append_log = Mock()
        app.send_command("start")
        app.serial_link.send_line.assert_called_once_with("{CMD,SYS,START}")
        app.append_log.assert_called_once_with("TX", "{CMD,SYS,START}")

    def test_arm_initialization_buttons_send_expected_commands(self):
        app = UpperComputerApp.__new__(UpperComputerApp)
        app.command_vars = {}
        app.serial_link = Mock()
        app.append_log = Mock()
        for name, action in (("InitArm_start", "INIT_START"), ("InitArm_look", "INIT_LOOK")):
            app.send_command(name)
            app.serial_link.send_line.assert_called_with(f"{{CMD,ARM,{action}}}")
            app.append_log.assert_called_with("TX", f"{{CMD,ARM,{action}}}")

    def test_stop_button_and_reply_do_not_claim_vision_has_stopped(self):
        app = UpperComputerApp.__new__(UpperComputerApp)
        app.serial_link = Mock()
        app.append_log = Mock()
        app.vision_request_var = Mock()
        for mode, label in (("DISC", "圆盘定位"), ("CORNER", "角点视觉识别")):
            app.send_vision_stop_request(mode)
            app.serial_link.send_line.assert_called_with(f"{{CMD,VISION,STOP_REQUEST,{mode}}}")
            app.vision_request_var.set.assert_called_with(f"结束命令已发送：{label}，等待小车接令")
            self.assertTrue(app._accept_vision_start_request(f"{{RSP,VISION,STOP_REQUEST,ACK,{mode}}}"))
            app.vision_request_var.set.assert_called_with(f"小车已接结束命令：{label}，等待请求事件")
            self.assertTrue(app._accept_vision_start_request(f"{{EVT,VISION,STOP_REQUEST,{mode}}}"))
            app.vision_request_var.set.assert_called_with(f"请求结束：{label}")
        self.assertTrue(app._accept_vision_start_request("{RSP,VISION,STOP_REQUEST,ERR,FORMAT}"))
        app.vision_request_var.set.assert_called_with("小车拒绝视觉请求：FORMAT")
        self.assertFalse(app._accept_vision_start_request("{RSP,VISION,STOP_REQUEST,ACK,UNKNOWN}"))

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
