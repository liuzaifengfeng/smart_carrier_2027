"""无需硬件的 H30 单次查询、错误处理与回复校验。"""
import unittest
from unittest.mock import Mock
from upper_computer import UpperComputerApp, parse_imu_response, parse_imu_heading_response


class ImuQueryTests(unittest.TestCase):
    def app(self):
        app = UpperComputerApp.__new__(UpperComputerApp)
        app.root = Mock()
        app.serial_link = Mock()
        app.append_log = Mock()
        app.imu_status = Mock()
        app.imu_query_timer = None
        return app

    def test_parse(self):
        self.assertEqual(parse_imu_response("{RSP,IMU,GET,OK,0.234,-0.108,-68.936,5}"),
                         (0.234, -0.108, -68.936, 5))
        for frame in ("{RSP,IMU,GET,OK,1,2,3}", "{RSP,IMU,GET,OK,nan,2,3,0}",
                      "{RSP,IMU,GET,OK,1,inf,3,0}", "{RSP,IMU,GET,OK,1,2,3,-1}",
                      "{RSP,IMU,GET,OK,1,2,3,201}", "{RSP,IMU,GET,OK,1,2,3,0.5}",
                      "{RSP,IMU,GET,ERR,OFFLINE}", "{RSP,POSE,GET,OK,1,2,3,0}"):
            with self.subTest(frame=frame):
                self.assertIsNone(parse_imu_response(frame))

    def test_map_heading(self):
        self.assertEqual(parse_imu_heading_response("{RSP,IMU,HEADING,OK,359.5,10}"), (359.5, 10))
        for heading in ("360", "-1", "nan", "inf"):
            self.assertIsNone(parse_imu_heading_response(f"{{RSP,IMU,HEADING,OK,{heading},10}}"))
        app = self.app()
        app.query_imu_heading()
        app.serial_link.send_line.assert_called_once_with("{CMD,IMU,HEADING}")
        app._accept_imu_response("{RSP,IMU,HEADING,OK,180,10}")
        self.assertIn("地图航向=180.000°", app.imu_status.set.call_args.args[0])
        app._accept_imu_response("{RSP,IMU,HEADING,ERR,NOT_CALIBRATED}")
        self.assertIn("尚未标定", app.imu_status.set.call_args.args[0])

    def test_reference_event(self):
        app = self.app()
        app._accept_imu_response("{EVT,IMU,REFERENCE,ERR,STALE}")
        self.assertIn("未同步", app.imu_status.set.call_args.args[0])
        app._accept_imu_response("{EVT,IMU,REFERENCE,OK,180.000}")
        self.assertIn("180.000°", app.imu_status.set.call_args.args[0])

    def test_one_click_sends_one_request_and_reply_does_not_reschedule(self):
        app = self.app()
        app.query_imu()
        app.serial_link.send_line.assert_called_once_with("{CMD,IMU,GET}")
        app.root.after.assert_called_once()
        app._accept_imu_response("{RSP,IMU,GET,OK,1,-2,3,10}")
        self.assertIsNone(app.imu_query_timer)
        self.assertIn("Roll=-2.000°", app.imu_status.set.call_args.args[0])
        app.serial_link.send_line.assert_called_once()
        app.root.after.assert_called_once()

    def test_offline_and_timeout(self):
        app = self.app()
        app.query_imu()
        app._accept_imu_response("{RSP,IMU,GET,ERR,OFFLINE}")
        app.root.after_cancel.assert_called_once()
        self.assertIn("离线", app.imu_status.set.call_args.args[0])
        app._imu_query_timeout()
        self.assertIn("超时", app.imu_status.set.call_args.args[0])

    def test_bad_reply_and_send_failure(self):
        app = self.app()
        app._accept_imu_response("{RSP,IMU,GET,OK,1,2}")
        self.assertIn("无效", app.imu_status.set.call_args.args[0])
        app.serial_link.send_line.side_effect = RuntimeError("未连接")
        app.query_imu()
        self.assertIsNone(app.imu_query_timer)
        app.root.after.assert_not_called()


if __name__ == "__main__":
    unittest.main()
