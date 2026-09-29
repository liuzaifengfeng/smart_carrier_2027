"""无硬件校验七轴串口回复和上位机显示更新。"""
import unittest
from unittest.mock import Mock

from upper_computer import ArmPose, Pose, UpperComputerApp, parse_pose_response


class PoseQueryTests(unittest.TestCase):
    def test_parse_valid_and_reject_bad_frames(self):
        self.assertEqual(
            parse_pose_response("{RSP,POSE,GET,OK,150,150,270,50,80,30,20}"),
            (Pose(150, 150, 270), ArmPose(50, 80, 30, 20)),
        )
        for frame in (
            "{RSP,POSE,GET,OK,1,2,3}", "{RSP,POSE,GET,OK,1,2,3,4,5,6,7,8}",
            "{RSP,POSE,GET,OK,nan,2,3,4,5,6,7}", "{RSP,POSE,GET,OK,1,2,3,4,5,6,inf}",
            "{RSP,POSE,GET,OK,1,2,3,4,5,6,abc}", "{RSP,POSE,GET,ERR,BUSY}",
            "{RSP,POSE,GET,OK,1,2,3,4,5,6,7", "noise{RSP,POSE,GET,OK,1,2,3,4,5,6,7}",
        ):
            with self.subTest(frame=frame):
                self.assertIsNone(parse_pose_response(frame))

    def app(self):
        app = UpperComputerApp.__new__(UpperComputerApp)
        app.root = Mock()
        app.serial_link = Mock()
        app.append_log = Mock()
        app.pose_query_timer = None
        app.pose_query_status = Mock()
        app.current_vars = (Mock(), Mock(), Mock())
        app.field = Mock()
        app.arm_preview = Mock()
        app.arm_status_var = Mock()
        return app

    def test_reply_updates_both_views_and_cancels_timeout(self):
        app = self.app()
        app.query_poses()
        app.serial_link.send_line.assert_called_once_with("{CMD,POSE,GET}")
        app._accept_pose_response("{RSP,POSE,GET,OK,150,150,270,50,80,30,20}")
        app.field.set_current_pose.assert_called_once_with(Pose(150, 150, 270))
        app.field.set_arm_pose.assert_called_once_with(ArmPose(50, 80, 30, 20))
        app.arm_preview.set_pose.assert_called_once_with(app.arm_estimate)
        for variable, value in zip(app.current_vars, ("150", "150", "270")):
            variable.set.assert_called_once_with(value)
        app.root.after_cancel.assert_called_once()
        self.assertIsNone(app.pose_query_timer)

    def test_invalid_reply_and_timeout_preserve_display(self):
        app = self.app()
        app._accept_pose_response("{RSP,POSE,GET,OK,1,2}")
        app._pose_query_timeout()
        app.field.set_current_pose.assert_not_called()
        app.arm_preview.set_pose.assert_not_called()

    def test_disconnected_query_does_not_schedule_timer(self):
        app = self.app()
        app.serial_link.send_line.side_effect = RuntimeError("串口尚未连接")
        app.query_poses()
        app.root.after.assert_not_called()

    def test_actual_negative_angles_reach_views_without_normalization(self):
        app = self.app()
        app._accept_pose_response("{RSP,POSE,GET,OK,150,150,270,50,80,-55.3,12.7}")
        measured = ArmPose(50, 80, -55.3, 12.7)
        app.field.set_arm_pose.assert_called_once_with(measured)
        app.arm_preview.set_pose.assert_called_once_with(measured)
        self.assertIn("实测", app.arm_status_var.set.call_args.args[0])

    def test_servo_failure_preserves_views_and_reports_axis(self):
        for servo_id, axis in ((1, "夹爪"), (2, "舵盘")):
            with self.subTest(servo_id=servo_id):
                app = self.app()
                app.pose_query_timer = "timer"
                app._accept_pose_response(f"{{RSP,POSE,GET,ERR,SERVO_READ,{servo_id}}}")
                app.field.set_current_pose.assert_not_called()
                app.arm_preview.set_pose.assert_not_called()
                app.root.after_cancel.assert_called_once_with("timer")
                self.assertIn(axis, app.pose_query_status.set.call_args.args[0])


if __name__ == "__main__":
    unittest.main()
