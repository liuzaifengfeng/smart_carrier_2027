"""计时统计行为测试：正式边界、未完成记录及并行区间去重。"""

import unittest

from scripts.summarize_motion_log import parse_events, render, summarize, union_ms


def report(*frames):
    return summarize(parse_events(frames))


class MotionLogTests(unittest.TestCase):
    def test_permission_wait_is_excluded_and_last_phase_closed_at_result(self):
        result = report(
            "{EVT,MOTION,PHASE,0,100,WAIT_START}",
            "{EVT,MOTION,RUN_START,0,1000}",
            "{EVT,MOTION,PHASE,0,1010,READ_TASK}",
            "{EVT,MOTION,PHASE,0,2000,RETURN_HOME}",
            "{EVT,MOTION,RUN_RESULT,0,3000,DONE,2000}",
            "{EVT,MOTION,PHASE,0,3010,DONE}")
        self.assertEqual((result["status"], result["total_ms"]), ("DONE", 2000))
        self.assertEqual(result["phases"], [("WAIT_START", 1000, 1010, 10),
                                           ("READ_TASK", 1010, 2000, 990),
                                           ("RETURN_HOME", 2000, 3000, 1000)])
        self.assertEqual(sum(row[3] for row in result["phases"]), 2000)

    def test_ack_flags_and_last_timestamp_do_not_imply_completion(self):
        result = report("{EVT,MOTION,RUN_START,0,100}",
                        "{EVT,MOTION,PHASE,0,100,GRAB_ROUND1}",
                        "{RSP,ARM,TRANSFER,ACK}",
                        "{EVT,MOTION,FLAG,7,900,ARM_READY,1}")
        self.assertEqual((result["status"], result["total_ms"]), ("INCOMPLETE", None))
        self.assertEqual(result["phases"], [("GRAB_ROUND1", 100, None, None)])
        self.assertIn("未知", render(result))

    def test_failed_phase_closes_preceding_phase_without_claiming_a_complete_run(self):
        result = report("{EVT,MOTION,RUN_START,0,100}",
                        "{EVT,MOTION,PHASE,0,100,READ_TASK}",
                        "{EVT,MOTION,PHASE,0,400,SCAN_FAILED}")
        self.assertEqual(result["phases"][0], ("READ_TASK", 100, 400, 300))
        self.assertIsNone(result["total_ms"])

    def test_parallel_jobs_are_not_added_to_competition_time(self):
        result = report("{EVT,MOTION,RUN_START,0,100}",
                        "{EVT,MOTION,RESULT,1,1000,DONE,900}",
                        "{EVT,MOTION,RESULT,2,1500,DONE,1000}",
                        "{EVT,MOTION,RUN_RESULT,0,2000,DONE,1900}")
        self.assertEqual((result["job_sum_ms"], result["job_wall_ms"], result["job_overlap_ms"]),
                         (1900, 1400, 500))
        self.assertEqual(result["total_ms"], 1900)

    def test_nested_adjacent_and_disjoint_intervals(self):
        self.assertEqual(union_ms([(100, 1000), (200, 300), (1000, 1100),
                                   (1500, 1600), (1600, 1600)]), 1100)
        self.assertEqual(union_ms([]), 0)

    def test_components_clip_to_run_and_keep_nested_steps_separate(self):
        result = report("{EVT,MOTION,SPAN,1,50,NAVIGATION,BEGIN}",
                        "{EVT,MOTION,RUN_START,0,100}",
                        "{EVT,MOTION,SPAN,1,200,ARM_PROGRAM,BEGIN}",
                        "{EVT,MOTION,STEP,1,250,0,LOOK_LIFT,BEGIN}",
                        "{EVT,MOTION,STEP,1,350,0,LOOK_LIFT,END}",
                        "{EVT,MOTION,SPAN,1,500,ARM_PROGRAM,END}",
                        "{EVT,MOTION,SPAN,1,800,NAVIGATION,END}",
                        "{EVT,MOTION,RUN_RESULT,0,700,DONE,600}")
        self.assertEqual(result["components"], [("ARM_PROGRAM", 1, 300, 300),
                         ("NAVIGATION", 1, 600, 600), ("STEP_LOOK_LIFT", 1, 100, 100)])

    def test_unfinished_or_restarted_steps_never_gain_duration(self):
        result = report("{EVT,MOTION,SPAN,1,100,ARM_PROGRAM,BEGIN}",
                        "{EVT,MOTION,SPAN,1,200,ARM_PROGRAM,BEGIN}",
                        "{EVT,MOTION,STEP,1,250,0,PICK_GRIP,BEGIN}")
        self.assertEqual(result["components"], [])
        self.assertEqual(len(result["unfinished"]), 3)

    def test_rx_prefix_is_ignored_and_device_clock_orders_events(self):
        events = parse_events(["[12:34:56.123] [RX] {EVT,MOTION,PHASE,0,200,READ_TASK}",
                               "[12:34:57.000] [RX] {EVT,MOTION,RUN_START,0,100}"])
        self.assertEqual([event.time for event in events], [100, 200])

    def test_partial_log_does_not_gain_a_competition_duration(self):
        result = report("{EVT,MOTION,PHASE,0,200,RETURN_HOME}",
                        "{EVT,MOTION,RUN_RESULT,0,900,DONE,800}")
        self.assertEqual((result["status"], result["total_ms"]), ("INCOMPLETE", None))

    def test_invalid_or_merged_logs_are_rejected(self):
        cases = [("{EVT,MOTION,RUN_START,0,100}", "{EVT,MOTION,RUN_START,0,200}"),
                 ("{EVT,MOTION,RUN_START,0,100}", "{EVT,MOTION,RUN_RESULT,0,900,DONE,900}"),
                 ("{EVT,MOTION,RESULT,1,900,DONE,1000}",),
                 ("{EVT,MOTION,RUN_START,0,-1}",),
                 ("{EVT,MOTION,PHASE,0,100}",)]
        for frames in cases:
            with self.subTest(frames=frames), self.assertRaises(ValueError):
                report(*frames)


if __name__ == "__main__":
    unittest.main()
