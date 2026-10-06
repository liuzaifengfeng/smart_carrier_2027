"""日志时间格式及串口入队时间，不依赖实物串口。"""
from datetime import datetime
import queue
import unittest
from unittest.mock import Mock, patch

from upper_computer import SerialLink, UpperComputerApp


class LogTimestampTests(unittest.TestCase):
    def test_timestamp_and_each_line_are_preserved(self):
        app = UpperComputerApp.__new__(UpperComputerApp)
        app.log_text = Mock()
        received_at = datetime(2026, 10, 6, 12, 34, 56, 123456)
        app.append_log("RX", "{RSP,VISION,COLOR,OK}\nsecond line", received_at)
        self.assertEqual(app.log_text.insert.call_args.args[1],
                         "[12:34:56.123] [RX] {RSP,VISION,COLOR,OK}\n"
                         "[12:34:56.123] [RX] second line\n")

    def test_local_logs_use_current_time(self):
        app = UpperComputerApp.__new__(UpperComputerApp)
        app.log_text = Mock()
        with patch("upper_computer.datetime") as clock:
            clock.now.return_value = datetime(2026, 10, 6, 12, 34, 56, 789000)
            app.append_log("TX", "{CMD,SYS,START}")
        self.assertEqual(app.log_text.insert.call_args.args[1],
                         "[12:34:56.789] [TX] {CMD,SYS,START}\n")

    def test_rx_records_time_before_ui_consumes_event(self):
        events = queue.Queue()
        link = SerialLink(events)
        link._port = Mock()
        def read_line():
            link._stop.set()
            return b"{RSP,VISION,COLOR,OK}\r\n"
        link._port.readline.side_effect = read_line
        received_at = datetime(2026, 10, 6, 12, 34, 56, 123000)
        with patch("upper_computer.datetime") as clock:
            clock.now.return_value = received_at
            link._read_loop()
        self.assertEqual(events.get_nowait(), ("RX", "{RSP,VISION,COLOR,OK}", received_at))


if __name__ == "__main__":
    unittest.main()
