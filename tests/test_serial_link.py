"""Serial reception must preserve complete UTF-8 protocol frames."""
import queue
import unittest

from upper_computer import SerialLink


class SerialLinkTests(unittest.TestCase):
    def test_reassembles_frames_across_reads_and_timeout(self):
        events = queue.Queue()
        link = SerialLink(events)
        chunks = iter([
            b"{RSP,CFG,GET,VALUE,1,0,\xe6",
            b"",  # A serial timeout must not end the pending frame.
            b"\x90\xac\xe8\xbf\x90,150,0,175,0}\r\n{RSP,CFG,GET,END,1,1}\r",
            b"\n",
        ])

        class Port:
            def read(self, _size):
                chunk = next(chunks)
                if chunk == b"\n":
                    link._stop.set()
                return chunk

        link._port = Port()
        link._read_loop()
        received = [events.get_nowait()[1] for _ in range(events.qsize())]
        self.assertEqual(received, [
            "{RSP,CFG,GET,VALUE,1,0,搬运,150,0,175,0}",
            "{RSP,CFG,GET,END,1,1}",
        ])


if __name__ == "__main__":
    unittest.main()
