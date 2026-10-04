import unittest

from hmmd_radar.protocol import DEBUG_MODE_COMMAND, HEADER
from hmmd_radar.serial_session import MAX_READ_BYTES, RECONNECT_DELAY_START, SerialSession
from test_protocol import make_frame


class FakeSerial:
    def __init__(self, incoming=b"", write_result=None, read_error=None):
        self.incoming = bytearray(incoming)
        self.write_result = write_result
        self.read_error = read_error
        self.is_open = False
        self.calls = []
        self._dtr = None
        self._rts = None

    @property
    def dtr(self):
        return self._dtr

    @dtr.setter
    def dtr(self, value):
        self._dtr = value
        self.calls.append(("dtr", value))

    @property
    def rts(self):
        return self._rts

    @rts.setter
    def rts(self, value):
        self._rts = value
        self.calls.append(("rts", value))

    @property
    def in_waiting(self):
        return len(self.incoming)

    def open(self):
        self.calls.append(("open",))
        self.is_open = True

    def write(self, data):
        self.calls.append(("write", bytes(data)))
        return len(data) if self.write_result is None else self.write_result

    def read(self, size):
        self.calls.append(("read", size))
        if self.read_error:
            raise self.read_error
        data = bytes(self.incoming[:size])
        del self.incoming[:size]
        return data

    def reset_input_buffer(self):
        self.calls.append(("reset_input",))
        self.incoming.clear()

    def close(self):
        self.calls.append(("close",))
        self.is_open = False


class SerialSessionTests(unittest.TestCase):
    def test_initializes_once_with_safe_lines_and_caps_read_size(self):
        fake = FakeSerial(incoming=b"x" * 4096)
        session = SerialSession("/dev/fake", 115200, lambda: fake)

        self.assertEqual(session.poll(now=0), [])

        self.assertLess(fake.calls.index(("dtr", False)), fake.calls.index(("open",)))
        self.assertLess(fake.calls.index(("rts", False)), fake.calls.index(("open",)))
        self.assertEqual([call for call in fake.calls if call[0] == "write"], [("write", DEBUG_MODE_COMMAND)])
        self.assertLessEqual(max(call[1] for call in fake.calls if call[0] == "read"), MAX_READ_BYTES)
        session.poll(now=0.01)
        self.assertEqual([call for call in fake.calls if call[0] == "write"], [("write", DEBUG_MODE_COMMAND)])

    def test_backlog_over_limit_is_reset_and_counted(self):
        fake = FakeSerial(incoming=b"x" * (MAX_READ_BYTES + 1))
        session = SerialSession("/dev/fake", 115200, lambda: fake)

        self.assertEqual(session.poll(now=0), [])

        self.assertEqual(session.input_backlog_overflows, 1)
        self.assertEqual(fake.in_waiting, 0)
        self.assertFalse([call for call in fake.calls if call[0] == "read"])

    def test_short_initialization_write_retries_after_bounded_delay(self):
        made = []

        def factory():
            fake = FakeSerial(write_result=1)
            made.append(fake)
            return fake

        session = SerialSession("/dev/fake", 115200, factory)
        session.poll(now=0)
        session.poll(now=RECONNECT_DELAY_START / 2)
        self.assertEqual(len(made), 1)
        session.poll(now=RECONNECT_DELAY_START)

        self.assertEqual(len(made), 2)
        self.assertIn("short initialization write", session.last_io_error)
        self.assertEqual(session.reconnects, 2)

    def test_disconnect_clears_partial_state_and_reconnects_to_valid_data(self):
        first = FakeSerial(incoming=HEADER + b"partial", read_error=OSError("unplugged"))
        second = FakeSerial(incoming=make_frame([42] * 320))
        made = iter([first, second])
        session = SerialSession("/dev/fake", 115200, lambda: next(made), stale_after=0.5)
        session.poll(now=0)
        self.assertFalse(session.connected)
        self.assertEqual(session.parser.buffered_bytes, 0)
        self.assertEqual(session.last_io_error, "unplugged")
        self.assertEqual(len(session.poll(now=RECONNECT_DELAY_START)), 1)
        self.assertTrue(session.connected)
        self.assertEqual(session.frames_received, 1)
        self.assertEqual(session.parser.buffered_bytes, 0)

    def test_partial_frame_expires_and_valid_frame_recovers(self):
        partial = HEADER + b"partial"
        first = FakeSerial(incoming=make_frame() + partial)
        session = SerialSession("/dev/fake", 115200, lambda: first, stale_after=0.5)
        self.assertEqual(len(session.poll(now=0)), 1)
        self.assertEqual(session.parser.buffered_bytes, len(partial))
        session.poll(now=0.4)
        self.assertEqual(session.parser.buffered_bytes, len(partial))
        session.poll(now=0.5)
        self.assertEqual(session.parser.buffered_bytes, 0)

    def test_stale_status_tracks_last_frame_receipt(self):
        fake = FakeSerial(incoming=make_frame())
        session = SerialSession("/dev/fake", 115200, lambda: fake, stale_after=1.0)
        session.poll(now=10.0)

        self.assertFalse(session.status(now=10.9).stale)
        self.assertTrue(session.status(now=11.1).stale)


if __name__ == "__main__":
    unittest.main()
