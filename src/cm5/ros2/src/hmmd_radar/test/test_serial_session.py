import unittest
import struct

from hmmd_radar.protocol import DEBUG_MODE_COMMAND, HEADER, encode_command, encode_read_parameter, encode_write_parameter
from hmmd_radar.serial_session import MAX_READ_BYTES, RECONNECT_DELAY_START, SerialSession
from test_protocol import make_command_reply, make_frame


class FakeSerial:
    def __init__(self, incoming=b"", write_result=None, read_error=None):
        self.incoming = bytearray(incoming)
        self.write_result = write_result
        self.read_error = read_error
        self.is_open = False
        self.calls = []
        self.write_hook = None
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
        if self.write_hook:
            response = self.write_hook(bytes(data))
            if response:
                self.incoming.extend(response)
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

    @staticmethod
    def _response_hook(fake, *, write_reply=True, include_maps=False):
        def respond(request):
            if request == DEBUG_MODE_COMMAND:
                return b""
            command = struct.unpack_from("<H", request, 6)[0]
            if command == 0x00FF:
                reply = make_command_reply(command, b"\x02\x00\x20\x00")
            elif command == 0x0008:
                parameter = struct.unpack_from("<H", request, 8)[0]
                value = 12 if parameter == 1 else 30
                reply = make_command_reply(command, struct.pack("<I", value))
            elif command == 0x0007:
                reply = make_command_reply(command) if write_reply else b""
            elif command == 0x00FE:
                reply = make_command_reply(command)
            else:
                raise AssertionError(f"unexpected command {command:#x}")
            if include_maps:
                return make_frame([command] * 320) + reply
            return reply
        fake.write_hook = respond

    def test_read_config_executes_captured_transaction_and_preserves_interleaved_map_frames(self):
        fake = FakeSerial()
        self._response_hook(fake, include_maps=True)
        session = SerialSession("/dev/fake", 115200, lambda: fake)

        result = session.read_radar_config()
        maps = session.poll()

        self.assertTrue(result.success)
        self.assertEqual((result.maximum_distance_gate, result.target_disappearance_delay_seconds), (12, 30))
        self.assertEqual(result.stage, "none")
        self.assertEqual([frame.amplitude_squared[0] for frame in maps], [0xFF, 0x0008, 0x0008, 0x00FE])
        commands = [call[1] for call in fake.calls if call[0] == "write" and call[1] != DEBUG_MODE_COMMAND]
        self.assertEqual(commands, [
            encode_command(0x00FF),
            encode_read_parameter(1),
            encode_read_parameter(4),
            encode_command(0x00FE),
        ])

    def test_setting_write_requires_write_ack_matching_readback_and_exit_ack(self):
        fake = FakeSerial()
        self._response_hook(fake)
        session = SerialSession("/dev/fake", 115200, lambda: fake)

        result = session.set_radar_setting(0, 12)

        self.assertTrue(result.success)
        self.assertTrue(result.write_acknowledged)
        self.assertTrue(result.has_observed_value)
        self.assertEqual(result.observed_value, 12)
        self.assertTrue(result.readback_matched)
        self.assertTrue(result.save_acknowledged)
        commands = [call[1] for call in fake.calls if call[0] == "write" and call[1] != DEBUG_MODE_COMMAND]
        self.assertEqual(commands, [encode_command(0x00FF), encode_write_parameter(1, 12), encode_read_parameter(1), encode_command(0x00FE)])

    def test_mismatched_readback_is_reported_and_requires_a_fresh_snapshot(self):
        fake = FakeSerial()

        def respond(request):
            if request == DEBUG_MODE_COMMAND:
                return b""
            command = struct.unpack_from("<H", request, 6)[0]
            if command == 0x00FF:
                return make_command_reply(command, b"\x02\x00\x20\x00")
            if command == 0x0007:
                return make_command_reply(command)
            if command == 0x0008:
                return make_command_reply(command, struct.pack("<I", 13))
            if command == 0x00FE:
                return make_command_reply(command)
            raise AssertionError(f"unexpected command {command:#x}")

        fake.write_hook = respond
        session = SerialSession("/dev/fake", 115200, lambda: fake)

        result = session.set_radar_setting(0, 12)
        blocked = session.set_radar_setting(0, 12)

        self.assertFalse(result.success)
        self.assertTrue(result.write_acknowledged)
        self.assertEqual(result.observed_value, 13)
        self.assertFalse(result.readback_matched)
        self.assertEqual(result.outcome, "readback_mismatch")
        self.assertEqual(blocked.outcome, "refresh_required")

    def test_invalid_setting_and_value_are_rejected_before_opening_uart(self):
        made = []
        session = SerialSession("/dev/fake", 115200, lambda: made.append(FakeSerial()) or made[-1])

        invalid_selector = session.set_radar_setting(True, 12)
        invalid_value = session.set_radar_setting(0, 16)

        self.assertEqual(invalid_selector.outcome, "invalid_request")
        self.assertEqual(invalid_value.outcome, "invalid_request")
        self.assertEqual(made, [])

    def test_write_timeout_blocks_another_write_until_a_fresh_snapshot(self):
        fake = FakeSerial()
        self._response_hook(fake, write_reply=False)
        session = SerialSession("/dev/fake", 115200, lambda: fake)

        timed_out = session.set_radar_setting(0, 12)
        writes_before_retry = [call for call in fake.calls if call[0] == "write" and call[1] == encode_write_parameter(1, 12)]
        blocked = session.set_radar_setting(0, 12)
        read = session.read_radar_config()

        self.assertEqual(timed_out.outcome, "timeout")
        self.assertEqual(writes_before_retry, [("write", encode_write_parameter(1, 12))])
        self.assertEqual(blocked.outcome, "refresh_required")
        self.assertTrue(read.success)
        self._response_hook(fake, write_reply=True)
        self.assertEqual(session.set_radar_setting(0, 12).outcome, "ok")


if __name__ == "__main__":
    unittest.main()
