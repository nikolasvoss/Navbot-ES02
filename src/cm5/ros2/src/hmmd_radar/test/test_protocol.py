import struct
import unittest

from hmmd_radar.protocol import (
    DOPPLER_BINS,
    FOOTER,
    FRAME_SIZE,
    HEADER,
    MAX_BUFFER_SIZE,
    PAYLOAD_SIZE,
    FrameParser,
    HMMDStreamParser,
    RadarCommandReply,
    encode_command,
    encode_read_parameter,
    encode_write_parameter,
)


def make_frame(values=None):
    if values is None:
        values = [0] * (DOPPLER_BINS * 16)
    payload = struct.pack("<320I", *values)
    assert len(payload) == PAYLOAD_SIZE
    return HEADER + payload + FOOTER


def make_command_reply(command, payload=b"", status=0):
    body = struct.pack("<HH", command | 0x0100, status) + payload
    return b"\xFD\xFC\xFB\xFA" + struct.pack("<H", len(body)) + body + b"\x04\x03\x02\x01"


class FrameParserTests(unittest.TestCase):
    def test_fragmented_frame_preserves_exact_uint32_values(self):
        values = [0, 1, 0x01020304, 0xFFFFFFFF] + list(range(316))
        frame = make_frame(values)
        parser = FrameParser()

        self.assertEqual(parser.feed(frame[:3]), [])
        self.assertEqual(parser.feed(frame[3:700]), [])
        parsed = parser.feed(frame[700:])

        self.assertEqual(len(parsed), 1)
        self.assertEqual(parsed[0].amplitude_squared, tuple(values))
        self.assertEqual(parser.buffered_bytes, 0)

    def test_bad_candidate_resynchronizes_to_following_valid_frame(self):
        bad = bytearray(make_frame())
        bad[-1] = 0
        parser = FrameParser()

        frames = parser.feed(bytes(bad) + make_frame([7] * 320))

        self.assertEqual([frame.amplitude_squared[0] for frame in frames], [7])
        self.assertGreaterEqual(parser.malformed_candidates, 1)
        self.assertEqual(parser.buffered_bytes, 0)

    def test_large_garbage_feed_never_exceeds_retained_limit(self):
        parser = FrameParser()
        frames = parser.feed(b"x" * (MAX_BUFFER_SIZE * 4) + make_frame())

        self.assertEqual(len(frames), 1)
        self.assertLessEqual(parser.buffered_bytes, MAX_BUFFER_SIZE)
        self.assertEqual(parser.buffered_bytes, 0)

    def test_reset_discards_partial_frame(self):
        parser = FrameParser()
        parser.feed(HEADER + b"partial")
        parser.reset()

        self.assertEqual(parser.buffered_bytes, 0)
        self.assertGreater(parser.discarded_bytes, 0)


class HMMDStreamParserTests(unittest.TestCase):
    def test_command_encoders_match_captured_v161_frames(self):
        self.assertEqual(encode_command(0x00FF), bytes.fromhex("fd fc fb fa 02 00 ff 00 04 03 02 01"))
        self.assertEqual(encode_read_parameter(1), bytes.fromhex("fd fc fb fa 04 00 08 00 01 00 04 03 02 01"))
        self.assertEqual(encode_read_parameter(4), bytes.fromhex("fd fc fb fa 04 00 08 00 04 00 04 03 02 01"))
        self.assertEqual(encode_write_parameter(1, 12), bytes.fromhex("fd fc fb fa 08 00 07 00 01 00 0c 00 00 00 04 03 02 01"))
        self.assertEqual(encode_write_parameter(4, 30), bytes.fromhex("fd fc fb fa 08 00 07 00 04 00 1e 00 00 00 04 03 02 01"))

    def test_routes_interleaved_map_and_read_reply_without_parameter_echo(self):
        parser = HMMDStreamParser()
        wire = make_frame([9] * 320) + make_command_reply(0x0008, struct.pack("<I", 12))

        self.assertEqual(parser.feed(wire[:500]), [])
        events = parser.feed(wire[500:])

        self.assertEqual(events[0].amplitude_squared, (9,) * 320)
        self.assertEqual(events[1], RadarCommandReply(0x0008, True, 0, struct.pack("<I", 12)))
        self.assertEqual(parser.buffered_bytes, 0)

    def test_decodes_literal_v161_ack_and_read_capture_layout(self):
        parser = HMMDStreamParser()
        responses = b"".join(bytes.fromhex(value) for value in (
            "fd fc fb fa 08 00 ff 01 00 00 02 00 20 00 04 03 02 01",
            "fd fc fb fa 08 00 08 01 00 00 0c 00 00 00 04 03 02 01",
            "fd fc fb fa 04 00 07 01 00 00 04 03 02 01",
            "fd fc fb fa 04 00 fe 01 00 00 04 03 02 01",
        ))

        events = parser.feed(responses)

        self.assertEqual(events, [
            RadarCommandReply(0x00FF, True, 0, b"\x02\x00\x20\x00"),
            RadarCommandReply(0x0008, True, 0, struct.pack("<I", 12)),
            RadarCommandReply(0x0007, True, 0, b""),
            RadarCommandReply(0x00FE, True, 0, b""),
        ])

    def test_command_parser_resynchronizes_after_bad_tail_and_stays_bounded(self):
        parser = HMMDStreamParser()
        bad = bytearray(make_command_reply(0x0008, struct.pack("<I", 12)))
        bad[-1] = 0

        events = parser.feed(b"x" * (MAX_BUFFER_SIZE * 3) + bytes(bad) + make_command_reply(0x00FE))

        self.assertEqual(events, [RadarCommandReply(0x00FE, True, 0, b"")])
        self.assertGreater(parser.malformed_candidates, 0)
        self.assertLessEqual(parser.buffered_bytes, MAX_BUFFER_SIZE)


if __name__ == "__main__":
    unittest.main()
