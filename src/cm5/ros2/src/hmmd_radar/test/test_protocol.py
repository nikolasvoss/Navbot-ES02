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
)


def make_frame(values=None):
    if values is None:
        values = [0] * (DOPPLER_BINS * 16)
    payload = struct.pack("<320I", *values)
    assert len(payload) == PAYLOAD_SIZE
    return HEADER + payload + FOOTER


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


if __name__ == "__main__":
    unittest.main()
