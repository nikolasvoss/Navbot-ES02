import os, random, sys, unittest
sys.path.insert(0, os.path.dirname(__file__))
from framing import FrameDecoder, cobs_decode, cobs_encode, crc16, encode_frame


class Framing(unittest.TestCase):
    def test_crc_check_value(self):
        self.assertEqual(crc16(b"123456789"), 0x29B1)

    def test_cobs_known_vectors(self):
        self.assertEqual(cobs_encode(b"\x00"), b"\x01\x01")
        self.assertEqual(cobs_encode(b"\x11\x22\x00\x33"), b"\x03\x11\x22\x02\x33")
        self.assertEqual(cobs_encode(bytes(range(1, 255))), b"\xff" + bytes(range(1, 255)))

    def test_cobs_roundtrip(self):
        rng = random.Random(1)
        for n in (0, 1, 2, 253, 254, 255, 300):
            d = bytes(rng.choice([0, 1, 255, rng.randrange(256)]) for _ in range(n))
            e = cobs_encode(d)
            self.assertNotIn(0, e)
            self.assertEqual(cobs_decode(e), d)

    def test_frames_resync_after_garbage(self):
        good = [encode_frame(bytes([i, 0, i])) for i in range(3)]
        bad = bytearray(good[1]); bad[2] ^= 0x40
        stream = b"\x07junk\x00" + good[0] + bytes(bad) + good[2]
        dec = FrameDecoder()
        out = []
        for i in range(len(stream)):  # byte-at-a-time
            out += list(dec.feed(stream[i:i + 1]))
        self.assertEqual(out, [b"\x00\x00\x00", b"\x02\x00\x02"])
        self.assertEqual(dec.bad_frames, 2)


if __name__ == "__main__":
    unittest.main()
