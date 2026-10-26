import random

from navbot_link_bridge.framing import FrameDecoder, cobs_decode, cobs_encode, crc16, encode_frame


def test_crc_check_value():
    assert crc16(b"123456789") == 0x29B1


def test_cobs_known_vectors():
    assert cobs_encode(b"\x00") == b"\x01\x01"
    assert cobs_encode(b"\x11\x22\x00\x33") == b"\x03\x11\x22\x02\x33"
    assert cobs_encode(bytes(range(1, 255))) == b"\xff" + bytes(range(1, 255))


def test_cobs_roundtrip():
    rng = random.Random(1)
    for n in (0, 1, 2, 253, 254, 255, 300):
        data = bytes(rng.choice([0, 1, 255, rng.randrange(256)]) for _ in range(n))
        encoded = cobs_encode(data)
        assert 0 not in encoded
        assert cobs_decode(encoded) == data


def test_frames_resync_after_garbage():
    good = [encode_frame(bytes([i, 0, i])) for i in range(3)]
    bad = bytearray(good[1])
    bad[2] ^= 0x40
    stream = b"\x07junk\x00" + good[0] + bytes(bad) + good[2]
    decoder = FrameDecoder()
    out = []
    for i in range(len(stream)):  # byte-at-a-time
        out += decoder.feed(stream[i:i + 1])
    assert out == [b"\x00\x00\x00", b"\x02\x00\x02"]
    assert decoder.bad_frames == 2
