import random

from navbot_link.framing import FrameDecoder, Text, cobs_decode, cobs_encode, crc16, encode_frame


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


# USB framing: the firmware writes 0x00 before each frame, so text between
# frames arrives as its own delimited chunk.
FRAME_SEQ_7 = b"\x05\x08\x07\x41\xe4\x00"  # payload 08 07: Message{seq: 7}


def test_usb_text_between_frames_passes_through():
    stream = b"\x00" + FRAME_SEQ_7 + b"rst:0x1 (POWERON),boot:0x8\r\n" + b"\x00" + FRAME_SEQ_7
    decoder = FrameDecoder()
    assert decoder.feed(stream) == [b"\x08\x07", Text("rst:0x1 (POWERON),boot:0x8\r\n"), b"\x08\x07"]
    assert decoder.bad_frames == 0


def test_text_without_delimiter_appears_when_line_goes_idle():
    decoder = FrameDecoder()
    assert decoder.feed(b"Guru Meditation Error: Core 1 panic'ed\r\n") == []
    assert decoder.flush() == [Text("Guru Meditation Error: Core 1 panic'ed\r\n")]
    assert decoder.flush() == []


def test_long_text_line_is_not_dropped():
    line = b"x" * 300 + b"\n"
    decoder = FrameDecoder()
    out = decoder.feed(line + b"\x00") + decoder.flush()
    assert "".join(t.text for t in out) == line.decode()
    assert decoder.bad_frames == 0


def test_binary_garbage_is_counted_not_shown():
    decoder = FrameDecoder()
    assert decoder.feed(b"\x01\xff\x80\x00") == []
    assert decoder.bad_frames == 1
