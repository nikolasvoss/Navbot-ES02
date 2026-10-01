import io
import json
from pathlib import Path
import struct
import tempfile
import unittest
import zlib
from unittest import mock

import decode_wifi_trace as decoder


def frame(kind, sequence, payload, count=0, recording_id=0x0102030405060708):
    prefix = struct.pack("<4sHHQIII", b"NBL1", 1, kind, recording_id, sequence, len(payload), count)
    checksum = zlib.crc32(payload, zlib.crc32(prefix)) & 0xffffffff
    return prefix + struct.pack("<I", checksum) + payload


def recording(reason="DURATION"):
    meta = json.dumps({"schema": "balance_v1", "requested_duration_s": 30}).encode()
    record = decoder.RECORD.pack(100, 0, 0x0b, 12, 1664, *([1.0] * 26))
    payload = record
    end = json.dumps({"reason": reason, "generated_records": 1, "queued_records": 1, "sent_records": 1,
                      "records_crc32": zlib.crc32(payload) & 0xffffffff,
                      "ring_high_water": 1}).encode()
    return frame(1, 0, meta) + frame(2, 1, payload, 1) + frame(3, 2, end)


class Fragmented:
    def __init__(self, data, maximum):
        self.stream, self.maximum = io.BytesIO(data), maximum
    def read(self, size=-1):
        return self.stream.read(min(size, self.maximum) if size >= 0 else self.maximum)


class DecoderTests(unittest.TestCase):
    def test_shared_golden_fixture_matches_record_and_meta_header(self):
        fixture = dict(line.split("=", 1) for line in
                       (Path(__file__).parent.parent / "tests/fixtures/telemetry-v1.hex").read_text().splitlines())
        record = bytearray(decoder.RECORD.pack(0x0102030405060708, 0x11223344, 0x55667788,
                                               0x99aabbcc, 0xddeeff00, 1.0, float("nan"), *([0.0] * 24)))
        struct.pack_into("<I", record, 28, 0x7fc00000)
        self.assertEqual(record.hex(), fixture["record"])
        self.assertEqual(frame(1, 0, b"{}", recording_id=7).hex(), fixture["meta_frame"])

    def test_fragmented_and_coalesced_frames_validate(self):
        for source in (io.BytesIO(recording()), Fragmented(recording(), 1), Fragmented(recording(), 7)):
            records, meta, end, _, partial = decoder.decode_stream(source)
            self.assertFalse(partial)
            self.assertEqual(len(records), 1)
            self.assertEqual(records[0]["sequence"], 0)
            self.assertEqual(meta["schema"], "balance_v1")
            self.assertEqual(end["reason"], "DURATION")

    def test_corruption_and_truncation_stop_at_last_good_frame(self):
        content = bytearray(recording())
        content[-1] ^= 1
        with self.assertRaises(decoder.ProtocolError) as raised:
            decoder.decode_stream(io.BytesIO(content))
        self.assertEqual(len(raised.exception.records), 1)
        self.assertIsNotNone(raised.exception.meta)
        with self.assertRaises(decoder.ProtocolError) as raised:
            decoder.decode_stream(io.BytesIO(recording()[:-4]))
        self.assertEqual(len(raised.exception.records), 1)

    def test_invalid_sequence_or_order_is_rejected_without_rescan(self):
        content = recording()
        bad = frame(2, 2, decoder.RECORD.pack(100, 0, 0, 0, 0, *([0.0] * 26)), 1)
        meta_size = len(frame(1, 0, json.dumps({"schema": "balance_v1", "requested_duration_s": 30}).encode()))
        with self.assertRaises(decoder.ProtocolError):
            decoder.decode_stream(io.BytesIO(content[:meta_size] + bad))

    def test_user_stop_is_a_valid_but_short_run(self):
        records, meta, end, _, _ = decoder.decode_stream(io.BytesIO(recording("USER_STOP")))
        report = decoder.summarize(records, meta, end, False)
        self.assertFalse(report["complete"])
        self.assertTrue(report["transport_complete"])
        self.assertEqual(report["reason"], "USER_STOP")
        self.assertEqual(report["record_count"], 1)

    def test_nan_in_meta_is_reported_as_corrupt_input(self):
        content = recording()
        meta_size = len(frame(1, 0, json.dumps({"schema": "balance_v1", "requested_duration_s": 30}).encode()))
        invalid_meta = frame(1, 0, b'{"schema":"balance_v1","unexpected":NaN}')
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "input.nblog"
            source.write_bytes(invalid_meta + content[meta_size:])
            report = Path(directory) / "report.json"
            with mock.patch("sys.stderr", new_callable=io.StringIO) as stderr:
                result = decoder.main([str(source), "--csv", str(Path(directory) / "out.csv"),
                                       "--report", str(report)])
            self.assertEqual(result, 5)
            self.assertIn("invalid JSON constant NaN", stderr.getvalue())
            self.assertFalse(report.exists())


if __name__ == "__main__":
    unittest.main()
