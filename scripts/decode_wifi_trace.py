#!/usr/bin/env python3
"""Validate and decode a Navbot WLAN recording into CSV and a JSON report."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import struct
import sys
import zlib
from typing import Any

HEADER = struct.Struct("<4sHHQIIII")
RECORD = struct.Struct("<QIIII26f")
RECORD_SIZE = 128
MAX_META = 8192
MAX_END = 4096
MAX_DATA = 32 * RECORD_SIZE
NAMES = (
    "roll_ok", "BodyPitching_f", "gyro_x", "gyro_y", "gyro_z", "MovementSpeed",
    "BodyTurn", "Motor1_Velocity_f", "Motor2_Velocity_f", "Speed_Pid.error",
    "Speed_Pid.outP", "Speed_Pid.outI", "Speed_Pid.outD", "Speed_Pid.output",
    "Angle_Pid.error", "Angle_Pid.outP", "Angle_Pid.outI", "Angle_Pid.outD",
    "Angle_Pid.output", "Yaw_Pid.output", "BodyX", "motor1.target", "motor2.target",
    "wheelSpeedFeedbackOutput", "Voltage", "wheelSpeedFeedbackGain",
)


class ProtocolError(ValueError):
    def __init__(self, message: str, offset: int, records: list[dict[str, Any]],
                 meta: dict[str, Any] | None, end: dict[str, Any] | None):
        super().__init__(message)
        self.offset, self.records, self.meta, self.end = offset, records, meta, end
        self.crc_failure = "crc" in message.lower()
        self.sequence_failure = "sequence" in message.lower()


def read_exact(stream: Any, length: int) -> bytes:
    parts = bytearray()
    while len(parts) < length:
        chunk = stream.read(length - len(parts))
        if not chunk:
            raise EOFError(f"truncated after {len(parts)} of {length} bytes")
        parts.extend(chunk)
    return bytes(parts)


def parse_record(raw: bytes) -> dict[str, Any]:
    values = RECORD.unpack(raw)
    result: dict[str, Any] = {
        "timestamp_us": values[0], "sequence": values[1], "flags": values[2],
        "imu_age_us": values[3], "control_dt_us": values[4],
    }
    result.update(zip(NAMES, values[5:]))
    return result


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"invalid JSON constant {value}")


def parse_json_object(payload: bytes, frame_name: str) -> dict[str, Any]:
    value = json.loads(payload.decode("utf-8"), parse_constant=_reject_json_constant)
    if not isinstance(value, dict):
        raise ValueError(f"{frame_name} must be a JSON object")
    return value


def decode_stream(stream: Any) -> tuple[list[dict[str, Any]], dict[str, Any] | None,
                                          dict[str, Any] | None, int, bool]:
    records: list[dict[str, Any]] = []
    meta = end = None
    offset = 0
    frame_sequence = 0
    recording_id = None
    records_crc = 0
    expected_sample_sequence = 0
    seen_end = False
    try:
        while True:
            first = stream.read(1)
            if not first:
                break
            raw_header = first + read_exact(stream, HEADER.size - 1)
            magic, version, kind, rec_id, sequence, size, count, crc = HEADER.unpack(raw_header)
            if magic != b"NBL1" or version != 1 or kind not in (1, 2, 3):
                raise ValueError("invalid magic, version, or frame type")
            if sequence != frame_sequence:
                raise ValueError(f"frame sequence {sequence}, expected {frame_sequence}")
            frame_sequence += 1
            if recording_id is None:
                recording_id = rec_id
            if rec_id != recording_id:
                raise ValueError("recording ID changed within stream")
            if kind == 1 and (meta is not None or frame_sequence != 1 or count != 0 or size > MAX_META):
                raise ValueError("invalid META position or length")
            if kind == 2 and (meta is None or end is not None or count < 1 or count > 32 or size != count * RECORD_SIZE):
                raise ValueError("invalid DATA position, count, or length")
            if kind == 3 and (meta is None or end is not None or count != 0 or size > MAX_END):
                raise ValueError("invalid END position or length")
            if kind == 1 and size == 0 or kind == 3 and size == 0:
                raise ValueError("empty JSON frame")
            if size > MAX_DATA and kind == 2:
                raise ValueError("DATA payload too large")
            payload = read_exact(stream, size)
            if zlib.crc32(payload, zlib.crc32(raw_header[:28])) & 0xffffffff != crc:
                raise ValueError("frame CRC mismatch")
            if kind == 1:
                meta = parse_json_object(payload, "META")
            elif kind == 2:
                records_crc = zlib.crc32(payload, records_crc) & 0xffffffff
                frame_records = []
                for index in range(count):
                    record = parse_record(payload[index * RECORD_SIZE:(index + 1) * RECORD_SIZE])
                    if record["sequence"] != expected_sample_sequence:
                        raise ValueError(f"sample sequence {record['sequence']}, expected {expected_sample_sequence}")
                    expected_sample_sequence += 1
                    frame_records.append(record)
                records.extend(frame_records)
            else:
                candidate_end = parse_json_object(payload, "END")
                if candidate_end.get("sent_records") != expected_sample_sequence:
                    raise ValueError("END sent record count disagrees with DATA frames")
                if candidate_end.get("generated_records", 0) < expected_sample_sequence or candidate_end.get("queued_records", 0) < expected_sample_sequence:
                    raise ValueError("END generated or queued count is below sent records")
                if candidate_end.get("reason") in ("DURATION", "USER_STOP") and not (
                    candidate_end.get("generated_records") == candidate_end.get("queued_records") == expected_sample_sequence
                ):
                    raise ValueError("successful END record counts disagree")
                if candidate_end.get("records_crc32") != records_crc:
                    raise ValueError("END records_crc32 mismatch")
                end = candidate_end
                seen_end = True
            offset += HEADER.size + size
            if seen_end and stream.read(1):
                raise ValueError("trailing bytes after END")
            if seen_end:
                break
    except (EOFError, ValueError, UnicodeDecodeError, json.JSONDecodeError, struct.error) as exc:
        raise ProtocolError(str(exc), offset, records, meta, end) from exc
    if meta is None:
        raise ProtocolError("META frame missing", offset, records, meta, end)
    if not seen_end:
        raise ProtocolError("END frame missing", offset, records, meta, end)
    return records, meta, end, offset, False


def percentile(values: list[float], p: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * p
    low = math.floor(index)
    high = math.ceil(index)
    return ordered[low] + (ordered[high] - ordered[low]) * (index - low)


def summarize(records: list[dict[str, Any]], meta: dict[str, Any] | None,
              end: dict[str, Any] | None, partial: bool,
              crc_failures: int = 0, sequence_failures: int = 0) -> dict[str, Any]:
    deltas = [(b["timestamp_us"] - a["timestamp_us"]) / 1_000_000
              for a, b in zip(records, records[1:]) if b["timestamp_us"] > a["timestamp_us"]]
    active = [r for r in records if r["flags"] & 3 and r["flags"] & (1 << 3) and not r["flags"] & (1 << 2)]
    active_flags = [bool(r["flags"] & 3 and r["flags"] & (1 << 3) and not r["flags"] & (1 << 2)) for r in records]
    active_segments = sum(is_active and (index == 0 or not active_flags[index - 1])
                          for index, is_active in enumerate(active_flags))
    inactive_segments = sum((not is_active) and (index == 0 or active_flags[index - 1])
                            for index, is_active in enumerate(active_flags))
    tilt = [r["roll_ok"] for r in active if math.isfinite(r["roll_ok"])]
    numeric_faults = sum(bool(r["flags"] & (1 << 6)) for r in records)
    invalid_active = sum(
        not all(math.isfinite(r[name]) for name in NAMES[:9]) or
        (r["flags"] & (1 << 3) and not all(math.isfinite(r[name]) for name in NAMES[9:20]))
        for r in active
    )
    return {
        "complete": not partial and bool(end) and end.get("reason") == "DURATION",
        "transport_complete": not partial and bool(end),
        "partial": partial,
        "reason": end.get("reason") if end else "INCOMPLETE",
        "requested_duration_s": meta.get("requested_duration_s") if meta else None,
        "observed_duration_s": (records[-1]["timestamp_us"] - records[0]["timestamp_us"]) / 1e6 if len(records) > 1 else 0,
        "record_count": len(records),
        "dt_seconds": {"median": percentile(deltas, .5), "p95": percentile(deltas, .95),
                       "p99": percentile(deltas, .99), "max": max(deltas) if deltas else None},
        "sequence_failures": sequence_failures,
        "crc_failures": crc_failures,
        "numeric_fault_records": numeric_faults,
        "invalid_active_records": invalid_active,
        "ring_high_water": end.get("ring_high_water") if end else None,
        "active_records": len(active), "inactive_records": len(records) - len(active),
        "active_segments": active_segments, "inactive_segments": inactive_segments,
        "active_tilt_roll_ok": {"min": min(tilt) if tilt else None,
                                 "max": max(tilt) if tilt else None,
                                 "rms": math.sqrt(sum(v * v for v in tilt) / len(tilt)) if tilt else None},
        "metadata": meta or {}, "end": end or {},
    }


def write_outputs(records: list[dict[str, Any]], meta: dict[str, Any] | None,
                  end: dict[str, Any] | None, partial: bool, csv_path: Path,
                  report_path: Path, crc_failures: int = 0,
                  sequence_failures: int = 0) -> None:
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        columns = ["timestamp_us", "relative_s", "sequence", "flags", "imu_age_us", "control_dt_us", *NAMES]
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        origin = records[0]["timestamp_us"] if records else 0
        for record in records:
            row = dict(record)
            row["relative_s"] = (record["timestamp_us"] - origin) / 1_000_000
            writer.writerow(row)
    report = summarize(records, meta, end, partial, crc_failures, sequence_failures)
    report_path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--allow-partial", action="store_true")
    args = parser.parse_args(argv)
    try:
        with args.input.open("rb") as stream:
            records, meta, end, _, _ = decode_stream(stream)
        write_outputs(records, meta, end, False, args.csv, args.report)
        reason = end.get("reason")
        return 0 if reason == "DURATION" else 5
    except ProtocolError as exc:
        if not args.allow_partial:
            print(f"incomplete/corrupt at byte {exc.offset}: {exc}", file=sys.stderr)
            return 5
        write_outputs(exc.records, exc.meta, exc.end, True, args.csv, args.report,
                      int(exc.crc_failure), int(exc.sequence_failure))
        print(f"partial prefix retained at byte {exc.offset}: {exc}", file=sys.stderr)
        return 5
    except OSError as exc:
        print(f"input/output error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
