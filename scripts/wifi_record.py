#!/usr/bin/env python3
"""Record Navbot balance telemetry over the home WLAN."""
from __future__ import annotations

import argparse
import http.client
import ipaddress
import json
import os
from pathlib import Path
import secrets
import socket
import struct
import sys
import threading
import time
import zlib
from typing import Any

import decode_wifi_trace as decoder

API = "/api/v1/recording"
HANDSHAKE_LIMIT = 512


class LocalError(Exception):
    pass


class TransportError(Exception):
    pass


class DeviceError(Exception):
    pass


class IncompleteError(Exception):
    pass


class Client:
    def __init__(self, host: str, timeout: float = 3.0):
        try:
            address = ipaddress.ip_address(host)
        except ValueError as exc:
            raise LocalError("--host must be a robot IPv4 address") from exc
        if not isinstance(address, ipaddress.IPv4Address):
            raise LocalError("only IPv4 home-network connections are supported")
        self.host, self.timeout = str(address), timeout

    def request(self, method: str, endpoint: str,
                payload: dict[str, Any] | None = None) -> dict[str, Any]:
        body = None if payload is None else json.dumps(payload, separators=(",", ":"), allow_nan=False)
        headers = {"Accept": "application/json"}
        if body is not None:
            headers["Content-Type"] = "application/json"
        connection = http.client.HTTPConnection(self.host, 80, timeout=self.timeout)
        try:
            connection.request(method, API + endpoint, body=body, headers=headers)
            response = connection.getresponse()
            raw = response.read(8192)
            data = json.loads(raw)
            if not isinstance(data, dict):
                raise TransportError("device returned a non-object JSON response")
            if response.status >= 400 or data.get("ok") is False:
                raise DeviceError(f"HTTP {response.status}: {data.get('error', 'request rejected')}")
            if response.status != 200:
                raise TransportError(f"unexpected HTTP status {response.status}")
            return data
        except (OSError, TimeoutError, http.client.HTTPException, json.JSONDecodeError) as exc:
            raise TransportError(f"control request failed: {exc}") from exc
        finally:
            connection.close()

    def status(self) -> dict[str, Any]:
        return self.request("GET", "/status")

    def mutate(self, name: str, boot_id: str, recording_id: str | None,
               extra: dict[str, Any] | None = None) -> dict[str, Any]:
        payload = {"request_id": secrets.token_hex(16), "expected_boot_id": boot_id}
        if recording_id is not None:
            payload["recording_id"] = recording_id
        if extra:
            payload.update(extra)
        return self.request("POST", f"/{name}", payload)


def load_config(path: Path, host_override: str | None) -> str:
    try:
        stat = path.stat()
        if stat.st_mode & 0o077:
            raise LocalError(f"config permissions must be 0600: {path}")
        if stat.st_size > 8192:
            raise LocalError("config file exceeds 8 KiB")
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        if host_override:
            return host_override
        raise LocalError(f"config not found: {path} (run wifi_tune.py once or set --host)") from exc
    except (OSError, ValueError) as exc:
        raise LocalError(f"cannot read config: {exc}") from exc
    if not isinstance(data, dict) or not isinstance(data.get("host"), str):
        raise LocalError("config must contain a string field host")
    return host_override or data["host"]


def recv_exact(sock: socket.socket, count: int, output: Any) -> bytes:
    data = bytearray()
    while len(data) < count:
        chunk = sock.recv(count - len(data))
        if not chunk:
            raise IncompleteError(f"peer closed after {len(data)} of {count} bytes")
        output.write(chunk)
        data.extend(chunk)
    return bytes(data)


def check_frame(raw_header: bytes, payload: bytes, expected_id: int,
                expected_frame: int, expected_sample: int, records_crc: int,
                meta: dict[str, Any] | None, end_seen: bool) -> tuple[int, int, dict[str, Any] | None, dict[str, Any] | None]:
    magic, version, kind, rec_id, sequence, size, count, crc = decoder.HEADER.unpack(raw_header)
    if magic != b"NBL1" or version != 1 or rec_id != expected_id or sequence != expected_frame:
        raise ValueError("invalid magic, protocol, recording ID, or frame sequence")
    if zlib.crc32(payload, zlib.crc32(raw_header[:28])) & 0xffffffff != crc:
        raise ValueError("frame CRC mismatch")
    if kind == 1:
        if expected_frame != 0 or meta is not None or count != 0 or size > decoder.MAX_META:
            raise ValueError("invalid META frame")
        parsed_meta = json.loads(payload.decode("utf-8"))
        if not isinstance(parsed_meta, dict):
            raise ValueError("META is not a JSON object")
        return expected_sample, records_crc, parsed_meta, None
    if kind == 2:
        if meta is None or end_seen or count < 1 or count > 32 or size != count * decoder.RECORD_SIZE:
            raise ValueError("invalid DATA frame")
        records_crc = zlib.crc32(payload, records_crc) & 0xffffffff
        for index in range(count):
            record = decoder.parse_record(payload[index * decoder.RECORD_SIZE:(index + 1) * decoder.RECORD_SIZE])
            if record["sequence"] != expected_sample:
                raise ValueError("sample sequence gap")
            expected_sample += 1
        return expected_sample, records_crc, meta, None
    if kind == 3:
        if meta is None or end_seen or count != 0 or size > decoder.MAX_END:
            raise ValueError("invalid END frame")
        parsed_end = json.loads(payload.decode("utf-8"))
        if not isinstance(parsed_end, dict):
            raise ValueError("END is not a JSON object")
        if parsed_end.get("sent_records") != expected_sample or parsed_end.get("records_crc32") != records_crc:
            raise ValueError("END record count or records CRC mismatch")
        if parsed_end.get("generated_records", 0) < expected_sample or parsed_end.get("queued_records", 0) < expected_sample:
            raise ValueError("END generated or queued count is below sent count")
        if parsed_end.get("reason") in ("DURATION", "USER_STOP") and not (
            parsed_end.get("generated_records") == parsed_end.get("queued_records") == expected_sample
        ):
            raise ValueError("successful END record counts disagree")
        return expected_sample, records_crc, meta, parsed_end
    raise ValueError("unsupported frame type")


def receive(sock: socket.socket, output: Any, recording_id: str,
            stop_event: threading.Event) -> dict[str, Any]:
    try:
        numeric_id = int(recording_id, 16)
    except ValueError as exc:
        raise ValueError("device returned a malformed recording ID") from exc
    received = bytearray()
    while len(received) <= HANDSHAKE_LIMIT:
        byte = sock.recv(1)
        if not byte:
            raise DeviceError("data connection closed before handshake response")
        if byte == b"\n":
            break
        received.extend(byte)
    if bytes(received) != b"OK":
        raise DeviceError("device rejected stream ticket")
    expected_frame = expected_sample = records_crc = 0
    meta = None
    while not stop_event.is_set():
        first = sock.recv(1)
        if not first:
            raise IncompleteError("data connection closed before END")
        output.write(first)
        raw_header = first + recv_exact(sock, decoder.HEADER.size - 1, output)
        header = decoder.HEADER.unpack(raw_header)
        kind, size = header[2], header[5]
        if kind not in (1, 2, 3):
            raise ValueError("unsupported frame type")
        if kind == 1 and size > decoder.MAX_META or kind == 2 and size > decoder.MAX_DATA or kind == 3 and size > decoder.MAX_END:
            raise ValueError("payload length exceeds protocol limit")
        payload = recv_exact(sock, size, output)
        expected_sample, records_crc, meta, end = check_frame(
            raw_header, payload, numeric_id, expected_frame, expected_sample,
            records_crc, meta, False)
        expected_frame += 1
        if end is not None:
            output.flush()
            os.fsync(output.fileno())
            return {"end": end, "records": expected_sample, "records_crc32": records_crc}
    raise InterruptedError("receiver stopped")


def write_error(path: Path, message: str, partial_path: Path | None = None) -> None:
    error_path = Path(str(path) + ".error.json")
    details: dict[str, Any] = {"error": message}
    if partial_path and partial_path.exists():
        try:
            with partial_path.open("rb") as stream:
                records, _, end, offset, _ = decoder.decode_stream(stream)
            details["last_valid_offset"] = offset
            details["valid_records_retained"] = len(records)
            details["transport_complete"] = True
            if end:
                details["device_reason"] = end.get("reason")
        except decoder.ProtocolError as exc:
            details["last_valid_offset"] = exc.offset
            details["valid_records_retained"] = len(exc.records)
            details["validation_error"] = str(exc)
        except OSError as exc:
            details["partial_inspection_error"] = str(exc)
    error_path.write_text(json.dumps(details, indent=2) + "\n", encoding="utf-8")


def publish_partial(partial_path: Path, final_path: Path) -> None:
    os.link(partial_path, final_path)
    try:
        partial_path.unlink()
    except OSError:
        try:
            final_path.unlink()
        except OSError:
            pass
        raise


def run_record(client: Client, seconds: int, output_path: Path, force: bool) -> int:
    final_path = output_path
    partial_path = Path(str(final_path) + ".partial")
    error_path = Path(str(final_path) + ".error.json")
    if not final_path.parent.is_dir():
        raise LocalError(f"output directory does not exist: {final_path.parent}")
    if not force and any(path.exists() for path in (final_path, partial_path, error_path)):
        raise LocalError("output or partial file already exists; pass --force to replace it")
    if force:
        for path in (final_path, partial_path, error_path):
            try:
                path.unlink()
            except FileNotFoundError:
                pass
    try:
        file = partial_path.open("xb")
    except FileExistsError as exc:
        raise LocalError(f"partial file already exists: {partial_path}") from exc

    sock = None
    thread_error: list[BaseException] = []
    result: list[dict[str, Any]] = []
    stop_event = threading.Event()
    receiver_thread = None
    session: dict[str, Any] | None = None
    renamed = False
    try:
        status = client.status()
        boot_id = status.get("boot_id")
        if not isinstance(boot_id, str) or not boot_id:
            raise TransportError("status response has no boot_id")
        prepared = client.mutate("prepare", boot_id, None, {"duration_s": seconds, "profile": "balance_v1"})
        session = prepared
        recording_id = prepared.get("recording_id")
        ticket = prepared.get("stream_ticket")
        port = prepared.get("port")
        if not isinstance(recording_id, str) or not isinstance(ticket, str) or not isinstance(port, int):
            raise TransportError("prepare response is missing recording_id, stream_ticket, or port")
        sock = socket.create_connection((client.host, port), timeout=3.0)
        sock.settimeout(10.0)
        handshake = json.dumps({"protocol": 1, "boot_id": boot_id,
                                "recording_id": recording_id, "ticket": ticket}, separators=(",", ":")).encode() + b"\n"
        if len(handshake) > HANDSHAKE_LIMIT:
            raise TransportError("stream handshake exceeds 512 bytes")
        sock.sendall(handshake)

        def reader() -> None:
            try:
                result.append(receive(sock, file, recording_id, stop_event))
            except (ValueError, struct.error, json.JSONDecodeError) as exc:
                thread_error.append(IncompleteError(str(exc)))
            except BaseException as exc:
                thread_error.append(exc)

        receiver_thread = threading.Thread(target=reader, name="navbot-recorder", daemon=True)
        receiver_thread.start()
        ready_deadline = time.monotonic() + 3.0
        while time.monotonic() < ready_deadline:
            if thread_error:
                raise thread_error[0]
            live = client.status()
            if live.get("stream_ready"):
                break
            time.sleep(0.05)
        else:
            raise DeviceError("data stream did not become ready")
        try:
            started = client.mutate("start", boot_id, recording_id)
        except (TransportError, DeviceError):
            observed = client.status()
            if observed.get("recording_id") != recording_id or observed.get("state") not in ("RECORDING", "DRAINING", "AWAIT_ACK", "COMPLETE"):
                raise
            started = observed
        deadline = time.monotonic() + seconds + 15
        while receiver_thread.is_alive() and time.monotonic() < deadline:
            receiver_thread.join(timeout=0.1)
        if receiver_thread.is_alive():
            raise TransportError("timed out waiting for END frame")
        if thread_error:
            raise thread_error[0]
        if not result:
            raise TransportError("data stream ended without a validated footer")
        end = result[0]["end"]
        if end.get("reason") not in ("DURATION", "USER_STOP"):
            raise TransportError(f"device ended recording with {end.get('reason')}")
        file.close()
        publish_partial(partial_path, final_path)
        renamed = True
        dir_fd = os.open(final_path.parent, os.O_RDONLY)
        try:
            os.fsync(dir_fd)
        finally:
            os.close(dir_fd)
        ack_confirmed = True
        try:
            client.mutate("ack", boot_id, recording_id,
                          {"received_records": result[0]["records"],
                           "records_crc32": result[0]["records_crc32"]})
        except (TransportError, DeviceError):
            ack_confirmed = False
        if end.get("reason") == "USER_STOP":
            state = "acknowledged" if ack_confirmed else "ACK unconfirmed"
            print(f"Stopped recording saved and verified ({state}): {final_path} ({result[0]['records']} records)")
            return 5
        state = "acknowledged" if ack_confirmed else "ACK unconfirmed"
        print(f"Recording saved and verified ({state}): {final_path} ({result[0]['records']} records)")
        return 0
    except KeyboardInterrupt:
        if session:
            try:
                client.mutate("stop", status.get("boot_id", ""), session.get("recording_id", ""))
            except Exception:
                pass
        if receiver_thread:
            receiver_thread.join(timeout=5)
        if result and receiver_thread and not receiver_thread.is_alive() and not thread_error and not file.closed:
            try:
                file.close()
                publish_partial(partial_path, final_path)
                if session and status.get("boot_id"):
                    client.mutate("ack", status["boot_id"], session["recording_id"],
                                  {"received_records": result[0]["records"],
                                   "records_crc32": result[0]["records_crc32"]})
                print(f"Stopped recording saved and verified: {final_path} ({result[0]['records']} records)")
                return 5
            except Exception as exc:
                write_error(final_path, f"stop completed but finalization failed: {exc}", partial_path)
        write_error(final_path, "interrupted by user; partial file retained", partial_path)
        print(f"Interrupted; partial data retained at {partial_path}", file=sys.stderr)
        return 5
    except (OSError, EOFError, ValueError, TransportError, DeviceError, IncompleteError) as exc:
        if renamed:
            print(f"Recording file is verified at {final_path}, but completion confirmation failed: {exc}", file=sys.stderr)
            return 0
        if session:
            try:
                client.mutate("stop", status.get("boot_id", ""), session.get("recording_id", ""))
            except Exception:
                pass
        stop_event.set()
        if sock:
            sock.close()
        if receiver_thread:
            receiver_thread.join(timeout=1)
        if not file.closed:
            try:
                file.flush()
                os.fsync(file.fileno())
            except OSError:
                pass
            file.close()
        write_error(final_path, str(exc), partial_path)
        print(f"Recording failed; partial file retained at {partial_path}: {exc}", file=sys.stderr)
        if isinstance(exc, IncompleteError):
            return 5
        return 3 if isinstance(exc, (OSError, EOFError, TransportError)) else 4
    finally:
        stop_event.set()
        if sock:
            sock.close()
        if not file.closed:
            file.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", help="robot IPv4 address; overrides --config")
    parser.add_argument("--config", type=Path, default=Path.home() / ".config/navbot/wifi-tuning.json")
    parser.add_argument("--seconds", type=int, default=30)
    parser.add_argument("--output", type=Path, default=Path("recording.nblog"))
    parser.add_argument("--force", action="store_true", help="replace existing output and partial files")
    parser.add_argument("--status", action="store_true", help="show device recording status")
    args = parser.parse_args(argv)
    if not args.status and not 20 <= args.seconds <= 40:
        print("--seconds must be between 20 and 40", file=sys.stderr)
        return 2
    try:
        host = load_config(args.config, args.host)
        client = Client(host)
        if args.status:
            print(json.dumps(client.status(), indent=2))
            return 0
        return run_record(client, args.seconds, args.output, args.force)
    except LocalError as exc:
        print(f"configuration error: {exc}", file=sys.stderr)
        return 2
    except DeviceError as exc:
        print(f"device refused recording: {exc}", file=sys.stderr)
        return 4
    except TransportError as exc:
        print(f"transport failure: {exc}", file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
