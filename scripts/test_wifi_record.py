import contextlib
import io
import json
import os
from pathlib import Path
import stat
import struct
import tempfile
import threading
import unittest
from unittest import mock
import zlib

import wifi_record as recorder


def frame(kind, sequence, payload, count=0, rec_id=17):
    prefix = struct.pack("<4sHHQIII", b"NBL1", 1, kind, rec_id, sequence, len(payload), count)
    crc = zlib.crc32(payload, zlib.crc32(prefix)) & 0xffffffff
    return prefix + struct.pack("<I", crc) + payload


def recording():
    meta = json.dumps({"schema": "balance_v1"}).encode()
    sample = struct.pack("<QIIII26f", 1, 0, 0x0b, 2, 3, *([0.0] * 26))
    end = json.dumps({"reason": "DURATION", "generated_records": 1, "queued_records": 1, "sent_records": 1,
                      "records_crc32": zlib.crc32(sample) & 0xffffffff}).encode()
    return frame(1, 0, meta) + frame(2, 1, sample, 1) + frame(3, 2, end)


class FakeSocket:
    def __init__(self, data):
        self.data = bytearray(data)
    def recv(self, count):
        if not self.data:
            return b""
        result = bytes(self.data[:min(count, 1)])
        del self.data[:len(result)]
        return result
    def sendall(self, data):
        pass
    def settimeout(self, timeout):
        pass
    def close(self):
        pass


class RecorderTests(unittest.TestCase):
    def test_status_mode_prints_device_state_without_starting_recording(self):
        output = io.StringIO()
        with mock.patch.object(recorder, "load_config", return_value="127.0.0.1"), \
             mock.patch.object(recorder.Client, "status", return_value={"ok": True, "state": "IDLE"}), \
             contextlib.redirect_stdout(output):
            result = recorder.main(["--status", "--seconds", "99"])
        self.assertEqual(result, 0)
        self.assertEqual(json.loads(output.getvalue())["state"], "IDLE")

    def test_publish_refuses_to_overwrite_existing_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            partial, final = root / "trial.nblog.partial", root / "trial.nblog"
            partial.write_bytes(b"verified")
            final.write_bytes(b"existing")
            with self.assertRaises(FileExistsError):
                recorder.publish_partial(partial, final)
            self.assertEqual(partial.read_bytes(), b"verified")
            self.assertEqual(final.read_bytes(), b"existing")

    def test_receive_writes_exact_frames_without_handshake(self):
        data = recording()
        with tempfile.TemporaryFile() as output:
            result = recorder.receive(FakeSocket(b"OK\n" + data), output, "0000000000000011", threading.Event())
            output.seek(0)
            self.assertEqual(output.read(), data)
        self.assertEqual(result["records"], 1)
        self.assertEqual(result["end"]["reason"], "DURATION")

    def test_bad_crc_keeps_partial_bytes(self):
        data = bytearray(recording())
        data[-1] ^= 1
        output = io.BytesIO()
        with self.assertRaises(ValueError):
            recorder.receive(FakeSocket(b"OK\n" + data), output, "0000000000000011", threading.Event())
        self.assertEqual(output.getvalue(), bytes(data))

    def test_config_permissions_and_ignores_legacy_token(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            path.write_text(json.dumps({"host": "127.0.0.1", "token": "x" * 32}))
            path.chmod(0o600)
            self.assertEqual(recorder.load_config(path, None), "127.0.0.1")
            path.chmod(0o644)
            with self.assertRaises(recorder.LocalError):
                recorder.load_config(path, None)
        with self.assertRaises(recorder.LocalError):
            recorder.Client("::1")

    def test_prepare_request_has_no_recording_id_or_auth_header(self):
        original = recorder.http.client.HTTPConnection
        calls = []
        class Response:
            status = 200
            def read(self, count): return b'{"ok":true}'
        class Connection:
            def __init__(self, host, port, timeout=None): pass
            def request(self, method, path, body=None, headers=None): calls.append((method, path, body, headers))
            def getresponse(self): return Response()
            def close(self): pass
        recorder.http.client.HTTPConnection = Connection
        try:
            client = recorder.Client("127.0.0.1")
            client.mutate("prepare", "deadbeef", None, {"duration_s": 30, "profile": "balance_v1"})
        finally:
            recorder.http.client.HTTPConnection = original
        method, path, body, headers = calls[0]
        self.assertEqual(method, "POST")
        self.assertEqual(path, "/api/v1/recording/prepare")
        self.assertNotIn("recording_id", json.loads(body))
        self.assertNotIn("Authorization", headers)

    def test_invalid_utf8_http_response_is_a_transport_error(self):
        class Response:
            status = 200
            def read(self, count): return b"\xff"
        class Connection:
            def __init__(self, host, port, timeout=None): pass
            def request(self, method, path, body=None, headers=None): pass
            def getresponse(self): return Response()
            def close(self): pass
        with mock.patch.object(recorder.http.client, "HTTPConnection", Connection):
            with self.assertRaises(recorder.TransportError):
                recorder.Client("127.0.0.1").status()

    def test_directory_fsync_failure_does_not_report_success(self):
        with tempfile.TemporaryDirectory() as directory:
            output_path = Path(directory) / "trial.nblog"
            client = mock.Mock()
            client.status.side_effect = [
                {"boot_id": "boot"},
                {"stream_ready": True},
            ]
            client.mutate.side_effect = [
                {"recording_id": "0000000000000011", "stream_ticket": "ticket", "port": 1234},
                {"state": "RECORDING"},
            ]
            def fsync(fd):
                if stat.S_ISDIR(os.fstat(fd).st_mode):
                    raise OSError("directory sync failed")
            stderr = io.StringIO()
            with mock.patch.object(recorder.socket, "create_connection", return_value=FakeSocket(b"OK\n" + recording())), \
                 mock.patch.object(recorder.os, "fsync", side_effect=fsync), \
                 contextlib.redirect_stderr(stderr):
                result = recorder.run_record(client, 20, output_path, False)
            self.assertEqual(result, 3)
            self.assertTrue(output_path.exists())
            self.assertIn("durability is unconfirmed", stderr.getvalue())
            self.assertNotIn("saved and verified", stderr.getvalue())
            self.assertEqual([call.args[0] for call in client.mutate.call_args_list], ["prepare", "start"])


if __name__ == "__main__":
    unittest.main()
