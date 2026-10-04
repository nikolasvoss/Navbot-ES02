#!/usr/bin/env python3
"""Serve the browser page with synthetic rosbridge messages for local checks."""

import base64
import hashlib
import http.server
import json
import select
import socket
import socketserver
import struct
import threading
import time
from pathlib import Path


WEB_DIR = Path(__file__).resolve().parents[2] / "web"
HTTP_HOST = "127.0.0.1"
HTTP_PORT = 8080
WS_HOST = "127.0.0.1"
WS_PORT = 9090
MAGIC = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

state_lock = threading.Lock()
publish_frames = True
publish_status = True
frame_count = 0


def websocket_frame(payload):
    data = payload.encode("utf-8")
    if len(data) < 126:
        header = bytes((0x81, len(data)))
    elif len(data) < 65536:
        header = bytes((0x81, 126)) + struct.pack("!H", len(data))
    else:
        header = bytes((0x81, 127)) + struct.pack("!Q", len(data))
    return header + data


def read_client_frame(client):
    head = client.recv(2)
    if len(head) != 2:
        return None
    first, second = head
    opcode = first & 0x0F
    length = second & 0x7F
    if length == 126:
        length = struct.unpack("!H", client.recv(2))[0]
    elif length == 127:
        length = struct.unpack("!Q", client.recv(8))[0]
    mask = client.recv(4) if second & 0x80 else b""
    payload = bytearray()
    while len(payload) < length:
        chunk = client.recv(length - len(payload))
        if not chunk:
            return None
        payload.extend(chunk)
    if mask:
        for index in range(len(payload)):
            payload[index] ^= mask[index % 4]
    if opcode == 8:
        return None
    if opcode != 1:
        return ""
    return payload.decode("utf-8")


class SyntheticWebSocketHandler(socketserver.BaseRequestHandler):
    def handle(self):
        client = self.request
        client.settimeout(2)
        request = bytearray()
        while b"\r\n\r\n" not in request:
            chunk = client.recv(4096)
            if not chunk:
                return
            request.extend(chunk)
        headers = {}
        for line in request.decode("latin-1").split("\r\n")[1:]:
            if ":" in line:
                key, value = line.split(":", 1)
                headers[key.lower()] = value.strip()
        key = headers.get("sec-websocket-key")
        if not key:
            return
        accept = base64.b64encode(hashlib.sha1((key + MAGIC).encode("ascii")).digest()).decode("ascii")
        client.sendall((
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept}\r\n\r\n"
        ).encode("ascii"))
        client.settimeout(0.05)
        topics = set()
        next_publish = 0.0
        global frame_count
        while True:
            try:
                readable, _, _ = select.select([client], [], [], 0.05)
                if readable:
                    raw = read_client_frame(client)
                    if raw is None:
                        return
                    if raw:
                        try:
                            message = json.loads(raw)
                            if message.get("op") == "subscribe":
                                topics.add(message.get("topic"))
                            elif message.get("op") == "unsubscribe":
                                topics.discard(message.get("topic"))
                        except (json.JSONDecodeError, AttributeError):
                            pass
            except (OSError, TimeoutError):
                return

            now = time.monotonic()
            if now < next_publish:
                continue
            next_publish = now + 0.5
            with state_lock:
                allow_frames = publish_frames
                allow_status = publish_status
                if allow_frames:
                    frame_count += 1
                count = frame_count

            if "/hmmd/rdmap" in topics and allow_frames:
                values = [((index * 37) + (count * 113)) % 5000 for index in range(320)]
                message = {
                    "header": {"stamp": {"sec": int(time.time()), "nanosec": 0}, "frame_id": "synthetic_hmmd"},
                    "doppler_bins": 20,
                    "range_gates": 16,
                    "amplitude_squared": values,
                }
                client.sendall(websocket_frame(json.dumps({"op": "publish", "topic": "/hmmd/rdmap", "msg": message})))

            if "/hmmd/status" in topics and allow_status:
                stale = not allow_frames
                message = {
                    "header": {"stamp": {"sec": int(time.time()), "nanosec": 0}, "frame_id": "synthetic_hmmd"},
                    "status": [{
                        "level": 1 if stale else 0,
                        "name": "hmmd_sensor",
                        "message": "no recent frames" if stale else "receiving frames",
                        "hardware_id": "synthetic",
                        "values": [
                            {"key": "connected", "value": "true"},
                            {"key": "stale", "value": str(stale).lower()},
                            {"key": "frames_received", "value": str(count)},
                            {"key": "frame_rate_hz", "value": "2.000" if allow_frames else "0.000"},
                            {"key": "last_io_error", "value": ""},
                        ],
                    }],
                }
                client.sendall(websocket_frame(json.dumps({"op": "publish", "topic": "/hmmd/status", "msg": message})))


class ThreadedTCPServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def input_controls():
    global publish_frames, publish_status
    print("Synthetic controls: m toggles map frames, s toggles all status, q exits.")
    while True:
        key = input().strip().lower()
        if key == "m":
            with state_lock:
                publish_frames = not publish_frames
                enabled = publish_frames
            print(f"Synthetic HMMD frames {'on' if enabled else 'off'}")
        elif key == "s":
            with state_lock:
                publish_status = not publish_status
                enabled = publish_status
            print(f"Synthetic status {'on' if enabled else 'off'}")
        elif key == "q":
            return


def main():
    handler = lambda *args, **kwargs: http.server.SimpleHTTPRequestHandler(
        *args, directory=str(WEB_DIR), **kwargs
    )
    http_server = http.server.ThreadingHTTPServer((HTTP_HOST, HTTP_PORT), handler)
    ws_server = ThreadedTCPServer((WS_HOST, WS_PORT), SyntheticWebSocketHandler)
    threading.Thread(target=http_server.serve_forever, daemon=True).start()
    threading.Thread(target=ws_server.serve_forever, daemon=True).start()
    print(f"Synthetic page: http://{HTTP_HOST}:{HTTP_PORT}/")
    print(f"Synthetic rosbridge: ws://{WS_HOST}:{WS_PORT}/")
    try:
        input_controls()
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        http_server.shutdown()
        ws_server.shutdown()
        http_server.server_close()
        ws_server.server_close()


if __name__ == "__main__":
    main()
