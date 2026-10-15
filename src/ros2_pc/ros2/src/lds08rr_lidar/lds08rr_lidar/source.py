from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from threading import Event, Lock
from typing import Literal

from lds2d import Lidar

Mode = Literal["demo", "serial"]


@dataclass(frozen=True)
class SourceConfig:
    mode: Mode = "demo"
    device: Path | None = None
    baud: int = 115200

    def __post_init__(self) -> None:
        if self.mode == "demo" and self.device is not None:
            raise ValueError("demo mode does not take a device")
        if self.mode == "serial" and self.device is None:
            raise ValueError("serial mode requires a device")


class ByteActivity:
    def __init__(self) -> None:
        self._lock = Lock()
        self._count = 0
        self._last_at: float | None = None

    def start_session(self) -> None:
        with self._lock:
            self._count = 0
            self._last_at = None

    def record(self, count: int) -> None:
        if count:
            with self._lock:
                self._count += count
                self._last_at = time.monotonic()

    def snapshot(self) -> tuple[int, float | None]:
        with self._lock:
            return self._count, self._last_at


class ObservedTransport:
    def __init__(self, transport, activity: ByteActivity, stop: Event) -> None:
        self._transport = transport
        self._activity = activity
        self._stop = stop

    def read(self, size: int) -> bytes:
        if self._stop.is_set():
            raise RuntimeError("LiDAR reader stopped")
        data = self._transport.read(size)
        if self._stop.is_set():
            raise RuntimeError("LiDAR reader stopped")
        self._activity.record(len(data))
        return data

    def write(self, data: bytes) -> int:
        return self._transport.write(data)

    def close(self) -> None:
        self._transport.close()


def open_driver(config: SourceConfig, activity: ByteActivity, stop: Event):
    if config.mode == "demo":
        from lds2d.demo import DemoLidar

        return DemoLidar()

    import serial

    port = serial.Serial(
        port=str(config.device), baudrate=config.baud, timeout=0.2, exclusive=True
    )
    transport = ObservedTransport(port, activity, stop)
    try:
        return Lidar.open("3IROBOTIX-LDS08RR-CAMSENSE", transport=transport)
    except Exception:
        transport.close()
        raise
