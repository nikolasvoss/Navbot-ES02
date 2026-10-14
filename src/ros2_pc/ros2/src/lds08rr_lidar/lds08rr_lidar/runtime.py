from __future__ import annotations

import time
from dataclasses import asdict, dataclass
from threading import Event, Lock, Thread
from typing import Callable

from lds2d import Scan
from lds2d.viz import ScanBuffer

from .source import ByteActivity, SourceConfig, open_driver

FRESH_FOR_SEC = 2.0
INVALID_AFTER_SEC = 2.0
RETRY_AFTER_SEC = 1.0


@dataclass(frozen=True)
class LidarStatus:
    state: str
    mode: str
    device: str | None
    protocol: str | None
    bytes_received: int
    scans_received: int
    last_scan_age_sec: float | None
    scan_rate_hz: float | None
    valid_points: int
    last_error: str | None


class LidarRuntime:
    def __init__(
        self,
        config: SourceConfig,
        on_scan: Callable[[Scan, float], None] | None = None,
        driver_factory=open_driver,
    ) -> None:
        self.config = config
        self.on_scan = on_scan
        self.driver_factory = driver_factory
        self.scan_buffer = ScanBuffer()
        self.byte_activity = ByteActivity()
        self._lock = Lock()
        self._stop = Event()
        self._thread: Thread | None = None
        self._driver = None
        self._driver_lock = Lock()
        self._opened_at: float | None = None
        self._last_scan_at: float | None = None
        self._last_scan: Scan | None = None
        self._scan_rate_hz: float | None = None
        self._scans_received = 0
        self._last_error: str | None = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            raise RuntimeError("LiDAR reader is already running")
        self._stop.clear()
        self._thread = Thread(target=self._read_loop, name="lds08rr-reader", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        with self._driver_lock:
            driver = self._driver
        if driver is not None:
            try:
                driver.close()
            except Exception:
                pass
        if self._thread:
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():
                raise RuntimeError("LiDAR reader did not stop within 2 seconds")
            self._thread = None

    def snapshot(self, now: float | None = None) -> LidarStatus:
        current = time.monotonic() if now is None else now
        byte_count, last_byte_at = self.byte_activity.snapshot()
        with self._lock:
            last_scan_at = self._last_scan_at
            opened_at = self._opened_at
            scan = self._last_scan
            rate = self._scan_rate_hz
            count = self._scans_received
            error = self._last_error
        age = None if last_scan_at is None else max(0.0, current - last_scan_at)

        if error:
            state = "error"
        elif age is not None:
            if age > FRESH_FOR_SEC:
                state = "stale"
            else:
                state = "demo" if self.config.mode == "demo" else "live"
        elif byte_count and opened_at is not None and current - opened_at > INVALID_AFTER_SEC:
            state = "invalid"
        elif last_byte_at is not None and current - last_byte_at > INVALID_AFTER_SEC:
            state = "invalid"
        else:
            state = "waiting"

        return LidarStatus(
            state=state,
            mode=self.config.mode,
            device=None if self.config.device is None else str(self.config.device),
            protocol=self.config.protocol,
            bytes_received=byte_count,
            scans_received=count,
            last_scan_age_sec=age,
            scan_rate_hz=rate,
            valid_points=0 if scan is None else len(scan.valid_points),
            last_error=error,
        )

    def status_dict(self) -> dict:
        return asdict(self.snapshot())

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            driver = None
            try:
                self.byte_activity.start_session()
                driver = self.driver_factory(self.config, self.byte_activity, self._stop)
                with self._driver_lock:
                    self._driver = driver
                with self._lock:
                    self._last_error = None
                    self._opened_at = time.monotonic()
                    self._last_scan_at = None
                    self._last_scan = None
                    self._scan_rate_hz = None
                scans = iter(driver.scans())
                discard_initial_rotation = self.config.mode == "serial"
                while not self._stop.is_set():
                    scan = next(scans)
                    if discard_initial_rotation:
                        discard_initial_rotation = False
                        continue
                    completed_at = time.monotonic()
                    self.scan_buffer.update(scan)
                    with self._lock:
                        self._last_scan = scan
                        self._last_scan_at = completed_at
                        self._scan_rate_hz = scan.scan_freq_hz or None
                        self._scans_received += 1
                        self._last_error = None
                    if self.on_scan is not None:
                        self.on_scan(scan, completed_at)
            except StopIteration:
                if not self._stop.is_set():
                    with self._lock:
                        self._last_error = "upstream driver stopped without a scan"
            except Exception as exc:
                if not self._stop.is_set():
                    with self._lock:
                        self._last_error = f"{type(exc).__name__}: {exc}"
            finally:
                with self._driver_lock:
                    if self._driver is driver:
                        self._driver = None
                if driver is not None:
                    try:
                        driver.close()
                    except Exception:
                        pass
            self._stop.wait(RETRY_AFTER_SEC)
