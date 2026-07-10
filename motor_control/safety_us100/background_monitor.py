import math
import threading
import time

from safety_us100.verdict import NO_RESPONSE, Verdict


def _exception_verdict(exc):
    try:
        message = str(exc)
    except BaseException:
        message = "<unprintable>"
    return Verdict(
        NO_RESPONSE,
        None,
        True,
        1,
        f"background_exception:{type(exc).__name__}: {message}",
    )


class BackgroundSafetyMonitor:
    def __init__(
        self,
        monitor,
        period_s=0.1,
        stale_timeout_s=0.75,
        clock=None,
    ):
        period_s = float(period_s)
        if not math.isfinite(period_s) or period_s <= 0.0:
            raise ValueError("period_s must be positive and finite")
        stale_timeout_s = float(stale_timeout_s)
        if not math.isfinite(stale_timeout_s) or stale_timeout_s <= 0.0:
            raise ValueError("stale_timeout_s must be positive and finite")
        self._monitor = monitor
        self._period_s = period_s
        self._stale_timeout_s = stale_timeout_s
        self._close_timeout_s = 1.0
        self._clock = time.monotonic if clock is None else clock
        self._lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._closed = False
        try:
            self._latest = monitor.verdict()
        except BaseException as exc:
            self._latest = _exception_verdict(exc)
        self._updated_at = self._clock()

    def start(self):
        with self._lifecycle_lock:
            if self._closed or self._thread is not None:
                return
            self._thread = threading.Thread(
                target=self._run,
                name="us100-monitor",
                daemon=True,
            )
            self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                self._monitor.tick()
                latest = self._monitor.verdict()
            except BaseException as exc:
                latest = _exception_verdict(exc)
            with self._lock:
                self._latest = latest
                self._updated_at = self._clock()
            remaining = self._period_s - (time.monotonic() - started)
            self._stop.wait(max(remaining, 0.0))

    def verdict(self):
        with self._lock:
            latest = self._latest
            updated_at = self._updated_at
        if self._clock() - updated_at > self._stale_timeout_s:
            return Verdict(
                NO_RESPONSE,
                None,
                True,
                1,
                "background_stale",
            )
        return latest

    def close(self):
        with self._lifecycle_lock:
            if not self._closed:
                self._closed = True
                self._stop.set()
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=self._close_timeout_s)
        alive = thread is not None and thread.is_alive()
        with self._lifecycle_lock:
            if not alive and self._thread is thread:
                self._thread = None
        return not alive
