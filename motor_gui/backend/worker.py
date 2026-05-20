from __future__ import annotations

import collections
import queue
import threading
import time

from .commands import normalize, CommandError
from .transport.base import Transport


class HardwareWorker:
    """Transport 를 단독 소유하는 100 Hz 샘플링/명령 스레드.

    웹 레이어는 submit()/estop()/latest()/history()/subscribe()/capabilities()
    만 사용한다. Transport 접근은 전부 이 스레드 안에서만 일어나 동시접근이 없다.
    """

    def __init__(self, transport: Transport, rate_hz: float = 100.0,
                 ring_size: int = 3000) -> None:
        self._t = transport
        self._dt = 1.0 / rate_hz
        self._ring: collections.deque = collections.deque(maxlen=ring_size)
        self._latest: dict | None = None
        self._cmd_q: queue.Queue = queue.Queue()
        self._estop = threading.Event()
        self._running = threading.Event()
        self._thread: threading.Thread | None = None
        self._subscribers: list = []
        self._sub_lock = threading.Lock()
        self._caps = transport.capabilities()

    # ── lifecycle ─────────────────────────────────
    def start(self) -> None:
        self._t.connect()
        self._caps = self._t.capabilities()
        self._running.set()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        if self._thread:
            self._thread.join(timeout=2.0)
        try:
            self._t.close()
        except Exception:
            pass

    # ── web-facing API ────────────────────────────
    def latest(self) -> dict | None:
        return self._latest

    def history(self) -> list:
        return list(self._ring)

    def capabilities(self) -> dict:
        return self._caps

    def submit(self, cmd: dict) -> dict:
        """동기 명령. 정규화 실패는 즉시 에러 ack. 그 외는 워커가 적용 후 ack."""
        try:
            norm = normalize(cmd, self._caps)
        except CommandError as e:
            return {"ok": False, "target": cmd.get("target"),
                    "op": cmd.get("op"), "detail": str(e)}
        done = threading.Event()
        box: dict = {}
        self._cmd_q.put((norm, done, box))
        if not done.wait(timeout=2.0):
            return {"ok": False, "target": norm["target"], "op": norm["op"],
                    "detail": "command timeout"}
        return box["ack"]

    def estop(self) -> None:
        """최우선 정지 — 다음 루프 톱에서 즉시 적용."""
        self._estop.set()

    def subscribe(self, callback) -> None:
        """callback(sample: dict) 등록 (스레드에서 호출됨, 가볍게 유지)."""
        with self._sub_lock:
            self._subscribers.append(callback)

    def unsubscribe(self, callback) -> None:
        with self._sub_lock:
            if callback in self._subscribers:
                self._subscribers.remove(callback)

    # ── thread loop ───────────────────────────────
    def _loop(self) -> None:
        while self._running.is_set():
            t0 = time.monotonic()

            if self._estop.is_set():
                self._estop.clear()
                for dev in self._caps.get("devices", []):
                    try:
                        self._t.apply({"target": dev, "op": "estop", "args": {}})
                    except Exception:
                        pass

            try:
                s = self._t.sample()
            except Exception as e:  # 샘플 실패는 루프를 죽이지 않음
                s = {"t_mono": time.monotonic(), "error": str(e)}
            self._latest = s
            self._ring.append(s)
            with self._sub_lock:
                subs = list(self._subscribers)
            for cb in subs:
                try:
                    cb(s)
                except Exception:
                    pass

            self._drain_commands()

            elapsed = time.monotonic() - t0
            if elapsed < self._dt:
                time.sleep(self._dt - elapsed)

    def _drain_commands(self) -> None:
        while True:
            try:
                norm, done, box = self._cmd_q.get_nowait()
            except queue.Empty:
                return
            try:
                box["ack"] = self._t.apply(norm)
            except Exception as e:
                box["ack"] = {"ok": False, "target": norm["target"],
                              "op": norm["op"], "detail": str(e)}
            done.set()
