from __future__ import annotations

import collections
import logging
import queue
import threading
import time
from collections.abc import Callable

from .commands import normalize, CommandError
from .transport.base import Transport, DEFAULT_TUNABLES

_log = logging.getLogger(__name__)

_STOP_JOIN_TIMEOUT = 2.0   # stop() 스레드 join 대기 (초)
_SUBMIT_TIMEOUT = 2.0      # submit() ack 대기 (초)
_RECONNECT_TIMEOUT = 20.0  # USB find_any 가 최대 ~15s 걸릴 수 있어 넉넉히


class HardwareWorker:
    """Transport 를 단독 소유하는 100 Hz 샘플링/명령 스레드.

    웹 레이어는 submit()/estop()/latest()/history()/capabilities()/
    subscribe()/unsubscribe() 만 사용한다. Transport 접근은 전부 이 스레드
    안에서만 일어나 동시접근이 없다 (접근법 A, 향후 C 프로세스격리 seam 유지).
    """

    def __init__(self, transport: Transport, rate_hz: float = 100.0,
                 ring_size: int = 3000) -> None:
        self._t = transport
        self._dt = 1.0 / rate_hz
        self._ring: collections.deque = collections.deque(maxlen=ring_size)
        self._latest: dict | None = None
        self._cmd_q: queue.Queue = queue.Queue()
        self._reconnect_q: queue.Queue = queue.Queue()
        self._estop = threading.Event()
        self._running = threading.Event()
        self._thread: threading.Thread | None = None
        self._subscribers: list[Callable[[dict], None]] = []
        self._sub_lock = threading.Lock()
        self._caps = transport.capabilities()
        self._tunables: dict = {}

    # ── lifecycle ─────────────────────────────────
    def start(self) -> None:
        if self._running.is_set():
            raise RuntimeError("HardwareWorker is already running")
        self._t.connect()
        self._caps = self._t.capabilities()
        self._apply_baseline()
        self._tunables = dict(DEFAULT_TUNABLES)
        self._running.set()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        if self._thread:
            self._thread.join(timeout=_STOP_JOIN_TIMEOUT)
            if self._thread.is_alive():
                _log.warning("worker thread did not exit cleanly; "
                             "close() may race with sample()")
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

    def tunables(self) -> dict:
        return self._tunables

    def submit(self, cmd: dict) -> dict:
        """동기 명령. 워커 미기동/정규화 실패는 즉시 에러 ack. 그 외는 적용 후 ack."""
        if not self._running.is_set():
            return {"ok": False, "target": cmd.get("target"),
                    "op": cmd.get("op"), "detail": "worker not running"}
        try:
            norm = normalize(cmd, self._caps)
        except CommandError as e:
            return {"ok": False, "target": cmd.get("target"),
                    "op": cmd.get("op"), "detail": str(e)}
        done = threading.Event()
        box: dict = {}
        self._cmd_q.put((norm, done, box))
        if not done.wait(timeout=_SUBMIT_TIMEOUT):
            return {"ok": False, "target": norm["target"], "op": norm["op"],
                    "detail": "command timeout"}
        return box["ack"]

    def estop(self) -> None:
        """최우선 정지 — 다음 루프 톱에서 즉시 적용, 같은 틱 큐 명령은 거부."""
        self._estop.set()

    def reconnect(self) -> dict:
        """하드웨어 transport 재연결 (close → connect → baseline). 워커 스레드에서 실행."""
        if not self._running.is_set():
            return {"ok": False, "detail": "worker not running"}
        done = threading.Event()
        box: dict = {}
        self._reconnect_q.put((done, box))
        if not done.wait(timeout=_RECONNECT_TIMEOUT):
            return {"ok": False, "detail": "reconnect timeout"}
        return box["result"]

    def subscribe(self, callback: Callable[[dict], None]) -> None:
        """callback(sample: dict) 등록 (스레드에서 호출됨, 가볍게 유지)."""
        with self._sub_lock:
            self._subscribers.append(callback)

    def unsubscribe(self, callback: Callable[[dict], None]) -> None:
        with self._sub_lock:
            if callback in self._subscribers:
                self._subscribers.remove(callback)

    def _handle_reconnect(self) -> None:
        """워커 스레드에서 1건 처리: transport 닫고 다시 연결 + baseline + caps 갱신."""
        try:
            done, box = self._reconnect_q.get_nowait()
        except queue.Empty:
            return
        try:
            try:
                self._t.close()
            except Exception:
                pass
            self._t.connect()
            self._caps = self._t.capabilities()
            self._apply_baseline()
            box["result"] = {"ok": True, "detail": "reconnected"}
            self._ring.append({"t_mono": time.monotonic(), "info": "reconnected"})
        except Exception as e:
            box["result"] = {"ok": False, "detail": f"reconnect failed: {e}"}
        done.set()

    def _apply_baseline(self) -> None:
        """odrive_can_setup.py 검증 baseline 을 startup 시 적용 (표시값=실제값 보장).
        IDLE 에서 config 쓰기는 안전. 실패는 무시 (예: 장치 미연결)."""
        gain_keys = ("pos_gain", "vel_gain", "vel_integrator_gain",
                     "input_filter_bandwidth", "trap_vel_limit",
                     "trap_accel_limit", "trap_decel_limit")
        gargs = {k: v for k, v in DEFAULT_TUNABLES.items() if k in gain_keys}
        largs = {k: v for k, v in DEFAULT_TUNABLES.items()
                 if k in ("vel_limit", "current_lim")}
        try:
            if gargs:
                self._t.apply({"target": "odrive", "op": "set_gain", "args": gargs})
            if largs:
                self._t.apply({"target": "odrive", "op": "set_limit", "args": largs})
        except Exception:
            pass

    # ── thread loop ───────────────────────────────
    def _loop(self) -> None:
        while self._running.is_set():
            t0 = time.monotonic()

            self._handle_reconnect()

            estopped = self._estop.is_set()
            if estopped:
                self._estop.clear()
                errs = []
                for dev in self._caps.get("devices", []):
                    try:
                        self._t.apply({"target": dev, "op": "estop", "args": {}})
                    except Exception as e:
                        errs.append(str(e))
                if errs:
                    self._ring.append({"t_mono": time.monotonic(),
                                       "error": f"estop failed: {errs}"})

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

            self._drain_commands(estopped=estopped)

            elapsed = time.monotonic() - t0
            if elapsed < self._dt:
                time.sleep(self._dt - elapsed)

    def _drain_commands(self, estopped: bool = False) -> None:
        while True:
            try:
                norm, done, box = self._cmd_q.get_nowait()
            except queue.Empty:
                return
            if estopped and norm.get("op") != "estop":
                box["ack"] = {"ok": False, "target": norm["target"],
                              "op": norm["op"], "detail": "rejected: estop active"}
                done.set()
                continue
            try:
                box["ack"] = self._t.apply(norm)
            except Exception as e:
                box["ack"] = {"ok": False, "target": norm["target"],
                              "op": norm["op"], "detail": str(e)}
            done.set()
