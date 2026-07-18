from __future__ import annotations

import collections
import logging
import queue
import threading
import time
from collections.abc import Callable

from .commands import normalize, CommandError
from .transport.base import (Transport, TUNABLE_PROFILES,
                             motor_to_wheel_tunables)

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
        self._estop_request = threading.Event()
        self._estop_latched = threading.Event()
        self._estop_generation = 0
        self._running = threading.Event()
        self._thread: threading.Thread | None = None
        self._subscribers: list[Callable[[dict], None]] = []
        self._sub_lock = threading.Lock()
        self._safety_lock = threading.Lock()
        self._caps: dict = {}
        self._transport_commands: dict[str, tuple[str, ...]] = {}
        self._armed: dict[str, bool] = {}
        self._refresh_capabilities(force_disarmed=True)
        self._tunables: dict = {}

    # ── lifecycle ─────────────────────────────────
    def start(self) -> None:
        if self._running.is_set():
            raise RuntimeError("HardwareWorker is already running")
        self._t.connect()
        self._refresh_capabilities(force_disarmed=True)
        errors = self._disarm_all()
        if errors:
            raise RuntimeError(f"startup disarm failed: {errors}")
        self._read_tunables()
        # 이전 사이클의 미처리 요청 제거 (타임아웃으로 큐에 남은 항목 방지)
        for q in (self._reconnect_q, self._cmd_q):
            while not q.empty():
                try:
                    q.get_nowait()
                except queue.Empty:
                    break
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
        self._set_all_armed(False)

    # ── web-facing API ────────────────────────────
    def latest(self) -> dict | None:
        return self._latest

    def history(self) -> list:
        return list(self._ring)

    def capabilities(self) -> dict:
        return self._caps

    def tunables(self) -> dict:
        return self._tunables

    def safety_state(self) -> dict:
        with self._safety_lock:
            armed = dict(self._armed)
            latched = self._estop_latched.is_set()
        return {"estop_latched": latched,
                "armed": armed}

    def tunable_profiles(self) -> dict:
        ratio = self._caps.get("drive_gear_ratio")
        profiles = {}
        for name, profile in TUNABLE_PROFILES.items():
            values = dict(profile["values"])
            if ratio is not None:
                values = motor_to_wheel_tunables(values, ratio)
            profiles[name] = {"label": profile["label"], "values": values}
        return profiles

    def apply_profile(self, profile: str) -> dict:
        """사용자가 선택한 프로파일만 워커 스레드에서 명시적으로 적용한다."""
        if not self._running.is_set():
            return {"ok": False, "profile": profile,
                    "detail": "worker not running"}
        if profile not in TUNABLE_PROFILES:
            return {"ok": False, "profile": profile,
                    "detail": f"unknown profile: {profile!r}"}
        done = threading.Event()
        box: dict = {}
        self._cmd_q.put(({"target": "odrive", "op": "apply_profile",
                          "args": {"profile": profile}}, done, box))
        if not done.wait(timeout=_SUBMIT_TIMEOUT):
            return {"ok": False, "profile": profile,
                    "detail": "profile apply timeout"}
        return box["ack"]

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
        """최우선 정지 요청과 영속 래치를 원자적으로 세운다."""
        with self._safety_lock:
            self._estop_generation += 1
            self._estop_latched.set()
            for target in self._armed:
                self._armed[target] = False
        self._estop_request.set()

    def reconnect(self) -> dict:
        """하드웨어 transport 재연결 (close → connect → readback)."""
        if not self._running.is_set():
            return {"ok": False, "detail": "worker not running"}
        if self._estop_latched.is_set():
            return {"ok": False, "detail": "rejected: estop active; reset required"}
        done = threading.Event()
        box: dict = {}
        self._reconnect_q.put((done, box, None))
        if not done.wait(timeout=_RECONNECT_TIMEOUT):
            return {"ok": False, "detail": "reconnect timeout"}
        return box["result"]

    def set_ids(self, mapping: dict) -> dict:
        """CAN ID 변경 + 재연결 (워커 스레드에서 set_device_ids → close/connect)."""
        if not self._running.is_set():
            return {"ok": False, "detail": "worker not running"}
        if self._estop_latched.is_set():
            return {"ok": False, "detail": "rejected: estop active; reset required"}
        done = threading.Event()
        box: dict = {}
        self._reconnect_q.put((done, box, dict(mapping)))
        if not done.wait(timeout=_RECONNECT_TIMEOUT):
            return {"ok": False, "detail": "set_ids timeout"}
        return box["result"]

    def subscribe(self, callback: Callable[[dict], None]) -> None:
        """callback(sample: dict) 등록 (스레드에서 호출됨, 가볍게 유지)."""
        with self._sub_lock:
            self._subscribers.append(callback)

    def unsubscribe(self, callback: Callable[[dict], None]) -> None:
        with self._sub_lock:
            if callback in self._subscribers:
                self._subscribers.remove(callback)

    def _refresh_capabilities(self, force_disarmed: bool = False) -> None:
        caps = self._t.capabilities()
        self._transport_commands = {
            target: tuple(ops)
            for target, ops in caps.get("commands", {}).items()
        }
        for target in caps.get("devices", []):
            ops = list(caps.setdefault("commands", {}).get(target, []))
            for safety_op in ("arm", "disarm", "reset"):
                if safety_op not in ops:
                    ops.append(safety_op)
            caps["commands"][target] = ops
        self._caps = caps
        with self._safety_lock:
            previous = self._armed
            self._armed = {
                target: (False if force_disarmed else previous.get(target, False))
                for target in caps.get("devices", [])
            }

    def _set_all_armed(self, value: bool) -> None:
        with self._safety_lock:
            for target in self._armed:
                self._armed[target] = value

    def _set_armed(self, target: str, value: bool) -> None:
        with self._safety_lock:
            if target in self._armed:
                self._armed[target] = value

    def _is_armed(self, target: str) -> bool:
        with self._safety_lock:
            return self._armed.get(target, False)

    def _transport_apply(self, target: str, op: str, args: dict) -> dict:
        return self._t.apply({"target": target, "op": op, "args": args})

    def _disarm_target(self, target: str) -> dict:
        ops = self._transport_commands.get(target, ())
        if "disarm" in ops:
            ack = self._transport_apply(target, "disarm", {})
        elif "set_state" in ops:
            ack = self._transport_apply(target, "set_state", {"state": "idle"})
        elif "estop" in ops:
            ack = self._transport_apply(target, "estop", {})
        else:
            ack = {"ok": True, "target": target, "op": "disarm",
                   "detail": "server disarmed"}
        if ack.get("ok"):
            self._set_armed(target, False)
        return ack

    def _disarm_all(self) -> list[str]:
        errors = []
        for target in self._caps.get("devices", []):
            try:
                ack = self._disarm_target(target)
                if not ack.get("ok"):
                    errors.append(f"{target}: {ack.get('detail', 'disarm failed')}")
            except Exception as exc:
                errors.append(f"{target}: {exc}")
        return errors

    def _arm_target(self, target: str) -> dict:
        ops = self._transport_commands.get(target, ())
        if "arm" in ops:
            ack = self._transport_apply(target, "arm", {})
        elif "set_state" in ops:
            ack = self._transport_apply(
                target, "set_state", {"state": "closed_loop"}
            )
        else:
            ack = {"ok": True, "target": target, "op": "arm",
                   "detail": "server armed"}
        if ack.get("ok"):
            self._set_armed(target, True)
            return {"ok": True, "target": target, "op": "arm",
                    "detail": ack.get("detail", "armed")}
        return {"ok": False, "target": target, "op": "arm",
                "detail": ack.get("detail", "arm failed")}

    def _reset_estop(self, target: str) -> dict:
        with self._safety_lock:
            reset_generation = self._estop_generation
        errors = []
        for device in self._caps.get("devices", []):
            ops = self._transport_commands.get(device, ())
            try:
                if "clear_errors" in ops:
                    ack = self._transport_apply(device, "clear_errors", {})
                    if not ack.get("ok"):
                        errors.append(
                            f"{device}: {ack.get('detail', 'clear_errors failed')}"
                        )
                        continue
                ack = self._disarm_target(device)
                if not ack.get("ok"):
                    errors.append(f"{device}: {ack.get('detail', 'disarm failed')}")
            except Exception as exc:
                errors.append(f"{device}: {exc}")
        if errors:
            return {"ok": False, "target": target, "op": "reset",
                    "detail": f"reset failed: {errors}"}
        with self._safety_lock:
            if self._estop_generation != reset_generation:
                return {"ok": False, "target": target, "op": "reset",
                        "detail": "reset rejected: new estop during reset"}
            for device in self._armed:
                self._armed[device] = False
            self._estop_latched.clear()
        return {"ok": True, "target": target, "op": "reset",
                "detail": "estop reset; all devices IDLE/disarmed"}

    def _handle_reconnect(self) -> None:
        """워커 스레드에서 1건 처리: transport 재연결 + readback + disarm."""
        try:
            done, box, ids = self._reconnect_q.get_nowait()
        except queue.Empty:
            return
        try:
            if self._estop_latched.is_set():
                raise RuntimeError("estop active; reset required")
            if ids:
                self._t.set_device_ids(ids)
            try:
                self._t.close()
            except Exception:
                pass
            self._t.connect()
            self._refresh_capabilities(force_disarmed=True)
            errors = self._disarm_all()
            if errors:
                raise RuntimeError(f"reconnect disarm failed: {errors}")
            self._read_tunables()
            box["result"] = {"ok": True, "detail": "reconnected",
                             "ids": self._t.device_ids()}
            self._ring.append({"t_mono": time.monotonic(), "info": "reconnected"})
        except Exception as e:
            box["result"] = {"ok": False, "detail": f"reconnect failed: {e}"}
        done.set()

    def _read_tunables(self) -> None:
        """연결된 장치의 현재값만 읽는다. 읽기 실패 시 추정값을 표시하지 않는다."""
        try:
            self._tunables = dict(self._t.read_tunables())
        except Exception:
            self._tunables = {}

    def _apply_tunable_profile(self, profile: str) -> dict:
        values = self.tunable_profiles()[profile]["values"]
        tunables = self._caps.get("tunables", {}).get("odrive", [])
        op_by_key = {item["key"]: item["op"] for item in tunables}
        applied = []
        for op in ("set_gain", "set_limit"):
            args = {key: value for key, value in values.items()
                    if op_by_key.get(key) == op}
            if not args:
                continue
            ack = self._t.apply({"target": "odrive", "op": op, "args": args})
            if not ack.get("ok"):
                return {"ok": False, "target": "odrive",
                        "op": "apply_profile", "profile": profile,
                        "detail": ack.get("detail", f"{op} failed")}
            applied.append(op)
        if not applied:
            return {"ok": False, "target": "odrive", "op": "apply_profile",
                    "profile": profile, "detail": "no supported profile tunables"}
        self._read_tunables()
        return {"ok": True, "target": "odrive", "op": "apply_profile",
                "profile": profile, "applied": applied,
                "readback": dict(self._tunables), "detail": "profile applied"}

    # ── thread loop ───────────────────────────────
    def _loop(self) -> None:
        while self._running.is_set():
            t0 = time.monotonic()

            self._handle_reconnect()

            if self._estop_request.is_set():
                self._estop_request.clear()
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
            op = norm.get("op")
            target = norm["target"]
            if op == "estop":
                self.estop()
                box["ack"] = {"ok": True, "target": target, "op": op,
                              "detail": "estop latched"}
                done.set()
                continue
            if op == "reset":
                box["ack"] = self._reset_estop(target)
                done.set()
                continue
            if self._estop_latched.is_set():
                box["ack"] = {"ok": False, "target": norm["target"],
                              "op": norm["op"], "detail": "rejected: estop active"}
                done.set()
                continue
            try:
                if op == "arm":
                    box["ack"] = self._arm_target(target)
                elif op == "disarm":
                    ack = self._disarm_target(target)
                    box["ack"] = {"ok": bool(ack.get("ok")),
                                  "target": target, "op": "disarm",
                                  "detail": ack.get("detail", "disarmed")}
                elif not self._is_armed(target):
                    box["ack"] = {"ok": False, "target": target, "op": op,
                                  "detail": "rejected: device disarmed; arm required"}
                elif op == "apply_profile":
                    box["ack"] = self._apply_tunable_profile(
                        norm["args"]["profile"]
                    )
                else:
                    box["ack"] = self._t.apply(norm)
                    if (op == "set_state"
                            and norm.get("args", {}).get("state") != "closed_loop"
                            and box["ack"].get("ok")):
                        self._set_armed(target, False)
            except Exception as e:
                box["ack"] = {"ok": False, "target": norm["target"],
                              "op": norm["op"], "detail": str(e)}
            done.set()
