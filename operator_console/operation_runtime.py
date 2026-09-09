"""GTK-independent integrated session and supervised-controller lifetime."""
from copy import deepcopy
import fcntl
import json
import os
from pathlib import Path
import subprocess
import threading
import time

from powertrain_runtime.client import SessionClient, open_session_channel


class ConsoleInstanceLock:
    """Keep the lock inode; deleting it would allow two independent owners."""
    def __init__(self, path):
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        self._file = path.open("a")
        try:
            fcntl.flock(self._file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            self._file.close()
            raise RuntimeError("운용 콘솔이 이미 실행 중입니다") from exc

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def close(self):
        self._file.close()


class ControllerSupervisor:
    """Pipes never block GTK or session heartbeats; no shell or fake-pad flags."""
    def __init__(self, python):
        self._python = python
        self._lock = threading.RLock()
        self._connection = None
        self._ops = None
        self._state = {}
        self._received = 0
        self._closed = threading.Event()
        self._thread = threading.Thread(target=self._run, name="controller-supervisor", daemon=True)
        self._thread.start()

    def set_connection(self, connection):
        with self._lock:
            self._connection = deepcopy(connection)

    def set_ops_state(self, state):
        with self._lock:
            self._ops = deepcopy(state)

    def snapshot(self):
        with self._lock:
            state = dict(self._state)
            state["age_s"] = time.monotonic() - self._received
            if state["age_s"] > 1:
                state.update(pad_connected=False, neutral=False, input_connected=False)
            return state

    def _status(self, detail):
        with self._lock:
            self._state = dict(pad_connected=False, neutral=False, input_connected=False, detail=detail)
            self._received = time.monotonic()

    @staticmethod
    def _send(child, payload):
        record = (json.dumps(payload, allow_nan=False, separators=(",", ":")) + "\n").encode()
        if len(record) > 8192 or os.write(child.stdin.fileno(), record) != len(record):
            raise OSError("controller command pipe is full")

    def _run_child(self, child):
        os.set_blocking(child.stdin.fileno(), False)
        os.set_blocking(child.stdout.fileno(), False)
        buffer = bytearray()
        sent_lease = None
        next_heartbeat = 0
        while not self._closed.is_set() and child.poll() is None:
            with self._lock:
                connection, ops = deepcopy(self._connection), deepcopy(self._ops)
            lease = None if connection is None else connection["lease_id"]
            if lease != sent_lease:
                self._send(child, dict(op="disconnect"))
                if connection:
                    self._send(child, dict(op="connect", **connection))
                sent_lease = lease
            if time.monotonic() >= next_heartbeat:
                self._send(child, dict(op="heartbeat"))
                if ops is not None:
                    self._send(child, dict(op="ops_state", state=ops))
                next_heartbeat = time.monotonic() + .4
            try:
                chunk = os.read(child.stdout.fileno(), 8192)
            except BlockingIOError:
                chunk = None
            if chunk == b"":
                break
            if chunk:
                buffer.extend(chunk)
                if len(buffer) > 16384:
                    raise OSError("controller status overflow")
                while b"\n" in buffer:
                    line, _, rest = buffer.partition(b"\n")
                    buffer[:] = rest
                    try:
                        status = json.loads(line)
                        if not isinstance(status, dict) or any(type(status.get(key)) is not bool for key in ("pad_connected", "neutral", "input_connected")):
                            continue
                    except ValueError:
                        continue
                    with self._lock:
                        self._state, self._received = status, time.monotonic()
            self._closed.wait(.02)

    def _run(self):
        while not self._closed.is_set():
            child = None
            try:
                child = subprocess.Popen([self._python, "-m", "motor_control.laptop.controller_process"],
                    cwd=Path(__file__).resolve().parents[1], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL, bufsize=0)
                self._run_child(child)
                self._status("패드 프로세스 재연결 대기")
            except (OSError, ValueError):
                self._status("패드 실행 환경을 확인하세요")
            finally:
                if child is not None:
                    child.stdin.close()  # EOF is the child's primary parent-loss signal.
                    try:
                        child.wait(timeout=.7)
                    except subprocess.TimeoutExpired:
                        child.terminate()
                        try:
                            child.wait(timeout=.7)
                        except subprocess.TimeoutExpired:
                            child.kill()
                            child.wait(timeout=.7)
                    child.stdout.close()
            self._closed.wait(2)

    def close(self):
        self._closed.set()
        self._thread.join(timeout=3)


class OperationRuntime:
    def __init__(self, config, *, session_factory=SessionClient, supervisor=None):
        self.config = dict(config)
        self._session_factory = session_factory
        self._child = supervisor or ControllerSupervisor(config["controller_python"])
        self._lock = threading.RLock()
        self._closed = threading.Event()
        self._session = None
        self._connection = None
        self._state = dict(connected=False, detail="로봇 연결 대기")
        self._session_generation = 0
        self._session_seen = 0
        self._front_live = False
        self._front_seen = 0
        self._holding = False
        self._cancel_requested = False
        self._stop_requested = False
        self._begin_at = None
        self._thread = threading.Thread(target=self._run, name="operator-session", daemon=True)
        self._thread.start()

    def snapshot(self):
        with self._lock:
            result = {k: deepcopy(v) for k, v in self._state.items() if k not in ("ticket", "lease_id")}
            result.update(holding=self._holding, session_generation=self._session_generation)
            pad = self._child.snapshot()
            reason = None
            if not self._state.get("connected") or time.monotonic() - self._session_seen > .5:
                reason = "로봇 연결 대기"
            elif not pad.get("pad_connected") or pad.get("age_s", 9) > 1:
                reason = pad.get("detail") or "패드 연결 대기"
            elif not pad.get("input_connected"):
                reason = "패드 입력 채널 연결 대기"
            elif not pad.get("neutral"):
                reason = "스틱·버튼을 모두 놓으세요"
            elif not self._front_live or time.monotonic() - self._front_seen > .5:
                reason = "전방 영상 수신 대기"
            elif self._state.get("status") == "PENDING":
                reason = "이전 조작 처리 중"
            elif (self._state.get("ops_state") or {}).get("chassis_mode") == "ARMED":
                reason = "운전 중 · 주행 해제로 권한 해제"
            elif self._state.get("start_blocker"):
                reason = self._state["start_blocker"]
            result.update(pad=pad, ready=reason is None, reason=reason or "운전 시작 가능")
            return result

    def connection(self):
        with self._lock:
            return deepcopy(self._connection)

    def set_front_live(self, live):
        with self._lock:
            self._front_live, self._front_seen = bool(live), time.monotonic()

    def begin_start(self):
        with self._lock:
            if not self.snapshot()["ready"]:
                return False
            self._holding = True
            return True

    def cancel_start(self):
        with self._lock:
            self._holding = False
            self._cancel_requested = True

    def stop(self):
        with self._lock:
            self._holding = False
            self._stop_requested = True

    def _accept(self, snapshot, host):
        with self._lock:
            old_lease = None if self._connection is None else self._connection["lease_id"]
            self._state = dict(snapshot, host=host)
            self._session_seen = time.monotonic()
            if snapshot.get("connected"):
                if snapshot["lease_id"] != old_lease:
                    self._session_generation += 1
                    self._front_live = False
                    self._front_seen = 0
                self._connection = {k: snapshot[k] for k in ("lease_id", "ticket", "input_port")}
                self._connection.update(host=host, ops_port=snapshot["ops_port"])
            else:
                self._connection = None
            child_connection = deepcopy(self._connection)
            if child_connection:
                child_connection.pop("ops_port", None)
        self._child.set_connection(child_connection)
        self._child.set_ops_state(snapshot.get("ops_state"))

    def _lose(self):
        with self._lock:
            self._front_live = False
            self._front_seen = 0
            self._holding = False
            self._begin_at = None
            self._stop_requested = False
            self._cancel_requested = False
        self._accept(dict(connected=False, detail="로봇 연결 대기 · 주행 재시작은 수동 확인 필요"), None)

    def _tick_commands(self, session):
        with self._lock:
            stop, cancel = self._stop_requested, self._cancel_requested
            self._stop_requested = self._cancel_requested = False
            holding = self._holding
        if stop:
            self._begin_at = None
            self._accept(session.request("stop"), session.host)
            return
        if cancel or (holding and not self.snapshot()["ready"]):
            with self._lock:
                self._holding = False
            self._begin_at = None
            self._accept(session.request("start_cancel"), session.host)
            return
        if holding:
            if self._begin_at is None:
                self._accept(session.request("start_begin"), session.host)
                self._begin_at = time.monotonic()
            elif time.monotonic() - self._begin_at >= 1.5:
                with self._lock:
                    # A release/stop that arrived while heartbeat was running wins.
                    if not self._holding or self._stop_requested:
                        return
                    self._holding = False
                self._begin_at = None
                self._accept(session.request("start"), session.host)

    def _run(self):
        while not self._closed.is_set():
            session = None
            try:
                session = self._session_factory(self.config)
                self._session = session
                self._accept(session.connect(), session.host)
                while not self._closed.is_set():
                    self._tick_commands(session)
                    self._accept(session.heartbeat(), session.host)
                    self._closed.wait(.1)
            except (OSError, ValueError, KeyError):
                pass
            finally:
                self._lose()
                if session is not None:
                    session.close()
                self._session = None
            self._closed.wait(1)

    def make_ops_client(self, host, port, token, **kwargs):
        """Each ops-client generation is pinned to one lease; no old command replay."""
        from operator_console.ops_client import ConsoleOpsClient
        from motor_control.laptop.ops_channel_client import OpsChannelClient
        runtime = self

        def factory(_host, _port, _token):
            connection = runtime.connection()
            if connection is None:
                raise OSError("session unavailable")
            lease_id = connection["lease_id"]
            opened = False

            def connector(*_):
                nonlocal opened
                current = runtime.connection()
                if opened or current is None or current["lease_id"] != lease_id:
                    raise OSError("ops generation ended")
                sock = open_session_channel(connection["host"], connection["ops_port"],
                    lease_id, connection["ticket"], "ops")
                opened = True
                sock.setblocking(False)
                return sock

            class LeaseOpsClient(OpsChannelClient):
                def pump(self):
                    current = runtime.connection()
                    if current is None or current["lease_id"] != lease_id or self.sock is None:
                        raise OSError("session changed")
                    replies = super().pump()
                    if self.sock is None:
                        raise OSError("ops channel lost")
                    return replies
            return LeaseOpsClient(host, port, token, connector=connector)
        return ConsoleOpsClient(host, port, token, client_factory=factory, **kwargs)

    def close(self):
        self._closed.set()
        self._child.close()
        self._thread.join(timeout=3)
