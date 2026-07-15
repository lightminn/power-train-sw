"""Singleton event journal daemon with same-UID Unix sockets."""
from __future__ import annotations

from collections import OrderedDict
import errno
import fcntl
import os
from pathlib import Path
import queue
import socket
import stat
import struct
import threading
import time

from .health import HealthState
from .journal import BoundedEventQueue, MissionJournal
from .protocol import (
    EVENT_SOCKET,
    LOCK_PATH,
    MAX_DATAGRAM_BYTES,
    MAX_STATUS_BYTES,
    RUN_DIRECTORY,
    STATUS_SOCKET,
    abstract_address,
    credentials_from_ancillary,
    decode_event_datagram,
    decode_status_request,
    encode_status_response,
    verify_credentials,
)


class DaemonAlreadyRunning(RuntimeError):
    pass


class DaemonLock:
    """Persistent lock-file ownership. Releasing never unlinks the file."""

    def __init__(self, path: str | os.PathLike[str]) -> None:
        self.path = os.fspath(path)
        self._fd: int | None = None

    def acquire(self) -> None:
        if self._fd is not None:
            return
        flags = os.O_RDWR | os.O_CREAT | os.O_CLOEXEC
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        fd = os.open(self.path, flags, 0o640)
        try:
            metadata = os.fstat(fd)
            if not stat.S_ISREG(metadata.st_mode):
                raise OSError(errno.EINVAL, "daemon lock is not a regular file")
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError as exc:
                if exc.errno in (errno.EACCES, errno.EAGAIN):
                    raise DaemonAlreadyRunning(
                        f"observability daemon is already running (lock: {self.path})"
                    ) from exc
                raise
            os.ftruncate(fd, 0)
            os.write(fd, f"{os.getpid()}\n".encode("ascii"))
            os.fsync(fd)
        except Exception:
            os.close(fd)
            raise
        self._fd = fd

    def close(self) -> None:
        fd = self._fd
        if fd is None:
            return
        self._fd = None
        try:
            fcntl.flock(fd, fcntl.LOCK_UN)
        finally:
            os.close(fd)


class ObservabilityServer:
    MAX_RECENT_EVENT_TYPES = 32
    MAX_CHANNELS = 32

    def __init__(
        self,
        *,
        event_socket: str = EVENT_SOCKET,
        status_socket: str = STATUS_SOCKET,
        lock_path: str | os.PathLike[str] = LOCK_PATH,
        run_directory: str | os.PathLike[str] = RUN_DIRECTORY,
        run_id: str | None = None,
        queue_capacity: int = 256,
        expected_uid: int | None = None,
        socket_runtime: bool = True,
    ) -> None:
        self.event_socket = str(event_socket)
        self.status_socket = str(status_socket)
        self.lock_path = os.fspath(lock_path)
        self.run_directory = Path(run_directory)
        self.run_id = run_id or self.new_run_id()
        self.expected_uid = os.geteuid() if expected_uid is None else int(expected_uid)
        self._socket_runtime = bool(socket_runtime)
        self.health = HealthState()
        self.queue = BoundedEventQueue(queue_capacity, health=self.health)
        self._lock = DaemonLock(self.lock_path)
        self._journal: MissionJournal | None = None
        self._event_socket: socket.socket | None = None
        self._status_socket: socket.socket | None = None
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._client_sockets: set[socket.socket] = set()
        self._client_threads: set[threading.Thread] = set()
        self._client_lock = threading.Lock()
        self._snapshot_lock = threading.Lock()
        self._recent_event = None
        self._recent_events: OrderedDict[str, dict] = OrderedDict()
        self._channel_health: OrderedDict[str, dict] = OrderedDict()
        self._started = False

    @staticmethod
    def new_run_id() -> str:
        now = time.time_ns()
        stamp = time.strftime("%Y%m%dT%H%M%S", time.gmtime(now / 1_000_000_000))
        return f"{stamp}.{now % 1_000_000_000:09d}Z-{os.getpid()}"

    @staticmethod
    def authorize_peer(
        credentials: tuple[int, int, int], expected_uid: int | None = None
    ) -> tuple[int, int, int]:
        pid, uid, gid = credentials
        owner_uid = os.geteuid() if expected_uid is None else int(expected_uid)
        if uid != owner_uid:
            raise PermissionError(f"peer UID {uid} does not match daemon UID {owner_uid}")
        return pid, uid, gid

    @staticmethod
    def configure_event_socket(sock: socket.socket) -> None:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_PASSCRED, 1)

    @property
    def is_running(self) -> bool:
        return self._started and not self._stop.is_set()

    def start(self) -> None:
        if self.is_running:
            return
        self._lock.acquire()
        try:
            self._journal = MissionJournal(
                self.run_directory, run_id=self.run_id, health=self.health
            )
            if self._socket_runtime:
                self._event_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
                self.configure_event_socket(self._event_socket)
                self._event_socket.bind(abstract_address(self.event_socket))
                self._event_socket.settimeout(0.1)

                self._status_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                self._status_socket.bind(abstract_address(self.status_socket))
                self._status_socket.listen(8)
                self._status_socket.settimeout(0.1)
            self._stop.clear()
            self._started = True
            self._threads = [
                threading.Thread(
                    target=self._write_events,
                    daemon=True,
                    name="observability-journal",
                ),
            ]
            if self._socket_runtime:
                self._threads.extend([
                    threading.Thread(
                        target=self._receive_events,
                        daemon=True,
                        name="observability-events",
                    ),
                    threading.Thread(
                        target=self._accept_status,
                        daemon=True,
                        name="observability-status",
                    ),
                ])
            for thread in self._threads:
                thread.start()
        except Exception:
            self._close_socket(self._event_socket)
            self._close_socket(self._status_socket)
            self._event_socket = None
            self._status_socket = None
            self._started = False
            if self._journal is not None:
                self._journal.close()
                self._journal = None
            self._lock.close()
            raise

    @staticmethod
    def _close_socket(sock: socket.socket | None) -> None:
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def _receive_events(self) -> None:
        ancillary_size = socket.CMSG_SPACE(struct.calcsize("3i"))
        while not self._stop.is_set():
            try:
                data, ancillary, flags, _address = self._event_socket.recvmsg(
                    MAX_DATAGRAM_BYTES + 1, ancillary_size
                )
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                if flags & socket.MSG_TRUNC:
                    raise ValueError("event datagram exceeds size limit")
                credentials = credentials_from_ancillary(
                    ancillary, expected_uid=self.expected_uid
                )
            except (PermissionError, ValueError):
                self.health.record_drop()
                continue
            self.ingest_datagram(data, credentials)

    def ingest_datagram(self, data: bytes, credentials) -> bool:
        """Validate one kernel-attributed packet and enqueue it without waiting."""
        try:
            if isinstance(credentials, tuple):
                self.authorize_peer(credentials, expected_uid=self.expected_uid)
            else:
                verify_credentials(credentials, expected_uid=self.expected_uid)
            event = decode_event_datagram(data)
        except (PermissionError, ValueError):
            self.health.record_drop()
            return False
        return self.queue.offer(event)

    def _write_events(self) -> None:
        while True:
            try:
                event = self.queue.get_nowait()
            except queue.Empty:
                if self._stop.is_set():
                    return
                self._stop.wait(0.01)
                continue
            try:
                record = self._journal.append(event)
            except Exception as exc:
                self.health.mark_degraded(f"journal append failed: {exc}")
                continue
            if record is not None:
                self._journal.flush()
                self._record_snapshot(record)

    def _record_snapshot(self, record: dict) -> None:
        with self._snapshot_lock:
            self._recent_event = record
            event_type = record["event_type"]
            self._recent_events.pop(event_type, None)
            self._recent_events[event_type] = record
            while len(self._recent_events) > self.MAX_RECENT_EVENT_TYPES:
                self._recent_events.popitem(last=False)
            if event_type == "CHANNEL_HEALTH":
                payload = dict(record["payload"])
                channel = str(payload.get("channel") or record["source"])
                self._channel_health.pop(channel, None)
                self._channel_health[channel] = payload
                while len(self._channel_health) > self.MAX_CHANNELS:
                    self._channel_health.popitem(last=False)

    def status_snapshot(self) -> dict:
        health = self.health.snapshot()
        with self._snapshot_lock:
            return {
                "run_id": self.run_id,
                "health": {
                    "status": health.status,
                    "last_error": health.last_error,
                },
                "drop_count": health.drop_count,
                "recent_event": self._recent_event,
                "recent_events": dict(self._recent_events),
                "channel_health": dict(self._channel_health),
            }

    def _accept_status(self) -> None:
        while not self._stop.is_set():
            try:
                client, _ = self._status_socket.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                raw = client.getsockopt(
                    socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("3i")
                )
                self.authorize_peer(
                    struct.unpack("3i", raw), expected_uid=self.expected_uid
                )
            except Exception:
                client.close()
                continue
            client.settimeout(1.0)
            thread = threading.Thread(
                target=self._serve_status,
                args=(client,),
                daemon=True,
                name="observability-status-client",
            )
            with self._client_lock:
                self._client_sockets.add(client)
                self._client_threads.add(thread)
            thread.start()

    def _serve_status(self, client: socket.socket) -> None:
        reader = None
        try:
            reader = client.makefile("rb")
            raw = reader.readline(MAX_STATUS_BYTES + 1)
            if not raw or len(raw) > MAX_STATUS_BYTES or not raw.endswith(b"\n"):
                return
            decode_status_request(raw[:-1])
            client.sendall(encode_status_response(self.status_snapshot()))
        except (BrokenPipeError, ConnectionError, OSError, ValueError):
            pass
        finally:
            if reader is not None:
                reader.close()
            with self._client_lock:
                self._client_sockets.discard(client)
                self._client_threads.discard(threading.current_thread())
            client.close()

    def stop(self) -> None:
        if not self._started:
            self._lock.close()
            return
        self._stop.set()
        self._started = False
        self._close_socket(self._event_socket)
        self._close_socket(self._status_socket)
        self._event_socket = None
        self._status_socket = None
        with self._client_lock:
            clients = list(self._client_sockets)
            client_threads = list(self._client_threads)
        for client in clients:
            try:
                client.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            client.close()
        current = threading.current_thread()
        for thread in self._threads + client_threads:
            if thread is not current:
                thread.join(timeout=2.0)
        self._threads = []
        if self._journal is not None:
            self._journal.close()
            self._journal = None
        self._lock.close()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, _exc_type, _exc, _traceback):
        self.stop()
