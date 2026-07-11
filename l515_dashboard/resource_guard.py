"""Identity-safe singleton ownership for physical resources."""

import fcntl
import json
import os
from pathlib import Path
import tempfile


class ResourceBusy(RuntimeError):
    """Raised when ownership is live or cannot be verified safely."""


class ResourceGuard:
    def __init__(self, lock_path, socket_path, *, pid=None, proc_root="/proc"):
        self.lock_path = Path(lock_path)
        self.socket_path = Path(socket_path)
        self.pid = os.getpid() if pid is None else int(pid)
        self.proc_root = Path(proc_root)
        self._lock_identity = None
        self._payload = None
        self._before_publish = lambda: None

    @property
    def acquired(self):
        return self._lock_identity is not None

    @staticmethod
    def _stat_start(text):
        return text.rsplit(")", 1)[1].split()[19]

    @classmethod
    def process_start_identity(cls, pid, proc_root="/proc"):
        try:
            text = (Path(proc_root) / str(int(pid)) / "stat").read_text()
            return cls._stat_start(text)
        except (OSError, ValueError, IndexError):
            return None

    def _owner_is_live(self, payload):
        try:
            pid = int(payload["pid"])
            expected = str(payload["start_identity"])
        except (KeyError, TypeError, ValueError):
            return None
        return self.process_start_identity(pid, self.proc_root) == expected

    def _mutex(self):
        path = self.lock_path.with_name(self.lock_path.name + ".mutex")
        fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o600)
        fcntl.flock(fd, fcntl.LOCK_EX)
        return fd

    @staticmethod
    def _unlock(fd):
        fcntl.flock(fd, fcntl.LOCK_UN)
        os.close(fd)

    def _publish(self, payload):
        encoded = (json.dumps(payload, sort_keys=True) + "\n").encode()
        fd, temporary = tempfile.mkstemp(
            prefix=self.lock_path.name + ".", dir=self.lock_path.parent
        )
        try:
            os.fchmod(fd, 0o600)
            os.write(fd, encoded)
            os.fsync(fd)
            os.close(fd)
            fd = -1
            self._before_publish()
            os.link(temporary, self.lock_path)
            stat = self.lock_path.stat()
            self._lock_identity = (stat.st_dev, stat.st_ino)
        finally:
            if fd >= 0:
                os.close(fd)
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass

    def _remove_owned_socket(self, payload):
        expected = payload.get("socket_identity")
        if not isinstance(expected, list) or len(expected) != 2:
            return
        try:
            stat = self.socket_path.stat()
            if [stat.st_dev, stat.st_ino] == expected:
                self.socket_path.unlink()
        except FileNotFoundError:
            pass

    def acquire(self):
        if self.acquired:
            return
        self.lock_path.parent.mkdir(parents=True, exist_ok=True, mode=0o750)
        mode = self.lock_path.parent.stat().st_mode & 0o777
        if mode & 0o027:
            raise ResourceBusy("resource directory permissions are unsafe")
        identity = self.process_start_identity(self.pid, self.proc_root)
        if identity is None:
            raise RuntimeError("cannot determine owner process start identity")
        mutex = self._mutex()
        try:
            if self.lock_path.exists():
                try:
                    payload = json.loads(self.lock_path.read_text())
                except (OSError, ValueError, TypeError) as exc:
                    raise ResourceBusy("lock owner identity is unknown") from exc
                live = self._owner_is_live(payload)
                if live is None:
                    raise ResourceBusy("lock owner identity is unknown")
                if live:
                    raise ResourceBusy(f"resource owned by pid {payload['pid']}")
                self._remove_owned_socket(payload)
                self.lock_path.unlink()
            self._payload = {"pid": self.pid, "start_identity": identity}
            self._publish(self._payload)
        finally:
            self._unlock(mutex)

    def claim_socket(self):
        if not self.acquired:
            raise RuntimeError("resource is not acquired")
        stat = self.socket_path.stat()
        mutex = self._mutex()
        try:
            current = self.lock_path.stat()
            if (current.st_dev, current.st_ino) != self._lock_identity:
                raise ResourceBusy("lock ownership changed")
            payload = dict(self._payload)
            payload["socket_identity"] = [stat.st_dev, stat.st_ino]
            self.lock_path.unlink()
            self._publish(payload)
            self._payload = payload
        finally:
            self._unlock(mutex)

    def release(self):
        identity = self._lock_identity
        if identity is None:
            return
        mutex = self._mutex()
        try:
            try:
                current = self.lock_path.stat()
            except FileNotFoundError:
                return
            if (current.st_dev, current.st_ino) != identity:
                return
            self._remove_owned_socket(self._payload)
            self.lock_path.unlink()
        finally:
            self._lock_identity = None
            self._payload = None
            self._unlock(mutex)
