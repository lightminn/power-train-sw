"""Atomic singleton ownership for physical resources and their Unix socket."""

import json
import os
from pathlib import Path


class ResourceBusy(RuntimeError):
    """Raised when a verified live process owns the resource."""


class ResourceGuard:
    def __init__(self, lock_path, socket_path, *, pid=None, proc_root="/proc"):
        self.lock_path = Path(lock_path)
        self.socket_path = Path(socket_path)
        self.pid = os.getpid() if pid is None else int(pid)
        self.proc_root = Path(proc_root)
        self._lock_identity = None

    @property
    def acquired(self):
        return self._lock_identity is not None

    @staticmethod
    def _stat_start(text):
        # comm is parenthesized and may contain spaces or ')'; fields following
        # the final ')' begin at field 3, making starttime field 22 index 19.
        tail = text.rsplit(")", 1)[1].split()
        return tail[19]

    @classmethod
    def process_start_identity(cls, pid, proc_root="/proc"):
        try:
            return cls._stat_start(
                (Path(proc_root) / str(int(pid)) / "stat").read_text()
            )
        except (OSError, ValueError, IndexError):
            return None

    def _owner_is_live(self, payload):
        try:
            pid = int(payload["pid"])
            expected = str(payload["start_identity"])
        except (KeyError, TypeError, ValueError):
            return False
        return self.process_start_identity(pid, self.proc_root) == expected

    def acquire(self):
        if self.acquired:
            return
        self.lock_path.parent.mkdir(parents=True, exist_ok=True)
        identity = self.process_start_identity(self.pid, self.proc_root)
        if identity is None:
            raise RuntimeError("cannot determine owner process start identity")
        encoded = json.dumps({"pid": self.pid, "start_identity": identity}) + "\n"
        for _ in range(32):
            try:
                fd = os.open(self.lock_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            except FileExistsError:
                try:
                    before = self.lock_path.stat()
                    payload = json.loads(self.lock_path.read_text())
                except (OSError, ValueError, TypeError):
                    payload = {}
                    try:
                        before = self.lock_path.stat()
                    except FileNotFoundError:
                        continue
                if self._owner_is_live(payload):
                    raise ResourceBusy(f"resource owned by pid {payload['pid']}")
                try:
                    after = self.lock_path.stat()
                    if (before.st_dev, before.st_ino) == (after.st_dev, after.st_ino):
                        self.lock_path.unlink()
                except FileNotFoundError:
                    pass
                continue
            try:
                os.write(fd, encoded.encode())
                os.fsync(fd)
                stat = os.fstat(fd)
                self._lock_identity = (stat.st_dev, stat.st_ino)
            finally:
                os.close(fd)
            # A socket without our newly-created lock cannot be a verified live
            # owner. Remove only the pathname; never signal its unknown process.
            try:
                self.socket_path.unlink()
            except FileNotFoundError:
                pass
            return
        raise ResourceBusy("resource lock changed too frequently")

    def release(self):
        identity = self._lock_identity
        self._lock_identity = None
        if identity is None:
            return
        try:
            stat = self.lock_path.stat()
            if (stat.st_dev, stat.st_ino) != identity:
                return
            self.lock_path.unlink()
        except FileNotFoundError:
            pass
        try:
            self.socket_path.unlink()
        except FileNotFoundError:
            pass

