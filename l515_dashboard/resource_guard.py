"""Persistent flock ownership for the L515 physical resource."""

import fcntl
import json
import os
from pathlib import Path
import stat


class ResourceBusy(RuntimeError):
    """Raised when the resource lock is busy or unsafe."""


class ResourceGuard:
    def __init__(self, lock_path, *, pid=None, proc_root="/proc"):
        self.lock_path = Path(lock_path)
        self.pid = os.getpid() if pid is None else int(pid)
        self.proc_root = Path(proc_root)
        self._fd = None

    @property
    def acquired(self):
        return self._fd is not None

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

    def acquire(self):
        if self.acquired:
            return
        self.lock_path.parent.mkdir(parents=True, exist_ok=True, mode=0o750)
        if self.lock_path.parent.stat().st_mode & 0o027:
            raise ResourceBusy("resource directory permissions are unsafe")
        flags = os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW | os.O_CLOEXEC
        try:
            fd = os.open(self.lock_path, flags, 0o600)
        except OSError as exc:
            raise ResourceBusy("resource lock path is unsafe") from exc
        try:
            current = os.fstat(fd)
            if not stat.S_ISREG(current.st_mode) or current.st_uid != os.geteuid():
                raise ResourceBusy("resource lock must be an owner-controlled regular file")
            if current.st_mode & 0o022:
                raise ResourceBusy("resource lock permissions are unsafe")
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise ResourceBusy("resource is already owned") from exc
            identity = self.process_start_identity(self.pid, self.proc_root)
            if identity is None:
                raise RuntimeError("cannot determine owner process start identity")
            payload = (json.dumps({"pid": self.pid, "start_identity": identity},
                                  sort_keys=True) + "\n").encode()
            os.ftruncate(fd, 0)
            os.lseek(fd, 0, os.SEEK_SET)
            view = memoryview(payload)
            while view:
                view = view[os.write(fd, view):]
            os.fsync(fd)
            self._fd = fd
        except Exception:
            try:
                fcntl.flock(fd, fcntl.LOCK_UN)
            finally:
                os.close(fd)
            raise

    def release(self):
        fd = self._fd
        if fd is None:
            return
        self._fd = None
        try:
            fcntl.flock(fd, fcntl.LOCK_UN)
        finally:
            os.close(fd)
