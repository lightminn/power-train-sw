"""Persistent positive-int32 mission ID allocator (ROS independent)."""

from dataclasses import dataclass
import os
from pathlib import Path
import tempfile


INT32_MAX = 2_147_483_647


@dataclass(frozen=True)
class MissionIdAllocation:
    accepted: bool
    mission_id: int | None = None
    hold_reason: str = ""


class MissionIdStore:
    def __init__(self, path="/var/lib/powertrain/mission_id"):
        self.path = Path(path)

    def allocate(self):
        """Persist and return the next ID, or a fail-closed hold signal.

        The caller is the single owner.  A successful return means the new ID
        survived a file fsync, atomic replace, and parent-directory fsync and
        can therefore be published safely.
        """
        try:
            current = self._read_current()
        except (OSError, UnicodeError, ValueError) as exc:
            return self._hold("corrupt:%s" % type(exc).__name__)

        if current < 0 or current > INT32_MAX:
            return self._hold("corrupt:out_of_range")
        if current == 0 and self.path.exists():
            return self._hold("corrupt:nonpositive")
        if current == INT32_MAX:
            return self._hold("int32_exhausted")

        mission_id = current + 1
        try:
            self._atomic_write(mission_id)
        except OSError as exc:
            return self._hold("io_error:%s" % type(exc).__name__)
        return MissionIdAllocation(True, mission_id=mission_id)

    def _read_current(self):
        try:
            raw = self.path.read_text(encoding="ascii")
        except FileNotFoundError:
            return 0
        value = raw.strip()
        if not value:
            raise ValueError("empty mission ID")
        current = int(value, 10)
        if current <= 0:
            raise ValueError("mission ID must be positive")
        return current

    def _atomic_write(self, mission_id):
        parent = self.path.parent
        fd = -1
        temp_path = None
        try:
            fd, name = tempfile.mkstemp(
                prefix=".%s." % self.path.name,
                suffix=".tmp",
                dir=parent,
            )
            temp_path = Path(name)
            with os.fdopen(fd, "w", encoding="ascii", newline="") as stream:
                fd = -1
                stream.write("%d\n" % mission_id)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp_path, self.path)
            temp_path = None

            flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
            directory_fd = os.open(parent, flags)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            if fd >= 0:
                os.close(fd)
            if temp_path is not None:
                try:
                    temp_path.unlink()
                except FileNotFoundError:
                    pass

    @staticmethod
    def _hold(detail):
        return MissionIdAllocation(
            False,
            hold_reason="mission_id_store:%s" % detail,
        )
