"""ROS-independent safety helpers for the chassis node."""

import json
import os
from pathlib import Path
import tempfile
from typing import NamedTuple


class EstopLatchRecord(NamedTuple):
    first_source: str
    first_detail: str


class ConsoleEstopLatchStore:
    """Durable first-cause record for console-origin chassis E-stop."""

    _FIELDS = {
        "schema_version",
        "latched",
        "first_source",
        "first_detail",
    }

    def __init__(self, path):
        self.path = Path(path)
        if not self.path.is_absolute() or self.path.name == "":
            raise ValueError(
                "console E-stop latch path must be an absolute file path"
            )

    def load_fail_closed(self):
        try:
            return self._load()
        except (OSError, TypeError, UnicodeError, ValueError) as exc:
            return EstopLatchRecord(
                "console_latch_store",
                "load_failed:%s" % type(exc).__name__,
            )

    def _load(self):
        if not self.path.parent.is_dir():
            raise OSError("console E-stop latch directory is unavailable")
        try:
            raw = self.path.read_text(encoding="utf-8")
        except FileNotFoundError:
            if not self.path.parent.is_dir():
                raise OSError(
                    "console E-stop latch directory became unavailable"
                )
            return None
        payload = json.loads(raw)
        if not isinstance(payload, dict) or set(payload) != self._FIELDS:
            raise ValueError(
                "console E-stop latch fields do not match contract"
            )
        if payload["schema_version"] != 1 or payload["latched"] is not True:
            raise ValueError("console E-stop latch version/state is invalid")
        source = payload["first_source"]
        detail = payload["first_detail"]
        if not isinstance(source, str) or not source:
            raise ValueError("console E-stop first source is invalid")
        if not isinstance(detail, str):
            raise ValueError("console E-stop first detail is invalid")
        return EstopLatchRecord(source, detail)

    def persist(self, first_source, first_detail):
        if not isinstance(first_source, str) or not first_source:
            raise ValueError("console E-stop first source is invalid")
        if not isinstance(first_detail, str):
            raise ValueError("console E-stop first detail is invalid")
        payload = {
            "schema_version": 1,
            "latched": True,
            "first_source": first_source,
            "first_detail": first_detail,
        }
        data = json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ) + "\n"
        fd = -1
        temp_path = None
        try:
            fd, name = tempfile.mkstemp(
                prefix=".%s." % self.path.name,
                suffix=".tmp",
                dir=self.path.parent,
            )
            temp_path = Path(name)
            with os.fdopen(fd, "w", encoding="utf-8", newline="") as stream:
                fd = -1
                stream.write(data)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp_path, self.path)
            temp_path = None
            self._fsync_parent()
        finally:
            if fd >= 0:
                os.close(fd)
            if temp_path is not None:
                try:
                    temp_path.unlink()
                except FileNotFoundError:
                    pass

    def clear(self):
        if not self.path.parent.is_dir():
            raise OSError("console E-stop latch directory is unavailable")
        try:
            self.path.unlink()
        except FileNotFoundError:
            if not self.path.parent.is_dir():
                raise OSError(
                    "console E-stop latch directory became unavailable"
                )
            return
        self._fsync_parent()

    def _fsync_parent(self):
        flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
        directory_fd = os.open(self.path.parent, flags)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)


def validate_runtime_clock_mode(*, fake, use_sim_time):
    """Reject simulated ROS time whenever real chassis hardware is selected."""
    if not bool(fake) and bool(use_sim_time):
        raise ValueError(
            "use_sim_time=true is forbidden with real chassis hardware"
        )
