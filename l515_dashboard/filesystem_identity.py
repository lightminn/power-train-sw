"""Filesystem path identities that remain safe across inode reuse."""

import ctypes
import errno
import os
from pathlib import Path
import secrets
import stat


AT_FDCWD = -100
RENAME_NOREPLACE = 1


class PathOwnershipConflict(RuntimeError):
    """An unknown quarantined path could not be restored safely."""

    def __init__(self, path, quarantine_path):
        self.path = Path(path)
        self.quarantine_path = Path(quarantine_path)
        super().__init__(
            f"path ownership changed; preserved unknown path at {quarantine_path}"
        )


def path_identity(path):
    """Return a no-follow identity suitable for ownership comparisons."""
    return _stat_identity(os.lstat(path))


def _stat_identity(current):
    return (current.st_dev, current.st_ino, current.st_ctime_ns,
            stat.S_IFMT(current.st_mode))


def serialized_identity(identity):
    return list(identity)


def parse_identity(value):
    """Decode current identities; legacy two-field values are unsafe."""
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        return None
    if any(not isinstance(part, int) for part in value):
        return None
    return tuple(value)


def path_has_identity(path, expected):
    expected = parse_identity(expected)
    if expected is None:
        return False
    try:
        return path_identity(path) == expected
    except FileNotFoundError:
        return False


def _rename_noreplace(source, destination):
    libc = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(libc, "renameat2", None)
    if renameat2 is None:
        raise OSError(errno.ENOSYS, "renameat2 is unavailable")
    renameat2.argtypes = (
        ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p,
        ctypes.c_uint,
    )
    renameat2.restype = ctypes.c_int
    result = renameat2(
        AT_FDCWD, os.fsencode(source), AT_FDCWD, os.fsencode(destination),
        RENAME_NOREPLACE,
    )
    if result != 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error), str(destination))


def quarantine_remove(path, expected):
    """Remove only an owned path while preserving all canonical successors.

    The canonical path is atomically moved out of the namespace before its
    identity is checked. An unknown path is restored without overwriting a
    successor. If restoration is impossible, the unknown quarantine remains.
    """
    path = Path(path)
    expected = parse_identity(expected)
    if expected is None:
        return False
    quarantine = path.with_name(
        f".{path.name}.quarantine-{os.getpid()}-{secrets.token_hex(16)}"
    )
    flags = os.O_PATH | os.O_NOFOLLOW | os.O_CLOEXEC
    try:
        fd = os.open(path, flags)
    except FileNotFoundError:
        return False
    try:
        if _stat_identity(os.fstat(fd)) != expected:
            return False
        try:
            os.rename(path, quarantine)
        except FileNotFoundError:
            return False
        if path_identity(quarantine) == _stat_identity(os.fstat(fd)):
            os.unlink(quarantine)
            return True
        try:
            _rename_noreplace(quarantine, path)
        except OSError as exc:
            raise PathOwnershipConflict(path, quarantine) from exc
        return False
    finally:
        os.close(fd)
