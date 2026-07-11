"""Filesystem path identities that remain safe across inode reuse."""

import os
import stat


def path_identity(path):
    """Return a no-follow identity suitable for ownership comparisons."""
    current = os.lstat(path)
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
