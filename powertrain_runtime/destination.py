"""Resolve a current operator destination without retaining expired leases."""
import ipaddress
import json
import math
import os
import time
from pathlib import Path


def resolve_destination(fallback_host, fallback_port, *, path=None, now=None):
    path = path if path is not None else os.environ.get('POWERTRAIN_OPERATOR_SESSION_FILE')
    if path is None:
        return fallback_host, fallback_port
    try:
        with Path(path).open() as stream:
            raw = stream.read(4097)
        if len(raw) > 4096:
            return None
        record = json.loads(raw)
        host = str(ipaddress.IPv4Address(record['host']))
        expires = float(record['expires_at'])
        current = time.monotonic() if now is None else now
        if not record['lease_id'] or not math.isfinite(expires) or expires <= current:
            return None
        return host, fallback_port
    except (OSError, ValueError, TypeError, KeyError):
        return None
