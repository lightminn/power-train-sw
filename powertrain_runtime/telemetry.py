"""Shared UDP send boundary for legacy and authenticated-session operation."""
from .destination import resolve_destination


def send_datagram(sock, payload, fallback):
    endpoint = resolve_destination(*fallback)
    if endpoint is None:
        return False
    sock.sendto(payload, endpoint)
    return True
