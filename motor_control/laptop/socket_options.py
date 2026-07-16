"""TCP command-socket options shared by the legacy laptop clients."""

import socket


def configure_command_socket(sock, *, user_timeout_ms: int = 5000) -> None:
    """Disable Nagle and bound how long unacknowledged commands may linger."""
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    if hasattr(socket, "TCP_USER_TIMEOUT"):
        sock.setsockopt(
            socket.IPPROTO_TCP,
            socket.TCP_USER_TIMEOUT,
            int(user_timeout_ms),
        )
