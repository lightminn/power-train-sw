import importlib
from pathlib import Path
import socket

import pytest


class FakeSocket:
    def __init__(self):
        self.options = []

    def setsockopt(self, level, option, value):
        self.options.append((level, option, value))


def test_legacy_command_socket_disables_nagle_and_bounds_unacked_data():
    try:
        module = importlib.import_module("laptop.socket_options")
    except ModuleNotFoundError:
        module = None
    assert module is not None, "legacy laptop socket option helper is missing"
    fake = FakeSocket()

    module.configure_command_socket(fake)

    assert (socket.IPPROTO_TCP, socket.TCP_NODELAY, 1) in fake.options
    if hasattr(socket, "TCP_USER_TIMEOUT"):
        assert (socket.IPPROTO_TCP, socket.TCP_USER_TIMEOUT, 5000) in fake.options


@pytest.mark.parametrize(
    "filename",
    (
        "laptop_client_basic.py",
        "laptop_client_video.py",
        "laptop_client_velocity.py",
    ),
)
def test_each_legacy_laptop_client_applies_shared_socket_options(filename):
    source = (Path(__file__).parents[1] / filename).read_text(encoding="utf-8")
    assert "configure_command_socket" in source
