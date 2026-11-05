"""FTDI latency must be written before anything holds the serial device open.

Writing latency_timer while the tty is open blacks out the bus for about 0.25 s
(bench 2026-11-06: lock-write-open failed every read, write-lock-open none).
The bridge looks for its servos once at startup, so that window lost the whole
arm on the first start after every reboot.
"""

from pathlib import Path

import pytest

PACKAGE = Path(__file__).parents[1] / 'dynamixel_control'


@pytest.mark.parametrize('name', ['moveit_dynamixel_bridge.py', 'dynamixel_position_node.py'])
def test_latency_is_set_before_the_bus_lock_opens_the_device(name):
    source = (PACKAGE / name).read_text()
    assert source.index('_set_ftdi_latency(', source.index('class ')) \
        < source.index('bus_lock.acquire(')
