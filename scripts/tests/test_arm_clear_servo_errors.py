"""Only servos reporting a hardware error are rebooted before the bridge starts."""
import importlib.util
from pathlib import Path

PATH = Path(__file__).parents[1] / 'arm_clear_servo_errors.py'
SPEC = importlib.util.spec_from_file_location('arm_clear_servo_errors', PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class Bus:
    def __init__(self, errors, stuck=()):
        self.errors, self.stuck, self.reboots = dict(errors), set(stuck), []

    def read1ByteTxRx(self, _port, dxl_id, _address):
        if dxl_id not in self.errors:
            return 0, -3001, 0
        return self.errors[dxl_id], 0, 128 if self.errors[dxl_id] else 0

    def reboot(self, _port, dxl_id):
        self.reboots.append(dxl_id)
        if dxl_id not in self.stuck:
            self.errors[dxl_id] = 0


def test_only_errored_servos_reboot_and_absent_ones_are_skipped():
    bus = Bus({11: 0x20, 14: 0, 13: 0})
    assert MODULE.clear_errors(bus, None, settle=lambda _s: None) == ([11], [])
    assert bus.reboots == [11]


def test_a_servo_that_stays_in_error_is_reported():
    bus = Bus({11: 0x20}, stuck=[11])
    assert MODULE.clear_errors(bus, None, settle=lambda _s: None) == ([11], [11])


def test_healthy_bus_reboots_nothing():
    bus = Bus({11: 0, 12: 0})
    assert MODULE.clear_errors(bus, None, settle=lambda _s: None) == ([], [])
    assert bus.reboots == []
