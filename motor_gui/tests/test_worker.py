import time

from motor_gui.backend.transport.fake import FakeTransport
from motor_gui.backend.worker import HardwareWorker


def _make() -> HardwareWorker:
    return HardwareWorker(FakeTransport(), rate_hz=200)


def test_start_produces_samples_then_stop():
    w = _make()
    w.start()
    try:
        time.sleep(0.2)
        s = w.latest()
        assert s is not None and "odrive.vel" in s
        assert len(w.history()) > 5
    finally:
        w.stop()


def test_submit_applies_command():
    w = _make()
    w.start()
    try:
        w.submit({"target": "odrive", "op": "set_mode",
                  "args": {"control_mode": "velocity"}})
        ack = w.submit({"target": "odrive", "op": "set_input",
                        "args": {"vel": 8.0}})
        assert ack["ok"] is True
        time.sleep(0.3)
        assert w.latest()["odrive.vel"] > 1.0
    finally:
        w.stop()


def test_invalid_command_returns_error_ack():
    w = _make()
    w.start()
    try:
        ack = w.submit({"target": "ghost", "op": "estop", "args": {}})
        assert ack["ok"] is False
        assert "ghost" in ack["detail"]
    finally:
        w.stop()


def test_estop_fast_path_zeros_velocity():
    w = _make()
    w.start()
    try:
        w.submit({"target": "odrive", "op": "set_input", "args": {"vel": 10.0}})
        time.sleep(0.1)
        w.estop()
        time.sleep(0.4)
        assert abs(w.latest()["odrive.vel"]) < 1.0
    finally:
        w.stop()
