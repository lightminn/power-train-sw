import pytest
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
        time.sleep(0.2)
        assert w.latest()["odrive.vel"] > 1.0   # 가속 확인 (estop 전)
        w.estop()
        time.sleep(0.4)
        assert abs(w.latest()["odrive.vel"]) < 1.0
    finally:
        w.stop()


def test_double_start_raises():
    w = _make()
    w.start()
    try:
        with pytest.raises(RuntimeError):
            w.start()
    finally:
        w.stop()


def test_submit_before_start_returns_not_running():
    w = _make()
    ack = w.submit({"target": "odrive", "op": "estop", "args": {}})
    assert ack["ok"] is False
    assert "not running" in ack["detail"]


def test_drain_rejects_queued_commands_during_estop():
    # Critical 회귀: estop 활성 틱에서 큐에 남은 비-estop 명령은 거부되어야 함.
    import threading
    w = _make()
    done = threading.Event()
    box: dict = {}
    w._cmd_q.put(({"target": "odrive", "op": "set_input",
                   "args": {"vel": 15.0}}, done, box))
    w._drain_commands(estopped=True)
    assert done.is_set()
    assert box["ack"]["ok"] is False
    assert "estop" in box["ack"]["detail"]


def test_reconnect_closes_and_reconnects():
    from motor_gui.backend.transport.fake import FakeTransport
    from motor_gui.backend.worker import HardwareWorker

    class CountingFake(FakeTransport):
        def __init__(self):
            super().__init__()
            self.connects = 0
            self.closes = 0
        def connect(self):
            self.connects += 1
            super().connect()
        def close(self):
            self.closes += 1
            super().close()

    t = CountingFake()
    w = HardwareWorker(t)
    w.start()
    try:
        assert t.connects == 1
        ack = w.reconnect()
        assert ack["ok"] is True
        assert t.connects == 2
        assert t.closes >= 1
    finally:
        w.stop()


def test_reconnect_when_not_running():
    from motor_gui.backend.transport.fake import FakeTransport
    from motor_gui.backend.worker import HardwareWorker
    w = HardwareWorker(FakeTransport())
    ack = w.reconnect()           # 미기동 상태
    assert ack["ok"] is False


def test_set_ids_applies_then_reconnects():
    from motor_gui.backend.transport.base import Transport
    from motor_gui.backend.worker import HardwareWorker

    class IdFake(Transport):
        name = "idfake"
        def __init__(self):
            self.ids = {"ak": 1}
            self.connects = 0
        def connect(self): self.connects += 1
        def sample(self): return {"t_mono": 0.0, "ak.x": 1.0}
        def apply(self, cmd): return {"ok": True}
        def capabilities(self):
            return {"track": "can", "devices": ["ak"], "signals": ["ak.x"],
                    "commands": {}, "can_ids": {"ak": {"id": self.ids["ak"]}}}
        def close(self): pass
        def device_ids(self):
            return {"ak": {"id": self.ids["ak"], "min": 1, "max": 127, "label": "AK"}}
        def set_device_ids(self, m):
            if "ak" in m:
                self.ids["ak"] = int(m["ak"])

    t = IdFake()
    w = HardwareWorker(t)
    w.start()
    try:
        res = w.set_ids({"ak": 2})
        assert res["ok"] is True
        assert t.ids["ak"] == 2                 # set_device_ids 적용됨
        assert res["ids"]["ak"]["id"] == 2      # 결과에 새 ID 반영
        assert t.connects == 2                  # start + 재연결
    finally:
        w.stop()


def test_set_ids_when_not_running():
    from motor_gui.backend.transport.fake import FakeTransport
    from motor_gui.backend.worker import HardwareWorker
    w = HardwareWorker(FakeTransport())
    res = w.set_ids({"ak": 2})
    assert res["ok"] is False
