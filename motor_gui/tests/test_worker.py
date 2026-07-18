import pytest
import time

from motor_gui.backend.transport.fake import FakeTransport
from motor_gui.backend.worker import HardwareWorker


def _make() -> HardwareWorker:
    return HardwareWorker(FakeTransport(), rate_hz=200)


def _arm(worker: HardwareWorker, target: str = "odrive") -> dict:
    return worker.submit({"target": target, "op": "arm", "args": {}})


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
        assert _arm(w)["ok"] is True
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
        assert _arm(w)["ok"] is True
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


def test_estop_latch_persists_across_ticks_and_rejects_commands():
    w = _make()
    w.start()
    try:
        assert _arm(w)["ok"] is True
        w.estop()
        time.sleep(0.1)  # 여러 200 Hz tick 뒤에도 래치가 남아야 함
        for _ in range(2):
            ack = w.submit({"target": "odrive", "op": "set_input",
                            "args": {"vel": 15.0}})
            assert ack["ok"] is False
            assert "estop active" in ack["detail"]
            time.sleep(0.02)
    finally:
        w.stop()


def test_reset_returns_idle_and_requires_device_specific_arm():
    w = _make()
    w.start()
    try:
        assert _arm(w, "odrive")["ok"] is True
        w.estop()
        time.sleep(0.05)

        reset = w.submit({"target": "odrive", "op": "reset", "args": {}})
        assert reset["ok"] is True
        assert w.safety_state() == {
            "estop_latched": False,
            "armed": {"odrive": False, "ak": False},
        }

        rejected = w.submit({"target": "odrive", "op": "set_input",
                             "args": {"vel": 2.0}})
        assert rejected["ok"] is False
        assert "disarmed" in rejected["detail"]

        assert _arm(w, "odrive")["ok"] is True
        assert w.submit({"target": "odrive", "op": "set_input",
                         "args": {"vel": 2.0}})["ok"] is True
        ak_rejected = w.submit({"target": "ak", "op": "set_input",
                                "args": {"pos_deg": 5.0}})
        assert ak_rejected["ok"] is False
        assert "disarmed" in ak_rejected["detail"]
    finally:
        w.stop()


def test_only_reset_or_estop_is_accepted_while_latched():
    w = _make()
    w.start()
    try:
        w.estop()
        time.sleep(0.05)
        arm = _arm(w)
        assert arm["ok"] is False
        assert "estop active" in arm["detail"]
        profile = w.apply_profile("x2212")
        assert profile["ok"] is False
        assert "estop active" in profile["detail"]
    finally:
        w.stop()


def test_start_is_disarmed_and_disarm_revokes_commands():
    w = _make()
    w.start()
    try:
        before_arm = w.submit({"target": "odrive", "op": "set_input",
                               "args": {"vel": 1.0}})
        assert before_arm["ok"] is False
        assert "disarmed" in before_arm["detail"]
        assert _arm(w)["ok"] is True
        assert w.submit({"target": "odrive", "op": "disarm",
                         "args": {}})["ok"] is True
        after_disarm = w.submit({"target": "odrive", "op": "set_input",
                                 "args": {"vel": 1.0}})
        assert after_disarm["ok"] is False
        assert "disarmed" in after_disarm["detail"]
    finally:
        w.stop()


def test_new_estop_during_reset_keeps_latch_set():
    import threading

    class BlockingResetFake(FakeTransport):
        def __init__(self):
            super().__init__()
            self.block_reset = False
            self.reset_entered = threading.Event()
            self.release_reset = threading.Event()

        def apply(self, cmd):
            if self.block_reset and cmd["op"] == "clear_errors":
                self.reset_entered.set()
                self.release_reset.wait(timeout=1.0)
            return super().apply(cmd)

    transport = BlockingResetFake()
    worker = HardwareWorker(transport, rate_hz=200)
    worker.start()
    try:
        worker.estop()
        time.sleep(0.05)
        transport.block_reset = True
        result = {}
        reset_thread = threading.Thread(
            target=lambda: result.update(ack=worker.submit({
                "target": "odrive", "op": "reset", "args": {}
            }))
        )
        reset_thread.start()
        assert transport.reset_entered.wait(timeout=1.0)

        worker.estop()  # reset이 끝나기 전에 새 안전 이벤트 발생
        transport.release_reset.set()
        reset_thread.join(timeout=1.0)

        assert result["ack"]["ok"] is False
        assert "new estop" in result["ack"]["detail"]
        assert worker.safety_state()["estop_latched"] is True
    finally:
        transport.release_reset.set()
        worker.stop()


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


def test_start_and_reconnect_read_tunables_without_writing_hardware():
    class ReadbackFake(FakeTransport):
        def __init__(self):
            super().__init__()
            self.commands = []
            self.reads = 0

        def capabilities(self):
            caps = super().capabilities()
            caps["drive_gear_ratio"] = 5.0
            return caps

        def apply(self, cmd):
            self.commands.append(cmd)
            return super().apply(cmd)

        def read_tunables(self):
            self.reads += 1
            return {"pos_gain": 2.0, "vel_gain": 0.12,
                    "vel_integrator_gain": 0.2, "current_lim": 9.0}

    transport = ReadbackFake()
    worker = HardwareWorker(transport)
    worker.start()
    try:
        assert not any(cmd["op"] in ("set_gain", "set_limit")
                       for cmd in transport.commands)
        assert transport.reads == 1
        assert worker.tunables() == {
            "pos_gain": 2.0,
            "vel_gain": 0.12,
            "vel_integrator_gain": 0.2,
            "current_lim": 9.0,
        }

        transport.commands.clear()
        assert worker.reconnect()["ok"] is True
        assert not any(cmd["op"] in ("set_gain", "set_limit")
                       for cmd in transport.commands)
        assert transport.reads == 2
    finally:
        worker.stop()


def test_tunable_profiles_are_named_x2212_and_bl70200():
    from motor_gui.backend.transport import base

    assert set(base.TUNABLE_PROFILES) == {"x2212", "bl70200"}
    assert base.TUNABLE_PROFILES["x2212"]["label"].startswith("X2212")
    assert base.TUNABLE_PROFILES["x2212"]["values"]["vel_gain"] == 0.015
    assert base.TUNABLE_PROFILES["bl70200"] == {
        "label": "BL70200",
        "values": {
            "pos_gain": 2.0,
            "vel_gain": 0.12,
            "vel_integrator_gain": 0.2,
            "current_lim": 9.0,
        },
    }


def test_selected_profile_is_applied_only_on_explicit_request():
    class ProfileFake(FakeTransport):
        def __init__(self):
            super().__init__()
            self.commands = []

        def apply(self, cmd):
            self.commands.append(cmd)
            return super().apply(cmd)

    transport = ProfileFake()
    worker = HardwareWorker(transport, rate_hz=200)
    worker.start()
    try:
        assert not any(cmd["op"] in ("set_gain", "set_limit")
                       for cmd in transport.commands)
        assert _arm(worker)["ok"] is True
        transport.commands.clear()
        ack = worker.apply_profile("bl70200")
        assert ack["ok"] is True
        assert [cmd["op"] for cmd in transport.commands] == [
            "set_gain", "set_limit"
        ]
        assert transport.commands[0]["args"] == {
            "pos_gain": 2.0,
            "vel_gain": 0.12,
            "vel_integrator_gain": 0.2,
        }
        assert transport.commands[1]["args"] == {"current_lim": 9.0}
    finally:
        worker.stop()
