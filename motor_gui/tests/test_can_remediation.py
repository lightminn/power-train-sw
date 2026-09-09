"""CAN GUI regressions using the production worker and protocol dispatchers."""
import struct
import threading
import queue
from concurrent.futures import ThreadPoolExecutor

import can
import pytest

from motor_gui.backend.transport.ak_device import AkDevice
from motor_gui.backend.transport.can_bus import CanBackend, AK
from motor_gui.backend.transport.can_device import CanTransport
from motor_gui.backend.transport.odrive_can_device import OdriveCanDevice
from motor_gui.backend.worker import HardwareWorker
from motor_gui.backend.transport.fake import FakeTransport


class Bus:
    def __init__(self, rx=(), fail=False):
        self.rx = list(rx)
        self.sent = []
        self.fail = fail

    def send(self, msg, timeout=None):
        if self.fail:
            raise can.CanOperationError("injected TX failure")
        self.sent.append(msg)

    def recv(self, timeout=None):
        return self.rx.pop(0) if self.rx else None

    def shutdown(self):
        pass


def enc(pos=7.5):
    return can.Message(arbitration_id=(11 << 5) | 9, is_extended_id=False,
                       data=struct.pack("<ff", pos, 0))


def ak_status():
    return can.Message(arbitration_id=(41 << 8) | 1, is_extended_id=True,
                       data=struct.pack(">hhhbb", 123, 0, 0, 20, 0))


def legacy(bus):
    t = CanBackend()
    t._bus = bus
    t._ak = AK(bus, 1)
    return t


@pytest.mark.parametrize("reverse", [False, True])
def test_legacy_single_rx_dispatch_keeps_both_protocols(reverse):
    frames = [ak_status(), enc()]
    t = legacy(Bus(frames[::-1] if reverse else frames))
    sample = t.sample()
    assert sample["ak.pos_deg"] == 12.3
    assert sample["odrive.pos"] == 7.5


@pytest.mark.parametrize("kind", ["legacy", "composed"])
def test_tx_query_failure_still_drains_rx(kind):
    bus = Bus([ak_status(), enc()], fail=True)
    t = legacy(bus) if kind == "legacy" else CanTransport(
        [AkDevice(), OdriveCanDevice()], bus=bus)
    if kind == "composed":
        t.connect()
    sample = t.sample()
    assert sample["ak.pos_deg"] == 12.3
    assert sample["odrive.pos"] == 7.5
    assert sample["can.tx_errors"] > 0


def test_legacy_polling_is_limited_between_back_to_back_samples():
    bus = Bus()
    t = legacy(bus)
    for _ in range(100):
        t.sample()
    assert len(bus.sent) <= 3


@pytest.mark.parametrize("kind", ["legacy", "composed"])
def test_sensorless_estimates_are_never_requested_or_reported_as_temperature(kind):
    bus = Bus()
    t = legacy(bus) if kind == "legacy" else CanTransport([OdriveCanDevice()], bus=bus)
    if kind == "composed":
        t.connect()
    bus.rx.append(can.Message(arbitration_id=(11 << 5) | 0x15,
                             is_extended_id=False, data=struct.pack("<ff", 123.5, 42)))
    sample = t.sample()
    assert 0x15 not in [m.arbitration_id & 31 for m in bus.sent]
    assert "odrive.temp_fet" not in t.capabilities()["signals"]
    assert "odrive.temp_fet" not in sample
    assert any("온도" in note for note in t.capabilities()["notes"])


@pytest.mark.parametrize("op,args", [("set_input", {"pos_deg": 20}),
                                     ("disarm", {}), ("set_origin", {})])
def test_ak_send_failure_is_failure_ack_and_cancels_resend(monkeypatch, op, args):
    monkeypatch.setattr("ak_control.time.sleep", lambda _: None)
    d = AkDevice()
    bus = Bus(fail=True)
    d.attach(bus)
    d.apply(bus, "arm", {})
    ack = d.apply(bus, op, args)
    assert ack["ok"] is False
    assert d._active is None
    assert d._armed is False


@pytest.mark.parametrize("fail", [False, True])
def test_worker_stops_old_id_before_change_and_never_changes_after_failed_stop(fail):
    bus = Bus(fail=fail)
    d = OdriveCanDevice(node_id=11)
    t = CanTransport([d], bus=bus)
    t.connect()
    w = HardwareWorker(t)
    done, box = threading.Event(), {}
    w._reconnect_q.put((done, box, {"odrive": 13}))
    w._handle_reconnect()
    assert done.is_set()
    if fail:
        assert box["result"]["ok"] is False
        assert d.can_id_spec()["id"] == 11
    else:
        assert box["result"]["ok"] is True
        idle = [m.arbitration_id >> 5 for m in bus.sent
                if m.arbitration_id & 31 == 7 and struct.unpack("<I", m.data)[0] == 1]
        assert idle[0] == 11
        assert 13 in idle


def test_failed_startup_disarm_closes_transport_before_propagating():
    class BrokenStop(FakeTransport):
        closed = False

        def apply(self, cmd):
            return {"ok": False, "detail": "cannot stop"}

        def close(self):
            self.closed = True

    t = BrokenStop()
    with pytest.raises(RuntimeError, match="startup disarm"):
        HardwareWorker(t).start()
    assert t.closed


def test_nvm_save_is_only_allowed_while_worker_disarmed():
    w = HardwareWorker(FakeTransport())
    w.start()
    try:
        ack = w.submit({"target": "odrive", "op": "save_nvm", "args": {}})
        assert ack["ok"] is True
        assert w.submit({"target": "odrive", "op": "arm", "args": {}})["ok"]
        ack = w.submit({"target": "odrive", "op": "save_nvm", "args": {}})
        assert ack["ok"] is False
    finally:
        w.stop()


class _ObservedQueue(queue.Queue):
    def __init__(self):
        super().__init__()
        self.arrivals = queue.Queue()

    def put(self, item, *args, **kwargs):
        super().put(item, *args, **kwargs)
        self.arrivals.put(True)


class _RetargetTransport(FakeTransport):
    def __init__(self):
        super().__init__()
        self.node = 11
        self.trace = []
        self.connect_started = None
        self.connect_release = None

    def capabilities(self):
        caps = super().capabilities()
        caps["commands"]["odrive"] += ["arm", "disarm"]
        return caps

    def apply(self, cmd):
        self.trace.append((self.node, cmd["op"], cmd.get("args", {})))
        return super().apply(cmd)

    def set_device_ids(self, ids):
        self.node = ids["odrive"]

    def device_ids(self):
        return {"odrive": self.node}

    def connect(self):
        if self.connect_started is not None:
            self.connect_started.set()
            assert self.connect_release.wait(2)
        super().connect()


def _paused_worker():
    transport = _RetargetTransport()
    worker = HardwareWorker(transport)
    worker._running.set()
    worker._cmd_q = _ObservedQueue()
    worker._reconnect_q = _ObservedQueue()
    return worker, transport


@pytest.mark.parametrize("retarget", [False, True])
def test_queued_commands_cannot_cross_reconnect_target_boundary(retarget):
    worker, transport = _paused_worker()
    arm = {"target": "odrive", "op": "arm", "args": {}}
    velocity = {"target": "odrive", "op": "set_input", "args": {"vel": 1}}
    with ThreadPoolExecutor(max_workers=4) as callers:
        pending = []
        for callback, arg in [(worker.submit, arm), (worker.submit, velocity),
                              (worker.apply_profile, "bl70200")]:
            pending.append(callers.submit(callback, arg))
            worker._cmd_q.arrivals.get(timeout=1)
        change = callers.submit(worker.set_ids, {"odrive": 12}) if retarget else callers.submit(worker.reconnect)
        worker._reconnect_q.arrivals.get(timeout=1)
        worker._handle_reconnect()
        worker._drain_commands()
        assert change.result(timeout=1)["ok"]
        assert not any(op in ("arm", "set_input", "set_gain", "set_limit")
                       for _, op, _ in transport.trace), transport.trace
        for result in pending:
            ack = result.result(timeout=1)
            assert not ack["ok"]
            assert ack["status"] == "FINAL_REJECTED"
            assert "target changed" in ack["detail"]
        for cmd in (arm, velocity):
            result = callers.submit(worker.submit, cmd)
            worker._cmd_q.arrivals.get(timeout=1)
            worker._drain_commands()
            assert result.result(timeout=1)["ok"]
    assert transport.trace[-1] == (12 if retarget else 11, "set_input", {"vel": 1.0})


def test_commands_submitted_during_reconnect_are_rejected(monkeypatch):
    monkeypatch.setattr("motor_gui.backend.worker._SUBMIT_TIMEOUT", .05)
    worker, transport = _paused_worker()
    transport.connect_started = threading.Event()
    transport.connect_release = threading.Event()
    with ThreadPoolExecutor(max_workers=2) as callers:
        change = callers.submit(worker.set_ids, {"odrive": 12})
        worker._reconnect_q.arrivals.get(timeout=1)
        reconnect = callers.submit(worker._handle_reconnect)
        assert transport.connect_started.wait(1)
        try:
            for ack in (worker.submit({"target": "odrive", "op": "arm", "args": {}}),
                        worker.apply_profile("bl70200")):
                assert not ack["ok"]
                assert ack["status"] == "FINAL_REJECTED"
                assert "reconnect in progress" in ack["detail"]
        finally:
            transport.connect_release.set()
        reconnect.result(timeout=1)
        assert change.result(timeout=1)["ok"]
        worker._drain_commands()
    assert not any(op == "arm" for _, op, _ in transport.trace)
