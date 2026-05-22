from motor_gui.backend.transport.can_device import CanDevice, CanTransport


class StubBus:
    def __init__(self, rx=None):
        self.sent = []
        self._rx = list(rx or [])
    def send(self, msg, timeout=None):
        self.sent.append(msg)
    def recv(self, timeout=None):
        return self._rx.pop(0) if self._rx else None
    def shutdown(self):
        pass


class StubDevice(CanDevice):
    name = "stub"
    def __init__(self):
        self.attached = None
        self.rx_count = 0
    def attach(self, bus):
        self.attached = bus
    def capabilities_fragment(self):
        return {"devices": ["stub"], "signals": ["stub.x"],
                "commands": {"stub": ["ping"]}, "control_modes": {},
                "inputs": {}, "tunables": {}, "limits": {"stub": {}},
                "signal_meta": {"stub.x": {"label": "X", "unit": ""}}}
    def on_rx(self, msg):
        self.rx_count += 1
    def sample(self):
        return {"stub.x": 1.0}
    def apply(self, bus, op, args):
        return {"ok": True, "target": "stub", "op": op, "detail": "ok"}


def test_capabilities_merges_device_fragments():
    t = CanTransport([StubDevice()], track="ak", bus=StubBus())
    caps = t.capabilities()
    assert caps["track"] == "ak"
    assert caps["devices"] == ["stub"]
    assert "stub.x" in caps["signals"]
    assert caps["commands"]["stub"] == ["ping"]
    assert caps["signal_meta"]["stub.x"]["label"] == "X"


def test_sample_merges_and_has_t_mono():
    t = CanTransport([StubDevice()], bus=StubBus())
    t.connect()
    s = t.sample()
    assert "t_mono" in s and s["stub.x"] == 1.0


def test_apply_routes_by_target():
    dev = StubDevice()
    t = CanTransport([dev], bus=StubBus())
    t.connect()
    ack = t.apply({"target": "stub", "op": "ping", "args": {}})
    assert ack["ok"] is True
    bad = t.apply({"target": "nope", "op": "ping", "args": {}})
    assert bad["ok"] is False and bad["detail"] == "unknown target"


def test_attach_called_on_connect():
    dev = StubDevice()
    bus = StubBus()
    t = CanTransport([dev], bus=bus)
    t.connect()
    assert dev.attached is bus
