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


def test_recv_frame_fans_out_to_all_devices():
    import can
    rx = [can.Message(arbitration_id=0x123, data=b"\x00", is_extended_id=False)]
    dev_a, dev_b = StubDevice(), StubDevice()
    t = CanTransport([dev_a, dev_b], bus=StubBus(rx=rx))
    t.connect()
    t.sample()
    assert dev_a.rx_count == 1
    assert dev_b.rx_count == 1


def test_close_nulls_owned_bus_so_reconnect_reopens():
    # owns_bus=True (bus 주입 안 함). 연결된 척 _bus 를 채워두고 close → None 이어야
    # 다음 connect() 가 socketcan 을 재오픈한다.
    t = CanTransport([StubDevice()], bus=None)
    assert t._owns_bus is True
    t._bus = StubBus()
    t.close()
    assert t._bus is None


def test_close_keeps_injected_bus():
    # 주입 버스(owns_bus=False)는 close 후에도 유지(테스트/외부 소유).
    bus = StubBus()
    t = CanTransport([StubDevice()], bus=bus)
    assert t._owns_bus is False
    t.close()
    assert t._bus is bus
