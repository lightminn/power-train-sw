import struct
import can

from motor_gui.backend.transport.ak_device import AkDevice, PKT_STATUS_1, AK_ID


class StubBus:
    def __init__(self):
        self.sent = []
    def send(self, msg, timeout=None):
        self.sent.append(msg)


def _status_msg(pos_deg=12.0, spd_erpm=0, cur_a=0.0, temp=40, fault=0):
    # AK40._parse_status: ">hhhbb" = pos*10, spd/10, cur*100, temp, fault
    data = struct.pack(">hhhbb", int(pos_deg * 10), int(spd_erpm / 10),
                       int(cur_a * 100), temp, fault)
    return can.Message(arbitration_id=(PKT_STATUS_1 << 8) | AK_ID,
                       data=data, is_extended_id=True)


def _mk():
    d = AkDevice()
    bus = StubBus()
    d.attach(bus)
    return d, bus


def test_capabilities_fragment_modes_and_commands():
    f = AkDevice().capabilities_fragment()
    assert f["devices"] == ["ak"]
    assert f["control_modes"]["ak"] == ["position", "velocity", "brake", "duty"]
    assert "set_param" in f["commands"]["ak"]
    assert f["inputs"]["ak"]["velocity"]["key"] == "rpm"
    assert "ak.fault" in f["signals"]


def test_set_input_velocity_sends_rpm_frame():
    d, bus = _mk()
    d.apply(bus, "set_mode", {"control_mode": "velocity"})
    d.apply(bus, "set_input", {"rpm": 30.0})
    # PKT_SET_RPM=3
    assert bus.sent[-1].arbitration_id == (3 << 8) | AK_ID


def test_set_input_brake_sends_brake_frame():
    d, bus = _mk()
    d.apply(bus, "set_input", {"brake_cur": 2.0})
    assert bus.sent[-1].arbitration_id == (2 << 8) | AK_ID
    assert bus.sent[-1].data == struct.pack(">i", 2000)


def test_on_rx_parses_status_and_sample_converts_speed():
    d, bus = _mk()
    # POLE_PAIRS=14, GEAR_RATIO=10 → out_rpm = spd_erpm / 140
    d.on_rx(_status_msg(pos_deg=12.0, spd_erpm=1400, cur_a=1.5, temp=42, fault=0))
    s = d.sample()
    assert abs(s["ak.pos_deg"] - 12.0) < 0.1
    assert abs(s["ak.speed"] - 10.0) < 0.2          # 1400 erpm / 140
    assert abs(s["ak.current"] - 1.5) < 0.05
    assert s["ak.temp"] == 42


def test_set_param_then_position_uses_spd():
    d, bus = _mk()
    d.apply(bus, "set_param", {"spd_erpm": 2222, "acc_erpm_s2": 3333, "max_cur_a": 7.0})
    d.apply(bus, "set_input", {"pos_deg": 90.0})
    # PKT_SET_POS_SPD=6, data=">ihh" (deg*1e4, spd, acc)
    msg = bus.sent[-1]
    assert msg.arbitration_id == (6 << 8) | AK_ID
    pos, spd, acc = struct.unpack(">ihh", msg.data)
    assert pos == 900000 and spd == 2222 and acc == 3333


def test_overcurrent_trips_in_tick():
    d, bus = _mk()
    d.apply(bus, "set_param", {"max_cur_a": 3.0})
    d.apply(bus, "set_input", {"rpm": 50.0})
    d.on_rx(_status_msg(cur_a=5.0))     # 한계 초과
    n_before = len(bus.sent)
    d.tick(bus)
    # rpm0 정지 프레임 송신 + active 해제
    assert len(bus.sent) > n_before
    assert d._active is None
