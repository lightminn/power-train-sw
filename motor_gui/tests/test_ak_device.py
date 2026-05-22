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
    d.apply(bus, "set_mode", {"control_mode": "brake"})
    d.apply(bus, "set_input", {"brake_cur": 2.0})
    assert bus.sent[-1].arbitration_id == (2 << 8) | AK_ID
    assert bus.sent[-1].data == struct.pack(">i", 2000)


def test_on_rx_parses_status_and_sample_converts_speed():
    d, bus = _mk()
    # -1400 erpm → +10 출력RPM (명령 부호 일치)
    d.on_rx(_status_msg(pos_deg=12.0, spd_erpm=-1400, cur_a=1.5, temp=42, fault=0))
    s = d.sample()
    assert abs(s["ak.pos_deg"] - 12.0) < 0.1
    assert abs(s["ak.speed"] - 10.0) < 0.2          # -1400 erpm → +10 출력RPM (명령 부호 일치)
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


def test_tick_resends_active_when_not_tripped():
    d, bus = _mk()
    d.apply(bus, "set_mode", {"control_mode": "velocity"})
    d.apply(bus, "set_input", {"rpm": 30.0})
    n_before = len(bus.sent)
    d._last_send = 0.0                  # 워치독 간격 강제 경과
    d.tick(bus)
    assert len(bus.sent) > n_before     # 재전송됨
    assert d._tripped is False


def test_estop_clears_active_and_sends_stop():
    d, bus = _mk()
    d.apply(bus, "set_mode", {"control_mode": "velocity"})
    d.apply(bus, "set_input", {"rpm": 30.0})
    n_before = len(bus.sent)
    ack = d.apply(bus, "estop", {})
    assert ack["ok"] is True
    assert d._active is None
    assert len(bus.sent) > n_before     # stop() 가 rpm0 프레임 송신


def test_set_input_wrong_mode_key_rejected():
    d, bus = _mk()
    d.apply(bus, "set_mode", {"control_mode": "velocity"})
    ack = d.apply(bus, "set_input", {"brake_cur": 2.0})   # velocity 모드에 brake 키
    assert ack["ok"] is False


def test_set_param_rpm_units_convert_to_erpm():
    d, bus = _mk()
    d.apply(bus, "set_param", {"spd_rpm": 10.0})    # 10 출력RPM × 140 = 1400 erpm
    assert d._spd == 1400.0
    d.apply(bus, "set_param", {"acc_rpm_s2": 5.0})
    assert d._acc == 700.0


def test_capabilities_tunables_carry_current_value():
    f = AkDevice().capabilities_fragment()
    tk = {t["key"]: t for t in f["tunables"]["ak"]}
    assert tk["spd_erpm"]["value"] == 1500.0
    assert abs(tk["spd_rpm"]["value"] - 1500.0 / 140) < 1e-6
    assert tk["max_cur_a"]["value"] == 5.0
