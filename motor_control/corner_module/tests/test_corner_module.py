import struct

import can
import pytest
from corner_module.config import CornerConfig, clamp
from corner_module.fake import FakeSteer, FakeDrive
from corner_module.corner_module import CornerModule
from corner_module.drive_odrive_can import DriveOdriveCan
from corner_module.actuator import DriveActuator
from corner_module.teleop_dualsense import map_input


def test_default_config_values():
    c = CornerConfig()
    assert c.steer_min_deg == -45.0
    assert c.steer_max_deg == 45.0
    assert c.drive_vel_limit == 5.0
    assert c.watchdog_ms == 300.0
    assert c.loop_hz == 50.0
    assert c.steer_gate is False
    assert c.gate_deg == 10.0
    assert c.stale_ms == 500.0


def test_clamp_bounds():
    assert clamp(100.0, -45.0, 45.0) == 45.0
    assert clamp(-100.0, -45.0, 45.0) == -45.0
    assert clamp(12.0, -45.0, 45.0) == 12.0


def test_fake_steer_converges_to_target_when_armed():
    s = FakeSteer(start_deg=0.0)
    s.connect()
    s.arm()
    s.set_angle(20.0)
    for _ in range(30):
        s.tick()
    assert abs(s.state()["actual_deg"] - 20.0) < 0.5


def test_fake_steer_arm_is_jump_safe():
    # arm 직후 목표는 현재 실제각과 같아야(점프 방지)
    s = FakeSteer(start_deg=15.0)
    s.connect()
    s.arm()
    assert s.state()["target_deg"] == 15.0


def test_fake_drive_arm_targets_zero_velocity():
    d = FakeDrive(start_vel=2.0)
    d.connect()
    d.arm()
    assert d.state()["target_vel"] == 0.0


def test_fake_steer_state_schema():
    s = FakeSteer()
    keys = set(s.state().keys())
    assert keys == {"target_deg", "actual_deg", "cur_a", "fault", "stale"}


def test_fake_drive_state_schema():
    d = FakeDrive()
    keys = set(d.state().keys())
    assert keys == {"target_vel", "actual_vel", "cur_a"}


def _make_cm(steer=None, drive=None, cfg=None, clock=None):
    return CornerModule(
        steer or FakeSteer(),
        drive or FakeDrive(),
        cfg or CornerConfig(),
        clock=clock,
    )


def test_lifecycle_modes():
    cm = _make_cm()
    assert cm.mode == "DISCONNECTED"
    cm.connect()
    assert cm.mode == "IDLE"
    cm.arm()
    assert cm.mode == "ARMED"
    cm.disarm()
    assert cm.mode == "IDLE"
    cm.close()
    assert cm.mode == "DISCONNECTED"


def test_arm_jump_prevention():
    cm = _make_cm(steer=FakeSteer(start_deg=15.0), drive=FakeDrive(start_vel=2.0))
    cm.connect()
    cm.arm()
    assert cm.state()["steer"]["target_deg"] == 15.0
    assert cm.state()["drive"]["target_vel"] == 0.0


def test_set_clamps_targets():
    cm = _make_cm(cfg=CornerConfig(steer_max_deg=45.0, drive_vel_limit=5.0))
    cm.connect()
    cm.arm()
    cm.set(100.0, 99.0)
    cm.tick()
    assert cm.state()["steer"]["target_deg"] == 45.0
    assert cm.state()["drive"]["target_vel"] == 5.0


def test_set_ignored_when_not_armed():
    cm = _make_cm()
    cm.connect()  # IDLE, not ARMED
    cm.set(30.0, 3.0)
    # IDLE 에서는 목표가 반영되지 않아야
    assert cm.state()["steer"]["target_deg"] == 0.0


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def advance(self, sec):
        self.t += sec


def test_watchdog_zeros_drive_on_timeout():
    clk = FakeClock()
    cm = _make_cm(cfg=CornerConfig(watchdog_ms=300.0), clock=clk)
    cm.connect()
    cm.arm()
    cm.set(10.0, 3.0)
    clk.advance(0.1)  # 100ms < 300ms
    cm.tick()
    assert cm.state()["drive"]["target_vel"] == 3.0
    clk.advance(0.5)  # 총 600ms > 300ms
    cm.tick()
    assert cm.state()["drive"]["target_vel"] == 0.0


def test_estop_stops_both_and_faults():
    cm = _make_cm()
    cm.connect()
    cm.arm()
    cm.set(30.0, 3.0)
    cm.tick()
    cm.estop()
    assert cm.mode == "FAULT"
    assert cm.state()["drive"]["target_vel"] == 0.0


def test_steer_fault_triggers_estop():
    s = FakeSteer()
    cm = _make_cm(steer=s)
    cm.connect()
    cm.arm()
    cm.set(10.0, 2.0)
    s.fault = 5
    cm.tick()
    assert cm.mode == "FAULT"


def test_steer_stale_triggers_estop():
    s = FakeSteer()
    cm = _make_cm(steer=s)
    cm.connect()
    cm.arm()
    cm.set(10.0, 2.0)
    s.stale_flag = True
    cm.tick()
    assert cm.mode == "FAULT"


def test_corner_state_schema():
    cm = _make_cm()
    cm.connect()
    st = cm.state()
    assert set(st.keys()) == {"mode", "steer", "drive", "faults"}


def test_steer_gate_holds_drive_until_settled():
    clk = FakeClock()
    cfg = CornerConfig(steer_gate=True, gate_deg=10.0, watchdog_ms=100000.0)
    cm = _make_cm(steer=FakeSteer(start_deg=0.0), cfg=cfg, clock=clk)
    cm.connect()
    cm.arm()
    cm.set(40.0, 4.0)
    cm.tick()  # 조향오차 40 > 10 → 구동 게이트
    assert cm.state()["drive"]["target_vel"] == 0.0
    for _ in range(30):  # 조향 수렴
        cm.set(40.0, 4.0)
        cm.tick()
    assert cm.state()["steer"]["actual_deg"] > 35.0
    assert cm.state()["drive"]["target_vel"] == 4.0  # 게이트 해제


def test_odrive_can_is_drive_actuator():
    d = DriveOdriveCan()
    assert isinstance(d, DriveActuator)


# ---------------------------------------------------------------------------
# DriveOdriveCan — 가짜 socketcan 버스로 프레임/텔레메트리 검증 (무하드웨어)
# ---------------------------------------------------------------------------
class _FakeCanBus:
    """송신을 기록하고 지정한 rx 프레임을 순서대로 돌려주는 가짜 버스."""

    def __init__(self, rx=None):
        self.sent = []
        self._rx = list(rx or [])
        self.shutdown_called = False

    def send(self, msg):
        self.sent.append(msg)

    def recv(self, timeout=0.0):
        return self._rx.pop(0) if self._rx else None

    def shutdown(self):
        self.shutdown_called = True


def _sent(bus, node_id, cmd):
    """bus.sent 에서 (node_id, cmd) arbitration id 프레임만."""
    arb = (node_id << 5) | cmd
    return [m for m in bus.sent if m.arbitration_id == arb]


def _hb(node, err=0, state=1):
    return can.Message(arbitration_id=(node << 5) | 0x01,
                       data=struct.pack("<I", err) + bytes([state, 0, 0, 0]),
                       is_extended_id=False)


def _enc(node, pos, vel):
    return can.Message(arbitration_id=(node << 5) | 0x09,
                       data=struct.pack("<ff", pos, vel), is_extended_id=False)


def _iq(node, iq_sp, iq_meas):
    return can.Message(arbitration_id=(node << 5) | 0x14,
                       data=struct.pack("<ff", iq_sp, iq_meas), is_extended_id=False)


def test_can_drive_connect_reuses_injected_bus():
    bus = _FakeCanBus()
    d = DriveOdriveCan(node_id=11, bus=bus)
    d.connect()                       # 주입 버스면 새 소켓 안 엶, 송신 없음
    assert bus.sent == []


def test_can_drive_arm_enters_closed_loop_at_zero_velocity():
    bus = _FakeCanBus()
    d = DriveOdriveCan(node_id=12, bus=bus)
    d.connect()
    d.arm()
    mode = _sent(bus, 12, 0x0B)       # Set_Controller_Mode = VELOCITY(2), PASSTHROUGH(1)
    assert mode and struct.unpack("<ii", bytes(mode[-1].data)) == (2, 1)
    vel = _sent(bus, 12, 0x0D)        # Set_Input_Vel = 0 (점프 방지)
    assert vel and struct.unpack("<ff", bytes(vel[-1].data)) == (0.0, 0.0)
    st = _sent(bus, 12, 0x07)         # Set_Axis_State = CLOSED_LOOP(8)
    assert st and struct.unpack("<I", bytes(st[-1].data[:4]))[0] == 8
    assert d.state()["target_vel"] == 0.0


def test_can_drive_set_velocity_deferred_until_tick():
    bus = _FakeCanBus()
    d = DriveOdriveCan(node_id=11, bus=bus)
    d.connect()
    d.set_velocity(3.0)
    assert _sent(bus, 11, 0x0D) == []          # tick 전엔 전송 안 함
    assert d.state()["target_vel"] == 3.0


def test_can_drive_tick_sends_target_and_rtr_polls():
    bus = _FakeCanBus()
    d = DriveOdriveCan(node_id=11, bus=bus)
    d.connect()
    d.set_velocity(2.5)
    d.tick()
    vel = _sent(bus, 11, 0x0D)
    assert struct.unpack("<ff", bytes(vel[-1].data))[0] == pytest.approx(2.5)
    assert any(m.is_remote_frame and m.arbitration_id == (11 << 5) | 0x09 for m in bus.sent)
    assert any(m.is_remote_frame and m.arbitration_id == (11 << 5) | 0x14 for m in bus.sent)


def test_can_drive_state_parses_heartbeat_encoder_iq():
    node = 13
    bus = _FakeCanBus(rx=[_hb(node, err=0, state=8), _enc(node, 1.5, 0.98), _iq(node, 0.3, 0.42)])
    d = DriveOdriveCan(node_id=node, bus=bus, stale_ms=1000.0)
    d.connect()
    d.tick()                                   # poll 이 rx 소비
    st = d.state()
    assert st["actual_vel"] == pytest.approx(0.98)
    assert st["cur_a"] == pytest.approx(0.42)
    assert st["axis_error"] == 0
    assert st["stale"] is False


def test_can_drive_ignores_other_nodes():
    bus = _FakeCanBus(rx=[_enc(15, 9.9, 9.9)])   # 남의 노드
    d = DriveOdriveCan(node_id=11, bus=bus, stale_ms=1000.0)
    d.connect()
    d.tick()
    st = d.state()
    assert st["actual_vel"] == 0.0
    assert st["stale"] is True


def test_can_drive_stale_without_rx():
    bus = _FakeCanBus()
    d = DriveOdriveCan(node_id=11, bus=bus)
    d.connect()
    d.tick()
    assert d.state()["stale"] is True


def test_can_drive_estop_commands_idle():
    bus = _FakeCanBus()
    d = DriveOdriveCan(node_id=14, bus=bus)
    d.connect()
    d.set_velocity(4.0)
    d.estop()
    vel = _sent(bus, 14, 0x0D)
    assert struct.unpack("<ff", bytes(vel[-1].data)) == (0.0, 0.0)
    st = _sent(bus, 14, 0x07)
    assert struct.unpack("<I", bytes(st[-1].data[:4]))[0] == 1     # IDLE
    assert d.state()["target_vel"] == 0.0


def test_can_drive_close_injected_bus_not_shutdown():
    bus = _FakeCanBus()
    d = DriveOdriveCan(node_id=11, bus=bus)
    d.connect()
    d.close()
    assert _sent(bus, 11, 0x07)                # IDLE 전송
    assert bus.shutdown_called is False        # 주입 버스는 소유 아님


def test_map_input_neutral_is_zero():
    cfg = CornerConfig(steer_max_deg=45.0, drive_vel_limit=5.0)
    steer, drive = map_input(left_x=0.0, rt=0.0, lt=0.0, cfg=cfg)
    assert steer == 0.0
    assert drive == 0.0


def test_map_input_deadzone():
    cfg = CornerConfig()
    steer, drive = map_input(left_x=0.03, rt=0.02, lt=0.0, cfg=cfg, deadzone=0.05)
    assert steer == 0.0
    assert drive == 0.0


def test_map_input_full_steer_and_drive():
    cfg = CornerConfig(steer_max_deg=45.0, drive_vel_limit=5.0)
    steer, drive = map_input(left_x=1.0, rt=1.0, lt=0.0, cfg=cfg)
    assert steer == 45.0
    assert drive == 5.0


def test_map_input_reverse_drive():
    cfg = CornerConfig(drive_vel_limit=5.0)
    steer, drive = map_input(left_x=0.0, rt=0.0, lt=1.0, cfg=cfg)
    assert drive == -5.0


def test_steer_overcurrent_triggers_estop():
    s = FakeSteer()
    cm = _make_cm(steer=s, cfg=CornerConfig(steer_current_limit_a=10.0))
    cm.connect()
    cm.arm()
    cm.set(10.0, 2.0)
    s.cur_a = 15.0  # 한계(10A) 초과
    cm.tick()
    assert cm.mode == "FAULT"
