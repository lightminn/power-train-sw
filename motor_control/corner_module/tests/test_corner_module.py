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


def test_odrive_can_connect_not_implemented():
    d = DriveOdriveCan()
    with pytest.raises(NotImplementedError):
        d.connect()


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
