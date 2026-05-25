from corner_module.config import CornerConfig, clamp
from corner_module.fake import FakeSteer, FakeDrive
from corner_module.corner_module import CornerModule


def test_default_config_values():
    c = CornerConfig()
    assert c.steer_min_deg == -45.0
    assert c.steer_max_deg == 45.0
    assert c.drive_vel_limit == 5.0
    assert c.watchdog_ms == 300.0
    assert c.loop_hz == 50.0
    assert c.steer_gate is False
    assert c.gate_deg == 10.0
    assert c.stale_ms == 200.0


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
