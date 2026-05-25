from corner_module.config import CornerConfig, clamp


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


from corner_module.fake import FakeSteer, FakeDrive


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
