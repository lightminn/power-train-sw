"""ChassisManager(WP3) 통합 검증 — 코너 6개를 하나의 차체로 묶어
kinematics 결과를 각 코너에 분배하고, estop 전파·안전 interlock·워치독을 총괄한다.

전부 fake 드라이버(무하드웨어). 실행:
  motor_control/ 에서  `python -m pytest chassis/tests/test_chassis_manager.py -q`
"""
from dataclasses import FrozenInstanceError

import pytest

from chassis.kinematics import default_geometry, solve
from chassis.chassis_manager import (
    ChassisManager, ChassisConfig, WheelMap, DEFAULT_WHEEL_MAP, build_corners,
)
from corner_module.config import CornerConfig
from corner_module.corner_module import CornerModule
from corner_module.fake import FakeSteer, FakeDrive
from corner_module.null_steer import NullSteer
from corner_module.actuator import SteerActuator

WHEEL_NAMES = {"front_left", "front_right", "mid_left", "mid_right",
               "rear_left", "rear_right"}


# ── 테스트 더블 ──────────────────────────────────────────────────────────

def _fake_corners(cfg=None):
    """6바퀴: 조향바퀴 4개=FakeSteer, 중간 2개=NullSteer(고정), 구동 전부 FakeDrive."""
    cfg = cfg or CornerConfig()
    corners = {}
    for w in default_geometry().wheels:
        steer = FakeSteer() if w.steerable else NullSteer()
        corners[w.name] = CornerModule(steer, FakeDrive(), cfg)
    return corners


def _armed_manager(cfg=None, clock=None):
    m = ChassisManager(_fake_corners(cfg and cfg.corner), cfg=cfg, clock=clock)
    m.connect()
    assert m.arm() is True
    return m


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def advance(self, sec):
        self.t += sec


class FalseyClock(FakeClock):
    def __bool__(self):
        return False


# ── 매핑·라이프사이클 ────────────────────────────────────────────────────

def test_requires_every_geometry_wheel_mapped():
    corners = _fake_corners()
    del corners["mid_left"]                       # 한 바퀴 누락
    with pytest.raises(ValueError):
        ChassisManager(corners)


def test_rejects_unexpected_geometry_wheel_mapping():
    corners = _fake_corners()
    corners["bogus_wheel"] = _fake_corners()["front_left"]
    with pytest.raises(ValueError, match="bogus_wheel"):
        ChassisManager(corners)


def test_lifecycle_modes():
    m = ChassisManager(_fake_corners())
    assert m.mode == "DISCONNECTED"
    m.connect()
    assert m.mode == "IDLE"
    assert m.arm() is True
    assert m.mode == "ARMED"
    m.disarm()
    assert m.mode == "IDLE"
    m.close()
    assert m.mode == "DISCONNECTED"


def test_arm_only_succeeds_from_idle_and_repeated_arm_is_noop():
    m = ChassisManager(_fake_corners())
    assert m.arm() is False
    assert m.mode == "DISCONNECTED"
    assert all(c.mode == "DISCONNECTED" for c in m.corners.values())

    m.connect()
    assert m.arm() is True
    m.set(0.4, 0.0)
    m.tick()
    targets_before = _drive_targets(m)
    assert m.arm() is False
    assert m.mode == "ARMED"
    assert _drive_targets(m) == targets_before


def test_set_ignored_when_not_armed():
    m = ChassisManager(_fake_corners())
    m.connect()                                   # IDLE, not ARMED
    m.set(0.5, 0.0)
    m.tick()
    for c in m.corners.values():
        assert c.state()["drive"]["target_vel"] == 0.0


# ── 분배 정확성 (kinematics 결과가 각 코너에 그대로 도달) ─────────────────

def test_straight_distributes_equal_forward():
    m = _armed_manager()
    m.set(0.5, 0.0)
    m.tick()
    expected = solve(default_geometry(), 0.5, 0.0)
    for name, c in m.corners.items():
        st = c.state()
        assert st["steer"]["target_deg"] == pytest.approx(0.0, abs=1e-9)
        assert st["drive"]["target_vel"] == pytest.approx(
            expected.wheels[name].drive_turns_per_s)


def test_left_turn_matches_kinematics_per_corner():
    m = _armed_manager()
    m.set(0.4, 0.4)                               # 좌회전
    m.tick()
    expected = solve(default_geometry(), 0.4, 0.4)
    for name, c in m.corners.items():
        st = c.state()
        assert st["steer"]["target_deg"] == pytest.approx(
            expected.wheels[name].steer_deg, abs=1e-6)
        assert st["drive"]["target_vel"] == pytest.approx(
            expected.wheels[name].drive_turns_per_s)
    # 성질 확인: 안쪽(좌)이 더 꺾이고 바깥(우)이 더 빠름
    assert (m.corners["front_left"].state()["steer"]["target_deg"]
            > m.corners["front_right"].state()["steer"]["target_deg"] > 0)


def test_pivot_drives_mid_wheels_opposite():
    m = _armed_manager()
    m.set(0.0, 0.5)                               # 제자리 좌회전
    m.tick()
    ml = m.corners["mid_left"].state()["drive"]["target_vel"]
    mr = m.corners["mid_right"].state()["drive"]["target_vel"]
    assert ml == pytest.approx(-mr, abs=1e-9)
    assert abs(ml) > 1e-6                          # 중간바퀴도 굴러 회전 생성
    # 조향바퀴는 접선방향으로 꺾임(0 아님)
    assert abs(m.corners["front_left"].state()["steer"]["target_deg"]) > 1e-3


# ── 안전 interlock (hold → 구동 0, ESTOP → 전체 정지) ─────────────────────

def test_checking_is_auto_clearing_motion_hold_without_disarm():
    m = _armed_manager()
    m.set(0.4, 0.4)
    m.update_external_safety("CHECKING", False, "warming up")
    m.tick()
    assert m.mode == "ARMED"
    assert m.snapshot().stop_state == "MOTION_HOLD"
    assert all(d == 0.0 for d in _drive_targets(m).values())
    assert m.corners["front_left"].state()["steer"]["target_deg"] > 0

    m.update_external_safety("VALID", False, "clear")
    m.tick()
    assert m.mode == "ARMED"
    assert m.snapshot().stop_state == "RUN"
    assert m.corners["front_left"].state()["drive"]["target_vel"] != 0.0


def test_external_estop_latches_after_condition_clears():
    m = _armed_manager()
    m.update_external_safety("VALID", True, "too_close")
    m.tick()
    assert m.mode == "ESTOP"
    m.update_external_safety("VALID", False, "clear")
    assert m.mode == "ESTOP"
    assert m.reset_estop() is True
    assert m.mode == "IDLE"
    assert m.arm() is True


def test_arm_rejected_before_estop_reset():
    m = _armed_manager()
    m.estop("manual", "button")
    assert m.arm() is False
    assert m.mode == "ESTOP"
    assert all(c.mode == "FAULT" for c in m.corners.values())
    assert all(d == 0.0 for d in _drive_targets(m).values())


def test_active_safety_condition_rejects_reset():
    m = _armed_manager()
    m.update_external_safety("NO_RESPONSE", True, "sensor")
    m.tick()
    assert m.reset_estop() is False
    assert m.mode == "ESTOP"


def test_safety_link_stale_blocks_reset_until_cleared_and_reset_never_arms():
    m = _armed_manager()
    m.set_safety_link_stale(True, "topic timeout")
    m.tick()
    assert m.mode == "ESTOP"
    assert m.reset_estop() is False

    m.set_safety_link_stale(False, "fresh")
    assert m.reset_estop() is True
    assert m.mode == "IDLE"
    assert all(c.mode == "IDLE" for c in m.corners.values())
    m.tick()
    assert m.mode == "IDLE"
    assert m.arm() is True


def test_reset_estop_when_not_latched_is_noop():
    m = _armed_manager()
    m.set(0.4, 0.0)
    m.tick()
    targets_before = _drive_targets(m)

    assert m.reset_estop() is False
    assert m.mode == "ARMED"
    assert all(c.mode == "ARMED" for c in m.corners.values())
    assert _drive_targets(m) == targets_before


def test_first_estop_cause_persists_after_condition_clears():
    m = _armed_manager()
    m.update_external_safety("VALID", True, "too_close")
    m.tick()
    m.update_external_safety("VALID", False, "clear")
    safety = m.state()["safety"]
    assert safety.first_source == "us100"
    assert safety.first_detail == "too_close"
    assert safety.active_estop_sources == ()
    assert m.snapshot().stop_state == "ESTOP"


def test_idle_hazard_becomes_estop_and_rejects_arm():
    m = ChassisManager(_fake_corners())
    m.connect()
    m.set_safety_link_stale(True, "never received")
    m.tick()
    assert m.mode == "ESTOP"
    assert m.arm() is False


# ── 워치독 (chassis.set 끊기면 구동 0) ───────────────────────────────────

def test_cmd_watchdog_is_motion_hold_not_estop():
    clk = FakeClock()
    cfg = ChassisConfig(watchdog_ms=300.0)
    m = _armed_manager(cfg=cfg, clock=clk)
    m.set(0.4, 0.0)
    clk.advance(0.5)
    m.tick()
    assert m.mode == "ARMED"
    assert m.snapshot().stop_state == "MOTION_HOLD"
    assert all(d == 0.0 for d in _drive_targets(m).values())

    m.set(0.4, 0.0)
    m.tick()
    assert m.mode == "ARMED"
    assert m.snapshot().stop_state == "RUN"
    assert m.corners["front_left"].state()["drive"]["target_vel"] != 0.0


def test_falsey_injected_clock_drives_watchdog():
    clk = FalseyClock()
    m = _armed_manager(cfg=ChassisConfig(watchdog_ms=300.0), clock=clk)
    m.set(0.4, 0.0)
    clk.advance(0.5)
    m.tick()
    assert m.snapshot().stop_state == "MOTION_HOLD"


# ── estop 전파 (1곳 트립 → 6코너 전부 정지) ──────────────────────────────

def test_corner_fault_propagates_to_all():
    m = _armed_manager()
    m.set(0.4, 0.4)
    m.tick()
    m.corners["front_left"].steer.fault = 5        # 조향 fault 주입
    m.tick()
    assert m.mode == "ESTOP"
    for c in m.corners.values():
        assert c.mode == "FAULT"
        assert c.state()["drive"]["target_vel"] == 0.0


def test_estop_stops_all_corners():
    m = _armed_manager()
    m.set(0.4, 0.4)
    m.tick()
    m.estop()
    assert m.mode == "ESTOP"
    for c in m.corners.values():
        assert c.state()["drive"]["target_vel"] == 0.0


class RaisingCorner:
    def __init__(self, wrapped, error=None):
        self._wrapped = wrapped
        self.mode = wrapped.mode
        self.error = error or RuntimeError("stop failed")

    def __getattr__(self, name):
        return getattr(self._wrapped, name)

    def estop(self):
        self.mode = "FAULT"
        raise self.error


def test_estop_continues_after_one_corner_raises():
    corners = _fake_corners()
    m = ChassisManager(corners)
    m.connect()
    assert m.arm() is True
    m.corners["front_left"] = RaisingCorner(m.corners["front_left"])
    m.estop("manual")
    assert m.mode == "ESTOP"
    assert isinstance(m.state()["last_estop_error"], RuntimeError)
    for name, corner in m.corners.items():
        if name != "front_left":
            assert corner.mode == "FAULT"


class StopSignal(BaseException):
    pass


def test_estop_continues_after_corner_raises_baseexception_and_keeps_first():
    corners = _fake_corners()
    m = ChassisManager(corners)
    m.connect()
    assert m.arm() is True
    first_error = StopSignal("first stop failed")
    m.corners["front_left"] = RaisingCorner(
        m.corners["front_left"], first_error,
    )
    m.corners["front_right"] = RaisingCorner(
        m.corners["front_right"], RuntimeError("second stop failed"),
    )

    m.estop("manual")

    assert m.mode == "ESTOP"
    assert m.state()["last_estop_error"] is first_error
    assert all(c.mode == "FAULT" for c in m.corners.values())


def test_snapshot_has_six_wheels_in_geometry_order():
    m = _armed_manager()
    snap = m.snapshot()
    assert [wheel.name for wheel in snap.wheels] == [
        "front_left", "front_right", "mid_left",
        "mid_right", "rear_left", "rear_right",
    ]
    assert snap.healthy is True


@pytest.mark.parametrize(
    ("component", "attribute", "value", "snapshot_field"),
    [
        ("drive", "stale_flag", True, "drive_stale"),
        ("steer", "stale_flag", True, "steer_stale"),
        ("drive", "axis_error", 0x10, "drive_axis_error"),
        ("steer", "fault", 5, "steer_fault"),
    ],
)
def test_snapshot_reports_stale_and_errors_unhealthy(
        component, attribute, value, snapshot_field):
    m = _armed_manager()
    setattr(getattr(m.corners["front_left"], component), attribute, value)
    snap = m.snapshot()
    assert snap.healthy is False
    assert getattr(snap.wheels[0], snapshot_field) == value


def test_snapshot_is_frozen_detached_and_uses_actual_feedback():
    m = _armed_manager()
    corner = m.corners["front_left"]
    corner.drive._target = 8.0
    corner.drive._actual = 1.25
    corner.steer._target = 30.0
    corner.steer._actual = 12.5
    corner.drive.cur_a = 3.5
    corner.steer.cur_a = 1.5

    snap = m.snapshot()
    wheel = snap.wheels[0]
    assert isinstance(snap.wheels, tuple)
    assert wheel.drive_turns_per_s == 1.25
    assert wheel.steer_deg == 12.5
    assert wheel.drive_current_a == 3.5
    assert wheel.steer_current_a == 1.5

    corner.drive._actual = 9.0
    corner.steer._actual = -9.0
    assert wheel.drive_turns_per_s == 1.25
    assert wheel.steer_deg == 12.5
    with pytest.raises(FrozenInstanceError):
        snap.chassis_mode = "IDLE"
    with pytest.raises(FrozenInstanceError):
        wheel.steer_deg = 0.0


def test_snapshot_missing_optional_health_keys_uses_safe_defaults():
    m = _armed_manager()
    corner = m.corners["front_left"]
    corner.drive.state = lambda: {"actual_vel": 0.5}
    corner.steer.state = lambda: {"actual_deg": 2.0}
    snap = m.snapshot()
    wheel = snap.wheels[0]
    assert snap.healthy is True
    assert wheel.drive_current_a == 0.0
    assert wheel.steer_current_a == 0.0
    assert wheel.drive_stale is False
    assert wheel.steer_stale is False
    assert wheel.drive_axis_error == 0
    assert wheel.steer_fault == 0


def test_drive_current_alone_does_not_add_python_health_threshold():
    m = _armed_manager()
    m.corners["front_left"].drive.cur_a = 999.0
    snap = m.snapshot()
    assert snap.healthy is True
    assert snap.wheels[0].drive_current_a == 999.0


def test_state_schema():
    m = _armed_manager()
    st = m.state()
    assert set(st.keys()) >= {
        "mode", "v", "omega", "safety", "last_estop_error", "corners",
    }
    assert "verdict" not in st
    assert set(st["corners"].keys()) == WHEEL_NAMES


# ── 매핑 표 · 팩토리 ──────────────────────────────────────────────────────

def test_default_wheel_map_covers_geometry():
    assert {wm.wheel for wm in DEFAULT_WHEEL_MAP} == WHEEL_NAMES
    fixed = {wm.wheel for wm in DEFAULT_WHEEL_MAP if wm.steer_can_id is None}
    assert fixed == {"mid_left", "mid_right"}      # 중간 2개만 조향 없음


def test_build_corners_middle_wheels_use_null_steer():
    corners = build_corners(steer_factory=lambda cid: FakeSteer(),
                            drive_factory=lambda nid: FakeDrive())
    assert set(corners) == WHEEL_NAMES
    assert isinstance(corners["mid_left"].steer, NullSteer)
    assert isinstance(corners["mid_right"].steer, NullSteer)
    assert isinstance(corners["front_left"].steer, FakeSteer)
    assert isinstance(corners["front_left"].drive, FakeDrive)


# ── NullSteer (고정 바퀴용 no-op 조향) ────────────────────────────────────

def test_null_steer_is_steer_actuator_and_never_moves():
    s = NullSteer()
    assert isinstance(s, SteerActuator)
    s.connect()
    s.arm()
    s.set_angle(30.0)
    s.tick()
    st = s.state()
    assert st["actual_deg"] == 0.0
    assert st["fault"] == 0
    assert st["stale"] is False
    assert set(st.keys()) == {"target_deg", "actual_deg", "cur_a", "fault", "stale"}


# ── 최저 구동속도 플로어 (저속 코깅존 회피) ─────────────────────────────────

def _drive_targets(m):
    st = m.state()
    return {n: st["corners"][n]["drive"]["target_vel"] for n in WHEEL_NAMES}


def test_min_drive_floor_lifts_small_forward_to_min():
    # v=0.1 m/s → 바퀴 ~0.16 rev/s(<1.0) → 전부 1.0 으로 끌어올림
    m = _armed_manager(ChassisConfig(min_drive_turns_per_s=1.0))
    m.set(0.1, 0.0)
    m.tick()
    for n, d in _drive_targets(m).items():
        assert d == pytest.approx(1.0), (n, d)


def test_min_drive_floor_preserves_sign_on_reverse():
    m = _armed_manager(ChassisConfig(min_drive_turns_per_s=1.0))
    m.set(-0.1, 0.0)                       # 살짝 후진
    m.tick()
    for n, d in _drive_targets(m).items():
        assert d == pytest.approx(-1.0), (n, d)


def test_min_drive_floor_keeps_zero_stopped():
    # 정지(v=ω=0)는 플로어에 안 걸림 — 0 유지
    m = _armed_manager(ChassisConfig(min_drive_turns_per_s=1.0))
    m.set(0.0, 0.0)
    m.tick()
    assert all(d == 0.0 for d in _drive_targets(m).values())


def test_min_drive_floor_off_by_default_keeps_small():
    # 기본(min=0)은 작은 명령을 그대로 둠(기존/자율주행 무영향)
    m = _armed_manager(ChassisConfig())
    m.set(0.1, 0.0)
    m.tick()
    assert all(abs(d) < 0.5 for d in _drive_targets(m).values())


def test_min_drive_floor_leaves_large_commands_unchanged():
    cfg = ChassisConfig(min_drive_turns_per_s=1.0)
    cfg.geometry.drive_limit_mps = 1.5    # 텔레옵처럼 속도상한 올림(0.8 클램프 회피)
    m = _armed_manager(cfg)
    m.set(1.2, 0.0)                        # 1.2 m/s ≈ 1.9 rev/s > 1.0 → 플로어 안 걸림
    m.tick()
    for n, d in _drive_targets(m).items():
        assert d > 1.5, (n, d)             # 실제 큰 값(≈1.9), 1.0 으로 안 깎임
