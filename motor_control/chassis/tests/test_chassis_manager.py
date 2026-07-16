"""ChassisManager(WP3) 통합 검증 — 코너 6개를 하나의 차체로 묶어
kinematics 결과를 각 코너에 분배하고, estop 전파·안전 interlock·워치독을 총괄한다.

전부 fake 드라이버(무하드웨어). 실행:
  motor_control/ 에서  `python -m pytest chassis/tests/test_chassis_manager.py -q`
"""
import math
from dataclasses import FrozenInstanceError
from datetime import datetime, timezone

import pytest

from chassis.kinematics import default_geometry, solve
from chassis.chassis_manager import (
    ChassisManager, ChassisConfig, WheelMap, DEFAULT_WHEEL_MAP, build_corners,
)
from chassis.telemetry import (
    CanBusHealth,
    CanBusStatsSampler,
    build_can_health_event,
)
from chassis.wheel_consistency import WheelConsistencyConfig
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


class StopSignal(BaseException):
    pass


class ArmRaisingCorner:
    def __init__(self, wrapped, error):
        self._wrapped = wrapped
        self.error = error

    @property
    def mode(self):
        return self._wrapped.mode

    def __getattr__(self, name):
        return getattr(self._wrapped, name)

    def arm(self):
        raise self.error


class ResetScriptCorner:
    def __init__(self, wrapped, outcomes):
        self._wrapped = wrapped
        self.outcomes = list(outcomes)
        self.reset_calls = 0

    @property
    def mode(self):
        return self._wrapped.mode

    def __getattr__(self, name):
        return getattr(self._wrapped, name)

    def reset_fault(self):
        self.reset_calls += 1
        outcome = self.outcomes.pop(0) if self.outcomes else None
        if isinstance(outcome, BaseException):
            raise outcome
        if outcome is False:
            return False
        if outcome == "stuck":
            return True
        return self._wrapped.reset_fault()


class HeartbeatCacheDrive(FakeDrive):
    """Fake RX drain: state() consumes heartbeat, health_state() is cache-only."""

    def __init__(self):
        super().__init__()
        self.stale_flag = True
        self._heartbeat_pending = False
        self.state_calls = 0

    def inject_heartbeat(self):
        self._heartbeat_pending = True

    def state(self):
        self.state_calls += 1
        if self._heartbeat_pending:
            self._heartbeat_pending = False
            self.stale_flag = False
        return self.health_state()

    def health_state(self):
        state = super().state()
        state.update({
            "last_heartbeat_age_ms": None if self.stale_flag else 0.0,
            "last_encoder_age_ms": None,
            "axis_state": 1,
            "recovery_count": 0,
        })
        return state


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


def test_later_corner_arm_failure_rolls_back_all_corners_and_latches_estop():
    m = ChassisManager(_fake_corners())
    m.connect()
    failure = RuntimeError("late arm failed")
    m.corners["rear_left"] = ArmRaisingCorner(
        m.corners["rear_left"], failure,
    )

    assert m.arm() is False

    safety = m.state()["safety"]
    assert m.mode == "ESTOP"
    assert safety.estop_latched is True
    assert safety.first_source == "arm_failure"
    assert "rear_left" in safety.first_detail
    assert "late arm failed" in safety.first_detail
    assert all(c.mode == "FAULT" for c in m.corners.values())
    assert all(d == 0.0 for d in _drive_targets(m).values())


def test_arm_baseexception_rolls_back_before_reraising():
    m = ChassisManager(_fake_corners())
    m.connect()
    failure = StopSignal("arm interrupted")
    m.corners["rear_left"] = ArmRaisingCorner(
        m.corners["rear_left"], failure,
    )

    with pytest.raises(StopSignal) as caught:
        m.arm()

    assert caught.value is failure
    assert m.mode == "ESTOP"
    assert m.state()["safety"].first_source == "arm_failure"
    assert all(c.mode == "FAULT" for c in m.corners.values())


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

def test_checking_auto_clears_but_requires_fresh_command_without_disarm():
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
    assert m.snapshot().stop_state == "MOTION_HOLD"
    assert all(d == 0.0 for d in _drive_targets(m).values())
    assert m.corners["front_left"].state()["steer"]["target_deg"] > 0

    m.set(0.4, 0.4)
    m.tick()
    assert m.snapshot().stop_state == "RUN"
    assert m.corners["front_left"].state()["drive"]["target_vel"] != 0.0


def test_commands_received_during_motion_hold_are_discarded():
    m = _armed_manager()
    m.set_motion_hold("network", True, "link down")
    m.set(0.7, 0.3)
    m.tick()

    assert m.mode == "ARMED"
    assert m.snapshot().stop_state == "MOTION_HOLD"
    assert all(d == 0.0 for d in _drive_targets(m).values())

    m.set_motion_hold("network", False, "link restored")
    m.tick()
    assert m.snapshot().stop_state == "MOTION_HOLD"
    assert all(d == 0.0 for d in _drive_targets(m).values())

    m.set(0.7, 0.3)
    m.tick()
    assert m.snapshot().stop_state == "RUN"


def test_arm_motion_hold_transition_discards_pending_command():
    m = _armed_manager()
    m.set(0.4, 0.4)
    m.tick()
    assert any(d != 0.0 for d in _drive_targets(m).values())

    m.set_arm_motion_hold(True, "arm_status_stale")
    m.tick()
    assert all(d == 0.0 for d in _drive_targets(m).values())
    assert m.state()["v"] == 0.0
    assert m.state()["omega"] == 0.0

    m.set_arm_motion_hold(False)
    m.tick()
    assert m.snapshot().stop_state == "RUN"
    assert all(d == 0.0 for d in _drive_targets(m).values())

    m.set(0.4, 0.4)
    m.tick()
    assert any(d != 0.0 for d in _drive_targets(m).values())


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


def test_reset_false_while_faulted_reestops_all_and_allows_retry():
    m = _armed_manager()
    m.estop("manual", "button")
    scripted = {
        name: ResetScriptCorner(
            corner,
            [False, None] if name == "front_left" else [None, None],
        )
        for name, corner in m.corners.items()
    }
    m.corners.update(scripted)

    assert m.reset_estop() is False
    assert all(corner.reset_calls == 1 for corner in scripted.values())
    assert m.mode == "ESTOP"
    assert m.state()["safety"].estop_latched is True
    assert all(c.mode == "FAULT" for c in m.corners.values())

    assert m.reset_estop() is True
    assert all(corner.reset_calls == 2 for corner in scripted.values())
    assert m.mode == "IDLE"
    assert m.state()["safety"].estop_latched is False
    assert all(c.mode == "IDLE" for c in m.corners.values())


def test_reset_exception_attempts_later_corners_reestops_and_allows_retry():
    m = _armed_manager()
    m.estop("manual", "button")
    failure = RuntimeError("reset failed")
    scripted = {
        name: ResetScriptCorner(
            corner,
            [failure, None] if name == "front_left" else [None, None],
        )
        for name, corner in m.corners.items()
    }
    m.corners.update(scripted)

    assert m.reset_estop() is False
    assert all(corner.reset_calls == 1 for corner in scripted.values())
    assert m.mode == "ESTOP"
    assert m.state()["safety"].estop_latched is True
    assert all(c.mode == "FAULT" for c in m.corners.values())

    assert m.reset_estop() is True
    assert all(corner.reset_calls == 2 for corner in scripted.values())
    assert m.mode == "IDLE"
    assert m.state()["safety"].estop_latched is False
    assert all(c.mode == "IDLE" for c in m.corners.values())


def test_reset_success_without_idle_reestops_all_and_allows_retry():
    m = _armed_manager()
    m.estop("manual", "button")
    scripted = {
        name: ResetScriptCorner(
            corner,
            ["stuck", None] if name == "front_left" else [None, None],
        )
        for name, corner in m.corners.items()
    }
    m.corners.update(scripted)

    assert m.reset_estop() is False
    assert all(corner.reset_calls == 1 for corner in scripted.values())
    assert m.mode == "ESTOP"
    assert m.state()["safety"].estop_latched is True
    assert all(c.mode == "FAULT" for c in m.corners.values())

    assert m.reset_estop() is True
    assert all(corner.reset_calls == 2 for corner in scripted.values())
    assert m.mode == "IDLE"
    assert all(c.mode == "IDLE" for c in m.corners.values())


def test_reset_baseexception_reestops_all_before_reraising():
    m = _armed_manager()
    m.estop("manual", "button")
    failure = StopSignal("reset interrupted")
    scripted = {
        name: ResetScriptCorner(
            corner,
            [failure] if name == "front_left" else [None],
        )
        for name, corner in m.corners.items()
    }
    m.corners.update(scripted)

    with pytest.raises(StopSignal) as caught:
        m.reset_estop()

    assert caught.value is failure
    assert all(corner.reset_calls == 1 for corner in scripted.values())
    assert m.mode == "ESTOP"
    assert m.state()["safety"].estop_latched is True
    assert all(c.mode == "FAULT" for c in m.corners.values())


def test_reset_false_is_acceptable_for_corner_already_idle():
    m = _armed_manager()
    m.estop("manual", "button")
    m.corners["front_left"].disarm()
    assert m.corners["front_left"].mode == "IDLE"

    assert m.reset_estop() is True
    assert m.mode == "IDLE"
    assert all(c.mode == "IDLE" for c in m.corners.values())


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


def test_estop_diagnostic_keeps_first_error_across_repeated_calls():
    m = _armed_manager()
    first_corner = m.corners["front_left"]
    first_error = RuntimeError("first stop failed")
    m.corners["front_left"] = RaisingCorner(first_corner, first_error)

    m.estop("manual", "first")
    assert m.state()["last_estop_error"] is first_error

    m.corners["front_left"] = first_corner
    m.estop("manual", "repeat succeeded")
    assert m.state()["last_estop_error"] is first_error

    second_error = RuntimeError("later stop failed")
    m.corners["front_right"] = RaisingCorner(
        m.corners["front_right"], second_error,
    )
    m.estop("manual", "repeat failed differently")
    assert m.state()["last_estop_error"] is first_error


def test_successful_reset_clears_estop_diagnostic_for_new_episode():
    m = _armed_manager()
    first_corner = m.corners["front_left"]
    first_error = RuntimeError("first episode")
    m.corners["front_left"] = RaisingCorner(first_corner, first_error)
    m.estop("manual", "first")
    assert m.state()["last_estop_error"] is first_error

    m.corners["front_left"] = first_corner
    first_corner.estop()
    assert m.reset_estop() is True
    assert m.state()["last_estop_error"] is None
    assert m.arm() is True

    second_error = RuntimeError("second episode")
    m.corners["front_right"] = RaisingCorner(
        m.corners["front_right"], second_error,
    )
    m.estop("manual", "second")
    assert m.state()["last_estop_error"] is second_error


def test_snapshot_has_six_wheels_in_geometry_order():
    m = _armed_manager()
    snap = m.snapshot()
    assert [wheel.name for wheel in snap.wheels] == [
        "front_left", "front_right", "mid_left",
        "mid_right", "rear_left", "rear_right",
    ]
    assert snap.healthy is True


def test_idle_tick_drains_corner_receive_before_cached_health_snapshot():
    corners = _fake_corners()
    drive = HeartbeatCacheDrive()
    corners["front_left"] = CornerModule(FakeSteer(), drive, CornerConfig())
    m = ChassisManager(corners)
    m.connect()

    # snapshot() must stay cache-only: the queued heartbeat is not consumed here.
    assert m.snapshot().odrive_nodes[0].stale is True
    drive.inject_heartbeat()

    m.tick()
    snapshot = m.snapshot()

    assert m.mode == "IDLE"
    assert drive.state_calls == 1
    assert snapshot.odrive_nodes[0].stale is False


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


def test_snapshot_has_immutable_ten_node_health_matrix_and_owner():
    owner = type("Owner", (), {
        "pid": 4321,
        "process_name": "chassis_node",
        "lock_path": "/run/powertrain/can0.lock",
        "acquired_at": datetime(2026, 7, 15, tzinfo=timezone.utc),
    })()
    m = ChassisManager(
        _fake_corners(),
        can_owner_snapshot=owner,
    )
    m.connect()
    assert m.arm() is True

    snapshot = m.snapshot()

    assert [(node.can_id, node.physical_wheel) for node in snapshot.ak_nodes] == [
        (1, "front_left"),
        (2, "front_right"),
        (3, "rear_left"),
        (4, "rear_right"),
    ]
    assert [(node.node_id, node.physical_wheel) for node in snapshot.odrive_nodes] == [
        (11, "front_left"),
        (12, "front_right"),
        (13, "mid_left"),
        (14, "mid_right"),
        (15, "rear_left"),
        (16, "rear_right"),
    ]
    assert snapshot.owner.pid == 4321
    assert snapshot.owner.process_name == "chassis_node"
    assert snapshot.owner.lock_path == "/run/powertrain/can0.lock"
    assert snapshot.owner.acquisition_time == "2026-07-15T00:00:00+00:00"
    assert snapshot.bus.rx_packet_delta == 0
    assert snapshot.bus.tx_packet_delta == 0
    assert snapshot.bus.error_warning is False
    assert snapshot.bus.error_passive is False
    assert snapshot.bus.bus_off_delta == 0
    assert snapshot.bus.restart_count == 0
    assert snapshot.interlock.motion_hold_sources == ()
    assert snapshot.interlock.latched_estop_sources == ()
    assert snapshot.interlock.reset_required is False

    with pytest.raises(FrozenInstanceError):
        snapshot.ak_nodes[0].stale = True
    with pytest.raises(TypeError):
        snapshot.ak_nodes[0]["stale"] = True


def test_snapshot_health_matrix_reports_stale_fault_recovery_and_interlock():
    m = _armed_manager()
    front_left = m.corners["front_left"]
    front_left.steer.stale_flag = True
    front_left.steer.fault = 7
    front_left.steer.state = lambda: {
        "actual_deg": 0.0,
        "fault": 7,
        "stale": True,
        "last_feedback_age_ms": 350.0,
        "feedback_rate_hz": 0.0,
        "recovery_count": 2,
    }
    front_left.drive.state = lambda: {
        "actual_vel": 0.0,
        "axis_error": 0x10,
        "axis_state": 1,
        "stale": True,
        "last_heartbeat_age_ms": 250.0,
        "last_encoder_age_ms": 275.0,
        "recovery_count": 3,
    }
    m.set_arm_motion_hold(True, "arm stale")
    m.estop("manual_service", "operator")

    snapshot = m.snapshot()
    ak = snapshot.ak_nodes[0]
    odrive = snapshot.odrive_nodes[0]

    assert ak.last_feedback_age_ms == 350.0
    assert ak.feedback_rate_hz == 0.0
    assert ak.steer_fault == 7
    assert ak.stale is True
    assert ak.recovery_count == 2
    assert odrive.last_heartbeat_age_ms == 250.0
    assert odrive.last_encoder_age_ms == 275.0
    assert odrive.axis_state == 1
    assert odrive.axis_error == 0x10
    assert odrive.stale is True
    assert odrive.recovery_count == 3
    assert snapshot.interlock.motion_hold_sources == ("robot_arm",)
    assert "manual_service" in snapshot.interlock.latched_estop_sources
    assert snapshot.interlock.reset_required is True


def test_snapshot_builds_wheel_consistency_from_one_cached_state_sample():
    cfg = ChassisConfig(wheel_consistency=WheelConsistencyConfig(
        same_side_delta_turns_per_s=0.25,
        yaw_mismatch_rad_s=10.0,
        spin_turns_per_s=10.0,
        stopped_turns_per_s=0.05,
        active_command_turns_per_s=0.5,
        min_response_ratio=0.1,
        max_response_ratio=10.0,
        warn_speed_cap=0.4,
    ))
    m = _armed_manager(cfg)
    for corner in m.corners.values():
        corner.drive._target = 1.0
        corner.drive._actual = 1.0
    m.corners["mid_left"].drive._actual = 0.4
    m.set_imu_yaw_rate(0.0)

    snapshot = m.snapshot()

    assert {warning.code for warning in snapshot.wheel_consistency.warnings} == {
        "same_side_delta",
    }
    assert snapshot.wheel_consistency.terrain_speed_cap == 0.4
    assert snapshot.wheel_consistency.imu_yaw_rate_rad_s == 0.0


def test_snapshot_copies_injected_bus_health_without_io():
    m = _armed_manager()
    health = CanBusHealth(
        rx_packet_delta=120,
        tx_packet_delta=80,
        error_warning=True,
        error_passive=False,
        bus_off_delta=2,
        restart_count=3,
    )

    m.set_can_bus_health(health)

    assert m.snapshot().bus == health


def test_can_bus_sampler_parses_owner_process_ip_stats_as_deltas():
    first = """
    can state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 100
    re-started bus-errors arbit-lost error-warn error-pass bus-off
    2 3 0 4 1 5
    RX: bytes packets errors dropped missed mcast
    1000 200 0 0 0 0
    TX: bytes packets errors dropped carrier collsns
    800 150 0 0 0 0
    """
    second = """
    can state ERROR-PASSIVE (berr-counter tx 140 rx 2) restart-ms 100
    re-started bus-errors arbit-lost error-warn error-pass bus-off
    3 4 0 5 2 7
    RX: bytes packets errors dropped missed mcast
    1200 230 0 0 0 0
    TX: bytes packets errors dropped carrier collsns
    1000 170 0 0 0 0
    """
    sampler = CanBusStatsSampler("can0")

    sampler.update_from_text(first)
    assert sampler.snapshot().rx_packet_delta == 0
    sampler.update_from_text(second)
    health = sampler.snapshot()

    assert health.rx_packet_delta == 30
    assert health.tx_packet_delta == 20
    assert health.error_warning is True
    assert health.error_passive is True
    assert health.bus_off_delta == 2
    assert health.restart_count == 3


def test_can_health_event_is_json_ready_warn_with_speed_cap_only():
    cfg = ChassisConfig(wheel_consistency=WheelConsistencyConfig(
        same_side_delta_turns_per_s=0.25,
        yaw_mismatch_rad_s=10.0,
        spin_turns_per_s=10.0,
        stopped_turns_per_s=0.05,
        active_command_turns_per_s=0.5,
        min_response_ratio=0.1,
        max_response_ratio=10.0,
        warn_speed_cap=0.4,
    ))
    m = _armed_manager(cfg)
    for corner in m.corners.values():
        corner.drive._target = 1.0
        corner.drive._actual = 1.0
    m.corners["mid_left"].drive._actual = 0.4

    event = build_can_health_event(
        m.snapshot(),
        wall_time_ns=10,
        monotonic_ns=20,
    )

    assert event["event_type"] == "CAN_HEALTH"
    assert event["severity"] == "WARN"
    assert event["wall_time_ns"] == 10
    assert event["monotonic_ns"] == 20
    assert len(event["payload"]["ak_nodes"]) == 4
    assert len(event["payload"]["odrive_nodes"]) == 6
    assert event["payload"]["wheel_consistency"]["warnings"][0]["severity"] == "WARN"
    assert event["payload"]["wheel_consistency"]["terrain_speed_cap"] == 0.4
    assert "torque" not in repr(event).lower()


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


# ── 전방 감속 힌트 (obstacle_zones → set_speed_scale) ─────────────────────
#
# 🛑 안전 게이트가 아니다. 게이트는 US-100 + SafetyInterlock(MOTION_HOLD/ESTOP).
#    여기서 검증하는 것은 "감속 힌트가 v 를 올바로 줄이는가" 뿐이다.

def _drive(m, name="front_left"):
    return _drive_targets(m)[name]


def test_speed_scale_reduces_forward_speed():
    m = _armed_manager()
    m.set(0.5, 0.0)
    m.tick()
    full = _drive(m)

    m.set_speed_scale(0.5)
    m.set(0.5, 0.0)
    m.tick()
    assert _drive(m) == pytest.approx(full * 0.5, rel=1e-6)


def test_speed_scale_zero_stops_drive():
    m = _armed_manager()
    m.set_speed_scale(0.0)
    m.set(0.5, 0.0)
    m.tick()
    for name in WHEEL_NAMES:
        assert _drive(m, name) == 0.0


def test_speed_scale_does_not_block_reverse():
    """앞에 장애물이 있다고 **후진을 막으면 안 된다** — 빠져나갈 길을 막는 꼴."""
    m = _armed_manager()
    m.set(-0.5, 0.0)
    m.tick()
    full_rev = _drive(m)

    m.set_speed_scale(0.0)          # 전방 STOP
    m.set(-0.5, 0.0)
    m.tick()
    assert _drive(m) == pytest.approx(full_rev, rel=1e-6)   # 후진은 그대로


def test_speed_scale_keeps_rotation():
    """정지 상태에서도 회전으로 회피할 수 있어야 한다 — ω 는 안 줄인다."""
    m = _armed_manager()
    m.set_speed_scale(0.0)
    m.set(0.0, 0.4)                 # 제자리 회전
    m.tick()
    assert any(abs(_drive(m, n)) > 0.0 for n in WHEEL_NAMES)


def test_speed_scale_clamped():
    m = _armed_manager()
    m.set_speed_scale(5.0)
    assert m._speed_scale == 1.0
    m.set_speed_scale(-1.0)
    assert m._speed_scale == 0.0


def test_speed_scale_does_not_reset_command_watchdog():
    """★ 감속 힌트는 `set()` 이 아닌 별도 채널이어야 한다.

    `set()` 은 `_last_set_ms` 를 갱신해 **명령 워치독**(300 ms)을 리셋한다. 힌트를
    `set()` 으로 밀어넣으면 상위 명령이 끊겨도 워치독이 영영 안 터진다 = stale 명령 재생.
    """
    clock = FakeClock()
    m = _armed_manager(clock=clock)
    m.set(0.5, 0.0)
    m.tick()
    assert _drive(m) != 0.0

    clock.advance(0.4)              # watchdog_ms(300) 초과 — /cmd_vel 끊김
    m.set_speed_scale(0.3)          # 힌트만 계속 온다
    m.tick()
    assert _drive(m) == 0.0         # 워치독이 살아 있어야 한다 (구동 0)


def test_speed_scale_floored_by_cogging_floor():
    """⚠️ **`min_drive_turns_per_s`(코깅존 플로어)가 SLOW 를 무력화할 수 있다.**

    실기 기본값은 `min_rev=1.0` rev/s (저속 HALL 코깅존 회피). 그런데 플로어는 0이 아닌
    모든 명령을 그 값까지 **끌어올린다**. 따라서 감속 힌트가 v 를 줄여도 바퀴 지령이
    플로어 아래로 내려가면 다시 1.0 rev/s 로 튄다 →  **달성 가능한 최저 속도는
    min_rev(≈0.63 m/s)이며 SLOW 는 그 아래로 못 내려간다.**

    STOP(scale=0)은 정상 동작한다 — 플로어는 `0 < |drive|` 일 때만 적용되므로 0은 0이다.
    """
    cfg = ChassisConfig(min_drive_turns_per_s=1.0)
    m = _armed_manager(cfg=cfg)
    circ = 2 * math.pi * cfg.geometry.wheel_radius_m
    v_cap = cfg.geometry.drive_limit_mps                 # 0.80 m/s 상한

    m.set(v_cap, 0.0)                                    # 1.27 rev/s (플로어 위)
    m.tick()
    assert _drive(m) == pytest.approx(v_cap / circ, rel=1e-3)

    m.set_speed_scale(0.3)                               # SLOW → 0.38 rev/s (플로어 아래)
    m.set(v_cap, 0.0)
    m.tick()
    assert _drive(m) == pytest.approx(1.0, rel=1e-6)     # 플로어로 끌어올려짐 (0.38 아님)

    m.set_speed_scale(0.0)                               # STOP 은 정상 동작
    m.set(v_cap, 0.0)
    m.tick()
    assert _drive(m) == 0.0
