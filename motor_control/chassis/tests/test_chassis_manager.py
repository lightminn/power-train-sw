"""ChassisManager(WP3) 통합 검증 — 코너 6개를 하나의 차체로 묶어
kinematics 결과를 각 코너에 분배하고, estop 전파·US-100 게이팅·워치독을 총괄한다.

전부 fake 드라이버(무하드웨어). 실행:
  motor_control/ 에서  `python -m pytest chassis/tests/test_chassis_manager.py -q`
"""
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


def _armed_manager(cfg=None, monitor=None, clock=None):
    m = ChassisManager(_fake_corners(cfg and cfg.corner), cfg=cfg,
                       monitor=monitor, clock=clock)
    m.connect()
    m.arm()
    return m


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def advance(self, sec):
        self.t += sec


class _Verdict:
    def __init__(self, level):
        self.level = level
        self.distance_mm = None


class _Monitor:
    """US-100 SafetyMonitor 스텁 — 고정 판정을 돌려준다."""
    def __init__(self, level):
        self._level = level
        self.ticks = 0

    def tick(self):
        self.ticks += 1

    def verdict(self):
        return _Verdict(self._level)


# ── 매핑·라이프사이클 ────────────────────────────────────────────────────

def test_requires_every_geometry_wheel_mapped():
    corners = _fake_corners()
    del corners["mid_left"]                       # 한 바퀴 누락
    with pytest.raises(ValueError):
        ChassisManager(corners)


def test_lifecycle_modes():
    m = ChassisManager(_fake_corners())
    assert m.mode == "DISCONNECTED"
    m.connect()
    assert m.mode == "IDLE"
    m.arm()
    assert m.mode == "ARMED"
    m.disarm()
    assert m.mode == "IDLE"
    m.close()
    assert m.mode == "DISCONNECTED"


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


# ── US-100 게이팅 (stop → 구동 0, 조향 유지) ──────────────────────────────

def test_us100_stop_zeros_drive_keeps_steer():
    mon = _Monitor("stop")
    m = _armed_manager(monitor=mon)
    m.set(0.4, 0.4)
    m.tick()
    assert mon.ticks > 0
    for c in m.corners.values():                  # 구동 전부 0
        assert c.state()["drive"]["target_vel"] == 0.0
    # 조향은 여전히 명령됨 (선회각 유지)
    assert m.corners["front_left"].state()["steer"]["target_deg"] > 0
    assert m.mode == "ARMED"                       # 정지지 fault 아님


def test_us100_safe_allows_drive():
    m = _armed_manager(monitor=_Monitor("safe"))
    m.set(0.4, 0.0)
    m.tick()
    assert m.corners["front_left"].state()["drive"]["target_vel"] != 0.0


# ── 워치독 (chassis.set 끊기면 구동 0) ───────────────────────────────────

def test_watchdog_zeros_drive_on_timeout():
    clk = FakeClock()
    cfg = ChassisConfig(watchdog_ms=300.0)
    m = _armed_manager(cfg=cfg, clock=clk)
    m.set(0.4, 0.0)
    clk.advance(0.1)                               # 100ms < 300ms
    m.tick()
    assert m.corners["front_left"].state()["drive"]["target_vel"] != 0.0
    clk.advance(0.5)                               # 총 600ms > 300ms, set 재호출 없음
    m.tick()
    for c in m.corners.values():
        assert c.state()["drive"]["target_vel"] == 0.0


# ── estop 전파 (1곳 트립 → 4코너 전부 정지) ──────────────────────────────

def test_corner_fault_propagates_to_all():
    m = _armed_manager()
    m.set(0.4, 0.4)
    m.tick()
    m.corners["front_left"].steer.fault = 5        # 조향 fault 주입
    m.tick()
    assert m.mode == "FAULT"
    for c in m.corners.values():
        assert c.mode == "FAULT"
        assert c.state()["drive"]["target_vel"] == 0.0


def test_estop_stops_all_corners():
    m = _armed_manager()
    m.set(0.4, 0.4)
    m.tick()
    m.estop()
    assert m.mode == "FAULT"
    for c in m.corners.values():
        assert c.state()["drive"]["target_vel"] == 0.0


def test_state_schema():
    m = _armed_manager()
    st = m.state()
    assert set(st.keys()) >= {"mode", "v", "omega", "verdict", "corners"}
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
