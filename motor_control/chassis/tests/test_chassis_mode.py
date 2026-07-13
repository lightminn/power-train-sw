"""차체 모드 — 로봇팔 자세 락 계약.

⚠️ **아무도 이 모드들을 발행하지 않고 있었다.** 팔은 계속 DRIVING 만 받아 코너·험지에서
   자세를 락하지 않았다. 계약은 있었지만 구현이 비어 있었다.
"""
import math

import pytest

from chassis.chassis_mode import (
    LOCK_MODES, MODE_CORNERING, MODE_DRIVING, MODE_FOLLOW_LEAD,
    MODE_MISSION_STOP, MODE_ROUGH_TERRAIN, ChassisModeSelector, ModeConfig,
)


def _s(**kw):
    return ChassisModeSelector(ModeConfig(**kw))


def test_straight_driving_is_unlocked():
    s = _s()
    assert s.update(0.0, omega=0.0) == MODE_DRIVING
    assert MODE_DRIVING not in LOCK_MODES


def test_cornering_locks_the_arm():
    """선회 중에는 원심력으로 팔이 흔들린다 → 자세 락."""
    s = _s(corner_omega=0.25)
    assert s.update(0.0, omega=0.4) == MODE_CORNERING
    assert MODE_CORNERING in LOCK_MODES


def test_rough_terrain_locks_the_arm():
    s = _s(rough_tilt_deg=8.0)
    assert s.update(0.0, pitch=math.radians(12.0)) == MODE_ROUGH_TERRAIN


def test_rough_terrain_beats_cornering():
    """둘 다면 험지가 이긴다 (더 위험하다)."""
    s = _s()
    assert s.update(0.0, omega=0.5, roll=math.radians(15.0)) == MODE_ROUGH_TERRAIN


def test_mission_stop_beats_everything():
    """★ 팔이 작업 중이면 무조건 정지 모드 — 다른 어떤 조건도 이걸 못 이긴다."""
    s = _s()
    m = s.update(0.0, omega=2.0, roll=math.radians(30.0),
                 mission_mode=MODE_MISSION_STOP, follow_lead=True)
    assert m == MODE_MISSION_STOP


def test_follow_lead_beats_motion_modes():
    s = _s()
    assert s.update(0.0, omega=0.5, follow_lead=True) == MODE_FOLLOW_LEAD


# ── 히스테리시스 · 유지시간 ──────────────────────────────────────────────

def test_cornering_hysteresis():
    """★ 경계에서 덜컥거리면 팔이 자세를 풀었다 잠갔다 반복해 오히려 흔들린다."""
    s = _s(corner_omega=0.25, corner_hyst=0.15, hold_s=0.0)
    assert s.update(0.0, omega=0.30) == MODE_CORNERING
    assert s.update(1.0, omega=0.20) == MODE_CORNERING   # 0.25 아래지만 유지
    assert s.update(2.0, omega=0.10) == MODE_DRIVING     # 0.15 아래 → 해제


def test_lock_is_held_for_minimum_time():
    """짧은 선회에도 락이 최소 hold_s 만큼 유지된다."""
    s = _s(corner_omega=0.25, hold_s=0.5)
    s.update(0.0, omega=0.4)                             # 락
    assert s.update(0.2, omega=0.0) == MODE_CORNERING    # 아직 유지
    assert s.update(0.6, omega=0.0) == MODE_DRIVING      # 0.5s 지나면 해제


def test_mission_stop_clears_lock_timer():
    """미션 정차는 즉시 전환된다 — 유지시간을 기다리지 않는다."""
    s = _s(hold_s=5.0)
    s.update(0.0, omega=0.5)                             # CORNERING (락)
    assert s.update(0.1, mission_mode=MODE_MISSION_STOP) == MODE_MISSION_STOP
    assert s.update(0.2, omega=0.0) == MODE_DRIVING      # 미션 끝나면 바로 풀린다


def test_negative_omega_also_corners():
    s = _s()
    assert s.update(0.0, omega=-0.5) == MODE_CORNERING


def test_negative_tilt_also_rough():
    s = _s()
    assert s.update(0.0, roll=math.radians(-12.0)) == MODE_ROUGH_TERRAIN
