"""미션 도착 자동 판정 — YOLO 검출 → "여기서 멈춰야 한다".

★ 인식은 로봇팔 팀 단일 소스다. 우리는 결과만 구독한다.
"""
import pytest

from chassis.mission_trigger import MissionTrigger, TriggerConfig, TriggerRule

PICKUP = "ARRIVED_PICKUP"
DROP = "ARRIVED_DROP"


def _t(**kw):
    rules = kw.pop("rules", [
        TriggerRule("box", PICKUP, stop_distance_m=1.0, min_confidence=0.6),
        TriggerRule("dropzone", DROP, stop_distance_m=1.2, min_confidence=0.6),
    ])
    return MissionTrigger(TriggerConfig(rules=rules, **kw))


def _feed(trig, dets, n, t0=0.0, dt=0.1):
    """n 프레임 먹이고 첫 트리거를 돌려준다."""
    for i in range(n):
        r = trig.on_detections(dets, t0 + i * dt)
        if r is not None:
            return r, t0 + i * dt
    return None, t0 + n * dt


# ── 디바운스 ─────────────────────────────────────────────────────────────

def test_single_frame_does_not_trigger():
    """★ YOLO 는 프레임마다 흔들린다. 한 번 잘못 잡았다고 급정거하면 안 된다.

    팔이 엉뚱한 곳에서 작업을 시작하고 대회가 끝난다.
    """
    trig = _t(consecutive=5)
    assert trig.on_detections([("box", 0.9, 0.8)], t=0.0) is None


def test_consecutive_frames_trigger():
    trig = _t(consecutive=5)
    r, _ = _feed(trig, [("box", 0.9, 0.8)], n=5)
    assert r == (PICKUP, "box")


def test_flicker_resets_the_streak():
    """★ 중간에 놓치면 처음부터 — 3프레임 보고 1프레임 놓치고 3프레임 봐도 안 된다."""
    trig = _t(consecutive=5)
    for i in range(3):
        assert trig.on_detections([("box", 0.9, 0.8)], t=i * 0.1) is None
    assert trig.on_detections([], t=0.3) is None            # 놓침
    for i in range(3):
        assert trig.on_detections([("box", 0.9, 0.8)], t=0.4 + i * 0.1) is None
    # 아직 5 연속이 아니다
    assert trig.on_detections([("box", 0.9, 0.8)], t=0.7) is None
    assert trig.on_detections([("box", 0.9, 0.8)], t=0.8) == (PICKUP, "box")


# ── 게이트 ───────────────────────────────────────────────────────────────

def test_far_object_does_not_trigger():
    """★ 5 m 앞에서 멈추면 팔이 못 닿는다."""
    trig = _t(consecutive=3)
    r, _ = _feed(trig, [("box", 0.9, 5.0)], n=10)
    assert r is None


def test_low_confidence_does_not_trigger():
    trig = _t(consecutive=3)
    r, _ = _feed(trig, [("box", 0.3, 0.8)], n=10)
    assert r is None


def test_unknown_class_is_ignored():
    trig = _t(consecutive=3)
    r, _ = _feed(trig, [("사람", 0.99, 0.5)], n=10)
    assert r is None


def test_zero_or_negative_distance_is_ignored():
    """depth 구멍 → 거리 0 이 들어온다. 그걸 '아주 가깝다'로 읽으면 안 된다."""
    trig = _t(consecutive=3)
    r, _ = _feed(trig, [("box", 0.9, 0.0)], n=10)
    assert r is None


def test_nearest_matching_object_wins():
    trig = _t(consecutive=1)
    r = trig.on_detections(
        [("box", 0.9, 0.9), ("box", 0.9, 0.4)], t=0.0)
    assert r == (PICKUP, "box")


def test_different_rules_have_different_distances():
    trig = _t(consecutive=3)
    r, _ = _feed(trig, [("dropzone", 0.9, 1.1)], n=5)   # dropzone 은 1.2 m 까지
    assert r == (DROP, "dropzone")


# ── 쿨다운 ───────────────────────────────────────────────────────────────

def test_finished_mission_does_not_retrigger():
    """★★ 가장 중요 — 작업을 마치고 재출발할 때 그 물체가 **아직 눈앞에 있다.**

    쿨다운이 없으면 즉시 다시 멈춰서 **무한 루프**에 빠진다.
    """
    trig = _t(consecutive=3, cooldown_s=10.0)
    r, t = _feed(trig, [("box", 0.9, 0.8)], n=3)
    assert r == (PICKUP, "box")

    trig.mission_finished("box", t)                      # 팔 작업 완료

    r2, _ = _feed(trig, [("box", 0.9, 0.8)], n=20, t0=t + 0.1)
    assert r2 is None                                    # 쿨다운 중 — 안 멈춘다


def test_cooldown_expires():
    trig = _t(consecutive=3, cooldown_s=5.0)
    r, t = _feed(trig, [("box", 0.9, 0.8)], n=3)
    trig.mission_finished("box", t)

    r2, _ = _feed(trig, [("box", 0.9, 0.8)], n=5, t0=t + 6.0)   # 쿨다운 지남
    assert r2 == (PICKUP, "box")


def test_cooldown_is_per_class():
    """box 쿨다운 중이어도 dropzone 은 트리거돼야 한다."""
    trig = _t(consecutive=3, cooldown_s=10.0)
    r, t = _feed(trig, [("box", 0.9, 0.8)], n=3)
    trig.mission_finished("box", t)

    r2, _ = _feed(trig, [("dropzone", 0.9, 0.9)], n=5, t0=t + 0.1)
    assert r2 == (DROP, "dropzone")
