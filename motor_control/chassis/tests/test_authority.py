"""명령 권한 — 단일 작성자 강제 · zero-confirmed handover.

🛑 안전 게이트가 아니다(그건 SafetyInterlock + US-100). 여기는 "누구 말을 들을지"만 정한다.
"""
import pytest

from chassis.authority import (
    AUTO, AUTO_SOURCE, IDLE, MANUAL, MANUAL_SOURCE,
    AuthorityConfig, CommandAuthority,
)


def _a(**kw):
    return CommandAuthority(AuthorityConfig(**kw))


# ── 기본 ─────────────────────────────────────────────────────────────────

def test_idle_forwards_nothing():
    """기본은 IDLE — 아무도 조종하지 않는다. 켜는 건 명시적 행위여야 한다."""
    a = _a()
    a.submit(MANUAL_SOURCE, 1.0, 0.5, t=0.0)
    assert a.select(0.0).ok is False


def test_bad_mode_rejected():
    a = _a()
    assert a.set_mode("TURBO") is False
    assert a.mode == IDLE


# ── zero-confirmed handover ──────────────────────────────────────────────

def test_mode_switch_requires_neutral_first():
    """★ 전환 순간이 가장 위험하다.

    AUTO 로 바꾸는데 레인 추종이 **1초 전에 보낸 전속 명령**이 살아 있으면 로봇이 즉시
    튀어나간다. 새 소스가 중립을 한 번 보내기 전까지 아무것도 전달하지 않는다.
    """
    a = _a()
    a.submit(AUTO_SOURCE, 0.8, 0.6, t=0.0)      # 자율이 전속으로 달리고 있었다
    a.set_mode(AUTO)

    c = a.select(0.0)
    assert c.ok is False                        # ★ 안 나간다
    assert "중립 대기" in c.reason

    a.submit(AUTO_SOURCE, 0.0, 0.0, t=0.1)      # 자율이 중립을 한 번 보냄
    assert a.select(0.1).ok is True             # 권한 인계

    a.submit(AUTO_SOURCE, 0.5, 0.2, t=0.2)      # 이제 통과
    c = a.select(0.2)
    assert (c.v, c.omega, c.ok) == (0.5, 0.2, True)


def test_neutral_confirm_is_required_again_on_every_switch():
    """AUTO → MANUAL → AUTO 로 돌아와도 **다시** 중립을 확인해야 한다."""
    a = _a()
    a.submit(AUTO_SOURCE, 0.0, 0.0, t=0.0)
    a.set_mode(AUTO)
    assert a.select(0.0).ok is True

    a.set_mode(MANUAL)                          # 전환
    a.set_mode(AUTO)                            # 되돌림
    a.submit(AUTO_SOURCE, 0.7, 0.0, t=0.1)      # 중립 없이 바로 전속
    assert a.select(0.1).ok is False            # ★ 다시 대기


def test_neutral_threshold():
    a = _a(neutral_v=0.02, neutral_omega=0.05)
    a.set_mode(MANUAL)
    a.submit(MANUAL_SOURCE, 0.015, 0.04, t=0.0)   # 임계값 안 → 중립으로 인정
    assert a.select(0.0).ok is True


# ── 소스 격리 ────────────────────────────────────────────────────────────

def test_only_the_active_source_is_forwarded():
    """★ 단일 작성자 — MANUAL 모드에서 자율이 아무리 떠들어도 무시한다."""
    a = _a()
    a.submit(MANUAL_SOURCE, 0.0, 0.0, t=0.0)
    a.set_mode(MANUAL)
    a.select(0.0)                                # 중립 확인

    a.submit(MANUAL_SOURCE, 0.3, 0.0, t=0.1)
    a.submit(AUTO_SOURCE, -0.9, 1.2, t=0.1)      # 자율이 반대로 명령
    c = a.select(0.1)
    assert (c.v, c.omega) == (0.3, 0.0)          # 텔레옵만 나간다


def test_auto_mode_ignores_teleop():
    a = _a()
    a.submit(AUTO_SOURCE, 0.0, 0.0, t=0.0)
    a.set_mode(AUTO)
    a.select(0.0)

    a.submit(AUTO_SOURCE, 0.4, 0.1, t=0.1)
    a.submit(MANUAL_SOURCE, 1.5, -1.0, t=0.1)
    c = a.select(0.1)
    assert (c.v, c.omega) == (0.4, 0.1)


# ── stale ────────────────────────────────────────────────────────────────

def test_stale_source_is_not_replayed():
    """★ 오래된 명령을 재생하면 안 된다 — 소스가 죽었는데 로봇이 계속 달린다."""
    a = _a(stale_s=0.3)
    a.submit(AUTO_SOURCE, 0.0, 0.0, t=0.0)
    a.set_mode(AUTO)
    a.select(0.0)

    a.submit(AUTO_SOURCE, 0.5, 0.0, t=1.0)
    assert a.select(1.1).ok is True               # 신선함
    c = a.select(1.5)                             # 0.5 초 조용 → stale
    assert c.ok is False
    assert "stale" in c.reason


def test_missing_source_is_not_ok():
    a = _a()
    a.set_mode(AUTO)
    c = a.select(0.0)
    assert c.ok is False
    assert "명령 없음" in c.reason
