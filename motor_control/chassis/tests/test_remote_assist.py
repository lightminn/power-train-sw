import math

import pytest

from motor_control.chassis.remote_assist import (
    AssistConfig,
    AssistCorrection,
    compose,
)


def _correction(**overrides):
    values = {
        "stamp_s": 9.8,
        "omega_correction_rad_s": 0.2,
        "speed_cap_m_s": 1.5,
        "confidence": 0.8,
    }
    values.update(overrides)
    return AssistCorrection(**values)


def _compose(**overrides):
    values = {
        "operator_v": 1.0,
        "operator_omega": -0.1,
        "now_s": 10.0,
        "correction": _correction(),
        "bypass_active": False,
        "bypass_stamp_s": 9.8,
        "enabled": True,
        "config": AssistConfig(),
        "profile_max_speed_m_s": 2.0,
    }
    values.update(overrides)
    return compose(**values)


def test_disabled_returns_raw_operator_intent():
    result = _compose(
        operator_v=-1.2,
        operator_omega=0.3,
        enabled=False,
        correction=None,
        bypass_stamp_s=None,
    )

    assert result.v_m_s == -1.2
    assert result.omega_rad_s == 0.3
    assert result.applied is False
    assert result.reasons == ("assist_disabled",)


@pytest.mark.parametrize(
    ("bypass_active", "bypass_stamp_s", "reason"),
    [
        (True, 10.0, "assist_bypass"),
        (True, 9.49, "bypass_unknown"),
        (False, None, "bypass_unknown"),
        (False, 9.49, "bypass_unknown"),
        (False, 10.100001, "bypass_unknown"),
    ],
)
def test_bypass_hold_or_unknown_signal_returns_raw_intent(
    bypass_active,
    bypass_stamp_s,
    reason,
):
    result = _compose(
        operator_v=-1.7,
        operator_omega=0.25,
        bypass_active=bypass_active,
        bypass_stamp_s=bypass_stamp_s,
    )

    assert result.v_m_s == -1.7
    assert result.omega_rad_s == 0.25
    assert result.applied is False
    assert result.reasons == (reason,)


@pytest.mark.parametrize(
    ("correction", "reason"),
    [
        (None, "correction_missing"),
        (_correction(stamp_s=9.49), "correction_stale"),
        (_correction(stamp_s=10.100001), "correction_stale"),
        (_correction(stamp_s=math.nan), "correction_stale"),
        (_correction(omega_correction_rad_s=math.inf), "correction_stale"),
        (_correction(speed_cap_m_s=math.nan), "correction_stale"),
        (_correction(confidence=math.nan), "correction_stale"),
    ],
)
def test_missing_stale_future_or_nonfinite_correction_degrades_speed(
    correction,
    reason,
):
    result = _compose(
        operator_v=-1.8,
        operator_omega=0.35,
        correction=correction,
    )

    assert result.v_m_s == pytest.approx(-1.2)
    assert result.omega_rad_s == 0.35
    assert result.applied is False
    assert result.reasons == (reason,)


def test_low_confidence_removes_correction_and_degrades_speed():
    result = _compose(
        operator_v=1.8,
        operator_omega=-0.3,
        correction=_correction(confidence=0.249),
    )

    assert result.v_m_s == pytest.approx(1.2)
    assert result.omega_rad_s == -0.3
    assert result.applied is False
    assert result.reasons == ("low_confidence",)


def test_neutral_operator_intent_returns_exact_raw_zero_before_correction():
    result = _compose(
        operator_v=0.02,
        operator_omega=-0.05,
        correction=_correction(omega_correction_rad_s=0.4),
    )

    assert (result.v_m_s, result.omega_rad_s) == (0.02, -0.05)
    assert result.applied is False
    assert result.reasons == ("operator_neutral",)


def test_moving_operator_intent_still_composes_fresh_correction():
    result = _compose(
        operator_v=0.3,
        operator_omega=0.0,
        correction=_correction(omega_correction_rad_s=0.2),
    )

    assert (result.v_m_s, result.omega_rad_s) == pytest.approx((0.3, 0.2))
    assert result.applied is True


@pytest.mark.parametrize(
    ("operator_v", "correction_omega", "speed_cap", "expected_v", "expected_omega"),
    [
        (1.8, 0.9, 1.5, 1.5, 0.3),
        (-1.8, -0.9, 1.5, -1.5, -0.5),
        (0.5, 0.2, math.inf, 0.5, 0.1),
        (1.0, 0.2, 0.0, 0.0, 0.1),
    ],
)
def test_normal_compose_clamps_correction_and_never_increases_speed(
    operator_v,
    correction_omega,
    speed_cap,
    expected_v,
    expected_omega,
):
    result = _compose(
        operator_v=operator_v,
        correction=_correction(
            omega_correction_rad_s=correction_omega,
            speed_cap_m_s=speed_cap,
        ),
    )

    assert result.v_m_s == pytest.approx(expected_v)
    assert result.omega_rad_s == pytest.approx(expected_omega)
    assert abs(result.v_m_s) <= abs(operator_v)
    if result.v_m_s:
        assert math.copysign(1.0, result.v_m_s) == math.copysign(
            1.0,
            operator_v,
        )
    assert result.applied is True
    assert result.reasons == ()
    assert math.isfinite(result.v_m_s)
    assert math.isfinite(result.omega_rad_s)


@pytest.mark.parametrize(
    ("operator_v", "operator_omega"),
    [
        (math.nan, 0.0),
        (math.inf, 0.0),
        (0.0, -math.inf),
    ],
)
def test_nonfinite_operator_input_is_rejected(operator_v, operator_omega):
    with pytest.raises(ValueError, match="operator"):
        _compose(operator_v=operator_v, operator_omega=operator_omega)


def test_compose_is_deterministic_for_identical_value_inputs():
    first = _compose()
    second = _compose(
        correction=AssistCorrection(
            stamp_s=9.8,
            omega_correction_rad_s=0.2,
            speed_cap_m_s=1.5,
            confidence=0.8,
        )
    )

    assert first == second
