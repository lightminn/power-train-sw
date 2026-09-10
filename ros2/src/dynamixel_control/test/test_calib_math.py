"""calib_math — 캘리브 측정식. 하드웨어·ROS 없이 도는 검증이다.

여기 숫자는 대부분 2026-08-07 실측값(`JOINT_CONFIG` 의 center/gear_ratio,
`gripper_presets` 의 끝단 tick)이다 — 식을 바꿨을 때 그 실측 결과가 그대로
재현되는지가 이 파일의 목적이다.
"""

import math

import pytest

from dynamixel_control import calib_math as cm


# ------------------------------------------------------------ 왕복
def test_rad_tick_roundtrip():
    for rad in (-1.2, 0.0, 0.35, 2.0):
        tick = cm.rad_to_tick(1627, 1, 9.034, rad)
        assert cm.tick_to_rad(1627, 1, 9.034, tick) == pytest.approx(rad)


def test_direction_flips_the_tick_side():
    plus = cm.rad_to_tick(2048, 1, 1.0, 0.5)
    minus = cm.rad_to_tick(2048, -1, 1.0, 0.5)
    assert plus > 2048 > minus


# ------------------------------------------------------------ 영점
def test_center_is_unchanged_when_measurement_matches_reference():
    """기준 자세에서 이미 기준각을 가리키면 영점은 그대로여야 한다."""
    assert cm.center_from_measurement(1627, 1, 9.034,
                                      rad_measured=0.0, rad_ref=0.0) == 1627


def test_center_absorbs_the_measured_offset():
    """기준 자세에서 0.1 rad 로 읽히면 그만큼이 새 center 로 흡수된다."""
    center = cm.center_from_measurement(2048, 1, 1.0, rad_measured=0.1, rad_ref=0.0)
    assert center == pytest.approx(2048 + 0.1 * cm.TICKS_PER_RAD)
    # 새 center 를 쓰면 같은 tick 이 0 rad 로 읽힌다.
    tick_now = cm.rad_to_tick(2048, 1, 1.0, 0.1)
    assert cm.tick_to_rad(center, 1, 1.0, tick_now) == pytest.approx(0.0, abs=1e-9)


def test_center_honours_a_non_zero_reference_angle():
    """구조상 0 으로 못 세우는 축은 기준각을 넣어서 잰다."""
    center = cm.center_from_measurement(2048, 1, 1.0,
                                        rad_measured=1.5708, rad_ref=1.5708)
    assert center == pytest.approx(2048)


def test_gear_ratio_scales_the_center_shift():
    """감속기가 있으면 같은 관절각 오차가 더 많은 tick 으로 나타난다."""
    plain = cm.center_from_measurement(2048, 1, 1.0, rad_measured=0.1)
    geared = cm.center_from_measurement(2048, 1, 9.034, rad_measured=0.1)
    assert (geared - 2048) == pytest.approx((plain - 2048) * 9.034)


def test_center_shift_deg_is_the_inverse_of_the_shift():
    shift = cm.center_from_measurement(2048, 1, 9.034, rad_measured=0.1) - 2048
    assert cm.center_shift_deg(shift, 9.034) == pytest.approx(math.degrees(0.1))


def test_out_of_range_center_is_reported_for_single_turn_joints():
    assert cm.center_out_of_range(5000, extended=False) is not None
    assert cm.center_out_of_range(-1, extended=False) is not None
    assert cm.center_out_of_range(2048, extended=False) is None


def test_multi_turn_joints_allow_negative_centers():
    """다회전 축은 음수 tick 이 정상이다 — 단일회전 기준으로 거절하면 안 된다."""
    assert cm.center_out_of_range(-5000, extended=True) is None
    assert cm.center_out_of_range(-5000, extended=False) is not None


# ------------------------------------------------------------ 기어비
def test_gear_ratio_from_a_quarter_turn():
    """관절 90° 를 돌렸는데 서보축이 9.034 배 돌았다면 그게 기어비다."""
    servo = math.radians(90.0) * 9.034
    ratio, inverted = cm.gear_ratio_from_span(servo, 90.0)
    assert ratio == pytest.approx(9.034)
    assert inverted is False


def test_gear_ratio_reports_inversion_but_keeps_the_magnitude():
    ratio, inverted = cm.gear_ratio_from_span(-math.radians(90.0) * 4.04, 90.0)
    assert ratio == pytest.approx(4.04)
    assert inverted is True


def test_gear_ratio_refuses_a_servo_that_barely_moved():
    """'안 움직였는데 잰 것으로 처리' 가 가장 나쁜 실패 모드다."""
    with pytest.raises(ValueError):
        cm.gear_ratio_from_span(0.01, 90.0)


def test_gear_ratio_refuses_a_zero_joint_angle():
    with pytest.raises(ValueError):
        cm.gear_ratio_from_span(1.0, 0.0)


# ------------------------------------------------------------ 그리퍼 끝단
def test_gripper_endpoints_pass_through_without_margin():
    """2026-08-07 실측값이 그대로 나와야 한다."""
    out = cm.gripper_endpoints(closed_tick=-401, opened_tick=1083, margin=0)
    assert (out['close'], out['open']) == (-401, 1083)
    assert out['stroke_tick'] == 1484


def test_gripper_margin_goes_inward_when_open_is_larger():
    out = cm.gripper_endpoints(closed_tick=-401, opened_tick=1083, margin=20)
    assert (out['close'], out['open']) == (-381, 1063)


def test_gripper_margin_goes_inward_when_the_sign_is_flipped():
    """개폐 방향은 조립에 따라 뒤집힌다(옛 값은 open<close 였다) — 마진 부호도 따라가야 한다."""
    out = cm.gripper_endpoints(closed_tick=3186, opened_tick=2446, margin=20)
    assert (out['close'], out['open']) == (3166, 2466)


def test_gripper_warns_about_negative_ticks_needing_extended_mode():
    out = cm.gripper_endpoints(closed_tick=-401, opened_tick=1083)
    assert any('extended' in w for w in out['warnings'])


def test_gripper_warns_when_the_stroke_approaches_a_full_turn():
    out = cm.gripper_endpoints(closed_tick=0, opened_tick=3600)
    assert any('한 바퀴' in w for w in out['warnings'])


def test_gripper_refuses_when_it_was_barely_moved():
    with pytest.raises(ValueError):
        cm.gripper_endpoints(closed_tick=1000, opened_tick=1030)


# ------------------------------------------------------------ 복사 블록
def test_joint_config_entry_is_pasteable():
    text = cm.format_joint_config_entry('arm_joint_2', 14, 1626.7, 1, 9.034, False)
    assert '"arm_joint_2": {"id": 14, "center": 1627, "direction": 1,' in text
    assert '"gear_ratio": 9.034, "extended": False},' in text
