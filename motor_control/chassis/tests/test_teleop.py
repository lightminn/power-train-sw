"""chassis.teleop_dualsense.map_chassis_input 순수 함수 단위 테스트 (무하드웨어)."""
import pytest
from chassis.teleop_dualsense import map_chassis_input


def test_neutral_is_zero():
    v, w = map_chassis_input(left_x=0.0, rt=0.0, lt=0.0)
    assert v == 0.0
    assert w == 0.0


def test_full_forward():
    v, w = map_chassis_input(left_x=0.0, rt=1.0, lt=0.0, v_max=0.6)
    assert v == pytest.approx(0.6)
    assert w == 0.0


def test_full_reverse():
    v, w = map_chassis_input(left_x=0.0, rt=0.0, lt=1.0, v_max=0.6)
    assert v == pytest.approx(-0.6)


def test_stick_right_turns_right_negative_omega():
    # REP-103: ω>0=좌회전 → 스틱 오른쪽(+x)=우회전=ω<0
    v, w = map_chassis_input(left_x=1.0, rt=0.0, lt=0.0, omega_max=1.2)
    assert w == pytest.approx(-1.2)
    assert v == 0.0


def test_stick_left_turns_left_positive_omega():
    v, w = map_chassis_input(left_x=-1.0, rt=0.0, lt=0.0, omega_max=1.2)
    assert w == pytest.approx(1.2)


def test_pivot_no_trigger_plus_stick():
    # 트리거 0(v=0) + 스틱 → 제자리 회전
    v, w = map_chassis_input(left_x=0.8, rt=0.0, lt=0.0, omega_max=1.0)
    assert v == 0.0
    assert w == pytest.approx(-0.8)


def test_deadzone_suppresses_small_inputs():
    v, w = map_chassis_input(left_x=0.03, rt=0.02, lt=0.0, deadzone=0.05)
    assert v == 0.0
    assert w == 0.0


def test_forward_left_turn_combined():
    v, w = map_chassis_input(left_x=-0.5, rt=1.0, lt=0.0, v_max=0.6, omega_max=1.2)
    assert v == pytest.approx(0.6)
    assert w == pytest.approx(0.6)     # -(-0.5)*1.2


# ── 무선 서버 입력 파싱 (chassis.teleop_server.parse_input_line) ───────────
from chassis.teleop_server import parse_input_line


def test_parse_valid_line():
    assert parse_input_line("-0.5 1.0 0.0 1 0") == (-0.5, 1.0, 0.0, 1, 0)


def test_parse_clamps_ranges():
    lx, rt, lt, sq, ci = parse_input_line("2.0 1.5 -0.3 0 0")
    assert lx == 1.0 and rt == 1.0 and lt == 0.0     # 클램프 [-1,1]/[0,1]


def test_parse_button_coercion():
    # 0 아닌 버튼값은 1 로
    assert parse_input_line("0 0 0 3 5")[3:] == (1, 1)


def test_parse_bad_returns_none():
    assert parse_input_line("0 0 0") is None          # 필드 부족
    assert parse_input_line("a b c d e") is None       # 숫자 아님
    assert parse_input_line("") is None
