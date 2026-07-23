"""Chassis teleop input and direct-CAN diagnostic gate tests."""
import pytest
from chassis import teleop_dualsense, teleop_server
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
from chassis.teleop_server import parse_input_line, make_status_line


def test_status_line_format():
    # 서버→클라 상태회신 — 클라 파서(공백 split, 'S' 접두)와 계약 일치
    assert make_status_line("ARMED", 1.5, -0.72) == "S ARMED +1.50 -0.72\n"
    assert make_status_line("IDLE", 0.0, 0.0) == "S IDLE +0.00 +0.00\n"


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


def test_parse_rejects_astronomical_button_integer():
    huge_button = "9" * 400

    assert parse_input_line(f"0 0 0 {huge_button} 0") is None


def test_wireless_receive_buffer_clears_oversized_unterminated_line():
    limit = teleop_server.WIRELESS_INPUT_BUFFER_MAX_BYTES

    buffered, overflowed = teleop_server.append_wireless_input_data(
        b"x" * limit,
        b"x",
    )

    assert buffered == b""
    assert overflowed is True


def test_wireless_disconnect_disarms_armed_manager_once():
    class Manager:
        mode = "ARMED"

        def __init__(self):
            self.disarm_calls = 0

        def disarm(self):
            self.disarm_calls += 1
            self.mode = "IDLE"

    state = {}
    teleop_server.reset_wireless_input(state)
    manager = Manager()

    teleop_server.mark_wireless_disconnected(state)
    # A fast replacement client can be accepted before the 50 Hz owner drains
    # the lifecycle event. Its input reset must not erase the pending disarm.
    teleop_server.reset_wireless_input(state)
    pending = teleop_server.consume_wireless_disconnect(state)

    assert teleop_server.apply_wireless_disconnect(manager, pending) is True
    assert manager.mode == "IDLE"
    assert manager.disarm_calls == 1
    assert teleop_server.consume_wireless_disconnect(state) is False


@pytest.mark.parametrize("module", (teleop_server, teleop_dualsense))
@pytest.mark.parametrize(
    "motion_input",
    (
        (0.1, 0.0, 0.0),
        (0.0, 0.1, 0.0),
        (0.0, 0.0, 0.1),
    ),
)
def test_square_refuses_idle_arm_with_non_neutral_motion(
    module,
    motion_input,
):
    class Manager:
        mode = "IDLE"

        def __init__(self):
            self.arm_calls = 0

        def arm(self):
            self.arm_calls += 1
            self.mode = "ARMED"
            return True

    manager = Manager()

    assert module.handle_chassis_square(manager, motion_input) is False
    assert manager.mode == "IDLE"
    assert manager.arm_calls == 0


def test_wireless_estop_edge_bypasses_closed_neutral_gate_once():
    class Manager:
        mode = "ARMED"

        def __init__(self):
            self.estop_calls = []

        def estop(self, source, detail):
            self.estop_calls.append((source, detail))
            self.mode = "ESTOP"

    state = {}
    teleop_server.reset_wireless_input(state)
    assert teleop_server.update_wireless_input(
        state,
        (0.0, 0.0, 0.0, 0, 0),
        now_ms=0.0,
    ) is True
    assert teleop_server.update_wireless_input(
        state,
        (0.0, 1.0, 0.0, 0, 0),
        now_ms=10.0,
    ) is True

    # The trigger remains held after an RX gap, so the neutral gate closes.
    assert teleop_server.update_wireless_input(
        state,
        (0.0, 1.0, 0.0, 0, 1),
        now_ms=10.0 + teleop_server.WIRELESS_RX_TIMEOUT_MS + 0.1,
    ) is False

    manager = Manager()
    pending = teleop_server.consume_wireless_estop(state)
    assert teleop_server.apply_wireless_estop(manager, pending) is True
    assert manager.estop_calls == [("manual", "dualsense")]

    # Repeated held-circle samples behind the gate are not new edges.
    assert teleop_server.update_wireless_input(
        state,
        (0.0, 1.0, 0.0, 0, 1),
        now_ms=320.0,
    ) is False
    pending = teleop_server.consume_wireless_estop(state)
    assert teleop_server.apply_wireless_estop(manager, pending) is False
    assert manager.estop_calls == [("manual", "dualsense")]


def test_wireless_estop_edge_suppresses_simultaneous_square():
    assert teleop_server.should_process_wireless_square(
        square=1,
        previous_square=0,
        estop_applied=True,
    ) is False


@pytest.mark.parametrize(
    "line",
    (
        "nan 0 0 0 0",
        "inf 0 0 0 0",
        "-inf 0 0 0 0",
        "0 nan 0 0 0",
        "0 0 inf 0 0",
    ),
)
def test_parse_rejects_nonfinite_numeric_fields(line):
    assert parse_input_line(line) is None


TELEOP_MODULES = (teleop_server, teleop_dualsense)
DIRECT_CAN_NOTICE = (
    "production 원격은 powertrain_control(teleop_command)+authority 경로. "
    "이 도구는 진단 전용"
)


def _parse_args(module, argv, input_fn=None):
    assert hasattr(module, "_parse_args"), (
        f"{module.__name__} must gate direct-CAN arguments before hardware init"
    )
    return module._parse_args(argv, input_fn=input_fn)


@pytest.mark.parametrize("module", TELEOP_MODULES)
def test_direct_can_teleop_requires_explicit_diagnostic_flag(module, capsys):
    with pytest.raises(SystemExit) as excinfo:
        _parse_args(module, [])

    assert excinfo.value.code == 2
    assert DIRECT_CAN_NOTICE in capsys.readouterr().err


@pytest.mark.parametrize("module", TELEOP_MODULES)
def test_direct_can_teleop_accepts_exact_yes_confirmation(module):
    prompts = []

    args = _parse_args(
        module,
        ["--diagnostic-direct-can"],
        input_fn=lambda prompt: prompts.append(prompt) or "yes",
    )

    assert args.diagnostic_direct_can is True
    assert prompts == [
        "로봇팔이 기계적으로 접혀 고정됐음을 확인했는가? "
        "계속하려면 yes 입력: "
    ]


@pytest.mark.parametrize("module", TELEOP_MODULES)
def test_direct_can_teleop_rejects_confirmation_refusal(module, capsys):
    with pytest.raises(SystemExit) as excinfo:
        _parse_args(
            module,
            ["--diagnostic-direct-can"],
            input_fn=lambda _prompt: "no",
        )

    assert excinfo.value.code == 2
    assert "arm-stowed confirmation required" in capsys.readouterr().err


@pytest.mark.parametrize("module", TELEOP_MODULES)
def test_direct_can_teleop_noninteractive_confirmation_bypasses_prompt(module):
    args = _parse_args(
        module,
        ["--diagnostic-direct-can", "--confirm-arm-stowed"],
        input_fn=lambda _prompt: pytest.fail("prompt must be bypassed"),
    )

    assert args.confirm_arm_stowed is True
