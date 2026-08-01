"""scripts/dxl_cli.py 의 하드웨어 없는 부분(변환/키 디코딩/타깃 구성/조그 규칙) 검증."""
import importlib.util
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
CLI_PATH = ROOT / "scripts/dxl_cli.py"
_CLI = None


def cli():
    assert CLI_PATH.exists(), "scripts/dxl_cli.py has not been implemented"
    if _CLI is None:
        spec = importlib.util.spec_from_file_location("dxl_cli_under_test", CLI_PATH)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        globals()["_CLI"] = module
    return _CLI


# ------------------------------------------------------------------ 단위 변환
def test_signed_restores_two_complement():
    assert cli().signed(0xFFFFFFFF, 4) == -1
    assert cli().signed(0xFFF6, 2) == -10
    assert cli().signed(266, 2) == 266


def test_tick_deg_round_trip():
    module = cli()
    assert module.tick_to_deg(module.CENTER_TICK) == 0.0
    assert module.tick_to_deg(module.CENTER_TICK + module.TICKS_PER_REV // 4) == 90.0
    assert module.deg_to_ticks(90.0) == module.TICKS_PER_REV // 4
    assert module.deg_to_ticks(-5.0) == -57


def test_deg_to_ticks_never_rounds_a_step_to_zero():
    """0.01° 처럼 작은 스텝이 0 tick 으로 반올림되면 키를 눌러도 안 움직인다."""
    assert cli().deg_to_ticks(0.01) == 1
    assert cli().deg_to_ticks(-0.01) == -1
    assert cli().deg_to_ticks(0.0) == 0


# ------------------------------------------------------------------ ID 파싱
def test_parse_ids_accepts_lists_ranges_and_dedupes():
    assert cli().parse_ids("21,22,23") == [21, 22, 23]
    assert cli().parse_ids("1-4") == [1, 2, 3, 4]
    assert cli().parse_ids("3,1-3, 15") == [3, 1, 2, 15]


@pytest.mark.parametrize("text", ["", "253", "-1"])
def test_parse_ids_rejects_bad_input(text):
    with pytest.raises(ValueError):
        cli().parse_ids(text)


# ------------------------------------------------------------------ 키 디코딩
def test_decode_keys_maps_arrows_and_chars():
    keys, rest = cli().decode_keys(b"\x1b[C\x1b[Dt[]q")
    assert keys == ["right", "left", "t", "[", "]", "q"]
    assert rest == b""


def test_decode_keys_keeps_partial_escape_sequence():
    """이스케이프가 쪼개져 들어오면 남겨 두고 다음 read 와 이어 붙여야 한다."""
    module = cli()
    keys, rest = module.decode_keys(b"\x1b[")
    assert keys == [] and rest == b"\x1b["
    keys, rest = module.decode_keys(rest + b"A")
    assert keys == ["up"] and rest == b""


def test_decode_keys_maps_ctrl_c():
    assert cli().decode_keys(b"\x03")[0] == ["ctrl-c"]


# ------------------------------------------------------------------ 하드웨어 에러
def test_hardware_error_labels():
    module = cli()
    assert module.hardware_error_labels(0) == []
    assert module.hardware_error_labels(0b100000) == ["과부하"]
    assert module.hardware_error_labels(0b100101) == ["입력전압", "과열", "과부하"]


# ------------------------------------------------------------------ 타깃 구성
def _motors(module, *ids, mode=None):
    mode = module.POSITION_MODE if mode is None else mode
    return [module.Motor(dxl_id=i, model=module.MODEL_XL430_W250, mode=mode) for i in ids]


def test_build_targets_groups_the_gripper_pair():
    module = cli()
    targets = module.build_targets(_motors(module, 3, 4, 21), module.DEFAULT_PAIR)
    assert [t.ids for t in targets] == [[3, 4], [21]]
    assert targets[0].label.startswith("GRIP")


def test_build_targets_leaves_a_half_present_pair_alone():
    """짝 중 하나만 응답하면 묶지 않는다 — 없는 모터에 goal 을 쓰지 않도록."""
    module = cli()
    targets = module.build_targets(_motors(module, 3, 21), module.DEFAULT_PAIR)
    assert [t.ids for t in targets] == [[3], [21]]
    assert [t.label for t in targets] == ["ID   3", "ID  21"]


def test_build_targets_without_pair_keeps_every_motor_separate():
    module = cli()
    targets = module.build_targets(_motors(module, 3, 4), None)
    assert [t.ids for t in targets] == [[3], [4]]


def test_motor_limits_widen_for_extended_position_mode():
    module = cli()
    motor = module.Motor(dxl_id=1, mode=module.EXTENDED_POSITION_MODE, position=9000)
    assert module.motor_limits(motor, (None, None)) == (9000 - 4096, 9000 + 4096)
    motor.mode = module.POSITION_MODE
    assert module.motor_limits(motor, (None, None)) == (0, 4095)
    assert module.motor_limits(motor, (100, 200)) == (100, 200)


def test_target_is_not_joggable_in_velocity_mode():
    module = cli()
    target = module.build_targets(_motors(module, 21, mode=1), None)[0]
    assert not target.joggable


# ------------------------------------------------------------------ 조그 규칙
class FakeBus:
    """실제 서보 대신 goal/torque 쓰기를 기록하는 더블."""

    def __init__(self, position=2048):
        self.position = position
        self.goals = []
        self.torque_writes = []
        self.profile_writes = []
        self.fail_write4 = False

    def read_position(self, dxl_id):
        return self.position

    def read1(self, dxl_id, addr):
        return 30

    def write1(self, dxl_id, addr, value):
        self.torque_writes.append((dxl_id, value))
        return True

    def write4(self, dxl_id, addr, value):
        module = cli()
        if addr == module.ADDR_GOAL_POSITION:
            if self.fail_write4:
                return False
            self.goals.append((dxl_id, value))
        else:
            self.profile_writes.append((dxl_id, addr, value))
        return True

    def sync_read(self, motors):
        return True


class Args:
    port = "/dev/null"
    baud = 1_000_000
    step_deg = 5.0
    fine_deg = 1.0
    min_tick = None
    max_tick = None
    profile_acc = 25
    profile_vel = 80


def _jog(module, bus, *ids, pair=None):
    targets = module.build_targets(_motors(module, *ids), pair)
    return module.Jog(bus, targets, Args()), targets


def test_step_is_ignored_while_torque_is_off():
    module = cli()
    bus = FakeBus()
    jog, _targets = _jog(module, bus, 21)
    jog.handle("right")
    assert bus.goals == []
    assert "토크가 꺼져" in jog.log[-1]


def test_torque_on_syncs_goal_to_present_position_and_writes_profile():
    """낡은 goal 이 남아 있으면 토크가 들어오는 순간 그리로 튄다 → 켜기 전에 현재값을 쓴다."""
    module = cli()
    bus = FakeBus(position=1500)
    jog, targets = _jog(module, bus, 21)
    jog.handle("t")
    assert bus.goals == [(21, 1500)]
    assert targets[0].goal == 1500
    assert targets[0].torque is True
    assert sorted(bus.profile_writes) == [
        (21, module.ADDR_PROFILE_ACCELERATION, 25),
        (21, module.ADDR_PROFILE_VELOCITY, 80),
    ]
    # 토크 ON 은 goal/profile 을 쓴 **뒤에** 나가야 한다.
    assert bus.torque_writes == [(21, module.TORQUE_ON)]


def test_arrow_step_moves_by_step_degrees_and_accumulates():
    module = cli()
    bus = FakeBus(position=2048)
    jog, targets = _jog(module, bus, 21)
    jog.handle("t")
    jog.handle("right")
    jog.handle("right")
    jog.handle("]")
    assert [goal for _id, goal in bus.goals] == [2048, 2105, 2162, 2173]
    assert targets[0].goal == 2173


def test_step_clamps_at_the_limit_and_says_so():
    module = cli()
    bus = FakeBus(position=4090)
    jog, targets = _jog(module, bus, 21)
    jog.handle("t")
    jog.handle("right")
    assert targets[0].goal == 4095
    jog.handle("right")
    assert targets[0].goal == 4095
    assert "범위 끝" in jog.log[-1]


def test_paired_gripper_motors_get_the_same_goal():
    module = cli()
    bus = FakeBus(position=2446)
    jog, targets = _jog(module, bus, 3, 4, pair=(3, 4))
    jog.handle("t")
    jog.handle("right")
    assert bus.goals == [(3, 2446), (4, 2446), (3, 2503), (4, 2503)]
    assert targets[0].goal == 2503


def test_step_is_blocked_while_a_hardware_error_is_latched():
    module = cli()
    bus = FakeBus()
    jog, targets = _jog(module, bus, 21)
    jog.handle("t")
    bus.goals.clear()
    targets[0].motors[0].hw_error = 0b100000  # 과부하
    jog.handle("right")
    assert bus.goals == []
    assert "하드웨어 에러" in jog.log[-1]


def test_emergency_key_drops_torque_on_every_target():
    module = cli()
    bus = FakeBus()
    jog, targets = _jog(module, bus, 21, 22)
    jog.handle("t")
    jog.handle("down")
    jog.handle("t")
    bus.torque_writes.clear()
    jog.handle("e")
    assert sorted(bus.torque_writes) == [(21, module.TORQUE_OFF), (22, module.TORQUE_OFF)]
    assert all(not target.torque for target in targets)
    assert all(target.goal is None for target in targets)


def test_torque_on_refused_for_a_non_position_mode_motor():
    module = cli()
    bus = FakeBus()
    targets = module.build_targets(_motors(module, 21, mode=1), None)
    jog = module.Jog(bus, targets, Args())
    jog.handle("t")
    assert bus.torque_writes == []
    assert "조그할 수 없습니다" in jog.log[-1]


def test_quit_keys_end_the_loop():
    module = cli()
    jog, _targets = _jog(module, FakeBus(), 21)
    assert jog.handle("q") is False
    assert jog.handle("ctrl-c") is False
    assert jog.handle("s") is True


def test_selection_wraps_around():
    module = cli()
    jog, _targets = _jog(module, FakeBus(), 21, 22, 23)
    jog.handle("up")
    assert jog.selected == 2
    jog.handle("down")
    assert jog.selected == 0


def test_resync_key_pulls_goal_back_to_the_present_position():
    module = cli()
    bus = FakeBus(position=2048)
    jog, targets = _jog(module, bus, 21)
    jog.handle("t")
    bus.position = 1900          # 외력으로 밀렸다고 가정
    jog.handle("c")
    assert targets[0].goal == 1900
    assert bus.goals[-1] == (21, 1900)


def test_goal_write_failure_leaves_the_tracked_goal_untouched():
    module = cli()
    bus = FakeBus()
    jog, targets = _jog(module, bus, 21)
    jog.handle("t")
    bus.fail_write4 = True
    jog.handle("right")
    assert targets[0].goal == 2048
    assert "goal 쓰기 실패" in jog.log[-1]


def test_over_temperature_drops_torque_by_itself():
    module = cli()
    bus = FakeBus()
    jog, targets = _jog(module, bus, 21)
    jog.handle("t")
    bus.read1 = lambda dxl_id, addr: module.TEMP_TRIP_C + 1
    jog._poll()
    assert targets[0].torque is False
    assert any("토크를 내렸습니다" in line for line in jog.log)


# ------------------------------------------------------------------ CLI 표면
def test_parser_defaults_match_the_arm_team_bus():
    module = cli()
    args = module.build_parser().parse_args([])
    assert args.port == "/dev/ttyUSB0"
    assert args.baud == 1_000_000
    assert (args.profile_acc, args.profile_vel) == (25, 80)


def test_zero_profile_is_rejected():
    """profile 0 = 명령마다 최고속 → 과전류로 토크가 풀린다(팔팀 HW-8 실증)."""
    assert cli().main(["--profile-acc", "0"]) == 2
    assert cli().main(["--profile-vel", "0"]) == 2


def test_bad_ids_are_rejected_before_touching_the_bus():
    assert cli().main(["--ids", "999"]) == 2
