"""브릿지의 캘리브 파라미터 — 파서와 런타임 반영 콜백.

이 테스트가 없으면 놓치는 것: `set_parameters` 는 값을 바꾸고 **성공을 돌려주지만**,
콜백이 없으면 브릿지는 기동 시 파싱해 둔 dict 를 계속 쓴다. 즉 캘리브 마법사의
"즉시 적용" 이 성공으로 보이면서 실제 변환식은 그대로인 상태가 된다.

하드웨어는 필요 없다 — 파서는 순수 함수고, 콜백은 self 의 몇 개 필드만 만진다.
"""

from types import SimpleNamespace

from dynamixel_control.moveit_dynamixel_bridge import (
    EMPTY_STR_ARRAY, JOINT_CONFIG, MoveItDynamixelBridge, _parse_centers,
    _parse_gear_ratios,
)

import pytest

ARM_JOINT = next(iter(JOINT_CONFIG))


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, text):
        self.messages.append(('info', text))

    def warn(self, text):
        self.messages.append(('warn', text))


def make_bridge(read_only=True):
    """콜백이 실제로 만지는 필드만 가진 가짜 self."""
    logger = _Logger()
    return SimpleNamespace(
        centers={}, gear_ratios={},
        gripper_open_tick=1083, gripper_close_tick=-401,
        read_only=read_only, get_logger=lambda: logger, _logger=logger,
    )


def param(name, value):
    return SimpleNamespace(name=name, value=value)


def call(bridge, params):
    return MoveItDynamixelBridge._on_set_parameters(bridge, params)


# ------------------------------------------------------------ 파서
def test_empty_default_is_not_an_error():
    """기본값은 [""] 이다 — 그걸 오류로 읽으면 브릿지가 매번 경고를 뿜는다."""
    assert _parse_centers(['']) == ({}, [])
    assert _parse_gear_ratios(['']) == ({}, [])
    assert _parse_centers([]) == ({}, [])


def test_empty_default_must_stay_a_string_array():
    """⚠️ `[]` 로 되돌리면 rclpy 가 BYTE_ARRAY 로 추론해 **런타임 설정이 거절된다.**

    CLI `-p centers:=...` 는 선언 시점에 값을 덮어써서 멀쩡히 동작하므로, 이 회귀는
    "즉시 적용" 을 실제로 눌러 보기 전까지 드러나지 않는다(2026-08-12 실기에서 그렇게
    발견했다). 값이 아니라 **타입 추론**을 지키는 테스트다.
    """
    assert EMPTY_STR_ARRAY, '빈 리스트면 타입 추론이 BYTE_ARRAY 로 떨어진다'
    assert all(isinstance(v, str) for v in EMPTY_STR_ARRAY)


def test_parsers_read_the_pair_format():
    values, errors = _parse_centers([f'{ARM_JOINT}:1627'])
    assert values == {ARM_JOINT: 1627} and errors == []
    values, errors = _parse_gear_ratios([f'{ARM_JOINT}:9.034'])
    assert values == {ARM_JOINT: 9.034} and errors == []


def test_parsers_reject_unknown_joints_and_bad_numbers():
    assert _parse_centers(['nope:1627'])[1]
    assert _parse_centers([f'{ARM_JOINT}:abc'])[1]
    assert _parse_gear_ratios([f'{ARM_JOINT}:0'])[1]        # 양수여야 한다
    assert _parse_gear_ratios([f'{ARM_JOINT}:-3'])[1]


def test_center_out_of_tick_range_is_rejected():
    """단일회전 축에 5000 tick 을 넣으면 그 축은 영원히 clamp 된 채로 돈다."""
    single = [n for n, c in JOINT_CONFIG.items() if not c['extended']]
    assert single, 'JOINT_CONFIG 에 단일회전 축이 없다 — 테스트 전제를 확인할 것'
    assert _parse_centers([f'{single[0]}:5000'])[1]


# ------------------------------------------------------------ 런타임 반영
def test_callback_actually_updates_the_runtime_dicts():
    bridge = make_bridge()
    result = call(bridge, [param('centers', [f'{ARM_JOINT}:1627'])])
    assert result.successful
    assert bridge.centers == {ARM_JOINT: 1627}


def test_callback_updates_gear_ratios_and_gripper_ticks():
    bridge = make_bridge()
    assert call(bridge, [param('gear_ratios', [f'{ARM_JOINT}:9.034'])]).successful
    assert bridge.gear_ratios == {ARM_JOINT: 9.034}

    assert call(bridge, [param('gripper_open_tick', 1063),
                         param('gripper_close_tick', -381)]).successful
    assert (bridge.gripper_close_tick, bridge.gripper_open_tick) == (-381, 1063)


def test_callback_rejects_with_a_reason_instead_of_silently_ignoring():
    """조용히 무시하면 '적용했는데 왜 그대로지?' 가 된다."""
    bridge = make_bridge()
    result = call(bridge, [param('centers', ['nope:10'])])
    assert not result.successful
    assert 'nope' in result.reason
    assert bridge.centers == {}


def test_gripper_endpoints_must_stay_apart():
    """개폐 tick 이 붙으면 rad→tick 변환이 무의미해진다."""
    bridge = make_bridge()
    result = call(bridge, [param('gripper_open_tick', 100),
                           param('gripper_close_tick', 90)])
    assert not result.successful
    assert bridge.gripper_open_tick == 1083


def test_gripper_pair_is_validated_together():
    """원자적 설정이라 둘이 한 번에 온다 — 한쪽만 보고 판단하면 안 된다."""
    bridge = make_bridge()
    # close 만 바꾸면 현재 open(1083)과의 간격으로 판정한다.
    assert call(bridge, [param('gripper_close_tick', 1080)]).successful is False
    assert call(bridge, [param('gripper_close_tick', -381)]).successful


def test_unrelated_parameters_pass_through():
    """기동 시에만 읽는 파라미터까지 여기서 거절하면 안 된다."""
    bridge = make_bridge()
    assert call(bridge, [param('read_only', True)]).successful


def test_changing_calibration_with_torque_on_is_warned():
    """토크가 살아 있으면 다음 명령부터 팔이 다른 위치를 목표로 삼는다."""
    bridge = make_bridge(read_only=False)
    call(bridge, [param('centers', [f'{ARM_JOINT}:1627'])])
    assert any(level == 'warn' for level, _ in bridge._logger.messages)


@pytest.mark.parametrize('value', [1627, 1627.4])
def test_center_accepts_float_strings_and_rounds(value):
    values, errors = _parse_centers([f'{ARM_JOINT}:{value}'])
    assert errors == []
    assert values[ARM_JOINT] == 1627
