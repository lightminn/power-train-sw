"""`/arm/teleop_cmd` 화이트리스트.

`teleop_core.on_cmd` 는 모르는 명령을 경고만 찍고 넘긴다 — 오타가 조용히 삼켜지는
게 제어 경로에서 가장 나쁜 실패라, GUI 쪽에서 먼저 거절하는 것을 여기서 고정한다.
"""

from robot_arm_gui.teleop_vocab import validate


def test_no_arg_commands_pass():
    for cmd in ('stop', 'resume', 'home', 'poses', 'freedrive',
                'freedrive_cancel', 'calib_start', 'calib_mark', 'calib_cancel'):
        assert validate(cmd) == (cmd, None)


def test_action_is_lowercased_but_name_is_not():
    """teleop_core 와 같은 정규화 규칙 — 자세 이름의 대소문자는 보존한다."""
    assert validate('STOP') == ('stop', None)
    assert validate('goto Pick_Ready') == ('goto Pick_Ready', None)


def test_no_arg_command_rejects_an_argument():
    cmd, reason = validate('stop now')
    assert cmd is None and '인자를' in reason


def test_delete_is_a_name_command():
    """자세 삭제(2026-08-19 추가) — teleop_core._cmd_delete_pose 와 짝이다."""
    assert validate('delete bench_test') == ('delete bench_test', None)
    cmd, reason = validate('delete')
    assert cmd is None and reason


def test_delete_rejects_names_it_would_not_let_you_create():
    """만들 수 없는 이름은 지울 수도 없다 — 같은 검증기를 타기 때문이다.

    검증이 생기기 전에 저장된 항목(붙여넣기 사고로 제어문자가 섞인 이름)이 실제로
    남아 있었고, GUI 로는 지울 수 없어 poses_file 을 직접 고쳐야 했다. 이 제약은
    의도된 것이다(명령 채널이 공백 구분이라 화이트리스트를 풀 수 없다) —
    프론트엔드는 그런 이름의 삭제 버튼을 비활성으로 그린다.
    """
    cmd, reason = validate('delete rt:=/dev/ttyusb0')
    assert cmd is None and reason


def test_name_commands_require_a_name():
    for action in ('save', 'goto', 'delete', 'reboot'):
        cmd, reason = validate(action)
        assert cmd is None and '이름이 필요' in reason


def test_name_with_comma_is_rejected():
    """쉼표는 /arm/teleop_poses 의 구분자다 — 이름에 들어가면 목록이 깨진다."""
    cmd, reason = validate('save a,b')
    assert cmd is None and '쓸 수 없는 문자' in reason


def test_name_with_space_is_rejected():
    cmd, reason = validate('save my pose')
    assert cmd is None


def test_reboot_all_is_allowed():
    assert validate('reboot all') == ('reboot all', None)


def test_threshold_commands_accept_on_off_and_integers():
    assert validate('spike on') == ('spike on', None)
    assert validate('trip off') == ('trip off', None)
    assert validate('spike 350') == ('spike 350', None)


def test_threshold_rejects_garbage():
    cmd, reason = validate('spike maybe')
    assert cmd is None and 'on|off' in reason


def test_limit_accepts_only_on_off():
    assert validate('limit off') == ('limit off', None)
    cmd, _ = validate('limit 5')
    assert cmd is None


def test_unknown_command_is_rejected():
    cmd, reason = validate('selfdestruct')
    assert cmd is None and '알 수 없는 명령' in reason


def test_empty_and_non_string_are_rejected():
    assert validate('')[0] is None
    assert validate('   ')[0] is None
    assert validate(None)[0] is None
    assert validate(42)[0] is None


def test_absurdly_long_command_is_rejected():
    cmd, reason = validate('goto ' + 'x' * 500)
    assert cmd is None
