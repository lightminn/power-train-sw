"""`/dynamixel/hardware_error` 파서 회귀 테스트 (ROS·하드웨어 불필요)."""

from robot_arm_gui.hw_error_parse import latched_ids, parse_hardware_error, summarize


def test_empty_is_no_error():
    # 노드는 에러가 없어도 매 read 주기(30Hz) 빈 문자열을 발행한다.
    assert parse_hardware_error('') == []
    assert parse_hardware_error('   ') == []
    assert parse_hardware_error(None) == []


def test_single_entry():
    out = parse_hardware_error('arm_joint_2(ID14):과부하')
    assert len(out) == 1
    assert out[0]['dxl_id'] == 14
    assert out[0]['joint'] == 'arm_joint_2'
    assert out[0]['labels'] == ['과부하']
    assert out[0]['soft_trip'] is False
    assert out[0]['soft_spike'] is False


def test_comma_inside_label_does_not_split():
    """⚠️ 핵심 회귀 — `전류급변(SW,비상정지)` 라벨 안의 쉼표.

    단순 split(",") 이면 모터 2개가 3조각이 된다.
    """
    raw = ('arm_joint_2(ID14):과부하,'
           'gripper_left_pinion_joint(ID3):전류급변(SW,비상정지)|과열')
    out = parse_hardware_error(raw)

    assert len(out) == 2, f'모터 2개여야 하는데 {len(out)}개로 쪼개짐: {out}'
    assert out[0]['dxl_id'] == 14
    assert out[1]['dxl_id'] == 3
    assert out[1]['joint'] == 'gripper_left_pinion_joint'
    assert out[1]['labels'] == ['전류급변(SW,비상정지)', '과열']
    assert out[1]['soft_spike'] is True
    assert out[1]['soft_trip'] is False


def test_comma_label_in_the_middle():
    """쉼표 라벨이 마지막이 아니라 가운데 있어도 경계를 지켜야 한다."""
    raw = ('a(ID1):과부하,'
           'b(ID2):전류급변(SW,비상정지),'
           'c(ID3):과열')
    out = parse_hardware_error(raw)
    assert [e['dxl_id'] for e in out] == [1, 2, 3]
    assert out[1]['labels'] == ['전류급변(SW,비상정지)']


def test_soft_trip_bit():
    out = parse_hardware_error('gripper_left_pinion_joint(ID3):전류초과(SW)')
    assert out[0]['soft_trip'] is True
    assert out[0]['soft_spike'] is False


def test_unknown_byte_label_survives():
    # _describe_hw_error 는 매칭되는 비트가 없으면 '알수없음(0xNN)' 을 낸다.
    out = parse_hardware_error('arm_joint_4(ID12):알수없음(0x02)')
    assert out[0]['labels'] == ['알수없음(0x02)']
    assert out[0]['dxl_id'] == 12


def test_malformed_entry_is_surfaced_not_dropped():
    out = parse_hardware_error('쓰레기문자열')
    assert len(out) == 1
    assert out[0]['dxl_id'] is None
    assert out[0]['labels'] == ['쓰레기문자열']


def test_latched_ids_and_summary():
    raw = 'a(ID1):과부하,b(ID2):전류급변(SW,비상정지)|과열'
    out = parse_hardware_error(raw)
    assert latched_ids(out) == frozenset({1, 2})
    assert summarize(out) == 'ID1 과부하 · ID2 전류급변(SW,비상정지) | 과열'
    assert summarize([]) == ''
