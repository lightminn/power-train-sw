"""calib_status_parse — `/arm/calib_status` 규약 고정. ROS 없이 도는 검증이다.

여기 문자열은 전부 `teleop_core_node._publish_calib_status` 가 실제로 내보내는
형태다. 그쪽이 형식을 바꾸면 이 파일이 먼저 깨져야 한다 — 화면이 조용히 이상한
값을 그리는 것보다 낫다.
"""

from robot_arm_gui.calib_status_parse import parse, summary


def test_idle():
    assert parse('idle')['state'] == 'idle'


def test_empty_is_idle_not_unknown():
    """노드가 뜨기 전에는 아무것도 안 왔을 뿐이다 — 오류로 보이면 안 된다."""
    assert parse('')['state'] == 'idle'
    assert parse(None)['state'] == 'idle'


def test_active_progress():
    info = parse('active,arm_joint_2,lower,1,4')
    assert info['state'] == 'active'
    assert info['joint'] == 'arm_joint_2'
    assert info['step'] == 'lower'
    assert info['step_label'] == '하한'
    assert (info['index'], info['total']) == (1, 4)


def test_progress_counts_both_steps_of_each_joint():
    """축 번호만 세면 진행률이 절반씩 튄다 — 축마다 하한·상한 두 단계다."""
    assert parse('active,arm_joint_1,lower,1,4')['progress'] == 0.0
    assert parse('active,arm_joint_1,upper,1,4')['progress'] == 0.125
    assert parse('active,arm_joint_2,lower,2,4')['progress'] == 0.25
    assert parse('active,arm_joint_4,upper,4,4')['progress'] == 0.875


def test_done_carries_the_rejected_count():
    info = parse('done,3,1')
    assert info['state'] == 'done'
    assert (info['applied'], info['rejected']) == (3, 1)


def test_cancelled():
    assert parse('cancelled')['state'] == 'cancelled'


def test_unknown_shapes_keep_the_original_text():
    """모르는 형식을 버리면 형식이 바뀐 사실 자체가 화면에서 사라진다."""
    for raw in ('active,arm_joint_2,lower', 'done,3', 'weird', 'active,a,b,c,d',
                'idle,extra'):
        info = parse(raw)
        assert info['state'] == 'unknown', raw
        assert info['raw'] == raw


def test_zero_total_is_not_a_division_by_zero():
    assert parse('active,arm_joint_2,lower,1,0')['state'] == 'unknown'


def test_summary_mentions_rejected_joints():
    """거절된 축은 재측정 전까지 리밋이 없는 상태다 — 조용히 넘기면 안 된다."""
    assert '거절' in summary(parse('done,3,1'))
    assert '거절' not in summary(parse('done,4,0'))


def test_summary_reads_like_a_status_line():
    assert summary(parse('idle')) == '대기 중'
    assert 'arm_joint_2' in summary(parse('active,arm_joint_2,upper,2,4'))
    assert '상한' in summary(parse('active,arm_joint_2,upper,2,4'))
    assert '취소' in summary(parse('cancelled'))
    assert '알 수 없는' in summary(parse('nonsense'))
