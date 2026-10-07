"""StateStore 회귀 테스트 (ROS·하드웨어 불필요).

특히 노드의 급변(spike) 판정 재현과 "온도 0 = 미수신" 규약을 고정한다.
"""

from robot_arm_gui.hw_error_parse import parse_hardware_error
from robot_arm_gui.state_store import StateStore


def make_store(**kw):
    # 벽시계를 고정해 이벤트 타임스탬프까지 결정론적으로 만든다.
    kw.setdefault('wall_fn', lambda: 1000.0)
    return StateStore(**kw)


def test_motor_mapping_uses_id_not_index():
    """모터가 ping 에 실패하면 그 자리가 통째로 빠진다 — id 로 매핑해야 한다."""
    s = make_store()
    s.update_motors([(14, 1024, 30, 480, 52), (3, 900, 0, 410, 44)], now=1.0)
    snap = s.snapshot(now=1.0)
    assert [m['id'] for m in snap['motors']] == [14, 3]
    assert snap['motors'][0]['current'] == 480


def test_temperature_zero_means_unknown_not_zero_celsius():
    s = make_store()
    s.update_motors([(12, 2048, 0, 15, 0)], now=1.0)
    m = s.snapshot(now=1.0)['motors'][0]
    assert m['temp'] is None, '온도 0 은 "미수신"이라 None 이어야 한다'
    assert m['temp_known'] is False

    # 라운드로빈이 닿으면 값이 실린다.
    s.update_motors([(12, 2048, 0, 15, 37)], now=2.0)
    m = s.snapshot(now=2.0)['motors'][0]
    assert m['temp'] == 37
    assert m['temp_known'] is True


def test_spike_baseline_is_taken_before_append():
    """노드(`read_state`)와 **순서까지** 같아야 한다.

    baseline 을 append 뒤에 뜨면 자기 자신이 최솟값 후보가 되어 delta 가
    항상 0 에 가깝게 나오고, 트립 여유가 과대평가된다.
    """
    s = make_store()
    # 첫 샘플: 창이 비어 있으므로 baseline 없음 → delta 없음 (노드와 동일)
    s.update_motors([(14, 0, 0, 100, 40)], now=1.0)
    assert s.snapshot(now=1.0)['motors'][0]['spike_delta'] is None

    s.update_motors([(14, 0, 0, 100, 40)], now=1.1)
    assert s.snapshot(now=1.1)['motors'][0]['spike_delta'] == 0

    # baseline 은 append 전 창의 최솟값(100) → delta = 450-100 = 350
    s.update_motors([(14, 0, 0, 450, 40)], now=1.2)
    m = s.snapshot(now=1.2)['motors'][0]
    assert m['spike_baseline'] == 100
    assert m['spike_delta'] == 350
    assert abs(m['spike_ratio'] - 1.0) < 1e-9


def test_spike_uses_absolute_value():
    """노드는 부호를 무시하고 |전류| 로 비교한다("더 힘쓰는 쪽"만 본다)."""
    s = make_store()
    s.update_motors([(14, 0, 0, -50, 40)], now=1.0)
    s.update_motors([(14, 0, 0, -400, 40)], now=1.1)
    m = s.snapshot(now=1.1)['motors'][0]
    assert m['spike_baseline'] == 50
    assert m['spike_delta'] == 350


def test_trip_headroom_and_ratio():
    s = make_store()
    s.update_motors([(14, 0, 0, 480, 52)], now=1.0)
    m = s.snapshot(now=1.0)['motors'][0]
    assert m['trip_headroom'] == 20          # 기본 임계 500
    assert abs(m['trip_ratio'] - 0.96) < 1e-9
    assert m['current_ma'] == 1291.2         # 480 × 2.69


def test_threshold_provenance():
    """토픽이 latched 가 아니라, GUI 가 늦게 붙으면 기동값만 안다."""
    s = make_store()
    assert s.snapshot(now=1.0)['thresholds']['trip']['source'] == 'startup'
    s.set_threshold('trip', True, 350, now=2.0)
    t = s.snapshot(now=2.0)['thresholds']['trip']
    assert t['value'] == 350
    assert t['source'] == 'runtime'
    assert t['at'] == 1000.0


def test_threshold_zero_keeps_previous_value():
    """노드 콜백과 같은 규약 — threshold <= 0 이면 값은 유지하고 enable 만 바꾼다."""
    s = make_store()
    s.set_threshold('spike', False, 0, now=1.0)
    t = s.snapshot(now=1.0)['thresholds']['spike']
    assert t['value'] == 350
    assert t['enabled'] is False


def test_hw_error_only_edges_become_events():
    """30Hz 로 같은 문자열이 계속 와도 이벤트는 엣지에서만 생긴다."""
    s = make_store()
    raw = 'arm_joint_2(ID14):과부하'
    for i in range(30):
        s.set_hardware_error(parse_hardware_error(raw), now=1.0 + i * 0.03)
    events = [e for e in s.snapshot(now=2.0)['events'] if e['kind'] == 'hw_error']
    assert len(events) == 1

    # 해제되면 하강 엣지 이벤트 하나.
    s.set_hardware_error(parse_hardware_error(''), now=3.0)
    clears = [e for e in s.snapshot(now=3.0)['events'] if e['kind'] == 'hw_error_clear']
    assert len(clears) == 1


def test_rising_edge_freezes_trace():
    """트립 당시 수치는 노드에서 로그로만 남는다 — 여기서 링버퍼를 동결한다."""
    s = make_store(state_rate_hz=30.0, trace_seconds=3.0)
    for i in range(200):                       # 링버퍼(90)를 넘겨서 채운다
        s.update_motors([(14, 0, 0, 100 + i, 40)], now=i * 0.033)

    trace_id = s.set_hardware_error(
        parse_hardware_error('arm_joint_2(ID14):전류급변(SW,비상정지)'), now=10.0)
    assert trace_id is not None

    trace = s.get_trace(trace_id)
    assert len(trace['samples']) == 90         # 30Hz × 3초
    assert trace['samples'][-1]['motors'][0][3] == 299   # 마지막 전류값
    assert trace['thresholds']['spike']['value'] == 350

    ev = [e for e in s.snapshot(now=10.0)['events'] if e['kind'] == 'hw_error']
    assert ev[0]['trace'] == trace_id
    assert ev[0]['severity'] == 'critical'


def test_tick_limits_empty_means_off():
    s = make_store()
    s.set_tick_limits([14, 100, 3000], now=1.0)
    assert s.snapshot(now=1.0)['tick_limits']['empty'] is False
    s.set_tick_limits([], now=2.0)
    snap = s.snapshot(now=2.0)
    assert snap['tick_limits']['empty'] is True
    assert any(e['kind'] == 'tick_limits' and e['severity'] == 'serious'
               for e in snap['events'])


def test_goal_error_from_subscribed_goal():
    """`/dynamixel/goal_position` 은 **구독**만 한다 — 계약이 금지하는 건 발행이다."""
    s = make_store()
    s.update_motors([(14, 1000, 0, 10, 40)], now=1.0)
    s.set_goal(14, 1120, now=1.0)
    assert s.snapshot(now=1.0)['motors'][0]['goal_error'] == 120


def test_deadman_is_observable_from_joy():
    s = make_store()
    s.set_joy_params(deadman=9, turbo=-1, estop=-1, resolved=True)
    s.set_joy([0] * 9 + [1], [], now=1.0)
    joy = s.snapshot(now=1.0)['joy']
    assert joy['deadman_held'] is True
    s.set_joy([0] * 12, [], now=1.1)
    assert s.snapshot(now=1.1)['joy']['deadman_held'] is False


def test_deadman_unknown_when_no_joy_yet():
    s = make_store()
    s.set_joy_params(deadman=9, turbo=-1, estop=-1, resolved=False)
    assert s.snapshot(now=1.0)['joy']['deadman_held'] is None


def test_topic_freshness_ages():
    s = make_store()
    s.set_arm_status('EXECUTING', 7, now=1.0)
    snap = s.snapshot(now=1.4)
    assert abs(snap['topics']['/arm_status']['age'] - 0.4) < 1e-9
    assert abs(snap['arm']['age'] - 0.4) < 1e-9


def test_sparkline_buckets_and_gaps():
    s = make_store(bucket_s=0.5, spark_buckets=120)
    s.update_motors([(14, 0, 0, 100, 40)], now=0.0)
    s.update_motors([(14, 0, 0, 300, 40)], now=0.2)     # 같은 버킷
    s.update_motors([(14, 0, 0, 50, 40)], now=0.6)      # 다음 버킷
    spark = s.snapshot(now=0.6)['motors'][0]['spark']
    assert spark[0] == [100, 300, 300]                  # min, max, last
    assert spark[-1] == [50, 50, 50]

    # 수신이 끊긴 구간은 None 으로 채워 "값 없음"과 "0"을 구분한다.
    s.update_motors([(14, 0, 0, 70, 40)], now=2.6)
    spark = s.snapshot(now=2.6)['motors'][0]['spark']
    assert None in spark


def test_hot_snapshot_has_no_events():
    s = make_store()
    s.set_arm_status('EXECUTING', 7, now=1.0)
    hot = s.hot_snapshot(now=1.0)
    assert 'events' not in hot
    assert hot['arm']['status'] == 'EXECUTING'


def test_calib_status_keeps_the_raw_text_next_to_the_parse():
    """파서가 형식을 못 알아봐도 운영자는 원문을 볼 수 있어야 한다."""
    s = make_store()
    s.set_calib_status('active,arm_joint_2,lower,1,4', now=1.0)
    teleop = s.snapshot(now=1.0)['teleop']
    assert teleop['calib'] == 'active,arm_joint_2,lower,1,4'
    assert teleop['calib_info']['joint'] == 'arm_joint_2'
    assert teleop['calib_info']['step_label'] == '하한'


def test_calib_event_is_readable_not_raw():
    """이벤트 로그에 'active,arm_joint_2,lower,1,4' 가 남으면 아무도 안 읽는다."""
    s = make_store()
    s.set_calib_status('done,3,1', now=1.0)
    event = [e for e in s.snapshot(now=1.0)['events'] if e['kind'] == 'calib'][-1]
    assert '거절' in event['text']


def test_hot_snapshot_carries_everything_the_motor_table_needs():
    """회귀 — thresholds 를 빼뒀더니 화면이 5Hz 갱신마다 예외로 죽었다.

    전체 스냅샷은 1Hz 라 표가 "가끔 되살아나는" 형태로 보여서 원인 찾기가 나쁘다.
    """
    s = make_store()
    s.update_motors([(14, 1024, 30, 480, 52)], now=1.0)
    hot = s.hot_snapshot(now=1.0)
    for key in ('motors', 'thresholds', 'arm', 'chassis', 'controller_fault',
                'hw_errors', 'topics'):
        assert key in hot, f'hot 스냅샷에 {key} 가 없다 — 모터 표 렌더가 깨진다'
    assert hot['thresholds']['trip']['value'] == 500
    assert hot['motors'][0]['spark']
