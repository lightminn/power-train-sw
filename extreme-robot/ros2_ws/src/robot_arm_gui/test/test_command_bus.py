"""CommandBus — 조종권 락과 워치독. ROS 없이 도는 검증이다."""

from robot_arm_gui.command_bus import CommandBus


def make_bus(**kw):
    kw.setdefault('token_ttl_s', 5.0)
    kw.setdefault('intent_timeout_s', 0.3)
    return CommandBus(**kw)


# ------------------------------------------------------------ 조종권
def test_second_claim_is_refused_while_first_is_alive():
    bus = make_bus()
    token, err = bus.claim('tab-A', now=100.0)
    assert token and err is None

    other, err = bus.claim('tab-B', now=100.5)
    assert other is None
    assert 'tab-A' in err


def test_claim_succeeds_after_ttl_expires():
    """조종하던 탭이 죽어도 TTL 뒤에는 자동으로 풀려야 한다."""
    bus = make_bus(token_ttl_s=5.0)
    first, _ = bus.claim('tab-A', now=100.0)
    second, err = bus.claim('tab-B', now=106.0)
    assert second is not None and err is None
    # 낡은 토큰으로는 아무것도 못 한다.
    assert bus.set_jog(first, {'arm_joint_2': 0.5}, now=106.1) is False


def test_force_claim_takes_over():
    bus = make_bus()
    first, _ = bus.claim('tab-A', now=100.0)
    second, err = bus.claim('tab-B', now=100.1, force=True)
    assert second is not None and err is None
    assert second != first


def test_renew_keeps_token_alive_past_ttl():
    bus = make_bus(token_ttl_s=5.0)
    token, _ = bus.claim('tab-A', now=100.0)
    assert bus.renew(token, now=104.0) is True
    assert bus.renew(token, now=108.0) is True     # 104 기준으로 아직 살아있다
    assert bus.holder(now=108.0) is not None


def test_release_frees_the_lock():
    bus = make_bus()
    token, _ = bus.claim('tab-A', now=100.0)
    assert bus.release(token, now=100.5) is True
    assert bus.holder(now=100.5) is None
    other, err = bus.claim('tab-B', now=100.6)
    assert other is not None and err is None


def test_claim_does_not_inherit_previous_jog():
    """새 조종자가 이전 조종자의 속도를 물려받으면 안 된다."""
    bus = make_bus()
    first, _ = bus.claim('tab-A', now=100.0)
    bus.set_jog(first, {'arm_joint_2': 0.7}, now=100.0)
    bus.release(first, now=100.1)

    second, _ = bus.claim('tab-B', now=100.2)
    state, _ = bus.take_jog(now=100.2)
    assert state != 'active'


# ------------------------------------------------------------ 워치독
def test_fresh_intent_is_published():
    bus = make_bus()
    token, _ = bus.claim('tab-A', now=100.0)
    bus.set_jog(token, {'arm_joint_2': 0.4}, now=100.0)
    state, velocities = bus.take_jog(now=100.1)
    assert state == 'active'
    assert velocities == {'arm_joint_2': 0.4}


def test_stale_intent_emits_stop_exactly_once():
    """0 을 계속 쏘면 teleop_core 의 deadman 이 영원히 발동하지 못한다."""
    bus = make_bus(intent_timeout_s=0.3)
    token, _ = bus.claim('tab-A', now=100.0)
    bus.set_jog(token, {'arm_joint_2': 0.4}, now=100.0)
    assert bus.take_jog(now=100.1)[0] == 'active'

    assert bus.take_jog(now=100.5)[0] == 'stop'
    assert bus.take_jog(now=100.6)[0] == 'idle'
    assert bus.take_jog(now=101.0)[0] == 'idle'


def test_boot_state_does_not_emit_stop():
    """부팅 직후엔 이미 정지 상태다 — 아무도 조종한 적 없는데 0 을 쏘면 안 된다."""
    bus = make_bus()
    assert bus.take_jog(now=100.0)[0] == 'idle'


def test_releasing_control_triggers_a_stop():
    """조종권을 놓으면 팔이 마지막 속도로 계속 돌면 안 된다."""
    bus = make_bus()
    token, _ = bus.claim('tab-A', now=100.0)
    bus.set_jog(token, {'arm_joint_2': 0.4}, now=100.0)
    assert bus.take_jog(now=100.0)[0] == 'active'

    bus.release(token, now=100.05)
    assert bus.take_jog(now=100.06)[0] == 'stop'


def test_expired_token_triggers_a_stop():
    """브라우저가 죽어 renew 가 끊긴 경우도 같은 결과여야 한다."""
    bus = make_bus(token_ttl_s=5.0, intent_timeout_s=0.3)
    token, _ = bus.claim('tab-A', now=100.0)
    bus.set_jog(token, {'arm_joint_2': 0.4}, now=100.0)
    assert bus.take_jog(now=100.0)[0] == 'active'
    assert bus.take_jog(now=200.0)[0] == 'stop'


# ------------------------------------------------------------ 정지 사유
def test_release_jog_stops_once_and_is_not_a_watchdog_event():
    """키를 떼는 정상 조작이 통신 두절과 같은 경고로 기록되면 안 된다."""
    bus = make_bus(intent_timeout_s=0.3)
    token, _ = bus.claim('tab-A', now=100.0)
    bus.set_jog(token, {'arm_joint_2': 0.4}, now=100.0)
    assert bus.take_jog(now=100.0)[0] == 'active'

    assert bus.release_jog(token, now=100.05) is True
    assert bus.take_jog(now=100.06)[0] == 'stop'
    assert bus.last_stop_reason() == 'released'
    # 정지는 한 번뿐 — 계속 쏘면 teleop_core 의 deadman 이 발동하지 못한다.
    assert bus.take_jog(now=100.1)[0] == 'idle'


def test_dropped_intent_is_still_a_watchdog_stop():
    bus = make_bus(intent_timeout_s=0.3)
    token, _ = bus.claim('tab-A', now=100.0)
    bus.set_jog(token, {'arm_joint_2': 0.4}, now=100.0)
    assert bus.take_jog(now=100.0)[0] == 'active'

    assert bus.take_jog(now=100.5)[0] == 'stop'
    assert bus.last_stop_reason() == 'watchdog'


def test_release_jog_then_new_intent_is_a_watchdog_again():
    """해제 플래그가 다음 정지까지 남아 진짜 두절을 가리면 안 된다."""
    bus = make_bus(intent_timeout_s=0.3)
    token, _ = bus.claim('tab-A', now=100.0)
    bus.set_jog(token, {'arm_joint_2': 0.4}, now=100.0)
    bus.take_jog(now=100.0)
    bus.release_jog(token, now=100.05)
    bus.take_jog(now=100.06)                       # released

    bus.set_jog(token, {'arm_joint_2': 0.4}, now=100.2)
    assert bus.take_jog(now=100.2)[0] == 'active'
    assert bus.take_jog(now=100.7)[0] == 'stop'
    assert bus.last_stop_reason() == 'watchdog'


def test_release_jog_while_already_stopped_emits_nothing():
    bus = make_bus()
    token, _ = bus.claim('tab-A', now=100.0)
    assert bus.release_jog(token, now=100.0) is True
    assert bus.take_jog(now=100.0)[0] == 'idle'


def test_release_jog_requires_the_token():
    bus = make_bus()
    bus.claim('tab-A', now=100.0)
    assert bus.release_jog('bogus-token', now=100.0) is False


def test_releasing_control_is_not_a_watchdog_stop():
    """조종권 반납도 운영자의 정상 조작이다."""
    bus = make_bus()
    token, _ = bus.claim('tab-A', now=100.0)
    bus.set_jog(token, {'arm_joint_2': 0.4}, now=100.0)
    bus.take_jog(now=100.0)

    bus.release(token, now=100.05)
    assert bus.take_jog(now=100.06)[0] == 'stop'
    assert bus.last_stop_reason() == 'released'


def test_jog_without_token_is_rejected():
    bus = make_bus()
    bus.claim('tab-A', now=100.0)
    assert bus.set_jog('bogus-token', {'arm_joint_2': 0.4}, now=100.0) is False


# ------------------------------------------------------------ 명령/작업
def test_commands_drain_once():
    bus = make_bus()
    token, _ = bus.claim('tab-A', now=100.0)
    assert bus.push_cmd(token, 'stop', now=100.0) is True
    assert bus.drain_cmds() == ['stop']
    assert bus.drain_cmds() == []


def test_commands_require_the_token():
    bus = make_bus()
    bus.claim('tab-A', now=100.0)
    assert bus.push_cmd(None, 'stop', now=100.0) is False
    assert bus.drain_cmds() == []


def test_task_lifecycle_is_visible_to_pollers():
    bus = make_bus()
    token, _ = bus.claim('tab-A', now=100.0)
    task_id, err = bus.push_task(token, 'model_swap', {'key': 'box'}, now=100.0)
    assert task_id == 1 and err is None
    assert bus.task_results()[0]['state'] == 'pending'

    tasks = bus.drain_tasks()
    assert tasks[0]['kind'] == 'model_swap'
    bus.finish_task(task_id, 'done', '0.83초', now=101.0)

    result = bus.task_results()[0]
    assert result['state'] == 'done'
    assert result['detail'] == '0.83초'


def test_task_results_can_be_polled_incrementally():
    bus = make_bus()
    token, _ = bus.claim('tab-A', now=100.0)
    first, _ = bus.push_task(token, 'a', {}, now=100.0)
    second, _ = bus.push_task(token, 'b', {}, now=100.1)
    assert [e['id'] for e in bus.task_results(since=first)] == [second]


# ------------------------------------------------------------ 스냅샷
def test_snapshot_reports_no_holder_when_idle():
    bus = make_bus()
    snap = bus.snapshot(now=100.0)
    assert snap['holder'] is None
    assert snap['stopped'] is True


def test_snapshot_counts_down_the_token():
    bus = make_bus(token_ttl_s=5.0)
    bus.claim('tab-A', now=100.0)
    snap = bus.snapshot(now=102.0)
    assert snap['holder']['label'] == 'tab-A'
    assert snap['holder']['expires_in_s'] == 3.0
