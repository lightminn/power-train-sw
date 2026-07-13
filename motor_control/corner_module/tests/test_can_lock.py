"""CAN 버스 단독 소유권 락.

`chassis_node` 와 `teleop_server` 를 동시에 띄우면 **같은 모터에 상반된 명령**이 50 Hz 로
번갈아 간다. socketcan 은 이를 막지 않는다. 지금까지 운영 관례로만 막고 있었다.
"""
import multiprocessing as mp
import time

import pytest

from corner_module.can_lock import CanBusBusy, can_bus_lock, is_locked

CH = "test_can_lock_ch"          # 실기 can0 를 건드리지 않는 전용 채널명


def test_lock_is_exclusive():
    with can_bus_lock(CH, owner="first"):
        with pytest.raises(CanBusBusy):
            with can_bus_lock(CH, owner="second"):
                pass


def test_lock_is_released_on_exit():
    with can_bus_lock(CH):
        assert is_locked(CH)
    assert not is_locked(CH)                 # 빠져나오면 풀린다

    with can_bus_lock(CH):                   # 다시 잡을 수 있다
        pass


def test_lock_released_on_exception():
    """예외로 죽어도 락이 남으면 안 된다 — 좀비 락이 다음 실행을 막는다."""
    with pytest.raises(ValueError):
        with can_bus_lock(CH):
            raise ValueError("boom")
    assert not is_locked(CH)


def test_different_channels_do_not_collide():
    with can_bus_lock(CH + "_a"):
        with can_bus_lock(CH + "_b"):        # 다른 버스는 독립
            pass


def _hold(ch, ready, stop):
    with can_bus_lock(ch, owner="child"):
        ready.set()
        stop.wait(5.0)


def test_lock_is_visible_across_processes():
    """★ 컨테이너를 넘나드는 게 목적이다 — 최소한 프로세스는 넘어야 한다.

    추상 유닉스 소켓은 파일시스템이 아니라 **네트워크 네임스페이스**에 속하므로
    (network_mode: host 인) 두 컨테이너가 서로 본다.
    """
    ctx = mp.get_context("fork")
    ready, stop = ctx.Event(), ctx.Event()
    p = ctx.Process(target=_hold, args=(CH, ready, stop))
    p.start()
    try:
        assert ready.wait(5.0)
        with pytest.raises(CanBusBusy):       # 부모는 못 잡는다
            with can_bus_lock(CH, owner="parent"):
                pass
    finally:
        stop.set()
        p.join(5.0)

    # ★ 프로세스가 죽으면 커널이 소켓을 닫아 **락이 자동 해제**된다.
    #   (좀비 teleop 이 계속 v=0 을 명령해 새 테스트와 싸운 사고가 있었다.)
    for _ in range(50):
        if not is_locked(CH):
            break
        time.sleep(0.02)
    assert not is_locked(CH)


def test_error_message_tells_you_what_to_do():
    with can_bus_lock(CH):
        with pytest.raises(CanBusBusy) as e:
            with can_bus_lock(CH):
                pass
    msg = str(e.value)
    assert "상반된 명령" in msg                # 왜 위험한지
    assert "pgrep" in msg                     # 어떻게 찾는지
