import pytest
from dynamixel_control.local_wrist_jog import WristJogLease


def request(**changes):
    return dict(dict(command='JOG', axis=5, direction=1, sequence=1,
                     expires_at_s=10.6), **changes)


def test_deadman_expires_and_stop_invalidates_older_jogs():
    lease = WristJogLease()
    assert lease.accept(request(), 10) == 1
    assert not lease.expired(10.59)
    assert lease.expired(10.6)
    assert lease.accept(request(command='STOP', sequence=2), 10.1) == 0
    assert lease.deadline is None
    with pytest.raises(ValueError, match='order'):
        lease.accept(request(), 10.2)


@pytest.mark.parametrize('changes', [dict(axis=1), dict(axis=True),
    dict(direction=True), dict(direction=0), dict(sequence=-1),
    dict(expires_at_s=9), dict(expires_at_s=12), dict(expires_at_s=float('nan'))])
def test_invalid_or_stale_jog_does_not_create_lease(changes):
    lease = WristJogLease()
    with pytest.raises(ValueError):
        lease.accept(request(**changes), 10)
    assert lease.deadline is None


def test_same_hold_can_refresh_but_reordered_deadlines_cannot():
    lease = WristJogLease()
    lease.accept(request(), 10)
    assert lease.accept(request(expires_at_s=10.8), 10.2) == 1
    with pytest.raises(ValueError, match='order'):
        lease.accept(request(expires_at_s=10.7), 10.3)


def test_fresh_refresh_can_resume_after_deadman_but_not_after_explicit_stop():
    lease = WristJogLease()
    lease.accept(request(), 10)
    lease.deadline = None
    assert lease.accept(request(expires_at_s=11.3), 10.8) == 1
    lease.accept(request(command='STOP', sequence=2), 10.9)
    with pytest.raises(ValueError, match='order'):
        lease.accept(request(expires_at_s=11.6), 11)
