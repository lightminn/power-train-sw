from chassis.authority import CommandAuthority, MANUAL_SOURCE, TELEOP


def test_selected_command_keeps_source_receipt_not_selection_time():
    authority = CommandAuthority()
    authority.set_mode(TELEOP)
    authority.submit(MANUAL_SOURCE, 0.0, 0.0, 10.0, steering=0.0)
    assert authority.select(10.1).received_s == 10.0
    authority.submit(MANUAL_SOURCE, 0.4, 0.0, 10.15, steering=0.3)
    assert authority.select(10.2).received_s == 10.15
    assert authority.select(10.3).received_s == 10.15


def test_stale_and_hold_generated_zeros_cannot_impersonate_received_input():
    authority = CommandAuthority()
    authority.set_mode(TELEOP)
    authority.submit(MANUAL_SOURCE, 0.0, 0.0, 10.0)
    authority.select(10.0)
    stale = authority.select(10.31)
    assert not stale.ok
    assert stale.received_s is None
    hold_zero = authority.select(10.32)
    assert hold_zero.ok and hold_zero.v == 0
    assert hold_zero.received_s is None


def test_tcp_source_receipt_stays_attached_to_its_exact_command_across_dds_republish():
    authority = CommandAuthority()
    authority.set_mode(TELEOP)
    authority.submit(MANUAL_SOURCE, 0, 0, 10., source_received_s=9.99)
    assert authority.select(10.).source_received_s == 9.99
    authority.submit(MANUAL_SOURCE, .4, 0, 10.1, source_received_s=10.01)
    old = authority.select(10.1)
    authority.submit(MANUAL_SOURCE, .4, 0, 10.2, source_received_s=10.01)
    repeated = authority.select(10.2)
    assert old.source_received_s == repeated.source_received_s == 10.01
    assert repeated.received_s == 10.2
    authority.submit(MANUAL_SOURCE, 0, 0, 10.21, source_received_s=10.2)
    neutral = authority.select(10.21)
    assert neutral.v == 0 and neutral.source_received_s == 10.2
