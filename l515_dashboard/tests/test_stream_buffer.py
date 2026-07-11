from l515_dashboard.stream_buffer import BoundedRing, LatestSlot, StreamSample


def sample(number, stream="color"):
    return StreamSample(stream, number, float(number), number, object())


def test_latest_slot_keeps_only_newest_without_blocking_producer():
    slot = LatestSlot()
    for number in range(1000):
        slot.publish(sample(number))
    sequence, value = slot.read_after(0)
    assert sequence == 1000
    assert value.frame_number == 999


def test_latest_slot_returns_no_sample_when_cursor_is_current_and_can_clear():
    slot = LatestSlot()
    slot.publish(sample(1))
    sequence, _ = slot.read_after(0)
    assert slot.read_after(sequence) == (sequence, None)
    slot.clear()
    assert slot.read_after(0) == (sequence, None)


def test_latest_slot_counts_only_replacement_of_unread_sample():
    slot = LatestSlot()
    slot.publish(sample(1)); sequence, _ = slot.read_after(0)
    slot.publish(sample(2))
    assert slot.overwrites == 0
    slot.publish(sample(3))
    assert slot.overwrites == 1
    slot.read_after(sequence)
    slot.publish(sample(4))
    assert slot.overwrites == 1


def test_ring_drops_oldest_and_reports_drop_count():
    ring = BoundedRing(capacity=4)
    for number in range(10):
        ring.publish(sample(number, "gyro"))
    result = ring.read_after(0, 10)
    assert [item.frame_number for item in result.samples] == [6, 7, 8, 9]
    assert result.sequence == 10
    assert ring.dropped == 6


def test_ring_cursor_reads_only_new_samples_and_clear_invalidates_contents():
    ring = BoundedRing(capacity=4)
    for number in range(3):
        ring.publish(sample(number, "accel"))
    first = ring.read_after(0, 2)
    assert [item.frame_number for item in first.samples] == [0, 1]
    second = ring.read_after(first.sequence, 2)
    assert [item.frame_number for item in second.samples] == [2]
    ring.clear()
    assert ring.read_after(0, 10).samples == ()


def test_stream_sample_is_immutable():
    value = sample(1)
    try:
        value.frame_number = 2
    except (AttributeError, TypeError):
        pass
    else:
        raise AssertionError("StreamSample must be immutable")
