from dataclasses import FrozenInstanceError

import pytest

from l515_dashboard.diagnostics import (
    ACCEL_TOPIC,
    ALL_TOPICS,
    COLOR_INFO_TOPIC,
    COLOR_TOPIC,
    DEPTH_INFO_TOPIC,
    DEPTH_TOPIC,
    GYRO_TOPIC,
    DiagnosticsTracker,
)


NS = 1_000_000_000


def test_observe_reports_rolling_rate_age_gap_and_nonincreasing_stamp():
    tracker = DiagnosticsTracker()

    tracker.observe(COLOR_TOPIC, stamp_ns=10 * NS, now_ns=20 * NS)
    tracker.observe(COLOR_TOPIC, stamp_ns=10 * NS, now_ns=20_500_000_000)
    tracker.observe(COLOR_TOPIC, stamp_ns=11 * NS, now_ns=21 * NS)

    metric = tracker.snapshot(now_ns=21_250_000_000).topics[COLOR_TOPIC]
    assert metric.count == 3
    assert metric.fps == pytest.approx(2.0)
    assert metric.age_s == pytest.approx(0.25)
    assert metric.max_gap_s == pytest.approx(1.0)
    assert metric.nonincreasing_count == 1


def test_old_arrivals_leave_bounded_rolling_window():
    tracker = DiagnosticsTracker(window_s=2.0, max_arrivals=3)
    for second in range(5):
        tracker.observe(COLOR_TOPIC, stamp_ns=second * NS, now_ns=second * NS)

    metric = tracker.snapshot(now_ns=4 * NS).topics[COLOR_TOPIC]
    assert metric.count == 3
    assert metric.fps == pytest.approx(1.0)
    assert metric.max_gap_s == pytest.approx(1.0)


def test_expired_nonincreasing_event_and_stamp_history_leave_window():
    tracker = DiagnosticsTracker(window_s=1.0)
    tracker.observe(COLOR_TOPIC, stamp_ns=100, now_ns=0)
    tracker.observe(COLOR_TOPIC, stamp_ns=100, now_ns=1)

    expired = tracker.snapshot(now_ns=NS + 2).topics[COLOR_TOPIC]
    assert expired.count == 0
    assert expired.nonincreasing_count == 0

    tracker.observe(COLOR_TOPIC, stamp_ns=50, now_ns=NS + 2)
    restarted = tracker.snapshot(now_ns=NS + 2).topics[COLOR_TOPIC]
    assert restarted.count == 1
    assert restarted.nonincreasing_count == 0


def test_health_requires_all_six_topics_with_type_specific_freshness():
    tracker = DiagnosticsTracker()
    now_ns = 10 * NS
    for topic in ALL_TOPICS:
        tracker.observe(topic, stamp_ns=now_ns, now_ns=now_ns)

    assert tracker.snapshot(now_ns=now_ns + 200_000_000).healthy is True
    assert tracker.snapshot(now_ns=now_ns + 300_000_000).healthy is False

    refreshed = DiagnosticsTracker()
    for topic in ALL_TOPICS:
        age_ns = 400_000_000 if topic in (COLOR_INFO_TOPIC, DEPTH_INFO_TOPIC) else 0
        refreshed.observe(topic, stamp_ns=now_ns, now_ns=now_ns - age_ns)
    assert refreshed.snapshot(now_ns=now_ns).healthy is True


def test_snapshot_is_immutable_and_retains_only_scalar_diagnostics():
    tracker = DiagnosticsTracker()
    tracker.observe(DEPTH_TOPIC, stamp_ns=1, now_ns=2)
    snapshot = tracker.snapshot(now_ns=3)

    with pytest.raises(FrozenInstanceError):
        snapshot.healthy = False
    with pytest.raises(TypeError):
        snapshot.topics[DEPTH_TOPIC] = snapshot.topics[DEPTH_TOPIC]

    assert set(snapshot.topics) == set(ALL_TOPICS)
    assert not any(
        hasattr(value, "data")
        for metric in snapshot.topics.values()
        for value in vars(metric).values()
    )


def test_unknown_topic_is_rejected_without_creating_state():
    tracker = DiagnosticsTracker()

    with pytest.raises(ValueError, match="unknown L515 topic"):
        tracker.observe("/not/l515", stamp_ns=1, now_ns=1)

    assert set(tracker.snapshot(now_ns=1).topics) == set(ALL_TOPICS)


def test_topic_contract_is_exact():
    assert ALL_TOPICS == (
        "/l515/color/image_raw",
        "/l515/color/camera_info",
        "/l515/depth/image_rect_raw",
        "/l515/depth/camera_info",
        "/l515/gyro/sample",
        "/l515/accel/sample",
    )
    assert GYRO_TOPIC in ALL_TOPICS
    assert ACCEL_TOPIC in ALL_TOPICS
