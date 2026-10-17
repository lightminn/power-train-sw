"""Console terrain reception and display decisions (spec §4.6, §7.1)."""
import json

import pytest

from operator_console.terrain import (
    LatestTerrainReceiver, TerrainSessionGate, terrain_view,
)
from terrain_overlay.contract import parse_packet

BOOT = "0082f17fdc0a49969ff0fe1dc4979560"
OTHER_BOOT = "1111111111111111111111111111111a"
A = (BOOT, 1000, "aaaaaaaaaaaaaaaa")
B = (BOOT, 2000, "aaaaaaaaaaaaaaaa")


def payload(session=A, sequence=0, status="OK", **overrides):
    boot, start, instance = session
    body = {
        "schema_version": 1, "type": "l515_terrain",
        "sender_boot_id": boot, "sender_start_mono_ns": start, "sender_instance": instance,
        "sequence": sequence, "sent_unix_ns": 1, "depth_stamp_ns": 1, "depth_age_ms": 40.0,
        "status": status, "reason": None, "frame_size": [1280, 720],
        "distance_origin": "l515_depth_optical", "alignment_verified": False,
        "ground": {"state": "FRESH", "age_ms": 40.0, "camera_height_m": 0.5, "camera_tilt_deg": 20.0},
        "path_half_width_m": 0.4445,
        "zones": {"left": None, "center": 1.23, "right": None},
        "obstacles": [{"dist_m": 1.23, "in_path": True, "poly": [[10, 10], [50, 10], [50, 50]]}],
        "proc_ms": 20.0,
    }
    if status != "OK":
        body.update(reason=overrides.pop("reason"), ground=None,
                    zones={"left": None, "center": None, "right": None}, obstacles=[])
    body.update(overrides)
    return json.dumps(body).encode()


def frame(session=A, sequence=0, rx=0.0, **kw):
    return parse_packet(payload(session, sequence, **kw), received_monotonic_s=rx)


@pytest.fixture
def receiver():
    made = []

    def make(delay_s=0.0):
        item = LatestTerrainReceiver(0, display_delay_s=delay_s)
        made.append(item)
        return item

    yield make
    for item in made:
        item.close()


def feed(rx, events):
    """events: (time, session, sequence) → list of accepted booleans."""
    return [rx.ingest(payload(session, seq), t) for t, session, seq in events]


def test_same_session_rejects_duplicates_and_reordering(receiver):
    rx = receiver()
    assert feed(rx, [(0.0, A, 5), (0.1, A, 5), (0.2, A, 4), (0.3, A, 6)]) == [True, False, False, True]


def test_new_session_of_the_same_boot_is_accepted_immediately(receiver):
    rx = receiver()
    assert feed(rx, [(0.0, A, 100), (0.1, B, 0), (0.2, A, 101)]) == [True, True, False]


def test_receive_gap_counterexample_never_returns_to_the_old_session(receiver):
    rx = receiver()
    events = [(0.00, A, 100), (1.10, B, 0), (1.30, B, 1), (2.41, A, 99), (2.42, B, 12)]
    assert feed(rx, events) == [True, True, True, False, True]


def test_concurrent_senders_show_only_the_larger_session(receiver):
    rx = receiver()
    events = []
    for i in range(5):
        events += [(i * 0.1, A, i), (i * 0.1 + 0.01, B, i)]
    accepted = feed(rx, events)
    assert accepted[0] is True and accepted[1] is True
    assert accepted[2::2] == [False] * 4 and accepted[3::2] == [True] * 4


def test_equal_start_is_split_by_instance(receiver):
    low = (BOOT, 5000, "1111111111111111")
    high = (BOOT, 5000, "2222222222222222")
    rx = receiver()
    assert feed(rx, [(0.0, low, 100), (0.1, high, 0), (0.2, low, 101), (0.3, high, 1)]) == [
        True, True, False, True]


def test_other_boot_waits_for_one_second_of_silence(receiver):
    rx = receiver()
    other = (OTHER_BOOT, 1, "bbbbbbbbbbbbbbbb")
    assert feed(rx, [(0.0, A, 1), (0.5, other, 0), (0.9, A, 2), (1.95, other, 1)]) == [
        True, False, True, True]


def test_session_switch_clears_the_display_buffer(receiver):
    rx = receiver(0.3)
    feed(rx, [(0.0, A, 1), (0.1, A, 2), (0.2, B, 0)])
    assert rx.display_frame(0.45) is None  # A frames gone, B not yet 0.3 s old
    assert rx.display_frame(0.5).session_key == B


def test_malformed_and_wrong_type_packets_count_as_invalid(receiver):
    rx = receiver()
    assert rx.ingest(b"{}", 0.0) is False
    assert rx.ingest(json.dumps({"schema_version": 1, "type": "yolo"}).encode(), 0.0) is False
    assert rx.invalid_packet_count == 2


def test_display_delay_keeps_link_alive_and_picks_an_older_frame(receiver):
    rx = receiver(0.3)
    for i in range(20):
        rx.ingest(payload(A, i), i * 0.1)
    now = 1.95
    assert now - rx.last_rx_monotonic < 0.15
    shown = rx.display_frame(now)
    assert shown.sequence == 16  # received at 1.6 <= 1.95 - 0.3
    view = terrain_view(rx.latest(), shown, rx.last_rx_monotonic, now, 0.3, (1280, 720))
    assert view.probe_state == "LIVE" and view.obstacles


@pytest.mark.parametrize("delay_s", [-0.1, 0.5, float("nan")])
def test_display_delay_range_is_enforced(delay_s):
    with pytest.raises(ValueError):
        LatestTerrainReceiver(0, display_delay_s=delay_s)


def view_at(now, *, delay=0.0, rx=0.0, video=(1280, 720), live=True, **kw):
    f = frame(rx=rx, **kw)
    display = f if f.received_monotonic_s <= now - delay else None
    return terrain_view(f, display, rx, now, delay, video, video_live=live)


def test_depth_age_hides_an_old_frame_without_delay():
    assert view_at(0.10, depth_age_ms=490.0).obstacles
    assert not view_at(0.111, depth_age_ms=490.0).obstacles


def test_depth_age_with_delay_hides_at_t_plus_0_41():
    assert not view_at(0.29, delay=0.3, depth_age_ms=490.0).obstacles  # not displayed yet
    assert view_at(0.40, delay=0.3, depth_age_ms=490.0).obstacles
    assert not view_at(0.411, delay=0.3, depth_age_ms=490.0).obstacles


def test_link_age_drops_everything_after_half_a_second():
    view = view_at(0.51)
    assert (view.badge, view.obstacles, view.probe_state) == ("지형: 수신 없음", (), "STALE")


def test_size_mismatch_or_unknown_video_size_hides_outlines():
    assert not view_at(0.05, video=(848, 480)).obstacles
    assert not view_at(0.05, video=None).obstacles
    assert view_at(0.05).obstacles


def test_badges_per_status():
    assert view_at(0.05).badge == "지형 OK · 경로 1.23 m (카메라 기준)"
    assert view_at(0.05, distance_origin="rover_front").badge.endswith("(차체 앞 기준)")
    held = view_at(0.05, ground={"state": "HELD", "age_ms": 300.0,
                                 "camera_height_m": 0.5, "camera_tilt_deg": 20.0})
    assert held.badge == "지형: 바닥 유지 0.3 s" and held.obstacles
    assert view_at(0.05, status="NO_GROUND", reason="GROUND_EXPIRED").badge == "지형: 바닥 미검출"
    error = view_at(0.05, status="SENSOR_ERROR", reason="DEPTH_STALE")
    assert error.badge == "지형: 센서 오류(DEPTH_STALE)" and not error.obstacles
    assert view_at(0.05).note.startswith("정합 미검증")
    assert view_at(0.05, alignment_verified=True).note == "참고용 · 낮은 장애물/구덩이 미표시"


def test_no_data_yet_is_waiting():
    view = terrain_view(None, None, None, 5.0, 0.0, (1280, 720))
    assert (view.badge, view.probe_state) == ("지형: 수신 없음", "WAITING")


def test_real_socket_path_delivers_a_packet(receiver):
    import socket
    import time
    rx = receiver()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sender.sendto(payload(A, 3), ("127.0.0.1", rx.port))
        deadline = time.monotonic() + 2.0
        while rx.latest() is None and time.monotonic() < deadline:
            time.sleep(0.01)
    finally:
        sender.close()
    assert rx.latest().sequence == 3


def test_hidden_outlines_explain_why_in_the_note():
    unknown = view_at(0.05, video=None)
    assert not unknown.obstacles and unknown.note.startswith("윤곽 숨김: 영상 크기 미확인")
    other = view_at(0.05, video=(960, 540))
    assert not other.obstacles and other.note.startswith("윤곽 숨김: 영상 960×540")
    assert not view_at(0.05).note.startswith("윤곽 숨김")


def test_address_move_keeps_the_sequence_floor(receiver):
    import socket
    import time
    rx = receiver()
    first = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)   # address 1
    second = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # address 2

    def wait_for(predicate):
        deadline = time.monotonic() + 2.0
        while not predicate() and time.monotonic() < deadline:
            time.sleep(0.005)
        assert predicate()

    try:
        first.sendto(payload(A, 100), ("127.0.0.1", rx.port))
        wait_for(lambda: rx.latest() is not None and rx.latest().sequence == 100)
        second.sendto(payload(A, 99), ("127.0.0.1", rx.port))
        wait_for(lambda: rx.rejected_packet_count == 1)
        second.sendto(payload(A, 101), ("127.0.0.1", rx.port))
        wait_for(lambda: rx.latest().sequence == 101)
        first.sendto(payload(A, 102), ("127.0.0.1", rx.port))
        wait_for(lambda: rx.latest().sequence == 102)
        assert rx.rejected_packet_count == 1
    finally:
        first.close()
        second.close()


def test_nested_or_oversize_datagrams_are_invalid(receiver):
    rx = receiver()
    assert rx.ingest(b"[" * 600 + b"]" * 600, 0.0) is False
    assert rx.ingest(b" " * 1201, 0.0) is False
    assert rx.invalid_packet_count == 2


def test_video_loss_hides_outlines_but_keeps_the_badge():
    view = view_at(0.05, live=False)
    assert not view.obstacles
    assert view.badge.startswith("지형 OK")
    assert view.note.startswith("윤곽 숨김: 영상 끊김")


def test_empty_path_says_protruding_obstacles_only():
    view = view_at(0.05, zones={"left": None, "center": None, "right": None})
    assert view.badge == "지형 OK · 경로 돌출 장애물 없음 (카메라 기준)"


def test_snapshot_never_mixes_sessions(receiver):
    rx = receiver()
    feed(rx, [(0.0, A, 1), (0.1, B, 0)])
    latest, display, last_rx = rx.snapshot(0.2)
    assert latest.session_key == display.session_key == B and last_rx == 0.1
