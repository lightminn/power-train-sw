import json
import sys

from scripts.recv_yolo3d import CoordPacketTracker, parse_args, parse_coord_packet


def packet(**changes):
    value = {
        "session_id": "session-a",
        "frame": 7,
        "t": 1.25,
        "w": 1280,
        "h": 720,
        "dets": [
            {
                "cls": "person",
                "conf": 0.8,
                "box": [10, 20, 110, 220],
                "xyz": [0.1, -0.2, 3.0],
                "d": 3.01,
                "az": 1.9,
                "el": 3.8,
            }
        ],
    }
    value.update(changes)
    return value


def encoded(**changes):
    return json.dumps(packet(**changes)).encode()


def test_boundary_rejects_nonobject_and_detection_with_missing_fields():
    assert parse_coord_packet(b"[]") is None
    assert parse_coord_packet(encoded(dets=[{}])) is None


def test_boundary_accepts_only_complete_finite_coordinate_schema():
    assert parse_coord_packet(encoded()) == packet()
    invalid = packet()
    invalid["dets"][0]["xyz"][2] = float("inf")
    assert parse_coord_packet(json.dumps(invalid).encode()) is None


def test_tracker_accepts_only_monotonic_sequence_per_live_session():
    tracker = CoordPacketTracker()

    assert tracker.update(packet(frame=7)) is True
    assert tracker.update(packet(frame=6)) is False
    assert tracker.update(packet(frame=7)) is False
    assert tracker.update(packet(frame=8)) is True


def test_tracker_does_not_roll_back_to_a_retired_session():
    tracker = CoordPacketTracker()

    assert tracker.update(packet(session_id="old", frame=100)) is True
    assert tracker.update(packet(session_id="new", frame=0)) is True
    assert tracker.update(packet(session_id="old", frame=99)) is False
    assert tracker.latest == packet(session_id="new", frame=0)


def test_tracker_retired_session_memory_is_finite():
    tracker = CoordPacketTracker()

    for index in range(100):
        assert tracker.update(packet(session_id=f"session-{index}", frame=0))

    assert len(tracker._retired_session_ids) == tracker.MAX_RETIRED_SESSIONS


def test_receiver_default_geometry_matches_serial_locked_l515_sender(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["recv_yolo3d.py"])

    args = parse_args()

    assert (args.width, args.height) == (1280, 720)
