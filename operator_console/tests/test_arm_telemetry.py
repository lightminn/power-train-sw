import json
from pathlib import Path
import queue
import time

import pytest

from operator_console import arm_telemetry
from operator_console.arm_telemetry import parse_arm_telemetry, temperature_state


def _payload(**overrides):
    payload = {
        "schema_version": 1,
        "sequence": 21,
        "stamp_s": 123.5,
        "dynamixel": [
            {
                "id": 11,
                "position_raw": 2048,
                "position_deg": 0.0,
                "velocity": 7,
                "current": -12,
                "temperature_c": 34,
            },
            {
                "id": 12,
                "position_raw": 3072,
                "position_deg": 90.0,
                "velocity": -3,
                "current": 18,
                "temperature_c": 55,
            },
        ],
        "joints": {
            "names": ["joint_a", "joint_b"],
            "position_rad": [0.25, -0.5],
            "velocity": [0.1, -0.2],
        },
        "source_age_s": {
            "dynamixel": 0.1,
            "joints": 0.2,
            "detections": 0.3,
        },
        "truncated": False,
    }
    payload.update(overrides)
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def test_parse_arm_telemetry_round_trips_motors_joints_and_ages():
    snapshot = parse_arm_telemetry(_payload(), received_monotonic_s=10.0)

    assert snapshot.sequence == 21
    assert snapshot.dynamixel is not None
    assert len(snapshot.dynamixel) == 2
    assert snapshot.dynamixel[0].id == 11
    assert snapshot.dynamixel[0].position_raw == 2048
    assert snapshot.dynamixel[0].position_deg == 0.0
    assert snapshot.dynamixel[0].velocity == 7
    assert snapshot.dynamixel[0].current == -12
    assert snapshot.dynamixel[0].temperature_c == 34
    assert snapshot.joint_names == ("joint_a", "joint_b")
    assert snapshot.joint_position_rad == (0.25, -0.5)
    assert snapshot.joint_velocity == (0.1, -0.2)
    assert snapshot.dynamixel_age_s == 0.1
    assert snapshot.joints_age_s == 0.2
    assert snapshot.detections_age_s == 0.3
    assert snapshot.received_monotonic_s == 10.0


def test_null_sources_remain_explicitly_unavailable():
    snapshot = parse_arm_telemetry(
        _payload(dynamixel=None, joints=None, source_age_s={}),
        received_monotonic_s=10.0,
    )

    assert snapshot.dynamixel is None
    assert snapshot.joint_names == ()
    assert snapshot.joint_position_rad == ()
    assert snapshot.joint_velocity == ()
    assert snapshot.dynamixel_age_s is None
    assert snapshot.joints_age_s is None
    assert snapshot.detections_age_s is None


def test_more_than_eight_dynamixel_motors_is_rejected():
    motor = {
        "id": 11,
        "position_raw": 2048,
        "position_deg": 0.0,
        "velocity": 0,
        "current": 0,
        "temperature_c": 30,
    }

    with pytest.raises(ValueError):
        parse_arm_telemetry(_payload(dynamixel=[motor] * 9))


def test_mismatched_joint_array_lengths_are_rejected():
    joints = {
        "names": ["joint_a", "joint_b"],
        "position_rad": [0.25],
        "velocity": [0.1, -0.2],
    }

    with pytest.raises(ValueError):
        parse_arm_telemetry(_payload(joints=joints))


def test_unsupported_arm_telemetry_schema_is_rejected():
    with pytest.raises(ValueError):
        parse_arm_telemetry(_payload(schema_version=2))


def test_boolean_arm_telemetry_schema_is_rejected():
    with pytest.raises(ValueError):
        parse_arm_telemetry(_payload(schema_version=True))


def test_arm_telemetry_over_4096_bytes_is_rejected():
    with pytest.raises(ValueError):
        parse_arm_telemetry(b"{" + b"x" * 4096)


def test_receiver_reads_past_limit_to_detect_oversize_datagrams():
    source = (Path(__file__).parents[1] / "arm_telemetry.py").read_text(
        encoding="utf-8",
    )

    assert "recvfrom(4097)" in source


class FakeDatagramSocket:
    def __init__(self, payloads):
        self._payloads = queue.Queue()
        for payload in payloads:
            self._payloads.put(payload)
        self.closed = False

    def setsockopt(self, *_args):
        pass

    def bind(self, _address):
        pass

    def settimeout(self, _timeout_s):
        pass

    def recvfrom(self, _limit):
        try:
            payload = self._payloads.get(timeout=0.01)
        except queue.Empty as exc:
            raise arm_telemetry.socket.timeout() from exc
        return payload, ("robot", 5007)

    def close(self):
        self.closed = True


@pytest.mark.parametrize(
    "poison",
    (
        _payload(source_age_s={
            "dynamixel": 10**400,
            "joints": 0.2,
            "detections": 0.3,
        }),
        b"[" * 2000,
    ),
    ids=("overflow", "recursion"),
)
def test_receiver_survives_poison_packet_and_accepts_next_valid_packet(
    monkeypatch,
    poison,
):
    fake_socket = FakeDatagramSocket((poison, _payload()))
    monkeypatch.setattr(
        arm_telemetry.socket,
        "socket",
        lambda *_args: fake_socket,
    )
    receiver = arm_telemetry.LatestArmTelemetryReceiver(5007)
    try:
        deadline = time.monotonic() + 1.0
        while receiver.latest() is None and time.monotonic() < deadline:
            time.sleep(0.005)

        assert receiver.latest() is not None
        assert receiver.latest().sequence == 21
        assert receiver.invalid_packet_count == 1
        assert receiver._thread.is_alive()
    finally:
        receiver.close()


def test_truncated_flag_round_trips():
    snapshot = parse_arm_telemetry(_payload(truncated=True))

    assert snapshot.truncated is True


@pytest.mark.parametrize(
    ("temperature_c", "expected"),
    ((54, "NORMAL"), (55, "WARN"), (64, "WARN"), (65, "CRIT")),
)
def test_temperature_state_boundaries(temperature_c, expected):
    assert temperature_state(temperature_c) == expected


def test_arm_summary_covers_normal_unavailable_and_critical_temperature():
    summary = getattr(arm_telemetry, "arm_summary", None)
    assert summary is not None
    normal = parse_arm_telemetry(_payload(
        dynamixel=[
            {
                "id": 11,
                "position_raw": 2048,
                "position_deg": 0.0,
                "velocity": 0,
                "current": 0,
                "temperature_c": 45,
            },
            {
                "id": 12,
                "position_raw": 2048,
                "position_deg": 0.0,
                "velocity": 0,
                "current": 0,
                "temperature_c": 40,
            },
        ],
    ))
    critical = parse_arm_telemetry(_payload(
        dynamixel=[
            {
                "id": 12,
                "position_raw": 2048,
                "position_deg": 0.0,
                "velocity": 0,
                "current": 0,
                "temperature_c": 65,
            },
        ],
    ))

    assert summary(normal) == "모터 2 · 최고 45 ℃ 정상"
    assert summary(None) == "미수신(UNAVAILABLE)"
    assert summary(parse_arm_telemetry(_payload(dynamixel=None))) == (
        "미수신(UNAVAILABLE)"
    )
    assert summary(critical) == "모터 1 · 최고 ⚠ 65 ℃"


def test_arm_source_freshness_uses_age_and_fails_closed_when_missing():
    freshness = getattr(arm_telemetry, "arm_source_freshness", None)
    assert freshness is not None

    assert freshness(None) == "UNAVAILABLE"
    assert freshness(1.0) == "LIVE"
    assert freshness(1.01) == "STALE"


def test_arm_summary_does_not_report_stale_dynamixel_data_as_normal():
    snapshot = parse_arm_telemetry(_payload(
        source_age_s={
            "dynamixel": 999.0,
            "joints": 0.2,
            "detections": 0.3,
        },
    ))

    assert arm_telemetry.arm_summary(snapshot) == "모터 원천 지연(STALE)"


def test_arm_panel_summary_covers_unavailable_receive_stale_and_fresh():
    panel_summary = getattr(arm_telemetry, "arm_panel_summary", None)
    assert panel_summary is not None
    snapshot = parse_arm_telemetry(_payload())

    assert panel_summary(None, 999.0) == arm_telemetry.arm_summary(None)
    assert panel_summary(snapshot, 1.01) == "지연(STALE)"
    assert panel_summary(snapshot, 1.0) == arm_telemetry.arm_summary(snapshot)


def test_arm_panel_marks_each_stale_source_instead_of_udp_transport_live():
    import ast
    import math
    from types import SimpleNamespace

    source_path = Path(__file__).parents[1] / "app.py"
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    panel = next(
        item
        for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "ArmTelemetryPanel"
    )
    method = next(
        item
        for item in panel.body
        if isinstance(item, ast.FunctionDef) and item.name == "_refresh"
    )
    module = ast.Module(body=[method], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {
        "ArmTelemetrySnapshot": object,
        "arm_panel_summary": getattr(arm_telemetry, "arm_panel_summary", None),
        "arm_summary": arm_telemetry.arm_summary,
        "arm_source_freshness": getattr(
            arm_telemetry,
            "arm_source_freshness",
            lambda _age: "LIVE",
        ),
        "freshness_korean": lambda state: state,
        "temperature_state": temperature_state,
        "math": math,
        "time": SimpleNamespace(monotonic=lambda: 10.0),
    }
    exec(compile(module, str(source_path), "exec"), namespace)

    class Label:
        def __init__(self):
            self.text = "untouched"

        def set_text(self, text):
            self.text = text

    snapshot = parse_arm_telemetry(
        _payload(source_age_s={
            "dynamixel": 999.0,
            "joints": 999.0,
            "detections": 999.0,
        }),
        received_monotonic_s=9.5,
    )
    labels = {
        key: Label()
        for key in ("link", "motors", "joints", "detections")
    }
    node = SimpleNamespace(
        _receiver=SimpleNamespace(latest=lambda: snapshot),
        _summary=Label(),
        _labels=labels,
        _port=5007,
        _report_temperature=lambda _snapshot: None,
    )

    namespace["_refresh"](node)

    assert "LIVE" in labels["link"].text
    assert "STALE" in labels["motors"].text
    assert "STALE" in labels["joints"].text
    assert "STALE" in labels["detections"].text
