from pathlib import Path
import threading
import time

from powertrain_ros import chassis_telemetry


def _worker_type():
    return getattr(chassis_telemetry, "LatestPollWorker", None)


def test_slow_poll_does_not_block_periodic_latest_cache_reads():
    worker_type = _worker_type()
    assert worker_type is not None, "latest-only poll worker is missing"
    poll_started = threading.Event()

    def slow_poll():
        poll_started.set()
        time.sleep(0.5)
        return {"state": "RUNNING"}

    worker = worker_type(slow_poll, period_s=1.0, name="slow-test")
    try:
        assert poll_started.wait(0.2)
        read_times = []
        for _ in range(6):
            worker.latest()
            read_times.append(time.monotonic())
            time.sleep(0.05)
        gaps = [right - left for left, right in zip(read_times, read_times[1:])]
        assert max(gaps) < 0.10
    finally:
        assert worker.close(join_timeout_s=1.0)


def test_poll_exception_is_cached_as_unavailable_and_worker_survives():
    worker_type = _worker_type()
    assert worker_type is not None, "latest-only poll worker is missing"
    attempts = 0

    def flaky_poll():
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise RuntimeError("poll exploded")
        return {"state": "RUNNING"}

    worker = worker_type(flaky_poll, period_s=0.01, name="flaky-test")
    try:
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            cached = worker.latest()
            if cached.value == {"state": "RUNNING"}:
                break
            time.sleep(0.005)
        assert attempts >= 2
        assert cached.value == {"state": "RUNNING"}
        assert cached.error is None
    finally:
        assert worker.close(join_timeout_s=1.0)


def test_sender_node_uses_workers_bounded_encoder_and_no_poll_timers():
    source = (
        Path(__file__).parents[1]
        / "powertrain_ros"
        / "chassis_telemetry_sender_node.py"
    ).read_text(encoding="utf-8")
    assert "LatestPollWorker" in source
    assert "encode_telemetry_payload" in source
    assert "create_timer(1.0, self._poll_l515)" not in source
    assert "create_timer(1.0, self._poll_observability)" not in source
    assert "join_timeout_s=1.0" in source


def test_encoder_accepts_frozen_status_client_mappings():
    # GatewayClient/ObservabilityClient freeze their payloads with
    # MappingProxyType; the encoder is the JSON boundary and must thaw them
    # (2026-07-18 on-robot crash loop: "mappingproxy is not JSON serializable").
    from types import MappingProxyType
    import json

    payload = {
        "schema_version": 1,
        "sequence": 0,
        "l515_ros_topic_rates_hz": MappingProxyType({"color": 30.0}),
        "nested": (MappingProxyType({"inner": 1}),),
    }
    decoded = json.loads(chassis_telemetry.encode_telemetry_payload(payload))
    assert decoded["l515_ros_topic_rates_hz"] == {"color": 30.0}
    assert decoded["nested"] == [{"inner": 1}]


def test_safety_state_component_mask_parser_and_fresh_payload_gate():
    parse_mask = getattr(chassis_telemetry, "parse_component_mask_state", None)
    payload_value = getattr(chassis_telemetry, "component_mask_payload_value", None)
    assert parse_mask is not None, "component mask state parser is missing"
    assert payload_value is not None, "component mask freshness helper is missing"
    mask = parse_mask(
        '{"component_mask":{"drive":true,"steer":false,'
        '"us100":true,"robot_arm":false}}'
    )

    assert mask == {
        "drive": True,
        "steer": False,
        "us100": True,
        "robot_arm": False,
    }
    assert payload_value(mask, updated_s=10.0, now_s=10.5) == mask
    assert payload_value(mask, updated_s=10.0, now_s=11.01) is None


def test_bounded_encoder_retains_component_mask_in_truncated_summary():
    mask = {
        "drive": False,
        "steer": True,
        "us100": False,
        "robot_arm": True,
    }
    raw = chassis_telemetry.encode_telemetry_payload({
        "schema_version": 1,
        "sequence": 21,
        "component_mask": mask,
        "wheel_statuses": [{"name": "w" * 5000}],
        "safety_detail": "s" * 5000,
        "unexpected_structure": {str(index): "x" * 500 for index in range(20)},
    })

    decoded = __import__("json").loads(raw)
    assert decoded["truncated"] is True
    assert decoded["component_mask"] == mask


def test_sender_subscribes_to_safety_state_and_mirrors_component_mask():
    source = (
        Path(__file__).parents[1]
        / "powertrain_ros"
        / "chassis_telemetry_sender_node.py"
    ).read_text(encoding="utf-8")

    assert '"/chassis/safety_state"' in source
    assert '"component_mask": component_mask' in source
