"""AS5048B suspension fields in the chassis datagram (ROS-free parts)."""
from __future__ import annotations

import json
from pathlib import Path
import time

from powertrain_ros import chassis_telemetry
from powertrain_ros.chassis_telemetry import SuspensionTelemetry


REPO = Path(__file__).resolve().parents[4]
DEFAULT_CONFIG = REPO / "config" / "suspension_encoders.json"


class _FakeBus:
    def __init__(self, devices):
        self.devices = devices
        self.closed = False

    def read_registers(self, address, register, length):
        if address not in self.devices:
            raise OSError(121, "Remote I/O error")
        counts = self.devices[address]
        return bytes([100, 0x01, 0x3E, 0x20, (counts >> 6) & 0xFF, counts & 0x3F])

    def close(self):
        self.closed = True


def _write_config(tmp_path, **changes):
    data = json.loads(DEFAULT_CONFIG.read_text(encoding="utf-8"))
    data.update(changes)
    path = tmp_path / "suspension.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


def _factory(bus):
    from suspension_encoders.reader import SuspensionEncoderReader

    return lambda config: SuspensionEncoderReader(config, bus=bus)


def _wait_fields(telemetry, timeout_s=1.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        fields = telemetry.fields()
        if fields:
            return fields
        time.sleep(0.01)
    return {}


def test_worker_publishes_fresh_valid_channels_only(tmp_path):
    bus = _FakeBus({0x40: 100, 0x41: 200, 0x43: 400})
    path = _write_config(tmp_path, publish_unverified=True)
    telemetry = SuspensionTelemetry(str(path), reader_factory=_factory(bus))
    try:
        assert telemetry.active
        fields = _wait_fields(telemetry)
        assert set(fields["suspension_joints_rad"]) == {
            "ml_rocker_joint", "mr_rocker_joint", "br_bogie_joint",
        }
        assert 0.0 <= fields["suspension_age_ms"] <= 500.0
        assert fields["suspension_frame"].startswith("UNCALIBRATED")
        assert fields["suspension_source"] == "AS5048B /dev/i2c-7 0x40,0x41,0x43"
        assert any("bl_bogie_joint" in error for error in telemetry.latest_errors())
        decoded = json.loads(chassis_telemetry.encode_telemetry_payload(
            {"schema_version": 1, "sequence": 0, **fields}
        ))
        assert decoded["suspension_joints_rad"] == fields["suspension_joints_rad"]
    finally:
        assert telemetry.close(join_timeout_s=1.0)
    assert bus.closed


def test_tracked_default_config_sends_nothing_until_channels_are_verified(tmp_path):
    bus = _FakeBus({0x40: 100, 0x41: 200, 0x42: 300, 0x43: 400})
    telemetry = SuspensionTelemetry(str(DEFAULT_CONFIG), reader_factory=_factory(bus))
    try:
        assert telemetry.active
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            assert telemetry.fields() == {}
            time.sleep(0.02)
        assert telemetry.latest_errors() == []
    finally:
        assert telemetry.close(join_timeout_s=1.0)


def test_stale_cache_drops_all_suspension_keys(tmp_path):
    bus = _FakeBus({0x40: 100, 0x41: 200, 0x42: 300, 0x43: 400})
    path = _write_config(tmp_path, publish_unverified=True)
    telemetry = SuspensionTelemetry(str(path), reader_factory=_factory(bus))
    try:
        assert _wait_fields(telemetry)
    finally:
        assert telemetry.close(join_timeout_s=1.0)
    # Worker stopped: one second later nothing is fresh, so nothing is sent.
    assert telemetry.fields(time.monotonic() + 1.0) == {}


def test_disabled_missing_or_invalid_config_never_crashes_or_sends(tmp_path):
    disabled = SuspensionTelemetry(str(_write_config(tmp_path, enabled=False)))
    assert not disabled.active and disabled.status.startswith("disabled")
    assert disabled.fields(time.monotonic()) == {}
    assert disabled.close()

    missing = SuspensionTelemetry(str(tmp_path / "absent.json"))
    assert not missing.active and "FileNotFoundError" in missing.status
    assert missing.fields(time.monotonic()) == {}

    bad = SuspensionTelemetry(str(_write_config(tmp_path, i2c_bus="seven")))
    assert not bad.active and "ConfigError" in bad.status


def test_absent_i2c_device_is_reported_per_channel_not_raised(tmp_path):
    path = _write_config(tmp_path, i2c_bus=250)  # /dev/i2c-250 does not exist
    telemetry = SuspensionTelemetry(str(path))
    try:
        deadline = time.monotonic() + 1.0
        errors = []
        while time.monotonic() < deadline and not errors:
            errors = telemetry.latest_errors()
            time.sleep(0.01)
        assert errors and all("FileNotFoundError" in error for error in errors)
        assert telemetry.fields(time.monotonic()) == {}
    finally:
        assert telemetry.close(join_timeout_s=1.0)


def test_truncated_summary_keeps_suspension_fields():
    fields = {
        "suspension_joints_rad": {"ml_rocker_joint": 0.1},
        "suspension_age_ms": 12.0,
        "suspension_source": "AS5048B /dev/i2c-7 0x40",
        "suspension_frame": "urdf_joint",
    }
    raw = chassis_telemetry.encode_telemetry_payload({
        "schema_version": 1,
        "sequence": 3,
        **fields,
        "wheel_statuses": [{"name": "w" * 5000}],
        "safety_detail": "s" * 5000,
        "unexpected_structure": {str(index): "x" * 500 for index in range(20)},
    })
    decoded = json.loads(raw)
    assert decoded["truncated"] is True
    for key, value in fields.items():
        assert decoded[key] == value


def test_sender_merges_suspension_fields_and_closes_worker():
    source = (
        Path(__file__).parents[1] / "powertrain_ros" / "chassis_telemetry_sender_node.py"
    ).read_text(encoding="utf-8")
    # fields() reads the cache before the clock (no negative ages).
    assert "payload.update(self._suspension.fields())" in source
    assert "self._suspension.close(join_timeout_s=1.0)" in source
    assert '"suspension_config"' in source


def test_reader_or_worker_construction_failure_is_isolated(tmp_path):
    def exploding_factory(config):
        raise PermissionError("no /dev/i2c-7 access")

    telemetry = SuspensionTelemetry(str(_write_config(tmp_path)), reader_factory=exploding_factory)
    assert not telemetry.active and "PermissionError" in telemetry.status
    assert telemetry.fields() == {}
    assert telemetry.close()
