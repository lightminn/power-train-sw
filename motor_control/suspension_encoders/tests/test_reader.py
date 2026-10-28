import json
import math

import pytest

from suspension_encoders.config import DEFAULT_CONFIG_PATH, load_config, parse_config
from suspension_encoders.reader import (
    FRAME_UNCALIBRATED,
    FRAME_UNVERIFIED,
    SuspensionEncoderReader,
    joint_angle_rad,
    telemetry_fields,
)
from suspension_encoders.tests.fake_bus import FakeAs5048bBus


def _config(**channel_overrides):
    channels = [
        {"joint": "ml_rocker_joint", "address": "0x40", "sign": 1, "verified": False, "zero_counts": None},
        {"joint": "mr_rocker_joint", "address": "0x41", "sign": -1, "verified": False, "zero_counts": None},
        {"joint": "bl_bogie_joint", "address": "0x42", "sign": 1, "verified": False, "zero_counts": None},
        {"joint": "br_bogie_joint", "address": "0x43", "sign": -1, "verified": False, "zero_counts": None},
    ]
    for channel in channels:
        channel.update(channel_overrides.get(channel["joint"], {}))
    return parse_config({
        "schema_version": 1, "enabled": True, "i2c_bus": 7, "read_hz": 20.0,
        "max_age_s": 0.5, "calibrated_frame": "urdf_joint", "channels": channels,
    })


class Clock:
    def __init__(self, now=100.0):
        self.now = now

    def __call__(self):
        return self.now


def console_accepts(payload):
    """Mirror of operator_console.telemetry._suspension_joints / _optional_source_text."""
    raw = payload.get("suspension_joints_rad", {})
    allowed = {"ml_rocker_joint", "mr_rocker_joint", "bl_bogie_joint", "br_bogie_joint"}
    assert isinstance(raw, dict) and set(raw) <= allowed
    for value in raw.values():
        assert not isinstance(value, bool) and isinstance(value, (int, float))
        assert math.isfinite(value) and abs(value) <= math.tau
    age = payload.get("suspension_age_ms")
    if age is not None:
        assert not isinstance(age, bool) and math.isfinite(age) and age >= 0
    for name in ("suspension_source", "suspension_frame"):
        value = payload.get(name)
        if value is not None:
            assert isinstance(value, str) and value.strip() and len(value) <= 80
            assert not any(ord(c) < 32 for c in value)
    json.dumps(payload, allow_nan=False)


def test_tracked_default_config_is_valid_and_uncalibrated():
    config = load_config(DEFAULT_CONFIG_PATH)
    assert config.i2c_bus == 7
    assert [hex(c.address) for c in config.channels] == ["0x40", "0x41", "0x42", "0x43"]
    assert all(c.zero_counts is None and not c.verified for c in config.channels)
    # CAD-derived signs from the Isaac archive encoder config.
    assert [c.sign for c in config.channels] == [1, -1, 1, -1]


def test_config_has_no_environment_override(monkeypatch):
    monkeypatch.setenv("POWERTRAIN_SUSPENSION_CONFIG", "/nonexistent.json")
    assert load_config().source_path == DEFAULT_CONFIG_PATH


@pytest.mark.parametrize("raw, zero, sign, expected_counts", [
    (100, 16300, 1, 184),         # wraps through 0
    (16300, 100, 1, -184),
    (100, 16300, -1, -184),       # sign flips the wrapped delta
    (5000, 5000, -1, 0),
    (8192 + 50, 50, 1, -8192),    # exactly half a turn maps to -pi
])
def test_joint_angle_applies_zero_sign_and_wrap(raw, zero, sign, expected_counts):
    channel = _config().channels[0]
    channel = type(channel)(channel.joint, channel.address, sign, zero, False)
    assert joint_angle_rad(raw, channel) == pytest.approx(expected_counts * 2 * math.pi / 16384)


def test_uncalibrated_channel_reports_wrapped_raw_angle():
    channel = _config().channels[0]
    assert joint_angle_rad(16000, channel) == pytest.approx((16000 - 16384) * 2 * math.pi / 16384)


def test_one_missing_sensor_does_not_hide_the_others():
    bus = FakeAs5048bBus({0x40: 10, 0x41: 20, 0x43: 40})
    reader = SuspensionEncoderReader(_config(), bus=bus, clock=Clock())
    sample = reader.read_once()
    assert [r.ok for r in sample.readings] == [True, True, False, True]
    assert "OSError" in sample.readings[2].error
    assert bus.reads[0] == (0x40, 0xFA, 6)


def test_cof_channel_is_invalid_and_omitted_from_telemetry():
    bus = FakeAs5048bBus({0x40: 10, 0x41: 20, 0x42: 30, 0x43: 40})
    bus.set_angle(0x41, 20, diag=0x03)
    clock = Clock()
    sample = SuspensionEncoderReader(_config(), bus=bus, clock=clock).read_once()
    fields = telemetry_fields(sample, now_s=clock.now, max_age_s=0.5, publish_unverified=True)
    assert set(fields["suspension_joints_rad"]) == {"ml_rocker_joint", "bl_bogie_joint", "br_bogie_joint"}
    console_accepts(fields)


def test_no_usable_channel_means_no_suspension_keys_at_all():
    clock = Clock()
    sample = SuspensionEncoderReader(_config(), bus=FakeAs5048bBus(), clock=clock).read_once()
    assert telemetry_fields(sample, now_s=clock.now, max_age_s=0.5, publish_unverified=True) == {}
    assert telemetry_fields(None, now_s=clock.now, max_age_s=0.5) == {}


def test_stale_channels_are_dropped_and_age_is_oldest_included():
    bus = FakeAs5048bBus({0x40: 10, 0x41: 20, 0x42: 30, 0x43: 40})
    clock = Clock(100.0)
    reader = SuspensionEncoderReader(_config(), bus=bus, clock=clock)
    readings = []
    for index, channel in enumerate(reader.config.channels):
        clock.now = 100.0 + 0.1 * index
        readings.append(reader.read_channel(channel))
    from suspension_encoders.reader import SuspensionSample
    sample = SuspensionSample(7, tuple(readings))
    fields = telemetry_fields(sample, now_s=100.55, max_age_s=0.5, publish_unverified=True)
    # ml sampled at 100.0 (0.55 s old) is dropped; oldest kept is mr at 100.1.
    assert set(fields["suspension_joints_rad"]) == {"mr_rocker_joint", "bl_bogie_joint", "br_bogie_joint"}
    assert fields["suspension_age_ms"] == pytest.approx(450.0)
    assert fields["suspension_source"] == "AS5048B /dev/i2c-7 0x41,0x42,0x43"
    console_accepts(fields)


def test_frame_label_tracks_the_least_calibrated_included_channel():
    bus = FakeAs5048bBus({0x40: 10, 0x41: 20, 0x42: 30, 0x43: 40})
    clock = Clock()

    def frame(config):
        sample = SuspensionEncoderReader(config, bus=bus, clock=clock).read_once()
        fields = telemetry_fields(sample, now_s=clock.now, max_age_s=0.5,
                                  calibrated_frame=config.calibrated_frame,
                                  publish_unverified=True)
        console_accepts(fields)
        return fields["suspension_frame"]

    all_zero = {j: {"zero_counts": 5} for j in ("ml_rocker_joint", "mr_rocker_joint", "bl_bogie_joint", "br_bogie_joint")}
    assert frame(_config()) == FRAME_UNCALIBRATED
    assert frame(_config(**all_zero)) == FRAME_UNVERIFIED
    verified = {j: {"zero_counts": 5, "verified": True} for j in all_zero}
    assert frame(_config(**verified)) == "urdf_joint"
    mixed = dict(verified, br_bogie_joint={"zero_counts": None, "verified": True})
    assert frame(_config(**mixed)) == FRAME_UNCALIBRATED
    # A missing uncalibrated channel no longer downgrades the label.
    bus.remove(0x43)
    assert frame(_config(**mixed)) == "urdf_joint"


def test_calibrated_values_match_expected_joint_angles():
    zeros = {"ml_rocker_joint": {"zero_counts": 1000, "verified": True},
             "mr_rocker_joint": {"zero_counts": 2000, "verified": True},
             "bl_bogie_joint": {"zero_counts": 16380, "verified": True},
             "br_bogie_joint": {"zero_counts": 0, "verified": True}}
    bus = FakeAs5048bBus({0x40: 1000 + 455, 0x41: 2000 + 455, 0x42: 451, 0x43: 16384 - 455})
    clock = Clock()
    sample = SuspensionEncoderReader(_config(**zeros), bus=bus, clock=clock).read_once()
    joints = telemetry_fields(sample, now_s=clock.now, max_age_s=0.5)["suspension_joints_rad"]
    ten_deg = 455 * 2 * math.pi / 16384
    assert joints["ml_rocker_joint"] == pytest.approx(ten_deg)
    assert joints["mr_rocker_joint"] == pytest.approx(-ten_deg)
    assert joints["bl_bogie_joint"] == pytest.approx(ten_deg)
    assert joints["br_bogie_joint"] == pytest.approx(ten_deg)


@pytest.mark.parametrize("patch, message", [
    ({"schema_version": 2}, "schema_version"),
    ({"i2c_bus": "7"}, "i2c_bus"),
    ({"read_hz": 0}, "read_hz"),
    ({"read_hz": 1e-309}, "read_hz"),
    ({"read_hz": 0.2}, "read_hz"),
    ({"max_age_s": 1.5}, "max_age_s"),
    ({"calibrated_frame": "x" * 81}, "calibrated_frame"),
])
def test_config_validation_rejects_bad_top_level(patch, message):
    data = _config().to_json_dict()
    data.update(patch)
    with pytest.raises(ValueError, match=message):
        parse_config(data)


@pytest.mark.parametrize("patch, message", [
    ({"joint": "front_wheel"}, "unknown joint"),
    ({"sign": 0}, "sign"),
    ({"sign": True}, "sign"),
    ({"zero_counts": 16384}, "zero_counts"),
    ({"address": "0x41"}, "duplicate address"),
    ({"address": "0x80"}, "outside"),
])
def test_config_validation_rejects_bad_channels(patch, message):
    data = _config().to_json_dict()
    data["channels"][0].update(patch)
    with pytest.raises(ValueError, match=message):
        parse_config(data)


def test_default_sends_only_verified_channels_with_a_zero():
    bus = FakeAs5048bBus({0x40: 10, 0x41: 20, 0x42: 30, 0x43: 40})
    clock = Clock()
    config = _config(
        ml_rocker_joint={"zero_counts": 10, "verified": True},
        mr_rocker_joint={"zero_counts": 20, "verified": False},
        bl_bogie_joint={"zero_counts": None, "verified": True},
    )
    sample = SuspensionEncoderReader(config, bus=bus, clock=clock).read_once()
    fields = telemetry_fields(sample, now_s=clock.now, max_age_s=0.5)
    assert fields["suspension_joints_rad"] == {"ml_rocker_joint": 0.0}
    assert fields["suspension_frame"] == "urdf_joint"
    console_accepts(fields)
    # Nothing verified -> nothing on the wire, even though all sensors answer.
    sample = SuspensionEncoderReader(_config(), bus=bus, clock=clock).read_once()
    assert telemetry_fields(sample, now_s=clock.now, max_age_s=0.5) == {}


def test_sample_newer_than_now_is_never_sent_with_negative_age():
    bus = FakeAs5048bBus({0x40: 10, 0x41: 20, 0x42: 30, 0x43: 40})
    clock = Clock(100.0)
    sample = SuspensionEncoderReader(_config(), bus=bus, clock=clock).read_once()
    assert telemetry_fields(sample, now_s=99.9999, max_age_s=0.5, publish_unverified=True) == {}


def test_contract_violation_drops_the_group_not_the_datagram():
    bus = FakeAs5048bBus({0x40: 10})
    clock = Clock()
    sample = SuspensionEncoderReader(_config(), bus=bus, clock=clock).read_once()
    assert telemetry_fields(sample, now_s=clock.now, max_age_s=0.5, publish_unverified=True,
                            calibrated_frame="x") != {}
    verified = _config(ml_rocker_joint={"zero_counts": 0, "verified": True})
    sample = SuspensionEncoderReader(verified, bus=bus, clock=clock).read_once()
    assert telemetry_fields(sample, now_s=clock.now, max_age_s=0.5, calibrated_frame="bad\nframe") == {}
    assert telemetry_fields(sample, now_s=clock.now, max_age_s=0.5, calibrated_frame=" " * 3) == {}


def test_sample_time_is_the_start_of_the_transfer():
    class SlowBus(FakeAs5048bBus):
        def read_registers(self, address, register, length):
            clock.now += 2.0  # transfer (or thread stall) takes 2 s
            return super().read_registers(address, register, length)

    clock = Clock(100.0)
    bus = SlowBus({0x40: 10})
    reading = SuspensionEncoderReader(_config(), bus=bus, clock=clock).read_channel(_config().channels[0])
    assert reading.sampled_monotonic_s == 100.0
    assert clock.now == 102.0
