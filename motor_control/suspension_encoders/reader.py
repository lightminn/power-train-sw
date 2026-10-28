"""Read the four suspension AS5048B channels and shape console telemetry fields.

Blocking I2C stays in the caller's worker thread (see
``powertrain_ros.chassis_telemetry.LatestPollWorker``); nothing here runs in
the 50 Hz chassis process.  Each channel carries its own monotonic sample time
and error so one missing sensor never hides the others, and a failed channel
is reported as absent instead of a fake zero.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
from collections.abc import Callable

from suspension_encoders.as5048b import (
    COUNTS_PER_TURN,
    RAD_PER_COUNT,
    As5048bReadout,
    LinuxI2cBus,
    read_readout,
)
from suspension_encoders.config import EncoderChannel, SuspensionEncoderConfig


FRAME_UNCALIBRATED = "UNCALIBRATED raw AS5048B angle (no zero offset), not urdf_joint"
FRAME_UNVERIFIED = "urdf_joint? zero captured, address mapping/sign unverified"
MAX_TEXT = 80

STATE_RAW = 0
STATE_ZERO_ONLY = 1
STATE_VERIFIED = 2


def wrap_pi(angle_rad: float) -> float:
    """Wrap to [-pi, pi)."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def joint_angle_rad(raw_counts: int, channel: EncoderChannel) -> float:
    """sign * wrap_pi(raw - zero); null zero means the chip's own 0 (uncalibrated)."""
    zero = 0 if channel.zero_counts is None else channel.zero_counts
    delta = (raw_counts - zero) % COUNTS_PER_TURN
    return channel.sign * wrap_pi(delta * RAD_PER_COUNT)


def calibration_state(channel: EncoderChannel) -> int:
    if channel.zero_counts is None:
        return STATE_RAW
    return STATE_VERIFIED if channel.verified else STATE_ZERO_ONLY


@dataclass(frozen=True)
class ChannelReading:
    channel: EncoderChannel
    sampled_monotonic_s: float
    readout: As5048bReadout | None
    joint_rad: float | None
    error: str | None

    @property
    def ok(self) -> bool:
        return self.error is None and self.joint_rad is not None


@dataclass(frozen=True)
class SuspensionSample:
    i2c_bus: int
    readings: tuple[ChannelReading, ...]


class SuspensionEncoderReader:
    def __init__(
        self,
        config: SuspensionEncoderConfig,
        *,
        bus=None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.config = config
        self.bus = LinuxI2cBus(config.i2c_bus) if bus is None else bus
        self._clock = clock

    def read_channel(self, channel: EncoderChannel) -> ChannelReading:
        # Stamp the START of the transfer: any delay inside or after the
        # blocking read then counts toward age instead of hiding it.
        sampled = self._clock()
        try:
            readout = read_readout(self.bus, channel.address)
        except (OSError, ValueError) as exc:
            return ChannelReading(
                channel, sampled, None, None, f"{type(exc).__name__}: {exc}",
            )
        invalid = readout.invalid_reason
        if invalid is not None:
            return ChannelReading(channel, sampled, readout, None, invalid)
        return ChannelReading(
            channel, sampled, readout, joint_angle_rad(readout.angle_counts, channel), None,
        )

    def read_once(self) -> SuspensionSample:
        return SuspensionSample(
            self.config.i2c_bus,
            tuple(self.read_channel(channel) for channel in self.config.channels),
        )

    def close(self) -> None:
        close = getattr(self.bus, "close", None)
        if close is not None:
            close()


def _source_text(bus: int, readings: list[ChannelReading]) -> str:
    addresses = ",".join(f"0x{reading.channel.address:02x}" for reading in readings)
    return f"AS5048B /dev/i2c-{bus} {addresses}"[:MAX_TEXT]


def _valid_text(value: object) -> bool:
    return (
        isinstance(value, str) and 1 <= len(value.strip()) <= MAX_TEXT
        and len(value) <= MAX_TEXT and not any(ord(c) < 32 for c in value)
    )


def telemetry_fields(
    sample: SuspensionSample | None,
    *,
    now_s: float,
    max_age_s: float,
    calibrated_frame: str = "urdf_joint",
    publish_unverified: bool = False,
) -> dict[str, object]:
    """Console fields for the latest poll; {} when no channel is usable.

    Only channels valid in the latest poll with ``0 <= age <= max_age_s`` are
    included, and (unless ``publish_unverified``) only verified channels with a
    captured zero.  ``suspension_age_ms`` is the age of the oldest included
    sample, so a group age never refreshes a stale channel.  The result is
    checked against the console contract; any violation drops the whole
    suspension group instead of letting the console reject the datagram.
    """
    if sample is None:
        return {}
    included = [
        reading for reading in sample.readings
        if reading.ok
        and (publish_unverified or calibration_state(reading.channel) == STATE_VERIFIED)
        and 0.0 <= now_s - reading.sampled_monotonic_s <= max_age_s
    ]
    if not included:
        return {}
    state = min(calibration_state(reading.channel) for reading in included)
    frame = {
        STATE_RAW: FRAME_UNCALIBRATED,
        STATE_ZERO_ONLY: FRAME_UNVERIFIED,
        STATE_VERIFIED: calibrated_frame,
    }[state]
    oldest_s = max(now_s - reading.sampled_monotonic_s for reading in included)
    fields = {
        "suspension_joints_rad": {
            reading.channel.joint: reading.joint_rad for reading in included
        },
        "suspension_age_ms": round(oldest_s * 1000.0, 1),
        "suspension_source": _source_text(sample.i2c_bus, included),
        "suspension_frame": frame,
    }
    angles_ok = all(
        isinstance(value, float) and math.isfinite(value) and abs(value) <= 2.0 * math.pi
        for value in fields["suspension_joints_rad"].values()
    )
    age = fields["suspension_age_ms"]
    if not (
        angles_ok and math.isfinite(age) and age >= 0.0
        and _valid_text(fields["suspension_source"]) and _valid_text(frame)
    ):
        return {}
    return fields
