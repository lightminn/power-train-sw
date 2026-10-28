"""Suspension encoder config: bus, address->joint mapping, sign and zero.

The tracked file ``<repo>/config/suspension_encoders.json`` is the single
source for both ``scripts/robot-start`` (device selection) and the telemetry
container (``/workspace/config/...``); there is deliberately no environment
override that one side could see and the other not.  Zero offsets are
software-only raw counts captured at the CAD/URDF reference pose; sensor OTP is
never written.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import json
import math
import os
from pathlib import Path

from suspension_encoders.as5048b import COUNTS_PER_TURN


DEFAULT_CONFIG_PATH = Path(__file__).resolve().parents[2] / "config" / "suspension_encoders.json"
JOINTS = ("ml_rocker_joint", "mr_rocker_joint", "bl_bogie_joint", "br_bogie_joint")


class ConfigError(ValueError):
    pass


@dataclass(frozen=True)
class EncoderChannel:
    joint: str
    address: int
    sign: int
    zero_counts: int | None
    # Address->joint mapping, sign AND zero pose checked on hardware.  Only
    # verified channels with a zero reach the console unless publish_unverified.
    verified: bool


@dataclass(frozen=True)
class SuspensionEncoderConfig:
    enabled: bool
    i2c_bus: int
    read_hz: float
    max_age_s: float
    calibrated_frame: str
    channels: tuple[EncoderChannel, ...]
    # Bench-only opt-in: also send raw/unverified angles (labelled in
    # suspension_frame).  The console 3D view applies any received angle, so
    # the default keeps unverified channels off the wire.
    publish_unverified: bool = False
    source_path: Path | None = None
    extra: dict | None = None  # unknown top-level keys preserved on rewrite

    def with_zero_counts(self, zeros: dict[str, int]) -> "SuspensionEncoderConfig":
        """New zeros always clear ``verified``: a re-captured zero must be
        re-checked at a known pose before it reaches the console again."""
        channels = tuple(
            replace(
                channel,
                zero_counts=int(zeros[channel.joint]) % COUNTS_PER_TURN,
                verified=False,
            )
            if channel.joint in zeros else channel
            for channel in self.channels
        )
        return replace(self, channels=channels)

    def to_json_dict(self) -> dict:
        data = {
            "schema_version": 1,
            "enabled": self.enabled,
            "i2c_bus": self.i2c_bus,
            "read_hz": self.read_hz,
            "max_age_s": self.max_age_s,
            "calibrated_frame": self.calibrated_frame,
            "publish_unverified": self.publish_unverified,
            "channels": [
                {
                    "joint": channel.joint,
                    "address": f"0x{channel.address:02x}",
                    "sign": channel.sign,
                    "verified": channel.verified,
                    "zero_counts": channel.zero_counts,
                }
                for channel in self.channels
            ],
        }
        for key, value in (self.extra or {}).items():
            data.setdefault(key, value)
        data["channels"] = data.pop("channels")  # keep notes above the channel list
        return data


def _address(value: object) -> int:
    if isinstance(value, bool):
        raise ConfigError("address must be an int or hex string")
    if isinstance(value, int):
        address = value
    elif isinstance(value, str):
        try:
            address = int(value, 0)
        except ValueError as exc:
            raise ConfigError(f"invalid address {value!r}") from exc
    else:
        raise ConfigError("address must be an int or hex string")
    if not 0x03 <= address <= 0x77:
        raise ConfigError(f"address {address:#x} outside 7-bit range")
    return address


def _positive(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{name} must be a number")
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ConfigError(f"{name} must be finite and positive")
    return number


def _frame_text(value: object) -> str:
    if not isinstance(value, str) or not 1 <= len(value.strip()) <= 80 or any(ord(c) < 32 for c in value):
        raise ConfigError("calibrated_frame must be 1-80 printable characters")
    return value.strip()


_KNOWN_KEYS = {
    "schema_version", "enabled", "i2c_bus", "read_hz", "max_age_s",
    "calibrated_frame", "channels", "publish_unverified",
}


def parse_config(data: object, *, source_path: Path | None = None) -> SuspensionEncoderConfig:
    if not isinstance(data, dict):
        raise ConfigError("config must be a JSON object")
    if data.get("schema_version") != 1:
        raise ConfigError("schema_version must be 1")
    enabled = data.get("enabled")
    if not isinstance(enabled, bool):
        raise ConfigError("enabled must be bool")
    bus = data.get("i2c_bus")
    if isinstance(bus, bool) or not isinstance(bus, int) or bus < 0:
        raise ConfigError("i2c_bus must be a non-negative int")
    read_hz = _positive(data.get("read_hz"), "read_hz")
    if not 0.5 <= read_hz <= 200.0:
        raise ConfigError("read_hz must be within 0.5..200")
    max_age_s = _positive(data.get("max_age_s"), "max_age_s")
    if max_age_s > 1.0:
        raise ConfigError("max_age_s must be <= 1.0 (console drops older joints)")
    frame = _frame_text(data.get("calibrated_frame", "urdf_joint"))
    publish_unverified = data.get("publish_unverified", False)
    if not isinstance(publish_unverified, bool):
        raise ConfigError("publish_unverified must be bool")
    raw_channels = data.get("channels")
    if not isinstance(raw_channels, list) or not raw_channels:
        raise ConfigError("channels must be a non-empty list")
    channels = []
    for raw in raw_channels:
        if not isinstance(raw, dict):
            raise ConfigError("channel must be an object")
        joint = raw.get("joint")
        if joint not in JOINTS:
            raise ConfigError(f"unknown joint {joint!r}")
        sign = raw.get("sign")
        if isinstance(sign, bool) or sign not in (1, -1):
            raise ConfigError(f"{joint}: sign must be 1 or -1")
        zero = raw.get("zero_counts")
        if zero is not None and (
            isinstance(zero, bool) or not isinstance(zero, int)
            or not 0 <= zero < COUNTS_PER_TURN
        ):
            raise ConfigError(f"{joint}: zero_counts must be null or 0..16383")
        verified = raw.get("verified", False)
        if not isinstance(verified, bool):
            raise ConfigError(f"{joint}: verified must be bool")
        channels.append(EncoderChannel(joint, _address(raw.get("address")), sign, zero, verified))
    joints = [channel.joint for channel in channels]
    addresses = [channel.address for channel in channels]
    if len(set(joints)) != len(joints):
        raise ConfigError("duplicate joint")
    if len(set(addresses)) != len(addresses):
        raise ConfigError("duplicate address")
    extra = {key: value for key, value in data.items() if key not in _KNOWN_KEYS}
    return SuspensionEncoderConfig(
        enabled=enabled,
        i2c_bus=bus,
        read_hz=read_hz,
        max_age_s=max_age_s,
        calibrated_frame=frame,
        channels=tuple(channels),
        publish_unverified=publish_unverified,
        source_path=source_path,
        extra=extra,
    )


def resolve_config_path(path: str | os.PathLike | None = None) -> Path:
    return Path(path) if path else DEFAULT_CONFIG_PATH


def load_config(path: str | os.PathLike | None = None) -> SuspensionEncoderConfig:
    resolved = resolve_config_path(path)
    with resolved.open(encoding="utf-8") as stream:
        data = json.load(stream)
    return parse_config(data, source_path=resolved)


def write_config(config: SuspensionEncoderConfig, path: str | os.PathLike) -> Path:
    """Atomically replace ``path`` with ``config`` (validated round-trip)."""
    target = Path(path)
    data = config.to_json_dict()
    parse_config(data)
    text = json.dumps(data, indent=2, ensure_ascii=False) + "\n"
    tmp = target.with_name(f".{target.name}.tmp-{os.getpid()}")
    try:
        with tmp.open("w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        if target.exists():
            os.chmod(tmp, target.stat().st_mode & 0o777)
        os.replace(tmp, target)
    finally:
        if tmp.exists():
            tmp.unlink()
    return target
