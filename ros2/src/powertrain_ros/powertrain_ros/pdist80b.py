"""PDIST80B read-only RS485 packet codec and BMS flag definitions.

PDIST80B is *not* Modbus.  Its RS485 default is 57600 bps, device ID 1,
and it responds to the MDROBOT PID request packet described in the V1.7
manual.  PID 238 flag meanings follow 매뉴얼 V1.7 p.16.  This module
intentionally contains no power/relay write command.
"""
from __future__ import annotations

from dataclasses import dataclass


PDIST_MID = 186
OPERATOR_MID = 172
PID_REQUEST_DATA = 4
PID_BMS_MONITOR = 238

BATTERY_FLAG_LABELS = (
    "수동 충전기 결합",
    "자동 충전기 결합",
    "과전압 보호",
    "저전압 보호",
    "충전 과온 보호",
    "충전 저온 보호",
    "방전 과온 보호",
    "방전 저온 보호",
)
PROTECTION_FLAG_LABELS = (
    "충전 과전류 보호",
    "방전 과전류 보호",
    "단락 보호",
    "단락 보호",
    "예약",
    "외부 제어",
    "충전 플래그",
)
BATTERY_FAULT_MASK = 0xFC
PROTECTION_FAULT_MASK = 0x0F


def _set_flag_labels(flags: int, labels: tuple[str, ...]) -> tuple[str, ...]:
    return tuple(
        label for bit, label in enumerate(labels) if flags & (1 << bit)
    )


def battery_flag_labels(flags: int) -> tuple[str, ...]:
    """Return every set D5 bit label in ascending bit order."""
    return _set_flag_labels(flags, BATTERY_FLAG_LABELS)


def protection_flag_labels(flags: int) -> tuple[str, ...]:
    """Return every set D6 bit label in ascending bit order."""
    return _set_flag_labels(flags, PROTECTION_FLAG_LABELS)


def fault_reasons(
    battery_flags: int,
    protection_flags: int,
) -> tuple[str, ...]:
    """Return unique, set fault labels in D5-then-D6 stable bit order."""
    labels = (
        battery_flag_labels(battery_flags & BATTERY_FAULT_MASK)
        + protection_flag_labels(protection_flags & PROTECTION_FAULT_MASK)
    )
    reasons: list[str] = []
    for label in labels:
        if label not in reasons:
            reasons.append(label)
    return tuple(reasons)


def checksum(packet_without_checksum: bytes) -> int:
    """MDROBOT packet checksum: all bytes including checksum sum to zero."""
    return (-sum(packet_without_checksum)) & 0xFF


def bms_monitor_request(device_id: int = 1) -> bytes:
    if not 0 <= device_id <= 253:
        raise ValueError("device_id must be within 0..253")
    body = bytes((PDIST_MID, OPERATOR_MID, device_id, PID_REQUEST_DATA, 1, PID_BMS_MONITOR))
    return body + bytes((checksum(body),))


@dataclass(frozen=True)
class Pdist80bStatus:
    voltage_v: float | None
    discharge_current_a: float | None
    soc_percent: int | None
    battery_flags: int
    protection_flags: int
    charge_current_a: float | None


def _u16_le(data: bytes, offset: int) -> int:
    return data[offset] | data[offset + 1] << 8


def _i16_le(data: bytes, offset: int) -> int:
    value = _u16_le(data, offset)
    return value - 0x10000 if value & 0x8000 else value


def _measurement_i16(data: bytes, offset: int) -> int | None:
    """Decode a signed measurement while preserving PDIST missing sentinels."""
    raw = _u16_le(data, offset)
    if raw in (0xFFFF, 0xFFFD):
        return None
    return raw - 0x10000 if raw & 0x8000 else raw


def parse_bms_monitor_response(packet: bytes, device_id: int = 1) -> Pdist80bStatus:
    """Validate and parse a PID 238 (12 data-byte) PDIST80B response."""
    if len(packet) != 18:
        raise ValueError("PID 238 response must be 18 bytes")
    if sum(packet) & 0xFF:
        raise ValueError("invalid checksum")
    receiver, sender, response_id, pid, data_length = packet[:5]
    if (receiver, sender, response_id, pid, data_length) != (
        OPERATOR_MID, PDIST_MID, device_id, PID_BMS_MONITOR, 12
    ):
        raise ValueError("unexpected PDIST80B response header")
    data = packet[5:-1]
    voltage_raw = _u16_le(data, 0)
    discharge_raw = _measurement_i16(data, 2)
    charge_raw = _measurement_i16(data, 8)
    return Pdist80bStatus(
        voltage_v=None if voltage_raw == 0xFFFF else voltage_raw / 10.0,
        discharge_current_a=(
            None if discharge_raw is None else discharge_raw / 10.0
        ),
        soc_percent=None if data[4] == 0xFF else data[4],
        battery_flags=data[5],
        protection_flags=data[6],
        charge_current_a=None if charge_raw is None else charge_raw / 10.0,
    )
