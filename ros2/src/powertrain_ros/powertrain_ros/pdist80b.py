"""PDIST80B read-only RS485 packet codec.

PDIST80B is *not* Modbus.  Its RS485 default is 57600 bps, device ID 1,
and it responds to the MDROBOT PID request packet described in the V1.7
manual.  This module intentionally contains no power/relay write command.
"""
from __future__ import annotations

from dataclasses import dataclass


PDIST_MID = 186
OPERATOR_MID = 172
PID_REQUEST_DATA = 4
PID_BMS_MONITOR = 238


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
    voltage_v: float
    discharge_current_a: float
    soc_percent: int
    battery_flags: int
    protection_flags: int
    charge_current_a: float


def _u16_le(data: bytes, offset: int) -> int:
    return data[offset] | data[offset + 1] << 8


def _i16_le(data: bytes, offset: int) -> int:
    value = _u16_le(data, offset)
    return value - 0x10000 if value & 0x8000 else value


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
    return Pdist80bStatus(
        voltage_v=_u16_le(data, 0) / 10.0,
        discharge_current_a=_i16_le(data, 2) / 10.0,
        soc_percent=data[4],
        battery_flags=data[5],
        protection_flags=data[6],
        charge_current_a=_i16_le(data, 8) / 10.0,
    )
