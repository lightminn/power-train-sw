"""ROS-free mirror of the PDIST80B BMS flags from 매뉴얼 V1.7 p.16.

Keep this table byte-for-byte equivalent to ``powertrain_ros.pdist80b``;
``test_pdist80b_flags.py`` enforces the anti-drift contract.
"""
from __future__ import annotations


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
