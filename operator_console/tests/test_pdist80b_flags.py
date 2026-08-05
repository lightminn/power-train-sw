"""ROS-free console mirror checks for the canonical PDIST80B flag table."""

import importlib
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2/src/powertrain_ros"))

from powertrain_ros import pdist80b as canonical  # noqa: E402


def test_console_flag_table_and_decoder_match_canonical_for_every_byte():
    console = importlib.import_module("operator_console.pdist80b_flags")

    assert console.BATTERY_FLAG_LABELS == canonical.BATTERY_FLAG_LABELS
    assert console.PROTECTION_FLAG_LABELS == canonical.PROTECTION_FLAG_LABELS
    assert console.BATTERY_FAULT_MASK == canonical.BATTERY_FAULT_MASK
    assert console.PROTECTION_FAULT_MASK == canonical.PROTECTION_FAULT_MASK
    for flags in range(256):
        assert console.battery_flag_labels(flags) == canonical.battery_flag_labels(flags)
        assert console.protection_flag_labels(flags) == canonical.protection_flag_labels(flags)
        assert console.fault_reasons(flags, 0) == canonical.fault_reasons(flags, 0)
        assert console.fault_reasons(0, flags) == canonical.fault_reasons(0, flags)


def test_console_combined_fault_reason_order_matches_canonical():
    console = importlib.import_module("operator_console.pdist80b_flags")

    assert console.fault_reasons(0xFC, 0x0F) == canonical.fault_reasons(0xFC, 0x0F)
