#!/usr/bin/env python3
"""Fail-closed SDK enumeration for the powertrain-owned L515."""

import argparse


def _canonical_serial(serial):
    normalized = str(serial).casefold().lstrip("0")
    return normalized or "0"


def select_exact_serial(context, serial_info, expected_serial):
    """Return expected_serial only when it occurs exactly once in context."""
    if not expected_serial:
        raise ValueError("expected serial must not be empty")

    serials = [
        device.get_info(serial_info) for device in context.query_devices()
    ]
    expected = _canonical_serial(expected_serial)
    matches = [
        serial for serial in serials
        if _canonical_serial(serial) == expected
    ]
    if len(matches) != 1:
        raise ValueError(
            f"expected exactly one SDK device serial {expected_serial}, "
            f"found {len(matches)}"
        )
    return expected_serial


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", required=True)
    args = parser.parse_args()

    import pyrealsense2 as rs

    print(
        select_exact_serial(
            rs.context(), rs.camera_info.serial_number, args.serial
        )
    )


if __name__ == "__main__":
    main()
