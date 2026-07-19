"""Exact-device guard shared by standalone powertrain RealSense tools."""

import os


EXPECTED_L515_SERIAL = os.environ.get(
    "POWERTRAIN_L515_SERIAL", "00000000F0271544"
)


def _canonical_serial(value):
    normalized = str(value).casefold().lstrip("0")
    return normalized or "0"


def start_l515_pipeline(pipeline, config, rs_module):
    """Start only the configured L515 and reject an unexpected SDK owner."""
    if not EXPECTED_L515_SERIAL.strip():
        raise RuntimeError("POWERTRAIN_L515_SERIAL must not be empty")
    config.enable_device(EXPECTED_L515_SERIAL)
    profile = pipeline.start(config)
    actual = profile.get_device().get_info(rs_module.camera_info.serial_number)
    if _canonical_serial(actual) != _canonical_serial(EXPECTED_L515_SERIAL):
        pipeline.stop()
        raise RuntimeError(
            "unexpected RealSense serial "
            f"{actual!r}; expected {EXPECTED_L515_SERIAL!r}"
        )
    return profile
