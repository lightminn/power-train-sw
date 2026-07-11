import os
from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[4]
SCRIPT = ROOT / "scripts" / "l515_preflight.sh"
SERIAL = "00000000F0271544"


def _executable(path, body):
    path.write_text("#!/bin/sh\n" + body, encoding="utf-8")
    path.chmod(0o755)


def _run_preflight(tmp_path, *, lsusb, speed="5000", sdk_output=SERIAL):
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    _executable(
        fake_bin / "lsusb",
        "if [ \"${1:-}\" = -d ]; then\n"
        "  printf '%s\\n' \"$FAKE_LSUSB\" | grep -i \"ID $2\"\n"
        "else\n"
        "  printf '%s\\n' \"$FAKE_LSUSB\"\n"
        "fi\n",
    )
    _executable(
        fake_bin / "docker",
        "test \"$1 $2 $3\" = 'exec -i powertrain_ros' || exit 90\n"
        "cat >/dev/null\n"
        "printf '%s\\n' \"$FAKE_SDK_OUTPUT\"\n",
    )

    sysfs = tmp_path / "sys" / "bus" / "usb" / "devices" / "2-1"
    sysfs.mkdir(parents=True)
    (sysfs / "busnum").write_text("2\n", encoding="ascii")
    (sysfs / "devnum").write_text("3\n", encoding="ascii")
    (sysfs / "speed").write_text(f"{speed}\n", encoding="ascii")

    env = os.environ.copy()
    env.update(
        PATH=f"{fake_bin}:{env['PATH']}",
        L515_SYSFS_ROOT=str(tmp_path / "sys" / "bus" / "usb" / "devices"),
        FAKE_LSUSB=lsusb,
        FAKE_SDK_OUTPUT=sdk_output,
    )
    return subprocess.run(
        ["bash", str(SCRIPT)], text=True, capture_output=True, env=env
    )


def test_accepts_expected_l515_on_usb3_and_sdk_serial(tmp_path):
    result = _run_preflight(
        tmp_path,
        lsusb=(
            "Bus 002 Device 003: ID 8086:0b64 "
            "Intel Corp. Intel RealSense L515"
        ),
    )

    assert result.returncode == 0, result.stderr
    assert "PASS" in result.stdout
    assert SERIAL in result.stdout


def test_rejects_missing_l515_usb_device(tmp_path):
    result = _run_preflight(
        tmp_path,
        lsusb="Bus 002 Device 009: ID 8086:0b3a Intel Corp. RealSense D435i",
    )

    assert result.returncode != 0
    assert "8086:0b64" in result.stderr


def test_rejects_usb2_480_mbps_link(tmp_path):
    result = _run_preflight(
        tmp_path,
        lsusb=(
            "Bus 002 Device 003: ID 8086:0b64 "
            "Intel Corp. Intel RealSense L515"
        ),
        speed="480",
    )

    assert result.returncode != 0
    assert "5000" in result.stderr
    assert "480" in result.stderr


@pytest.mark.parametrize("sdk_output", ["", "250222071245"])
def test_rejects_sdk_missing_or_wrong_serial(tmp_path, sdk_output):
    result = _run_preflight(
        tmp_path,
        lsusb=(
            "Bus 002 Device 003: ID 8086:0b64 "
            "Intel Corp. Intel RealSense L515"
        ),
        sdk_output=sdk_output,
    )

    assert result.returncode != 0
    assert "SDK" in result.stderr
    assert SERIAL in result.stderr
