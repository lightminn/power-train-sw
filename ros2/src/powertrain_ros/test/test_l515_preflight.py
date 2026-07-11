import importlib.util
import os
from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[4]
SCRIPT = ROOT / "scripts" / "l515_preflight.sh"
PROBE = ROOT / "scripts" / "l515_sdk_probe.py"
SERIAL = "00000000F0271544"
D435_SERIAL = "250222071245"


def _load_probe():
    spec = importlib.util.spec_from_file_location("l515_sdk_probe", PROBE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _FakeDevice:
    def __init__(self, serial):
        self.serial = serial

    def get_info(self, key):
        assert key == "serial_number"
        return self.serial


class _FakeContext:
    def __init__(self, serials):
        self.devices = [_FakeDevice(serial) for serial in serials]

    def query_devices(self):
        return self.devices


def test_probe_selects_exact_l515_once_while_d435_coexists():
    probe = _load_probe()
    context = _FakeContext([D435_SERIAL, SERIAL])

    selected = probe.select_exact_serial(context, "serial_number", SERIAL)

    assert selected == SERIAL


def test_probe_canonicalizes_sdk_l515_serial_case_and_leading_zeroes():
    probe = _load_probe()
    context = _FakeContext([D435_SERIAL, "f0271544"])

    selected = probe.select_exact_serial(context, "serial_number", SERIAL)

    assert selected == SERIAL


@pytest.mark.parametrize(
    "serials,expected",
    [
        ([SERIAL], ""),
        ([SERIAL], D435_SERIAL),
        ([D435_SERIAL], SERIAL),
        ([SERIAL, SERIAL, D435_SERIAL], SERIAL),
        (["f0271544", SERIAL, D435_SERIAL], SERIAL),
    ],
)
def test_probe_rejects_invalid_expected_selection(serials, expected):
    probe = _load_probe()

    with pytest.raises(ValueError):
        probe.select_exact_serial(
            _FakeContext(serials), "serial_number", expected
        )


def _executable(path, body):
    path.write_text("#!/bin/sh\n" + body, encoding="utf-8")
    path.chmod(0o755)


def _run_preflight(
    tmp_path, *, lsusb, speed="5000", sdk_output=SERIAL, docker_status=0
):
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
        "test \"$#\" -eq 7 || exit 90\n"
        "test \"$1 $2 $3 $4 $5 $6\" = "
        "'exec -i powertrain_ros python3 "
        "/workspace/scripts/l515_sdk_probe.py --serial' "
        "|| exit 91\n"
        "test \"$7\" = \"$FAKE_EXPECTED_SERIAL\" || exit 92\n"
        "test \"$FAKE_DOCKER_STATUS\" -eq 0 || exit \"$FAKE_DOCKER_STATUS\"\n"
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
        FAKE_DOCKER_STATUS=str(docker_status),
        FAKE_EXPECTED_SERIAL=SERIAL,
    )
    return subprocess.run(
        ["bash", str(SCRIPT)], text=True, capture_output=True, env=env
    )


def _l515_lsusb():
    return "Bus 002 Device 003: ID 8086:0b64 Intel Corp. Intel RealSense L515"


def test_accepts_expected_l515_on_usb3_and_sdk_serial(tmp_path):
    result = _run_preflight(tmp_path, lsusb=_l515_lsusb())

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
    result = _run_preflight(tmp_path, lsusb=_l515_lsusb(), speed="480")

    assert result.returncode != 0
    assert "5000" in result.stderr
    assert "480" in result.stderr


@pytest.mark.parametrize(
    "sdk_output", ["", D435_SERIAL, f"{SERIAL}\n{SERIAL}"]
)
def test_rejects_invalid_sdk_serial_output(tmp_path, sdk_output):
    result = _run_preflight(
        tmp_path, lsusb=_l515_lsusb(), sdk_output=sdk_output
    )

    assert result.returncode != 0
    assert "SDK" in result.stderr
    assert SERIAL in result.stderr


def test_rejects_nonzero_docker_or_sdk_probe_failure(tmp_path):
    result = _run_preflight(
        tmp_path, lsusb=_l515_lsusb(), docker_status=23
    )

    assert result.returncode != 0
    assert "SDK enumeration failed" in result.stderr
