import os
from pathlib import Path
import subprocess
import time

import pytest


ROOT = Path(__file__).resolve().parents[2]
WATCHDOG = ROOT / "scripts/can_watchdog.sh"


def _executable(path, source):
    path.write_text(source, encoding="utf-8")
    path.chmod(0o755)


def _fake_commands(tmp_path, *, fail_up=False, reported_qlen=1000):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    ip_log = tmp_path / "ip.log"
    _executable(
        bin_dir / "tc",
        "#!/bin/sh\n"
        "printf '%s\\n' ' Sent 100 bytes 1 pkt' ' backlog 10b 2p requeues 0'\n",
    )
    fail_clause = "case \"$*\" in *' up') exit 42;; esac\n" if fail_up else ""
    _executable(
        bin_dir / "ip",
        "#!/bin/sh\n"
        f"printf '%s\\n' \"$*\" >> {ip_log}\n"
        f"{fail_clause}"
        "case \"$*\" in\n"
        "  '-details link show dev can0')\n"
        f"    printf '%s\\n' '2: can0: <NOARP,UP,LOWER_UP> state UP qlen {reported_qlen}' "
        "'    can state ERROR-ACTIVE restart-ms 100' "
        "'    bitrate 500000' '    loopback off';;\n"
        "esac\n",
    )
    _executable(bin_dir / "sleep", "#!/bin/sh\nexec /bin/sleep 0.01\n")
    env = os.environ.copy()
    env["PATH"] = f"{bin_dir}:/usr/bin:/bin"
    return env, ip_log


def _wait_for(path, predicate, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        content = path.read_text(encoding="utf-8") if path.exists() else ""
        if predicate(content):
            return content
        time.sleep(0.01)
    return path.read_text(encoding="utf-8") if path.exists() else ""


def test_can_watchdog_restores_and_verifies_full_can_invariant(tmp_path):
    env, ip_log = _fake_commands(tmp_path)
    process = subprocess.Popen(
        ["bash", str(WATCHDOG), "can0"],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        log = _wait_for(ip_log, lambda text: "-details link show dev can0" in text)
    finally:
        process.terminate()
        process.wait(timeout=1.0)

    assert "link set dev can0 down" in log
    assert "link set dev can0 type can bitrate 500000 loopback off restart-ms 100" in log
    assert "link set dev can0 txqueuelen 1000" in log
    assert "link set dev can0 up" in log
    assert "-details link show dev can0" in log


def test_can_watchdog_exits_nonzero_when_link_up_fails(tmp_path):
    env, _ip_log = _fake_commands(tmp_path, fail_up=True)
    process = subprocess.Popen(
        ["bash", str(WATCHDOG), "can0"],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        returncode = process.wait(timeout=0.5)
    except subprocess.TimeoutExpired:
        process.terminate()
        process.wait(timeout=1.0)
        returncode = 0

    assert returncode != 0


def test_can_watchdog_exits_nonzero_when_restored_qlen_is_not_verified(tmp_path):
    env, _ip_log = _fake_commands(tmp_path, reported_qlen=10)
    process = subprocess.Popen(
        ["bash", str(WATCHDOG), "can0"],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        returncode = process.wait(timeout=0.5)
    except subprocess.TimeoutExpired:
        process.terminate()
        process.wait(timeout=1.0)
        returncode = 0

    assert returncode != 0


def test_gateway_healthcheck_replaces_unrelated_import_probe_everywhere():
    dockerfile = (ROOT / "docker/Dockerfile.ros").read_text(encoding="utf-8")
    compose = (ROOT / "docker/docker-compose.jetson.yml").read_text(encoding="utf-8")

    assert "python3 -m l515_dashboard.healthcheck" in dockerfile
    assert "python3 -m l515_dashboard.healthcheck" in compose
    assert 'python3 -c "import powertrain_observability"' not in dockerfile
    assert 'python3 -c "import powertrain_observability"' not in compose


@pytest.mark.parametrize(
    "name",
    (
        "powertrain-chassis-telemetry.service",
        "powertrain-operator-console.service",
        "powertrain-pdist80b-telemetry.service",
    ),
)
def test_restart_units_have_explicit_finite_start_limit_and_action(name):
    text = (ROOT / "scripts/systemd" / name).read_text(encoding="utf-8")
    unit = text.split("[Service]", 1)[0]
    assert "StartLimitIntervalSec=60" in unit
    assert "StartLimitBurst=5" in unit
    assert "StartLimitAction=none" in unit


def test_pdist_udev_rule_requires_commissioned_id_path():
    rule = (ROOT / "scripts/systemd/99-powertrain-pdist80b.rules").read_text(
        encoding="utf-8"
    )
    installer = (ROOT / "scripts/install_pdist80b_telemetry_service.sh").read_text(
        encoding="utf-8"
    )

    assert 'ENV{ID_PATH}=="@PDIST_ID_PATH@"' in rule
    assert 'ATTRS{idVendor}=="1a86"' in rule
    assert 'ATTRS{idProduct}=="7523"' in rule
    assert 'pdist_id_path="$2"' in installer
    assert "@PDIST_ID_PATH@" in installer


def test_pdist_installer_fails_if_restarted_service_is_not_active():
    installer = (ROOT / "scripts/install_pdist80b_telemetry_service.sh").read_text(
        encoding="utf-8"
    )

    assert "systemctl is-active --quiet powertrain-pdist80b-telemetry.service" in installer
    assert "journalctl --no-pager -u powertrain-pdist80b-telemetry.service" in installer
    assert "installed service is not active" in installer
