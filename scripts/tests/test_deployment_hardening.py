import os
from pathlib import Path
import subprocess
import time

import pytest


ROOT = Path(__file__).resolve().parents[2]
WATCHDOG = ROOT / "scripts/can_watchdog.sh"


def _run_installer(name, *args):
    return subprocess.run(
        ["bash", str(ROOT / "scripts" / name), *args],
        capture_output=True,
        check=False,
        text=True,
    )


def _executable(path, source):
    path.write_text(source, encoding="utf-8")
    path.chmod(0o755)


def test_can_watchdog_wrapper_executes_shared_cli_without_probing_hardware():
    result = subprocess.run(["bash", str(WATCHDOG), "can0", "--help"],
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert "--channel" in result.stdout and "--period" in result.stdout


def test_can_watchdog_wrapper_reports_invalid_arguments():
    result = subprocess.run(["bash", str(WATCHDOG), "can0", "--invalid-option"],
                            capture_output=True, text=True, timeout=5)
    assert result.returncode != 0
    assert "unrecognized arguments" in result.stderr


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


@pytest.mark.parametrize(
    ("name", "usage"),
    (
        (
            "install_chassis_telemetry_service.sh",
            "usage: sudo bash scripts/install_chassis_telemetry_service.sh "
            "[--operator-host IPV4]",
        ),
        (
            "install_pdist80b_telemetry_service.sh",
            "usage: sudo bash scripts/install_pdist80b_telemetry_service.sh "
            "[--operator-host IPV4] PDIST_ID_PATH",
        ),
    ),
)
def test_telemetry_installers_expose_current_usage_without_root(name, usage):
    result = _run_installer(name, "--help")

    assert result.returncode == 0
    assert result.stdout == f"{usage}\n"
    assert result.stderr == ""


@pytest.mark.skipif(os.geteuid() == 0, reason="requires a non-root process")
@pytest.mark.parametrize(
    ("name", "args", "selection"),
    (
        (
            "install_chassis_telemetry_service.sh",
            (),
            "operator host: 192.168.8.163 (default)",
        ),
        (
            "install_chassis_telemetry_service.sh",
            ("--operator-host", "192.0.2.10"),
            "operator host: 192.0.2.10 (--operator-host)",
        ),
        (
            "install_chassis_telemetry_service.sh",
            ("192.0.2.11",),
            "operator host: 192.0.2.11 (positional)",
        ),
        (
            "install_pdist80b_telemetry_service.sh",
            ("/devices/platform/test",),
            "operator host: 192.168.8.163 (default)",
        ),
        (
            "install_pdist80b_telemetry_service.sh",
            ("--operator-host", "192.0.2.12", "/devices/platform/test"),
            "operator host: 192.0.2.12 (--operator-host)",
        ),
        (
            "install_pdist80b_telemetry_service.sh",
            ("192.0.2.13", "/devices/platform/test"),
            "operator host: 192.0.2.13 (positional)",
        ),
    ),
)
def test_telemetry_installers_select_default_option_and_legacy_hosts(
    name, args, selection
):
    result = _run_installer(name, *args)

    assert result.returncode != 0
    assert selection in result.stdout
    assert "must run as root" in result.stderr


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
    assert "@PDIST_ID_PATH@" in installer


def test_pdist_installer_fails_if_restarted_service_is_not_active():
    installer = (ROOT / "scripts/install_pdist80b_telemetry_service.sh").read_text(
        encoding="utf-8"
    )

    assert "systemctl is-active --quiet powertrain-pdist80b-telemetry.service" in installer
    assert "journalctl --no-pager -u powertrain-pdist80b-telemetry.service" in installer
    assert "installed service is not active" in installer
