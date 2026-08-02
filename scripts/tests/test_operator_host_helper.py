import os
from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[2]
HELPER = ROOT / "scripts/powertrain-set-operator-host"
INSTALLER = ROOT / "scripts/install_operator_host_helper.sh"


def _executable(path: Path, source: str) -> None:
    path.write_text(source, encoding="utf-8")
    path.chmod(0o755)


def _root_path(tmp_path: Path, *, visudo_rc: int = 0) -> tuple[dict[str, str], Path]:
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    systemctl_log = tmp_path / "systemctl.log"

    _executable(
        bin_dir / "id",
        "#!/bin/sh\n"
        "[ \"${1:-}\" = -u ] && { printf '%s\\n' 0; exit 0; }\n"
        "exec /usr/bin/id \"$@\"\n",
    )
    _executable(
        bin_dir / "chown",
        "#!/bin/sh\nexit 0\n",
    )
    _executable(
        bin_dir / "systemctl",
        "#!/bin/sh\n"
        f"printf '%s\\n' \"$*\" >> {systemctl_log}\n"
        "case \"${1:-}\" in\n"
        "  cat|restart) exit 0;;\n"
        "esac\n"
        "exit 1\n",
    )
    _executable(
        bin_dir / "visudo",
        "#!/bin/sh\n"
        "[ \"${1:-}\" = -cf ] || exit 2\n"
        f"exit {visudo_rc}\n",
    )

    env = os.environ.copy()
    env.update({"PATH": f"{bin_dir}:/usr/bin:/bin"})
    return env, systemctl_log


@pytest.mark.parametrize(
    "address",
    ("1.2.3", "1.2.3.4.5", "1.2.x.4", "256.2.3.4", "1..2.3"),
)
def test_helper_rejects_invalid_ipv4_with_usage_exit(tmp_path, address):
    env, _systemctl_log = _root_path(tmp_path)
    env["POWERTRAIN_ENV_DIR"] = str(tmp_path / "default")

    result = subprocess.run(
        [str(HELPER), address],
        cwd=ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert result.returncode == 64


def test_helper_updates_both_existing_files_and_restarts_only_target_units(tmp_path):
    env, systemctl_log = _root_path(tmp_path)
    environment_dir = tmp_path / "default"
    environment_dir.mkdir()
    chassis = environment_dir / "powertrain-chassis-telemetry"
    pdist80b = environment_dir / "powertrain-pdist80b-telemetry"
    chassis.write_text(
        "OPERATOR_HOST=192.168.50.203\nOPERATOR_PORT=5005\n", encoding="utf-8"
    )
    pdist80b.write_text(
        "OPERATOR_PORT=5004\nOPERATOR_HOST=192.168.50.203\n", encoding="utf-8"
    )
    env["POWERTRAIN_ENV_DIR"] = str(environment_dir)

    result = subprocess.run(
        [str(HELPER), "192.168.8.163"],
        cwd=ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout == "operator host: 192.168.8.163\n"
    assert chassis.read_text(encoding="utf-8") == (
        "OPERATOR_HOST=192.168.8.163\nOPERATOR_PORT=5005\n"
    )
    assert pdist80b.read_text(encoding="utf-8") == (
        "OPERATOR_PORT=5004\nOPERATOR_HOST=192.168.8.163\n"
    )
    systemctl_calls = systemctl_log.read_text(encoding="utf-8").splitlines()
    assert systemctl_calls == [
        "cat powertrain-chassis-telemetry.service",
        "restart powertrain-chassis-telemetry.service",
        "cat powertrain-pdist80b-telemetry.service",
        "restart powertrain-pdist80b-telemetry.service",
    ]


def test_helper_appends_host_and_does_not_create_missing_environment_file(tmp_path):
    env, _systemctl_log = _root_path(tmp_path)
    environment_dir = tmp_path / "default"
    environment_dir.mkdir()
    chassis = environment_dir / "powertrain-chassis-telemetry"
    pdist80b = environment_dir / "powertrain-pdist80b-telemetry"
    chassis.write_text("OPERATOR_PORT=5005\n", encoding="utf-8")
    env["POWERTRAIN_ENV_DIR"] = str(environment_dir)

    result = subprocess.run(
        [str(HELPER), "10.20.30.40"],
        cwd=ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert result.returncode == 0, result.stderr
    assert chassis.read_text(encoding="utf-8") == (
        "OPERATOR_PORT=5005\nOPERATOR_HOST=10.20.30.40\n"
    )
    assert not pdist80b.exists()
    assert f"없어 건너뜁니다: {pdist80b}" in result.stderr


def test_helper_rejects_non_root_with_permission_exit(tmp_path):
    env, _systemctl_log = _root_path(tmp_path)
    _executable(
        tmp_path / "bin" / "id",
        "#!/bin/sh\n"
        "[ \"${1:-}\" = -u ] && { printf '%s\\n' 1000; exit 0; }\n"
        "exec /usr/bin/id \"$@\"\n",
    )
    env["POWERTRAIN_ENV_DIR"] = str(tmp_path / "default")

    result = subprocess.run(
        [str(HELPER), "192.168.8.163"],
        cwd=ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert result.returncode == 77


def test_installer_uses_override_directories_and_requested_user(tmp_path):
    env, _systemctl_log = _root_path(tmp_path)
    sbin_dir = tmp_path / "sbin"
    sudoers_dir = tmp_path / "sudoers.d"
    sbin_dir.mkdir()
    sudoers_dir.mkdir()
    env.update(
        {
            "POWERTRAIN_SBIN_DIR": str(sbin_dir),
            "POWERTRAIN_SUDOERS_DIR": str(sudoers_dir),
            "SUDO_USER": "ignored-user",
        }
    )

    result = subprocess.run(
        ["bash", str(INSTALLER), "--user", "operator"],
        cwd=ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert result.returncode == 0, result.stderr
    installed_helper = sbin_dir / "powertrain-set-operator-host"
    sudoers_dropin = sudoers_dir / "powertrain-operator-host"
    assert installed_helper.read_bytes() == HELPER.read_bytes()
    assert installed_helper.stat().st_mode & 0o777 == 0o755
    assert sudoers_dropin.read_text(encoding="utf-8") == (
        "operator ALL=(root) NOPASSWD: "
        f"{sbin_dir}/powertrain-set-operator-host\n"
    )
    assert sudoers_dropin.stat().st_mode & 0o777 == 0o440
    assert str(installed_helper) in result.stdout
    assert str(sudoers_dropin) in result.stdout
    assert (
        "sudo -n /usr/local/sbin/powertrain-set-operator-host <IP>"
        in result.stdout
    )


def test_installer_visudo_failure_leaves_no_final_sudoers_file(tmp_path):
    env, _systemctl_log = _root_path(tmp_path, visudo_rc=1)
    sbin_dir = tmp_path / "sbin"
    sudoers_dir = tmp_path / "sudoers.d"
    sbin_dir.mkdir()
    sudoers_dir.mkdir()
    env.update(
        {
            "POWERTRAIN_SBIN_DIR": str(sbin_dir),
            "POWERTRAIN_SUDOERS_DIR": str(sudoers_dir),
        }
    )

    result = subprocess.run(
        ["bash", str(INSTALLER)],
        cwd=ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "visudo" in result.stderr
    assert (sbin_dir / "powertrain-set-operator-host").exists()
    assert not (sudoers_dir / "powertrain-operator-host").exists()
    assert list(sudoers_dir.iterdir()) == []
