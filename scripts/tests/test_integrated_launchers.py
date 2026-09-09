import json
import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
ROBOT_START = ROOT / "scripts/robot-start"
ROBOT_PREPARE = ROOT / "scripts/robot_prepare.sh"
CONSOLE_INSTALLER = ROOT / "scripts/install_integrated_console.sh"


def test_observers_do_not_inherit_image_camera_healthcheck():
    import yaml
    services = yaml.safe_load((ROOT / "docker/docker-compose.integrated.yml").read_text())["services"]
    for name in ("powertrain_chassis_telemetry", "powertrain_pdist80b_telemetry", "powertrain_arm_console_bridge"):
        assert services[name].get("healthcheck", {}).get("disable") is True, name


def test_ros_setup_optional_environment_is_allowed_by_daily_shell(tmp_path):
    import yaml
    fake_setup = tmp_path / "setup.bash"
    fake_setup.write_text('test -z "$AMENT_TRACE_SETUP_FILES"\n')
    services = yaml.safe_load((ROOT / "docker/docker-compose.integrated.yml").read_text())["services"]
    tested = 0
    for service in services.values():
        command = "\n".join(service.get("command", []))
        if "source /opt/ros/humble/setup.bash" not in command:
            continue
        lines = []
        for line in command.splitlines():
            if line.startswith("source "):
                lines.append('source "' + str(fake_setup) + '"')
            elif line in ("set -e", "set -eu", "set -u", "set +u"):
                lines.append(line)
        env = dict(os.environ)
        env.pop("AMENT_TRACE_SETUP_FILES", None)
        result = subprocess.run(["bash", "-c", "\n".join(lines)], env=env, capture_output=True, text=True)
        assert result.returncode == 0, result.stderr
        tested += 1
    assert tested >= 5


def _write_executable(path: Path, source: str) -> None:
    path.write_text(source, encoding="utf-8")
    path.chmod(0o755)


def _fake_path(
    tmp_path: Path,
    *,
    legacy=False,
    legacy_telemetry=False,
    can_details="5: can0: <NOARP,UP,LOWER_UP> state UNKNOWN\n    can state ERROR-ACTIVE bitrate 500000",
) -> tuple[Path, Path]:
    binary_dir = tmp_path / "bin"
    binary_dir.mkdir()
    log = tmp_path / "commands.log"
    _write_executable(
        binary_dir / "docker",
        "#!/bin/sh\nprintf 'docker %s\\n' \"$*\" >>\"$FAKE_COMMAND_LOG\"\n",
    )
    _write_executable(
        binary_dir / "ip",
        "#!/bin/sh\n"
        "if [ \"$*\" = '-details link show can0' ]; then\n"
        f"  printf '%s\\n' '{can_details}'\n"
        "  exit 0\n"
        "fi\nexit 1\n",
    )
    legacy_rc = "0" if legacy else "1"
    telemetry_rc = "0" if legacy_telemetry else "1"
    _write_executable(
        binary_dir / "systemctl",
        "#!/bin/sh\n"
        "printf 'systemctl %s\\n' \"$*\" >>\"$FAKE_COMMAND_LOG\"\n"
        "case \"$*\" in\n"
        "  '--user is-enabled --quiet powertrain-operator-console.service'|"
        "'--user is-active --quiet powertrain-operator-console.service') "
        f"exit {legacy_rc};;\n"
        "  'is-enabled --quiet powertrain-chassis-telemetry.service'|"
        "'is-active --quiet powertrain-chassis-telemetry.service'|"
        "'is-enabled --quiet powertrain-pdist80b-telemetry.service'|"
        "'is-active --quiet powertrain-pdist80b-telemetry.service') "
        f"exit {telemetry_rc};;\n"
        "  '--user disable --now powertrain-operator-console.service'|"
        "'disable --now powertrain-chassis-telemetry.service'|"
        "'disable --now powertrain-pdist80b-telemetry.service') exit 0;;\n"
        "esac\nexit 1\n",
    )
    return binary_dir, log


def _env(tmp_path: Path, binary_dir: Path, log: Path) -> dict[str, str]:
    return {
        **os.environ,
        "PATH": f"{binary_dir}:{os.environ['PATH']}",
        "FAKE_COMMAND_LOG": str(log),
        "POWERTRAIN_ROOT": str(tmp_path / "target"),
        "HOME": str(tmp_path / "home"),
    }


def _run(path: Path, *args: str, env: dict[str, str]):
    return subprocess.run(
        ["bash", str(path), *args],
        cwd=ROOT,
        env=env,
        text=True,
        capture_output=True,
        timeout=15,
    )


def _prepared_tree(tmp_path: Path):
    target = tmp_path / "target"
    etc = target / "etc/powertrain"
    run = target / "run/powertrain"
    var = target / "var/lib/powertrain"
    for directory in (etc, run, var):
        directory.mkdir(parents=True)
    (etc / "operator_console.token").write_text("paired-secret\n")
    (etc / "ops_console.token").write_text("ops-secret\n")
    (etc / "powertrain.env").write_text(
        "STOP_MM=350\nSTOP_MM_PROVENANCE=COMMISSIONED\n"
    )
    return target, etc


def test_robot_start_refuses_missing_prepared_manifest_without_docker(tmp_path):
    binary_dir, log = _fake_path(tmp_path)
    _prepared_tree(tmp_path)

    result = _run(ROBOT_START, env=_env(tmp_path, binary_dir, log))

    assert result.returncode != 0
    assert "robot_prepare.sh" in result.stderr
    assert not log.exists() or "docker " not in log.read_text()


def test_robot_start_is_idempotent_no_build_and_needs_no_operator_ip(tmp_path):
    binary_dir, log = _fake_path(tmp_path)
    _, etc = _prepared_tree(tmp_path)
    (etc / "robot.json").write_text(
        json.dumps(
            {
                "robot_id": "zetin-rover-1",
                "token_file": "/etc/powertrain/operator_console.token",
                "host": "0.0.0.0",
                "session_port": 9002,
                "input_port": 9000,
                "ops_port": 9001,
                "input_target_port": 19000,
                "ops_target_port": 19001,
                "destination_file": "/run/powertrain/operator-session.json",
                "lease_timeout_s": 2.0,
                "ops_token_file": "/etc/powertrain/ops_console.token",
            }
        )
    )
    (etc / "integrated-prepared.env").write_text(
        "POWERTRAIN_INTEGRATED_VERSION=1\nPOWERTRAIN_PROFILE=can-4ws\n"
        "OPTIONAL_COMPOSE_FILE=\nOPTIONAL_PROFILES=\nOPTIONAL_SERVICES=\n"
    )
    env = _env(tmp_path, binary_dir, log)

    first = _run(ROBOT_START, env=env)
    second = _run(ROBOT_START, env=env)

    assert first.returncode == second.returncode == 0, first.stderr + second.stderr
    commands = log.read_text(encoding="utf-8")
    docker_lines = [line for line in commands.splitlines() if line.startswith("docker ")]
    assert len(docker_lines) == 2
    for line in docker_lines:
        assert "compose -f docker/docker-compose.jetson.yml -f docker/docker-compose.integrated.yml" in line
        assert "up -d --no-build" in line
        assert "powertrain_session" in line
        assert "powertrain_control" in line
        assert "powertrain_chassis" in line
        assert " build" not in line and "--build" not in line
        assert "192.168." not in line and "operator-host" not in line
    assert "restart" not in commands


def test_robot_start_rejects_bus_off_even_when_interface_has_up_flag(tmp_path):
    binary_dir, log = _fake_path(
        tmp_path,
        can_details="5: can0: <NOARP,UP,LOWER_UP> state UNKNOWN\n    can state BUS-OFF bitrate 500000",
    )
    _, etc = _prepared_tree(tmp_path)
    (etc / "robot.json").write_text(
        json.dumps(
            {
                "robot_id": "zetin-rover-1",
                "token_file": "/etc/powertrain/operator_console.token",
                "host": "0.0.0.0",
                "session_port": 9002,
                "input_port": 9000,
                "ops_port": 9001,
                "input_target_port": 19000,
                "ops_target_port": 19001,
                "destination_file": "/run/powertrain/operator-session.json",
                "lease_timeout_s": 2.0,
            }
        )
    )
    (etc / "integrated-prepared.env").write_text(
        "POWERTRAIN_INTEGRATED_VERSION=1\nPOWERTRAIN_PROFILE=can-4ws\n"
        "OPTIONAL_COMPOSE_FILE=\nOPTIONAL_PROFILES=\nOPTIONAL_SERVICES=\n"
    )

    result = _run(ROBOT_START, env=_env(tmp_path, binary_dir, log))

    assert result.returncode != 0
    assert "BUS-OFF" in result.stderr
    assert not log.exists() or "docker " not in log.read_text()


def test_robot_prepare_builds_once_and_writes_explicit_can_profile(tmp_path):
    binary_dir, log = _fake_path(tmp_path)
    _, etc = _prepared_tree(tmp_path)
    env = _env(tmp_path, binary_dir, log)

    result = _run(
        ROBOT_PREPARE,
        "--robot-id",
        "zetin-rover-1",
        "--token-file",
        "/etc/powertrain/operator_console.token",
        "--drive-ops-token-file",
        "/etc/powertrain/ops_console.token",
        env=env,
    )

    assert result.returncode == 0, result.stderr
    config = json.loads((etc / "robot.json").read_text(encoding="utf-8"))
    assert config == {
        "robot_id": "zetin-rover-1",
        "token_file": "/etc/powertrain/operator_console.token",
        "host": "0.0.0.0",
        "session_port": 9002,
        "input_port": 9000,
        "ops_port": 9001,
        "input_target_port": 19000,
        "ops_target_port": 19001,
        "destination_file": "/run/powertrain/operator-session.json",
        "lease_timeout_s": 2.0,
        "ops_token_file": "/etc/powertrain/ops_console.token",
    }
    manifest = (etc / "integrated-prepared.env").read_text(encoding="utf-8")
    assert "POWERTRAIN_PROFILE=can-4ws" in manifest
    dropin = (
        tmp_path
        / "target/etc/systemd/system/powertrain-bringup-preflight.service.d/integrated-repo.conf"
    ).read_text(encoding="utf-8")
    assert f"WorkingDirectory={ROOT}" in dropin
    assert f"PYTHONPATH={ROOT}/ros2/src/powertrain_ros" in dropin
    commands = log.read_text(encoding="utf-8")
    assert "docker compose -f docker/docker-compose.jetson.yml -f docker/docker-compose.integrated.yml build" in commands
    assert "colcon build --packages-select robot_arm_msgs powertrain_msgs powertrain_ros" in commands
    assert "calibrat" not in commands.lower()


def test_robot_prepare_reuses_existing_images_but_rebuilds_ros_source(tmp_path):
    binary_dir, log = _fake_path(tmp_path)
    _, etc = _prepared_tree(tmp_path)
    result = _run(
        ROBOT_PREPARE, "--robot-id", "zetin-rover-1",
        "--token-file", "/etc/powertrain/ops_console.token",
        "--use-existing-images", env=_env(tmp_path, binary_dir, log),
    )
    assert result.returncode == 0, result.stderr
    assert (etc / "robot.json").exists()
    commands = log.read_text()
    assert "docker image inspect powertrain-sw:jetson powertrain-sw:ros" in commands
    assert "colcon build --packages-select robot_arm_msgs powertrain_msgs powertrain_ros" in commands
    assert not any("docker compose" in line and " build powertrain" in line for line in commands.splitlines())


def test_robot_prepare_reuse_refuses_missing_image_before_service_changes(tmp_path):
    binary_dir, log = _fake_path(tmp_path)
    _, etc = _prepared_tree(tmp_path)
    _write_executable(binary_dir / "docker", "#!/bin/sh\nexit 1\n")
    result = _run(
        ROBOT_PREPARE, "--robot-id", "zetin-rover-1",
        "--token-file", "/etc/powertrain/ops_console.token",
        "--use-existing-images", env=_env(tmp_path, binary_dir, log),
    )
    assert result.returncode != 0
    assert "existing runtime images" in result.stderr
    assert not (etc / "robot.json").exists()
    assert not (etc / "integrated-prepared.env").exists()
    assert not log.exists(), "missing images must be detected before systemctl changes"


def test_robot_prepare_requires_explicit_legacy_telemetry_migration(tmp_path):
    binary_dir, log = _fake_path(tmp_path, legacy_telemetry=True)
    _prepared_tree(tmp_path)
    env = _env(tmp_path, binary_dir, log)
    args = (
        "--robot-id", "zetin-rover-1",
        "--token-file", "/etc/powertrain/operator_console.token",
        "--drive-ops-token-file", "/etc/powertrain/ops_console.token",
    )

    refused = _run(ROBOT_PREPARE, *args, env=env)
    assert refused.returncode != 0
    assert "--replace-legacy-telemetry" in refused.stderr
    assert "docker " not in log.read_text(encoding="utf-8")

    replaced = _run(
        ROBOT_PREPARE,
        "--replace-legacy-telemetry",
        *args,
        env=env,
    )
    assert replaced.returncode == 0, replaced.stderr
    commands = log.read_text(encoding="utf-8")
    assert "systemctl disable --now powertrain-chassis-telemetry.service" in commands
    assert "systemctl disable --now powertrain-pdist80b-telemetry.service" in commands


def test_console_installer_refuses_legacy_restart_service_until_replaced(tmp_path):
    binary_dir, log = _fake_path(tmp_path, legacy=True)
    home = tmp_path / "home"
    home.mkdir()
    token = home / "paired.token"
    token.write_text("paired-secret\n")
    env = _env(tmp_path, binary_dir, log)

    refused = _run(
        CONSOLE_INSTALLER,
        "--robot-id", "zetin-rover-1",
        "--host", "jetson-orin.local",
        "--token-file", str(token),
        "--console-python", "/usr/bin/python3",
        "--controller-python", sys.executable,
        env=env,
    )
    assert refused.returncode != 0
    assert "--replace-legacy" in refused.stderr
    assert not (home / ".config/powertrain/operator.json").exists()

    replaced = _run(
        CONSOLE_INSTALLER,
        "--replace-legacy",
        "--robot-id", "zetin-rover-1",
        "--host", "jetson-orin.local",
        "--token-file", str(token),
        "--console-python", "/usr/bin/python3",
        "--controller-python", sys.executable,
        env=env,
    )
    assert replaced.returncode == 0, replaced.stderr
    commands = log.read_text(encoding="utf-8")
    assert "systemctl --user disable --now powertrain-operator-console.service" in commands
    config = json.loads((home / ".config/powertrain/operator.json").read_text())
    assert config["hosts"] == ["jetson-orin.local"]
    assert config["profile"] == "can-4ws"
    assert config["controller_python"] == sys.executable
    launcher = (home / ".local/bin/powertrain-integrated-console").read_text()
    assert 'exec /usr/bin/python3 -m operator_console' in launcher
    assert str(ROOT) in launcher
    desktop = (home / ".local/share/applications/powertrain-integrated-console.desktop").read_text()
    assert str(home / ".local/bin/powertrain-integrated-console") in desktop


def test_console_installer_rejects_python_missing_gui_or_controller_dependencies(tmp_path):
    binary_dir, log = _fake_path(tmp_path)
    home = tmp_path / "home"
    home.mkdir()
    token = home / "paired.token"
    token.write_text("paired-secret\n")
    broken_python = tmp_path / "broken-python"
    _write_executable(broken_python, "#!/bin/sh\nexit 1\n")
    env = _env(tmp_path, binary_dir, log)

    gui = _run(
        CONSOLE_INSTALLER,
        "--robot-id", "zetin-rover-1",
        "--host", "jetson-orin.local",
        "--token-file", str(token),
        "--console-python", str(broken_python),
        "--controller-python", sys.executable,
        env=env,
    )
    assert gui.returncode != 0
    assert "GTK/GStreamer" in gui.stderr

    controller = _run(
        CONSOLE_INSTALLER,
        "--robot-id", "zetin-rover-1",
        "--host", "jetson-orin.local",
        "--token-file", str(token),
        "--console-python", "/usr/bin/python3",
        "--controller-python", str(broken_python),
        env=env,
    )
    assert controller.returncode != 0
    assert "pygame" in controller.stderr
    assert not (home / ".config/powertrain/operator.json").exists()


def test_integrated_control_bindings_are_loopback_and_public_ports_belong_to_session():
    compose = (ROOT / "docker/docker-compose.integrated.yml").read_text(encoding="utf-8")
    assert "input_host:=127.0.0.1" in compose
    assert "input_port:=19000" in compose
    assert "ops_host:=127.0.0.1" in compose
    assert "ops_port:=19001" in compose
    assert "python3 -m powertrain_runtime.robot_service --config /etc/powertrain/robot.json" in compose
    assert "POWERTRAIN_OPERATOR_SESSION_FILE=/run/powertrain/operator-session.json" in compose
    assert "/proc/net/tcp" in compose
    control_health = compose.split("powertrain_control:", 1)[1].split("powertrain_observability:", 1)[0]
    assert "create_connection" not in control_health
