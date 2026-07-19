import os
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts/jetson_gui_up.sh"


def _executable(path: Path, source: str) -> None:
    path.write_text(source, encoding="utf-8")
    path.chmod(0o755)


def _fake_environment(
    tmp_path: Path,
    *,
    metadata_sender: bool = False,
    ros_health: str = "healthy",
    ros_running: bool = True,
) -> tuple[dict[str, str], Path]:
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    docker_log = tmp_path / "docker.log"

    arm_repo = tmp_path / "extreme-robot"
    arm_repo.mkdir()
    (arm_repo / "docker-compose.yml").write_text("services: {}\n", encoding="utf-8")
    (arm_repo / "docker-compose.gpu.yml").write_text(
        "services: {}\n", encoding="utf-8"
    )

    _executable(
        bin_dir / "docker",
        "#!/bin/bash\n"
        "set -u\n"
        f"printf '%s\\n' \"$*\" >> {docker_log}\n"
        "if [ \"${1:-}\" = inspect ]; then\n"
        "  container=\"${@: -1}\"\n"
        "  format=\"${3:-}\"\n"
        "  if [[ \"$format\" == *Health.Status* ]]; then\n"
        "    if [ \"$container\" = powertrain_ros ]; then\n"
        f"      printf '%s\\n' {ros_health!r}\n"
        "    else\n"
        "      printf '%s\\n' healthy\n"
        "    fi\n"
        "  elif [[ \"$format\" == *State.Running* ]]; then\n"
        "    if [ \"$container\" = powertrain_ros ]; then\n"
        f"      printf '%s\\n' {'true' if ros_running else 'false'}\n"
        "    else\n"
        "      printf '%s\\n' true\n"
        "    fi\n"
        "  elif [[ \"$format\" == *State.Status* ]]; then\n"
        "    if [ \"$container\" = powertrain_ros ]; then\n"
        f"      printf '%s\\n' {'running' if ros_running else 'exited'}\n"
        "    else\n"
        "      printf '%s\\n' running\n"
        "    fi\n"
        "  fi\n"
        "  exit 0\n"
        "fi\n"
        "if [ \"${1:-}\" = exec ]; then\n"
        "  case \"$*\" in\n"
        "    *'test -f /root/ros2_ws/install/setup.bash'*) exit 0;;\n"
        "    *'pgrep -f metadata_sender_node'*) "
        + ("exit 0" if metadata_sender else "exit 1")
        + ";;\n"
        "    *'pgrep -f perception_node'*) exit 0;;\n"
        "    *'pgrep -f stream_node'*) exit 0;;\n"
        "    *'pgrep -f arm_console_bridge'*) exit 0;;\n"
        "  esac\n"
        "fi\n"
        "if [ \"${1:-}\" = logs ]; then\n"
        "  printf '%s\\n' log1 log2 log3 log4 log5\n"
        "fi\n"
        "exit 0\n",
    )
    _executable(
        bin_dir / "systemctl",
        "#!/bin/sh\n"
        "case \"$*\" in\n"
        "  'is-active powertrain-chassis-telemetry.service'|"
        "'is-active powertrain-pdist80b-telemetry.service'|"
        "'is-active powertrain-bringup-preflight.service') printf '%s\\n' active;;\n"
        "esac\n"
        "exit 0\n",
    )
    _executable(
        bin_dir / "ss",
        "#!/bin/sh\n"
        "case \"$*\" in\n"
        "  *-tln*) printf '%s\\n' 'LISTEN 0 128 0.0.0.0:9000 0.0.0.0:*' "
        "'LISTEN 0 128 0.0.0.0:9001 0.0.0.0:*';;\n"
        "  *-uln*) printf '%s\\n' 'UNCONN 0 0 0.0.0.0:5000 0.0.0.0:*' "
        "'UNCONN 0 0 0.0.0.0:5002 0.0.0.0:*';;\n"
        "esac\n",
    )
    _executable(
        bin_dir / "lsusb",
        "#!/bin/sh\n"
        "printf '%s\\n' "
        "'Bus 001 Device 002: ID 8086:0b64 Intel Corp. L515' "
        "'Bus 001 Device 003: ID 8086:0b3a Intel Corp. D435i'\n",
    )
    _executable(
        bin_dir / "ip",
        "#!/bin/sh\n"
        "printf '%s\\n' '2: can0: <NOARP,UP,LOWER_UP> state UP qlen 1000' "
        "'    can state ERROR-ACTIVE restart-ms 100' '    bitrate 500000'\n",
    )
    _executable(
        bin_dir / "hostname",
        "#!/bin/sh\n"
        "[ \"${1:-}\" = -I ] && printf '%s\\n' '192.168.50.98 172.17.0.1'\n",
    )
    _executable(bin_dir / "sudo", "#!/bin/sh\nexit 0\n")
    _executable(bin_dir / "sleep", "#!/bin/sh\nexec /bin/sleep 0.01\n")

    bash_env = tmp_path / "bash_env"
    bash_env.write_text(
        "function [ {\n"
        "  if [[ $# -eq 3 && ( $1 = -e || $1 = -d || $1 = -f ) ]]; then\n"
        "    case $2 in\n"
        "      /run/powertrain|/var/lib/powertrain|"
        "/etc/powertrain/powertrain.env|/etc/powertrain/ops_console.token|"
        "/etc/default/powertrain-chassis-telemetry|"
        "/etc/default/powertrain-pdist80b-telemetry|/dev/powertrain-pdist80b) "
        "return 0;;\n"
        "    esac\n"
        "  fi\n"
        "  builtin [ \"$@\"\n"
        "}\n",
        encoding="utf-8",
    )

    env = os.environ.copy()
    env.update(
        {
            "PATH": f"{bin_dir}:/usr/bin:/bin",
            "HOME": str(tmp_path),
            "ARM_REPO": str(arm_repo),
            "GUI_UP_POLL_S": "0.05",
            "BASH_ENV": str(bash_env),
        }
    )
    return env, docker_log


def _run(tmp_path: Path, *args: str, **scenario) -> tuple[subprocess.CompletedProcess, str]:
    env, docker_log = _fake_environment(tmp_path, **scenario)
    result = subprocess.run(
        ["bash", str(SCRIPT), "--timeout", "2", "--operator-host", "192.168.50.10", *args],
        cwd=ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=15,
    )
    log = docker_log.read_text(encoding="utf-8") if docker_log.exists() else ""
    return result, log


def _powertrain_up_lines(log: str) -> list[str]:
    return [
        line
        for line in log.splitlines()
        if "compose -f docker/docker-compose.jetson.yml up -d" in line
    ]


def test_happy_path_starts_exact_stacks_and_prints_operator_command(tmp_path):
    result, log = _run(tmp_path)

    assert result.returncode == 0, result.stdout
    up_lines = _powertrain_up_lines(log)
    assert len(up_lines) == 1
    assert all(
        service in up_lines[0]
        for service in (
            "canwatchdog",
            "powertrain_ros",
            "powertrain_control",
            "powertrain_chassis",
            "powertrain_observability",
        )
    )
    assert up_lines[0].strip() != "compose -f docker/docker-compose.jetson.yml up -d"
    assert (
        f"compose -f {tmp_path}/extreme-robot/docker-compose.yml "
        f"-f {tmp_path}/extreme-robot/docker-compose.gpu.yml up -d"
    ) in log
    assert "arm_console_bridge" in log
    assert "console_host:=192.168.50.10" in log
    assert (
        "운영 PC에서: /usr/bin/python3 -m operator_console.app "
        "--host 192.168.50.98"
    ) in result.stdout


def test_fresh_adds_force_recreate_to_powertrain_compose(tmp_path):
    result, log = _run(tmp_path, "--fresh")

    assert result.returncode == 0, result.stdout
    assert "--force-recreate" in _powertrain_up_lines(log)[0]


def test_no_arm_skips_arm_stack_but_starts_bridge(tmp_path):
    result, log = _run(tmp_path, "--no-arm")

    assert result.returncode == 0, result.stdout
    assert "extreme-robot/docker-compose" not in log
    assert "ros2_humble" not in log
    assert "arm_console_bridge" in log


def test_metadata_sender_prevents_bridge_start(tmp_path):
    result, log = _run(tmp_path, metadata_sender=True)

    assert result.returncode == 2, result.stdout
    assert "팔팀 metadata_sender 가동 중" in result.stdout
    assert ":5003 이중 송신 금지" in result.stdout
    assert "ros2 run powertrain_ros arm_console_bridge" not in log


def test_unhealthy_powertrain_ros_is_critical_and_prints_l515_hint(tmp_path):
    result, _log = _run(tmp_path, ros_health="unhealthy")

    assert result.returncode == 1
    assert "❌" in result.stdout
    assert "powertrain_ros" in result.stdout
    assert "L515 미연결 가능성" in result.stdout
    assert "lsusb 8086:0b64 확인" in result.stdout


def test_bridge_falls_back_to_control_when_ros_container_down(tmp_path):
    result, log = _run(tmp_path, ros_running=False)

    assert result.returncode == 1, result.stdout
    assert "exec -d powertrain_control bash -lc" in log
    assert "console_host:=192.168.50.10" in log
    pkill_bridge = [
        line for line in log.splitlines() if "pkill -f arm_console_bridge" in line
    ]
    assert any("powertrain_ros" in line for line in pkill_bridge)
    assert any("powertrain_control" in line for line in pkill_bridge)
    assert "폴백 기동" in result.stdout


def test_kill_targets_exclude_metadata_sender(tmp_path):
    result, log = _run(tmp_path)

    assert result.returncode == 0, result.stdout
    pkill_lines = [line for line in log.splitlines() if "pkill -f" in line]
    assert any("robot_arm_perception.perception_node" in line for line in pkill_lines)
    assert any("robot_arm_perception.stream_node" in line for line in pkill_lines)
    assert not any("pkill -f metadata_sender" in line for line in pkill_lines)


def test_script_has_valid_bash_syntax():
    result = subprocess.run(
        ["bash", "-n", str(SCRIPT)],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )

    assert result.returncode == 0, result.stdout
