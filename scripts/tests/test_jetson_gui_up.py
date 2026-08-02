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
    client_route_src: str | None = None,
    fallback_rc: int = 0,
    helper_rc: int = 0,
    interface_ipv4: str | None = "192.168.8.106",
    metadata_sender: bool = False,
    ros_health: str = "healthy",
    ros_running: bool = True,
    route_src: str | None = "192.168.8.106",
    ssh_connection: str | None = None,
    telemetry_env_content: str | None = None,
) -> tuple[dict[str, str], Path]:
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    docker_log = tmp_path / "docker.log"
    sudo_log = tmp_path / "sudo.log"
    systemctl_log = tmp_path / "systemctl.log"
    chassis_env = tmp_path / "powertrain-chassis-telemetry"
    pdist80b_env = tmp_path / "powertrain-pdist80b-telemetry"
    if telemetry_env_content is not None:
        chassis_env.write_text(telemetry_env_content, encoding="utf-8")
        pdist80b_env.write_text(telemetry_env_content, encoding="utf-8")
    ssh_fields = ssh_connection.split() if ssh_connection is not None else []
    fake_ssh_client_ip = ssh_fields[0] if len(ssh_fields) == 4 else ""

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
        "#!/bin/bash\n"
        f"printf '%s\\n' \"$*\" >> {systemctl_log}\n"
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
        "#!/bin/bash\n"
        "case \"$*\" in\n"
        "  '-details link show can0')\n"
        "    printf '%s\\n' '2: can0: <NOARP,UP,LOWER_UP> state UP qlen 1000' "
        "'    can state ERROR-ACTIVE restart-ms 100' '    bitrate 500000'\n"
        "    ;;\n"
        "  '-4 -o addr show dev '*)\n"
        + (
            "    interface=\"${@: -1}\"\n"
            "    printf '%s\\n' "
            f"\"2: $interface inet {interface_ipv4}/24 scope global $interface\"\n"
            "    ;;\n"
            if interface_ipv4 is not None
            else "    exit 1\n    ;;\n"
        )
        + (
            f"  'route get {fake_ssh_client_ip}')\n"
            "    printf '%s\\n' "
            f"'{fake_ssh_client_ip} dev eth0 src {client_route_src} uid 1000'\n"
            "    ;;\n"
            if client_route_src is not None and fake_ssh_client_ip
            else ""
        )
        + "  'route get '* )\n"
        + (
            "    printf '%s\\n' "
            f"'192.168.50.10 via 192.168.8.1 dev eth0 src {route_src} uid 1000'\n"
            "    ;;\n"
            if route_src is not None
            else "    exit 1\n    ;;\n"
        )
        + "esac\n",
    )
    _executable(
        bin_dir / "hostname",
        "#!/bin/sh\n"
        "[ \"${1:-}\" = -I ] && printf '%s\\n' '192.168.50.98 172.17.0.1'\n",
    )
    _executable(
        bin_dir / "awk",
        "#!/bin/bash\n"
        "args=()\n"
        "for arg in \"$@\"; do\n"
        "  case \"$arg\" in\n"
        "    /etc/default/powertrain-chassis-telemetry) "
        "arg=\"$FAKE_CHASSIS_ENV\";;\n"
        "    /etc/default/powertrain-pdist80b-telemetry) "
        "arg=\"$FAKE_PDIST80B_ENV\";;\n"
        "  esac\n"
        "  args+=(\"$arg\")\n"
        "done\n"
        "exec /usr/bin/awk \"${args[@]}\"\n",
    )
    _executable(
        bin_dir / "sudo",
        "#!/bin/bash\n"
        "set -u\n"
        f"printf '%s\\n' \"$*\" >> {sudo_log}\n"
        "[ \"${1:-}\" = -n ] && shift\n"
        "if [ \"${1:-}\" = /usr/local/sbin/powertrain-set-operator-host ]; then\n"
        f"  exit {helper_rc}\n"
        "fi\n"
        "if [ \"${1:-}\" = bash ] && [ \"${2:-}\" = -c ]; then\n"
        f"  [ {fallback_rc} -eq 0 ] || exit {fallback_rc}\n"
        "  command=\"$3\"\n"
        "  command=\"${command//\\/etc\\/default\\/powertrain-chassis-telemetry/"
        "$FAKE_CHASSIS_ENV}\"\n"
        "  command=\"${command//\\/etc\\/default\\/powertrain-pdist80b-telemetry/"
        "$FAKE_PDIST80B_ENV}\"\n"
        "  exec /bin/bash -c \"$command\" \"${@:4}\"\n"
        "fi\n"
        "exec \"$@\"\n",
    )
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
    env.pop("SSH_CONNECTION", None)
    env.update(
        {
            "PATH": f"{bin_dir}:/usr/bin:/bin",
            "HOME": str(tmp_path),
            "ARM_REPO": str(arm_repo),
            "GUI_UP_POLL_S": "0.05",
            "BASH_ENV": str(bash_env),
            "FAKE_CHASSIS_ENV": str(chassis_env),
            "FAKE_PDIST80B_ENV": str(pdist80b_env),
            "FAKE_SUDO_LOG": str(sudo_log),
        }
    )
    if ssh_connection is not None:
        env["SSH_CONNECTION"] = ssh_connection
    return env, docker_log


def _run(
    tmp_path: Path,
    *args: str,
    operator_host: str | None = "192.168.50.10",
    **scenario,
) -> tuple[subprocess.CompletedProcess, str]:
    env, docker_log = _fake_environment(tmp_path, **scenario)
    command = ["bash", str(SCRIPT), "--timeout", "2"]
    if operator_host is not None:
        command.extend(("--operator-host", operator_host))
    command.extend(args)
    result = subprocess.run(
        command,
        cwd=ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=15,
    )
    docker_calls = docker_log.read_text(encoding="utf-8") if docker_log.exists() else ""
    sudo_log = Path(env["FAKE_SUDO_LOG"])
    sudo_calls = sudo_log.read_text(encoding="utf-8") if sudo_log.exists() else ""
    log = docker_calls + "".join(f"sudo {line}\n" for line in sudo_calls.splitlines())
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
        "--host 192.168.8.106"
    ) in result.stdout


def test_operator_command_falls_back_to_first_hostname_address(tmp_path):
    result, _log = _run(tmp_path, route_src=None)

    assert result.returncode == 0, result.stdout
    assert (
        "운영 PC에서: /usr/bin/python3 -m operator_console.app "
        "--host 192.168.50.98"
    ) in result.stdout


def test_operator_command_prefers_ssh_server_ipv4(tmp_path):
    result, _log = _run(
        tmp_path,
        operator_host="192.168.8.163",
        route_src="192.168.50.98",
        ssh_connection="192.168.8.163 48488 192.168.8.106 22",
    )

    assert result.returncode == 0, result.stdout
    assert result.stdout.rstrip().splitlines()[-1] == (
        "운영 PC에서: /usr/bin/python3 -m operator_console.app "
        "--host 192.168.8.106"
    )


def test_operator_command_uses_ipv4_on_ssh_link_local_interface(tmp_path):
    result, _log = _run(
        tmp_path,
        interface_ipv4="192.168.8.106",
        route_src="192.168.50.98",
        ssh_connection=(
            "fe80::d02:7cc:d302:20f4%enP8p1s0 48488 "
            "fe80::6939:619b:3d96:a5d7%enP8p1s0 22"
        ),
    )

    assert result.returncode == 0, result.stdout
    assert result.stdout.rstrip().splitlines()[-1] == (
        "운영 PC에서: /usr/bin/python3 -m operator_console.app "
        "--host 192.168.8.106"
    )


def test_operator_command_routes_to_ssh_client_when_server_is_global_ipv6(tmp_path):
    result, _log = _run(
        tmp_path,
        client_route_src="192.168.8.106",
        route_src="192.168.50.98",
        ssh_connection="192.168.8.163 48488 2001:db8::106 22",
    )

    assert result.returncode == 0, result.stdout
    assert result.stdout.rstrip().splitlines()[-1] == (
        "운영 PC에서: /usr/bin/python3 -m operator_console.app "
        "--host 192.168.8.106"
    )


def test_operator_command_without_ssh_keeps_operator_host_route_fallback(tmp_path):
    result, _log = _run(
        tmp_path,
        operator_host=None,
        route_src="192.168.8.106",
        telemetry_env_content="OPERATOR_HOST=192.168.50.203\n",
    )

    assert result.returncode == 0, result.stdout
    assert result.stdout.rstrip().splitlines()[-1] == (
        "운영 PC에서: /usr/bin/python3 -m operator_console.app "
        "--host 192.168.8.106"
    )


def test_ssh_client_ipv4_becomes_destination_and_calls_helper(tmp_path):
    result, log = _run(
        tmp_path,
        operator_host=None,
        ssh_connection="192.168.8.163 48488 192.168.8.106 22",
        telemetry_env_content="OPERATOR_HOST=192.168.50.203\n",
    )

    assert result.returncode == 0, result.stdout
    assert (
        "sudo -n /usr/local/sbin/powertrain-set-operator-host 192.168.8.163"
        in log
    )
    assert "console_host:=192.168.8.163" in log
    assert "192.168.8.163로 갱신하고 유닛 재시작" in result.stdout
    assert "OPERATOR_HOST 불일치" not in result.stdout


def test_explicit_operator_host_takes_priority_over_ssh_client(tmp_path):
    result, log = _run(
        tmp_path,
        operator_host="10.20.30.40",
        ssh_connection="192.168.8.163 48488 192.168.8.106 22",
        telemetry_env_content="OPERATOR_HOST=192.168.50.203\n",
    )

    assert result.returncode == 0, result.stdout
    assert (
        "sudo -n /usr/local/sbin/powertrain-set-operator-host 10.20.30.40" in log
    )
    assert "powertrain-set-operator-host 192.168.8.163" not in log
    assert "console_host:=10.20.30.40" in log
    assert "OPERATOR_HOST 불일치" not in result.stdout


def test_helper_failure_tries_legacy_sed_fallback(tmp_path):
    result, log = _run(
        tmp_path,
        operator_host=None,
        helper_rc=1,
        fallback_rc=0,
        ssh_connection="192.168.8.163 48488 192.168.8.106 22",
        telemetry_env_content="OPERATOR_HOST=192.168.50.203\n",
    )

    assert result.returncode == 0, result.stdout
    assert "powertrain-set-operator-host 192.168.8.163" in log
    assert "sudo -n bash -c" in log
    assert (
        "sudo -n systemctl restart powertrain-chassis-telemetry.service "
        "powertrain-pdist80b-telemetry.service"
    ) in log
    assert "192.168.8.163로 갱신하고 유닛 재시작" in result.stdout


def test_helper_and_legacy_fallback_failure_warns_but_bridge_uses_new_host(tmp_path):
    result, log = _run(
        tmp_path,
        operator_host=None,
        helper_rc=1,
        fallback_rc=1,
        ssh_connection="192.168.8.163 48488 192.168.8.106 22",
        telemetry_env_content="OPERATOR_HOST=192.168.50.203\n",
    )

    assert result.returncode == 2, result.stdout
    assert "powertrain-set-operator-host 192.168.8.163" in log
    assert "sudo -n bash -c" in log
    assert "console_host:=192.168.8.163" in log
    assert (
        "반영 실패 — :5004/:5005 텔레메트리는 여전히 192.168.50.203 "
        "으로 나갑니다. 젯슨에서 'sudo bash "
        "scripts/install_operator_host_helper.sh' 를 1회 실행하면 이후 자동 반영됩니다"
    ) in result.stdout


def test_matching_installed_host_skips_helper_and_restart(tmp_path):
    result, log = _run(
        tmp_path,
        operator_host=None,
        ssh_connection="192.168.8.163 48488 192.168.8.106 22",
        telemetry_env_content="OPERATOR_HOST=192.168.8.163\n",
    )

    assert result.returncode == 0, result.stdout
    assert "powertrain-set-operator-host" not in log
    assert "sudo -n bash -c" not in log
    assert "sudo -n systemctl restart" not in log
    assert "console_host:=192.168.8.163" in log


def test_ssh_link_local_client_falls_back_to_installed_operator_host(tmp_path):
    result, log = _run(
        tmp_path,
        operator_host=None,
        ssh_connection=(
            "fe80::d02:7cc:d302:20f4%enP8p1s0 48488 "
            "fe80::6939:619b:3d96:a5d7%enP8p1s0 22"
        ),
        telemetry_env_content="OPERATOR_HOST=192.168.50.203\n",
    )

    # 자동감지 실패는 비크리티컬 경고이므로 종료 코드 2가 정상이다.
    assert result.returncode == 2, result.stdout
    assert "powertrain-set-operator-host" not in log
    assert "console_host:=192.168.50.203" in log
    assert "OPERATOR_HOST 불일치" not in result.stdout
    # 침묵하지 않고 `ssh -4` 재실행을 안내해야 한다.
    assert "OPERATOR_HOST 자동감지" in result.stdout
    assert "ssh -4" in result.stdout


def test_ipv4_ssh_client_does_not_emit_autodetect_hint(tmp_path):
    result, _log = _run(
        tmp_path,
        operator_host=None,
        ssh_connection="192.168.8.163 56296 192.168.8.106 22",
        telemetry_env_content="OPERATOR_HOST=192.168.50.203\n",
    )

    assert result.returncode == 0, result.stdout
    assert "OPERATOR_HOST 자동감지" not in result.stdout


def test_local_run_without_ssh_does_not_emit_autodetect_hint(tmp_path):
    result, _log = _run(
        tmp_path,
        operator_host=None,
        ssh_connection=None,
        telemetry_env_content="OPERATOR_HOST=192.168.50.203\n",
    )

    assert result.returncode == 0, result.stdout
    assert "OPERATOR_HOST 자동감지" not in result.stdout


def test_operator_host_missing_line_is_added_and_units_are_restarted(tmp_path):
    result, _log = _run(
        tmp_path,
        helper_rc=1,
        telemetry_env_content="OPERATOR_PORT=5005\n",
    )

    assert result.returncode == 0, result.stdout
    for name in (
        "powertrain-chassis-telemetry",
        "powertrain-pdist80b-telemetry",
    ):
        content = (tmp_path / name).read_text(encoding="utf-8")
        assert "OPERATOR_HOST=192.168.50.10\n" in content
        assert content.count("OPERATOR_HOST=") == 1
    systemctl_log = (tmp_path / "systemctl.log").read_text(encoding="utf-8")
    assert (
        "restart powertrain-chassis-telemetry.service "
        "powertrain-pdist80b-telemetry.service"
    ) in systemctl_log


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
