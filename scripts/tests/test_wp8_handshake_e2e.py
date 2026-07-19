import os
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "wp8_handshake_e2e.sh"


def _executable(path: Path, source: str) -> None:
    path.write_text(source, encoding="utf-8")
    path.chmod(0o755)


def _fake_environment(
    tmp_path: Path,
    *,
    arm_present: bool = True,
    pickup_branch: str = "work_accepted",
    negative_probe_rc: int = 1,
):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    docker_log = tmp_path / "docker.log"
    probe_log = tmp_path / "probe.log"
    _executable(
        bin_dir / "docker",
        "#!/bin/bash\n"
        "set -u\n"
        f"printf '%s\\n' \"$*\" >> {docker_log}\n"
        "if [ \"${1:-}\" = inspect ]; then\n"
        "  container=\"${@: -1}\"\n"
        "  if [ \"$container\" = powertrain_ros ]; then\n"
        "    printf '%s\\n' true\n"
        "  elif [ \"$container\" = ros2_humble ]; then\n"
        f"    printf '%s\\n' {'true' if arm_present else 'false'}\n"
        "  else\n"
        "    printf '%s\\n' false\n"
        "  fi\n"
        "  exit 0\n"
        "fi\n"
        "if [ \"${1:-}\" = exec ]; then\n"
        "  command_line=\"$*\"\n"
        "  case \"$command_line\" in\n"
        "    *'ros2 pkg executables dynamixel_control'*)\n"
        "      printf '%s\\n' 'dynamixel_control arm_fsm'; exit 0;;\n"
        "    *'cat /tmp/'*'chassis.pgid'*) printf '%s\\n' 4101; exit 0;;\n"
        "    *'cat /tmp/'*'arm.pgid'*) printf '%s\\n' 4201; exit 0;;\n"
        "    *'ros2 topic info'*) printf '%s\\n' 'Publisher count: 1'; exit 0;;\n"
        "  esac\n"
        "  if [[ \"$command_line\" == *'wp8_handshake_probe '* ]]; then\n"
        "    if [[ \"$command_line\" == *'WP8_PROBE_OBSERVE_ONLY=1'* ]]; then\n"
        f"      printf '%s\\n' pickup-negative >> {probe_log}\n"
        f"      printf '%s\\n' '{{\"subcommand\":\"pickup\",\"pass\":"
        f"{'true' if negative_probe_rc == 0 else 'false'},"
        "\"branch\":\"violation\",\"findings\":[]}'\n"
        f"      exit {negative_probe_rc}\n"
        "    fi\n"
        "    case \"$command_line\" in\n"
        f"      *'wp8_handshake_probe baseline'*) printf '%s\\n' baseline >> {probe_log};;\n"
        f"      *'wp8_handshake_probe pickup'*) printf '%s\\n' pickup >> {probe_log};;\n"
        f"      *'wp8_handshake_probe resume'*) printf '%s\\n' resume >> {probe_log};;\n"
        f"      *'wp8_handshake_probe full-cycle'*) printf '%s\\n' full-cycle >> {probe_log};;\n"
        f"      *'wp8_handshake_probe fault'*'no_response'*) printf '%s\\n' fault:no_response >> {probe_log};;\n"
        f"      *'wp8_handshake_probe fault'*'late_done'*) printf '%s\\n' fault:late_done >> {probe_log};;\n"
        f"      *'wp8_handshake_probe fault'*'failed_latch'*) printf '%s\\n' fault:failed_latch >> {probe_log};;\n"
        f"      *'wp8_handshake_probe fault'*'dup_done'*) printf '%s\\n' fault:dup_done >> {probe_log};;\n"
        "    esac\n"
        "    printf '%s\\n' '{\"pass\":true}'\n"
        "  fi\n"
        "  exit 0\n"
        "fi\n"
        "if [ \"${1:-}\" = cp ]; then\n"
        "  source_path=\"${2:-}\"\n"
        "  host_path=\"${3:-}\"\n"
        "  case \"$source_path\" in\n"
        "    *phase1_pickup.json)\n"
        f"      printf '%s\\n' '{{\"subcommand\":\"pickup\",\"pass\":true,"
        f"\"branch\":\"{pickup_branch}\",\"findings\":[]}}' > \"$host_path\";;\n"
        "    *) printf '%s\\n' '{\"pass\":true}' > \"$host_path\";;\n"
        "  esac\n"
        "  exit 0\n"
        "fi\n"
        "exit 0\n",
    )
    _executable(bin_dir / "sleep", "#!/bin/sh\nexec /bin/sleep 0.001\n")
    env = os.environ.copy()
    env.update(
        {
            "PATH": f"{bin_dir}:/usr/bin:/bin",
            "WP8_E2E_LOG_ROOT": str(tmp_path),
        }
    )
    return env, docker_log, probe_log


def _run(
    tmp_path: Path,
    *args: str,
    arm_present: bool = True,
    pickup_branch: str = "work_accepted",
    negative_probe_rc: int = 1,
):
    env, docker_log, probe_log = _fake_environment(
        tmp_path,
        arm_present=arm_present,
        pickup_branch=pickup_branch,
        negative_probe_rc=negative_probe_rc,
    )
    result = subprocess.run(
        ["bash", str(SCRIPT), "--timeout", "2", *args],
        cwd=ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=15,
    )
    docker_calls = docker_log.read_text(encoding="utf-8") if docker_log.exists() else ""
    probes = probe_log.read_text(encoding="utf-8").splitlines() if probe_log.exists() else []
    return result, docker_calls, probes


def test_script_has_valid_bash_syntax():
    result = subprocess.run(
        ["bash", "-n", str(SCRIPT)],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )

    assert result.returncode == 0, result.stdout


def test_happy_path_runs_probes_in_contract_order(tmp_path):
    result, _docker_calls, probes = _run(tmp_path)

    assert result.returncode == 0, result.stdout
    assert probes == [
        "baseline",
        "pickup",
        "resume",
        "full-cycle",
        "fault:no_response",
        "fault:late_done",
        "fault:failed_latch",
        "fault:dup_done",
    ]


def test_negative_control_skips_pickup_stimulus_and_requires_probe_failure(tmp_path):
    result, docker_calls, probes = _run(tmp_path, "--negative-control")

    assert result.returncode == 0, result.stdout
    assert probes == ["pickup-negative"]
    assert "WP8_PROBE_OBSERVE_ONLY=1" in docker_calls
    assert "mission_arrive_pickup" not in docker_calls
    assert "ros2 run powertrain_ros chassis" in docker_calls
    assert "pkill -TERM -g 4101" in docker_calls
    assert "음성 대조 OK" in result.stdout


def test_negative_control_rejects_observe_only_pass_and_stops_chassis(tmp_path):
    result, docker_calls, probes = _run(
        tmp_path,
        "--negative-control",
        negative_probe_rc=0,
    )

    assert result.returncode == 1, result.stdout
    assert probes == ["pickup-negative"]
    assert "pkill -TERM -g 4101" in docker_calls
    assert "살아있는 chassis 무자극 관측이 PASS" in result.stdout


def test_phase1_fail_closed_skips_watchdog_resume(tmp_path):
    result, docker_calls, probes = _run(
        tmp_path,
        "--phase1-only",
        pickup_branch="fail_closed",
    )

    assert result.returncode == 0, result.stdout
    assert probes == ["baseline", "pickup"]
    assert "pkill -STOP" not in docker_calls
    assert "pkill -CONT" not in docker_calls
    assert (
        "fail-closed — headless 잠금자세 게이트, 스펙 §3-7 한계"
        in result.stdout
    )


def test_cleanup_and_watchdog_target_only_owned_processes():
    source = SCRIPT.read_text(encoding="utf-8")

    assert 'pkill -STOP -g "$CHASSIS_PGID"' in source
    assert 'pkill -CONT -g "$CHASSIS_PGID"' in source
    assert "pkill -STOP -f" not in source
    assert "pkill -CONT -f" not in source
    assert "chassis_telemetry" in source
    assert "pkill -TERM -g" in source
    assert "arm_fsm" in source
    for forbidden in ("perception", "stream", "metadata_sender"):
        assert forbidden not in source


def test_missing_arm_container_skips_phase1_runs_phase2_and_exits_three(tmp_path):
    result, _docker_calls, probes = _run(tmp_path, arm_present=False)

    assert result.returncode == 3, result.stdout
    assert probes == [
        "full-cycle",
        "fault:no_response",
        "fault:late_done",
        "fault:failed_latch",
        "fault:dup_done",
    ]
    assert "Phase 1 SKIP" in result.stdout
