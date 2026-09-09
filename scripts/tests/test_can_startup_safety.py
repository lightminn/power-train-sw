"""Execute startup paths with fake external commands; never touch a real CAN link."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess

import pytest

from chassis.runtime_lock import RealCanSession
from test_integrated_launchers import (_fake_path, _env, _prepared_tree,
                                      _write_executable, _run, ROBOT_START, ROOT)


def prepared(tmp_path, details):
    binary, log = _fake_path(tmp_path, can_details=details)
    target, etc = _prepared_tree(tmp_path)
    (etc / "robot.json").write_text(json.dumps({
        "robot_id": "test", "token_file": "/etc/powertrain/operator_console.token",
        "host": "0.0.0.0", "session_port": 9002, "input_port": 9000,
        "ops_port": 9001, "input_target_port": 19000, "ops_target_port": 19001,
        "destination_file": "/run/powertrain/operator-session.json", "lease_timeout_s": 2,
    }))
    (etc / "integrated-prepared.env").write_text(
        "POWERTRAIN_INTEGRATED_VERSION=1\nPOWERTRAIN_PROFILE=can-4ws\n")
    return binary, log, target, _env(tmp_path, binary, log)


NORMAL = "5: can0: <NOARP,UP,LOWER_UP> state UNKNOWN qlen 1000\n    can state ERROR-ACTIVE restart-ms 100\n    bitrate 500000"


@pytest.mark.parametrize("details,reason", [
    (NORMAL.replace("can state", "can <LOOPBACK> state"), "LOOPBACK"),
    (NORMAL.replace("can state", "can <LISTEN-ONLY> state"), "LISTEN-ONLY"),
    (NORMAL.replace("500000", "5000000"), "500000"),
    (NORMAL.replace("ERROR-ACTIVE", "BUS-OFF"), "BUS-OFF"),
    (NORMAL.replace("NOARP,UP,LOWER_UP", "NOARP,LOWER_UP"), "UP"),
])
def test_robot_start_rejects_invalid_can_before_compose(tmp_path, details, reason):
    _, log, _, env = prepared(tmp_path, details)
    result = _run(ROBOT_START, env=env)
    assert result.returncode != 0, result.stdout
    assert reason in result.stderr
    assert not log.exists() or "docker " not in log.read_text()


@pytest.mark.parametrize("details,good", [
    (NORMAL, True),
    (NORMAL.replace("state UNKNOWN", "state UP").replace("can state", "can <LOOPBACK> state"), False),
    (NORMAL.replace("state UNKNOWN", "state UP").replace("can state", "can <LISTEN-ONLY> state"), False),
    (NORMAL.replace("state UNKNOWN", "state UP").replace("500000", "250000"), False),
    (NORMAL.replace("state UNKNOWN", "state UP").replace("ERROR-ACTIVE", "BUS-OFF"), False),
])
def test_hil_preflight_uses_same_readiness_contract(tmp_path, monkeypatch, details, good):
    binary, _, _, env = prepared(tmp_path, details)
    monkeypatch.setenv("PATH", env["PATH"])
    spec = importlib.util.spec_from_file_location("can_preflight", ROOT / "scripts/preflight_hil.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    level, detail = module.can_up()
    assert (level == module.OK) is good, detail


def test_can_setup_refuses_active_owner_before_any_link_mutation(tmp_path):
    binary, log, target, env = prepared(tmp_path, NORMAL.replace("500000", "250000"))
    _write_executable(binary / "sudo", '#!/bin/sh\n[ "$1" != -n ] || shift\nexec "$@"\n')
    for command in ("ip", "modprobe", "busybox"):
        _write_executable(binary / command,
            '#!/bin/sh\nprintf "%s %s\\n" "' + command + '" "$*" >> "$FAKE_COMMAND_LOG"\n')
    with RealCanSession(path=str(target / "run/powertrain/can0.lock")):
        result = _run(ROOT / "scripts/can_setup.sh", env=env)
    assert result.returncode != 0, result.stdout
    calls = log.read_text() if log.exists() else ""
    assert "link set" not in calls
    assert "modprobe" not in calls and "busybox" not in calls
    assert "owner" in result.stderr.lower() or "소유" in result.stderr


def test_legacy_start_refuses_prepared_integrated_stack_before_mutation(tmp_path):
    from test_jetson_gui_up import _fake_environment
    env, _ = _fake_environment(tmp_path)
    target, etc = _prepared_tree(tmp_path)
    (etc / "integrated-prepared.env").write_text("POWERTRAIN_INTEGRATED_VERSION=1\n")
    env["POWERTRAIN_ROOT"] = str(target)
    result = _run(ROOT / "scripts/jetson_gui_up.sh", "--no-arm", "--timeout", "1", env=env)
    assert result.returncode != 0
    assert "robot-start" in result.stderr
    for name in ("docker.log", "sudo.log", "systemctl.log"):
        log = tmp_path / name
        assert not log.exists() or not log.read_text()


def test_legacy_start_accepts_normal_unknown_without_link_reset(tmp_path):
    from test_jetson_gui_up import _fake_environment
    env, _ = _fake_environment(tmp_path)
    ip = tmp_path / "bin/ip"
    ip.write_text(ip.read_text().replace("state UP", "state UNKNOWN"))
    result = _run(ROOT / "scripts/jetson_gui_up.sh", "--no-arm", "--timeout", "1", env=env)
    log = tmp_path / "sudo.log"
    calls = log.read_text() if log.exists() else ""
    assert "compose" in (tmp_path / "docker.log").read_text()
    assert "can_setup.sh" not in calls, result.stdout + result.stderr


def test_can_setup_healthy_active_owner_is_observation_only(tmp_path):
    binary, log, target, env = prepared(tmp_path, NORMAL)
    _write_executable(binary / "sudo", '#!/bin/sh\nexec "$@"\n')
    with RealCanSession(path=str(target / "run/powertrain/can0.lock")):
        result = _run(ROOT / "scripts/can_setup.sh", env=env)
    assert result.returncode == 0, result.stderr
    assert not log.exists() or "link set" not in log.read_text()


def test_can_setup_restores_full_invariant_without_explicit_disabled_mode_output(tmp_path):
    binary, log, target, env = prepared(tmp_path, NORMAL)
    state = tmp_path / "link-up"
    env["FAKE_LINK_UP"] = str(state)
    _write_executable(binary / "sudo", '#!/bin/sh\nexec "$@"\n')
    for command in ("modprobe", "busybox"):
        _write_executable(binary / command, '#!/bin/sh\nexit 0\n')
    _write_executable(binary / "ip", '''#!/bin/sh
printf 'ip %s\n' "$*" >> "$FAKE_COMMAND_LOG"
case "$*" in
 '-details link show can0')
   [ -f "$FAKE_LINK_UP" ] || exit 1
   printf '%s\n' '5: can0: <NOARP,UP,LOWER_UP> state UNKNOWN qlen 1000' '    can state ERROR-ACTIVE restart-ms 100' '    bitrate 500000';;
 'link set can0 up') touch "$FAKE_LINK_UP";;
esac
''')
    result = _run(ROOT / "scripts/can_setup.sh", env=env)
    assert result.returncode == 0, result.stderr
    calls = log.read_text()
    assert "link set can0 down" in calls
    assert "bitrate 500000 loopback off listen-only off restart-ms 100" in calls
    assert "link set can0 txqueuelen 1000" in calls
    assert "link set can0 up" in calls
    assert state.exists()


def test_legacy_start_refuses_running_integrated_session_without_manifest(tmp_path):
    from test_jetson_gui_up import _fake_environment
    env, _ = _fake_environment(tmp_path)
    docker = tmp_path / "bin/docker"
    docker.write_text(docker.read_text().replace('set -u\n', 'set -u\nif [ "$1" = ps ]; then echo powertrain_session; exit 0; fi\n'))
    result = _run(ROOT / "scripts/jetson_gui_up.sh", "--no-arm", "--timeout", "1", env=env)
    assert result.returncode != 0
    assert "robot-start" in result.stderr
    for name in ("sudo.log", "systemctl.log"):
        assert not (tmp_path / name).exists()


def test_prepare_refuses_active_owner_before_rebuild_or_config_writes(tmp_path):
    binary, log, target, env = prepared(tmp_path, NORMAL)
    config_path = target / "etc/powertrain/robot.json"
    before = config_path.read_bytes()
    with RealCanSession(path=str(target / "run/powertrain/can0.lock")):
        result = _run(ROOT / "scripts/robot_prepare.sh", "--robot-id", "replacement",
                      "--token-file", "/etc/powertrain/operator_console.token", env=env)
    assert result.returncode != 0, result.stdout
    assert config_path.read_bytes() == before
    calls = log.read_text() if log.exists() else ""
    assert "docker " not in calls and "systemctl" not in calls


def test_legacy_invalid_can_owner_refusal_stops_before_compose(tmp_path):
    from test_jetson_gui_up import _fake_environment
    env, docker_log = _fake_environment(tmp_path)
    target, _ = _prepared_tree(tmp_path)
    env["POWERTRAIN_ROOT"] = str(target)
    ip = tmp_path / "bin/ip"
    ip.write_text(ip.read_text().replace("bitrate 500000", "bitrate 250000"))
    with RealCanSession(path=str(target / "run/powertrain/can0.lock")):
        result = _run(ROOT / "scripts/jetson_gui_up.sh", "--no-arm", "--timeout", "1", env=env)
    assert result.returncode != 0
    calls = docker_log.read_text() if docker_log.exists() else ""
    assert "compose" not in calls, calls


def test_usb_deployer_stops_actual_watchdog_and_integrated_session(tmp_path):
    binary = tmp_path / "bin"
    binary.mkdir()
    log = tmp_path / "remote-calls"
    _write_executable(binary / "tar", '#!/bin/sh\nprintf "fake archive"\n')
    _write_executable(binary / "ssh", '''#!/bin/bash
remote="${@: -1}"
printf '%s\n' "$remote" >> "$FAKE_REMOTE_LOG"
case "$remote" in
 *'git status --porcelain'*) echo 0;;
 *'docker inspect'*) echo true;;
 *'tar -xzf'*|*'sh -s'*) cat >/dev/null;;
esac
''')
    env = {**os.environ, "PATH": f"{binary}:/usr/bin:/bin",
           "FAKE_REMOTE_LOG": str(log), "JETSON_SSH_PASS": ""}
    result = _run(ROOT / "scripts/deploy_usb_skid_to_jetson.sh", "test.invalid", env=env)
    assert result.returncode == 0, result.stdout + result.stderr
    calls = log.read_text()
    assert "docker stop 'powertrain_canwatchdog'" in calls
    assert "docker stop 'powertrain_session'" in calls
    assert "docker stop 'canwatchdog'" not in calls


def test_legacy_first_install_prepares_lock_directory_before_can(tmp_path):
    from test_jetson_gui_up import _fake_environment
    env, docker_log = _fake_environment(tmp_path)
    state = tmp_path / 'runtime-installed'
    env['FAKE_RUNTIME_STATE'] = str(state)
    bash_env = Path(env['BASH_ENV'])
    bash_env.write_text(bash_env.read_text().replace(
        'function [ {\n',
        'function [ {\n  if [[ $1 = -d && ( $2 = /run/powertrain || $2 = /var/lib/powertrain ) ]]; then builtin [ -f "$FAKE_RUNTIME_STATE" ]; return; fi\n'))
    ip = tmp_path / 'bin/ip'
    ip.write_text(ip.read_text().replace('bitrate 500000', 'bitrate 250000'))
    sudo = tmp_path / 'bin/sudo'
    sudo.write_text(sudo.read_text().replace('set -u\n', '''set -u
case "$*" in
 *install_powertrain_runtime_dir.sh*) touch "$FAKE_RUNTIME_STATE"; exit 0;;
 *can_setup.sh*)
   if [ -f "$FAKE_RUNTIME_STATE" ]; then echo prepared-before-can >&2; else echo missing-lock-directory >&2; fi
   exit 1;;
esac
'''))
    result = _run(ROOT / 'scripts/jetson_gui_up.sh', '--no-arm', '--timeout', '1', env=env)
    assert result.returncode != 0
    assert 'prepared-before-can' in result.stderr, result.stdout + result.stderr
    assert 'missing-lock-directory' not in result.stderr
    assert not docker_log.exists() or 'compose' not in docker_log.read_text()
