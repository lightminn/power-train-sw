"""Demo mode on/off switches (docs/specs/2026-11-06-demo-mode.md §1).

`scripts/demo-mode` (Jetson) re-runs robot-start keeping the running opt-in
services; `scripts/console-demo-mode` (laptop) flips the console config.
"""
import json
from pathlib import Path
import subprocess
import sys

import pytest

from test_integrated_launchers import ROOT, _env, _fake_path, _ready_robot_tree, _write_executable

DEMO_MODE = ROOT / "scripts/demo-mode"
CONSOLE_DEMO_MODE = ROOT / "scripts/console-demo-mode"


def _robot(tmp_path, *, running, mode="IDLE", chassis_env="POWERTRAIN_DEMO_MODE=false"):
    binary_dir, log = _fake_path(tmp_path)
    _write_executable(binary_dir / "docker", f"""#!/bin/sh
printf 'docker %s demo=%s\\n' "$*" "${{POWERTRAIN_DEMO_MODE:-unset}}" >>"$FAKE_COMMAND_LOG"
case "$1" in
  ps) if [ -n "$FAKE_LOCK" ] && flock -n "$FAKE_LOCK" true 2>/dev/null; then echo ps-unlocked >>"$FAKE_COMMAND_LOG"; fi
      for n in {running}; do echo "$n"; done ;;
  exec) n=$(cat "$FAKE_MODE_COUNT" 2>/dev/null || echo 0); echo $((n+1)) >"$FAKE_MODE_COUNT"
        m=$(printf '%s\\n' $FAKE_MODES | sed -n "$((n+1))p"); [ -z "$m" ] && m=$(printf '%s\\n' $FAKE_MODES | tail -n 1)
        [ -n "$m" ] && [ "$m" != NONE ] && printf "data: '{{\\"mode\\":\\"%s\\"}}'\\n" "$m" ;;
  inspect) echo "$FAKE_CHASSIS_ENV" ;;
esac
exit 0
""")
    _ready_robot_tree(tmp_path)
    env = _env(tmp_path, binary_dir, log)
    env.pop("POWERTRAIN_DEMO_MODE", None)
    env.update(FAKE_MODES=mode or "NONE", FAKE_CHASSIS_ENV=chassis_env,
               FAKE_MODE_COUNT=str(tmp_path / "mode-count"),
               FAKE_LOCK=str(tmp_path / "state/powertrain/demo-mode.lock"),
               XDG_STATE_HOME=str(tmp_path / "state"))
    return env, log


def _run(env, *args):
    result = subprocess.run(["bash", str(DEMO_MODE), *args], cwd=ROOT, env=env,
                            text=True, capture_output=True, timeout=30)
    return result


def _up(log):
    lines = [l for l in log.read_text().splitlines() if l.startswith("docker compose") and " up " in l] \
        if log.exists() else []
    return lines


FIELD = "powertrain_chassis powertrain_terrain_map powertrain_terrain_overlay powertrain_odrive_autoclear"


def test_on_keeps_running_opt_ins_and_swaps_the_autoclear(tmp_path):
    env, log = _robot(tmp_path, running=FIELD)
    result = _run(env, "on")
    assert result.returncode == 0, result.stderr
    (up,) = _up(log)
    assert "--profile terrain-map" in up and "--profile terrain-overlay" in up
    assert "--profile demo" in up and "powertrain_demo_autoclear" in up
    assert "odrive-autoclear" not in up and "powertrain_odrive_autoclear" not in up
    assert up.endswith("demo=true")


def test_off_restores_the_field_autoclear_that_on_replaced(tmp_path):
    env, log = _robot(tmp_path, running=FIELD)
    assert _run(env, "on").returncode == 0
    log.unlink()
    env["FAKE_CHASSIS_ENV"] = "POWERTRAIN_DEMO_MODE=true"
    result = _run(env, "off")
    assert result.returncode == 0, result.stderr
    (up,) = _up(log)
    assert "--profile odrive-autoclear" in up and "powertrain_odrive_autoclear" in up
    assert "--profile demo" not in up and up.endswith("demo=false")


def test_off_does_not_add_an_autoclear_that_was_not_running(tmp_path):
    env, log = _robot(tmp_path, running="powertrain_chassis powertrain_terrain_map")
    result = _run(env, "off")
    assert result.returncode == 0, result.stderr
    (up,) = _up(log)
    assert "odrive-autoclear" not in up and "--profile terrain-map" in up


def test_arm_bridge_opt_in_is_kept(tmp_path):
    env, log = _robot(tmp_path, running="powertrain_chassis powertrain_arm_console_bridge")
    assert _run(env, "on").returncode == 0
    (up,) = _up(log)
    assert "--profile arm-d435" in up and "powertrain_arm_console_bridge" in up


@pytest.mark.parametrize("mode", ["ARMED", ""])
def test_switch_refuses_while_driving_or_unknown_unless_forced(tmp_path, mode):
    env, log = _robot(tmp_path, running=FIELD, mode=mode)
    result = _run(env, "on")
    assert result.returncode != 0
    assert _up(log) == []
    forced = _run(env, "on", "--force")
    assert forced.returncode == 0, forced.stderr
    assert len(_up(log)) == 1


def test_switch_runs_without_a_check_when_the_chassis_is_not_running(tmp_path):
    env, log = _robot(tmp_path, running="", mode="")
    assert _run(env, "on").returncode == 0
    assert len(_up(log)) == 1


@pytest.mark.parametrize("args", [[], ["maybe"], ["on", "--yes"]])
def test_bad_arguments_change_nothing(tmp_path, args):
    env, log = _robot(tmp_path, running=FIELD)
    result = _run(env, *args)
    assert result.returncode != 0
    assert _up(log) == []


def test_status_reports_the_chassis_switch(tmp_path):
    env, log = _robot(tmp_path, running=FIELD + " powertrain_demo_autoclear",
                      chassis_env="POWERTRAIN_DEMO_MODE=true")
    result = _run(env, "status")
    assert result.returncode == 0, result.stderr
    assert "robot demo=true" in result.stdout
    assert _up(log) == []


# ── laptop console switch ───────────────────────────────────────────────
def _console(*args, home):
    return subprocess.run([sys.executable, str(CONSOLE_DEMO_MODE), *args], cwd=ROOT,
                          env={"HOME": str(home), "PATH": "/usr/bin:/bin"},
                          text=True, capture_output=True, timeout=30)


def _configs(home):
    base = home / ".config/powertrain"
    base.mkdir(parents=True)
    for name in ("operator.json", "operator-demo.json"):
        (base / name).write_text(json.dumps({"robot_id": "r", "hosts": ["192.168.8.106"]}))
        (base / name).chmod(0o600)
    return base


def test_console_switch_flips_every_existing_config_and_keeps_other_keys(tmp_path):
    base = _configs(tmp_path)
    result = _console("on", home=tmp_path)
    assert result.returncode == 0, result.stderr
    for name in ("operator.json", "operator-demo.json"):
        data = json.loads((base / name).read_text())
        assert data == {"robot_id": "r", "hosts": ["192.168.8.106"], "demo_mode": True}
        assert (base / name).stat().st_mode & 0o777 == 0o600
    assert _console("off", home=tmp_path).returncode == 0
    assert json.loads((base / "operator.json").read_text())["demo_mode"] is False
    status = _console("status", home=tmp_path)
    assert "demo_mode=false" in status.stdout


def test_console_switch_uses_only_an_explicit_config(tmp_path):
    base = _configs(tmp_path)
    result = _console("on", "--config", str(base / "operator-demo.json"), home=tmp_path)
    assert result.returncode == 0, result.stderr
    assert json.loads((base / "operator-demo.json").read_text())["demo_mode"] is True
    assert "demo_mode" not in json.loads((base / "operator.json").read_text())


def test_console_switch_refuses_bad_json_without_touching_it(tmp_path):
    base = _configs(tmp_path)
    (base / "operator.json").write_text("{broken")
    result = _console("on", home=tmp_path)
    assert result.returncode != 0
    assert (base / "operator.json").read_text() == "{broken"
    assert "demo_mode" not in json.loads((base / "operator-demo.json").read_text())


def test_console_switch_with_robot_runs_the_jetson_switch_over_ssh(tmp_path):
    _configs(tmp_path)
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    log = tmp_path / "ssh.log"
    _write_executable(bin_dir / "ssh", f"#!/bin/sh\nprintf '%s\\n' \"$*\" >>{log}\n")
    result = subprocess.run([sys.executable, str(CONSOLE_DEMO_MODE), "on", "--robot", "192.168.8.106"],
                            cwd=ROOT, env={"HOME": str(tmp_path), "PATH": f"{bin_dir}:/usr/bin:/bin"},
                            text=True, capture_output=True, timeout=30)
    assert result.returncode == 0, result.stderr
    line = log.read_text()
    assert "zetin@192.168.8.106" in line
    assert "cd ~/power-train-sw-integrated && scripts/demo-mode on" in line


@pytest.mark.parametrize("host", ["192.168.8.106;id", "-oProxyCommand=x"])
def test_console_switch_rejects_unsafe_robot_hosts(tmp_path, host):
    _configs(tmp_path)
    result = _console("on", "--robot", host, home=tmp_path)
    assert result.returncode != 0


def test_session_is_stopped_before_the_switch_and_the_mode_is_rechecked(tmp_path):
    env, log = _robot(tmp_path, running=FIELD + " powertrain_session")
    assert _run(env, "on").returncode == 0
    lines = log.read_text().splitlines()
    stop = next(i for i, l in enumerate(lines) if l.startswith("docker stop powertrain_session"))
    up = next(i for i, l in enumerate(lines) if l.startswith("docker compose") and " up " in l)
    execs = [i for i, l in enumerate(lines) if l.startswith("docker exec powertrain_chassis")]
    assert stop < up and any(stop < i < up for i in execs)


def test_a_start_that_wins_the_race_is_refused_and_the_session_restored(tmp_path):
    env, log = _robot(tmp_path, running=FIELD + " powertrain_session", mode="IDLE ARMED")
    result = _run(env, "on")
    assert result.returncode != 0
    assert _up(log) == []
    assert any(l.startswith("docker start powertrain_session") for l in log.read_text().splitlines())


def test_concurrent_switch_is_refused(tmp_path):
    import fcntl
    env, log = _robot(tmp_path, running=FIELD)
    lock = tmp_path / "state/powertrain/demo-mode.lock"
    lock.parent.mkdir(parents=True)
    with open(lock, "w") as held:
        fcntl.flock(held, fcntl.LOCK_EX)
        result = _run(env, "on")
    assert result.returncode != 0
    assert _up(log) == []


def test_switch_uses_only_running_opt_ins_not_the_prepared_manifest(tmp_path):
    env, log = _robot(tmp_path, running="powertrain_chassis powertrain_terrain_map")
    manifest = tmp_path / "target/etc/powertrain/integrated-prepared.env"
    manifest.write_text(manifest.read_text() + "INTEGRATED_PROFILES='arm-d435 odrive-autoclear'\n"
                        "INTEGRATED_SERVICES='powertrain_arm_console_bridge powertrain_odrive_autoclear'\n")
    result = _run(env, "on")
    assert result.returncode == 0, result.stderr
    (up,) = _up(log)
    assert "arm-d435" not in up and "odrive-autoclear" not in up and "--profile terrain-map" in up


def test_console_on_with_robot_validates_configs_before_touching_the_robot(tmp_path):
    base = _configs(tmp_path)
    (base / "operator.json").write_text("{broken")
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    log = tmp_path / "ssh.log"
    _write_executable(bin_dir / "ssh", f"#!/bin/sh\nprintf '%s\\n' \"$*\" >>{log}\n")
    result = subprocess.run([sys.executable, str(CONSOLE_DEMO_MODE), "on", "--robot", "192.168.8.106"],
                            cwd=ROOT, env={"HOME": str(tmp_path), "PATH": f"{bin_dir}:/usr/bin:/bin"},
                            text=True, capture_output=True, timeout=30)
    assert result.returncode != 0
    assert not log.exists()


@pytest.mark.parametrize("action", ["on", "off"])
def test_running_services_are_read_only_while_holding_the_switch_lock(tmp_path, action):
    env, log = _robot(tmp_path, running=FIELD + " powertrain_session")
    (tmp_path / "state/powertrain").mkdir(parents=True)
    assert _run(env, action).returncode == 0
    assert "ps-unlocked" not in log.read_text()
