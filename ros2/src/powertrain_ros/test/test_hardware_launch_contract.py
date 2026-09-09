import importlib.util
import os
from pathlib import Path
import subprocess
import sys

import pytest


REPO_MARKERS = (
    Path("ros2/src/powertrain_ros/launch/wp5_control.launch.py"),
    Path("scripts/can_setup.sh"),
)


def _has_repo_contract(root):
    return all((root / marker).is_file() for marker in REPO_MARKERS)


def _discover_repo_root(start_path=None):
    explicit = os.environ.get("POWERTRAIN_REPO_ROOT")
    if explicit:
        candidate = Path(explicit).expanduser().resolve()
        if _has_repo_contract(candidate):
            return candidate
        raise RuntimeError(
            f"POWERTRAIN_REPO_ROOT is not a power-train-sw checkout: "
            f"{candidate}"
        )

    motor_control = os.environ.get("MOTOR_CONTROL_PATH")
    if motor_control:
        candidate = Path(motor_control).expanduser().resolve().parent
        if _has_repo_contract(candidate):
            return candidate
        raise RuntimeError(
            f"MOTOR_CONTROL_PATH is not inside a power-train-sw checkout: "
            f"{candidate}"
        )

    start = Path(start_path or __file__).resolve()
    for candidate in (start, *start.parents):
        if _has_repo_contract(candidate):
            return candidate

    raise RuntimeError(
        "power-train-sw root not found; set POWERTRAIN_REPO_ROOT "
        "or MOTOR_CONTROL_PATH"
    )


ROOT = _discover_repo_root()
LAUNCH_FILE = (
    ROOT
    / "ros2"
    / "src"
    / "powertrain_ros"
    / "launch"
    / "wp5_control.launch.py"
)
CAN_SETUP = ROOT / "scripts" / "can_setup.sh"
CHASSIS_NODE = (
    ROOT / "ros2/src/powertrain_ros/powertrain_ros/chassis_node.py"
)
OBSTACLE_ZONES_NODE = (
    ROOT / "ros2/src/powertrain_ros/powertrain_ros/obstacle_zones_node.py"
)


def _load_launch_module(module_name):
    spec = importlib.util.spec_from_file_location(module_name, LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _make_repo_contract(root):
    for marker in REPO_MARKERS:
        path = root / marker
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()


def test_repo_root_discovery_uses_explicit_override(tmp_path, monkeypatch):
    repo = tmp_path / "explicit-repo"
    _make_repo_contract(repo)
    monkeypatch.setenv("POWERTRAIN_REPO_ROOT", str(repo))
    monkeypatch.setenv(
        "MOTOR_CONTROL_PATH",
        str(tmp_path / "different-repo" / "motor_control"),
    )

    assert _discover_repo_root(tmp_path / "isolated" / "test.py") == repo


def test_repo_root_discovery_uses_motor_control_parent(
    tmp_path,
    monkeypatch,
):
    repo = tmp_path / "motor-repo"
    _make_repo_contract(repo)
    monkeypatch.delenv("POWERTRAIN_REPO_ROOT", raising=False)
    monkeypatch.setenv("MOTOR_CONTROL_PATH", str(repo / "motor_control"))

    assert _discover_repo_root(tmp_path / "isolated" / "test.py") == repo


def test_repo_root_discovery_rejects_stale_motor_control_path(
    tmp_path,
    monkeypatch,
):
    repo = tmp_path / "valid-source-repo"
    _make_repo_contract(repo)
    monkeypatch.delenv("POWERTRAIN_REPO_ROOT", raising=False)
    monkeypatch.setenv(
        "MOTOR_CONTROL_PATH",
        str(tmp_path / "stale-repo" / "motor_control"),
    )
    nested_test = repo / "ros2/src/powertrain_ros/test/test_contract.py"

    with pytest.raises(RuntimeError, match="MOTOR_CONTROL_PATH"):
        _discover_repo_root(nested_test)


def test_repo_root_discovery_falls_back_to_source_ancestors(
    tmp_path,
    monkeypatch,
):
    repo = tmp_path / "ancestor-repo"
    _make_repo_contract(repo)
    monkeypatch.delenv("POWERTRAIN_REPO_ROOT", raising=False)
    monkeypatch.delenv("MOTOR_CONTROL_PATH", raising=False)
    nested_test = repo / "ros2/src/powertrain_ros/test/test_contract.py"

    assert _discover_repo_root(nested_test) == repo


def test_hardware_launch_requires_stop_mm_without_default():
    from launch.actions import DeclareLaunchArgument
    from launch import LaunchContext

    description = _load_launch_module("wp5_control_required_stop_mm") \
        .generate_launch_description()
    arguments = [
        entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
        and entity.name == "stop_mm"
    ]

    assert len(arguments) == 1
    assert arguments[0].default_value is None
    with pytest.raises(RuntimeError, match="stop_mm.*not.*provided"):
        arguments[0].execute(LaunchContext())


def test_bench_obstacle_hint_cannot_feed_production_chassis():
    chassis_source = CHASSIS_NODE.read_text(encoding="utf-8")
    obstacle_source = OBSTACLE_ZONES_NODE.read_text(encoding="utf-8")

    assert '"/obstacle/speed_scale"' not in chassis_source
    assert '"/diagnostics/obstacle/speed_scale"' in obstacle_source


@pytest.mark.parametrize(
    ("launch_value", "expected"),
    (("200", 200.0), ("321.5", 321.5)),
)
def test_hardware_launch_passes_stop_mm_to_us100_node(
    monkeypatch,
    launch_value,
    expected,
):
    from launch.action import Action
    from launch import LaunchContext
    import launch_ros.actions
    from launch_ros.parameter_descriptions import ParameterValue

    recorded_nodes = []

    class RecordingNode(Action):
        def __init__(self, **kwargs):
            super().__init__()
            self.kwargs = kwargs
            recorded_nodes.append(self)

    monkeypatch.setattr(launch_ros.actions, "Node", RecordingNode)
    description = _load_launch_module("wp5_control_wired_stop_mm") \
        .generate_launch_description()
    us100 = next(
        node
        for node in recorded_nodes
        if node.kwargs["executable"] == "us100_safety"
    )
    chassis = next(
        node
        for node in recorded_nodes
        if node.kwargs["executable"] == "chassis"
    )

    assert len(description.entities) == 3
    assert len(recorded_nodes) == 2
    parameters = us100.kwargs["parameters"]
    assert len(parameters) == 1
    stop_mm = parameters[0]["stop_mm"]
    assert isinstance(stop_mm, ParameterValue)
    context = LaunchContext()
    context.launch_configurations["stop_mm"] = launch_value
    evaluated = stop_mm.evaluate(context)
    assert evaluated == expected
    assert isinstance(evaluated, float)
    assert "parameters" not in chassis.kwargs


def _can_setup_environment(tmp_path, *, fail_modes=False):
    """Execute the real shell + common CLI; fake only external OS commands."""
    binary = tmp_path / "bin"
    binary.mkdir()
    state = tmp_path / "link-state"
    state.mkdir()
    runtime = tmp_path / "root/run/powertrain"
    runtime.mkdir(parents=True, mode=0o750)
    log_path = tmp_path / "commands.log"
    commands = {
        "sudo": '''#!/bin/sh
printf 'sudo %s\\n' "$*" >> "$CAN_SETUP_LOG"
exec "$@"
''',
        "modprobe": '''#!/bin/sh
printf 'modprobe %s\\n' "$*" >> "$CAN_SETUP_LOG"
''',
        "busybox": '''#!/bin/sh
[ "$1" = devmem ] || exit 97
shift
exec devmem "$@"
''',
        "devmem": '''#!/bin/sh
printf 'devmem %s\\n' "$*" >> "$CAN_SETUP_LOG"
''',
        "ip": '''#!/bin/sh
set -eu
printf 'ip %s\\n' "$*" >> "$CAN_SETUP_LOG"
case "$*" in
 '-details link show can0')
   [ -f "$CAN_SETUP_STATE/up" ] || exit 1
   printf '%s\\n' \\
     '5: can0: <NOARP,UP,LOWER_UP> state UNKNOWN qlen 1000' \\
     '    can state ERROR-ACTIVE restart-ms 100' \\
     '    bitrate 500000';;
 'link set can0 down') : > "$CAN_SETUP_STATE/down";;
 'link set can0 type can bitrate 500000 loopback off listen-only off restart-ms 100')
   [ -f "$CAN_SETUP_STATE/down" ] || exit 98
   [ "$CAN_SETUP_FAIL_MODES" != 1 ] || exit 42
   : > "$CAN_SETUP_STATE/modes";;
 'link set can0 txqueuelen 1000') : > "$CAN_SETUP_STATE/queue";;
 'link set can0 up')
   [ -f "$CAN_SETUP_STATE/modes" ] && [ -f "$CAN_SETUP_STATE/queue" ] || exit 99
   : > "$CAN_SETUP_STATE/up";;
 *) exit 97;;
esac
''',
    }
    for name, source in commands.items():
        path = binary / name
        path.write_text(source, encoding="utf-8")
        path.chmod(0o755)
    # can_setup.sh calls python3; keep this subprocess on the test interpreter.
    (binary / "python3").symlink_to(sys.executable)
    env = os.environ.copy()
    env.pop("BASH_ENV", None)
    env.pop("ENV", None)
    env.update({
        "PATH": f"{binary}:/usr/bin:/bin",
        "POWERTRAIN_ROOT": str(tmp_path / "root"),
        "CAN_SETUP_LOG": str(log_path),
        "CAN_SETUP_STATE": str(state),
        "CAN_SETUP_FAIL_MODES": "1" if fail_modes else "0",
    })
    return env, log_path, state


def test_can_setup_disables_loopback_and_retains_bus_parameters(tmp_path):
    env, log_path, state = _can_setup_environment(tmp_path)

    result = subprocess.run(
        ["bash", str(CAN_SETUP)],
        check=False,
        env=env,
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    commands = log_path.read_text(encoding="utf-8").splitlines()

    ordered = [
        "ip link set can0 down",
        "ip link set can0 type can bitrate 500000 "
        "loopback off listen-only off restart-ms 100",
        "ip link set can0 txqueuelen 1000",
        "ip link set can0 up",
    ]
    indices = [commands.index(command) for command in ordered]
    assert indices == sorted(indices)
    assert commands[-1] == "ip -details link show can0"
    assert commands.count("ip -details link show can0") == 3
    assert (state / "up").is_file()
    assert "500000 bps" in result.stdout
    assert "LOOPBACK, LISTEN-ONLY off" in result.stdout


def test_can_setup_fails_closed_when_loopback_configuration_fails(tmp_path):
    env, log_path, state = _can_setup_environment(tmp_path, fail_modes=True)

    result = subprocess.run(
        ["bash", str(CAN_SETUP)],
        check=False,
        env=env,
        capture_output=True,
        text=True,
        timeout=10,
    )

    assert result.returncode != 0
    assert "CAN FAIL" in result.stderr
    assert "42" in result.stderr
    assert "500000 bps" not in result.stdout
    commands = log_path.read_text(encoding="utf-8").splitlines()
    assert "ip link set can0 down" in commands
    assert "ip link set can0 type can bitrate 500000 " \
           "loopback off listen-only off restart-ms 100" in commands
    assert "ip link set can0 txqueuelen 1000" not in commands
    assert "ip link set can0 up" not in commands
    assert not (state / "up").exists()
