import importlib.util
import os
from pathlib import Path
import subprocess

import pytest
from launch.action import Action
from launch.actions import DeclareLaunchArgument
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


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


def test_hardware_launch_passes_stop_mm_to_us100_node(monkeypatch):
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
    assert isinstance(stop_mm, LaunchConfiguration)
    context = LaunchContext()
    context.launch_configurations["stop_mm"] = "321"
    assert stop_mm.perform(context) == "321"
    assert "parameters" not in chassis.kwargs


def test_can_setup_disables_loopback_and_retains_bus_parameters(tmp_path):
    log_path = tmp_path / "sudo.log"
    sudo_stub = tmp_path / "sudo"
    sudo_stub.write_text(
        "#!/bin/sh\n"
        "printf '%s\\n' \"$*\" >> \"$CAN_SETUP_LOG\"\n",
        encoding="utf-8",
    )
    sudo_stub.chmod(0o755)
    env = os.environ.copy()
    env["PATH"] = f"{tmp_path}:{env['PATH']}"
    env["CAN_SETUP_LOG"] = str(log_path)

    subprocess.run(
        ["bash", str(CAN_SETUP)],
        check=True,
        env=env,
        capture_output=True,
        text=True,
    )
    commands = log_path.read_text(encoding="utf-8").splitlines()

    assert commands.index("ip link set can0 down") < commands.index(
        "ip link set can0 up type can bitrate 500000 "
        "loopback off restart-ms 100"
    )
    assert "ip link set can0 txqueuelen 1000" in commands


def test_can_setup_fails_closed_when_loopback_configuration_fails(tmp_path):
    log_path = tmp_path / "sudo.log"
    sudo_stub = tmp_path / "sudo"
    sudo_stub.write_text(
        "#!/bin/sh\n"
        "printf '%s\\n' \"$*\" >> \"$CAN_SETUP_LOG\"\n"
        "case \"$*\" in\n"
        "  *'loopback off'*) exit 42 ;;\n"
        "esac\n",
        encoding="utf-8",
    )
    sudo_stub.chmod(0o755)
    env = os.environ.copy()
    env["PATH"] = f"{tmp_path}:{env['PATH']}"
    env["CAN_SETUP_LOG"] = str(log_path)

    result = subprocess.run(
        ["bash", str(CAN_SETUP)],
        check=False,
        env=env,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "세팅 완료" not in result.stdout
    assert "ip link set can0 txqueuelen 1000" not in log_path.read_text(
        encoding="utf-8"
    ).splitlines()
