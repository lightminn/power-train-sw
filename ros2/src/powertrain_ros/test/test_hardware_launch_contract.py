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


ROOT = Path(__file__).resolve().parents[4]
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
