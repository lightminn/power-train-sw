import ast
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def test_config_pins_exact_l515_and_stream_contract():
    config = yaml.safe_load((PACKAGE / "config/l515.yaml").read_text())
    params = config["l515_camera_node"]["ros__parameters"]
    assert params == {
        "serial": "00000000F0271544",
        "width": 640,
        "height": 480,
        "fps": 30,
        "reconnect_interval": 2.0,
    }


def test_retired_launch_fails_closed_without_constructing_ros_node(monkeypatch):
    launch_file = PACKAGE / "launch/l515.launch.py"
    spec = importlib.util.spec_from_file_location("l515_launch", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "Node", lambda **kwargs: (_ for _ in ()).throw(
        AssertionError("ROS node must not be constructed")))
    with __import__('pytest').raises(RuntimeError, match="l515_dashboard.gateway_main"):
        module.generate_launch_description()


def test_setup_does_not_install_retired_launch_or_console_entry_point():
    tree = ast.parse((PACKAGE / "setup.py").read_text())
    source = ast.unparse(tree)
    assert "config/l515.yaml" in source
    assert "launch/l515.launch.py" not in source
    assert "l515_camera = powertrain_ros.l515_node:main" not in source
    package_xml = (PACKAGE / "package.xml").read_text()
    assert "<exec_depend>sensor_msgs</exec_depend>" in package_xml
    docs = (PACKAGE.parents[2] / "ros2/README.md").read_text()
    assert "l515_camera` console entry point" in docs
    assert "fail-closed" in docs
