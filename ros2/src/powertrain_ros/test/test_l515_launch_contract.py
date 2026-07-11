import ast
import importlib.util
from pathlib import Path

from launch_ros.actions import Node
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def test_config_pins_exact_l515_and_stream_contract():
    config = yaml.safe_load((PACKAGE / "config/l515.yaml").read_text())
    params = config["l515_camera"]["ros__parameters"]
    assert params == {
        "serial": "00000000F0271544",
        "width": 640,
        "height": 480,
        "fps": 30,
        "reconnect_interval": 2.0,
    }


def test_launch_contains_one_powertrain_l515_node_using_config():
    launch_file = PACKAGE / "launch/l515.launch.py"
    spec = importlib.util.spec_from_file_location("l515_launch", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    nodes = [entity for entity in description.entities if isinstance(entity, Node)]
    assert len(nodes) == 1
    assert nodes[0].node_package == "powertrain_ros"
    assert nodes[0].node_executable == "l515_camera"
    assert nodes[0]._Node__node_name == "l515_camera"
    assert len(nodes[0]._Node__parameters) == 1


def test_setup_installs_config_launch_and_registers_entry_point():
    tree = ast.parse((PACKAGE / "setup.py").read_text())
    source = ast.unparse(tree)
    assert "config/l515.yaml" in source
    assert "launch/l515.launch.py" in source
    assert (
        "l515_camera = powertrain_ros.l515_node:main" in source
    )
    package_xml = (PACKAGE / "package.xml").read_text()
    assert "<exec_depend>sensor_msgs</exec_depend>" in package_xml
