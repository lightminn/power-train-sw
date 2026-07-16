"""Dependency-light WP6-C ownership and launch contracts."""

import ast
from pathlib import Path


REPO = Path(__file__).resolve().parents[4]
PACKAGE = REPO / "ros2/src/powertrain_ros"
NODES = PACKAGE / "powertrain_ros"
AUTONOMY = REPO / "powertrain_autonomy"
CONTROLLER_NODE = NODES / "autonomy_controller_node.py"


def _literal_topics(call):
    return {
        arg.value
        for arg in call.args
        if isinstance(arg, ast.Constant) and isinstance(arg.value, str)
    }


def test_autonomy_controller_never_uses_external_cmd_vel_or_chassis_manager():
    tree = ast.parse(CONTROLLER_NODE.read_text(encoding="utf-8"))
    subscriptions = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "create_subscription"
    ]
    publishers = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "create_publisher"
    ]

    assert all("/cmd_vel" not in _literal_topics(call) for call in subscriptions)
    assert all("/cmd_vel" not in _literal_topics(call) for call in publishers)
    assert any("/autonomy/cmd_vel" in _literal_topics(call) for call in publishers)
    assert not any(
        isinstance(node, ast.Name) and node.id == "ChassisManager"
        for node in ast.walk(tree)
    )


def test_autonomy_core_tree_has_no_ros_or_adapter_imports():
    forbidden = {"rclpy", "powertrain_ros"}
    violations = []
    for path in AUTONOMY.rglob("*.py"):
        if "tests" in path.parts:
            continue
        tree = ast.parse(path.read_text(encoding="utf-8"))
        imports = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.append(node.module)
        if any(name.split(".")[0] in forbidden for name in imports):
            violations.append(path.relative_to(REPO))
    assert violations == []


def test_chassis_node_is_only_ros_process_that_constructs_chassis_manager():
    owners = []
    for path in NODES.glob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        if any(
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "ChassisManager"
            for node in ast.walk(tree)
        ):
            owners.append(path.name)
    assert owners == ["chassis_node.py"]


def test_terrain_guidance_launches_one_process_controller_and_estimator():
    launch = (PACKAGE / "launch/autonomy.launch.py").read_text(encoding="utf-8")
    setup = (PACKAGE / "setup.py").read_text(encoding="utf-8")
    node = CONTROLLER_NODE.read_text(encoding="utf-8")

    assert "terrain_on" in launch
    assert "none | lane | wall | follow | terrain" in launch
    assert 'executable="autonomy_controller"' in launch
    assert '"enabled": LaunchConfiguration("propose")' in launch
    assert "autonomy_controller = powertrain_ros.autonomy_controller_node:main" in setup
    assert "TerrainEstimator" in node
    assert "AutonomyController" in node


def test_depth_callback_only_enqueues_and_worker_owns_processing():
    tree = ast.parse(CONTROLLER_NODE.read_text(encoding="utf-8"))
    methods = {
        item.name: item
        for item in next(
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef)
            and node.name == "AutonomyControllerNode"
        ).body
        if isinstance(item, ast.FunctionDef)
    }

    callback_calls = {
        node.func.attr
        for node in ast.walk(methods["_on_depth"])
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
    }
    callback_name_calls = {
        node.func.id
        for node in ast.walk(methods["_on_depth"])
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
    }
    processor_calls = {
        node.func.attr
        for node in ast.walk(methods["_process_depth_now"])
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
    }

    assert "_decode_depth" not in callback_calls
    assert "update" not in callback_calls
    assert callback_calls <= {"_now_s", "notify"}
    assert callback_name_calls == set()
    assert "_decode_depth" in processor_calls
    assert "update" in processor_calls


def test_worker_and_tick_consume_atomic_reference_snapshots():
    tree = ast.parse(CONTROLLER_NODE.read_text(encoding="utf-8"))
    methods = {
        item.name: item
        for item in next(
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef)
            and node.name == "AutonomyControllerNode"
        ).body
        if isinstance(item, ast.FunctionDef)
    }

    processor_attrs = {
        node.attr
        for node in ast.walk(methods["_process_depth_now"])
        if isinstance(node, ast.Attribute)
    }
    tick_attrs = {
        node.attr
        for node in ast.walk(methods["_tick"])
        if isinstance(node, ast.Attribute)
    }

    assert "_grid_snapshot" in processor_attrs
    assert "_grid_source_shape" not in processor_attrs
    assert "_terrain_snapshot" in tick_attrs
    assert "_terrain" not in tick_attrs
    assert "_terrain_seen" not in tick_attrs
