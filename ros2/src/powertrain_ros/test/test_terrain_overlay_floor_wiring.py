"""Host-runnable contract for the optional floor field's ROS parameter wiring."""
import ast
from pathlib import Path


NODE = Path(__file__).parents[1] / "powertrain_ros" / "terrain_overlay_node.py"


def test_floor_mask_parameter_is_boolean_default_on_and_forwarded_unchanged():
    tree = ast.parse(NODE.read_text(encoding="utf-8"))
    declarations = [
        call for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == "declare_parameter"
        and call.args
        and isinstance(call.args[0], ast.Constant)
        and call.args[0].value == "publish_floor_mask"
    ]
    assert len(declarations) == 1
    assert len(declarations[0].args) == 2
    assert isinstance(declarations[0].args[1], ast.Constant)
    assert declarations[0].args[1].value is True

    runtime_configs = [
        call for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "RuntimeConfig"
    ]
    assert len(runtime_configs) == 1
    keyword = {item.arg: item.value for item in runtime_configs[0].keywords}
    forwarded = keyword["publish_floor_mask"]
    assert isinstance(forwarded, ast.Call)
    assert isinstance(forwarded.func, ast.Name) and forwarded.func.id == "value"
    assert len(forwarded.args) == 1
    assert isinstance(forwarded.args[0], ast.Constant)
    assert forwarded.args[0].value == "publish_floor_mask"
