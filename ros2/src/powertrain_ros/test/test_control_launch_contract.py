"""D5 배포 계약 — control.launch가 teleop+broker를 함께 감독한다."""
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / "launch/control.launch.py"
COMPOSE = PACKAGE.parents[2] / "docker/docker-compose.jetson.yml"


def _launch_actions(filename):
    source = PACKAGE / "launch" / filename
    tree = ast.parse(source.read_text())
    tree.body = [node for node in tree.body if not isinstance(node, (ast.Import, ast.ImportFrom))]
    scope = {
        "LaunchDescription": list,
        "DeclareLaunchArgument": lambda name, **kw: SimpleNamespace(kind="argument", name=name, **kw),
        "LaunchConfiguration": lambda name: SimpleNamespace(kind="configuration", name=name),
        "Node": lambda **kw: SimpleNamespace(kind="node", **kw),
        "GroupAction": lambda **kw: SimpleNamespace(kind="group", **kw),
        "SetParameter": lambda **kw: SimpleNamespace(kind="parameter", **kw),
        "ParameterValue": lambda value, **kw: SimpleNamespace(value=value, **kw),
        "Shutdown": lambda: None,
    }
    exec(compile(tree, str(source), "exec"), scope)
    return scope["generate_launch_description"]()


@pytest.mark.parametrize("filename", ["control.launch.py", "wp5_control.launch.py"])
def test_launch_manual_format_defaults_to_steering_and_forwards_explicit_twist(filename):
    actions = _launch_actions(filename)
    if filename == "wp5_control.launch.py":
        actions = next(action for action in actions if action.kind == "group").actions
    declarations = [action for action in actions if action.kind == "argument" and action.name == "manual_command_format"]
    assert len(declarations) == 1, "manual command format cannot be selected at launch"
    assert declarations[0].default_value == "steering"
    assert set(declarations[0].choices) == {"steering", "twist"}
    if filename == "control.launch.py":
        teleop = next(action for action in actions if action.kind == "node" and action.executable == "teleop_command")
        forwarding = teleop.parameters[0]["manual_command_format"]
    else:
        forwarding = next(action.value for action in actions if action.kind == "parameter" and action.name == "manual_command_format")
    assert forwarding.kind == "configuration"
    assert forwarding.name == "manual_command_format"


def test_manual_yaw_limit_is_forwarded_as_float_to_chassis_group():
    actions = next(action for action in _launch_actions("wp5_control.launch.py") if action.kind == "group").actions
    values = [action.value for action in actions if action.kind == "parameter" and action.name == "manual_max_angular"]
    assert len(values) == 1
    assert values[0].value_type is float
    assert values[0].value.name == "manual_max_angular"


def test_control_launch_runs_teleop_and_broker_as_separate_nodes():
    source = LAUNCH.read_text(encoding="utf-8")
    assert 'executable="teleop_command"' in source
    assert 'executable="ops_broker"' in source
    assert '"token_dir"' in source


def test_compose_control_service_uses_launch_and_checks_both_ports():
    source = COMPOSE.read_text(encoding="utf-8")
    assert "control.launch.py" in source
    assert "9001" in source


def test_compose_chassis_service_is_persistent_and_explicit_about_stop_mm():
    # 2026-07-18 사용자 결정: 벤치 상시 chassis 스택. stop_mm 은 env 명시
    # 전달만(부재 시 기동 거부), healthcheck 는 래퍼가 아닌 실제 노드
    # 프로세스를 본다(cmdline grep 함정 회피).
    source = COMPOSE.read_text(encoding="utf-8")
    assert "powertrain_chassis:" in source
    assert "wp5_control.launch.py stop_mm:=$$STOP_MM" in source
    assert "STOP_MM missing" in source
    assert "us100_safety" in source


def test_compose_chassis_enables_command_authority():
    source = COMPOSE.read_text(encoding="utf-8")
    assert (
        "wp5_control.launch.py stop_mm:=$$STOP_MM "
        "authority_enabled:=true"
    ) in source


def test_wp5_launch_shuts_down_whole_stack_when_any_node_dies():
    # 반쪽 생존 금지(07-18 실측: chassis만 죽고 us100은 살아 unhealthy 방치)
    # — 노드 사망 = launch 종료 = compose restart 전체 복구.
    source = (PACKAGE / "launch/wp5_control.launch.py").read_text(
        encoding="utf-8"
    )
    assert source.count("on_exit=Shutdown()") == 2
