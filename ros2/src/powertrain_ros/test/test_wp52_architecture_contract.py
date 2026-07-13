"""WP5.2 command/mission boundary regression tests.

These tests parse the ROS adapters so the opt-in safety boundary can be
checked without constructing hardware-owning nodes.
"""

import ast
from pathlib import Path
from types import SimpleNamespace


PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[2]
NODES = PACKAGE / "powertrain_ros"
CHASSIS_NODE = NODES / "chassis_node.py"
MISSION_NODE = NODES / "mission_node.py"


def _method_ast(path, class_name, method_name):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    cls = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    return next(
        node
        for node in cls.body
        if isinstance(node, ast.FunctionDef) and node.name == method_name
    )


def _load_method(path, class_name, method_name, globals_=None):
    method = _method_ast(path, class_name, method_name)
    module = ast.Module(body=[method], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = dict(globals_ or {})
    exec(compile(module, str(path), "exec"), namespace)
    return namespace[method_name]


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _ForbiddenPublisher:
    def publish(self, _message):
        raise AssertionError("contract output must not be published")


class _DataMessage:
    def __init__(self, data=None):
        self.data = data


class _ArrivalMessage:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None)
        self.mission_id = 0
        self.status = ""


def test_standalone_command_authority_node_and_entry_point_are_removed():
    assert not (NODES / "command_authority_node.py").exists()
    setup_source = (PACKAGE / "setup.py").read_text(encoding="utf-8")
    assert "command_authority_node" not in setup_source
    assert '"command_authority =' not in setup_source


def test_chassis_authority_input_is_opt_in_and_mutually_exclusive():
    source = CHASSIS_NODE.read_text(encoding="utf-8")
    assert 'declare_parameter("authority_enabled", False)' in source

    initialize = _method_ast(CHASSIS_NODE, "ChassisNode", "_initialize")
    guard = next(
        node
        for node in ast.walk(initialize)
        if isinstance(node, ast.If)
        and ast.unparse(node.test) == "self._authority_enabled"
    )
    enabled = ast.unparse(ast.Module(body=guard.body, type_ignores=[]))
    disabled = ast.unparse(ast.Module(body=guard.orelse, type_ignores=[]))

    assert "/teleop/cmd_vel" in enabled
    assert "/autonomy/cmd_vel" in enabled
    assert "~/authority_manual" in enabled
    assert "~/authority_auto" in enabled
    assert "~/authority_idle" in enabled
    assert disabled.count("'/cmd_vel'") == 1


def test_chassis_never_republishes_ros_cmd_vel():
    tree = ast.parse(CHASSIS_NODE.read_text(encoding="utf-8"))
    cmd_vel_publishers = [
        call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == "create_publisher"
        and any(
            isinstance(arg, ast.Constant) and arg.value == "/cmd_vel"
            for arg in call.args
        )
    ]
    assert cmd_vel_publishers == []


def test_authority_tick_sets_only_an_accepted_selection():
    tick = _load_method(
        CHASSIS_NODE,
        "ChassisNode",
        "_tick_authority",
        {"String": _DataMessage},
    )
    calls = []
    state = _Publisher()
    node = SimpleNamespace(
        _authority=SimpleNamespace(
            mode="AUTO",
            select=lambda _now: SimpleNamespace(
                ok=True,
                v=0.42,
                omega=-0.17,
                reason="auto",
            ),
        ),
        pub_authority_state=state,
        cm=SimpleNamespace(set=lambda v, omega: calls.append((v, omega))),
    )

    tick(node, 12.5)

    assert calls == [(0.42, -0.17)]
    assert state.messages[0].data == "AUTO|auto"


def test_authority_tick_does_not_set_when_selection_is_stale():
    tick = _load_method(
        CHASSIS_NODE,
        "ChassisNode",
        "_tick_authority",
        {"String": _DataMessage},
    )
    calls = []
    state = _Publisher()
    node = SimpleNamespace(
        _authority=SimpleNamespace(
            mode="AUTO",
            select=lambda _now: SimpleNamespace(
                ok=False,
                v=0.0,
                omega=0.0,
                reason="auto stale (0.31s)",
            ),
        ),
        pub_authority_state=state,
        cm=SimpleNamespace(set=lambda v, omega: calls.append((v, omega))),
    )

    tick(node, 12.5)

    assert calls == []
    assert state.messages[0].data == "AUTO|auto stale (0.31s)"


def test_mission_contract_output_is_disabled_by_default_and_guarded():
    source = MISSION_NODE.read_text(encoding="utf-8")
    assert 'declare_parameter("contract_output_enabled", False)' in source

    initialize = _method_ast(MISSION_NODE, "MissionNode", "__init__")
    guard = next(
        node
        for node in ast.walk(initialize)
        if isinstance(node, ast.If)
        and ast.unparse(node.test) == "self._contract_output_enabled"
    )
    enabled = ast.unparse(ast.Module(body=guard.body, type_ignores=[]))
    disabled = ast.unparse(ast.Module(body=guard.orelse, type_ignores=[]))
    assert "create_publisher" in enabled
    assert "TOPIC_ARRIVAL" in enabled
    assert "pub_arrival = None" in disabled


def _mission_tick_node(contract_output_enabled, arrival_publisher):
    decision = SimpleNamespace(
        chassis_mode="MISSION_STOP",
        allow_drive=False,
        state="WAITING",
        reason="stopped",
        publish_arrival=(7, "ARRIVED_PICKUP"),
    )
    return SimpleNamespace(
        seq=SimpleNamespace(update=lambda _now, _speed: decision),
        _now=lambda: 1.0,
        _speed=0.0,
        _last=None,
        pub_mode=_Publisher(),
        pub_allow=_Publisher(),
        pub_state=_Publisher(),
        _contract_output_enabled=contract_output_enabled,
        pub_arrival=arrival_publisher,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: "stamp")
        ),
        get_logger=lambda: SimpleNamespace(warn=lambda _message: None),
    )


def test_mission_tick_drops_contract_event_when_output_is_disabled():
    tick = _load_method(
        MISSION_NODE,
        "MissionNode",
        "_tick",
        {
            "String": _DataMessage,
            "Bool": _DataMessage,
            "ArrivalStatus": _ArrivalMessage,
        },
    )
    node = _mission_tick_node(False, _ForbiddenPublisher())

    tick(node)

    assert node._last.publish_arrival == (7, "ARRIVED_PICKUP")


def test_mission_tick_publishes_contract_event_only_when_enabled():
    tick = _load_method(
        MISSION_NODE,
        "MissionNode",
        "_tick",
        {
            "String": _DataMessage,
            "Bool": _DataMessage,
            "ArrivalStatus": _ArrivalMessage,
        },
    )
    arrival = _Publisher()
    node = _mission_tick_node(True, arrival)

    tick(node)

    assert len(arrival.messages) == 1
    assert arrival.messages[0].mission_id == 7
    assert arrival.messages[0].status == "ARRIVED_PICKUP"


def test_python_sources_use_wp52_command_topics_only():
    legacy_auto = "/cmd_vel" + "/auto"
    legacy_teleop = "/cmd_vel" + "/teleop"
    stale = []
    for path in REPO.rglob("*.py"):
        source = path.read_text(encoding="utf-8")
        if legacy_auto in source or legacy_teleop in source:
            stale.append(path.relative_to(REPO))
    assert stale == []


def test_followers_publish_autonomy_topic_and_name_chassis_services():
    for name in (
        "lane_follower_node.py",
        "wall_follower_node.py",
        "lead_follower_node.py",
    ):
        source = (NODES / name).read_text(encoding="utf-8")
        assert '"/autonomy/cmd_vel"' in source
        assert "/chassis_node/authority_auto" in source


def test_autonomy_launch_uses_embedded_authority():
    source = (PACKAGE / "launch/autonomy.launch.py").read_text(encoding="utf-8")
    assert 'executable="command_authority"' not in source
    assert '"authority_enabled": True' in source
    assert "/chassis_node/authority_auto" in source


def test_can_lock_documents_real_can_session_replacement():
    source = (REPO / "motor_control/corner_module/can_lock.py").read_text(
        encoding="utf-8"
    )
    assert "임시 보호막이다" in source
    assert "RealCanSession" in source
    assert "/run/powertrain/can0.lock" in source
    assert "owner snapshot" in source
    assert "abstract socket의 자동해제 장점" in source
