"""Pure fake tests for the chassis-owned contract-v2 supervisor adapter."""

import ast
from pathlib import Path
import sys
from types import SimpleNamespace

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[2]
sys.path.insert(0, str(REPO / "motor_control"))

from chassis.mission import (  # noqa: E402
    EVENT_HOLD,
    MODE_MISSION_STOP,
    MODE_STOW_REQUEST,
    READY,
    SupervisorResult,
)


CHASSIS_NODE = PACKAGE / "powertrain_ros" / "chassis_node.py"
MISSION_NODE = PACKAGE / "powertrain_ros" / "mission_node.py"
COMPOSE = REPO / "docker" / "docker-compose.jetson.yml"
INSTALL_RUNTIME_DIR = REPO / "scripts" / "install_powertrain_runtime_dir.sh"


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


def _node_class(*method_names, globals_=None):
    methods = [
        _method_ast(CHASSIS_NODE, "ChassisNode", name)
        for name in method_names
    ]
    cls = ast.ClassDef(
        name="FakeChassisNode",
        bases=[],
        keywords=[],
        body=methods,
        decorator_list=[],
    )
    module = ast.Module(body=[cls], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = dict(globals_ or {})
    exec(compile(module, str(CHASSIS_NODE), "exec"), namespace)
    return namespace["FakeChassisNode"]


class _Logger:
    def __init__(self):
        self.messages = []

    def warning(self, message):
        self.messages.append(("warning", message))

    def error(self, message):
        self.messages.append(("error", message))

    def info(self, message):
        self.messages.append(("info", message))


class _Interlock:
    def __init__(self):
        self.calls = []

    def set_motion_hold(self, source, active, detail=""):
        self.calls.append((source, active, detail))


def test_chassis_initializes_supervisor_only_inside_verified_v2_boundary():
    source = CHASSIS_NODE.read_text(encoding="utf-8")

    assert 'declare_parameter("mission_id_path", "/var/lib/powertrain/mission_id")' in source
    assert "MissionIdStore" in source
    assert "MissionSupervisor" in source
    assert "self._mission_supervisor_enabled = self._contract_v2_verified" in source
    assert "if self._mission_supervisor_enabled:" in source
    assert "wheel_stop=self._wheel_stop" in source
    assert "clear_grip_lost=self._arm_interlock.clear_grip_lost" in source


def test_arrival_qos_is_explicit_reliable_keep_last_10_volatile():
    source = CHASSIS_NODE.read_text(encoding="utf-8")

    assert "arrival_qos = QoSProfile(" in source
    assert "depth=10" in source
    assert "reliability=ReliabilityPolicy.RELIABLE" in source
    assert "durability=DurabilityPolicy.VOLATILE" in source
    assert "contract.TOPIC_ARRIVAL,\n                arrival_qos," in source


def test_apply_result_sets_mission_hold_and_mode_before_arrival_publish():
    Node = _node_class(
        "_set_mission_motion_hold",
        "_apply_mission_result",
        globals_={
            "contract": SimpleNamespace(
                MODE_MISSION_STOP=MODE_MISSION_STOP,
                MODE_STOW_REQUEST=MODE_STOW_REQUEST,
            )
        },
    )
    node = Node()
    node._mission_supervisor_enabled = True
    node._arm_gate_mode = "production"
    interlock = _Interlock()
    node.cm = SimpleNamespace(_interlock=interlock, set_motion_hold=interlock.set_motion_hold)
    node._authority_final_v = 0.4
    node._authority_final_omega = -0.2
    events = []
    node.set_chassis_mode_intent = lambda mode: events.append(("mode", mode)) or True
    node.publish_arrival = lambda mission_id, status: events.append(
        ("arrival", mission_id, status)
    ) or True
    node.get_logger = lambda: _Logger()
    result = SupervisorResult(
        state=EVENT_HOLD,
        mode_intent=MODE_MISSION_STOP,
        publish_arrival=(3, "ARRIVED_PICKUP"),
        hold_reason="arrival_ack_pending",
    )

    assert node._apply_mission_result(result) is True

    assert events == [
        ("mode", MODE_MISSION_STOP),
        ("arrival", 3, "ARRIVED_PICKUP"),
    ]
    assert node.cm._interlock.calls == [
        ("mission", True, "arrival_ack_pending"),
    ]
    assert (node._authority_final_v, node._authority_final_omega) == (0.0, 0.0)


def test_arm_absent_field_never_applies_mission_stop_or_arrival():
    Node = _node_class(
        "_set_mission_motion_hold",
        "_apply_mission_result",
        globals_={
            "contract": SimpleNamespace(
                MODE_MISSION_STOP=MODE_MISSION_STOP,
                MODE_STOW_REQUEST=MODE_STOW_REQUEST,
            )
        },
    )
    node = Node()
    node._mission_supervisor_enabled = True
    node._arm_gate_mode = "arm_absent_field"
    interlock = _Interlock()
    node.cm = SimpleNamespace(_interlock=interlock, set_motion_hold=interlock.set_motion_hold)
    node._authority_final_v = 0.0
    node._authority_final_omega = 0.0
    modes = []
    arrivals = []
    node.set_chassis_mode_intent = lambda mode: modes.append(mode) or True
    node.publish_arrival = lambda *event: arrivals.append(event) or True
    node.get_logger = lambda: _Logger()
    result = SupervisorResult(
        state=EVENT_HOLD,
        mode_intent=MODE_MISSION_STOP,
        publish_arrival=(4, "ARRIVED_DROP"),
    )

    assert node._apply_mission_result(result) is False

    assert modes == [MODE_STOW_REQUEST]
    assert arrivals == []
    assert node.cm._interlock.calls[-1][0:2] == ("mission", True)


def test_arm_absent_field_rejects_event_before_supervisor_can_allocate_work():
    Node = _node_class(
        "_request_supervisor_work",
        globals_={"ARM_GATE_ABSENT_FIELD": "arm_absent_field"},
    )
    calls = []
    node = Node()
    node._mission_supervisor_enabled = True
    node._arm_gate_mode = "arm_absent_field"
    node._mission_supervisor = SimpleNamespace(
        request_work=lambda *_args: calls.append(_args)
    )
    response = SimpleNamespace(success=None, message="")

    returned = node._request_supervisor_work("ARRIVED_PICKUP", response)

    assert returned.success is False
    assert calls == []


def test_arm_absent_field_idle_supervisor_does_not_block_powertrain_only_drive():
    Node = _node_class(
        "_set_mission_motion_hold",
        "_apply_mission_result",
        globals_={
            "contract": SimpleNamespace(
                MODE_MISSION_STOP=MODE_MISSION_STOP,
                MODE_STOW_REQUEST=MODE_STOW_REQUEST,
            )
        },
    )
    node = Node()
    node._mission_supervisor_enabled = True
    node._arm_gate_mode = "arm_absent_field"
    interlock = _Interlock()
    node.cm = SimpleNamespace(_interlock=interlock, set_motion_hold=interlock.set_motion_hold)
    node._authority_final_v = 0.2
    node._authority_final_omega = 0.0
    modes = []
    node.set_chassis_mode_intent = lambda mode: modes.append(mode) or True
    node.publish_arrival = lambda *_event: True
    node.get_logger = lambda: _Logger()

    applied = node._apply_mission_result(
        SupervisorResult(
            state=READY,
            mode_intent=MODE_STOW_REQUEST,
            hold_reason="fresh_locked_posture_required",
        )
    )

    assert applied is False
    assert modes == [MODE_STOW_REQUEST]
    assert node.cm._interlock.calls == [("mission", False, "")]
    assert node._authority_final_v == 0.2


def test_supervisor_exception_is_converted_to_hold_without_escaping_tick():
    Node = _node_class(
        "_set_mission_motion_hold",
        "_mission_supervisor_failure",
        "_tick_mission_supervisor",
        globals_={"contract": SimpleNamespace(MODE_STOW_REQUEST=MODE_STOW_REQUEST)},
    )
    node = Node()
    node._mission_supervisor_enabled = True
    node._mission_supervisor = SimpleNamespace(
        tick=lambda _now: (_ for _ in ()).throw(RuntimeError("injected"))
    )
    interlock = _Interlock()
    node.cm = SimpleNamespace(_interlock=interlock, set_motion_hold=interlock.set_motion_hold)
    node._authority_final_v = 0.3
    node._authority_final_omega = 0.1
    node.set_chassis_mode_intent = lambda _mode: True
    logger = _Logger()
    node.get_logger = lambda: logger

    assert node._tick_mission_supervisor(12.0) is False

    assert node.cm._interlock.calls[-1][0:2] == ("mission", True)
    assert any(level == "error" for level, _message in logger.messages)


def test_adapter_apply_exception_is_also_converted_without_escaping_tick():
    Node = _node_class(
        "_tick_mission_supervisor",
        globals_={},
    )
    node = Node()
    node._mission_supervisor_enabled = True
    node._mission_supervisor = SimpleNamespace(
        tick=lambda _now: SupervisorResult(state=EVENT_HOLD)
    )
    node._apply_mission_result = lambda _result: (_ for _ in ()).throw(
        RuntimeError("apply injected")
    )
    failures = []
    node._mission_supervisor_failure = lambda detail: failures.append(detail) or False

    assert node._tick_mission_supervisor(12.0) is False

    assert failures == ["mission_tick_exception:apply injected"]


def test_override_aborts_supervisor_before_setting_override_flag():
    Node = _node_class(
        "_srv_arm_lock_override",
        globals_={"contract": SimpleNamespace(MODE_STOW_REQUEST=MODE_STOW_REQUEST)},
    )
    node = Node()
    events = []
    node._arm_override_requested = False
    node._mission_supervisor_enabled = True
    supervisor = SimpleNamespace(
        arrival_republish_active=True,
        mode_intent=MODE_STOW_REQUEST,
    )

    def abort(now):
        events.append(("abort", now))
        supervisor.arrival_republish_active = False
        return True

    supervisor.abort_for_override = abort
    node._mission_supervisor = supervisor
    node._now_s = lambda: 22.0
    node._override_activation_error = lambda _now: ""
    node.set_chassis_mode_intent = lambda mode: events.append(("mode", mode)) or True
    node._discard_pending_command = lambda: events.append(("discard",))
    response = SimpleNamespace(success=None, message="")

    returned = node._srv_arm_lock_override(SimpleNamespace(data=True), response)

    assert returned.success is True
    assert node._arm_override_requested is True
    assert events == [
        ("abort", 22.0),
        ("mode", MODE_STOW_REQUEST),
        ("discard",),
    ]


def test_override_abort_exception_is_rejected_and_converted_to_hold():
    Node = _node_class(
        "_srv_arm_lock_override",
        globals_={"contract": SimpleNamespace(MODE_STOW_REQUEST=MODE_STOW_REQUEST)},
    )
    node = Node()
    node._arm_override_requested = False
    node._mission_supervisor_enabled = True
    node._mission_supervisor = SimpleNamespace(
        abort_for_override=lambda _now: (_ for _ in ()).throw(
            RuntimeError("abort injected")
        ),
        arrival_republish_active=True,
    )
    node._now_s = lambda: 22.0
    node._override_activation_error = lambda _now: ""
    failures = []
    node._mission_supervisor_failure = lambda detail: failures.append(detail) or False
    response = SimpleNamespace(success=None, message="")

    returned = node._srv_arm_lock_override(SimpleNamespace(data=True), response)

    assert returned.success is False
    assert node._arm_override_requested is False
    assert failures == ["mission_override_abort_exception:abort injected"]


def test_legacy_mission_writer_requires_explicit_exclusive_owner_parameter():
    source = MISSION_NODE.read_text(encoding="utf-8")

    assert "SUPERSEDED" in source
    assert 'declare_parameter("mission_contract_owner", "chassis_supervisor")' in source
    assert "legacy_mission_node" in source
    assert "contract_output_enabled=true requires " in source
    assert "mission_contract_owner=legacy_mission_node" in source


def test_ros_and_control_services_share_preprovisioned_persistent_id_directory():
    compose = yaml.safe_load(COMPOSE.read_text(encoding="utf-8"))

    for service_name in ("powertrain_ros", "powertrain_control"):
        mounts = compose["services"][service_name]["volumes"]
        persistent = [
            mount
            for mount in mounts
            if isinstance(mount, dict)
            and mount.get("source") == "/var/lib/powertrain"
        ]
        assert persistent == [
            {
                "type": "bind",
                "source": "/var/lib/powertrain",
                "target": "/var/lib/powertrain",
                "bind": {"create_host_path": False},
            }
        ]

    source = COMPOSE.read_text(encoding="utf-8")
    assert "install_powertrain_runtime_dir.sh provisions this persistent path" in source


def test_install_script_creates_and_verifies_persistent_directory_without_tmpfiles():
    source = INSTALL_RUNTIME_DIR.read_text(encoding="utf-8")

    assert 'readonly PERSISTENT_DIR="/var/lib/powertrain"' in source
    assert 'install -d -o root -g root -m 0750 "$PERSISTENT_DIR"' in source
    assert 'stat -c \'%U:%G:%a:%F\' "$PERSISTENT_DIR"' in source
    assert "$PERSISTENT_DIR must be root:root mode 0750 directory" in source
