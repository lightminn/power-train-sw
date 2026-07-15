"""WP5.2 chassis/arm final-gate regressions without constructing a ROS node."""

import ast
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[2]
CHASSIS_NODE = PACKAGE / "powertrain_ros/chassis_node.py"
LAUNCH = PACKAGE / "launch/wp5_control.launch.py"
README = REPO / "ros2/README.md"
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(REPO / "motor_control"))

from chassis.chassis_manager import ChassisConfig, ChassisManager  # noqa: E402
from chassis.kinematics import default_geometry  # noqa: E402
from corner_module.config import CornerConfig  # noqa: E402
from corner_module.corner_module import CornerModule  # noqa: E402
from corner_module.fake import FakeDrive, FakeSteer  # noqa: E402
from corner_module.null_steer import NullSteer  # noqa: E402
from powertrain_ros import contract  # noqa: E402
from powertrain_ros.arm_interlock import ArmInterlock  # noqa: E402


def _method_ast(name):
    tree = ast.parse(CHASSIS_NODE.read_text(encoding="utf-8"))
    cls = next(
        item
        for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "ChassisNode"
    )
    return next(
        item
        for item in cls.body
        if isinstance(item, ast.FunctionDef) and item.name == name
    )


def _node_class(*method_names, globals_=None):
    namespace = dict(globals_ or {})
    for name in method_names:
        method = _method_ast(name)
        module = ast.Module(body=[method], type_ignores=[])
        ast.fix_missing_locations(module)
        exec(compile(module, str(CHASSIS_NODE), "exec"), namespace)
    return type(
        "ExtractedChassisNode",
        (),
        {name: namespace[name] for name in method_names},
    )


def _fake_corners():
    corners = {}
    config = CornerConfig()
    for wheel in default_geometry().wheels:
        steer = FakeSteer() if wheel.steerable else NullSteer()
        corners[wheel.name] = CornerModule(steer, FakeDrive(), config)
    return corners


def _armed_manager():
    manager = ChassisManager(
        _fake_corners(),
        cfg=ChassisConfig(watchdog_ms=1000.0),
    )
    manager.connect()
    assert manager.arm()
    return manager


def _drive_targets(manager):
    return {
        name: corner.state()["drive"]["target_vel"]
        for name, corner in manager.corners.items()
    }


def _gate_node(manager, publisher_count=1, gate_mode="production"):
    GateNode = _node_class(
        "_remote_owner_selected",
        "_arm_gate_decision",
        "_tick_arm_gate",
        globals_={"contract": contract},
    )
    node = GateNode()
    node.cm = manager
    node._arm_interlock = ArmInterlock()
    node._arm_absent_interlock = ArmInterlock()
    node._arm_gate_mode = gate_mode
    node._arm_override_requested = False
    node._authority_enabled = False
    node._authority = None
    node.count_publishers = lambda _topic: publisher_count
    return node


def _arm_status(status, stamp_s, mission_id=0):
    sec = int(stamp_s)
    nanosec = int(round((stamp_s - sec) * 1_000_000_000))
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
        ),
        status=status,
        mission_id=mission_id,
    )


def _feed_arm_status(node, status, stamp_s, mission_id=0):
    CallbackNode = _node_class(
        "_arm_status_stamp_s",
        "_on_arm_status",
        globals_={"ArmStatus": object, "contract": contract},
    )
    node.__class__ = type(
        "GateAndCallbackNode",
        (node.__class__, CallbackNode),
        {},
    )
    node._now_s = lambda: stamp_s
    node._last_arm_status = None
    node.get_logger = lambda: SimpleNamespace(info=lambda _message: None)
    node._on_arm_status(_arm_status(status, stamp_s, mission_id))


def test_stale_arm_stops_all_drives_and_fresh_status_needs_new_command():
    manager = _armed_manager()
    node = _gate_node(manager)
    manager.set(0.4, 0.2)

    assert node._tick_arm_gate(10.0) is False
    manager.tick()
    assert all(value == 0.0 for value in _drive_targets(manager).values())

    _feed_arm_status(node, contract.ARM_STOWED_LOCKED, 10.1)
    assert node._tick_arm_gate(10.1) is True
    manager.tick()
    assert all(value == 0.0 for value in _drive_targets(manager).values())


def test_fresh_carrying_locked_allows_drive_after_new_command():
    manager = _armed_manager()
    node = _gate_node(manager)
    assert node._tick_arm_gate(20.0) is False
    _feed_arm_status(node, contract.ARM_CARRYING_LOCKED, 20.1, mission_id=7)

    assert node._tick_arm_gate(20.1) is True
    manager.set(0.4, 0.0)
    manager.tick()
    assert all(value != 0.0 for value in _drive_targets(manager).values())


class _Message:
    def __init__(self):
        self.header = None
        self.mode = ""


class _Arrival:
    def __init__(self):
        self.header = None
        self.mission_id = 0
        self.status = ""


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _mode_node(contract_v2_verified, gate_mode="production"):
    ModeNode = _node_class(
        "set_chassis_mode_intent",
        "_effective_chassis_mode",
        "_publish_mode",
        "publish_arrival",
        globals_={
            "contract": contract,
            "ChassisMode": _Message,
            "ArrivalStatus": _Arrival,
        },
    )
    node = ModeNode()
    node._contract_v2_verified = contract_v2_verified
    node._arm_gate_mode = gate_mode
    node._arm_override_requested = False
    node._chassis_mode_intent = contract.MODE_STOW_REQUEST
    node._header = lambda: "header"
    node.pub_mode = _Publisher()
    node.pub_arrival = _Publisher()
    node.get_logger = lambda: SimpleNamespace(warning=lambda _message: None)
    node._mode_sel = SimpleNamespace(
        update=lambda *_args, **_kwargs: pytest.fail(
            "v1 ChassisModeSelector must remain dormant before Task 5"
        )
    )
    return node


def test_compatibility_lock_only_publishes_cornering_and_no_arrival():
    node = _mode_node(False)
    for intent in (contract.MODE_DRIVING, contract.MODE_MISSION_STOP):
        node.set_chassis_mode_intent(intent)
        node._publish_mode()

    assert [message.mode for message in node.pub_mode.messages] == [
        contract.MODE_CORNERING,
        contract.MODE_CORNERING,
    ]
    assert node.publish_arrival(3, contract.ARRIVED_PICKUP) is False
    assert node.pub_arrival.messages == []


def test_contract_v2_defaults_to_stow_request_intent():
    node = _mode_node(True)

    node._publish_mode()

    assert [message.mode for message in node.pub_mode.messages] == [
        contract.MODE_STOW_REQUEST,
    ]


def test_arm_absent_field_blocks_mission_stop_and_arrival_even_in_v2():
    node = _mode_node(True, gate_mode="arm_absent_field")
    node.set_chassis_mode_intent(contract.MODE_MISSION_STOP)

    node._publish_mode()

    assert [message.mode for message in node.pub_mode.messages] == [
        contract.MODE_STOW_REQUEST,
    ]
    assert node.publish_arrival(4, contract.ARRIVED_DROP) is False
    assert node.pub_arrival.messages == []


class _RecordingManager:
    def __init__(self, mode="IDLE"):
        self.mode = mode
        self.holds = []
        self.commands = []
        self.safety = SimpleNamespace(
            estop_latched=False,
            active_estop_sources=(),
            hold_sources=("robot_arm",) if mode == "ARMED" else (),
        )
        self.healthy = True

    def set_arm_motion_hold(self, active, detail=""):
        self.holds.append((active, detail))

    def set(self, v, omega):
        self.commands.append((v, omega))

    def state(self):
        return {"safety": self.safety}

    def snapshot(self):
        return SimpleNamespace(healthy=self.healthy)


def test_arm_absent_field_mock_stops_on_first_publisher_tick():
    manager = _RecordingManager()
    count = SimpleNamespace(value=0)
    node = _gate_node(manager, gate_mode="arm_absent_field")
    node.count_publishers = lambda _topic: count.value

    assert node._tick_arm_gate(30.0) is True
    assert manager.holds[-1] == (False, "")

    count.value = 1
    assert node._tick_arm_gate(30.02) is False
    assert manager.holds[-1][0] is True
    assert "stale" in manager.holds[-1][1]


def _override_node(gate=None, manager=None):
    OverrideNode = _node_class(
        "_remote_owner_selected",
        "set_chassis_mode_intent",
        "_discard_pending_command",
        "_override_activation_error",
        "_srv_arm_lock_override",
        "_arm_gate_decision",
        "_tick_arm_gate",
        globals_={"contract": contract},
    )
    node = OverrideNode()
    node.cm = manager or _RecordingManager(mode="ARMED")
    node._arm_interlock = gate or ArmInterlock()
    node._arm_absent_interlock = ArmInterlock()
    node._arm_gate_mode = "production"
    node._contract_v2_verified = True
    node._arm_override_requested = False
    node._arm_override_activated_s = None
    node._arm_override_expired = False
    node._arm_override_ttl_s = 30.0
    node._chassis_mode_intent = contract.MODE_STOW_REQUEST
    node._authority_enabled = True
    node._authority = SimpleNamespace(mode="MANUAL")
    node.count_publishers = lambda _topic: 1
    node._now_s = lambda: 40.0
    return node


def _set_override(node, active=True):
    response = SimpleNamespace(success=None, message="")
    request = SimpleNamespace(data=active)
    return node._srv_arm_lock_override(request, response)


def test_override_succeeds_only_while_stale_and_discards_command():
    node = _override_node()

    response = _set_override(node)

    assert response.success is True
    assert node._arm_override_requested is True
    assert node._chassis_mode_intent == contract.MODE_STOW_REQUEST
    assert node.cm.commands == [(0.0, 0.0)]

    fresh = ArmInterlock()
    assert fresh.update(contract.ARM_STOWED_LOCKED, 0, 40.0, 40.0)
    fresh_node = _override_node(gate=fresh)
    response = _set_override(fresh_node)
    assert response.success is False
    assert "stale" in response.message


def test_override_rejects_grip_lost_latch_even_after_status_is_stale():
    gate = ArmInterlock()
    assert gate.update(contract.ARM_GRIP_LOST, 7, 50.0, 50.0)
    node = _override_node(gate=gate)

    node._now_s = lambda: 51.0
    response = _set_override(node)

    assert response.success is False
    assert "grip_lost" in response.message


def test_override_rejects_contract_violation():
    gate = ArmInterlock()
    assert not gate.update("UNKNOWN_ARM_STATUS", 0, 40.0, 40.0)
    node = _override_node(gate=gate)

    response = _set_override(node)

    assert response.success is False
    assert "contract_violation" in response.message


def test_override_fresh_arm_immediately_removes_permission_but_keeps_flag():
    node = _override_node()
    assert _set_override(node).success is True
    assert node._tick_arm_gate(60.0) is True

    assert node._arm_interlock.update(
        contract.ARM_STOWED_LOCKED,
        0,
        60.02,
        60.02,
    )
    assert node._tick_arm_gate(60.02) is False
    assert node.cm.holds[-1][0] is True
    assert node._arm_override_requested is True


def test_override_ttl_allows_drive_until_strictly_after_deadline():
    node = _override_node()
    assert _set_override(node).success is True

    assert node._arm_override_activated_s == 40.0
    assert node._tick_arm_gate(70.0) is True
    assert node._arm_override_expired is False


def test_override_ttl_expiry_holds_drive_but_keeps_audit_flag():
    node = _override_node()
    assert _set_override(node).success is True

    assert node._tick_arm_gate(70.001) is False

    assert node._arm_override_requested is True
    assert node._arm_override_expired is True
    assert node.cm.holds[-1] == (True, "operator_override_expired")


def test_override_service_reactivation_starts_a_new_ttl():
    node = _override_node()
    assert _set_override(node).success is True
    assert node._tick_arm_gate(70.001) is False

    node._now_s = lambda: 71.0
    assert _set_override(node).success is True

    assert node._arm_override_activated_s == 71.0
    assert node._arm_override_expired is False
    assert node._tick_arm_gate(101.0) is True


def test_arm_status_qos_service_and_fail_closed_defaults_are_explicit():
    source = CHASSIS_NODE.read_text(encoding="utf-8")

    assert 'declare_parameter("contract_v2_verified", False)' in source
    assert 'declare_parameter("arm_gate_mode", "production")' in source
    assert 'declare_parameter("arm_override_ttl_s", 30.0)' in source
    assert "TODO(WP5.2 Task 7/remote gate)" in source
    assert "deadman" in source
    assert "independent joint proof" in source
    assert "DurabilityPolicy.VOLATILE" in source
    assert "ReliabilityPolicy.RELIABLE" in source
    assert "HistoryPolicy.KEEP_LAST" in source
    assert "count_publishers(contract.TOPIC_ARM_STATUS)" in source
    assert '"~/arm_lock_override"' in source
    assert "SetBool" in source


def test_arm_status_callback_uses_one_node_clock_sample():
    CallbackNode = _node_class(
        "_arm_status_stamp_s",
        "_on_arm_status",
        globals_={"ArmStatus": object, "contract": contract},
    )
    node = CallbackNode()
    node._arm_interlock = ArmInterlock()
    node._last_arm_status = None
    clock_calls = []
    node._now_s = lambda: clock_calls.append(70.0) or 70.0
    node.get_logger = lambda: SimpleNamespace(
        info=lambda _message: None,
        warning=lambda _message: None,
    )

    node._on_arm_status(_arm_status(contract.ARM_STOWED_LOCKED, 70.0))

    assert clock_calls == [70.0]
    assert node._arm_interlock.drive_allowed("EMPTY_STOWED", 70.0)


def test_legacy_simplenamespace_tick_fixture_stays_usable_without_arm_fields():
    TickNode = _node_class(
        "_tick",
        globals_={
            "time": time,
            "WheelStates": SimpleNamespace,
            "WheelState": SimpleNamespace,
            "fill_wheel_states_message": lambda *_args, **_kwargs: None,
        },
    )
    manager = SimpleNamespace(
        cfg=SimpleNamespace(loop_hz=50.0),
        tick_count=0,
        tick=lambda: setattr(manager, "tick_count", manager.tick_count + 1),
        snapshot=lambda: SimpleNamespace(),
        estop=lambda _source, _detail: None,
    )
    node = TickNode()
    node.cm = manager
    node._now_ms = lambda: 10_000.0
    node._authority_enabled = False
    node._safety_required = False
    node._overrun_count = 0
    node._wheel_telemetry_failed = False
    node.pub_wheels = SimpleNamespace(publish=lambda _message: None)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: "stamp")
    )
    node.get_logger = lambda: SimpleNamespace(
        error=lambda _message: None,
        info=lambda _message: None,
    )

    node._tick()

    assert manager.tick_count == 1


def test_hardware_launch_and_readme_expose_fail_closed_arm_gate_defaults():
    launch = LAUNCH.read_text(encoding="utf-8")
    readme = README.read_text(encoding="utf-8")

    assert '"contract_v2_verified"' in launch
    assert 'default_value="false"' in launch
    assert '"arm_gate_mode"' in launch
    assert 'default_value="production"' in launch
    assert 'LaunchConfiguration("contract_v2_verified")' in launch
    assert 'LaunchConfiguration("arm_gate_mode")' in launch
    assert '"arm_override_ttl_s"' in launch
    assert 'LaunchConfiguration("arm_override_ttl_s")' in launch
    assert "arm_absent_field" in readme
    assert "contract_v2_verified" in readme
    assert "STOW_REQUEST" in readme


def test_srv_arm_refreshes_safety_freshness_baseline():
    """cm.arm()이 executor를 ~0.8s 블로킹해 verdict 콜백이 밀리는 동안
    freshness가 거짓 stale로 래치되던 실기 결함(2026-07-16: age 783ms 래치,
    직후 rx gap 800ms) — arm 종료 시점으로 기준선을 당겨야 한다."""
    ArmNode = _node_class("_srv_arm")
    node = ArmNode()
    node.cm = SimpleNamespace(
        arm=lambda: True,
        state=lambda: {"safety": SimpleNamespace(estop_latched=False)},
        mode="ARMED",
    )
    node._now_ms = lambda: 100_000.0
    node._last_safety_ms = 99_000.0          # arm 블로킹 동안 1s 낡음
    node.get_logger = lambda: SimpleNamespace(info=lambda *_a, **_k: None)
    response = SimpleNamespace(success=None, message=None)
    node._srv_arm(None, response)
    assert response.success is True
    assert node._last_safety_ms == 100_000.0  # 기준선 재설정

    # 아직 verdict를 한 번도 못 받은 상태(None)에서는 재설정하지 않는다 —
    # startup timeout 경로의 의미를 바꾸면 안 된다.
    node._last_safety_ms = None
    node._srv_arm(None, response)
    assert node._last_safety_ms is None
