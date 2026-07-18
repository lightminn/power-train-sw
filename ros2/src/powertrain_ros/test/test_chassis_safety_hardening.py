"""Host-pure regressions for chassis-node safety hardening."""

import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
CHASSIS_NODE = PACKAGE / "powertrain_ros/chassis_node.py"
SAFETY_CORE = PACKAGE / "powertrain_ros/chassis_safety.py"


def _load_safety_core():
    assert SAFETY_CORE.exists(), (
        "ROS-independent chassis safety core is missing"
    )
    spec = importlib.util.spec_from_file_location(
        "chassis_safety", SAFETY_CORE
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _class_method(tree, name):
    chassis = next(
        item
        for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "ChassisNode"
    )
    return next(
        item
        for item in chassis.body
        if isinstance(item, ast.FunctionDef) and item.name == name
    )


def _node_class(*method_names):
    tree = ast.parse(CHASSIS_NODE.read_text(encoding="utf-8"))
    namespace = {}
    methods = {}
    for name in method_names:
        method = _class_method(tree, name)
        module = ast.Module(body=[method], type_ignores=[])
        ast.fix_missing_locations(module)
        exec(compile(module, str(CHASSIS_NODE), "exec"), namespace)
        methods[name] = namespace[name]
    return type("ExtractedChassisNode", (), methods)


def test_real_hardware_rejects_sim_time_but_fake_mode_allows_it():
    safety = _load_safety_core()

    with pytest.raises(ValueError, match="use_sim_time"):
        safety.validate_runtime_clock_mode(fake=False, use_sim_time=True)

    safety.validate_runtime_clock_mode(fake=False, use_sim_time=False)
    safety.validate_runtime_clock_mode(fake=True, use_sim_time=True)


def test_real_hardware_rejects_runtime_sim_time_enable():
    source = CHASSIS_NODE.read_text(encoding="utf-8")
    tree = ast.parse(source)
    initialize = ast.unparse(_class_method(tree, "_initialize"))
    method_names = {
        item.name
        for item in next(
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef) and node.name == "ChassisNode"
        ).body
        if isinstance(item, ast.FunctionDef)
    }

    assert "_validate_runtime_parameter_change" in method_names
    callback = ast.unparse(
        _class_method(tree, "_validate_runtime_parameter_change")
    )
    assert "add_on_set_parameters_callback" in initialize
    assert "use_sim_time" in callback
    assert "validate_runtime_clock_mode" in callback
    assert "SetParametersResult(successful=False" in callback


def test_control_and_local_timeout_clock_are_steady_not_ros_time():
    source = CHASSIS_NODE.read_text(encoding="utf-8")
    tree = ast.parse(source)
    now_ms = ast.unparse(_class_method(tree, "_now_ms"))
    initialize = _class_method(tree, "_initialize")
    timers = [
        call
        for call in ast.walk(initialize)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == "create_timer"
    ]

    assert "_steady_clock.now()" in now_ms
    assert "get_clock" not in now_ms
    assert len(timers) == 3
    assert all(
        any(
            keyword.arg == "clock"
            and isinstance(keyword.value, ast.Attribute)
            and keyword.value.attr == "_steady_clock"
            for keyword in timer.keywords
        )
        for timer in timers
    )
    assert source.index("validate_runtime_clock_mode(") < source.index(
        "from chassis.runtime_lock import RealCanSession"
    )


def test_console_estop_store_is_atomic_and_corruption_loads_fail_closed(
    tmp_path, monkeypatch
):
    safety = _load_safety_core()
    path = tmp_path / "console-estop.json"
    store = safety.ConsoleEstopLatchStore(path)
    assert store.load_fail_closed() is None

    store.persist("console", "first operator cause")
    record = store.load_fail_closed()
    assert record.first_source == "console"
    assert record.first_detail == "first operator cause"
    assert list(tmp_path.glob(".console-estop.json.*.tmp")) == []

    original_replace = safety.os.replace

    def fail_replace(_source, _target):
        raise OSError("injected replace failure")

    monkeypatch.setattr(safety.os, "replace", fail_replace)
    with pytest.raises(OSError, match="injected replace failure"):
        store.persist("console", "must not replace the first cause")
    monkeypatch.setattr(safety.os, "replace", original_replace)
    assert store.load_fail_closed() == record

    store.clear()
    assert store.load_fail_closed() is None
    store.persist("console", "first operator cause")

    path.write_text("not-json", encoding="utf-8")
    corrupt = store.load_fail_closed()
    assert corrupt.first_source == "console_latch_store"
    assert "load_failed" in corrupt.first_detail


def test_console_estop_store_missing_parent_loads_fail_closed(tmp_path):
    safety = _load_safety_core()
    store = safety.ConsoleEstopLatchStore(
        tmp_path / "missing-runtime-dir" / "console-estop.json"
    )

    record = store.load_fail_closed()

    assert record.first_source == "console_latch_store"
    assert "load_failed" in record.first_detail

    with pytest.raises(OSError):
        store.clear()


class _RecordingStore:
    def __init__(self):
        self.persisted = []
        self.clear_calls = 0

    def persist(self, source, detail):
        self.persisted.append((source, detail))

    def clear(self):
        self.clear_calls += 1


class _RecordingManager:
    def __init__(self):
        self.mode = "IDLE"
        self.reset_result = True
        self.estop_calls = []
        self.safety = SimpleNamespace(
            estop_latched=False,
            active_estop_sources=(),
            first_source=None,
            first_detail="",
        )

    def estop(self, source, detail):
        self.estop_calls.append((source, detail))
        if not self.safety.estop_latched:
            self.safety.first_source = source
            self.safety.first_detail = detail
        self.safety.estop_latched = True
        self.mode = "ESTOP"

    def reset_estop(self):
        if not self.reset_result:
            return False
        self.safety.estop_latched = False
        self.safety.first_source = None
        self.safety.first_detail = ""
        self.mode = "IDLE"
        return True

    def state(self):
        return {"safety": self.safety}


def test_console_estop_persists_restores_and_clears_only_after_reset_success():
    Node = _node_class(
        "_restore_console_estop_latch",
        "_latch_console_estop",
        "_srv_estop",
        "_srv_reset_estop",
    )
    store = _RecordingStore()
    manager = _RecordingManager()
    node = Node()
    node.cm = manager
    node._console_estop_store = store
    node._refresh_safety_baseline = lambda: None
    node._emit_console_estop_event = lambda _mode: None
    node.get_logger = lambda: SimpleNamespace(warning=lambda _message: None)

    response = node._srv_estop(
        object(), SimpleNamespace(success=None, message="")
    )
    assert response.success is True
    assert store.persisted == [("console", "operator emergency stop")]

    restored_manager = _RecordingManager()
    restored = Node()
    restored.cm = restored_manager
    restored._restore_console_estop_latch(
        SimpleNamespace(
            first_source="console",
            first_detail="operator emergency stop",
        )
    )
    assert restored_manager.estop_calls == [
        ("console", "operator emergency stop")
    ]

    manager.reset_result = False
    rejected = node._srv_reset_estop(
        object(), SimpleNamespace(success=None, message="")
    )
    assert rejected.success is False
    assert store.clear_calls == 0

    manager.reset_result = True
    accepted = node._srv_reset_estop(
        object(), SimpleNamespace(success=None, message="")
    )
    assert accepted.success is True
    assert store.clear_calls == 1


def test_console_estop_persists_before_latch_and_topic_uses_same_path():
    source = CHASSIS_NODE.read_text(encoding="utf-8")
    tree = ast.parse(source)
    method_names = {
        item.name
        for item in next(
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef) and node.name == "ChassisNode"
        ).body
        if isinstance(item, ast.FunctionDef)
    }
    assert "_latch_console_estop" in method_names
    assert "_latch_console_estop" in ast.unparse(
        _class_method(tree, "_on_teleop_estop")
    )
    assert "self._latch_console_estop(" in ast.unparse(
        _class_method(tree, "_srv_estop")
    )

    events = []

    class OrderedStore(_RecordingStore):
        def persist(self, source, detail):
            events.append(("persist", source, detail))
            super().persist(source, detail)

    class OrderedManager(_RecordingManager):
        def estop(self, source, detail):
            events.append(("estop", source, detail))
            super().estop(source, detail)

    Node = _node_class("_latch_console_estop")
    node = Node()
    node.cm = OrderedManager()
    node._console_estop_store = OrderedStore()
    node._refresh_safety_baseline = lambda: None

    node._latch_console_estop("remote_operator", "circle edge")

    assert [event[0] for event in events] == ["persist", "estop"]


def test_console_estop_store_load_precedes_hardware_connection_and_services():
    source = CHASSIS_NODE.read_text(encoding="utf-8")

    assert '"console_estop_latch_path"' in source
    load_at = source.index(".load_fail_closed()")
    real_can_at = source.index(
        "from chassis.runtime_lock import RealCanSession"
    )
    connect_at = source.index("self.cm.connect()")
    restore_at = source.index("self._restore_console_estop_latch(")
    service_at = source.index('self.create_service(Trigger, "~/arm"')
    assert load_at < real_can_at < connect_at < restore_at < service_at


def test_cached_safety_parameters_are_declared_read_only():
    tree = ast.parse(CHASSIS_NODE.read_text(encoding="utf-8"))
    initialize = _class_method(tree, "_initialize")
    declarations = {}
    for call in ast.walk(initialize):
        if (
            isinstance(call, ast.Call)
            and isinstance(call.func, ast.Attribute)
            and call.func.attr == "declare_parameter"
            and call.args
            and isinstance(call.args[0], ast.Constant)
        ):
            declarations[call.args[0].value] = call

    for name in (
        "fake",
        "channel",
        "four_wheel",
        "min_rev",
        "friction_ff",
        "friction_v_knee",
        "gear_ratio",
        "v_max",
        "cmd_timeout",
        "safety_required",
        "safety_topic_timeout",
        "safety_startup_timeout",
        "extraction_enabled",
        "authority_enabled",
        "section_enforcement",
        "assist_enabled",
        "contract_v2_verified",
        "boot_qualification_enabled",
        "arm_gate_mode",
        "arm_override_ttl_s",
        "mission_contract_owner",
        "mission_id_path",
        "console_estop_latch_path",
        "wheel_stop_config",
        "authority_handover_timeout_s",
    ):
        descriptor = next(
            (
                keyword.value
                for keyword in declarations[name].keywords
                if keyword.arg == "descriptor"
            ),
            None,
        )
        assert isinstance(descriptor, ast.Name), "%s must be read-only" % name
        assert descriptor.id == "read_only_safety_parameter"

    assignments = [
        node
        for node in ast.walk(initialize)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name)
            and target.id == "read_only_safety_parameter"
            for target in node.targets
        )
    ]
    assert len(assignments) == 1
    assert "ParameterDescriptor(read_only=True)" in ast.unparse(assignments[0])


def test_package_declares_parameter_descriptor_runtime_dependency():
    package_xml = (PACKAGE / "package.xml").read_text(encoding="utf-8")

    assert "<exec_depend>rcl_interfaces</exec_depend>" in package_xml
