import ast
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from powertrain_ros import contract
from powertrain_ros.wp8_handshake_probe import (
    _run_ros,
    events_from_records,
    observe_only_enabled,
    parse_args,
    serialize_summary,
)
from powertrain_ros.wp8_scenario import Finding


MODULE = (
    Path(__file__).resolve().parents[1]
    / "powertrain_ros"
    / "wp8_handshake_probe.py"
)


def test_cli_baseline_uses_documented_defaults():
    args = parse_args(["baseline"])

    assert args.subcommand == "baseline"
    assert args.timeout_s == 8.0
    assert args.node_ns == "/chassis_node"
    assert args.json is None


def test_cli_parses_pickup_and_full_cycle_common_options(tmp_path):
    pickup = parse_args(
        [
            "pickup",
            "--timeout-s",
            "3.5",
            "--node-ns",
            "/fixture",
            "--json",
            str(tmp_path / "pickup.json"),
        ]
    )
    full_cycle = parse_args(["full-cycle", "--timeout-s", "12"])

    assert pickup.timeout_s == 3.5
    assert pickup.node_ns == "/fixture"
    assert pickup.json == str(tmp_path / "pickup.json")
    assert full_cycle.subcommand == "full-cycle"
    assert full_cycle.timeout_s == 12.0


def test_cli_resume_requires_timestamp_file(tmp_path):
    resume_file = tmp_path / "resume_t"
    args = parse_args(["resume", "--resume-t-file", str(resume_file)])

    assert args.resume_t_file == str(resume_file)

    with pytest.raises(SystemExit) as exc_info:
        parse_args(["resume"])
    assert exc_info.value.code == 2


@pytest.mark.parametrize(
    "scenario",
    ["no_response", "late_done", "failed_latch", "dup_done"],
)
def test_cli_fault_accepts_each_scenario(scenario):
    args = parse_args(["fault", "--scenario", scenario])

    assert args.scenario == scenario


def test_cli_rejects_unknown_argument_and_fault_scenario():
    with pytest.raises(SystemExit) as unknown_arg:
        parse_args(["baseline", "--surprise"])
    with pytest.raises(SystemExit) as unknown_scenario:
        parse_args(["fault", "--scenario", "other"])

    assert unknown_arg.value.code == 2
    assert unknown_scenario.value.code == 2


def test_events_from_records_converts_ros_message_shapes_and_marker():
    records = [
        (
            1.0,
            contract.TOPIC_CHASSIS_MODE,
            SimpleNamespace(mode=contract.MODE_MISSION_STOP),
        ),
        (
            1.1,
            contract.TOPIC_ARRIVAL,
            SimpleNamespace(status=contract.ARRIVED_PICKUP, mission_id=12),
        ),
        (
            1.2,
            contract.TOPIC_ARM_STATUS,
            SimpleNamespace(status=contract.ARM_WORK_READY, mission_id=12),
        ),
        (1.3, "marker", "sigcont"),
    ]

    events = events_from_records(records)

    assert [(event.topic, event.value, event.mission_id) for event in events] == [
        ("chassis_mode", contract.MODE_MISSION_STOP, None),
        ("arrival", contract.ARRIVED_PICKUP, 12),
        ("arm_status", contract.ARM_WORK_READY, 12),
        ("marker", "sigcont", None),
    ]


def test_events_from_records_rejects_unknown_topic():
    with pytest.raises(ValueError, match="topic"):
        events_from_records([(1.0, "/unknown", SimpleNamespace())])


def test_serialize_summary_is_one_line_with_exact_contract_fields():
    encoded = serialize_summary(
        "pickup",
        [
            Finding("ordering", True, "stop before arrival"),
            Finding("work", False, "missing"),
        ],
        branch="violation",
    )

    assert "\n" not in encoded
    assert json.loads(encoded) == {
        "subcommand": "pickup",
        "pass": False,
        "branch": "violation",
        "findings": [
            {"check": "ordering", "ok": True, "detail": "stop before arrival"},
            {"check": "work", "ok": False, "detail": "missing"},
        ],
    }


@pytest.mark.parametrize(
    ("service_results", "expected_calls", "expected_markers"),
    [
        (
            [(False, "lock heartbeat stale")],
            ["arm"],
            ["arm_rejected:lock heartbeat stale"],
        ),
        (
            [(True, "armed"), (False, "chassis not driving")],
            ["arm", "mission_arrive_pickup"],
            ["arm_ok", "arrival_rejected:chassis not driving"],
        ),
        (
            [(True, "armed"), (True, "arrival accepted")],
            ["arm", "mission_arrive_pickup"],
            ["arm_ok", "arrival_ok"],
        ),
    ],
)
def test_pickup_records_service_results_and_keeps_observing(
    monkeypatch,
    service_results,
    expected_calls,
    expected_markers,
):
    class Probe:
        def __init__(self):
            self.records = []

        def _record(self, topic, message):
            self.records.append((1.0 + len(self.records), topic, message))

        def events(self):
            return events_from_records(self.records)

    responses = iter(
        SimpleNamespace(success=success, message=message)
        for success, message in service_results
    )
    service_calls = []
    spin_deadlines = []

    def call_trigger(_probe, _executor, suffix, *, deadline):
        service_calls.append(suffix)
        return next(responses)

    monkeypatch.setattr(
        "powertrain_ros.wp8_handshake_probe._call_trigger",
        call_trigger,
    )
    monkeypatch.setattr(
        "powertrain_ros.wp8_handshake_probe._spin_until",
        lambda _executor, _predicate, *, deadline: spin_deadlines.append(deadline),
    )
    monkeypatch.setattr(
        "powertrain_ros.wp8_handshake_probe.observe_only_enabled",
        lambda: False,
    )
    probe = Probe()

    events = _run_ros(
        SimpleNamespace(subcommand="pickup", timeout_s=1.0),
        executor=object(),
        probe=probe,
    )

    assert [event.value for event in events] == expected_markers
    assert service_calls == expected_calls
    assert len(spin_deadlines) == 2


def test_module_has_no_top_level_rclpy_import():
    tree = ast.parse(MODULE.read_text(encoding="utf-8"))

    top_level_imports = [
        node
        for node in tree.body
        if isinstance(node, (ast.Import, ast.ImportFrom))
    ]
    imported_names = {
        alias.name
        for node in top_level_imports
        for alias in node.names
    }

    assert "rclpy" not in imported_names


def test_observe_only_is_enabled_only_by_explicit_harness_flag():
    assert observe_only_enabled({"WP8_PROBE_OBSERVE_ONLY": "1"}) is True
    assert observe_only_enabled({"WP8_PROBE_OBSERVE_ONLY": "0"}) is False
    assert observe_only_enabled({}) is False
