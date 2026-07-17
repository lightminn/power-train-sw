"""Static contracts for opt-in section enforcement in chassis_node."""

import ast
from pathlib import Path


MODULE = (
    Path(__file__).resolve().parents[1]
    / "powertrain_ros"
    / "chassis_node.py"
)


def _source():
    return MODULE.read_text(encoding="utf-8")


def test_parameter_is_default_off_requires_authority_and_subscribes_state():
    source = _source()
    assert 'declare_parameter("section_enforcement", False)' in source
    assert "section_enforcement=true requires authority_enabled=true" in source
    assert '"/section/state"' in source
    assert "self._on_section_state" in source


def test_enforcement_decision_is_applied_immediately_before_final_set():
    tree = ast.parse(_source())
    method = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == "_tick_authority"
    )
    calls = [node for node in ast.walk(method) if isinstance(node, ast.Call)]
    decide = next(
        node
        for node in calls
        if isinstance(node.func, ast.Attribute)
        and node.func.attr == "decide"
    )
    final_set = next(
        node
        for node in calls
        if isinstance(node.func, ast.Attribute)
        and node.func.attr == "set"
        and len(node.args) == 2
        and all(isinstance(arg, ast.Name) for arg in node.args)
        and [arg.id for arg in node.args] == ["final_v", "final_omega"]
    )
    assert decide.lineno < final_set.lineno
    assert final_set.lineno - decide.lineno < 25


def test_floor_conversion_clamp_and_journal_throttle_are_explicit():
    source = _source()
    compact = "".join(source.split())
    assert (
        "cfg.min_drive_turns_per_s*2.0*math.pi*0.10" in compact
    )
    assert (
        "max(-decision.v_cap,min(decision.v_cap,final_v))" in compact
    )
    assert '"SECTION_ENFORCEMENT"' in source
    assert "_section_enforcement_event_period_ns = 1_000_000_000" in source
