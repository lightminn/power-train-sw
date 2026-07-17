"""broker ops-state 소스 토픽 계약 (스펙 r6 §3.1)."""
import ast
import json
import re
from pathlib import Path

from powertrain_ros.remote_input_gateway import frame_is_neutral
from test_remote_input_gateway import _frame

PACKAGE = Path(__file__).resolve().parents[1]
TELEOP = (PACKAGE / "powertrain_ros/teleop_command_node.py").read_text(
    encoding="utf-8"
)
CHASSIS = (PACKAGE / "powertrain_ros/chassis_node.py").read_text(
    encoding="utf-8"
)


def test_frame_is_neutral_matches_gateway_semantics():
    assert frame_is_neutral(_frame())
    assert not frame_is_neutral(_frame(deadman=True))
    assert not frame_is_neutral(_frame(right_trigger=0.2))
    assert not frame_is_neutral(_frame(dpad_x=1))
    assert not frame_is_neutral(_frame(estop_edge=True))


def test_teleop_publishes_gateway_state_each_tick():
    assert '"/teleop/gateway_state"' in TELEOP
    assert '"neutral"' in TELEOP and '"input_fresh"' in TELEOP


def test_chassis_publishes_safety_state():
    assert '"/chassis/safety_state"' in CHASSIS
    assert '"estop_latched"' in CHASSIS
    assert '"active_estop_sources"' in CHASSIS
