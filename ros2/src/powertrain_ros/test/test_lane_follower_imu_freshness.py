"""Host-side safety contract for lane-follower IMU freshness."""

import ast
from pathlib import Path

import pytest

from powertrain_ros.lane_freshness import ImuFreshnessGate


NODE_PATH = (
    Path(__file__).parents[1]
    / "powertrain_ros"
    / "lane_follower_node.py"
)


def test_imu_freshness_requires_recent_receipt_and_header():
    gate = ImuFreshnessGate(timeout_s=0.25)

    assert not gate.is_fresh(now_steady_s=10.0, now_ros_s=100.0)

    gate.update(received_steady_s=10.0, header_ros_s=100.0)
    assert gate.is_fresh(now_steady_s=10.20, now_ros_s=100.20)
    assert not gate.is_fresh(now_steady_s=10.26, now_ros_s=100.20)

    gate.update(received_steady_s=20.0, header_ros_s=200.0)
    assert not gate.is_fresh(now_steady_s=20.20, now_ros_s=200.26)


@pytest.mark.parametrize(
    "bad_stamp", [float("nan"), float("inf"), float("-inf")]
)
def test_imu_freshness_rejects_nonfinite_stamps(bad_stamp):
    gate = ImuFreshnessGate(timeout_s=0.25)
    gate.update(received_steady_s=10.0, header_ros_s=bad_stamp)
    assert not gate.is_fresh(now_steady_s=10.1, now_ros_s=10.1)


def test_lane_node_gates_commands_with_steady_and_header_freshness():
    source = NODE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    methods = {
        item.name: ast.get_source_segment(source, item)
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "LaneFollowerNode"
        for item in node.body
        if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef))
    }

    assert 'declare_parameter("imu_timeout_s", 0.25)' in methods["__init__"]
    assert "ClockType.STEADY_TIME" in methods["__init__"]
    assert "self._imu_freshness.update(" in methods["_on_imu"]
    assert "header_ros_s=self._stamp_s(msg.header.stamp)" in methods["_on_imu"]
    assert (
        "imu_is_fresh = self._imu_freshness.is_fresh("
        in methods["_on_image"]
    )
    assert "imu_is_fresh = True" not in methods["_on_image"]
    assert "ok and imu_is_fresh and self._allow_drive" in methods["_on_image"]
