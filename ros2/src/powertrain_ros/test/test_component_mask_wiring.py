"""CMASK chassis-node services, source gating, state, and journal wiring."""
import json
import threading
import time
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from powertrain_msgs.msg import SafetyVerdict
from powertrain_ros.chassis_node import ChassisNode


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class _RecordingEventClient:
    def __init__(self):
        self.events = []

    def emit(self, event):
        self.events.append(event)
        return True


class _Harness:
    def __init__(self, chassis):
        self.chassis = chassis
        self.node = rclpy.create_node("component_mask_test_harness")
        self.safety = self.node.create_publisher(
            SafetyVerdict,
            "/safety_verdict",
            10,
        )
        self.safety_states = []
        self.node.create_subscription(
            String,
            "/chassis/safety_state",
            lambda message: self.safety_states.append(json.loads(message.data)),
            10,
        )
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(chassis)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def wait_for(self, predicate, timeout_s=2.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.01)
        return bool(predicate())

    def set_component(self, component, enabled):
        client = self.node.create_client(
            SetBool,
            "/chassis_node/component_enable_%s" % component,
        )
        assert client.wait_for_service(timeout_sec=2.0)
        future = client.call_async(SetBool.Request(data=enabled))
        assert self.wait_for(future.done)
        result = future.result()
        self.node.destroy_client(client)
        return result

    def arm(self):
        client = self.node.create_client(Trigger, "/chassis_node/arm")
        assert client.wait_for_service(timeout_sec=2.0)
        future = client.call_async(Trigger.Request())
        assert self.wait_for(future.done)
        result = future.result()
        self.node.destroy_client(client)
        return result

    def trigger(self, service):
        client = self.node.create_client(
            Trigger,
            "/chassis_node/%s" % service,
        )
        assert client.wait_for_service(timeout_sec=2.0)
        future = client.call_async(Trigger.Request())
        assert self.wait_for(future.done)
        result = future.result()
        self.node.destroy_client(client)
        return result

    def publish_no_response(self):
        message = SafetyVerdict()
        message.status = SafetyVerdict.NO_RESPONSE
        message.estop_required = True
        message.detail = "test no response"
        previous = getattr(self.chassis, "_last_safety_ms", None)
        self.safety.publish(message)
        assert self.wait_for(
            lambda: getattr(self.chassis, "_last_safety_ms", None) != previous
        )

    def close(self):
        self.executor.shutdown(timeout_sec=2.0)
        self.thread.join(timeout=2.0)
        self.chassis.close()
        self.chassis.destroy_node()
        self.node.destroy_node()


@pytest.fixture()
def harness():
    chassis = ChassisNode(
        parameter_overrides=[
            Parameter("fake", value=True),
            Parameter("safety_required", value=False),
            Parameter("arm_gate_mode", value="arm_absent_field"),
        ]
    )
    result = _Harness(chassis)
    try:
        yield result
    finally:
        result.close()


def test_component_services_round_trip_and_motor_changes_require_idle(harness):
    disabled = harness.set_component("drive", False)
    assert disabled.success is True
    assert disabled.message == "drive disabled"

    enabled = harness.set_component("drive", True)
    assert enabled.success is True
    assert enabled.message == "drive enabled"
    assert harness.arm().success is True

    rejected = harness.set_component("drive", False)
    assert rejected.success is False
    assert rejected.message == "not_idle"


def test_us100_off_ignores_no_response_and_reenable_restores_estop(harness):
    assert harness.set_component("us100", False).success is True

    harness.publish_no_response()

    safety = harness.chassis.cm.safety_snapshot()
    assert safety.state == "RUN"
    assert safety.estop_latched is False

    assert harness.set_component("us100", True).success is True
    harness.publish_no_response()
    assert harness.wait_for(
        lambda: harness.chassis.cm.safety_snapshot().estop_latched
    )
    assert harness.chassis.cm.safety_snapshot().state == "ESTOP"


def test_robot_arm_off_ignores_stale_arm_gate_hold(harness):
    assert harness.set_component("robot_arm", False).success is True

    assert harness.wait_for(
        lambda: harness.chassis.cm.component_mask["robot_arm"] is False
    )
    time.sleep(0.1)

    assert "robot_arm" not in harness.chassis.cm.safety_snapshot().hold_sources


def test_safety_state_json_contains_current_component_mask(harness):
    assert harness.set_component("steer", False).success is True

    assert harness.wait_for(
        lambda: any(
            item.get("component_mask", {}).get("steer") is False
            for item in harness.safety_states
        )
    )


def test_successful_and_idempotent_changes_emit_component_mask_journal(harness):
    events = _RecordingEventClient()
    harness.chassis._observability_event_client = events

    assert harness.set_component("us100", False).success is True
    assert harness.set_component("us100", False).success is True

    mask_events = [
        event for event in events.events
        if event.get("event_type") == "COMPONENT_MASK"
    ]
    assert len(mask_events) == 2
    assert all(
        event["payload"] == {"component": "us100", "enabled": False}
        for event in mask_events
    )


def test_console_estop_round_trip_is_idempotent_and_journaled(harness):
    events = _RecordingEventClient()
    harness.chassis._observability_event_client = events

    first = harness.trigger("estop")
    assert first.success is True
    assert first.message == "mode=ESTOP"
    safety = harness.chassis.cm.safety_snapshot()
    assert safety.first_source == "console"
    assert safety.first_detail == "operator emergency stop"
    assert harness.wait_for(
        lambda: any(
            item.get("mode") == "ESTOP"
            and item.get("estop_latched") is True
            for item in harness.safety_states
        )
    )

    second = harness.trigger("estop")
    assert second.success is True
    assert second.message == "mode=ESTOP"

    reset_state_index = len(harness.safety_states)
    reset = harness.trigger("reset_estop")
    assert reset.success is True
    assert harness.chassis.cm.mode == "IDLE"
    assert harness.wait_for(
        lambda: any(
            item.get("mode") == "IDLE"
            and item.get("estop_latched") is False
            for item in harness.safety_states[reset_state_index:]
        )
    )

    estop_events = [
        event for event in events.events
        if event.get("event_type") == "CONSOLE_ESTOP"
    ]
    assert len(estop_events) == 2
    assert all(event.get("severity") == "WARN" for event in estop_events)


def test_console_estop_fails_closed_without_chassis_manager():
    response = SimpleNamespace(success=None, message=None)

    returned = ChassisNode._srv_estop(
        SimpleNamespace(),
        object(),
        response,
    )

    assert returned is response
    assert response.success is False
    assert response.message == "chassis manager unavailable"
