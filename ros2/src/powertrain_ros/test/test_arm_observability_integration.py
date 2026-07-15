import asyncio
import os
from pathlib import Path
import socket
import sys
import time
import uuid

import pytest


ROOT = Path(__file__).parents[4]
sys.path.insert(0, str(ROOT / "motor_control"))

from chassis.mission import FailureRecord, SupervisorResult
from l515_dashboard.app import DashboardApp
from powertrain_observability.client import EventClient, ObservabilityClient
from powertrain_observability.server import ObservabilityServer
from powertrain_ros.arm_interlock import ArmInterlock
from powertrain_ros.chassis_node import ChassisNode
from robot_arm_msgs.msg import ArmStatus


def require_abstract_socket_runtime():
    probe = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_PASSCRED, 1)
        probe.bind("\0test-t5-probe-" + uuid.uuid4().hex)
    except PermissionError as exc:
        pytest.skip(
            f"sandbox blocks AF_UNIX abstract sockets/SO_PASSCRED: {exc}"
        )
    finally:
        probe.close()


def wait_for(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.01)
    return predicate()


class Logger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass

    def error(self, _message):
        pass


class GatewayClient:
    last_error = None

    def poll(self):
        return None


class Supervisor:
    def __init__(self, result, failure):
        self.active_mission_id = 71
        self.result = result
        self.failure = failure
        self.calls = []

    def on_arm_status(self, status, mission_id, stamp_s, now_s):
        self.calls.append((status, mission_id, stamp_s, now_s))
        return self.result


class CallbackHarness:
    _arm_status_stamp_s = ChassisNode._arm_status_stamp_s
    _emit_arm_result_event = ChassisNode._emit_arm_result_event
    _on_arm_status = ChassisNode._on_arm_status

    def __init__(self, event_client, supervisor):
        self._arm_interlock = ArmInterlock(timeout_s=0.5)
        self._mission_supervisor_enabled = True
        self._mission_supervisor = supervisor
        self._observability_event_client = event_client
        self._last_arm_result_event_ns = 0
        self._last_arm_result_event_key = None
        self._arm_result_event_period_ns = 0
        self._last_arm_status = None
        self._last_arm_posture = "STOWED_LOCKED"
        self.applied_results = []
        self._now_s = lambda: 123.25
        self.get_logger = lambda: Logger()

    def _apply_mission_result(self, result):
        self.applied_results.append(result)

    def _mission_supervisor_failure(self, detail):
        pytest.fail(f"unexpected mission supervisor failure: {detail}")


def arm_status(status):
    message = ArmStatus()
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 200_000_000
    message.mission_id = 71
    message.status = status
    return message


def supervisor_fixture(status, hold_reason):
    if status == "FUTURE_ARM_STATUS":
        return Supervisor(
            pytest.fail,
            None,
        )
    state = "GRIP_LOST_HOLD" if status == "GRIP_LOST" else "FAILED_HOLD"
    result = SupervisorResult(
        state=state,
        hold_reason=hold_reason,
        operator_notice="operator_action_required",
    )
    failure = FailureRecord(
        wire_status=status,
        mission_id=71,
        stamp_s=123.2,
        last_locked_posture="STOWED_LOCKED",
        operation="PICKUP",
        arm_latched=True,
    )
    return Supervisor(result, failure)


@pytest.mark.parametrize(
    ("status", "hold_reason"),
    (
        ("FUTURE_ARM_STATUS", "arm_contract_violation:FUTURE_ARM_STATUS"),
        ("FAILED", "arm_failure:FAILED"),
        ("GRIP_LOST", "grip_lost_latched"),
    ),
)
def test_arm_result_crosses_real_daemon_socket_and_reaches_tui_row(
    tmp_path,
    status,
    hold_reason,
):
    require_abstract_socket_runtime()
    suffix = f"{os.getpid()}-{uuid.uuid4().hex}"
    event_socket = f"@test-t5-events-{suffix}"
    status_socket = f"@test-t5-status-{suffix}"
    server = ObservabilityServer(
        event_socket=event_socket,
        status_socket=status_socket,
        lock_path=tmp_path / "observability.lock",
        run_directory=tmp_path / "runs",
        run_id="task5-integration",
    )
    server.start()
    try:
        supervisor = supervisor_fixture(status, hold_reason)
        node = CallbackHarness(EventClient(event_socket), supervisor)
        node._on_arm_status(arm_status(status))

        client = ObservabilityClient(status_socket, request_timeout_s=0.2)

        def arm_result_received():
            current = client.poll()
            if current is None:
                return None
            recent = current.payload["recent_events"]
            if "ARM_RESULT" not in recent:
                return None
            if status == "FUTURE_ARM_STATUS" and "CONTRACT_VIOLATION" not in recent:
                return None
            return current

        observed = wait_for(arm_result_received)
        assert observed is not None
        arm_event = observed.payload["recent_events"]["ARM_RESULT"]
        assert arm_event["payload"]["raw_status"] == status
        assert arm_event["payload"]["mission_id"] == 71
        assert arm_event["payload"]["hold_reason"] == hold_reason
        assert arm_event["payload"]["arm_posture"] == "STOWED_LOCKED"
        assert "state=" in arm_event["payload"]["source_detail"]
        if status == "FUTURE_ARM_STATUS":
            violation = observed.payload["recent_events"][
                "CONTRACT_VIOLATION"
            ]
            assert violation["payload"]["raw_status"] == status
            assert violation["payload"]["stamp"] == {
                "sec": 123,
                "nanosec": 200_000_000,
            }
            assert supervisor.calls == []
        else:
            assert node.applied_results == [supervisor.result]

        async def scenario():
            app = DashboardApp(
                GatewayClient(),
                observability_client=client,
                poll_interval_s=60,
            )
            async with app.run_test() as pilot:
                app.refresh_status()
                await pilot.pause()
                text = app.query_one("#observability-status").render().plain
                assert status in text
                assert "stamp=123.200000000" in text
                assert "mission_id=71" in text
                assert hold_reason in text

        asyncio.run(scenario())
    finally:
        server.stop()


def test_adapter_failure_does_not_change_applied_hold_result():
    class FailingClient:
        def emit(self, _event):
            raise OSError("observability unavailable")

    result = SupervisorResult(
        state="FAILED_HOLD",
        hold_reason="arm_failure:FAILED",
    )
    failure = FailureRecord(
        wire_status="FAILED",
        mission_id=71,
        stamp_s=123.2,
        last_locked_posture="STOWED_LOCKED",
        operation="PICKUP",
        arm_latched=True,
    )
    supervisor = Supervisor(result, failure)
    node = CallbackHarness(FailingClient(), supervisor)

    started = time.perf_counter()
    node._on_arm_status(arm_status("FAILED"))
    elapsed = time.perf_counter() - started

    assert node.applied_results == [result]
    assert elapsed < 0.05


def test_duplicate_known_heartbeat_is_not_a_contract_violation():
    class RecordingClient:
        def __init__(self):
            self.events = []

        def emit(self, event):
            self.events.append(event)
            return True

    result = SupervisorResult(state="READY")
    supervisor = Supervisor(result, None)
    client = RecordingClient()
    node = CallbackHarness(client, supervisor)
    message = arm_status("STOWED_LOCKED")

    node._on_arm_status(message)
    node._on_arm_status(message)

    assert [event["event_type"] for event in client.events] == ["ARM_RESULT"]


def test_current_posture_heartbeat_wins_over_prior_failure_posture():
    class RecordingClient:
        def __init__(self):
            self.events = []

        def emit(self, event):
            self.events.append(event)
            return True

    result = SupervisorResult(state="FAILED_HOLD")
    failure = FailureRecord(
        wire_status="FAILED",
        mission_id=71,
        stamp_s=122.0,
        last_locked_posture="STOWED_LOCKED",
        operation="PICKUP",
        arm_latched=True,
    )
    client = RecordingClient()
    node = CallbackHarness(client, Supervisor(result, failure))

    node._on_arm_status(arm_status("CARRYING_LOCKED"))

    event, = client.events
    assert event["payload"]["raw_status"] == "CARRYING_LOCKED"
    assert event["payload"]["arm_posture"] == "CARRYING_LOCKED"
