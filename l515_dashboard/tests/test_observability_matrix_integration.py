import asyncio
import os
from pathlib import Path
import socket
import sys
import time
import uuid

import pytest


ROOT = Path(__file__).parents[2]
sys.path.insert(0, str(ROOT / "motor_control"))

from chassis.chassis_manager import ChassisManager
from chassis.telemetry import build_can_health_event
from corner_module.config import CornerConfig
from corner_module.corner_module import CornerModule
from corner_module.fake import FakeDrive, FakeSteer
from corner_module.null_steer import NullSteer
from chassis.kinematics import default_geometry
from l515_dashboard.app import DashboardApp
from powertrain_observability.client import EventClient, ObservabilityClient
from powertrain_observability.server import ObservabilityServer


def require_abstract_socket_runtime():
    probe = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_PASSCRED, 1)
        probe.bind("\0test-t3-probe-" + uuid.uuid4().hex)
    except PermissionError as exc:
        pytest.skip(f"sandbox blocks AF_UNIX abstract sockets/SO_PASSCRED: {exc}")
    finally:
        probe.close()


def fake_manager():
    cfg = CornerConfig()
    corners = {}
    for wheel in default_geometry().wheels:
        steer = FakeSteer() if wheel.steerable else NullSteer()
        corners[wheel.name] = CornerModule(steer, FakeDrive(), cfg)
    manager = ChassisManager(corners)
    manager.connect()
    assert manager.arm()
    return manager


def wait_for(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.01)
    return predicate()


class GatewayClient:
    last_error = None

    def poll(self):
        return None


def test_chassis_event_crosses_real_daemon_socket_and_reaches_tui_row(tmp_path):
    require_abstract_socket_runtime()
    suffix = f"{os.getpid()}-{uuid.uuid4().hex}"
    event_socket = f"@test-t3-events-{suffix}"
    status_socket = f"@test-t3-status-{suffix}"
    server = ObservabilityServer(
        event_socket=event_socket,
        status_socket=status_socket,
        lock_path=tmp_path / "observability.lock",
        run_directory=tmp_path / "runs",
        run_id="task3-integration",
    )
    server.start()
    try:
        manager = fake_manager()
        assert EventClient(event_socket).emit(build_can_health_event(manager.snapshot()))
        client = ObservabilityClient(status_socket, request_timeout_s=0.2)
        observed = wait_for(
            lambda: (
                current
                if (current := client.poll())
                and "CAN_HEALTH" in current.payload["recent_events"]
                else None
            )
        )
        assert observed is not None

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
                assert "AK1 front_left" in text
                assert "OD16 rear_right" in text

        asyncio.run(scenario())
    finally:
        server.stop()


def test_disconnected_daemon_event_datagram_returns_without_waiting():
    missing = f"@test-t3-missing-{os.getpid()}-{uuid.uuid4().hex}"
    event = build_can_health_event(fake_manager().snapshot())
    client = EventClient(missing)

    started = time.perf_counter()
    accepted = client.emit(event)
    elapsed = time.perf_counter() - started

    assert accepted is False
    assert elapsed < 0.05
