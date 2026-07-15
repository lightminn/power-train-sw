import time
from types import SimpleNamespace
import uuid

from builtin_interfaces.msg import Time

from powertrain_observability.client import EventClient
from powertrain_ros.chassis_node import ChassisNode
from chassis.chassis_manager import ChassisManager
from chassis.kinematics import default_geometry
from corner_module.config import CornerConfig
from corner_module.corner_module import CornerModule
from corner_module.fake import FakeDrive, FakeSteer
from corner_module.null_steer import NullSteer


class Publisher:
    def publish(self, _message):
        pass


class Logger:
    def error(self, _message):
        pass

    def info(self, _message):
        pass


def tick_node(manager, event_client):
    return SimpleNamespace(
        cm=manager,
        _authority_enabled=False,
        _safety_required=False,
        _mission_supervisor_enabled=False,
        _overrun_count=0,
        _wheel_telemetry_failed=False,
        _observability_event_client=event_client,
        _last_can_health_event_ns=0,
        _can_health_event_period_ns=0,
        _can_bus_sampler=None,
        _now_ms=lambda: 0.0,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=Time),
        ),
        get_logger=lambda: Logger(),
        pub_wheels=Publisher(),
    )


def fake_chassis_manager():
    cfg = CornerConfig()
    corners = {}
    for wheel in default_geometry().wheels:
        steer = FakeSteer() if wheel.steerable else NullSteer()
        corners[wheel.name] = CornerModule(steer, FakeDrive(), cfg)
    manager = ChassisManager(corners)
    manager.connect()
    return manager


def test_disconnected_daemon_never_blocks_control_callback():
    manager = fake_chassis_manager()
    missing = "@test-chassis-event-missing-" + uuid.uuid4().hex
    node = tick_node(manager, EventClient(missing))

    started = time.perf_counter()
    ChassisNode._tick(node)
    elapsed = time.perf_counter() - started

    assert elapsed < 0.05
    assert manager.snapshot().chassis_mode in {
        "IDLE", "ARMED", "ESTOP",
    }
