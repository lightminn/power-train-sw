"""Built ROS message/topology/receipt checks; fake chassis, isolated test domain."""
import time
import uuid
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.parameter import Parameter

from chassis.authority import CommandAuthority, MANUAL_SOURCE, TELEOP
from powertrain_msgs.msg import ManualDriveCommand
from powertrain_ros.chassis_node import ChassisNode
from powertrain_ros.command_receipt import ReceiptTimeCallback, ReceiptTimeExecutor
from powertrain_ros.teleop_command_node import TeleopCommandNode


@pytest.fixture
def ros():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.mark.parametrize("command_format,topic,excluded", [
    ("steering", "/teleop/drive_command", "/teleop/cmd_vel"),
    ("twist", "/teleop/cmd_vel", "/teleop/drive_command"),
])
def test_startup_selects_one_manual_endpoint_and_rejects_live_format_changes(ros, tmp_path, command_format, topic, excluded):
    # Exercise the shipped default without a parameter override.
    format_parameters = ([] if command_format == "steering" else
                         [Parameter("manual_command_format", value=command_format)])
    teleop = TeleopCommandNode(parameter_overrides=[
        Parameter("host", value="127.0.0.1"), Parameter("port", value=0),
    ] + format_parameters)
    chassis = None
    try:
        chassis = ChassisNode(parameter_overrides=[
            Parameter("fake", value=True), Parameter("authority_enabled", value=True),
            Parameter("manual_max_angular", value=0.7),
            Parameter("console_estop_latch_path", value=str(tmp_path / "estop.json")),
        ] + format_parameters)
        assert teleop.pub_drive.topic_name == topic
        publishers = {publisher.topic_name for publisher in teleop.publishers}
        subscriptions = {subscription.topic_name for subscription in chassis.subscriptions}
        assert topic in publishers and topic in subscriptions
        assert excluded not in publishers and excluded not in subscriptions
        assert chassis.cm.cfg.manual_max_omega_rad_s == pytest.approx(0.7)
        for node in (teleop, chassis):
            result = node.set_parameters([Parameter(
                "manual_command_format", value="twist" if command_format == "steering" else "steering")])
            assert not result[0].successful
            assert node.get_parameter("manual_command_format").value == command_format
    finally:
        teleop.close()
        teleop.destroy_node()
        if chassis is not None:
            chassis.close()
            chassis.destroy_node()


def test_steering_plus_assist_rejected_before_hardware_construction(ros, tmp_path):
    with pytest.raises(ValueError, match="assist_enabled=true requires manual_command_format=twist"):
        ChassisNode(parameter_overrides=[
            Parameter("fake", value=True), Parameter("authority_enabled", value=True),
            Parameter("assist_enabled", value=True),
            Parameter("console_estop_latch_path", value=str(tmp_path / "estop.json")),
        ])


def test_manual_message_survives_dds_and_old_receipt_cannot_refresh_authority(ros):
    node = rclpy.create_node("manual_receipt_" + uuid.uuid4().hex[:8])
    executor = ReceiptTimeExecutor()
    executor.add_node(node)
    authority = CommandAuthority()
    assert authority.set_mode(TELEOP)
    now_s = time.monotonic()
    authority.submit(MANUAL_SOURCE, 0.0, 0.0, now_s, steering=0.0)
    assert authority.select(now_s).ok
    receiver = SimpleNamespace(_now_s=time.monotonic, _authority=authority)
    receiver._command_received_s = lambda info: ChassisNode._command_received_s(receiver, info)
    receipts = []

    def receive(message, info):
        receipts.append(info)
        ChassisNode._on_manual_drive_command(receiver, message, info)

    topic = "/manual_receipt_" + uuid.uuid4().hex
    node.create_subscription(ManualDriveCommand, topic, ReceiptTimeCallback(receive), 1)
    publisher = node.create_publisher(ManualDriveCommand, topic, 1)
    try:
        deadline = time.monotonic() + 5
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert publisher.get_subscription_count() == 1
        # Keep the initial neutral command alive separately; delayed non-neutral
        # DDS input must not replace it when the callback eventually executes.
        message = ManualDriveCommand(speed_mps=-0.4, steering=0.6)
        publisher.publish(message)
        time.sleep(0.45)
        fresh_s = time.monotonic()
        authority.submit(MANUAL_SOURCE, 0.0, 0.0, fresh_s, steering=0.0)
        deadline = time.monotonic() + 2
        while not receipts and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert receipts
        command = authority.select(fresh_s)
        assert (command.v, command.omega, command.steering) == (0.0, 0.0, 0.0)
        publisher.publish(message)
        deadline = time.monotonic() + 2
        while len(receipts) < 2 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert len(receipts) == 2
        command = authority.select(time.monotonic())
        assert command.ok
        assert (command.v, command.omega, command.steering) == (-0.4, 0.0, 0.6)
    finally:
        executor.shutdown()
        node.destroy_node()
