"""Real Humble/RMW receipt timestamps; execute in the built ROS environment."""
import time
import uuid

from geometry_msgs.msg import Twist
import rclpy
from rclpy.context import Context

from powertrain_ros.chassis_node import ChassisNode
from powertrain_ros.command_receipt import ReceiptTimeCallback, ReceiptTimeExecutor


def test_missing_executor_metadata_never_delivers_command():
    calls = []
    ReceiptTimeCallback(lambda *args: calls.append(args))(Twist())
    assert calls == []


def test_real_dds_metadata_survives_executor_backlog_and_rejects_old_twist():
    from types import SimpleNamespace
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('receipt_runtime_' + uuid.uuid4().hex[:8], context=context)
    executor = ReceiptTimeExecutor(context=context)
    executor.add_node(node)
    records, commands = [], []
    receiver = SimpleNamespace(_now_s=time.monotonic)
    receiver._command_received_s = lambda info: ChassisNode._command_received_s(receiver, info)
    receiver.cm = SimpleNamespace(set=lambda *args, **kwargs: commands.append((args, kwargs)))

    def callback(message, info):
        records.append(info)
        ChassisNode._on_cmd_vel(receiver, message, info)

    topic = '/test_receipt_' + uuid.uuid4().hex
    subscription = node.create_subscription(Twist, topic, ReceiptTimeCallback(callback), 1)
    publisher = node.create_publisher(Twist, topic, 1)
    try:
        deadline = time.monotonic() + 5
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        assert publisher.get_subscription_count() == 1
        message = Twist()
        message.linear.x = .4
        publisher.publish(message)
        time.sleep(.45)  # Intentionally delay the executor after DDS receipt.
        deadline = time.monotonic() + 2
        while not records and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        assert records and records[0]['received_timestamp'] > 0
        assert (time.time_ns() - records[0]['received_timestamp']) / 1e9 > .3
        assert commands == []
        publisher.publish(message)
        deadline = time.monotonic() + 2
        while len(records) < 2 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        assert len(records) == 2 and len(commands) == 1
        assert commands[0][0] == (.4, 0.)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
