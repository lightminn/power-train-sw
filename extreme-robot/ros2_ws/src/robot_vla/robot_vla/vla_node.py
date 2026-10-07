"""Mock/text VLA backend which can only publish high-level TaskCommand messages."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robot_arm_msgs.msg import TaskCommand, TaskResult

from robot_vla.command_adapter import parse_text


class VlaNode(Node):
    def __init__(self):
        super().__init__('vla_node')
        self.declare_parameter('text_topic', '/vla/text_command')
        self.declare_parameter('command_topic', '/vla/command')
        self.declare_parameter('result_topic', '/vla/result')
        self.declare_parameter('default_confidence', 1.0)
        self.declare_parameter('first_mission_id', 1)
        self.declare_parameter('mock_target_frame', 'base_link')
        self.declare_parameter('mock_target_z', 0.5)
        self._next_mission_id = int(self.get_parameter('first_mission_id').value)
        self._publisher = self.create_publisher(
            TaskCommand, self.get_parameter('command_topic').value, 10)
        self.create_subscription(
            String, self.get_parameter('text_topic').value, self._on_text, 10)
        self.create_subscription(
            TaskResult, self.get_parameter('result_topic').value, self._on_result, 10)
        self.get_logger().info('VLA adapter ready; high-level commands only')

    def _on_text(self, msg):
        try:
            command, target, tool = parse_text(msg.data)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        out = TaskCommand()
        out.header.stamp = self.get_clock().now().to_msg()
        out.command = command
        out.tool_type = tool
        out.target_object = target
        out.target_pose.header = out.header
        out.target_pose.header.frame_id = self.get_parameter('mock_target_frame').value
        out.target_pose.pose.position.z = float(
            self.get_parameter('mock_target_z').value)
        out.target_pose.pose.orientation.w = 1.0
        out.confidence = float(self.get_parameter('default_confidence').value)
        out.mission_id = self._next_mission_id
        self._next_mission_id += 1
        self._publisher.publish(out)

    def _on_result(self, msg):
        self.get_logger().info(
            f'mission={msg.mission_id} success={msg.success} '
            f'state={msg.state} reason={msg.reason}')


def main(args=None):
    rclpy.init(args=args)
    node = VlaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
