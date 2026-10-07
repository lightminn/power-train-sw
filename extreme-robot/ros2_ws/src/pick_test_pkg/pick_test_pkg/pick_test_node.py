import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class PickTestNode(Node):
    def __init__(self):
        super().__init__('pick_test_node')
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
        )
        self.create_subscription(Point, '/fake_object_position', self.cb, 10)
        self.get_logger().info('Waiting for /fake_object_position')

    def cb(self, msg):
        self.get_logger().info(f'Received x={msg.x}, y={msg.y}, z={msg.z}')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['left_finger_joint', 'right_finger_joint']

        p = JointTrajectoryPoint()
        p.positions = [0.04, 0.04]   # 열기
        p.time_from_start.sec = 2
        goal.trajectory.points.append(p)

        self.client.wait_for_server()
        self.client.send_goal_async(goal)
        self.get_logger().info('Gripper command sent')


def main(args=None):
    rclpy.init(args=args)
    node = PickTestNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
