import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class YoloBridge(Node):
    def __init__(self):
        super().__init__("yolo_bridge")

        self.image_center_x = 320
        self.center_position = 2048
        self.gain = 2

        self.sub = self.create_subscription(
            Int32MultiArray,
            "/yolo/target_center",
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            Int32MultiArray,
            "/dynamixel/goal_position",
            10
        )

    def callback(self, msg):
        cx = int(msg.data[0])

        error = cx - self.image_center_x
        goal_position = int(self.center_position + error * self.gain)
        goal_position = max(0, min(4095, goal_position))

        out = Int32MultiArray()
        out.data = [1, goal_position]

        self.pub.publish(out)

        self.get_logger().info(f"cx={cx} -> goal={goal_position}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
