import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image
from robot_arm_msgs.msg import DetectedObject
from cv_bridge import CvBridge
import cv2, math, sys, time

class Capture(Node):
    def __init__(self):
        super().__init__('capture_pick')
        self.bridge = CvBridge()
        self.got_img = False
        self.got_pick = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Image, '/perception/debug_image', self.on_img, 10)
        self.create_subscription(DetectedObject, '/pick_target', self.on_pick, latched)

    def on_img(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite('/root/ros2_ws/debug_capture.png', img)
        self.got_img = True

    def on_pick(self, msg):
        q = msg.pose.orientation
        yaw = math.degrees(2*math.atan2(q.z, q.w))
        print(f'PICK: {msg.class_name} conf={msg.confidence:.2f} '
              f'pos=({msg.pose.position.x:.3f},{msg.pose.position.y:.3f},{msg.pose.position.z:.3f}) '
              f'yaw_deg={yaw:.1f}', flush=True)
        self.got_pick = True

rclpy.init()
node = Capture()
deadline = time.time() + float(sys.argv[1]) if len(sys.argv) > 1 else time.time() + 60
while rclpy.ok() and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.5)
    if node.got_img and node.got_pick:
        break
node.destroy_node()
rclpy.shutdown()
print('DONE got_img=%s got_pick=%s' % (node.got_img, node.got_pick))
