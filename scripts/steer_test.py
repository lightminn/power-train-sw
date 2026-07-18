"""조향 개별 검증 — /cmd_vel 로 v,w 를 N초간 유지 후 정지. 사용: python3 steer_test.py <v> <w> <sec>"""
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

v, w, dur = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])

rclpy.init()
n = Node("steer_test")
pub = n.create_publisher(Twist, "/cmd_vel", 10)
time.sleep(0.5)

msg = Twist()
msg.linear.x = v
msg.angular.z = w
t_end = time.monotonic() + dur
print(f"명령 시작 v={v} w={w} ({dur}s)")
while time.monotonic() < t_end:
    pub.publish(msg)
    time.sleep(0.05)

stop = Twist()
pub.publish(stop)
pub.publish(stop)
time.sleep(0.2)
print("정지 완료")
n.destroy_node()
rclpy.shutdown()
