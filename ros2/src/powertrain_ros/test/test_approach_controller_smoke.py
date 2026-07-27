"""approach_controller 설치 엔트리포인트 실기동 스모크 (도메인77 격리).

fixture: 정적 TF + /detected_objects + /odom 발행 + mock mission_arrive_pickup 서버.
단언: /approach/active=true, /autonomy/cmd_vel 전진 크립, 정렬 시 서비스 호출.
"""
import os
import signal
import subprocess
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import Twist, TransformStamped          # noqa: E402
from nav_msgs.msg import Odometry                              # noqa: E402
from std_msgs.msg import Bool                                  # noqa: E402
from std_srvs.srv import Trigger                               # noqa: E402
from tf2_ros import StaticTransformBroadcaster                 # noqa: E402
from robot_arm_msgs.msg import DetectedObject, DetectedObjectArray  # noqa: E402


def _spawn_node():
    env = dict(os.environ, ROS_DOMAIN_ID="77")
    return subprocess.Popen(
        ["ros2", "run", "powertrain_ros", "approach_controller",
         "--ros-args", "-p", "enabled:=true", "-p", "consecutive:=2.0",
         "-p", "stop_m:=1.0", "-p", "engage_m:=2.0",
         "-p", "lat_tol:=0.08", "-p", "dist_tol:=0.08", "-p", "v_settle:=0.05"],
        env=env, preexec_fn=os.setsid,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


@pytest.mark.timeout(40)
def test_approach_controller_fires_arrive_on_alignment():
    os.environ["ROS_DOMAIN_ID"] = "77"
    rclpy.init()
    node = rclpy.create_node("approach_smoke_fixture")

    got = {"active": False, "creep": False, "called": False}
    node.create_subscription(Bool, "/approach/active",
                             lambda m: got.__setitem__("active", got["active"] or m.data), 10)
    node.create_subscription(
        Twist, "/autonomy/cmd_vel",
        lambda m: got.__setitem__("creep", got["creep"] or m.linear.x > 0.0), 10)

    def _srv(req, resp):
        got["called"] = True
        resp.success = True
        resp.message = "mock"
        return resp
    node.create_service(Trigger, "/chassis_node/mission_arrive_pickup", _srv)
    node.create_service(Trigger, "/chassis_node/mission_arrive_drop", _srv)

    stf = StaticTransformBroadcaster(node)
    t = TransformStamped()
    t.header.frame_id = "base_link"
    t.child_frame_id = "arm_cam"
    t.transform.rotation.w = 1.0                # identity: pose 그대로 base_link
    stf.sendTransform(t)

    pub_det = node.create_publisher(DetectedObjectArray, "/detected_objects", 10)
    pub_odom = node.create_publisher(Odometry, "/odom", 10)

    proc = _spawn_node()
    try:
        t0 = time.time()
        stage = "approach"
        while time.time() - t0 < 30 and not got["called"]:
            arr = DetectedObjectArray()
            arr.header.frame_id = "arm_cam"
            arr.header.stamp = node.get_clock().now().to_msg()
            o = DetectedObject()
            o.class_name = "box"
            o.confidence = 0.9
            if got["active"] and got["creep"]:
                stage = "align"
            o.pose.position.x = 1.0 if stage == "align" else 1.8
            o.pose.position.y = 0.0
            arr.objects = [o]
            pub_det.publish(arr)
            od = Odometry()
            od.header.stamp = node.get_clock().now().to_msg()
            od.twist.twist.linear.x = 0.0
            pub_odom.publish(od)
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.05)
        assert got["active"], "/approach/active 가 true가 되지 않음"
        assert got["creep"], "/autonomy/cmd_vel 전진 크립 없음"
        assert got["called"], "정렬 후 mission_arrive_pickup 미호출"
    finally:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        node.destroy_node()
        rclpy.shutdown()
