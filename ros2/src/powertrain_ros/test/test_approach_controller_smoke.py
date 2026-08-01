"""approach_controller 설치 엔트리포인트 실기동 스모크 (도메인77 격리).

fixture: 정적 TF + /detected_objects + /odom 발행 + mock mission_arrive_pickup 서버.
단언: 접근·정렬·서비스 호출 후 같은 mission_id의 팔 잠금 완료에서 재출발한다.
"""
import os
import signal
import subprocess
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import Twist, TransformStamped          # noqa: E402
from nav_msgs.msg import Odometry                              # noqa: E402
from std_msgs.msg import Bool, String                          # noqa: E402
from std_srvs.srv import Trigger                               # noqa: E402
from tf2_ros import StaticTransformBroadcaster                 # noqa: E402
from robot_arm_msgs.msg import (                               # noqa: E402
    ArmStatus,
    ArrivalStatus,
    DetectedObject,
    DetectedObjectArray,
)


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

    got = {
        "active": False,
        "active_seen": False,
        "creep": False,
        "called": False,
        "arrival_published": False,
        "state": "",
        "resumed_before_call": False,
        "resumed_seen": False,
    }

    def _on_active(msg):
        got["active"] = bool(msg.data)
        got["active_seen"] = got["active_seen"] or bool(msg.data)

    def _on_state(msg):
        got["state"] = str(msg.data)
        # reason="resumed"는 DONE→SEARCHING 전이 한 틱만 나타난다 → 놓치지 않게 래치.
        if "resumed" in got["state"]:
            got["resumed_seen"] = True
            if not got["called"]:
                got["resumed_before_call"] = True

    node.create_subscription(Bool, "/approach/active", _on_active, 10)
    node.create_subscription(String, "/approach/state", _on_state, 10)
    node.create_subscription(
        Twist, "/autonomy/cmd_vel",
        lambda m: got.__setitem__("creep", got["creep"] or m.linear.x > 0.0), 10)

    pub_arrival = node.create_publisher(ArrivalStatus, "/arrival_status", 10)
    pub_arm = node.create_publisher(ArmStatus, "/arm_status", 10)

    def _publish_arrival():
        # 실 chassis는 EVENT_HOLD 동안 arrival을 주기 재발행한다 → VOLATILE 유실 방지 위해
        # one-shot이 아니라 반복 발행한다(노드의 _active_mission_id 학습을 확실히).
        msg = ArrivalStatus()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.mission_id = 42
        msg.status = "ARRIVED_PICKUP"
        pub_arrival.publish(msg)
        got["arrival_published"] = True

    arrival_timer = node.create_timer(0.1, _publish_arrival)
    arrival_timer.cancel()

    def _srv(req, resp):
        del req
        got["called"] = True
        resp.success = True
        resp.message = "mock"
        # 서비스 응답 후 fixture timer가 실제 mission_id를 주기적으로 발행하기 시작한다.
        arrival_timer.reset()
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

    def _publish_arm(status, mission_id):
        msg = ArmStatus()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.mission_id = int(mission_id)
        msg.status = status
        pub_arm.publish(msg)

    def _spin_arm(status, mission_id, frames=3):
        for _ in range(frames):
            _publish_arm(status, mission_id)
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.05)

    proc = _spawn_node()
    try:
        t0 = time.time()
        stage = "approach"
        while time.time() - t0 < 30 and not got["called"]:
            # 팔 idle heartbeat는 미션 전부터 상시 오지만 재출발 권위가 아니다.
            _publish_arm("STOWED_LOCKED", 0)
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
        assert got["active_seen"], "/approach/active 가 true가 되지 않음"
        assert got["creep"], "/autonomy/cmd_vel 전진 크립 없음"
        assert got["called"], "정렬 후 mission_arrive_pickup 미호출"
        assert not got["resumed_before_call"], (
            "미션 전 STOWED_LOCKED mission_id=0 에서 재출발 오발화")

        # ACK 뒤 ARRIVED_FIRED 진입과 timer 기반 /arrival_status 발행을 기다린다.
        deadline = time.time() + 5.0
        while time.time() < deadline and not (
            got["arrival_published"] and got["state"].startswith("ARRIVED_FIRED")
        ):
            _publish_arm("STOWED_LOCKED", 0)
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.05)
        assert got["arrival_published"], "/arrival_status mission_id=42 미발행"
        assert got["state"].startswith("ARRIVED_FIRED"), (
            "서비스 ACK 후 ARRIVED_FIRED 미진입")

        # 음성대조: drive-ready 상승엣지여도 mission_id=0이면 재출발 금지.
        _spin_arm("EXECUTING", 0)
        _spin_arm("STOWED_LOCKED", 0)
        assert got["active"], "불일치 mission_id의 idle 상태가 재출발을 오발화"
        assert not got["resumed_seen"], (
            "불일치 mission_id의 STOWED_LOCKED를 완료로 오인")

        # 실제 팔 작업 진행 후 같은 mission_id의 잠금 완료만 재출발시킨다.
        # (reason="resumed"는 전이 한 틱만 나오므로 resumed_seen 래치로 판정한다.)
        _spin_arm("EXECUTING", 42)
        deadline = time.time() + 5.0
        while time.time() < deadline and not (
            not got["active"] and got["resumed_seen"]
        ):
            _publish_arm("CARRYING_LOCKED", 42)
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.05)
        assert got["resumed_seen"], (
            "같은 mission_id의 CARRYING_LOCKED에서 재출발(resumed) 미관측")
        assert not got["active"], "팔 작업 완료 후 /approach/active 가 false가 아님"
    finally:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        node.destroy_node()
        rclpy.shutdown()
