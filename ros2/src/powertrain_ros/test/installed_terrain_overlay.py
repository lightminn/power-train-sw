"""Installed-entrypoint smoke for terrain_overlay (spec §7.2, domain 77).

Run inside the ROS image after ``colcon build`` and sourcing the install
space.  Never touches a camera: a fixture node publishes synthetic depth and
CameraInfo.  Checks: OK packet with an in-path outline arrives; after the
fixture stops publishing depth, DEPTH_STALE arrives within the 0.65 s
contract; SIGINT stops the node cleanly.

    ROS_DOMAIN_ID=77 python3 ros2/src/powertrain_ros/test/installed_terrain_overlay.py
    # negative control: re-inject the PR #13 double depth scale → must FAIL
    ROS_DOMAIN_ID=77 python3 ros2/src/powertrain_ros/test/installed_terrain_overlay.py --inject-double-scale
"""
import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from terrain_overlay.tests.scenes import COLOR, DEPTH, default_scene

STALE_CONTRACT_S = 0.65
SLACK_S = 0.05


def info(camera):
    message = CameraInfo()
    message.width, message.height = camera.width, camera.height
    message.k = [camera.fx, 0.0, camera.cx, 0.0, camera.fy, camera.cy, 0.0, 0.0, 1.0]
    message.distortion_model = "plumb_bob"
    message.d = [0.0] * 5
    return message


def depth_image(array, stamp_s):
    message = Image()
    message.header.stamp.sec = stamp_s
    message.height, message.width = array.shape
    message.encoding, message.is_bigendian = "16UC1", 0
    message.step = array.shape[1] * 2
    message.data = array.tobytes()
    return message


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--inject-double-scale", action="store_true",
                        help="negative control: metres scaled twice (PR #13 defect)")
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "77":
        print("refusing to run outside ROS_DOMAIN_ID=77", file=sys.stderr)
        return 2
    prefix = subprocess.run(["ros2", "pkg", "prefix", "powertrain_ros"],
                            capture_output=True, text=True, check=True).stdout.strip()
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(("127.0.0.1", 0))
    udp.setblocking(False)
    env = {k: v for k, v in os.environ.items() if k != "POWERTRAIN_OPERATOR_SESSION_FILE"}
    node = subprocess.Popen(
        ["ros2", "run", "powertrain_ros", "terrain_overlay", "--ros-args",
         "-p", "operator_host:=127.0.0.1", "-p", f"operator_port:={udp.getsockname()[1]}"]
        + (["-p", "depth_scale_m:=6.25e-08"] if args.inject_double_scale else []),
        env=env, start_new_session=True,
    )
    rclpy.init()
    fixture = rclpy.create_node("installed_terrain_fixture")
    depth = fixture.create_publisher(Image, "/l515/depth/image_rect_raw", qos_profile_sensor_data)
    depth_info = fixture.create_publisher(CameraInfo, "/l515/depth/camera_info", qos_profile_sensor_data)
    color_info = fixture.create_publisher(CameraInfo, "/l515/color/camera_info", qos_profile_sensor_data)
    scene = default_scene()
    packets = []

    def drain():
        while True:
            try:
                packets.append((time.monotonic(), json.loads(udp.recv(2048))))
            except BlockingIOError:
                return

    evidence = {"install_prefix": prefix}
    try:
        stamp = 0
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            stamp += 1
            depth_info.publish(info(DEPTH))
            color_info.publish(info(COLOR))
            depth.publish(depth_image(scene, stamp))
            time.sleep(0.1)
            drain()
            if any(p["status"] == "OK" and p["obstacles"] for _, p in packets):
                break
        ok = [p for _, p in packets if p["status"] == "OK" and p["obstacles"]]
        evidence["ok_packets"] = len(ok)
        if not ok or not ok[-1]["obstacles"][0]["in_path"]:
            print(json.dumps(dict(evidence, verdict="FAIL: no OK packet with an in-path outline")))
            return 1
        evidence["proc_ms"] = ok[-1]["proc_ms"]
        stopped = time.monotonic()
        while time.monotonic() < stopped + 1.5:
            time.sleep(0.01)
            drain()
        stale = [t for t, p in packets if t > stopped and p["reason"] == "DEPTH_STALE"]
        evidence["stale_after_s"] = None if not stale else round(stale[0] - stopped, 3)
        if not stale or stale[0] - stopped > STALE_CONTRACT_S + SLACK_S:
            print(json.dumps(dict(evidence, verdict="FAIL: DEPTH_STALE contract")))
            return 1
    finally:
        os.killpg(node.pid, signal.SIGINT)
        try:
            evidence["exit_code"] = node.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(node.pid, signal.SIGKILL)
            evidence["exit_code"] = "killed"
        fixture.destroy_node()
        rclpy.try_shutdown()
        udp.close()
    if evidence["exit_code"] != 0:
        print(json.dumps(dict(evidence, verdict="FAIL: node did not exit cleanly on SIGINT")))
        return 1
    print(json.dumps(dict(evidence, verdict="PASS")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
