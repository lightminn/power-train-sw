"""Installed-entrypoint smoke for terrain_map (spec §7.2, domain 77).

Run inside the ROS image after ``colcon build`` and sourcing the install space.
Never touches a camera or a motor: a fixture publishes rendered depth,
CameraInfo, an optical-frame gyro and kinematics-made ``/wheel_states`` for a
straight drive toward a box.  Checks: an OK status, obstacle cells where the
box is, the pose matching the drive, and a clean SIGINT exit.

    ROS_DOMAIN_ID=77 python3 ros2/src/powertrain_ros/test/installed_terrain_map.py
    # negative control: a (nearly) doubled wheel radius must move the box → FAIL
    ROS_DOMAIN_ID=77 python3 ros2/src/powertrain_ros/test/installed_terrain_map.py --inject-double-radius
    # the same drive recorded to a bag and played with `ros2 bag play --rate 1.0`
    ROS_DOMAIN_ID=77 python3 ros2/src/powertrain_ros/test/installed_terrain_map.py --bag-play
"""
import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu

from powertrain_msgs.msg import WheelStates
from terrain_map import contract as C
from terrain_map.tests.odometry_sim import Command

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import terrain_map_fixture as F  # noqa: E402

DRIVE_S = 4.5
BOX_TOLERANCE_M = 0.2
POSE_TOLERANCE_M = 0.1


def wait_for_node(name="/terrain_map", timeout_s=20.0) -> None:
    """The scenario starts with a 1.5 s stop for bias learning: publish only once the node listens."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        listed = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True).stdout.split()
        if name in listed:
            time.sleep(1.0)                                 # subscription matching
            return
        time.sleep(0.2)
    raise RuntimeError(f"{name} did not appear within {timeout_s} s")


def drain(udp, packets):
    while True:
        try:
            packets.append(C.parse_packet(udp.recv(2048)))
        except BlockingIOError:
            return


def publish_live(events, udp, packets):
    rclpy.init()
    fixture = rclpy.create_node("installed_terrain_map_fixture")
    pubs = {
        F.DEPTH_TOPIC: fixture.create_publisher(Image, F.DEPTH_TOPIC, qos_profile_sensor_data),
        F.INFO_TOPIC: fixture.create_publisher(CameraInfo, F.INFO_TOPIC, qos_profile_sensor_data),
        F.GYRO_TOPIC: fixture.create_publisher(Imu, F.GYRO_TOPIC, qos_profile_sensor_data),
        F.WHEEL_TOPIC: fixture.create_publisher(WheelStates, F.WHEEL_TOPIC, 10),
    }
    try:
        wait_for_node()
        wall0 = time.monotonic()
        for e in events:
            delay = wall0 + (e.stamp_s - F.START) - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            pubs[e.topic].publish(e.message)
            drain(udp, packets)
    finally:
        fixture.destroy_node()
        rclpy.try_shutdown()


def play_bag(events, udp, packets):
    from test_terrain_map_replay import write_bag
    root = tempfile.mkdtemp(prefix="terrain_map_bag_")
    bag = os.path.join(root, "drive")
    write_bag(bag, [(e.stamp_s, e.topic, e.message) for e in events])
    wait_for_node()
    # --delay: the player matches subscribers before the first message (best effort drops it otherwise).
    player = subprocess.Popen(["ros2", "bag", "play", "--rate", "1.0", "--delay", "2.0", bag], start_new_session=True,
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    deadline = time.monotonic() + DRIVE_S + 10.0
    while player.poll() is None and time.monotonic() < deadline:
        time.sleep(0.01)
        drain(udp, packets)
    if player.poll() is None:
        os.killpg(player.pid, signal.SIGKILL)


def verdict(packets, truth) -> tuple[dict, str | None]:
    status = [p for p in packets if isinstance(p, C.StatusPacket)]
    ok = [s for s in status if s.status == "OK" and s.map_epoch == 0 and s.pose is not None]
    evidence = {"status_packets": len(status), "ok_packets": len(ok),
                "last_status": None if not status else [status[-1].status, status[-1].reason]}
    if not ok:
        return evidence, "FAIL: no OK status with a pose"
    x_true, _, _ = truth(F.START + DRIVE_S)
    evidence["pose_x_m"] = round(ok[-1].pose.x_m, 3)
    evidence["pose_x_true_m"] = round(x_true, 3)
    evidence["proc_ms"] = ok[-1].proc_ms
    xs = []
    for t in (p for p in packets if isinstance(p, C.TilePacket) and p.map_epoch == 0):
        _, codes, _ = C.unpack_cells(t.cells, ok[-1].tile_cells)
        xs += [(t.tx * ok[-1].tile_cells + i % ok[-1].tile_cells + 0.5) * ok[-1].cell_m
               for i, c in enumerate(codes) if c == C.OBSTACLE]
    evidence["obstacle_cells"] = len(xs)
    if not xs:
        return evidence, "FAIL: no obstacle cells"
    evidence["box_x_m"] = round(float(np.median(xs)), 3)
    if abs(evidence["box_x_m"] - F.BOX_CENTRE_X) > BOX_TOLERANCE_M:
        return evidence, "FAIL: box is not where the world puts it"
    if abs(ok[-1].pose.x_m - x_true) > POSE_TOLERANCE_M:
        return evidence, "FAIL: pose does not follow the drive"
    return evidence, None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--inject-double-radius", action="store_true",
                        help="negative control: wheel_radius_m 0.2 (1.93x) → the box must move")
    parser.add_argument("--bag-play", action="store_true", help="feed the node with `ros2 bag play --rate 1.0`")
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "77":
        print("refusing to run outside ROS_DOMAIN_ID=77", file=sys.stderr)
        return 2
    prefix = subprocess.run(["ros2", "pkg", "prefix", "powertrain_ros"],
                            capture_output=True, text=True, check=True).stdout.strip()
    events, truth = F.scenario(F.drive_then(Command("skid", v=0.3)), DRIVE_S)
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(("127.0.0.1", 0))
    udp.setblocking(False)
    env = {k: v for k, v in os.environ.items() if k != "POWERTRAIN_OPERATOR_SESSION_FILE"}
    params = ["-p", "operator_host:=127.0.0.1", "-p", f"operator_port:={udp.getsockname()[1]}",
              "-p", "pixel_stride:=1"]
    if args.inject_double_radius:
        params += ["-p", "wheel_radius_m:=0.2"]            # 1.93x — the largest value the config accepts
    node = subprocess.Popen(["ros2", "run", "powertrain_ros", "terrain_map", "--ros-args", *params],
                            env=env, start_new_session=True)
    packets = []
    evidence = {"install_prefix": prefix, "mode": "bag-play" if args.bag_play else "live"}
    try:
        (play_bag if args.bag_play else publish_live)(events, udp, packets)
        end = time.monotonic() + 0.3
        while time.monotonic() < end:
            time.sleep(0.01)
            drain(udp, packets)
        found, failure = verdict(packets, truth)
        evidence.update(found)
    finally:
        os.killpg(node.pid, signal.SIGINT)
        try:
            evidence["exit_code"] = node.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(node.pid, signal.SIGKILL)
            evidence["exit_code"] = "killed"
        udp.close()
    if failure is None and evidence["exit_code"] != 0:
        failure = "FAIL: node did not exit cleanly on SIGINT"
    print(json.dumps(dict(evidence, verdict=failure or "PASS")))
    return 1 if failure else 0


if __name__ == "__main__":
    raise SystemExit(main())
