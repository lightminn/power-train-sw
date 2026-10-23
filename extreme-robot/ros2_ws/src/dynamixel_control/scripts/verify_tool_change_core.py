"""설치된 순수 교체 코어를 실제 DDS 관절 피드백으로 실행하는 검증용 fixture."""

from pathlib import Path
import sys
import time

import rclpy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from dynamixel_control.auto_tool_change import AutoToolChange
from dynamixel_control.toolchanger_lock import ToolchangerLock
from dynamixel_control.toolchanger_sequence import ToolchangerSequence

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'test'))
from auto_tool_change_fixtures import dock_fixture
from test_toolchanger_lock import IO


def main():
    rclpy.init()
    node = rclpy.create_node('tool_change_core_verifier')
    targets = []
    positions = dict.fromkeys(dock_fixture()['joint_names'], 0.0)
    joints = node.create_publisher(JointState, '/tool_change_probe/joints', 10)
    command = node.create_publisher(JointTrajectory, '/tool_change_probe/target', 10)
    def move(names, values):
        command.publish(JointTrajectory(joint_names=names,
                                        points=[JointTrajectoryPoint(positions=values)]))
    def receive_target(msg):
        assert len(msg.points) == 1
        targets.append(msg)
        positions.update(zip(msg.joint_names, msg.points[0].positions))
    node.create_subscription(JointTrajectory, '/tool_change_probe/target', receive_target, 10)
    lock = ToolchangerLock(IO())
    lock.capture_zero('LOCKED')
    lock.enable()
    core = AutoToolChange(ToolchangerSequence(lock, lambda: None), move, lambda: None,
                         lambda: None)
    def receive_joints(msg):
        core.observe(dict(zip(msg.name, msg.position)), dict(zip(msg.name, msg.velocity)))
    node.create_subscription(JointState, '/tool_change_probe/joints', receive_joints, 10)
    def publish():
        msg = JointState(name=list(positions), position=list(positions.values()),
                         velocity=[0.0] * len(positions))
        msg.header.stamp = node.get_clock().now().to_msg()
        joints.publish(msg)
    def poll():
        lock.observe()
        core.poll()
    node.create_timer(0.05, publish)
    node.create_timer(0.05, poll)
    end = time.monotonic() + 10
    started = False
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if core.feedback and not started:
            core.start(dock_fixture(), '1', '2', 'dual_motor_gripper',
                       feedback_timeout_s=0.5, home_timeout_s=2.0,
                       home_tolerance_rad=0.01, unlock_direction=-1, lock_direction=1)
            started = True
        if core.state == 'DONE':
            break
        assert core.state != 'ABORTED', core.error
    assert core.state == 'DONE' and len(targets) == 7, (core.state, targets)
    assert core.take_confirm()['state'] == 'LOCKED'
    assert core.take_confirm() is None
    print('PASS installed core + DDS: seven single-target moves, fresh feedback arrival, final lock confirm once', flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
