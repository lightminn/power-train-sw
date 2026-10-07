#!/usr/bin/env python3
"""Verify the running mock launch, including the persistent MoveIt FK service."""
import json
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def main():
    rclpy.init()
    node = Node('integrated_tool_mock_probe')
    status = {}
    arm_fsm = []
    samples = []
    names = [f'arm_joint_{i}' for i in range(1, 6)]
    node.create_subscription(String, '/tool/status',
                             lambda msg: status.update(json.loads(msg.data)), 10)
    node.create_subscription(String, '/fsm/state', lambda msg: arm_fsm.append(msg.data), 10)
    node.create_subscription(JointState, '/joint_states', samples.append, 10)
    change = node.create_publisher(String, '/tool/change', 10)
    mode = node.create_publisher(String, '/control/mode', 10)
    fk = node.create_client(GetPositionFK, '/compute_fk')
    move = ActionClient(node, MoveGroup, '/move_action')
    trajectory = ActionClient(node, FollowJointTrajectory,
                              '/arm_controller/follow_joint_trajectory')

    def wait(predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate():
                return
        raise RuntimeError(f'mock probe timed out: {status}')

    def positions(msg):
        current = dict(zip(msg.name, msg.position))
        return {name: current[name] for name in names}

    try:
        wait(lambda: status and samples and arm_fsm)
        if not status.get('mock_mode') or status.get('control_scope') != 'FULL_ROBOT':
            raise RuntimeError('Probe refuses any non-mock or non-full-robot bridge')
        wait(lambda: fk.service_is_ready() and move.server_is_ready()
             and trajectory.server_is_ready())
        mode.publish(String(data='MANUAL'))
        wait(lambda: status.get('control_mode') == 'MANUAL')
        # Drain in-flight FSM heartbeat samples before recording the baseline.
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=.05)
        baseline = positions(samples[-1])
        baseline_fsm = arm_fsm[-1]
        reference_pose = None
        results = []
        for tool, ids in [('dual_motor_gripper', [3, 4]),
                          ('spur_1motor_gripper', [5]),
                          ('cleaner', [6]), ('dual_motor_gripper', [3, 4])]:
            start = len(samples)
            change.publish(String(data=tool))
            wait(lambda: status.get('tool_type') == tool and len(samples) >= start + 5)
            if status.get('tool_change', {}).get('error'):
                raise RuntimeError(status['tool_change']['error'])
            assert status['tool_profile']['actuator_ids'] == ids
            assert status['mock_mode'] and arm_fsm[-1] == baseline_fsm
            assert all(positions(msg) == baseline for msg in samples[start:])
            assert move.server_is_ready() and trajectory.server_is_ready()
            request = GetPositionFK.Request()
            request.header.frame_id = 'base_link'
            request.fk_link_names = ['link_043']
            request.robot_state.joint_state = samples[-1]
            future = fk.call_async(request)
            wait(future.done)
            response = future.result()
            assert response.error_code.val == 1, response.error_code
            pose = response.pose_stamped[0].pose
            if reference_pose is None:
                reference_pose = pose
            assert pose == reference_pose
            results.append({'tool': tool, 'ids': ids,
                            'tool_fsm': status['fsm_state'],
                            'arm_fsm': arm_fsm[-1], 'arm_joint_count': len(baseline),
                            'joint_samples': len(samples) - start, 'moveit_fk': 'PASS'})
        print(json.dumps({'result': 'PASS', 'transitions': results}, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
