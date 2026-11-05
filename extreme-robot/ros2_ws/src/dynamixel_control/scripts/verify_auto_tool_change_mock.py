"""설치된 브리지의 교체 입력과 기존 입력 차단을 도메인 77에서 검증한다."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from robot_arm_msgs.msg import ChassisMode
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool, Int32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'test'))
from auto_tool_change_fixtures import dock_fixture
from dynamixel_control.qos_profiles import HEARTBEAT_QOS


def main():
    rclpy.init()
    node = rclpy.create_node('auto_tool_change_verifier')
    status, lock, positions, dock_status = {}, {}, {}, {}
    node.create_subscription(String, '/toolchanger/change_status',
                             lambda m: status.update(json.loads(m.data)), 10)
    node.create_subscription(String, '/toolchanger/status',
                             lambda m: lock.update(json.loads(m.data)), 10)
    node.create_subscription(JointState, '/joint_states',
                             lambda m: positions.update(zip(m.name, m.position)), 10)
    node.create_subscription(String, '/toolchanger/dock_status',
                             lambda m: dock_status.update(json.loads(m.data)), 10)
    dock_cmd = node.create_publisher(String, '/toolchanger/dock_command', 10)
    torque = node.create_publisher(Int32MultiArray, '/dynamixel/torque_request', 10)
    session = node.create_publisher(String, '/toolchanger/session_command', 10)
    normal = node.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
    dedicated = node.create_publisher(JointTrajectory, '/toolchanger/arm_trajectory', 10)
    lock_cmd = node.create_publisher(String, '/toolchanger/command', 10)
    emergency = node.create_publisher(Bool, '/tool/emergency_stop', 10)
    chassis = node.create_publisher(ChassisMode, '/chassis_mode', HEARTBEAT_QOS)
    permit = [False]

    def heartbeat():
        if permit[0]:
            msg = ChassisMode()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.mode = 'MISSION_STOP'
            chassis.publish(msg)
    node.create_timer(0.1, heartbeat)

    def wait(condition, timeout=6):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.03)
            if condition():
                return
        raise AssertionError(dict(status=status, lock=lock, joints=positions))

    def command(value):
        session.publish(String(data=json.dumps(dict(command=value))))

    def target(value):
        msg = JointTrajectory(joint_names=dock_fixture()['joint_names'])
        msg.points = [JointTrajectoryPoint(positions=[0.0, 0.0, 0.0, value, 0.0])]
        return msg

    with tempfile.TemporaryDirectory(prefix='auto-tool-') as directory:
        docks = Path(directory) / 'docks.json'
        docks.write_text(json.dumps(dock_fixture()))
        logfile = Path(directory) / 'bridge.log'
        with logfile.open('w') as log:
            process = subprocess.Popen([
                'ros2', 'run', 'dynamixel_control', 'moveit_dynamixel_bridge', '--ros-args',
                '-p', 'mock_mode:=true', '-p', 'control_scope:=TOOL_CHANGE',
                '-p', 'tool_type:=spur_1motor_gripper', '-p', 'toolchanger_enabled:=true',
                '-p', 'toolchanger_zero_tick:=2048', '-p', 'toolchanger_zero_state:=LOCKED',
                '-p', 'toolchanger_dock_file:=' + str(docks)],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                wait(lambda: len(positions) == 5 and lock.get('zero_tick') == 2048)
                dedicated.publish(target(0.05))
                wait(lambda: 'operator arm' in status.get('error', ''))
                assert abs(positions['arm_joint_4']) < 0.005
                command('ARM')
                wait(lambda: 'MISSION_STOP' in status.get('error', ''))
                assert status['armed'] is False
                permit[0] = True
                wait(lambda: chassis.get_subscription_count() > 0)
                # 다음 heartbeat가 실제로 도착한 후 요청한다.
                begin = time.monotonic()
                wait(lambda: time.monotonic() - begin > 0.25)
                command('ARM')
                wait(lambda: status.get('armed') is True)
                normal.publish(target(0.10))
                client = ActionClient(node, FollowJointTrajectory,
                                      '/arm_controller/follow_joint_trajectory')
                assert client.wait_for_server(timeout_sec=3)
                future = client.send_goal_async(FollowJointTrajectory.Goal(trajectory=target(0.10)))
                wait(future.done)
                assert not future.result().accepted
                client.destroy()
                assert abs(positions['arm_joint_4']) < 0.005
                dedicated.publish(target(0.05))
                wait(lambda: abs(positions['arm_joint_4'] - 0.05) < 0.005)
                lock_cmd.publish(String(data='{"command":"ENABLE"}'))
                wait(lambda: lock.get('torque') == 1)
                lock_cmd.publish(String(data='{"command":"CHANGE","to":"UNLOCKED","direction":-1}'))
                wait(lambda: lock.get('sequence_state') == 'CONFIRMED'
                     and lock.get('state') == 'UNLOCKED')
                permit[0] = False
                wait(lambda: status.get('state') == 'ABORTED')
                wait(lambda: lock.get('torque') == 0)
                assert status['armed'] is False
                dedicated.publish(target(0.10))
                wait(lambda: 'operator arm' in status.get('error', ''))
                assert abs(positions['arm_joint_4'] - 0.05) < 0.005
                capture = dict(command='CAPTURE', slot_id='1', pose='approach',
                               tool_type='spur_1motor_gripper', tolerance_rad=0.01, timeout_s=2.0)
                dock_cmd.publish(String(data=json.dumps(capture)))
                wait(lambda: 'torque OFF' in dock_status.get('error', ''))
                before = docks.read_bytes()
                assert json.loads(before)['slots']['1']['poses']['approach']['positions'][3] == 0.0
                torque.publish(Int32MultiArray(data=[0, 11, 14, 13, 12, 16]))
                begin = time.monotonic()
                wait(lambda: time.monotonic() - begin > 0.2)
                dock_status.clear()
                dock_cmd.publish(String(data=json.dumps(capture)))
                wait(lambda: dock_status.get('calibrated') is True and not dock_status.get('error'))
                assert abs(json.loads(docks.read_text())['slots']['1']['poses']['approach']['positions'][3] - 0.05) < 0.005
                dock_status.clear()
                dock_cmd.publish(String(data='{"command":"LOAD"}'))
                wait(lambda: dock_status.get('calibrated') is True)
                permit[0] = True
                begin = time.monotonic()
                wait(lambda: time.monotonic() - begin > 0.25)
                command('ARM')
                wait(lambda: status.get('armed') is True)
                emergency.publish(Bool(data=True))
                wait(lambda: status.get('state') == 'ABORTED' and not status['armed'])
                docks.write_text('broken json')
                dock_status.clear()
                dock_cmd.publish(String(data='{"command":"LOAD"}'))
                wait(lambda: dock_status.get('error') and not dock_status['calibrated'])
                print('PASS installed teach: torque-ON rejection, torque-OFF capture/save/load, corruption uncalibrated', flush=True)
                print('PASS installed TOOL_CHANGE: unarmed/no-permission rejection, dedicated arm + lock acceptance, mission topic/action rejection, stale abort, emergency abort', flush=True)
            finally:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                text = logfile.read_text()
                if 'Traceback' in text or process.returncode not in (0, 130, -2):
                    raise AssertionError(text)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
