"""설치된 ros2 run 브리지의 전체 교체와 단계별 실패를 도메인 77에서 검증한다."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from robot_arm_msgs.msg import ChassisMode
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from dynamixel_control.qos_profiles import HEARTBEAT_QOS

TESTS = Path(__file__).resolve().parents[1] / 'test'
sys.path.insert(0, str(TESTS))
from auto_tool_change_fixtures import dock_fixture


def verify(case):
    node = rclpy.create_node('auto_exchange_verifier')
    status, lock, tool, positions, confirms, history = {}, {}, {}, {}, [], []
    def on_status(msg):
        value = json.loads(msg.data)
        status.update(value)
        history.append(value)
    node.create_subscription(String, '/toolchanger/change_status', on_status, 10)
    node.create_subscription(String, '/toolchanger/status',
                             lambda m: lock.update(json.loads(m.data)), 10)
    node.create_subscription(String, '/tool/status', lambda m: tool.update(json.loads(m.data)), 10)
    node.create_subscription(String, '/toolchanger/confirm',
                             lambda m: confirms.append(json.loads(m.data)), 10)
    node.create_subscription(JointState, '/joint_states',
                             lambda m: positions.update(zip(m.name, m.position)), 10)
    session = node.create_publisher(String, '/toolchanger/session_command', 10)
    change = node.create_publisher(String, '/toolchanger/change_tool', 10)
    chassis = node.create_publisher(ChassisMode, '/chassis_mode', HEARTBEAT_QOS)
    def heartbeat():
        msg = ChassisMode()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.mode = 'MISSION_STOP'
        chassis.publish(msg)
    node.create_timer(0.05, heartbeat)
    def wait(condition, timeout=10):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
            if condition():
                return
        raise AssertionError(dict(case=case, status=status, lock=lock, tool=tool))
    with tempfile.TemporaryDirectory(prefix='auto-exchange-') as directory:
        data = dock_fixture()
        for slot, poses in (('1', (0.04, 0.08, 0.03)), ('2', (0.12, 0.16, 0.10))):
            for name, value in zip(('approach', 'seat', 'retreat'), poses):
                pose = data['slots'][slot]['poses'][name]
                pose['positions'][3] = value
                pose['tolerance_rad'] = 0.005
        docks = Path(directory) / 'docks.json'
        docks.write_text(json.dumps(data))
        environment = os.environ.copy()
        if case not in ('normal', 'normal_cleaner', 'unarmed', 'same_tool'):
            # 별도 검증 fixture에서 설치된 클래스에 실패를 주입한다.
            (Path(directory) / 'sitecustomize.py').write_text(
                'import sys\nsys.path.insert(0, ' + repr(str(TESTS)) + ')\n'
                'from auto_tool_change_faults import install\ninstall(' + repr(case) + ')\n')
            environment['PYTHONPATH'] = directory + os.pathsep + environment.get('PYTHONPATH', '')
        logfile = Path(directory) / 'bridge.log'
        with logfile.open('w') as log:
            process = subprocess.Popen([
                'ros2', 'run', 'dynamixel_control', 'moveit_dynamixel_bridge', '--ros-args',
                '-p', 'mock_mode:=true', '-p', 'control_scope:=TOOL_CHANGE',
                '-p', 'tool_type:=spur_1motor_gripper', '-p', 'toolchanger_enabled:=true',
                '-p', 'toolchanger_zero_tick:=2048', '-p', 'toolchanger_zero_state:=LOCKED',
                '-p', 'toolchanger_dock_file:=' + str(docks)],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=environment)
            try:
                wait(lambda: status.get('state') == 'IDLE' and set(data['joint_names']).issubset(positions)
                     and lock.get('zero_tick') == 2048)
                initial = {name: positions[name] for name in data['joint_names']}
                if case != 'unarmed':
                    session.publish(String(data='{"command":"ARM"}'))
                    wait(lambda: status.get('armed') is True)
                requested = ('spur_1motor_gripper' if case == 'same_tool' else
                             'cleaner' if case == 'normal_cleaner' else 'dual_motor_gripper')
                change.publish(String(data=requested))
                if case == 'unarmed':
                    wait(lambda: 'operator arm' in status.get('error', ''))
                    assert status['moves_sent'] == 0 and not confirms
                else:
                    wait(lambda: status.get('state') in ('DONE', 'ABORTED'))
                    if case in ('normal', 'normal_cleaner'):
                        assert status['state'] == 'DONE', status
                        wait(lambda: len(confirms) == 1)
                        assert status['moves_sent'] == 7 and status['verified_ready']
                        assert status['attached_tool'] == requested and not status['armed']
                        wait(lambda: tool.get('tool_type') == requested)
                        assert tool['tool_preparation_state'] == 'READY'
                        assert all(abs(positions[n] - value) <= 0.01 for n, value in initial.items())
                        assert confirms[0]['state'] == 'LOCKED'
                        assert confirms[0]['coupling_proxy'] == 'DISABLED'
                    else:
                        assert status['state'] == 'ABORTED' and not status['armed'], status
                        assert not confirms, confirms
                        count = status['moves_sent']
                        end = time.monotonic() + 0.15
                        wait(lambda: time.monotonic() >= end)
                        assert status['moves_sent'] == count and not confirms
                        if case.startswith('abort:'):
                            assert status['step_index'] == int(case.split(':')[1]), status
                        if case == 'undershoot':
                            assert count == 2 and lock['position'] == 2048, (status, lock)
                            assert not any(h['state'] == 'UNLOCK' for h in history)
                        if case == 'same_tool':
                            assert count == 0
                    wait(lambda: lock.get('torque') == 0)
                    wait(lambda: tool and all(a.get('torque_state') == 'OFF'
                         for a in tool.get('actuators', [])))
                print(f'PASS installed exchange {case}: {status["state"]}, moves={status["moves_sent"]}, confirms={len(confirms)}', flush=True)
            except Exception:
                print(logfile.read_text(), file=sys.stderr)
                raise
            finally:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                output = logfile.read_text()
                if 'Traceback' in output or process.returncode not in (0, 130, -2):
                    raise AssertionError(output)
    node.destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--case', default='all')
    args = parser.parse_args()
    rclpy.init()
    cases = (['normal', 'normal_cleaner', 'unarmed', 'same_tool', 'undershoot', 'absent', 'ready',
              'permission', 'emergency', 'stale'] + [f'abort:{i}' for i in range(11)]
             if args.case == 'all' else [args.case])
    try:
        for case in cases:
            verify(case)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
