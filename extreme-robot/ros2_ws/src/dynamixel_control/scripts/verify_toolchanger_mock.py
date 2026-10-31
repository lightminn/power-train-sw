"""설치된 mock 브리지의 ID1 영점·잠금·도구 게이트와 read-only 재시작 검사."""

import math
import json, math, os, signal, subprocess, time
from pathlib import Path
import rclpy
from std_msgs.msg import String, Int32MultiArray, MultiArrayDimension, Bool
rclpy.init()
n = rclpy.create_node('toolchanger_smoke_client')
lock_angles = []
status = {}
tool = {}
n.create_subscription(String, '/toolchanger/status', lambda m: status.update(json.loads(m.data)), 10)
n.create_subscription(String, '/tool/status', lambda m: tool.update(json.loads(m.data)), 10)
from sensor_msgs.msg import JointState
n.create_subscription(JointState, '/joint_states', lambda m: lock_angles.append(m.position[m.name.index('toolchanger_lock_wheel')]) if 'toolchanger_lock_wheel' in m.name else None, 10)
cmd = n.create_publisher(String, '/toolchanger/command', 10)
mode = n.create_publisher(String, '/control/mode', 10)
change = n.create_publisher(String, '/tool/change', 10)
torque = n.create_publisher(Int32MultiArray, '/dynamixel/torque_request', 10)
emergency = n.create_publisher(Bool, '/tool/emergency_stop', 10)

def wait(condition, timeout=10):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
        if condition():
            return
    raise AssertionError(dict(status=status, tool=tool))

def send(command, **values):
    status.pop('command_error', None)
    cmd.publish(String(data=json.dumps(dict(command=command, **values))))

def enable_tool():
    m = Int32MultiArray(data=[1, 5])
    m.layout.dim = [MultiArrayDimension(label='tool_context:' + str(tool['tool_context_generation']), size=2, stride=2)]
    torque.publish(m)
zero_file = '/tmp/lock_smoke_zero_' + str(os.getpid()) + '.json'
log = open('/tmp/lock_ros_smoke.log', 'w')
proc = subprocess.Popen(['ros2', 'run', 'dynamixel_control', 'moveit_dynamixel_bridge', '--ros-args', '-p', 'mock_mode:=true', '-p', 'control_scope:=END_EFFECTOR_ONLY', '-p', 'tool_type:=spur_1motor_gripper', '-p', 'toolchanger_enabled:=true', '-p', 'toolchanger_zero_file:=' + zero_file], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
try:
    wait(lambda : status.get('state') == 'UNCALIBRATED' and 'tool_context_generation' in tool)
    mode.publish(String(data='MANUAL'))
    wait(lambda : tool.get('control_mode') == 'MANUAL')
    enable_tool()
    time.sleep(0.2)
    rclpy.spin_once(n, timeout_sec=0.2)
    assert not tool['motion_allowed']
    send('ENABLE')
    wait(lambda : 'command_error' in status)
    assert status['position'] == 2048 and status['torque'] == 0
    send('SET_ZERO', state='LOCKED')
    wait(lambda : status.get('zero_tick') == 2048)
    assert Path(zero_file).exists()
    send('ENABLE')
    wait(lambda : status.get('ready') is True)
    change.publish(String(data='spur_1motor_gripper'))
    wait(lambda : tool.get('tool_preparation_state') == 'READY')
    enable_tool()
    wait(lambda : tool.get('motion_allowed') is True)
    lock_angles.clear()
    send('UNLOCK', direction=-1)
    wait(lambda : status.get('state') == 'UNLOCKED')
    assert status['position'] == 1792
    wait(lambda: lock_angles and abs(lock_angles[-1] + math.pi / 8) < 1e-8)
    assert status['mechanically_confirmed'] is False
    wait(lambda : tool.get('motion_allowed') is False)
    send('LOCK', direction=1)
    wait(lambda : status.get('ready') is True)
    assert status['position'] == 2048
    enable_tool()
    wait(lambda : tool.get('motion_allowed') is True)
    emergency.publish(Bool(data=True))
    wait(lambda : status.get('torque') == 0 and tool.get('emergency_stop') is True)
    assert not status['ready'] and (not tool['motion_allowed'])
    print('PASS installed ROS bridge: uncalibrated rejection, zero capture/save, jump-free enable, tool gate, -22.5deg unlock/+22.5deg lock, emergency stop', flush=True)
finally:
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
    log.close()
    n.destroy_node()
    rclpy.shutdown()
status.clear()
tool.clear()
log = open('/tmp/lock_ros_readonly.log', 'w')
proc = subprocess.Popen(['ros2', 'run', 'dynamixel_control', 'moveit_dynamixel_bridge', '--ros-args', '-p', 'mock_mode:=true', '-p', 'read_only:=true', '-p', 'control_scope:=END_EFFECTOR_ONLY', '-p', 'tool_type:=spur_1motor_gripper', '-p', 'toolchanger_zero_file:=' + zero_file], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
try:
    rclpy.init()
    n = rclpy.create_node('toolchanger_readonly_smoke')
    n.create_subscription(String, '/toolchanger/status', lambda m: status.update(json.loads(m.data)), 10)
    n.create_subscription(String, '/tool/status', lambda m: tool.update(json.loads(m.data)), 10)
    cmd = n.create_publisher(String, '/toolchanger/command', 10)
    wait(lambda : status.get('zero_tick') == 2048 and tool.get('read_only') is True)
    send('ENABLE')
    wait(lambda : 'command_error' in status)
    assert status['position'] == 2048 and status['torque'] == 0 and (not status['ready'])
    assert not tool['motion_allowed']
    print('PASS read-only cold restart: zero loaded, enable rejected, torque OFF', flush=True)
finally:
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
    log.close()
    n.destroy_node()
    rclpy.shutdown()

# 자동 교체 시퀀스: confirm 1회 발행과 결합 부하 프록시 게이트의 음성 대조까지 확인한다.
status.clear()
tool.clear()
confirms = []
zero_file = '/tmp/lock_seq_zero_' + str(os.getpid()) + '.json'
log = open('/tmp/lock_ros_sequence.log', 'w')
proc = subprocess.Popen(['ros2', 'run', 'dynamixel_control', 'moveit_dynamixel_bridge', '--ros-args', '-p', 'mock_mode:=true', '-p', 'control_scope:=END_EFFECTOR_ONLY', '-p', 'tool_type:=spur_1motor_gripper', '-p', 'toolchanger_enabled:=true', '-p', 'toolchanger_coupling_load_threshold:=120', '-p', 'toolchanger_mock_load:=0', '-p', 'toolchanger_zero_file:=' + zero_file], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
try:
    rclpy.init()
    n = rclpy.create_node('toolchanger_sequence_smoke')
    n.create_subscription(String, '/toolchanger/status', lambda m: status.update(json.loads(m.data)), 10)
    n.create_subscription(String, '/tool/status', lambda m: tool.update(json.loads(m.data)), 10)
    n.create_subscription(String, '/toolchanger/confirm', lambda m: confirms.append(json.loads(m.data)), 10)
    cmd = n.create_publisher(String, '/toolchanger/command', 10)
    mode = n.create_publisher(String, '/control/mode', 10)
    change = n.create_publisher(String, '/tool/change', 10)
    torque = n.create_publisher(Int32MultiArray, '/dynamixel/torque_request', 10)
    wait(lambda: status.get('state') == 'UNCALIBRATED' and 'tool_context_generation' in tool)
    assert status['coupling_load_threshold'] == 120
    mode.publish(String(data='MANUAL'))
    wait(lambda: tool.get('control_mode') == 'MANUAL')
    send('SET_ZERO', state='LOCKED')
    wait(lambda: status.get('zero_tick') == 2048)
    send('ENABLE')
    wait(lambda: status.get('ready') is True)
    change.publish(String(data='spur_1motor_gripper'))
    wait(lambda: tool.get('tool_preparation_state') == 'READY')
    enable_tool()
    wait(lambda: tool.get('motion_allowed') is True)

    # 분리는 부하로 통과/실패를 가르지 않는다.
    send('CHANGE', to='UNLOCKED', direction=-1)
    wait(lambda: status.get('sequence_state') == 'CONFIRMED')
    assert status['position'] == 1792 and status['coupling_proxy'] == 'RELEASED'
    wait(lambda: len(confirms) == 1)
    assert confirms[0]['state'] == 'UNLOCKED' and confirms[0]['mechanically_confirmed'] is False
    wait(lambda: tool.get('motion_allowed') is False)

    # 음성 대조: 부하 프록시가 임계값 미달이면 결합을 confirm하지 않고 토크를 끈다.
    send('CHANGE', to='LOCKED', direction=1)
    # 중단 직후 1틱은 피드백을 버린 상태다. 다음 observe까지 기다려 토크OFF를 확인한다.
    wait(lambda: status.get('sequence_state') == 'ABORTED' and status.get('torque') == 0)
    assert status['coupling_proxy'] == 'NOT_ENGAGED'
    assert status['ready'] is False
    assert len(confirms) == 1
    assert 'below threshold 120' in status['sequence_error']

    subprocess.run(['ros2', 'param', 'set', '/moveit_dynamixel_bridge', 'toolchanger_mock_load', '150'], check=True, stdout=log, stderr=subprocess.STDOUT)
    send('ENABLE')
    wait(lambda: status.get('ready') is True)
    send('CHANGE', to='UNLOCKED', direction=-1)
    wait(lambda: len(confirms) == 2 and status.get('sequence_state') == 'CONFIRMED')
    send('CHANGE', to='LOCKED', direction=1)
    # confirm은 status보다 먼저 도착한다. 두 토픽을 함께 기다린다.
    wait(lambda: len(confirms) == 3 and status.get('sequence_state') == 'CONFIRMED'
         and status.get('position') == 2048)
    assert confirms[2]['coupling_proxy'] == 'ENGAGED' and confirms[2]['peak_load'] == 150
    assert status['mechanically_confirmed'] is False
    send('CHANGE', to='LOCKED', direction=1)
    wait(lambda: status.get('sequence_state') == 'UNCHANGED')
    assert len(confirms) == 3
    print('PASS change sequence: unlock confirm, load-proxy negative control abort with torque OFF, re-enable, engaged lock confirm, no-op change', flush=True)
finally:
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
    log.close()
    n.destroy_node()
    rclpy.shutdown()
