"""Installed demo-mode acceptance (docs/specs/2026-11-06-demo-mode.md D1·D2).

Run only in an isolated domain77 container whose only CAN link is vcan77:
installed ``teleop_command``, ``ops_broker`` and ``chassis`` entrypoints, the
demo ``odrive_auto_clear`` service process, real TCP/ops/DDS and the real six
CAN drive drivers against six emulated ODrives and four AK steering motors.
Each case saves ``result.json`` and node logs and rejects any Traceback.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parent))
from installed_can_auto_resume import Emulator  # noqa: E402

CASES = ('demo_resume', 'latch_without_demo', 'other_error_latches',
         'stale_seq_rejected', 'motor_error_outside_allow')
CLEAR = 24


class DemoEmulator(Emulator):
    """Adds GET_MOTOR_ERROR (cmd 3) replies to the shared ODrive emulator."""

    def __init__(self):
        self.motor_error = {n: 0 for n in range(11, 17)}
        super().__init__()

    def run(self):
        original_recv = self.bus.recv

        def recv(timeout):
            message = original_recv(timeout)
            if (message is not None and not message.is_extended_id and message.is_remote_frame
                    and message.arbitration_id & 31 == 3):
                node = message.arbitration_id >> 5
                if node in self.motor_error:
                    with self.lock:
                        self.send(node, 3, struct.pack('<Q', self.motor_error[node]))
                        self.commands.append({'stamp_s': time.monotonic(), 'node': node,
                                              'command': 3, 'value': None, 'payload': []})
                return None
            return message
        self.bus.recv = recv
        super().run()

    def fail_axis(self, node, axis_error, motor_error):
        with self.lock:
            self.error[node], self.state[node] = axis_error, 1
            self.motor_error[node] = motor_error
            self.velocity[node] = 0.
            self.heartbeat(node)
            return len(self.commands)

    def run_clear(self, node):
        # A real Clear_Errors also resets motor.error.
        with self.lock:
            if self.error[node] == 0:
                self.motor_error[node] = 0


def run_case(case, evidence):
    import rclpy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from powertrain_msgs.msg import SafetyVerdict, ManualDriveCommand
    from powertrain_ros.command_receipt import ReceiptTimeExecutor
    import powertrain_ros.chassis_node as chassis_module
    from motor_control.laptop.remote_operation_client import ClientInput, encode_frame
    from motor_control.laptop.ops_channel_client import OpsChannelClient

    installed_path = str(Path(chassis_module.__file__).resolve())
    assert '/install/' in installed_path, installed_path
    assert hasattr(ManualDriveCommand(), 'source_received_s'), 'rebuild powertrain_msgs'
    demo = case != 'latch_without_demo'
    evidence.mkdir(parents=True, exist_ok=True)
    token = evidence / 'ops_console.token'
    token.write_text('isolated-demo-mode-token')
    token.chmod(0o600)
    emulator = DemoEmulator()
    rclpy.init()
    node = rclpy.create_node('demo_mode_acceptance_' + uuid.uuid4().hex[:8])
    executor = ReceiptTimeExecutor()
    executor.add_node(node)
    safety_publisher = node.create_publisher(SafetyVerdict, '/safety_verdict', 10)
    direct_arm = node.create_publisher(String, '/chassis_node/demo_auto_arm', 10)
    observed = {'safety': {}, 'gateway': {}}
    node.create_subscription(String, '/chassis/safety_state',
        lambda msg: observed.__setitem__('safety', json.loads(msg.data)), 10)
    node.create_subscription(String, '/teleop/gateway_state',
        lambda msg: observed.__setitem__('gateway', json.loads(msg.data)), 10)
    processes, logs = [], []
    raw = ops = None
    replies = []
    sequence = 0
    session_id = str(uuid.uuid4())
    sample = ClientInput()
    last_send = last_sensors = 0.

    def pump(predicate=lambda: False, timeout=5., *, duration=None, label='condition'):
        nonlocal sequence, last_send, last_sensors
        start = time.monotonic()
        while time.monotonic() - start < (duration if duration is not None else timeout):
            now = time.monotonic()
            for process in processes:
                assert process.poll() is None, (process.args, process.returncode)
            assert emulator.failure is None, emulator.failure
            if now - last_sensors >= .025:
                verdict = SafetyVerdict()
                verdict.status, verdict.distance_mm = SafetyVerdict.VALID, 1000.
                verdict.estop_required = False
                verdict.header.stamp = node.get_clock().now().to_msg()
                verdict.detail = 'isolated vcan demo acceptance sensor'
                safety_publisher.publish(verdict)
                last_sensors = now
            if raw is not None and now - last_send >= .015:
                raw.sendall(encode_frame(sample, session_id=session_id, sequence=sequence,
                                         client_monotonic_ns=time.monotonic_ns()))
                sequence += 1
                last_send = now
                try:
                    raw.recv(65536)
                except BlockingIOError:
                    pass
            if ops is not None:
                replies.extend(ops.pump())
            for n in emulator.state:
                emulator.run_clear(n)
            executor.spin_once(timeout_sec=.003)
            if duration is None and predicate():
                return
        if duration is None:
            raise AssertionError(label + ': ' + json.dumps(observed['safety']))

    def service(name):
        client = node.create_client(Trigger, '/chassis_node/' + name)
        pump(lambda: client.service_is_ready(), timeout=15, label=name + ' discovered')
        future = client.call_async(Trigger.Request())
        pump(future.done, label=name + ' returned')
        node.destroy_client(client)
        return future.result()

    def ops_request(action, params):
        request_id = ops.submit(action, params=params)
        pump(lambda: any(r.get('request_id') == request_id and str(r.get('status', '')).startswith('FINAL')
                         for r in replies), timeout=8, label=action + ' final')
        return next(r for r in replies if r.get('request_id') == request_id
                    and str(r.get('status', '')).startswith('FINAL'))

    def snap():
        return emulator.snapshot()

    def all_idle():
        s = snap()
        return all(v == 1 for v in s['states'].values()) and all(v == 0 for v in s['velocities'].values())

    def moving():
        return all(abs(v) > .1 for v in snap()['velocities'].values())

    def clears(since):
        return [c for c in snap()['commands'][since:] if c['command'] == CLEAR]

    result = None
    try:
        chassis = ['ros2', 'run', 'powertrain_ros', 'chassis', '--ros-args', '-p', 'fake:=false',
                   '-p', 'channel:=vcan77', '-p', 'authority_enabled:=true',
                   '-p', 'safety_startup_timeout:=5.0',
                   '-p', 'demo_mode:=%s' % ('true' if demo else 'false'),
                   '-p', 'arm_gate_mode:=arm_absent_field',
                   '-p', 'console_estop_latch_path:=' + str(evidence / 'estop.json'),
                   '-p', 'mission_id_path:=' + str(evidence / 'mission-id')]
        commands = [
            ['ros2', 'run', 'powertrain_ros', 'teleop_command', '--ros-args', '-p', 'host:=127.0.0.1',
             '-p', 'port:=39000', '-p', 'input_timeout_s:=0.50'],
            ['ros2', 'run', 'powertrain_ros', 'ops_broker', '--ros-args', '-p', 'host:=127.0.0.1',
             '-p', 'port:=39001', '-p', 'token_dir:=' + str(evidence)],
            chassis,
        ]
        if demo:
            commands.append([sys.executable, '-u', '-m', 'drive.bl70200.odrive_auto_clear',
                             '--channel', 'vcan77', '--mask', '0x840', '--motor-allow', '0x1000'])
        for index, command in enumerate(commands):
            log = (evidence / ('node-%s.log' % index)).open('w')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True))
        pump(lambda: observed['safety'].get('mode') == 'IDLE', timeout=20, label='installed chassis idle')
        assert observed['safety'].get('demo_mode') is demo, observed['safety']
        deadline = time.monotonic() + 5
        while raw is None:
            try:
                raw = socket.create_connection(('127.0.0.1', 39000), timeout=.2)
                raw.setblocking(False)
            except OSError:
                if time.monotonic() > deadline:
                    raise
                pump(duration=.05)
        ops = OpsChannelClient('127.0.0.1', 39001, token.read_text())
        pump(lambda: observed['gateway'].get('state') == 'DRIVE', label='TCP neutral accepted')
        assert service('authority_manual').success
        pump(duration=.25)
        assert service('arm').success
        sample = ClientInput(deadman=True, right_trigger=.4)
        pump(moving, label='TCP to installed ROS to six real CAN drivers')
        seq_before = observed['safety'].get('disarm_seq')
        axis_error = 0x100 if case == 'other_error_latches' else 0x40
        motor_error = 0x1002 if case == 'motor_error_outside_allow' else 0x1000
        start = emulator.fail_axis(15, axis_error, motor_error)

        if case in ('latch_without_demo', 'other_error_latches'):
            pump(lambda: observed['safety'].get('estop_latched') and all_idle(),
                 label='drive fault latches E-stop')
            pump(duration=1.0)
            assert not clears(start), 'auto clear touched a latched fault'
            sample = ClientInput()
            pump(duration=.3)
            reply = ops_request('demo_auto_arm', {'expect_disarm_seq': observed['safety'].get('disarm_seq')})
            assert reply['status'] == 'FINAL_REJECTED', reply
        else:
            pump(lambda: observed['safety'].get('disarm_cause') == 'demo_fault' and all_idle(),
                 label='demo fault stop disarms instead of latching')
            safety = observed['safety']
            assert safety['mode'] == 'IDLE' and not safety['estop_latched'], safety
            assert safety['disarm_seq'] == seq_before + 1, (seq_before, safety)
            assert 'node15' in safety['disarm_detail'], safety
            sample = ClientInput()
            pump(duration=1.0)
            node15_clears = [c for c in clears(start) if c['node'] == 15]
            other_clears = [c for c in clears(start) if c['node'] != 15]
            assert not other_clears, other_clears
            seq = observed['safety']['disarm_seq']
            if case == 'motor_error_outside_allow':
                assert not node15_clears, 'cleared a motor error outside the allow list'
                reply = ops_request('demo_auto_arm', {'expect_disarm_seq': seq})
                pump(duration=.5)
                assert observed['safety']['mode'] == 'IDLE' and all_idle(), observed['safety']
            elif case == 'stale_seq_rejected':
                reply = ops_request('demo_auto_arm', {'expect_disarm_seq': seq - 1})
                assert reply['status'] == 'FINAL_REJECTED', reply
                direct_arm.publish(String(data=json.dumps({'expect_disarm_seq': seq - 1})))
                pump(duration=.5)
                assert observed['safety']['mode'] == 'IDLE' and all_idle(), observed['safety']
            else:
                assert len(node15_clears) == 1, node15_clears
                assert snap()['errors'][15] == 0
                before_arm = len(snap()['commands'])
                reply = ops_request('demo_auto_arm', {'expect_disarm_seq': seq})
                assert reply['status'] == 'FINAL_SUCCESS', reply
                pump(lambda: observed['safety'].get('mode') == 'ARMED', label='compare-and-arm')
                assert not clears(before_arm), 'chassis sent CLEAR_ERRORS on the demo arm'
                pump(duration=.3)
                assert all(v == 0 for v in snap()['velocities'].values()), 'moved before new input'
                sample = ClientInput(deadman=True, right_trigger=.4)
                pump(moving, label='new TCP input drives after demo auto arm')
        result = {'result': 'PASS', 'case': case, 'installed_chassis_module': installed_path,
                  'safety': observed['safety'], 'gateway': observed['gateway'],
                  'ops_replies': replies, 'can': snap()}
    finally:
        if raw is not None:
            raw.close()
        if ops is not None:
            ops.close()
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        for log in logs:
            log.close()
        emulator.stop()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    for path in evidence.glob('node-*.log'):
        assert 'Traceback' not in path.read_text(), str(path)
    assert result is not None
    (evidence / 'result.json').write_text(json.dumps(result, indent=2, default=str))
    print('PASS installed demo mode:', case, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=('all',) + CASES, default='all')
    parser.add_argument('--evidence-dir', type=Path, required=True)
    args = parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID') == '77', 'domain77 only'
    assert not Path('/sys/class/net/can0').exists(), 'physical CAN interface present'
    assert Path('/sys/class/net/vcan77').exists(), 'create isolated vcan77 first'
    links = json.loads(subprocess.check_output(['ip', '-details', '-json', 'link', 'show', 'vcan77']))
    assert links[0].get('linkinfo', {}).get('info_kind') == 'vcan', links
    if args.case == 'all':
        for case in CASES:
            subprocess.run([sys.executable, __file__, '--case', case,
                            '--evidence-dir', str(args.evidence_dir / case)], check=True, timeout=90)
    else:
        run_case(args.case, args.evidence_dir)


if __name__ == '__main__':
    main()
