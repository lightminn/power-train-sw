"""Installed ROS/TCP/SocketCAN acceptance. Run only in isolated domain77/vcan77.

Source the rebuilt ROS install first, then run this file with --case all.
No physical interface is ever opened. Each case starts the three installed
entrypoints and uses their real services, TCP parser and CAN drivers.
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
import threading
import time
import uuid


CASES = ('resume', 'cached_input', 'disarm', 'disconnect', 'reconnect', 'timeout', 'estop')


class Emulator:
    def __init__(self):
        import can
        self.can = can
        self.bus = can.interface.Bus(channel='vcan77', interface='socketcan')
        self.lock = threading.Lock()
        self.done = threading.Event()
        self.error = {n: 0 for n in range(11, 17)}
        self.state = {n: 1 for n in self.error}
        self.flags = {n: 0 for n in self.error}
        self.generation = {n: 0 for n in self.error}
        self.velocity = {n: 0. for n in self.error}
        self.position = {n: 0. for n in self.error}
        self.steering = {n: 0. for n in range(1, 5)}
        self.commands = []
        self.failure = None
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def send(self, node, command, data):
        self.bus.send(self.can.Message(arbitration_id=(node << 5) | command,
                      data=data, is_extended_id=False), timeout=.05)

    def heartbeat(self, node):
        self.send(node, 1, struct.pack('<IBBBB', self.error[node], self.state[node],
                  0xA3, self.flags[node], self.generation[node]))

    def run(self):
        previous = next_status = time.monotonic()
        try:
            while not self.done.is_set():
                now = time.monotonic()
                with self.lock:
                    for n in self.state:
                        self.position[n] += self.velocity[n] * (now - previous)
                    previous = now
                    if now >= next_status:
                        for n in self.state:
                            self.heartbeat(n)
                        for n in self.steering:
                            self.bus.send(self.can.Message(arbitration_id=(41 << 8) | n,
                                is_extended_id=True, data=struct.pack('>hhhbb',
                                int(self.steering[n] * 10), 0, 0, 25, 0)), timeout=.05)
                        next_status = now + .02
                message = self.bus.recv(.002)
                if message is None or message.is_error_frame:
                    continue
                with self.lock:
                    if message.is_extended_id:
                        if message.arbitration_id >> 8 == 6:
                            n = message.arbitration_id & 255
                            if n in self.steering:
                                self.steering[n] = struct.unpack('>i', message.data[:4])[0] / 10000.
                        continue
                    n, command = message.arbitration_id >> 5, message.arbitration_id & 31
                    if n not in self.state:
                        continue
                    if message.is_remote_frame:
                        if command == 9:
                            self.send(n, command, struct.pack('<ff', self.position[n], self.velocity[n]))
                        elif command == 20:
                            self.send(n, command, struct.pack('<ff', 0., 0.))
                        continue
                    value = None
                    if command == 24:
                        if bytes(message.data) == bytes(8) or (
                                bytes(message.data) == bytes([0xA3,self.generation[n],0,0,0,0,0,0])
                                and self.error[n] == 0x4000 and self.state[n] == 1 and self.flags[n] == 5):
                            self.error[n] = self.flags[n] = 0
                    elif command == 7:
                        value = struct.unpack('<I', message.data[:4])[0]
                        guarded = len(message.data) == 8 and message.data[4] == 0xA3
                        if guarded:
                            accepted = (value == 8 and self.error[n] == 0x4000 and self.state[n] == 1
                                        and self.flags[n] == 5 and message.data[5] == self.generation[n]
                                        and bytes(message.data[6:]) == bytes(2))
                            if accepted:
                                self.error[n] = self.flags[n] = 0
                                self.velocity[n] = 0.
                                self.state[n] = 8
                        elif value != 8 or self.error[n] == 0:
                            self.state[n] = value
                        if self.state[n] != 8:
                            self.velocity[n] = 0.
                        self.heartbeat(n)
                    elif command == 13:
                        value = struct.unpack('<ff', message.data)[0]
                        self.velocity[n] = value if self.state[n] == 8 and self.error[n] == 0 else 0.
                    self.commands.append({'stamp_s': now, 'node': n, 'command': command, 'value': value, 'payload': list(message.data)})
        except BaseException as exc:
            self.failure = repr(exc)

    def inject(self):
        with self.lock:
            self.error[13], self.state[13], self.flags[13] = 0x4000, 1, 7
            self.velocity[13] = 0.
            self.generation[13] = (self.generation[13] + 1) & 255
            self.heartbeat(13)
            return len(self.commands)

    def settle(self):
        with self.lock:
            self.flags[13] = 5
            self.heartbeat(13)

    def snapshot(self):
        with self.lock:
            return {'states': dict(self.state), 'velocities': dict(self.velocity),
                    'errors': dict(self.error), 'commands': list(self.commands)}

    def stop(self):
        self.done.set()
        self.thread.join(2)
        self.bus.shutdown()


def run_case(case, evidence):
    import rclpy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from powertrain_msgs.msg import SafetyVerdict, ManualDriveCommand, WheelStates
    from robot_arm_msgs.msg import ArmStatus
    from powertrain_ros import contract
    from powertrain_ros.command_receipt import ReceiptTimeExecutor
    import powertrain_ros.chassis_node as chassis_module
    from motor_control.laptop.remote_operation_client import ClientInput, encode_frame

    installed_path = str(Path(chassis_module.__file__).resolve())
    assert '/install/' in installed_path, installed_path
    assert hasattr(ManualDriveCommand(), 'source_received_s'), 'rebuild powertrain_msgs'
    evidence.mkdir(parents=True, exist_ok=True)
    token = evidence / 'ops_console.token'
    token.write_text('isolated-auto-resume-token')
    token.chmod(0o600)
    emulator = Emulator()
    rclpy.init()
    node = rclpy.create_node('can_auto_resume_acceptance_' + uuid.uuid4().hex[:8])
    executor = ReceiptTimeExecutor()
    executor.add_node(node)
    safety_publisher = node.create_publisher(SafetyVerdict, '/safety_verdict', 10)
    arm_publisher = node.create_publisher(ArmStatus, contract.TOPIC_ARM_STATUS, 10)
    observed = {'safety': {}, 'wheels': None, 'gateway': {}, 'manual': []}
    node.create_subscription(String, '/chassis/safety_state',
        lambda msg: observed.__setitem__('safety', json.loads(msg.data)), 10)
    node.create_subscription(String, '/teleop/gateway_state',
        lambda msg: observed.__setitem__('gateway', json.loads(msg.data)), 10)
    node.create_subscription(WheelStates, '/wheel_states',
        lambda msg: observed.__setitem__('wheels', msg), 10)
    node.create_subscription(ManualDriveCommand, '/teleop/drive_command',
        lambda msg: observed['manual'].append((time.monotonic(), msg.speed_mps, msg.source_received_s)), 10)
    processes, logs = [], []
    raw = None
    sequence = 0
    session_id = str(uuid.uuid4())
    sample = ClientInput()
    send_input = True
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
                verdict.detail = 'isolated vcan acceptance sensor'
                safety_publisher.publish(verdict)
                arm = ArmStatus()
                arm.status = contract.ARM_STOWED_LOCKED
                stamp = time.monotonic_ns()
                arm.header.stamp.sec, arm.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
                arm_publisher.publish(arm)
                last_sensors = now
            if raw is not None:
                if send_input and now - last_send >= .015:
                    raw.sendall(encode_frame(sample, session_id=session_id,
                        sequence=sequence, client_monotonic_ns=time.monotonic_ns()))
                    sequence += 1
                    last_send = now
                try:
                    raw.recv(65536)
                except BlockingIOError:
                    pass
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
        result = future.result()
        assert result.success, (name, result.message)
        node.destroy_client(client)

    def all_idle():
        snap = emulator.snapshot()
        return all(s == 1 for s in snap['states'].values()) and all(v == 0 for v in snap['velocities'].values())

    def all_armed():
        return all(s == 8 for s in emulator.snapshot()['states'].values())

    def moving():
        return all(abs(v) > .1 for v in emulator.snapshot()['velocities'].values())

    result = None
    try:
        commands = [
            ['ros2','run','powertrain_ros','teleop_command','--ros-args','-p','host:=127.0.0.1',
             '-p','port:=39000','-p','input_timeout_s:=0.30'],
            ['ros2','run','powertrain_ros','ops_broker','--ros-args','-p','host:=127.0.0.1',
             '-p','port:=39001','-p','token_dir:=' + str(evidence)],
            ['ros2','run','powertrain_ros','chassis','--ros-args','-p','fake:=false','-p','channel:=vcan77',
             '-p','authority_enabled:=true','-p','safety_startup_timeout:=5.0',
             '-p','console_estop_latch_path:=' + str(evidence / 'estop.json'),
             '-p','mission_id_path:=' + str(evidence / 'mission-id')],
        ]
        for index, command in enumerate(commands):
            log = (evidence / ('node-%s.log' % index)).open('w')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        pump(lambda: observed['safety'].get('mode') == 'IDLE', timeout=20, label='installed chassis idle')
        deadline = time.monotonic() + 5
        while raw is None:
            try:
                raw = socket.create_connection(('127.0.0.1', 39000), timeout=.2)
                raw.setblocking(False)
            except OSError:
                if time.monotonic() > deadline:
                    raise
                pump(duration=.05)
        pump(lambda: observed['gateway'].get('state') == 'DRIVE', label='TCP neutral accepted')
        service('authority_manual')
        pump(duration=.25)
        service('arm')
        sample = ClientInput(deadman=True, right_trigger=.4)
        pump(moving, label='TCP to installed ROS to all six real CAN drivers')
        start_index = emulator.inject()
        pump(lambda: all_idle() and observed['safety'].get('can_recovery', {}).get('state') == 'WAIT_STOP',
             label='identified CAN recovery paused all six axes')
        held = emulator.snapshot()['commands'][start_index:]
        assert not any(c['command'] == 24 for c in held), 'clear before stopped dwell'
        if case in ('resume', 'cached_input'):
            emulator.settle()
            # Keep input alive through most of the stopped dwell, then stop
            # TCP frames while the gateway still republishes the same values.
            pump(duration=.14)
            send_input = False
            manual_before = len(observed['manual'])
            pump(all_armed, timeout=.16, label='bounded fresh state8 confirmations')
            pump(duration=.045)
            assert all(v == 0 for v in emulator.snapshot()['velocities'].values()), 'cached TCP input resumed motion'
            repeated = observed['manual'][manual_before:]
            assert len(repeated) >= 2, 'did not observe repeated DDS publication'
            assert len({entry[2] for entry in repeated}) == 1, repeated
            rearmed_commands = emulator.snapshot()['commands'][start_index:]
            assert sum(c['command'] == 24 for c in rearmed_commands) == 0
            assert all(c['value'] == 0 for c in rearmed_commands if c['command'] == 13), 'old motion replayed during rearm'
            if case == 'resume':
                send_input = True
                pump(moving, label='new actual TCP receipt resumes motion')
                pump(lambda: observed['safety'].get('can_recovery', {}).get('count') == 1, label='observable successful recovery')
            else:
                pump(lambda: observed['safety'].get('mode') == 'IDLE' and all_idle(), label='cached-only input expires and cancels')
                send_input = True
                pump(duration=.4)
                assert all_idle(), 'new frames after cancellation rearmed automatically'
        else:
            if case == 'disarm':
                service('disarm')
            elif case == 'estop':
                service('estop')
            elif case in ('disconnect', 'reconnect'):
                raw.close()
                raw = None
                if case == 'reconnect':
                    # Reuse the client session ID intentionally; only the
                    # server's new accept UUID proves the ownership change.
                    raw = socket.create_connection(('127.0.0.1',39000), timeout=.5)
                    raw.setblocking(False)
                    sequence = 0
                    sample = ClientInput()
            else:
                send_input = False
            pump(lambda: observed['safety'].get('mode') in ('IDLE', 'ESTOP') and all_idle(),
                 label=case + ' cancels recovery')
            emulator.settle()
            if case == 'reconnect':
                sample = ClientInput(deadman=True,right_trigger=.4)
            pump(duration=.5)
            assert all_idle()
            assert not any(c['command'] == 24 for c in emulator.snapshot()['commands'][start_index:])
            assert observed['safety']['can_recovery']['count'] == 0
        result = {'result':'PASS','case':case,'installed_chassis_module':installed_path,
                  'safety':observed['safety'], 'gateway':observed['gateway'],
                  'manual_publications':observed['manual'], 'can':emulator.snapshot()}
    finally:
        if raw is not None:
            raw.close()
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
    (evidence / 'result.json').write_text(json.dumps(result, indent=2))
    print('PASS installed CAN auto resume:', case, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=('all',) + CASES, default='all')
    parser.add_argument('--evidence-dir', type=Path, required=True)
    args = parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID') == '77', 'domain77 only'
    assert not Path('/sys/class/net/can0').exists(), 'physical CAN interface present'
    assert Path('/sys/class/net/vcan77').exists(), 'create isolated vcan77 first'
    assert (Path('/sys/class/net/vcan77/type').read_text().strip() == '280'), 'CAN interface required'
    # Linux reports vcan through link kind; do not trust a renamed physical CAN.
    links = json.loads(subprocess.check_output(['ip','-details','-json','link','show','vcan77']))
    assert links[0].get('linkinfo', {}).get('info_kind') == 'vcan', links
    if args.case == 'all':
        for case in CASES:
            subprocess.run([sys.executable, __file__, '--case', case,
                '--evidence-dir', str(args.evidence_dir / case)], check=True, timeout=70)
    else:
        run_case(args.case, args.evidence_dir)


if __name__ == '__main__':
    main()
