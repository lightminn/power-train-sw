#!/usr/bin/env python3
"""Installed ROS/TCP/UDP + headless Python API + real terminal PTY, no devices."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import pty
import signal
import socket
import struct
import subprocess
import sys
import termios
import time


def port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '77' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('requires isolated ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1')
    base = args.output_dir.resolve()
    base.mkdir(parents=True, exist_ok=True)
    (base / 'ops_console.token').write_text('terminal-fixture-token')
    (base / 'ops_console.token').chmod(0o600)
    processes, streams, checks = [], [], []
    client = node = None
    import rclpy
    from std_msgs.msg import String
    from operator_console.arm_terminal import ArmTerminalClient
    assert 'gi' not in sys.modules, 'headless client imported GTK'
    rclpy.init()
    node = rclpy.create_node('arm_terminal_verifier')
    observed = {}
    node.create_subscription(String, '/tool/status',
        lambda m: observed.update(json.loads(m.data)), 10)
    ops_port, arm_port = port(), port()

    def start(label, package, executable, **parameters):
        stream = (base / f'{label}.log').open('w')
        streams.append(stream)
        command = ['ros2', 'run', package, executable, '--ros-args']
        for name, value in parameters.items():
            command += ['-p', f'{name}:={value}']
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append(process)

    def wait(predicate, label, timeout=10):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            client.tick()
            rclpy.spin_once(node, timeout_sec=.01)
            assert all(p.poll() is None for p in processes), 'installed process exited'
            if predicate():
                checks.append(label)
                return
        raise AssertionError(f'{label}: {observed}; {client.last_ack}; {client.last_outcome}; '
            f'caps={client.state.capabilities}; lock={client.state.coupling_lock}; '
            f'runtime={client.snapshot.tool_runtime if client.snapshot else None}')

    try:
        start('bridge', 'dynamixel_control', 'moveit_dynamixel_bridge', mock_mode='true',
            control_scope='END_EFFECTOR_ONLY', tool_type='dual_motor_gripper',
            toolchanger_enabled='true', toolchanger_zero_tick=2048,
            toolchanger_zero_state='LOCKED')
        start('broker', 'powertrain_ros', 'ops_broker', host='127.0.0.1',
            port=ops_port, token_dir=str(base))
        start('mirror', 'powertrain_ros', 'arm_console_bridge', console_host='127.0.0.1',
            telemetry_port=arm_port, publish_hz=10.0)
        client = ArmTerminalClient('127.0.0.1', ops_port, base / 'ops_console.token',
                                   telemetry_port=arm_port)
        wait(lambda: client.state.link.state == 'LIVE'
             and 'control_mode' in client.session.context.capabilities,
             'installed ROS → UDP → headless state')
        assert not client._pending, 'construction emitted a command'
        client.command('manual')
        wait(lambda: client.session.context.manual_granted, 'authenticated MANUAL grant')
        wait(lambda: 'coupling_lock' in client.session.context.capabilities,
             'fresh coupling status capability')
        client.command('lock-enable')
        wait(lambda: client.state.coupling_lock.get('state') == 'LOCKED'
             and client.state.coupling_lock.get('torque') == 1, 'lock activation feedback')
        client.command('unlock')
        wait(lambda: client.state.coupling_lock.get('state') == 'UNLOCKED', 'ball unlock at 1792')
        assert client.state.coupling_lock['position'] == 1792
        client.command('lock')
        wait(lambda: client.state.coupling_lock.get('state') == 'LOCKED'
             and client.state.coupling_lock.get('torque') == 1, 'ball relock at 2048')
        assert client.state.coupling_lock['position'] == 2048
        client.command('select-tool dual')
        wait(lambda: 'tool_enable' in client.session.context.capabilities,
             'tool rescan after coupling preparation')
        client.command('tool-enable')
        wait(lambda: observed.get('motion_allowed')
             and 'gripper_command' in client.session.context.capabilities,
             'explicit tool activation')
        client.command('tool open left')
        wait(lambda: observed.get('actuators', [{}])[0].get('position') == -675,
             'left tool command reaches only ID3')
        client.command('tool open right')
        wait(lambda: [a['position'] for a in observed.get('actuators', [])] == [-675, 2559],
             'right tool prepared at matching endpoint')
        wait(lambda: client.state.diagnostics.sync_state == 'SYNCED',
             'matching endpoint synchronization feedback')
        client.command('tool close both')
        wait(lambda: [a['position'] for a in observed.get('actuators', [])] == [-2211, 1017],
             'dual synchronized command')
        start_time = time.monotonic()
        while time.monotonic() - start_time < .7:
            client.jog(6, 1)
            client.tick()
            rclpy.spin_once(node, timeout_sec=.02)
        wait(lambda: observed.get('fsm_state') == 'READY', 'terminal key-release timeout holds tool')
        assert all(a['torque_state'] == 'ON' for a in observed['actuators'])

        # Exercise the shipped curses entrypoint with an actual TTY, including
        # Ctrl-C handling and termios restoration, rather than a mocked screen.
        master, slave = pty.openpty()
        terminal_process = None
        try:
            fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack('HHHH', 35, 120, 0, 0))
            previous = termios.tcgetattr(slave)
            terminal_process = subprocess.Popen([
                sys.executable, '-m', 'operator_console.arm_terminal',
                '--host', '127.0.0.1', '--port', str(ops_port),
                '--token-file', str(base / 'ops_console.token'),
                '--telemetry-port', str(port())], stdin=slave, stdout=slave, stderr=slave,
                env=dict(os.environ, TERM='xterm-256color'), start_new_session=True)
            os.set_blocking(master, False)
            output = bytearray()
            end = time.monotonic() + 3
            while time.monotonic() < end:
                try:
                    output.extend(os.read(master, 65536))
                except BlockingIOError:
                    pass
                time.sleep(.02)
            os.write(master, b':ignored\x18')
            wait(lambda: client.state.coupling_lock.get('torque') == 0,
                 'Ctrl-X inside command prompt reaches lock STOP')
            terminal_process.send_signal(signal.SIGINT)
            end = time.monotonic() + 6
            while terminal_process.poll() is None and time.monotonic() < end:
                try:
                    output.extend(os.read(master, 65536))
                except BlockingIOError:
                    pass
                time.sleep(.02)
            assert terminal_process.poll() == 0, 'terminal failed to exit on SIGINT'
            assert termios.tcgetattr(slave) == previous, 'terminal settings not restored'
            assert '로봇팔 터미널'.encode() in output
            assert b'Traceback' not in output
            (base / 'terminal-output.bin').write_bytes(output)
            checks.append('real curses PTY, SIGINT stop and termios restore')
        finally:
            if terminal_process is not None and terminal_process.poll() is None:
                terminal_process.kill()
                terminal_process.wait()
            os.close(master)
            os.close(slave)
        # Remove observation while keeping ops alive: state must go stale and
        # refuse a lock move, while STOP is still publishable.
        os.killpg(processes[2].pid, signal.SIGINT)
        processes[2].wait(timeout=8)
        processes.pop()
        wait(lambda: client.state.link.state == 'STALE', 'telemetry-loss state becomes STALE')
        assert not client.command('lock')
        assert client.command('lock-stop')
        checks.append('stale motion refusal and ungated lock STOP')
        (base / 'result.json').write_text(json.dumps(dict(passed=True, checks=checks,
            physical_hardware=False, gtk_imported='gi' in sys.modules),
            ensure_ascii=False, indent=2))
        print(f'PASS: {len(checks)} headless installed-path checks')
    finally:
        if client is not None:
            client.close()
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        for stream in streams:
            stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
