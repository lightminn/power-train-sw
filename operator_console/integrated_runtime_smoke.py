"""Real integrated GTK entrypoint/no-robot lifecycle gate; no hardware required."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time


def run_smoke():
    with tempfile.TemporaryDirectory(prefix='integrated-gtk-') as directory:
        base = Path(directory)
        token = base / 'token'
        token.write_text('integrated-smoke-token')
        port_socket = socket.socket()
        port_socket.bind(('127.0.0.1', 0))
        # Keep port bound but not listening to guarantee no robot connection.
        session_port = port_socket.getsockname()[1]
        config = base / 'operator.json'
        from operator_console.runtime_smoke import _free_udp_port
        config.write_text(json.dumps(dict(robot_id='smoke-absent-robot', hosts=['127.0.0.1'],
            session_port=session_port, token_file=str(token), controller_python=sys.executable,
            metadata_port=_free_udp_port(), telemetry_port=_free_udp_port(),
            chassis_telemetry_port=_free_udp_port(), arm_telemetry_port=_free_udp_port())))
        probe = base / 'probe.json'
        env = dict(os.environ, XDG_RUNTIME_DIR=str(base), GDK_BACKEND='x11')
        command = ['xvfb-run', '-a', sys.executable, '-m', 'operator_console',
                   '--config', str(config), '--smoke-probe-file', str(probe)]
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   start_new_session=True, env=env)
        child_pids = []
        try:
            deadline = time.monotonic() + 15
            record = None
            while time.monotonic() < deadline and process.poll() is None:
                try:
                    record = json.loads(probe.read_text())
                    if 'operation' in record:
                        break
                except (OSError, ValueError):
                    pass
                time.sleep(.1)
            assert record and 'operation' in record, 'integrated window never reached GTK refresh'
            state = record['operation']
            assert not state['connected'] and not state['ready'] and not state['holding']
            assert not state['pad'].get('pad_connected'), 'no-pad fixture falsely ready'
            console_pid = record['console_pid']
            child_query = subprocess.run(['pgrep', '-P', str(console_pid)], capture_output=True, text=True)
            child_pids = [int(pid) for pid in child_query.stdout.split()]
            assert child_pids, 'controller supervisor never launched its child'
            duplicate = subprocess.run(command, capture_output=True, env=env, timeout=8)
            assert duplicate.returncode != 0 and '이미 실행 중'.encode() in duplicate.stderr
            os.kill(console_pid, signal.SIGINT)
            _, stderr = process.communicate(timeout=15)
            assert process.returncode == 0, stderr.decode(errors='replace')
            assert b'Traceback' not in stderr, stderr.decode(errors='replace')
            assert all(not Path(f'/proc/{pid}').exists() for pid in child_pids), 'controller child leaked'
            from operator_console.operation_runtime import ConsoleInstanceLock
            with ConsoleInstanceLock(base / 'powertrain' / 'operator-console.lock'):
                pass
            return True, 'GTK waiting with no robot/pad; duplicate rejected; SIGINT cleaned child and lock'
        except (AssertionError, subprocess.TimeoutExpired) as exc:
            return False, str(exc)
        finally:
            port_socket.close()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
            process.communicate(timeout=5)


def main():
    passed, report = run_smoke()
    print(f'INTEGRATED-CONSOLE-SMOKE: {"PASS" if passed else "FAIL"}\n{report}')
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
