"""Configured integrated console; legacy manual entrypoint is operator_console.app."""
import argparse
import os
from pathlib import Path
import signal
import sys

from powertrain_runtime.config import load_config
from operator_console.operation_runtime import ConsoleInstanceLock, OperationRuntime


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='~/.config/powertrain/operator.json')
    parser.add_argument('--smoke-probe-file', default=None)
    args = parser.parse_args(argv)
    runtime = None
    try:
        config = load_config(args.config, role='operator')
        if not Path(config['token_file']).read_text().strip():
            raise ValueError('token_file이 비어 있습니다')
        defaults = dict(d435_port=5002, l515_port=5000, metadata_port=5003,
                        telemetry_port=5004, chassis_telemetry_port=5005,
                        arm_telemetry_port=5007, environment_telemetry_port=5008)
        ports = {key: config.get(key, value) for key, value in defaults.items()}
        for key, value in ports.items():
            if type(value) is not int or not 1 <= value <= 65535:
                raise ValueError(f'{key}: 1–65535 포트가 필요합니다')
        latency = config.get('latency_ms', 60)
        if type(latency) is not int or not 0 <= latency <= 10000:
            raise ValueError('latency_ms: 0–10000 정수가 필요합니다')
        lock_root = Path(os.environ.get('XDG_RUNTIME_DIR', str(Path.home() / '.cache')))
        with ConsoleInstanceLock(lock_root / 'powertrain' / 'operator-console.lock'):
            from operator_console.app import OperatorConsole, Gst, Gtk, GLib, _add_unix_signal_watch
            Gst.init(None)
            if not Gtk.init_check()[0]:
                raise RuntimeError('화면에 연결할 수 없습니다. 데스크톱 세션에서 실행하세요')
            runtime = OperationRuntime(config)
            console = OperatorConsole('127.0.0.1', latency_ms=latency, **ports,
                ops_token_file=config['token_file'], operation_runtime=runtime,
                smoke_probe_file=args.smoke_probe_file)
            console.show_all()

            def quit_window(*_args):
                console.destroy()
                return GLib.SOURCE_REMOVE

            for stop_signal in (signal.SIGINT, signal.SIGTERM):
                _add_unix_signal_watch(stop_signal, quit_window)
            try:
                Gtk.main()
            finally:
                # Joining session/gamepad threads belongs outside GTK callbacks.
                runtime.close()
                runtime = None
        return 0
    except (OSError, ValueError, RuntimeError, ImportError) as exc:
        print(f'운용 콘솔 시작 실패: {exc}\n설정 확인: {Path(args.config).expanduser()} '
              '(robot_id, hosts, token_file, controller_python)', file=sys.stderr)
        return 2
    finally:
        if runtime is not None:
            runtime.close()


if __name__ == '__main__':
    result = main()
    # As in the legacy app, avoid the installed libsrt global destructor race
    # only after GTK channels, session, controller child and lock are closed.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(result)
