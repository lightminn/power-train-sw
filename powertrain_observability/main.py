"""Process entry point for the supervised observability daemon."""
from __future__ import annotations

import argparse
import signal
import sys
import threading

from .protocol import EVENT_SOCKET, LOCK_PATH, RUN_DIRECTORY, STATUS_SOCKET
from .server import DaemonAlreadyRunning, ObservabilityServer


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Powertrain observability daemon")
    parser.add_argument("--event-socket", default=EVENT_SOCKET)
    parser.add_argument("--status-socket", default=STATUS_SOCKET)
    parser.add_argument("--lock-path", default=LOCK_PATH)
    parser.add_argument("--run-directory", default=RUN_DIRECTORY)
    parser.add_argument("--queue-capacity", type=int, default=256)
    return parser


def run(argv=None) -> int:
    args = build_parser().parse_args(argv)
    stopped = threading.Event()

    def request_stop(_signum, _frame):
        stopped.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGHUP, lambda _signum, _frame: None)
    server = ObservabilityServer(
        event_socket=args.event_socket,
        status_socket=args.status_socket,
        lock_path=args.lock_path,
        run_directory=args.run_directory,
        queue_capacity=args.queue_capacity,
    )
    try:
        server.start()
    except DaemonAlreadyRunning as exc:
        print(str(exc), file=sys.stderr)
        return 1
    try:
        stopped.wait()
    finally:
        server.stop()
    return 0


def main() -> None:
    raise SystemExit(run())


if __name__ == "__main__":
    main()
