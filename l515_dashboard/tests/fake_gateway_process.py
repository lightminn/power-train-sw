"""Signal-safe real-process fake for Dashboard ownership tests."""

import os
import signal
import socket
import subprocess
import sys

from l515_dashboard.protocol import decode_request, encode_message, response


def main():
    path, pid_path = sys.argv[1:3]
    stopping = False

    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    old = {
        sig: signal.signal(sig, request_stop)
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
    }
    child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(300)"])
    with open(pid_path, "w", encoding="ascii") as stream:
        stream.write(str(child.pid))
    sock = socket.socket(socket.AF_UNIX)
    try:
        sock.bind(path)
        sock.listen()
        sock.settimeout(0.1)
        while not stopping:
            try:
                conn, _ = sock.accept()
            except socket.timeout:
                continue
            with conn:
                req = decode_request(conn.makefile("rb").readline().rstrip(b"\n"), 65536)
                if req["type"] == "stop_gateway":
                    stopping = True
                    payload = {"accepted": True}
                else:
                    payload = {
                        "state": "RUNNING", "sdk": {}, "ros_publish_counts": {},
                        "srt": {"enabled": True}, "system": {}, "last_error": None,
                    }
                conn.sendall(encode_message(response(req["request_id"], payload), 65536))
    finally:
        sock.close()
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass
        if child.poll() is None:
            child.terminate()
        child.wait(timeout=2)
        for sig, handler in old.items():
            signal.signal(sig, handler)


if __name__ == "__main__":
    main()
