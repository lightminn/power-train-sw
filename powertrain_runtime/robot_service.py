"""Prepared Jetson session-service entrypoint; never opens hardware directly."""
import argparse
from pathlib import Path
import signal
import threading
import time

from .config import load_config
from .drive_service import ManualDriveService
from .session import SessionServer


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="/etc/powertrain/robot.json")
    args = parser.parse_args(argv)
    config = load_config(args.config, role="robot")
    from motor_control.laptop.ops_channel_client import OpsChannelClient
    token = Path(config.get("ops_token_file", config["token_file"])).expanduser().read_text().strip()
    drive = ManualDriveService(lambda: OpsChannelClient(
        "127.0.0.1", config.get("ops_target_port", 19001), token))
    stopped = threading.Event()
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda *_: stopped.set())
    server = SessionServer(config, drive.start, drive.stop, drive.snapshot)
    try:
        server.start()
        print("Integrated robot service listening; waiting for paired console", flush=True)
        while not stopped.wait(.5):
            server.check_health()
    finally:
        server.close()
        drive.stop()
        deadline = time.monotonic() + 6
        while drive.snapshot()["status"] == "PENDING" and time.monotonic() < deadline:
            time.sleep(.02)
        print("Stop outcome: " + drive.snapshot()["status"], flush=True)
        drive.close()


if __name__ == "__main__":
    main()
