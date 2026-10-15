"""Continuous dummy encoder rehearsal in the real mobility motor card.

Uses isolated receive ports and no ops token, device, session or gamepad.
Run with /usr/bin/python3 -m operator_console.rover_3d_demo.
"""
import math
import os
from pathlib import Path
import signal
import socket
import sys
import tempfile
import time
import json

from .app import OperatorConsole, Gst, Gtk, GLib, _add_unix_signal_watch


def dummy_payload(sequence, elapsed):
    # Hold the CAD reference pose, then perturb one measured joint at a time.
    # This is a joint-display rehearsal, not a ground-contact simulation.
    cycle = elapsed % 16.0
    angles = dict(ml_rocker_joint=0.0, mr_rocker_joint=0.0,
                  bl_bogie_joint=0.0, br_bogie_joint=0.0)
    names = tuple(angles)
    segment = int(cycle // 4)
    local = cycle % 4
    if 1.0 < local < 3.0:
        angles[names[segment]] = math.radians(8) * math.sin((local-1)*math.pi)
    return dict(
        schema_version=1, sequence=sequence, drive_state="TEST / IDLE",
        can_state="TEST", roll_rad=0.0, pitch_rad=0.0, wheel_count=6,
        wheel_statuses=[dict(name=name, mode="IDLE", drive_turns_per_s=0.0,
            command_turns_per_s=0.0, steer_deg=0.0,
            drive_current_a=0.0,
            steer_current_a=0.4, stale=False, drive_axis_error=0, steer_fault=0)
            for index, name in enumerate(("front_left", "front_right", "mid_left",
                                         "mid_right", "rear_left", "rear_right"))],
        suspension_joints_rad=angles,
        suspension_age_ms=0.0,
    )


def main():
    Gst.init(None)
    sockets = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(8)]
    for channel in sockets:
        channel.bind(("127.0.0.1", 0))
    ports = [channel.getsockname()[1] for channel in sockets]
    for channel in sockets:
        channel.close()
    with tempfile.TemporaryDirectory(prefix="rover-dummy-") as tmp:
        console = OperatorConsole(
            "127.0.0.1", latency_ms=60, l515_port=ports[0], d435_port=ports[1],
            metadata_port=ports[2], telemetry_port=ports[3],
            chassis_telemetry_port=ports[4], arm_telemetry_port=ports[5],
            environment_telemetry_port=ports[6],
            ops_token_file=str(Path(tmp) / "missing-token"), input_source="TEST",
        )
        console.set_title("파워트레인 운영 콘솔 · 더미 엔코더 시연")
        console.show_all()
        console.resize(1100, 680)
        console._stack.set_visible_child_name("mobility")
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        started = time.monotonic()
        sequence = 0
        closed = False

        def tick():
            nonlocal sequence
            if closed:
                return False
            sequence += 1
            sender.sendto(json.dumps(dummy_payload(sequence, time.monotonic()-started)).encode(),
                          ("127.0.0.1", ports[4]))
            return True

        def quit_window(*_args):
            nonlocal closed
            closed = True
            console.destroy()
            return False

        GLib.timeout_add(40, tick)
        console.connect("destroy", lambda *_args: sender.close())
        for stop_signal in (signal.SIGINT, signal.SIGTERM):
            _add_unix_signal_watch(stop_signal, quit_window)
        Gtk.main()
        closed = True
    return 0


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
