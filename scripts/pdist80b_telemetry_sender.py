#!/usr/bin/env python3
"""Forward PDIST80B read-only measurements to the operator UDP console."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2/src/powertrain_ros"))

import serial  # noqa: E402

from powertrain_ros.pdist80b import bms_monitor_request, parse_bms_monitor_response  # noqa: E402


def read_status(port: serial.Serial, device_id: int):
    port.reset_input_buffer()
    port.write(bms_monitor_request(device_id))
    port.flush()
    return parse_bms_monitor_response(port.read(18), device_id)


def payload_for(status, *, sequence: int, rs485_state: str,
                consecutive_failures: int, detail: str) -> dict[str, object]:
    """Build a visible read-only snapshot even if the RS485 request failed."""
    if status is None:
        voltage_v = current_a = power_w = None
        soc_percent = battery_flags = protection_flags = charge_current_a = None
        drive_state = "PDIST unavailable"
    else:
        voltage_v = status.voltage_v
        current_a = status.discharge_current_a
        power_w = status.voltage_v * current_a
        soc_percent = status.soc_percent
        battery_flags = status.battery_flags
        protection_flags = status.protection_flags
        charge_current_a = status.charge_current_a
        drive_state = f"PDIST SOC {status.soc_percent}%"
    return {
        "schema_version": 1,
        "sequence": sequence,
        "odometry_source": "unavailable",
        "x_m": None, "y_m": None, "yaw_rad": None,
        "voltage_v": voltage_v, "current_a": current_a, "power_w": power_w,
        "drive_state": drive_state,
        "can_state": "unavailable",
        "pdist_soc_percent": soc_percent,
        "pdist_battery_flags": battery_flags,
        "pdist_protection_flags": protection_flags,
        "pdist_charge_current_a": charge_current_a,
        "rs485_state": rs485_state,
        "rs485_consecutive_failures": consecutive_failures,
        "rs485_detail": detail,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--operator-host", required=True)
    parser.add_argument("--operator-port", type=int, default=5004)
    parser.add_argument("--device-id", type=int, default=1)
    parser.add_argument("--hz", type=float, default=2.0)
    args = parser.parse_args()
    if not 0.2 <= args.hz <= 5.0:
        raise ValueError("hz must be within 0.2..5.0")
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sequence = 0
    port: serial.Serial | None = None
    consecutive_failures = 0
    last_error = ""
    while True:
        started = time.monotonic()
        status = None
        try:
            if port is None:
                port = serial.Serial(args.port, 57600, bytesize=8, parity="N", stopbits=1,
                                     timeout=0.25, write_timeout=0.25)
            status = read_status(port, args.device_id)
            consecutive_failures = 0
            last_error = ""
            frame = payload_for(status, sequence=sequence, rs485_state="LIVE",
                                consecutive_failures=0, detail="")
        except (OSError, ValueError) as exc:
            consecutive_failures += 1
            if str(exc) != last_error:
                print(f"PDIST80B read failed: {exc}", file=sys.stderr, flush=True)
                last_error = str(exc)
            if port is not None:
                port.close()
                port = None
            frame = payload_for(None, sequence=sequence, rs485_state="ERROR",
                                consecutive_failures=consecutive_failures, detail=last_error)
        try:
            udp.sendto(json.dumps(frame, separators=(",", ":")).encode(),
                       (args.operator_host, args.operator_port))
            sequence += 1
        except OSError as exc:
            print(f"PDIST80B telemetry send failed: {exc}", file=sys.stderr, flush=True)
        time.sleep(max(0.0, 1.0 / args.hz - (time.monotonic() - started)))


if __name__ == "__main__":
    main()
