#!/usr/bin/env python3
"""Forward PDIST80B read-only measurements to the operator UDP console."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import re
import socket
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2/src/powertrain_ros"))

from powertrain_ros.pdist80b import bms_monitor_request, parse_bms_monitor_response  # noqa: E402


BRINGUP_STATUS_INTERVAL_S = 5.0
DEFAULT_JOURNAL_LINES = 20
MAX_TELEMETRY_BYTES = 8192
BRINGUP_UNITS = (
    "powertrain-bringup-preflight.service",
    "powertrain-pdist80b-telemetry.service",
    "docker.service",
)
COMPOSE_CONTAINERS = (
    "powertrain_control",
    "powertrain_ros",
    "powertrain_observability",
    "powertrain_canwatchdog",
)

_SECRET_VALUE = re.compile(
    r"(?i)(\b(?:ops_[A-Za-z0-9_-]*\.token|"
    r"[A-Za-z0-9_]*password[A-Za-z0-9_]*|[A-Za-z0-9_]+_pass)"
    r"\b[\"']?\s*(?:=|:)\s*)"
    r"(?:\"[^\"\r\n]*\"|'[^'\r\n]*'|[^\s,;]+)"
)


def read_status(port, device_id: int):
    port.reset_input_buffer()
    port.write(bms_monitor_request(device_id))
    port.flush()
    return parse_bms_monitor_response(port.read(18), device_id)


def _redact_journal_line(line: str) -> str:
    return _SECRET_VALUE.sub(r"\1[REDACTED]", line)


def build_bringup_status(unit_probe, compose_probe, journal_probe) -> dict[str, object]:
    """Build optional bring-up fields without binding tests to host services."""
    try:
        unit_status = dict(unit_probe())
    except Exception:
        unit_status = None

    try:
        compose_status = dict(compose_probe())
    except Exception:
        compose_status = None

    try:
        raw_tail = journal_probe()
        lines = raw_tail.splitlines() if isinstance(raw_tail, str) else list(raw_tail)
        journal_tail = [_redact_journal_line(str(line)) for line in lines]
    except Exception:
        journal_tail = None

    return {
        "unit_status": unit_status,
        "compose_status": compose_status,
        "journal_tail": journal_tail,
    }


def probe_unit_status() -> dict[str, str]:
    status = {}
    for unit in BRINGUP_UNITS:
        completed = subprocess.run(
            ["systemctl", "is-active", unit],
            capture_output=True,
            check=False,
            text=True,
            timeout=2.0,
        )
        status[unit] = completed.stdout.strip() or "unknown"
    return status


def probe_compose_status() -> dict[str, str]:
    status = {}
    for container in COMPOSE_CONTAINERS:
        completed = subprocess.run(
            ["docker", "inspect", "--format", "{{json .State}}", container],
            capture_output=True,
            check=False,
            text=True,
            timeout=2.0,
        )
        if completed.returncode != 0:
            status[container] = "unavailable"
            continue
        state = json.loads(completed.stdout)
        health = state.get("Health") or {}
        status[container] = str(health.get("Status") or state.get("Status") or "unknown")
    return status


def probe_journal_tail(lines: int = DEFAULT_JOURNAL_LINES) -> list[str]:
    completed = subprocess.run(
        [
            "journalctl", "--no-pager", "--output=short-iso", f"--lines={lines}",
            "--unit=powertrain-bringup-preflight.service",
            "--unit=powertrain-pdist80b-telemetry.service",
            "--unit=docker.service",
        ],
        capture_output=True,
        check=True,
        text=True,
        timeout=2.0,
    )
    return completed.stdout.splitlines()[-lines:]


def payload_for(status, *, sequence: int, rs485_state: str,
                consecutive_failures: int, detail: str,
                bringup_status: dict[str, object] | None = None) -> dict[str, object]:
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
    payload = {
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
    if bringup_status is not None:
        payload.update(bringup_status)
    return payload


def encode_payload(payload: dict[str, object]) -> bytes:
    """Encode one console datagram, dropping oldest journal lines if needed."""
    bounded = dict(payload)

    def _encode() -> bytes:
        return json.dumps(
            bounded,
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")

    encoded = _encode()
    raw_tail = bounded.get("journal_tail")
    if len(encoded) <= MAX_TELEMETRY_BYTES or not isinstance(raw_tail, list):
        if len(encoded) > MAX_TELEMETRY_BYTES:
            raise ValueError("telemetry payload exceeds 8192 bytes")
        return encoded

    journal_tail = list(raw_tail)
    while journal_tail and len(encoded) > MAX_TELEMETRY_BYTES:
        journal_tail.pop(0)
        bounded["journal_tail"] = journal_tail
        encoded = _encode()
    if len(encoded) > MAX_TELEMETRY_BYTES:
        raise ValueError("telemetry payload exceeds 8192 bytes")
    return encoded


def main() -> None:
    import serial

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--operator-host", required=True)
    parser.add_argument("--operator-port", type=int, default=5004)
    parser.add_argument("--device-id", type=int, default=1)
    parser.add_argument("--hz", type=float, default=2.0)
    parser.add_argument("--include-unit-status", action="store_true")
    parser.add_argument("--journal-lines", type=int, default=DEFAULT_JOURNAL_LINES)
    args = parser.parse_args()
    if not 0.2 <= args.hz <= 5.0:
        raise ValueError("hz must be within 0.2..5.0")
    if not 1 <= args.journal_lines <= 100:
        raise ValueError("journal-lines must be within 1..100")
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sequence = 0
    port = None
    consecutive_failures = 0
    last_error = ""
    bringup_status = None
    next_bringup_probe_s = 0.0
    bringup_executor = ThreadPoolExecutor(max_workers=1) if args.include_unit_status else None
    bringup_future = None
    while True:
        started = time.monotonic()
        if bringup_future is not None and bringup_future.done():
            bringup_status = bringup_future.result()
            bringup_future = None
            next_bringup_probe_s = started + BRINGUP_STATUS_INTERVAL_S
        if bringup_executor is not None and bringup_future is None \
                and started >= next_bringup_probe_s:
            bringup_future = bringup_executor.submit(
                build_bringup_status,
                probe_unit_status,
                probe_compose_status,
                lambda: probe_journal_tail(args.journal_lines),
            )
        status = None
        try:
            if port is None:
                port = serial.Serial(args.port, 57600, bytesize=8, parity="N", stopbits=1,
                                     timeout=0.25, write_timeout=0.25)
            status = read_status(port, args.device_id)
            consecutive_failures = 0
            last_error = ""
            frame = payload_for(status, sequence=sequence, rs485_state="LIVE",
                                consecutive_failures=0, detail="",
                                bringup_status=bringup_status)
        except (OSError, ValueError) as exc:
            consecutive_failures += 1
            if str(exc) != last_error:
                print(f"PDIST80B read failed: {exc}", file=sys.stderr, flush=True)
                last_error = str(exc)
            if port is not None:
                port.close()
                port = None
            frame = payload_for(None, sequence=sequence, rs485_state="ERROR",
                                consecutive_failures=consecutive_failures, detail=last_error,
                                bringup_status=bringup_status)
        try:
            udp.sendto(encode_payload(frame), (args.operator_host, args.operator_port))
            sequence += 1
        except (OSError, ValueError) as exc:
            print(f"PDIST80B telemetry send failed: {exc}", file=sys.stderr, flush=True)
        time.sleep(max(0.0, 1.0 / args.hz - (time.monotonic() - started)))


if __name__ == "__main__":
    main()
