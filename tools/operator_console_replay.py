#!/usr/bin/env python3
"""Replay captured operator-console UDP payloads on isolated test ports.

JSONL rows:
  {"t": 0.0, "channel": "metadata", "payload": {...}}

The payload object is transmitted unchanged, so the production decoder and
adapter path are exercised. This tool never opens devices and has no live-port
defaults.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket
import time


DEFAULT_TEST_PORTS = {
    "metadata": 15003,
    "power": 15004,
    "chassis": 15005,
    "arm": 15007,
}


def load_rows(path: Path) -> list[tuple[float, str, dict]]:
    rows: list[tuple[float, str, dict]] = []
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not raw.strip() or raw.lstrip().startswith("#"):
            continue
        item = json.loads(raw)
        channel = item.get("channel")
        payload = item.get("payload")
        timestamp = item.get("t")
        if channel not in DEFAULT_TEST_PORTS:
            raise ValueError(f"line {line_number}: unsupported channel {channel!r}")
        if not isinstance(payload, dict):
            raise ValueError(f"line {line_number}: payload must be an object")
        if not isinstance(timestamp, (int, float)) or timestamp < 0:
            raise ValueError(f"line {line_number}: t must be non-negative")
        rows.append((float(timestamp), channel, payload))
    if not rows:
        raise ValueError("capture contains no replay rows")
    if any(rows[index][0] > rows[index + 1][0] for index in range(len(rows) - 1)):
        raise ValueError("capture timestamps must be monotonic")
    return rows


def replay(
    rows: list[tuple[float, str, dict]], *, host: str, speed: float,
    ports: dict[str, int],
) -> None:
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    started = time.monotonic()
    try:
        for timestamp, channel, payload in rows:
            deadline = started + timestamp / speed
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(remaining, 0.05))
            sender.sendto(
                json.dumps(payload, separators=(",", ":")).encode("utf-8"),
                (host, ports[channel]),
            )
    finally:
        sender.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--speed", type=float, default=1.0)
    for channel, port in DEFAULT_TEST_PORTS.items():
        parser.add_argument(f"--{channel}-port", type=int, default=port)
    args = parser.parse_args()
    if args.speed <= 0:
        parser.error("--speed must be greater than zero")
    ports = {
        channel: getattr(args, f"{channel}_port")
        for channel in DEFAULT_TEST_PORTS
    }
    if any(port in {5003, 5004, 5005, 5007} for port in ports.values()):
        parser.error("live UDP ports are blocked; use isolated replay ports")
    replay(load_rows(args.capture), host=args.host, speed=args.speed, ports=ports)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
