#!/usr/bin/env python3
"""Forward Raspberry Pi JSON-lines sensor output to the local GUI over UDP.

Use this when the Pi and laptop can reach each other over SSH but an
inter-subnet UDP route is blocked.  SSH carries the live stream; this process
only repackages the already-processed values for the GUI's v1 contract.
"""
from __future__ import annotations

import argparse
import json
import socket
import sys


def packet_from_sensor(data: dict, sequence: int) -> dict:
    bme = data.get("bme280", {})
    sgp = data.get("sgp30", {})
    mq7 = data.get("mq7", {})
    mq2 = data.get("mq2", {})
    flame = data.get("flame", {})
    return {
        "schema_version": 1,
        "sequence": sequence,
        "source": "rpi-environment-over-ssh",
        "source_timestamp": data.get("timestamp"),
        "sensor_ok": data.get("sensor_ok") is True,
        "errors": [str(error)[:160] for error in data.get("errors", [])[:8]],
        "temperature_c": bme.get("temperature_c"),
        "humidity_pct": bme.get("humidity_pct"),
        "pressure_hpa": bme.get("pressure_hpa"),
        "eco2_ppm": sgp.get("eco2_ppm"),
        "tvoc_ppb": sgp.get("tvoc_ppb"),
        "sgp30_warming_up": sgp.get("warming_up") is True,
        "co_estimated_ppm": mq7.get("co_estimated_ppm"),
        "co_rs_ro": mq7.get("rs_ro"),
        "co_range": mq7.get("range", "unknown"),
        "co_quality": mq7.get("quality", "unknown"),
        "lpg_estimated_ppm": mq2.get("lpg_estimated_ppm"),
        "lpg_rs_ro": mq2.get("rs_ro"),
        "lpg_range": mq2.get("range", "unknown"),
        "lpg_quality": mq2.get("quality", "unknown"),
        "flame_detected": flame.get("detected"),
        "flame_voltage_v": flame.get("voltage_v"),
        "flame_threshold_v": flame.get("threshold_v"),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=5008)
    args = parser.parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sequence = 0
    print(
        f"SSH sensor bridge → {args.udp_host}:{args.udp_port}",
        file=sys.stderr,
        flush=True,
    )
    try:
        for line in sys.stdin:
            try:
                data = json.loads(line)
                if not isinstance(data, dict):
                    continue
                sequence += 1
                encoded = json.dumps(
                    packet_from_sensor(data, sequence),
                    ensure_ascii=False,
                    allow_nan=False,
                    separators=(",", ":"),
                ).encode("utf-8")
                if len(encoded) > 4096:
                    raise ValueError("oversize packet")
                sock.sendto(encoded, (args.udp_host, args.udp_port))
                print(
                    "forwarded "
                    f"seq={sequence} "
                    f"temp={data.get('bme280', {}).get('temperature_c')} "
                    f"rh={data.get('bme280', {}).get('humidity_pct')} "
                    f"co={data.get('mq7', {}).get('co_estimated_ppm')} "
                    f"lpg={data.get('mq2', {}).get('lpg_estimated_ppm')}",
                    file=sys.stderr,
                    flush=True,
                )
            except (json.JSONDecodeError, TypeError, ValueError, OSError) as exc:
                print(f"bridge packet skipped: {exc}", file=sys.stderr)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
