#!/usr/bin/env python3
"""
BENCH ONLY: read a torque-free XL430 and stream its position over TCP.

This is the laptop half of a deliberately small, one-axis master/slave HIL
probe.  It never writes a DYNAMIXEL register.  The production robot arm must
use ArmCommandAuthority/MoveIt Servo rather than this direct bench transport.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import time
import uuid

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler


PROTOCOL_VERSION = 2.0
XL430_MODEL_NUMBER = 1060
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_PRESENT_POSITION = 132


def _read1(packet, port, dxl_id: int, address: int, label: str) -> int:
    value, result, error = packet.read1ByteTxRx(port, dxl_id, address)
    if result != COMM_SUCCESS or error != 0:
        raise RuntimeError(
            f"{label} read failed: result={packet.getTxRxResult(result)}, "
            f"device_error={error}"
        )
    return int(value)


def _read4(packet, port, dxl_id: int, address: int, label: str) -> int:
    value, result, error = packet.read4ByteTxRx(port, dxl_id, address)
    if result != COMM_SUCCESS or error != 0:
        raise RuntimeError(
            f"{label} read failed: result={packet.getTxRxResult(result)}, "
            f"device_error={error}"
        )
    return int(value)


def parse_args() -> argparse.Namespace:
    """Parse laptop-side command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", required=True, help="Jetson IPv4 address")
    parser.add_argument("--tcp-port", type=int, default=9201)
    parser.add_argument(
        "--device",
        default=(
            "/dev/serial/by-id/"
            "usb-FTDI_USB__-__Serial_Converter_FTAO4U2V-if00-port0"
        ),
    )
    parser.add_argument("--master-id", type=int, default=5)
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--rate-hz", type=float, default=30.0)
    return parser.parse_args()


def main() -> int:
    """Read the torque-free master and stream positions until interrupted."""
    args = parse_args()
    if not math.isfinite(args.rate_hz) or not 1.0 <= args.rate_hz <= 100.0:
        raise SystemExit("--rate-hz must be between 1 and 100")

    port = PortHandler(args.device)
    packet = PacketHandler(PROTOCOL_VERSION)
    if not port.openPort():
        raise SystemExit(f"failed to open master port: {args.device}")
    if not port.setBaudRate(args.baud):
        port.closePort()
        raise SystemExit(f"failed to set master baud: {args.baud}")

    sock: socket.socket | None = None
    try:
        model, result, error = packet.ping(port, args.master_id)
        if result != COMM_SUCCESS or error != 0:
            raise RuntimeError(
                "master ping failed: "
                f"{packet.getTxRxResult(result)}, device_error={error}"
            )
        if int(model) != XL430_MODEL_NUMBER:
            raise RuntimeError(
                f"master model {model} is not XL430 ({XL430_MODEL_NUMBER})"
            )

        torque = _read1(
            packet, port, args.master_id, ADDR_TORQUE_ENABLE, "torque"
        )
        if torque != 0:
            raise RuntimeError(
                "master torque is enabled; disable it before hand operation"
            )
        hardware_error = _read1(
            packet,
            port,
            args.master_id,
            ADDR_HARDWARE_ERROR_STATUS,
            "hardware error",
        )
        if hardware_error != 0:
            raise RuntimeError(
                f"master hardware error: 0x{hardware_error:02x}"
            )

        initial = _read4(
            packet,
            port,
            args.master_id,
            ADDR_PRESENT_POSITION,
            "present position",
        )
        print(
            f"master ready: id={args.master_id} position={initial} "
            f"server={args.server}:{args.tcp_port}"
        )

        sock = socket.create_connection(
            (args.server, args.tcp_port),
            timeout=3.0,
        )
        sock.settimeout(3.0)
        session = uuid.uuid4().hex
        period = 1.0 / args.rate_hz
        next_tick = time.monotonic()
        last_report = 0.0
        seq = 0

        while True:
            position = _read4(
                packet,
                port,
                args.master_id,
                ADDR_PRESENT_POSITION,
                "present position",
            )
            frame = {
                "version": 1,
                "session": session,
                "seq": seq,
                "position": position,
            }
            sock.sendall(
                (json.dumps(frame, separators=(",", ":")) + "\n").encode()
            )
            seq += 1

            now = time.monotonic()
            if now - last_report >= 0.5:
                print(
                    f"master={position:4d} delta={position - initial:+5d}",
                    flush=True,
                )
                last_report = now
            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        print("stopping master stream")
        return 0
    finally:
        if sock is not None:
            sock.close()
        port.closePort()


if __name__ == "__main__":
    raise SystemExit(main())
