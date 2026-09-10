#!/usr/bin/env python3
"""
BENCH ONLY: drive one XL430 from a fresh TCP master-position stream.

The server seeds the current slave position before torque-on, clamps motion
around that startup position, and disables torque on stale input, disconnect,
malformed data, or process exit.  Mode-dependent gains and profile values are
restored during normal cleanup.

This direct DYNAMIXEL path is not the production robot-arm authority path.
"""

from __future__ import annotations

import argparse
import json
import socket
import time
from dataclasses import dataclass, replace

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler


PROTOCOL_VERSION = 2.0
XL430_MODEL_NUMBER = 1060
POSITION_MODE = 3
TICKS_PER_REVOLUTION = 4096
DEGREES_PER_TICK = 360.0 / TICKS_PER_REVOLUTION
PROFILE_VELOCITY_RPM_PER_UNIT = 0.229
PWM_PERCENT_PER_UNIT = 0.113
LOAD_PERCENT_PER_UNIT = 0.1

ADDR_DRIVE_MODE = 10
ADDR_OPERATING_MODE = 11
ADDR_VELOCITY_LIMIT = 44
ADDR_MAX_POSITION_LIMIT = 48
ADDR_MIN_POSITION_LIMIT = 52
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_VELOCITY_I_GAIN = 76
ADDR_VELOCITY_P_GAIN = 78
ADDR_POSITION_D_GAIN = 80
ADDR_POSITION_I_GAIN = 82
ADDR_POSITION_P_GAIN = 84
ADDR_FEEDFORWARD_2ND_GAIN = 88
ADDR_FEEDFORWARD_1ST_GAIN = 90
ADDR_GOAL_PWM = 100
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_LOAD = 126
ADDR_PRESENT_POSITION = 132


@dataclass(frozen=True)
class Snapshot:
    """Registers that must survive temporary bench ownership."""

    drive_mode: int
    operating_mode: int
    velocity_limit: int
    min_position: int
    max_position: int
    velocity_i_gain: int
    velocity_p_gain: int
    position_d_gain: int
    position_i_gain: int
    position_p_gain: int
    feedforward_2nd_gain: int
    feedforward_1st_gain: int
    goal_pwm: int
    profile_acceleration: int
    profile_velocity: int
    start_position: int


def bounded_target(
    slave_start: int,
    master_start: int,
    master_position: int,
    direction: int,
    max_delta: int,
    min_position: int = 0,
    max_position: int = 4095,
) -> int:
    """Map a master displacement into the slave's safe position range."""
    delta = direction * (master_position - master_start)
    delta = max(-max_delta, min(max_delta, delta))
    return max(min_position, min(max_position, slave_start + delta))


def signed16(value: int) -> int:
    """Interpret an unsigned SDK register value as signed 16-bit."""
    return value - 65536 if value >= 32768 else value


class XL430:
    """Small checked wrapper for one XL430 on a dedicated serial bus."""

    def __init__(self, device: str, baud: int, dxl_id: int) -> None:
        """Create handlers without opening the serial device."""
        self.port = PortHandler(device)
        self.packet = PacketHandler(PROTOCOL_VERSION)
        self.baud = baud
        self.dxl_id = dxl_id

    def open(self) -> None:
        """Open the port and verify the expected XL430 model."""
        if not self.port.openPort():
            raise RuntimeError("failed to open slave serial port")
        if not self.port.setBaudRate(self.baud):
            self.port.closePort()
            raise RuntimeError(f"failed to set slave baud: {self.baud}")
        try:
            model, result, error = self.packet.ping(self.port, self.dxl_id)
            self._check(result, error, "ping")
            if int(model) != XL430_MODEL_NUMBER:
                raise RuntimeError(
                    f"slave model {model} is not XL430 "
                    f"({XL430_MODEL_NUMBER})"
                )
        except Exception:
            self.port.closePort()
            raise

    def close(self) -> None:
        """Close the serial device."""
        self.port.closePort()

    def _check(self, result: int, error: int, label: str) -> None:
        if result != COMM_SUCCESS or error != 0:
            raise RuntimeError(
                f"{label} failed: result={self.packet.getTxRxResult(result)}, "
                f"device_error={error}"
            )

    def read1(self, address: int, label: str) -> int:
        """Read and validate one byte."""
        value, result, error = self.packet.read1ByteTxRx(
            self.port, self.dxl_id, address
        )
        self._check(result, error, label)
        return int(value)

    def read4(self, address: int, label: str) -> int:
        """Read and validate four bytes."""
        value, result, error = self.packet.read4ByteTxRx(
            self.port, self.dxl_id, address
        )
        self._check(result, error, label)
        return int(value)

    def read2(self, address: int, label: str) -> int:
        """Read and validate two bytes."""
        value, result, error = self.packet.read2ByteTxRx(
            self.port, self.dxl_id, address
        )
        self._check(result, error, label)
        return int(value)

    def write1(self, address: int, value: int, label: str) -> None:
        """Write and validate one byte."""
        result, error = self.packet.write1ByteTxRx(
            self.port, self.dxl_id, address, value
        )
        self._check(result, error, label)

    def write4(self, address: int, value: int, label: str) -> None:
        """Write and validate four bytes."""
        result, error = self.packet.write4ByteTxRx(
            self.port, self.dxl_id, address, value
        )
        self._check(result, error, label)

    def write2(self, address: int, value: int, label: str) -> None:
        """Write and validate two bytes."""
        result, error = self.packet.write2ByteTxRx(
            self.port, self.dxl_id, address, value
        )
        self._check(result, error, label)

    def prepare(
        self, profile_accel: int, profile_velocity: int, goal_pwm: int
    ) -> Snapshot:
        """Snapshot configuration, seed position, and enable slave torque."""
        if self.read1(ADDR_TORQUE_ENABLE, "torque") != 0:
            raise RuntimeError(
                "slave torque was already enabled; refusing takeover"
            )
        hardware_error = self.read1(
            ADDR_HARDWARE_ERROR_STATUS, "hardware error"
        )
        if hardware_error != 0:
            raise RuntimeError(f"slave hardware error: 0x{hardware_error:02x}")

        snapshot = Snapshot(
            drive_mode=self.read1(ADDR_DRIVE_MODE, "drive mode"),
            operating_mode=self.read1(ADDR_OPERATING_MODE, "operating mode"),
            velocity_limit=self.read4(ADDR_VELOCITY_LIMIT, "velocity limit"),
            min_position=self.read4(
                ADDR_MIN_POSITION_LIMIT, "minimum position limit"
            ),
            max_position=self.read4(
                ADDR_MAX_POSITION_LIMIT, "maximum position limit"
            ),
            velocity_i_gain=self.read2(
                ADDR_VELOCITY_I_GAIN, "velocity I gain"
            ),
            velocity_p_gain=self.read2(
                ADDR_VELOCITY_P_GAIN, "velocity P gain"
            ),
            position_d_gain=self.read2(
                ADDR_POSITION_D_GAIN, "position D gain"
            ),
            position_i_gain=self.read2(
                ADDR_POSITION_I_GAIN, "position I gain"
            ),
            position_p_gain=self.read2(
                ADDR_POSITION_P_GAIN, "position P gain"
            ),
            feedforward_2nd_gain=self.read2(
                ADDR_FEEDFORWARD_2ND_GAIN, "feedforward 2nd gain"
            ),
            feedforward_1st_gain=self.read2(
                ADDR_FEEDFORWARD_1ST_GAIN, "feedforward 1st gain"
            ),
            goal_pwm=self.read2(ADDR_GOAL_PWM, "goal pwm"),
            profile_acceleration=self.read4(
                ADDR_PROFILE_ACCELERATION, "profile acceleration"
            ),
            profile_velocity=self.read4(
                ADDR_PROFILE_VELOCITY, "profile velocity"
            ),
            start_position=self.read4(
                ADDR_PRESENT_POSITION,
                "present position",
            ),
        )
        if snapshot.drive_mode & 0x04:
            raise RuntimeError(
                "time-based profile is not supported by this bench tool"
            )
        if profile_velocity > snapshot.velocity_limit:
            raise RuntimeError(
                f"profile velocity {profile_velocity} exceeds motor velocity "
                f"limit {snapshot.velocity_limit}"
            )
        if not (
            snapshot.min_position
            <= snapshot.start_position
            <= snapshot.max_position
        ):
            raise RuntimeError(
                f"start position {snapshot.start_position} is outside motor "
                f"limits [{snapshot.min_position}, {snapshot.max_position}]"
            )
        try:
            self.write1(ADDR_TORQUE_ENABLE, 0, "torque disable")
            if snapshot.operating_mode != POSITION_MODE:
                self.write1(
                    ADDR_OPERATING_MODE,
                    POSITION_MODE,
                    "position mode",
                )
                snapshot = replace(
                    snapshot,
                    start_position=self.read4(
                        ADDR_PRESENT_POSITION,
                        "position-mode start position",
                    ),
                )
                if not (
                    snapshot.min_position
                    <= snapshot.start_position
                    <= snapshot.max_position
                ):
                    raise RuntimeError(
                        "position-mode start is outside motor limits"
                    )
            self.write4(
                ADDR_PROFILE_ACCELERATION,
                profile_accel,
                "profile acceleration",
            )
            self.write4(
                ADDR_PROFILE_VELOCITY,
                profile_velocity,
                "profile velocity",
            )
            self.write2(ADDR_GOAL_PWM, goal_pwm, "goal pwm cap")
            self.write4(
                ADDR_GOAL_POSITION,
                snapshot.start_position,
                "seed goal position",
            )
            self.write1(ADDR_TORQUE_ENABLE, 1, "torque enable")
        except Exception:
            self.stop_and_restore(snapshot)
            raise
        return snapshot

    def stop_and_restore(self, snapshot: Snapshot | None) -> None:
        """Disable torque and best-effort restore all changed registers."""
        try:
            self.write1(ADDR_TORQUE_ENABLE, 0, "torque disable")
        except Exception as exc:  # best-effort fail-safe cleanup
            print(f"WARNING: torque-disable failed: {exc}", flush=True)
            return
        if snapshot is None:
            return
        try:
            self.write1(
                ADDR_OPERATING_MODE,
                snapshot.operating_mode,
                "restore operating mode",
            )
            self.write2(
                ADDR_VELOCITY_I_GAIN,
                snapshot.velocity_i_gain,
                "restore velocity I gain",
            )
            self.write2(
                ADDR_VELOCITY_P_GAIN,
                snapshot.velocity_p_gain,
                "restore velocity P gain",
            )
            self.write2(
                ADDR_POSITION_D_GAIN,
                snapshot.position_d_gain,
                "restore position D gain",
            )
            self.write2(
                ADDR_POSITION_I_GAIN,
                snapshot.position_i_gain,
                "restore position I gain",
            )
            self.write2(
                ADDR_POSITION_P_GAIN,
                snapshot.position_p_gain,
                "restore position P gain",
            )
            self.write2(
                ADDR_FEEDFORWARD_2ND_GAIN,
                snapshot.feedforward_2nd_gain,
                "restore feedforward 2nd gain",
            )
            self.write2(
                ADDR_FEEDFORWARD_1ST_GAIN,
                snapshot.feedforward_1st_gain,
                "restore feedforward 1st gain",
            )
            self.write4(
                ADDR_PROFILE_ACCELERATION,
                snapshot.profile_acceleration,
                "restore profile acceleration",
            )
            self.write4(
                ADDR_PROFILE_VELOCITY,
                snapshot.profile_velocity,
                "restore profile velocity",
            )
            self.write2(ADDR_GOAL_PWM, snapshot.goal_pwm, "restore goal pwm")
        except Exception as exc:
            print(f"WARNING: configuration restore failed: {exc}", flush=True)


def parse_frame(line: bytes) -> dict:
    """Decode and strictly validate one newline-delimited JSON frame."""
    if len(line) > 1024:
        raise ValueError("frame too large")
    frame = json.loads(line.decode("utf-8"))
    if not isinstance(frame, dict) or frame.get("version") != 1:
        raise ValueError("unsupported frame")
    if not isinstance(frame.get("session"), str) or not frame["session"]:
        raise ValueError("invalid session")
    if type(frame.get("seq")) is not int or frame["seq"] < 0:
        raise ValueError("invalid sequence")
    if type(frame.get("position")) is not int:
        raise ValueError("invalid position")
    if not -1_000_000 <= frame["position"] <= 1_000_000:
        raise ValueError("position out of transport range")
    return frame


def parse_args() -> argparse.Namespace:
    """Parse Jetson-side command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--confirm-bench", action="store_true")
    parser.add_argument("--slave-id", type=int, required=True)
    parser.add_argument("--allowed-client", required=True)
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--tcp-port", type=int, default=9201)
    parser.add_argument(
        "--device",
        default=(
            "/dev/serial/by-id/"
            "usb-FTDI_USB__-__Serial_Converter_FTBEO3M5-if00-port0"
        ),
    )
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--direction", type=int, choices=(-1, 1), default=1)
    parser.add_argument(
        "--max-delta-ticks",
        type=int,
        default=20,
        help="relative travel cap; 114 ticks is about 10 degrees",
    )
    parser.add_argument(
        "--profile-velocity",
        type=int,
        default=5,
        help="XL430 raw value; 1 unit is about 1.374 degrees/second",
    )
    parser.add_argument("--profile-acceleration", type=int, default=2)
    parser.add_argument(
        "--goal-pwm",
        type=int,
        default=100,
        help="XL430 raw cap; 100 is about 11.3 percent",
    )
    parser.add_argument(
        "--load-limit-raw",
        type=int,
        default=180,
        help="software stop threshold; XL430 Present Load is 0.1 percent/raw",
    )
    parser.add_argument("--stale-ms", type=int, default=200)
    return parser.parse_args()


def main() -> int:
    """Serve one authorized master connection with fail-safe cleanup."""
    args = parse_args()
    if not args.confirm_bench:
        raise SystemExit("refusing direct control without --confirm-bench")
    if not 1 <= args.max_delta_ticks <= 4095:
        raise SystemExit("--max-delta-ticks must be between 1 and 4095")
    if not 50 <= args.stale_ms <= 1000:
        raise SystemExit("--stale-ms must be between 50 and 1000")
    if not 2 <= args.profile_velocity <= 50:
        raise SystemExit("--profile-velocity must be between 2 and 50")
    if not 1 <= args.profile_acceleration <= 25:
        raise SystemExit("--profile-acceleration must be between 1 and 25")
    if 2 * args.profile_acceleration > args.profile_velocity:
        raise SystemExit(
            "--profile-acceleration must not exceed half of "
            "--profile-velocity"
        )
    if not 1 <= args.goal_pwm <= 885:
        raise SystemExit("--goal-pwm must be between 1 and 885")
    if not 1 <= args.load_limit_raw <= 1000:
        raise SystemExit("--load-limit-raw must be between 1 and 1000")

    actuator = XL430(args.device, args.baud, args.slave_id)
    actuator.open()
    snapshot: Snapshot | None = None
    listener: socket.socket | None = None
    conn: socket.socket | None = None
    try:
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((args.listen_host, args.tcp_port))
        listener.listen(1)
        profile_degrees_per_second = (
            args.profile_velocity
            * PROFILE_VELOCITY_RPM_PER_UNIT
            * 6.0
        )
        print(
            f"slave safe/off: id={args.slave_id}; waiting on "
            f"{args.listen_host}:{args.tcp_port}; "
            f"profile≈{profile_degrees_per_second:.1f}°/s; "
            f"pwm≈{args.goal_pwm * PWM_PERCENT_PER_UNIT:.1f}%; "
            f"load soft-stop≈"
            f"{args.load_limit_raw * LOAD_PERCENT_PER_UNIT:.1f}%",
            flush=True,
        )
        conn, address = listener.accept()
        if address[0] != args.allowed_client:
            raise RuntimeError(
                f"rejected client {address[0]}; expected {args.allowed_client}"
            )
        print(f"client connected: {address[0]}", flush=True)
        conn.settimeout(0.05)

        buffer = b""
        master_start: int | None = None
        session: str | None = None
        last_seq = -1
        last_rx = time.monotonic()
        last_target: int | None = None
        last_feedback = 0.0
        overload_samples = 0

        while True:
            try:
                chunk = conn.recv(2048)
            except socket.timeout:
                if snapshot is not None and (
                    time.monotonic() - last_rx > args.stale_ms / 1000.0
                ):
                    raise RuntimeError("master stream stale")
                continue
            if not chunk:
                raise RuntimeError("master disconnected")
            buffer += chunk
            if len(buffer) > 4096:
                raise RuntimeError("receive buffer overflow")

            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                frame = parse_frame(line)
                if session is None:
                    session = frame["session"]
                    master_start = frame["position"]
                    snapshot = actuator.prepare(
                        args.profile_acceleration,
                        args.profile_velocity,
                        args.goal_pwm,
                    )
                    print(
                        f"slave enabled: start={snapshot.start_position} "
                        f"master_start={master_start} "
                        f"relative_limit=±{args.max_delta_ticks} "
                        f"motor_limits=[{snapshot.min_position},"
                        f"{snapshot.max_position}] "
                        f"velocity_limit={snapshot.velocity_limit}",
                        flush=True,
                    )
                if frame["session"] != session or frame["seq"] <= last_seq:
                    raise RuntimeError("session/sequence replay")
                last_seq = frame["seq"]
                last_rx = time.monotonic()
                assert snapshot is not None and master_start is not None
                target = bounded_target(
                    snapshot.start_position,
                    master_start,
                    frame["position"],
                    args.direction,
                    args.max_delta_ticks,
                    snapshot.min_position,
                    snapshot.max_position,
                )
                if target != last_target:
                    actuator.write4(
                        ADDR_GOAL_POSITION,
                        target,
                        "goal position",
                    )
                    last_target = target
                now = time.monotonic()
                if now - last_feedback >= 0.2:
                    present = actuator.read4(
                        ADDR_PRESENT_POSITION, "present position"
                    )
                    load = signed16(
                        actuator.read2(ADDR_PRESENT_LOAD, "present load")
                    )
                    print(
                        f"master={frame['position']:4d} target={target:4d} "
                        f"present={present:4d} load_raw={load:+4d}",
                        flush=True,
                    )
                    if abs(load) > args.load_limit_raw:
                        overload_samples += 1
                    else:
                        overload_samples = 0
                    if overload_samples >= 3:
                        raise RuntimeError(
                            f"load limit exceeded: raw={load}, "
                            f"limit={args.load_limit_raw}"
                        )
                    last_feedback = now
    except KeyboardInterrupt:
        print("server interrupted", flush=True)
        return 0
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"safety stop: {exc}", flush=True)
        return 2
    finally:
        actuator.stop_and_restore(snapshot)
        if conn is not None:
            conn.close()
        if listener is not None:
            listener.close()
        actuator.close()
        print("slave torque disabled", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
