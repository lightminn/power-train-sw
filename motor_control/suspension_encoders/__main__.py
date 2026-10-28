"""Read-only AS5048B suspension encoder CLI.

    python3 -m suspension_encoders scan            # probe addresses + OTP regs
    python3 -m suspension_encoders read --count 20 # live per-joint angles
    python3 -m suspension_encoders capture-zero    # dry run: print new config
    python3 -m suspension_encoders capture-zero --write
    python3 -m suspension_encoders capture-zero --joint bl_bogie_joint --reference bl_bogie_joint=5.0

Run with ``PYTHONPATH=<repo>/motor_control`` on the Jetson host (stdlib only)
or inside a container that has the I2C device.  Nothing writes sensor
registers or OTP; ``capture-zero --write`` only rewrites the JSON config.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import json
import math
import sys
import time

from suspension_encoders.as5048b import (
    COUNTS_PER_TURN,
    LinuxI2cBus,
    read_otp_info,
    read_readout,
)
from suspension_encoders.config import (
    ConfigError,
    SuspensionEncoderConfig,
    load_config,
    resolve_config_path,
    write_config,
)
from suspension_encoders.reader import (
    ChannelReading,
    SuspensionEncoderReader,
    calibration_state,
)


def _diag_text(readout) -> str:
    flags = [name for name, on in (
        ("OCF", readout.ocf), ("COF", readout.cof),
        ("CompLow", readout.comp_low), ("CompHigh", readout.comp_high),
    ) if on]
    return f"0x{readout.diag:02x}[{','.join(flags) or '-'}]"


def _format_reading(reading: ChannelReading) -> str:
    channel = reading.channel
    head = f"{channel.joint:<16} 0x{channel.address:02x}"
    readout = reading.readout
    if readout is None:
        return f"{head} ERROR {reading.error}"
    state = ("raw", "zero-unverified", "calibrated")[calibration_state(channel)]
    text = (
        f"{head} raw={readout.angle_counts:5d} abs={math.degrees(readout.angle_rad):7.2f}deg "
        f"agc={readout.agc:3d} mag={readout.magnitude:5d} diag={_diag_text(readout)}"
    )
    if reading.ok:
        text += f" joint={math.degrees(reading.joint_rad):+8.3f}deg ({state})"
    else:
        text += f" INVALID {reading.error}"
    if readout.warnings:
        text += " WARN " + "; ".join(readout.warnings)
    return text


def _load(args) -> SuspensionEncoderConfig:
    config = load_config(args.config)
    if args.bus is not None:
        config = replace(config, i2c_bus=args.bus)
    return config


def cmd_scan(args) -> int:
    config = _load(args)
    bus = LinuxI2cBus(config.i2c_bus)
    # Only the configured AS5048B addresses: the register-pointer byte this
    # read sends would be an output write on other I2C parts (e.g. PCF8574).
    addresses = sorted(c.address for c in config.channels)
    joints = {c.address: c.joint for c in config.channels}
    found = 0
    print(f"bus /dev/i2c-{config.i2c_bus} (config {config.source_path})")
    try:
        for address in addresses:
            label = joints.get(address, "-")
            try:
                readout = read_readout(bus, address)
                otp = read_otp_info(bus, address)
            except (OSError, ValueError) as exc:
                print(f"0x{address:02x} {label:<16} no AS5048B: {type(exc).__name__}: {exc}")
                continue
            found += 1
            note = "" if otp.zero_position_counts == 0 else "  WARN OTP/volatile zero register nonzero"
            print(
                f"0x{address:02x} {label:<16} raw={readout.angle_counts:5d} "
                f"agc={readout.agc:3d} mag={readout.magnitude:5d} diag={_diag_text(readout)} "
                f"reg0x15={otp.slave_address_bits:#04x} zero_reg={otp.zero_position_counts}{note}"
            )
    finally:
        bus.close()
    print(f"found {found} AS5048B device(s)")
    return 0 if found else 1


def cmd_read(args) -> int:
    config = _load(args)
    reader = SuspensionEncoderReader(config)
    period = 1.0 / args.hz
    count = 0
    try:
        while args.count <= 0 or count < args.count:
            started = time.monotonic()
            sample = reader.read_once()
            print(f"--- t={started:.3f}")
            for reading in sample.readings:
                print(_format_reading(reading))
            sys.stdout.flush()
            count += 1
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    finally:
        reader.close()
    return 0


def circular_mean_counts(counts: list[int]) -> tuple[int, float]:
    """Return (mean count, max |deviation| in counts) on the 14-bit circle."""
    angles = [2.0 * math.pi * c / COUNTS_PER_TURN for c in counts]
    mean = math.atan2(
        sum(math.sin(a) for a in angles) / len(angles),
        sum(math.cos(a) for a in angles) / len(angles),
    )
    mean_counts = round(mean / (2.0 * math.pi) * COUNTS_PER_TURN) % COUNTS_PER_TURN
    half = COUNTS_PER_TURN // 2
    spread = max(abs((c - mean_counts + half) % COUNTS_PER_TURN - half) for c in counts)
    return mean_counts, float(spread)


def capture_zero(
    reader: SuspensionEncoderReader,
    *,
    joints: set[str],
    samples: int,
    period_s: float,
    max_spread_counts: int,
    reference_deg: dict[str, float] | None = None,
    sleep=time.sleep,
) -> tuple[dict[str, int], list[str]]:
    """Average raw counts per joint; returns (zeros, problems).

    The joint must be held at a KNOWN angle: ``reference_deg[joint]`` (default
    0 = CAD/URDF reference pose).  Stillness alone does not prove the pose;
    the zero is chosen so that this pose reads exactly the reference angle.
    """
    reference_deg = reference_deg or {}
    channels = [c for c in reader.config.channels if c.joint in joints]
    collected: dict[str, list[int]] = {c.joint: [] for c in channels}
    problems: list[str] = []
    for index in range(samples):
        for channel in channels:
            reading = reader.read_channel(channel)
            if reading.readout is None or not reading.ok:
                problems.append(f"{channel.joint}: sample {index}: {reading.error}")
            else:
                collected[channel.joint].append(reading.readout.angle_counts)
        if index + 1 < samples:
            sleep(period_s)
    zeros: dict[str, int] = {}
    for joint, counts in collected.items():
        if len(counts) != samples:
            continue
        mean, spread = circular_mean_counts(counts)
        if spread > max_spread_counts:
            problems.append(
                f"{joint}: spread {spread:.0f} counts > {max_spread_counts} (joint moving or noisy)"
            )
            continue
        channel = next(c for c in channels if c.joint == joint)
        offset = round(channel.sign * reference_deg.get(joint, 0.0) * COUNTS_PER_TURN / 360.0)
        zeros[joint] = (mean - offset) % COUNTS_PER_TURN
    return zeros, problems


def _parse_references(items: list[str] | None, joints: set[str]) -> dict[str, float]:
    references: dict[str, float] = {}
    for item in items or []:
        joint, separator, text = item.partition("=")
        if not separator or joint not in joints:
            raise ValueError(f"--reference must be JOINT=DEG for a captured joint: {item!r}")
        value = float(text)
        if not math.isfinite(value) or abs(value) > 180.0:
            raise ValueError(f"--reference angle must be within +-180 deg: {item!r}")
        references[joint] = value
    return references


def _channel_identity(config: SuspensionEncoderConfig):
    return config.i2c_bus, tuple((c.joint, c.address, c.sign) for c in config.channels)


def cmd_capture_zero(args) -> int:
    if args.bus is not None:
        # Zeros are only meaningful for the bus/addresses in the file they go to.
        print("capture-zero does not accept --bus; edit i2c_bus in the config", file=sys.stderr)
        return 2
    config = _load(args)
    names = {c.joint for c in config.channels}
    joints = set(args.joint) if args.joint else names
    unknown = joints - names
    if unknown:
        print(f"unknown joint(s): {sorted(unknown)}", file=sys.stderr)
        return 2
    try:
        references = _parse_references(args.reference, joints)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    reader = SuspensionEncoderReader(config)
    try:
        zeros, problems = capture_zero(
            reader,
            joints=joints,
            samples=args.samples,
            period_s=1.0 / args.hz,
            max_spread_counts=args.max_spread_counts,
            reference_deg=references,
        )
    finally:
        reader.close()
    for problem in problems:
        print(f"FAIL {problem}", file=sys.stderr)
    if problems or set(zeros) != joints:
        print("zero capture refused; config unchanged", file=sys.stderr)
        return 1
    # Re-load and require the same bus/address/sign the zeros were taken with.
    current = load_config(config.source_path)
    if _channel_identity(current) != _channel_identity(config):
        print("config changed during capture (bus/address/sign); refused", file=sys.stderr)
        return 1
    updated = current.with_zero_counts(zeros)
    for channel in updated.channels:
        if channel.joint in zeros:
            print(
                f"{channel.joint:<16} 0x{channel.address:02x} zero_counts={channel.zero_counts} "
                f"(pose held at {references.get(channel.joint, 0.0):+.2f} deg)"
            )
    print(
        "note: captured channels are reset to verified=false and are NOT sent to the "
        "console until mapping, sign and zero are checked at a known pose and "
        "verified=true is set by hand"
    )
    if not args.write:
        print(json.dumps(updated.to_json_dict(), indent=2, ensure_ascii=False))
        print("dry run; pass --write to update", resolve_config_path(args.config))
        return 0
    path = write_config(updated, config.source_path)
    print(f"wrote {path}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python3 -m suspension_encoders", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", help="config path (default: <repo>/config/suspension_encoders.json)")
    parser.add_argument("--bus", type=int, help="override i2c bus number")
    sub = parser.add_subparsers(dest="command", required=True)
    scan = sub.add_parser("scan", help="probe AS5048B addresses and OTP registers")
    scan.set_defaults(func=cmd_scan)
    read = sub.add_parser("read", help="print live readings")
    read.add_argument("--count", type=int, default=0, help="samples (0 = until Ctrl-C)")
    read.add_argument("--hz", type=float, default=5.0)
    read.set_defaults(func=cmd_read)
    capture = sub.add_parser("capture-zero", help="capture current raw counts as zero offsets")
    capture.add_argument("--joint", action="append", help="limit to joint (repeatable)")
    capture.add_argument("--reference", action="append", metavar="JOINT=DEG",
                         help="known joint angle held during capture (default 0 = CAD pose)")
    capture.add_argument("--samples", type=int, default=25)
    capture.add_argument("--hz", type=float, default=20.0)
    capture.add_argument("--max-spread-counts", type=int, default=23, help="~0.5 deg")
    capture.add_argument("--write", action="store_true", help="rewrite the config file")
    capture.set_defaults(func=cmd_capture_zero)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if getattr(args, "hz", 1.0) <= 0 or getattr(args, "samples", 1) <= 0:
        print("hz and samples must be positive", file=sys.stderr)
        return 2
    try:
        return args.func(args)
    except (ConfigError, OSError) as exc:
        print(f"error: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
