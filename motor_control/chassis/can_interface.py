"""One CAN readiness contract for host startup and HIL checks.

iproute2 omits disabled CAN controller modes. IFF_UP is authoritative for CAN;
operstate UNKNOWN is normal. Controller <LOOPBACK> is not IFF_LOOPBACK.
"""
import argparse
from pathlib import Path
import re
import subprocess
import sys


class CanReadinessError(RuntimeError):
    pass


def validate_details(details, channel="can0", *, full=False):
    header = re.search(r"^\d+:\s+([^:]+):\s+<([^>]*)>", details, re.M)
    if header is None or header.group(1).split("@", 1)[0] != channel:
        raise CanReadinessError(f"{channel}: missing interface details")
    flags = set(header.group(2).split(","))
    if "UP" not in flags:
        raise CanReadinessError(f"{channel}: interface UP flag missing")
    controller = re.search(r"^\s+can\s+(?:<([^>]*)>\s+)?state\s+(\S+)", details, re.M)
    if controller is None:
        raise CanReadinessError(f"{channel}: CAN controller state missing")
    modes = set((controller.group(1) or "").split(","))
    for mode in ("LOOPBACK", "LISTEN-ONLY"):
        if mode in flags or mode in modes or re.search(
                rf"\b{mode.lower()}\s+on\b", details, re.I):
            raise CanReadinessError(f"{channel}: CAN {mode} enabled")
    state = controller.group(2)
    if state not in ("ERROR-ACTIVE", "ERROR-WARNING", "ERROR-PASSIVE"):
        raise CanReadinessError(f"{channel}: CAN controller {state}")
    bitrate = re.search(r"\bbitrate\s+(\d+)\b", details)
    if bitrate is None or int(bitrate.group(1)) != 500000:
        raise CanReadinessError(f"{channel}: bitrate must be exactly 500000")
    if full:
        for name, expected in (("restart-ms", 100), ("qlen", 1000)):
            value = re.search(rf"\b{name}\s+(\d+)\b", details)
            if value is None or int(value.group(1)) != expected:
                raise CanReadinessError(f"{channel}: {name} must be {expected}")
    return f"{channel} UP / 500000 bps / controller {state} / LOOPBACK, LISTEN-ONLY off"


def readiness(channel="can0", *, full=False):
    result = subprocess.run(["ip", "-details", "link", "show", channel],
                            capture_output=True, text=True, timeout=5)
    if result.returncode:
        raise CanReadinessError(f"{channel}: ip query failed: {result.stderr.strip()}")
    return validate_details(result.stdout, channel, full=full)


def setup(channel="can0", *, lock_path=None):
    from chassis.runtime_lock import CanMaintenanceSession
    # A healthy repeated invocation is observation-only, including under an owner.
    try:
        return readiness(channel, full=True)
    except (CanReadinessError, OSError, subprocess.SubprocessError):
        pass
    with CanMaintenanceSession(channel, path=lock_path) as maintenance:
        # Recheck after acquiring the mutation lock, before any pin/link changes.
        try:
            return readiness(channel, full=True)
        except (CanReadinessError, OSError, subprocess.SubprocessError):
            pass
        maintenance.mark_reset()
        for module in ("can", "can_raw", "mttcan"):
            subprocess.run(["modprobe", module], check=True)
        for address, value in (("0x0c303018", "0xc458"), ("0x0c303010", "0xc400")):
            subprocess.run(["busybox", "devmem", address, "w", value], check=True)
        for suffix in (("down",), ("type", "can", "bitrate", "500000", "loopback", "off",
                        "listen-only", "off", "restart-ms", "100"),
                       ("txqueuelen", "1000"), ("up",)):
            subprocess.run(["ip", "link", "set", channel, *suffix], check=True)
        return readiness(channel, full=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", default="can0")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--setup", action="store_true")
    mode.add_argument("--require-idle", action="store_true")
    parser.add_argument("--lock-path")
    args = parser.parse_args(argv)
    try:
        if args.require_idle:
            from chassis.runtime_lock import RealCanSession
            with RealCanSession(channel=args.channel, owner="prepare-check",
                                path=args.lock_path or f"/run/powertrain/{args.channel}.lock"):
                result = f"{args.channel}: no active owner"
        else:
            result = setup(args.channel, lock_path=args.lock_path) if args.setup else readiness(args.channel)
    except (RuntimeError, OSError, ValueError, subprocess.SubprocessError) as exc:
        print(f"CAN FAIL: {exc}", file=sys.stderr)
        return 1
    print(result)
    return 0


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    raise SystemExit(main())
