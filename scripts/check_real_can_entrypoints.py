#!/usr/bin/env python3
"""Fail when a production Python file opens python-can outside the review allowlist."""

import argparse
from pathlib import Path
import re
import sys


ALLOWED_DIRECT_OPENERS = frozenset(
    {
        "motor_control/can_ak_odrive_demo.py",
        "motor_control/corner_module/drive_odrive_can.py",
        "motor_control/corner_module/steer_ak40.py",
        "motor_control/drive/bl70200/can_calibrate_all.py",
        "motor_control/drive/bl70200/can_drive_test.py",
        "motor_control/drive/x2212_test/odrive_can_drive.py",
        "motor_control/steering/ak_control.py",
        "motor_control/steering/calibrate_ak.py",
        "motor_control/steering/status_ak.py",
        "motor_gui/backend/transport/can_bus.py",
        "motor_gui/backend/transport/can_device.py",
        "scripts/preflight_hil.py",
    }
)

PRODUCTION_ROOTS = (
    "motor_control",
    "motor_gui",
    "ros2/src",
    "scripts",
)
_DIRECT_BUS = re.compile(r"\bcan\s*\.\s*(?:interface\s*\.\s*)?Bus\s*\(")
_EXCLUDED_PARTS = frozenset({"test", "tests", "__pycache__"})


def _is_test_fake_or_vcan(path: Path) -> bool:
    if _EXCLUDED_PARTS.intersection(path.parts):
        return True
    lowered = path.name.lower()
    return "fake" in lowered or "vcan" in lowered


def find_violations(root: Path):
    violations = []
    for relative_root in PRODUCTION_ROOTS:
        tree = root / relative_root
        if not tree.is_dir():
            continue
        for path in sorted(tree.rglob("*.py")):
            relative = path.relative_to(root)
            relative_text = relative.as_posix()
            if (
                relative_text in ALLOWED_DIRECT_OPENERS
                or _is_test_fake_or_vcan(relative)
            ):
                continue
            for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(),
                start=1,
            ):
                match = _DIRECT_BUS.search(line)
                if match:
                    violations.append(
                        (relative_text, line_number, match.group(0))
                    )
    return violations


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="check reviewed ownership of direct real CAN openers"
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="repository root (default: script parent)",
    )
    args = parser.parse_args(argv)
    root = args.root.resolve()
    violations = find_violations(root)
    if violations:
        for path, line_number, opener in violations:
            print(
                f"real CAN opener outside allowlist: {path}:{line_number}: {opener}",
                file=sys.stderr,
            )
        return 1
    print("real CAN entrypoint check PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
