#!/usr/bin/env python3
"""Regenerate reliability.patch from an unpatched and an edited MKS source tree."""
import argparse
from pathlib import Path
import subprocess

PATCH_FILES = [
    'Firmware/communication/interface_can.cpp',
    'Firmware/communication/interface_can.hpp',
    'Firmware/communication/can_simple.cpp',
    'Firmware/MotorControl/axis.hpp',
    'Firmware/MotorControl/axis.cpp',
    'Firmware/MotorControl/low_level.cpp',
    'Firmware/odrive-interface.yaml',
    'Firmware/Tupfile.lua',
    'tools/odrive/mks_reliability_version.py',
]


def regenerate(original, work, output):
    chunks = []
    for rel in PATCH_FILES:
        old, new = original / rel, work / rel
        if not new.is_file():
            raise FileNotFoundError(new)
        if old.is_file():
            cmd = ['diff', '-u', '--label', 'a/' + rel, '--label', 'b/' + rel, str(old), str(new)]
        else:
            cmd = ['diff', '-u', '--label', '/dev/null', '--label', 'b/' + rel, '/dev/null', str(new)]
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode not in (0, 1):  # diff: 0 same, 1 different, 2 trouble
            raise RuntimeError(result.stderr)
        if result.returncode == 0:
            raise ValueError(f'{rel} has no patch change; remove it from PATCH_FILES deliberately')
        chunks.append(result.stdout)
    output.write_text(''.join(chunks))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--original', type=Path, required=True)
    ap.add_argument('--work', type=Path, required=True)
    ap.add_argument('--output', type=Path, default=Path(__file__).with_name('reliability.patch'))
    args = ap.parse_args()
    regenerate(args.original, args.work, args.output)
    print(args.output.resolve())


if __name__ == '__main__':
    main()
