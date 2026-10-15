#!/usr/bin/env python3
"""patch4 NVM acceptance (spec 5절 0번): read-only check of saved board configs."""
import argparse
import json
from pathlib import Path

STARTUP = ('startup_motor_calibration', 'startup_encoder_index_search',
           'startup_encoder_offset_calibration', 'startup_closed_loop_control',
           'startup_sensorless_control', 'startup_homing')


def check(configs, expected):
    violations = [f'missing board {s}' for s in sorted(set(expected) - set(configs))]
    violations += [f'unexpected board {s}' for s in sorted(set(configs) - set(expected))]
    owners = {}
    for serial, cfg in sorted(configs.items()):
        can = cfg['can']['config']
        if can['baud_rate'] != 500000:
            violations.append(f"{serial} can baud_rate={can['baud_rate']}")
        if can['protocol'] != 0:
            violations.append(f"{serial} can protocol={can['protocol']}")
        for axis in ('axis0', 'axis1'):
            ac = cfg[axis]['config']
            for flag in STARTUP:
                if ac.get(flag) is not False:
                    violations.append(f'{serial} {axis} {flag}={ac.get(flag)}')
            node, ext = ac['can_node_id'], ac['can_node_id_extended']
            limit = 0xFFFFFF if ext else 0x3F
            if not 0 <= node <= limit:
                violations.append(f"{serial} {axis} node {node} out of {'EXT' if ext else 'STD'} range")
            if not ext and node == 0x3F:
                violations.append(f'{serial} {axis} node 63 is reserved for the CAN probe')
            if ac['can_heartbeat_rate_ms'] <= 0:
                violations.append(f"{serial} {axis} can_heartbeat_rate_ms={ac['can_heartbeat_rate_ms']}")
            key = (node, ext)
            if key in owners:
                violations.append(f'duplicate node {node} ({owners[key]}, {serial} {axis})')
            owners[key] = f'{serial} {axis}'
    return violations


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--expect', action='append', required=True, help='serial that must be present (repeat)')
    ap.add_argument('--json-out', help='write {pass, expected, violations} for can_tx_order_probe')
    ap.add_argument('configs', nargs='+', help='SERIAL=path/to/config.json')
    args = ap.parse_args()
    configs = {}
    for item in args.configs:
        serial, path = item.split('=', 1)
        if serial in configs:
            raise SystemExit(f'duplicate config argument for {serial}')
        configs[serial] = json.loads(Path(path).read_text())
    found = check(configs, set(args.expect))
    if args.json_out:
        Path(args.json_out).write_text(json.dumps(
            {'pass': not found, 'expected': sorted(set(args.expect)), 'violations': found}, indent=2) + '\n')
    for line in found:
        print('FAIL', line)
    print('PASS' if not found else f'{len(found)} violation(s)')
    raise SystemExit(1 if found else 0)


if __name__ == '__main__':
    main()
