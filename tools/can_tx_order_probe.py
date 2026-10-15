#!/usr/bin/env python3
"""Measure same-ID TX order and queue latency on SocketCAN from own-message echoes.

Sends only on STD node 63 (0x3F). Requires a passing mks_nvm_acceptance report for
all drive boards (which reserves node 63 and forbids silent axes); AK servos use EXT IDs
(motor_control/steering/ak_control.py). It also listens first and aborts on node 63 traffic.
Latency = kernel echo timestamp - wall time taken just before send(); both CLOCK_REALTIME.
"""
import argparse
import json
import struct
import subprocess
import threading
import time

PROBE_NODE = 0x3F
MARKER = 0xA5A5


def probe_id(cmd):
    if not 0 <= cmd < 32:
        raise ValueError('command must be 0..31')
    return (PROBE_NODE << 5) | cmd


def analyze(sent, echoed):
    sent_seq = [seq for seq, _ in sent]
    echo_seq = [seq for seq, _ in echoed]
    sent_time = dict(sent)
    latencies = sorted((t - sent_time[seq]) * 1000.0 for seq, t in echoed if seq in sent_time)

    def pct(q):
        return latencies[min(len(latencies) - 1, int(q * (len(latencies) - 1) + 0.5))] if latencies else None
    return {'accepted': bool(sent_seq) and echo_seq == sent_seq,
            'missing': sorted(set(sent_seq) - set(echo_seq)),
            'echoed': len(echo_seq), 'sent': len(sent_seq),
            'p50_ms': pct(0.5), 'p99_ms': pct(0.99), 'max_ms': latencies[-1] if latencies else None}


def require_inventory(report, minimum_boards=3):
    if report.get('pass') is not True or report.get('violations'):
        raise SystemExit('NVM acceptance did not pass; refusing to transmit')
    if len(report.get('expected', [])) < minimum_boards:
        raise SystemExit('NVM acceptance did not cover every drive board; refusing to transmit')


def echo_enabled(channel):
    out = subprocess.run(['ip', '-details', 'link', 'show', channel],
                         capture_output=True, text=True, check=True).stdout
    flags = out.split('<', 1)[1].split('>', 1)[0].split(',')
    return 'ECHO' in flags


def run(channel, cmd, count, gap_s, listen_s, timeout_s, report):
    import can  # python-can, present in the Jetson container
    require_inventory(report)
    can_id = probe_id(cmd)
    if not echo_enabled(channel):
        raise SystemExit(f'{channel} lacks IFF_ECHO: echoes would not mean TX completion')
    bus = can.Bus(interface='socketcan', channel=channel, receive_own_messages=True)
    try:
        end = time.time() + listen_s
        while time.time() < end:
            msg = bus.recv(timeout=0.05)
            if msg is not None and not msg.is_extended_id and msg.arbitration_id >> 5 == PROBE_NODE:
                raise SystemExit(f'node 63 already in use on the bus (0x{msg.arbitration_id:X}); refusing')
        sent, echoed = [], []
        deadline = time.time() + timeout_s

        def receive():
            while len(echoed) < count and time.time() < deadline:
                msg = bus.recv(timeout=0.05)
                if (msg is not None and msg.arbitration_id == can_id and len(msg.data) == 8
                        and struct.unpack('<H', bytes(msg.data[4:6]))[0] == MARKER):
                    echoed.append((struct.unpack('<I', bytes(msg.data[:4]))[0], msg.timestamp))
        reader = threading.Thread(target=receive)
        reader.start()
        for seq in range(count):
            sent.append((seq, time.time()))  # before send(): includes any queueing inside send
            bus.send(can.Message(arbitration_id=can_id, is_extended_id=False,
                                 data=struct.pack('<IHxx', seq, MARKER)), timeout=1.0)
            if gap_s:
                time.sleep(gap_s)
        reader.join()
    finally:
        bus.shutdown()
    return analyze(sent, echoed)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--channel', default='can0')
    ap.add_argument('--cmd', type=int, default=7, help='command bits on node 63 (0..31)')
    ap.add_argument('--count', type=int, default=1000)
    ap.add_argument('--gap-ms', type=float, default=0.0)
    ap.add_argument('--listen-s', type=float, default=2.0)
    ap.add_argument('--timeout-s', type=float, default=10.0)
    ap.add_argument('--nvm-report', required=True, help='mks_nvm_acceptance --json-out result')
    args = ap.parse_args()
    with open(args.nvm_report) as stream:
        report = json.load(stream)
    print(json.dumps(run(args.channel, args.cmd, args.count, args.gap_ms / 1000.0,
                         args.listen_s, args.timeout_s, report)))


if __name__ == '__main__':
    main()
