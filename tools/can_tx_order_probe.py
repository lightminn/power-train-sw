#!/usr/bin/env python3
"""Measure same-ID TX order and queue latency on SocketCAN from this socket's own echoes.

Sends only STD 0x7E1 = node 63 heartbeat (cmd 0x01), a no-op in the patched ODrive router.
Node 63 is reserved by mks_nvm_acceptance; before sending, the probe requires a passing
NVM report for every drive board, listens for any node-63 traffic and queries node 63 with
an RTR so a board renumbered at runtime with heartbeats off is still found. AK servos use
EXT IDs (motor_control/steering/ak_control.py).

Own echoes are recognised by MSG_CONFIRM from recvmsg() (set only for frames this socket
sent), plus a per-run nonce. Latency = kernel SO_TIMESTAMPNS of the echo minus wall time
taken just before send(); both CLOCK_REALTIME. Python standard library only.
"""
import argparse
import errno
import json
import math
import secrets
import socket
import struct
import subprocess
import threading
import time

PROBE_NODE = 0x3F
PROBE_ID = (PROBE_NODE << 5) | 0x01  # heartbeat: ignored by the ODrive CAN router
QUERY_ID = (PROBE_NODE << 5) | 0x09  # encoder estimates: any live node-63 axis answers
MARKER = 0xA5A5
FRAME = struct.Struct('=IB3x8s')  # struct can_frame
TIMESPEC = struct.Struct('@qq')
SO_TIMESTAMPNS = getattr(socket, 'SO_TIMESTAMPNS', 35)  # Linux value; some Pythons lack the name
SO_RXQ_OVFL = getattr(socket, 'SO_RXQ_OVFL', 40)        # per-socket receive-queue drop counter
SO_RCVBUFFORCE = getattr(socket, 'SO_RCVBUFFORCE', 33)
RECEIVE_BUFFER = 8 * 1024 * 1024
DROPS = struct.Struct('=I')


def encode(seq, nonce):
    return struct.pack('<IHH', seq, MARKER, nonce)


def pack_frame(can_id, data):
    return FRAME.pack(can_id, len(data), data.ljust(8, b'\0'))


def own_echo_seq(frame, flags, nonce):
    """Sequence number of this socket's confirmed echo of this run, else None."""
    if not flags & socket.MSG_CONFIRM:
        return None
    can_id, length, data = FRAME.unpack(frame[:FRAME.size])
    if can_id != PROBE_ID or length != 8:  # EFF/RTR flags make can_id differ
        return None
    seq, marker, tag = struct.unpack('<IHH', data)
    return seq if marker == MARKER and tag == nonce else None


def query_confirmed(frame, flags):
    """True when this socket's own node-63 RTR query has actually left on the bus."""
    if not flags & socket.MSG_CONFIRM:
        return False
    return FRAME.unpack(frame[:FRAME.size])[0] == QUERY_ID | socket.CAN_RTR_FLAG


def foreign_node63(frame, flags):
    """True for any STD node-63 frame that this socket did not send."""
    if flags & socket.MSG_CONFIRM:
        return False
    can_id = FRAME.unpack(frame[:FRAME.size])[0]
    if can_id & socket.CAN_EFF_FLAG:
        return False
    return (can_id & socket.CAN_SFF_MASK) >> 5 == PROBE_NODE


def ancillary_drops(ancillary):
    """Receive-queue drop counter from SO_RXQ_OVFL ancillary data, else None."""
    for level, kind, value in ancillary:
        if level == socket.SOL_SOCKET and kind == SO_RXQ_OVFL:
            return DROPS.unpack(value[:DROPS.size])[0]
    return None


def analyze(sent, echoed, send_error=None, node63_seen=False, requested=None, socket_drops=0):
    sent_seq = [seq for seq, _ in sent]
    echo_seq = [seq for seq, _ in echoed]
    sent_time = dict(sent)
    latencies = sorted((t - sent_time[seq]) * 1000.0 for seq, t in echoed if seq in sent_time)

    def pct(q):
        return latencies[min(len(latencies) - 1, int(q * (len(latencies) - 1) + 0.5))] if latencies else None
    return {'accepted': (bool(sent_seq) and echo_seq == sent_seq and send_error is None
                         and not node63_seen and (requested is None or len(sent_seq) == requested)
                         and socket_drops == 0),
            'requested': requested,
            # >0 means this tool's receive queue overflowed: a tool failure, not a bus verdict.
            'socket_drops': socket_drops,
            'node63_seen': node63_seen,
            'missing': sorted(set(sent_seq) - set(echo_seq)),
            'echoed': len(echo_seq), 'sent': len(sent_seq), 'send_error': send_error,
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


def open_socket(channel):
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 1)
    sock.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
    sock.setsockopt(socket.SOL_SOCKET, SO_RXQ_OVFL, 1)
    try:  # a deep queue keeps a slow host from losing its own echoes; root may exceed rmem_max
        sock.setsockopt(socket.SOL_SOCKET, SO_RCVBUFFORCE, RECEIVE_BUFFER)
    except PermissionError:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RECEIVE_BUFFER)
    sock.bind((channel,))
    sock.settimeout(0.05)
    return sock


def receive(sock):
    """(frame, flags, kernel timestamp or None, drop counter or None), or None on timeout."""
    try:
        frame, ancillary, flags, _ = sock.recvmsg(
            FRAME.size, socket.CMSG_SPACE(TIMESPEC.size) + socket.CMSG_SPACE(DROPS.size))
    except socket.timeout:
        return None
    stamp = None
    for level, kind, value in ancillary:
        if level == socket.SOL_SOCKET and kind == SO_TIMESTAMPNS:
            sec, nsec = TIMESPEC.unpack(value[:TIMESPEC.size])
            stamp = sec + nsec / 1e9
    return frame, flags, stamp, ancillary_drops(ancillary)


def check_node63_absent(sock, listen_s, confirm_timeout_s=2.0):
    """Query node 63 and listen listen_s after it is confirmed; return the drop counter."""
    sock.send(pack_frame(QUERY_ID | socket.CAN_RTR_FLAG, b''))
    confirm_by, listen_until, drops = time.time() + confirm_timeout_s, None, 0
    while listen_until is None or time.time() < listen_until:
        if listen_until is None and time.time() > confirm_by:
            raise SystemExit('node-63 query was never confirmed on the bus; refusing to transmit')
        got = receive(sock)
        if got is None:
            continue
        drops = got[3] if got[3] is not None else drops
        if foreign_node63(got[0], got[1]):
            raise SystemExit('node 63 is live on the bus; refusing to transmit')
        if listen_until is None and query_confirmed(got[0], got[1]):
            listen_until = time.time() + listen_s
    return drops


def run(channel, count, gap_s, listen_s, timeout_s, report):
    require_inventory(report)
    if not echo_enabled(channel):
        raise SystemExit(f'{channel} lacks IFF_ECHO: echoes would not mean TX completion')
    nonce = secrets.randbits(16)
    sock = open_socket(channel)
    try:
        drops = {'base': check_node63_absent(sock, listen_s)}
        drops['last'] = drops['base']
        sent, echoed, send_error = [], [], None
        node63 = threading.Event()
        deadline = time.time() + timeout_s

        def reader():
            while len(echoed) < len(sent) or (send_error is None and len(echoed) < count):
                if time.time() > deadline or node63.is_set():
                    return
                got = receive(sock)
                if got is None:
                    continue
                if got[3] is not None:
                    drops['last'] = got[3]
                if foreign_node63(got[0], got[1]):
                    node63.set()  # a live node 63 appeared mid-run: stop and fail
                    return
                seq = own_echo_seq(got[0], got[1], nonce)
                if seq is not None:
                    echoed.append((seq, got[2]))
        thread = threading.Thread(target=reader)
        thread.start()
        for seq in range(count):
            # Never keep transmitting unsupervised: stop once the receiver is gone.
            if node63.is_set() or not thread.is_alive() or time.time() > deadline:
                break
            sent.append((seq, time.time()))  # before send(): includes any queueing inside send
            try:
                sock.send(pack_frame(PROBE_ID, encode(seq, nonce)))
            except OSError as exc:
                sent.pop()
                send_error = errno.errorcode.get(exc.errno, str(exc))
                break
            if gap_s:
                time.sleep(gap_s)
        thread.join()
    finally:
        sock.close()
    if any(stamp is None for _, stamp in echoed):
        raise SystemExit('kernel echo timestamps unavailable')
    return analyze(sent, echoed, send_error, node63.is_set(), requested=count,
                   socket_drops=drops['last'] - drops['base'])


def listen_seconds(text):
    value = float(text)
    if not math.isfinite(value) or value < 0.5:
        raise argparse.ArgumentTypeError('listen window must be at least 0.5 s')
    return value


def finite(minimum, inclusive=True):
    def parse(text):
        value = float(text)
        if not math.isfinite(value) or value < minimum or (not inclusive and value == minimum):
            raise argparse.ArgumentTypeError(f'must be a finite number {">=" if inclusive else ">"} {minimum}')
        return value
    return parse


def positive_int(text):
    value = int(text)
    if value < 1:
        raise argparse.ArgumentTypeError('must be at least 1')
    return value


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--channel', default='can0')
    ap.add_argument('--count', type=positive_int, default=1000)
    ap.add_argument('--gap-ms', type=finite(0.0), default=0.0)
    ap.add_argument('--listen-s', type=listen_seconds, default=2.0)
    ap.add_argument('--timeout-s', type=finite(0.0, inclusive=False), default=10.0)
    ap.add_argument('--nvm-report', required=True, help='mks_nvm_acceptance --json-out result')
    return ap


def main():
    args = build_parser().parse_args()
    with open(args.nvm_report) as stream:
        report = json.load(stream)
    print(json.dumps(run(args.channel, args.count, args.gap_ms / 1000.0,
                         args.listen_s, args.timeout_s, report)))


if __name__ == '__main__':
    main()
