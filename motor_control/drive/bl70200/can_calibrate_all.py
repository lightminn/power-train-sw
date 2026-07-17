#!/usr/bin/env python3
"""구동 ODrive 다축 CAN 풀캘리 — 전원 켤 때마다 필요(캘리 RAM-only).

BL70200 HALL 캘리는 `Set_Axis_State`(CANSimple cmd 0x07) = **state 3**
(FULL_CALIBRATION_SEQUENCE) 로 CAN 만으로 된다(HALL 은 index 없어 MOTOR_CAL→
OFFSET_CAL 로 내부 분해 실행 → heartbeat 상태전이 4→7→1 로 관찰됨).
USB 없이 can0 로 6축을 한 번에(순차) 캘리. 한 축씩 = 48V 전류 스파이크 방지.

⚠️ 각 축 출력축(바퀴)이 ~55s 양방향 회전 → 반드시 바퀴 자유(무부하) 상태에서.
사전: can0 500k UP (`bash scripts/can_setup.sh` 또는 host `ip link set can0 up
type can bitrate 500000 restart-ms 100`). 컨테이너(powertrain_jetson)에서 실행.

실행:
  python3 motor_control/drive/bl70200/can_calibrate_all.py            # node 11~16
  python3 motor_control/drive/bl70200/can_calibrate_all.py --nodes 11 12
"""
import argparse
from pathlib import Path
import struct
import sys
import time

CMD_HEARTBEAT, CMD_SET_STATE, CMD_CLEAR_ERRORS = 0x01, 0x07, 0x18
S_IDLE, S_FULL_CAL = 1, 3                     # AxisState
S_MOTOR_CAL, S_OFFSET_CAL = 4, 7
CAL_STATES = (3, 4, 6, 7)                     # FULL/MOTOR/INDEX/OFFSET (진행 중)
DRIVE_NODES = [11, 12, 13, 14, 15, 16]


def arb(node, cmd):
    return (node << 5) | cmd


def drain(bus):
    while bus.recv(timeout=0.0) is not None:
        pass


def heartbeat(bus, node, timeout=2.0):
    """해당 node heartbeat 한 개 → (axis_error, axis_state) 또는 None."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        m = bus.recv(timeout=timeout)
        if m is None:
            return None
        if (not m.is_extended_id and (m.arbitration_id >> 5) == node
                and (m.arbitration_id & 0x1F) == CMD_HEARTBEAT and len(m.data) >= 5):
            return struct.unpack("<I", m.data[0:4])[0], m.data[4]
    return None


def _can_message(**kwargs):
    import can

    return can.Message(**kwargs)


def send_state(bus, node, state):
    bus.send(_can_message(arbitration_id=arb(node, CMD_SET_STATE),
                          data=struct.pack("<I", state) + bytes(4), is_extended_id=False))


def clear_errors(bus, node):
    bus.send(_can_message(arbitration_id=arb(node, CMD_CLEAR_ERRORS),
                          data=bytes(8), is_extended_id=False))


def _run_state(bus, node, state, *, timeout, accepted_states):
    """Request one calibration state and monitor entry then IDLE completion."""

    send_state(bus, node, state)
    t0, seen, states = time.time(), False, []
    while time.time() - t0 < timeout:
        r = heartbeat(bus, node, 2.0)
        if r is None:
            continue
        err, current = r
        if current not in states:
            states.append(current)
        if current in accepted_states:
            seen = True
        if seen and current == S_IDLE:
            return err == 0, True, states
        if not seen and (time.time() - t0) > 8:
            return False, False, states
        time.sleep(0.05)
    return False, seen, states


def calibrate(bus, node, timeout=90.0):
    """Calibrate one axis, retaining the fw 0.5.1 split-state fallback."""

    clear_errors(bus, node)
    time.sleep(0.3)
    drain(bus)
    r = heartbeat(bus, node, 2.0)
    print("  node %-2d 시작 err=%s state=%s → FULL_CAL(3)"
          % (node, hex(r[0]) if r else "?", r[1] if r else "?"))
    started = time.time()
    ok, entered, states = _run_state(
        bus, node, S_FULL_CAL, timeout=timeout, accepted_states=CAL_STATES
    )
    if entered:
        print("     완료 (%.0fs) 전이=%s → %s"
              % (time.time() - started, states, "OK" if ok else "FAIL"))
        return ok

    print("     FULL_CAL(3) 진입 거부/무응답 전이=%s → 분리 캘리" % states)
    clear_errors(bus, node)
    time.sleep(0.3)
    drain(bus)

    remaining = max(0.0, timeout - (time.time() - started))
    motor_ok, motor_entered, motor_states = _run_state(
        bus,
        node,
        S_MOTOR_CAL,
        timeout=remaining,
        accepted_states=(S_MOTOR_CAL,),
    )
    if not (motor_entered and motor_ok):
        print("     MOTOR_CAL(4) 실패 전이=%s" % motor_states)
        return False

    remaining = max(0.0, timeout - (time.time() - started))
    offset_ok, offset_entered, offset_states = _run_state(
        bus,
        node,
        S_OFFSET_CAL,
        timeout=remaining,
        accepted_states=(S_OFFSET_CAL,),
    )
    ok = offset_entered and offset_ok
    print("     분리 완료 (%.0fs) motor=%s offset=%s → %s"
          % (time.time() - started, motor_states, offset_states, "OK" if ok else "FAIL"))
    return ok


def calibrate_nodes(bus, nodes, *, observe=None):
    """Calibrate nodes sequentially on an injected bus and return per-node results."""

    results = {}
    for node in nodes:
        ok = calibrate(bus, node)
        results[node] = ok
        if observe is not None:
            observe(node, ok)
        time.sleep(0.6)
    return results


def main():
    import can

    ap = argparse.ArgumentParser(description="구동 ODrive 다축 CAN 풀캘리")
    ap.add_argument("--nodes", type=int, nargs="+", default=DRIVE_NODES,
                    help="캘리할 CAN node id (기본 11~16)")
    ap.add_argument("--channel", default="can0")
    args = ap.parse_args()

    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from chassis.runtime_lock import RealCanSession

    with RealCanSession(channel=args.channel, owner="can_calibrate_all"):
        bus = can.Bus(channel=args.channel, interface="socketcan")
        try:
            print("=== 구동 %d축 CAN 풀캘리 (순차, 각 ~55s) — 바퀴 자유 필수 ===" % len(args.nodes))
            res = calibrate_nodes(bus, args.nodes)
        finally:
            bus.shutdown()

    ok = [n for n in args.nodes if res[n]]
    bad = [n for n in args.nodes if not res[n]]
    print("\n===== 결과: 성공 %d/%d  실패: %s =====" % (len(ok), len(args.nodes), bad if bad else "없음"))
    return 0 if not bad else 1


if __name__ == "__main__":
    raise SystemExit(main())
