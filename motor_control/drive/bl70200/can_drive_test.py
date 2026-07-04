#!/usr/bin/env python3
"""구동 6축(node 11~16) CAN 동시 주행 브링업 테스트.

can0 로 6축 전부 속도제어(VELOCITY/VEL_RAMP) arm →
  ① 전진 동시 +1.0 rev/s
  ② 제자리선회: 좌(11/13/15)=+1.0 / 우(12/14/16)=−1.0
  ③ 정지 →0
각 축 RTR(Get_Encoder_Estimates cmd 0x09)로 실제 vel 읽어 추종 확인 + heartbeat err 감시.
CANSimple: Set_Controller_Mode(0x0B)=<ii(2,2), Set_Input_Vel(0x0D)=<ff, Set_Axis_State(0x07).
좌/우 매핑은 chassis.ChassisManager DEFAULT_WHEEL_MAP 기준(앞좌11 앞우12 중좌13 중우14 뒤좌15 뒤우16).

⚠️ 바퀴 자유(무부하) 전제. 종료 시 try/finally 로 전 축 IDLE(런어웨이 방지).
사전: 6축 캘리(can_calibrate_all.py) + can0 500k UP. 컨테이너에서 실행.

실행: python3 motor_control/drive/bl70200/can_drive_test.py [--speed 1.0]
"""
import argparse
import struct
import time

import can

CMD_HEARTBEAT, CMD_SET_STATE = 0x01, 0x07
CMD_SET_CTRL_MODE, CMD_SET_INPUT_VEL, CMD_GET_ENC_EST = 0x0B, 0x0D, 0x09
CMD_CLEAR_ERRORS = 0x18
S_IDLE, S_CLOSED_LOOP = 1, 8
CTRL_VELOCITY, INPUT_VEL_RAMP = 2, 2
NODES = [11, 12, 13, 14, 15, 16]
LEFT, RIGHT = [11, 13, 15], [12, 14, 16]


def arb(n, c):
    return (n << 5) | c


def drain(bus):
    while bus.recv(timeout=0.0) is not None:
        pass


def send(bus, n, c, data):
    bus.send(can.Message(arbitration_id=arb(n, c), data=data, is_extended_id=False))


def set_mode(bus, n):
    send(bus, n, CMD_SET_CTRL_MODE, struct.pack("<ii", CTRL_VELOCITY, INPUT_VEL_RAMP))


def set_vel(bus, n, v):
    send(bus, n, CMD_SET_INPUT_VEL, struct.pack("<ff", v, 0.0))


def set_state(bus, n, s):
    send(bus, n, CMD_SET_STATE, struct.pack("<I", s) + bytes(4))


def clear_errors(bus, n):
    send(bus, n, CMD_CLEAR_ERRORS, bytes(8))


def heartbeat(bus, n, timeout=1.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        m = bus.recv(timeout=timeout)
        if m is None:
            return None
        if (not m.is_extended_id and (m.arbitration_id >> 5) == n
                and (m.arbitration_id & 0x1F) == CMD_HEARTBEAT and len(m.data) >= 5):
            return struct.unpack("<I", m.data[0:4])[0], m.data[4]
    return None


def get_vel(bus, n, timeout=0.4):
    bus.send(can.Message(arbitration_id=arb(n, CMD_GET_ENC_EST), is_remote_frame=True, is_extended_id=False))
    t0 = time.time()
    while time.time() - t0 < timeout:
        m = bus.recv(timeout=timeout)
        if m is None:
            return None
        if (not m.is_extended_id and m.arbitration_id == arb(n, CMD_GET_ENC_EST)
                and not m.is_remote_frame and len(m.data) >= 8):
            return struct.unpack("<ff", m.data[0:8])[1]
    return None


def arm_check(bus, n):
    """set_state 직후 전이 지연 감안: 버퍼 비우고 최신 heartbeat 재조회."""
    drain(bus)
    r = None
    for _ in range(4):
        r = heartbeat(bus, n, 0.5)
        if r and r[1] == S_CLOSED_LOOP:
            return r
    return r


def readall(bus, label):
    time.sleep(0.1)
    vels = {n: get_vel(bus, n) for n in NODES}
    errs = {n: (heartbeat(bus, n, 0.6) or (None, None))[0] for n in NODES}
    print("  [%s]" % label)
    for n in NODES:
        v, e = vels[n], errs[n]
        print("     node %-2d vel=%-7s err=%s" % (n, ("%+.3f" % v) if v is not None else "?",
              hex(e) if e is not None else "?"))


def main():
    ap = argparse.ArgumentParser(description="구동 6축 CAN 동시 주행 테스트")
    ap.add_argument("--speed", type=float, default=1.0, help="테스트 속도 rev/s (기본 1.0)")
    ap.add_argument("--channel", default="can0")
    args = ap.parse_args()
    sp = args.speed

    bus = can.Bus(channel=args.channel, interface="socketcan")
    print("셋업 + arm(폐루프)...")
    for n in NODES:
        clear_errors(bus, n)
        set_mode(bus, n)
        set_vel(bus, n, 0.0)
    time.sleep(0.2)
    for n in NODES:
        set_state(bus, n, S_CLOSED_LOOP)
    time.sleep(0.5)
    armed = {}
    for n in NODES:
        r = arm_check(bus, n)
        armed[n] = bool(r and r[1] == S_CLOSED_LOOP)
        print("  node %-2d state=%s err=%s %s" % (n, r[1] if r else "?", hex(r[0]) if r else "?",
              "arm" if armed[n] else "FAIL"))
    n_arm = sum(armed.values())
    print("  → %d/%d arm" % (n_arm, len(NODES)))

    try:
        print("\n① 전진 동시 +%.1f rev/s (2s)" % sp)
        for n in NODES:
            set_vel(bus, n, sp)
        time.sleep(2.0)
        readall(bus, "전진")

        print("\n② 제자리선회: 좌(11/13/15)=+%.1f / 우(12/14/16)=−%.1f (2s)" % (sp, sp))
        for n in LEFT:
            set_vel(bus, n, sp)
        for n in RIGHT:
            set_vel(bus, n, -sp)
        time.sleep(2.0)
        readall(bus, "선회")

        print("\n③ 정지 →0 (1.5s)")
        for n in NODES:
            set_vel(bus, n, 0.0)
        time.sleep(1.5)
        readall(bus, "정지")
    finally:
        for n in NODES:
            set_vel(bus, n, 0.0)
            set_state(bus, n, S_IDLE)
        bus.shutdown()
        print("\n전 축 IDLE (안전정지)")


if __name__ == "__main__":
    main()
