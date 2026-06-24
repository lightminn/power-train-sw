#!/usr/bin/env python3
"""AK + ODrive 동시 CAN 제어 데모 (단일 500 kbps can0 버스).

ODrive(구동, node 11, CANSimple 표준 프레임)와 AK45-36 ×2(조향, id 1·2,
VESC 확장 프레임)를 한 버스에서 동시에 독립 제어한다. 단일 Bus + 단일 수신
드레인으로 받아 is_extended_id 로 분기 — 디바이스마다 따로 recv 하면 서로
프레임을 삼키므로 금지.

사전:
  1) can0 500k 올리기 (호스트):   bash scripts/can_setup.sh
  2) ODrive 캘리: 처음 1회 --calibrate (출력축 자유, 모터 ~55s 회전)

실행 (Jetson 컨테이너 /workspace):
  python3 motor_control/can_ak_odrive_demo.py              # 캘리됐다고 가정, 데모만
  python3 motor_control/can_ak_odrive_demo.py --calibrate  # 캘리부터 1회

문서: Notion "AK + ODrive 동시 CAN 제어 (단일 500k 버스)".
"""
import argparse
import math
import struct
import time

import can

ON = 11  # ODrive node id (axis1, 구동)


def main(calibrate: bool) -> None:
    bus = can.interface.Bus(channel="can0", interface="socketcan")  # 필터 없음 → 표준·확장 모두 수신

    # ----- ODrive (node 11, 표준 프레임) -----
    def o_send(cmd, data=b""):
        bus.send(can.Message(arbitration_id=(ON << 5) | cmd, data=data, is_extended_id=False))

    def o_rtr(cmd):
        bus.send(can.Message(arbitration_id=(ON << 5) | cmd, is_remote_frame=True, is_extended_id=False))

    # ----- AK (id 1·2, 확장 프레임) -----
    def ak(mid, pkt, data):
        bus.send(can.Message(arbitration_id=(pkt << 8) | mid, data=data, is_extended_id=True))

    def ak_pos(mid, deg, spd=2000, acc=8000):  # 출력축 deg, spd/acc 는 ÷10 전송
        ak(mid, 6, struct.pack(">ihh", int(deg * 10000), int(spd / 10), int(acc / 10)))

    def ak_rpm0(mid):
        ak(mid, 3, struct.pack(">i", 0))  # 정지/모닝콜 (brake·current 는 이 펌웨어서 폭주 → 금지)

    def ak_origin(mid):
        ak(mid, 5, bytes([0x01]))  # 현재 위치를 0 으로

    # ----- 단일 수신 디스패치 -----
    st = {"ovel": 0.0, "oerr": 0, 1: {"pos": None, "fault": 0}, 2: {"pos": None, "fault": 0}}

    def dispatch(m):
        if m.is_extended_id:  # AK (확장)
            pkt = (m.arbitration_id >> 8) & 0xFF
            nid = m.arbitration_id & 0xFF
            if pkt == 41 and nid in (1, 2) and len(m.data) >= 8:  # STATUS_1
                p, s, c, t, f = struct.unpack(">hhhbb", m.data[:8])
                st[nid].update(pos=p / 10.0, fault=f)
        else:  # ODrive (표준)
            if m.arbitration_id == (ON << 5) | 0x01:  # heartbeat
                st["oerr"] = struct.unpack("<I", m.data[:4])[0]
            elif m.arbitration_id == (ON << 5) | 0x09:  # encoder estimates
                st["ovel"] = struct.unpack("<ff", m.data)[1]

    def pump(dur):
        t = time.time()
        while time.time() - t < dur:
            m = bus.recv(timeout=0.004)
            if m:
                dispatch(m)

    def wait_idle(timeout):  # 캘리 시작(state≠1) 후 IDLE(1) 복귀까지 대기
        t = time.time()
        started = False
        while time.time() - t < timeout:
            m = bus.recv(timeout=0.2)
            if m and not m.is_extended_id and m.arbitration_id == (ON << 5) | 0x01:
                if m.data[4] != 1:
                    started = True
                elif started:
                    return

    try:
        if calibrate:  # ⚠️ 출력축 자유 필수, 모터 ~55s 회전. CAN 은 FULL_CAL(6) 거부 → 4→7 분리
            o_send(0x18)
            time.sleep(0.3)
            o_send(0x07, struct.pack("<i", 4))  # MOTOR_CAL
            time.sleep(1.5)
            wait_idle(25)
            o_send(0x07, struct.pack("<i", 7))  # OFFSET_CAL (~55s)
            time.sleep(1.5)
            wait_idle(90)
            print("캘리 완료")

        # ----- 준비 -----
        o_send(0x18)
        time.sleep(0.1)  # clear_errors
        o_send(0x0F, struct.pack("<ff", 50.0, 15.0))  # Set_Limits (vel, current)
        o_send(0x1B, struct.pack("<ff", 0.06, 0.2))  # Set_Vel_Gains (BL70200 HALL 최적)
        o_send(0x0B, struct.pack("<ii", 2, 2))  # VELOCITY, VEL_RAMP
        o_send(0x07, struct.pack("<i", 8))  # CLOSED_LOOP
        pump(0.8)
        for mid in (1, 2):  # AK 모닝콜 + 영점
            for _ in range(3):
                ak_rpm0(mid)
                time.sleep(0.03)
            ak_origin(mid)
            time.sleep(0.2)
        pump(0.4)

        # ----- 동시 구동 (25 Hz, 10초): ODrive 1.0 rev/s + AK ±30° 사인(역위상) -----
        t0 = time.time()
        while time.time() - t0 < 10.0:
            ph = 2 * math.pi * (time.time() - t0) / 4.0
            o_send(0x0D, struct.pack("<ff", 1.0, 0.0))  # ODrive 속도 1.0 rev/s
            ak_pos(1, 30 * math.sin(ph))
            ak_pos(2, -30 * math.sin(ph))
            o_rtr(0x09)  # ODrive 피드백 요청
            pump(1 / 25)  # ≥20 Hz → AK 워치독 유지
            print(
                "\rODrive %.2f rev/s | AK1 %+5.0f° AK2 %+5.0f°"
                % (st["ovel"], st[1]["pos"] or 0, st[2]["pos"] or 0),
                end="",
            )
            if st["oerr"] or st[1]["fault"] or st[2]["fault"]:  # 안전
                print("\n안전정지 oerr=%s f1=%s f2=%s" % (hex(st["oerr"]), st[1]["fault"], st[2]["fault"]))
                break
    finally:
        # ----- 정지 (필수): 런어웨이 방지 -----
        o_send(0x07, struct.pack("<i", 1))  # ODrive IDLE
        for _ in range(5):
            ak_rpm0(1)
            ak_rpm0(2)
            time.sleep(0.04)
        bus.shutdown()
        print("\n정지 완료")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="AK + ODrive 동시 CAN 데모")
    ap.add_argument("--calibrate", action="store_true", help="데모 전 ODrive 캘리 1회 (출력축 자유 필수)")
    main(ap.parse_args().calibrate)
