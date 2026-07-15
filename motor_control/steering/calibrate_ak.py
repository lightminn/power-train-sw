"""AK 출력축 위치 단위 확인용 실물 CAN 도구."""

from pathlib import Path
import struct
import sys
import time

import can


GEAR = 1.0
POLES = 14.0
MID = 10
PKT_STATUS = 41


def send_pos_raw(bus, deg, spd=800, acc=4000):
    ext = (6 << 8) | MID
    data = struct.pack(">ihh", int(deg * 10000), spd, acc)
    bus.send(can.Message(arbitration_id=ext, data=data, is_extended_id=True))


def send_rpm(bus, out_rpm):
    ext = (3 << 8) | MID
    data = struct.pack(">i", int(-out_rpm * GEAR * POLES))
    bus.send(can.Message(arbitration_id=ext, data=data, is_extended_id=True))


def origin(bus):
    ext = (5 << 8) | MID
    bus.send(can.Message(arbitration_id=ext, data=bytes([1]), is_extended_id=True))


def read_pos(bus, timeout=0.5):
    t_end = time.time() + timeout
    latest = None
    while time.time() < t_end:
        message = bus.recv(timeout=0.05)
        if (
            message
            and ((message.arbitration_id >> 8) & 0xFF) == PKT_STATUS
            and (message.arbitration_id & 0xFF) == MID
        ):
            latest = message.data
    if latest is None:
        return None
    return struct.unpack(">h", latest[:2])[0] / 10.0


def main():
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from chassis.runtime_lock import RealCanSession

    with RealCanSession(channel="can0", owner="calibrate_ak"):
        bus = can.interface.Bus(channel="can0", interface="socketcan")
        try:
            send_rpm(bus, 0)
            time.sleep(0.1)
            origin(bus)
            time.sleep(0.2)
            print(f"origin pos = {read_pos(bus):.1f}°")

            input("\n>>> 출력축 마크 위치 표시하고 엔터 ")

            target_raw = 36.0
            for _ in range(60):
                send_pos_raw(bus, target_raw, spd=800, acc=4000)
                time.sleep(0.05)
            time.sleep(0.5)

            print(f"\nstatus pos (cmd 36.0°) = {read_pos(bus):.2f}°")
            print(">>> 출력축 실제 회전량은?")
            print("    ~3.6°  → 단위 = 모터축 (GEAR=10.0 유지)")
            print("    ~36°   → 단위 = 출력축 (GEAR=1.0 로 변경)")
        finally:
            try:
                send_rpm(bus, 0)
                time.sleep(0.1)
            finally:
                bus.shutdown()


if __name__ == "__main__":
    main()
