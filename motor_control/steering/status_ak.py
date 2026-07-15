"""AK status 프레임을 3초 동안 확인하는 실물 CAN 도구."""

from pathlib import Path
import struct
import sys
import time

import can


MID = 10


def main():
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from chassis.runtime_lock import RealCanSession

    with RealCanSession(channel="can0", owner="status_ak"):
        bus = can.interface.Bus(channel="can0", interface="socketcan")
        try:
            print("send rpm=0 + listen 3s...")
            t_end = time.time() + 3.0
            last_send = 0.0
            while time.time() < t_end:
                if time.time() - last_send > 0.05:
                    ext_id = (3 << 8) | MID
                    bus.send(
                        can.Message(
                            arbitration_id=ext_id,
                            data=struct.pack(">i", 0),
                            is_extended_id=True,
                        )
                    )
                    last_send = time.time()

                message = bus.recv(timeout=0.01)
                if message is None:
                    continue
                packet = (message.arbitration_id >> 8) & 0xFF
                node = message.arbitration_id & 0xFF
                print(
                    f"RX ext_id=0x{message.arbitration_id:08X}  "
                    f"pkt={packet}  node={node}  len={message.dlc}  "
                    f"data={message.data.hex()}"
                )
        finally:
            bus.shutdown()


if __name__ == "__main__":
    main()
