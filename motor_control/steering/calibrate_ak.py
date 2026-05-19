import can, struct, time

GEAR=1.0; POLES=14.0; MID=10; PKT_STATUS=41

bus = can.interface.Bus(channel='can0', interface='socketcan')

def send_pos_raw(deg, spd=800, acc=4000):
    ext = (6<<8)|MID
    data = struct.pack(">ihh", int(deg*10000), spd, acc)
    bus.send(can.Message(arbitration_id=ext, data=data, is_extended_id=True))

def send_rpm(out_rpm):
    ext = (3<<8)|MID
    data = struct.pack(">i", int(-out_rpm*GEAR*POLES))
    bus.send(can.Message(arbitration_id=ext, data=data, is_extended_id=True))

def origin():
    ext=(5<<8)|MID
    bus.send(can.Message(arbitration_id=ext, data=bytes([1]), is_extended_id=True))

def read_pos(timeout=0.5):
    t_end=time.time()+timeout; latest=None
    while time.time()<t_end:
        m=bus.recv(timeout=0.05)
        if m and ((m.arbitration_id>>8)&0xFF)==PKT_STATUS and (m.arbitration_id&0xFF)==MID:
            latest=m.data
    if latest is None: return None
    return struct.unpack(">h", latest[:2])[0]/10.0

send_rpm(0); time.sleep(0.1)
origin();    time.sleep(0.2)
print(f"origin pos = {read_pos():.1f}°")

input("\n>>> 출력축 마크 위치 표시하고 엔터 ")

TARGET_RAW = 36.0
for _ in range(60):
    send_pos_raw(TARGET_RAW, spd=800, acc=4000)
    time.sleep(0.05)
time.sleep(0.5)

print(f"\nstatus pos (cmd 36.0°) = {read_pos():.2f}°")
print(">>> 출력축 실제 회전량은?")
print("    ~3.6°  → 단위 = 모터축 (GEAR=10.0 유지)")
print("    ~36°   → 단위 = 출력축 (GEAR=1.0 로 변경)")

send_rpm(0); time.sleep(0.1)
bus.shutdown()
