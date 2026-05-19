import can, struct, time

MID = 10
bus = can.interface.Bus(channel='can0', interface='socketcan')

# RPM 0을 주기적으로 흘리면서 응답 listen
print("send rpm=0 + listen 3s...")
t_end = time.time() + 3.0
last_send = 0.0
while time.time() < t_end:
    if time.time() - last_send > 0.05:
        ext_id = (3 << 8) | MID
        bus.send(can.Message(arbitration_id=ext_id,
                             data=struct.pack(">i", 0),
                             is_extended_id=True))
        last_send = time.time()

    msg = bus.recv(timeout=0.01)
    if msg is None:
        continue
    pkt = (msg.arbitration_id >> 8) & 0xFF
    nid =  msg.arbitration_id        & 0xFF
    print(f"RX ext_id=0x{msg.arbitration_id:08X}  pkt={pkt}  node={nid}  "
          f"len={msg.dlc}  data={msg.data.hex()}")

bus.shutdown()
