import time, odrive
from odrive.enums import *

drv = odrive.find_any()
ax = drv.axis1

if not (ax.motor.is_calibrated and ax.encoder.is_ready):
    print("캘리 필요"); exit()

ax.error = 0; ax.motor.error = 0; ax.encoder.error = 0; ax.controller.error = 0

ax.requested_state = AxisState.CLOSED_LOOP_CONTROL
time.sleep(0.5)
print(f"state: {ax.current_state}, err: {ax.error:#x}")

if ax.current_state != 8:
    print(f"진입 실패: motor={ax.motor.error:#x}")
    exit()

start = ax.encoder.pos_estimate
print(f"원점: {start:.2f}")

# 2바퀴 회전
ax.controller.input_pos = start + 2.0
for i in range(20):
    pos = ax.encoder.pos_estimate
    print(f"  pos={pos-start:+.2f} vel={ax.encoder.vel_estimate:+.2f} Iq={ax.motor.current_control.Iq_measured:+.2f}A")
    if abs(pos - (start + 2.0)) < 0.2:
        break
    time.sleep(0.3)

time.sleep(1)
ax.requested_state = AxisState.IDLE
print("IDLE")
