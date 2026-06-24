import time
import odrive
from odrive.enums import *

GEAR_RATIO = 5.0
TARGET_VEL = 8.0

drv = odrive.find_any()
ax = drv.axis1

print(f"vbus: {drv.vbus_voltage:.2f}V, brake_armed: {drv.brake_resistor_armed}")
print(
    f"is_calibrated: motor={ax.motor.is_calibrated}, encoder.is_ready={ax.encoder.is_ready}")

if not (ax.motor.is_calibrated and ax.encoder.is_ready):
    print("캘리 먼저")
    exit()

ax.error = 0
ax.motor.error = 0
ax.encoder.error = 0
ax.controller.error = 0

ax.motor.config.current_lim = 15.0
ax.controller.config.vel_limit = 50.0

# === 노이즈 억제 강화 ===
ax.encoder.config.bandwidth = 20         # 50 → 20 (vel_estimate 필터 강화)
ax.controller.config.vel_gain = 0.05     # 0.05 → 0.015 (1/3로)
ax.controller.config.vel_integrator_gain = 0.1  # 0.2 → 0.05 (적분기 약화)

ax.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
ax.controller.config.input_mode = InputMode.VEL_RAMP
ax.controller.config.vel_ramp_rate = 5.0

ax.controller.input_vel = 0.0
ax.requested_state = AxisState.CLOSED_LOOP_CONTROL
time.sleep(0.5)

if ax.current_state != 8:
    print(f"진입 실패: motor={ax.motor.error:#x}")
    exit()

print(f"\n타겟: 모터 {TARGET_VEL:+.2f} rev/s")
print("Ctrl+C 로 정지\n")

ax.controller.input_vel = TARGET_VEL

try:
    t0 = time.time()
    samples = []
    while True:
        elapsed = time.time() - t0
        vel = ax.encoder.vel_estimate
        pos = ax.encoder.pos_estimate
        iq = ax.motor.current_control.Iq_measured
        samples.append((elapsed, vel, iq))
        # 정상상태 통계 (시작 2초 이후)
        recent = [s for s in samples if elapsed - s[0] < 3 and s[0] > 2]
        if recent:
            vels = [s[1] for s in recent]
            avg = sum(vels) / len(vels)
            rms = (sum((v - avg)**2 for v in vels) / len(vels)) ** 0.5
            stat = f" avg={avg:+.2f} rms={rms:.2f}"
        else:
            stat = ""
        print(
            f"  t={elapsed:5.1f}s vel={vel:+5.2f} (출력 {vel/GEAR_RATIO:+.2f}) Iq={iq:+5.2f}A{stat}")

        if ax.error != 0:
            print(
                f"\n트립: motor={ax.motor.error:#x}, ctrl={ax.controller.error:#x}")
            break
        if ax.current_state != 8:
            break
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\n감속...")

ax.controller.input_vel = 0.0
for _ in range(20):
    if abs(ax.encoder.vel_estimate) < 0.1:
        break
    time.sleep(0.3)

ax.requested_state = AxisState.IDLE
print("IDLE")
