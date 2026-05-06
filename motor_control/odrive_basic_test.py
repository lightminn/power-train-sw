import time
import odrive
from odrive.enums import *

GEAR_RATIO = 5.0

drv = odrive.find_any()
ax = drv.axis1

print(f"vbus: {drv.vbus_voltage:.2f}V, brake_armed: {drv.brake_resistor_armed}")
print(f"is_calibrated: motor={ax.motor.is_calibrated}, encoder.is_ready={ax.encoder.is_ready}")

if not (ax.motor.is_calibrated and ax.encoder.is_ready):
    print("캘리 먼저 (odrive_calib.py 실행)")
    exit()

ax.error = 0
ax.motor.error = 0
ax.encoder.error = 0
ax.controller.error = 0

# 위치 제어용 게인 — 캘리 스크립트의 POS_FILTER 설정 override
ax.motor.config.current_lim = 15.0
ax.controller.config.vel_limit = 5.0

ax.encoder.config.bandwidth = 50
ax.controller.config.pos_gain = 3.0
ax.controller.config.vel_gain = 0.04
ax.controller.config.vel_integrator_gain = 0.0

ax.trap_traj.config.vel_limit = 5.0
ax.trap_traj.config.accel_limit = 10.0
ax.trap_traj.config.decel_limit = 10.0

ax.controller.config.control_mode = ControlMode.POSITION_CONTROL
ax.controller.config.input_mode = InputMode.TRAP_TRAJ

ax.requested_state = AxisState.CLOSED_LOOP_CONTROL
time.sleep(0.5)
print(f"state: {ax.current_state}, err: {ax.error:#x}")
if ax.current_state != 8:
    print(f"motor: {ax.motor.error:#x}, ctrl: {ax.controller.error:#x}, enc: {ax.encoder.error:#x}")
    exit()

start_pos = ax.encoder.pos_estimate
ax.controller.input_pos = start_pos
time.sleep(0.5)
print(f"원점: {start_pos:.2f} (모터 turns)")


def move_to(target_motor_turns, label):
    target = start_pos + target_motor_turns
    print(f"\n=== {label}: 모터 {target_motor_turns:+.1f} turns (출력 {target_motor_turns/GEAR_RATIO:+.2f}) ===")
    ax.controller.input_pos = target

    t0 = time.time()
    settled_count = 0
    while True:
        pos = ax.encoder.pos_estimate
        vel = ax.encoder.vel_estimate
        iq = ax.motor.current_control.Iq_measured
        elapsed = time.time() - t0
        err_pos = pos - target
        print(f"  t={elapsed:4.1f}s pos={pos-start_pos:+7.2f} (err={err_pos:+5.2f}) vel={vel:+5.2f} Iq={iq:+5.2f}A")

        if abs(err_pos) < 0.3 and abs(vel) < 0.5:
            settled_count += 1
            if settled_count >= 3:
                print(f"  도달 ({elapsed:.1f}s)")
                return True
        else:
            settled_count = 0

        if elapsed > 10:
            print("  타임아웃")
            return False
        if ax.error != 0:
            print(f"  트립: axis={ax.error:#x}, motor={ax.motor.error:#x}, ctrl={ax.controller.error:#x}")
            return False
        if ax.current_state != 8:
            print(f"  폐루프 이탈: state={ax.current_state}")
            return False
        time.sleep(0.3)


move_to(5.0,  "1단계 +5")
time.sleep(0.5)
move_to(-5.0, "2단계 -5")
time.sleep(0.5)
move_to(10.0, "3단계 +10")
time.sleep(0.5)
move_to(0.0,  "4단계 원점")

time.sleep(1.0)
ax.requested_state = AxisState.IDLE
print("\nIDLE")
