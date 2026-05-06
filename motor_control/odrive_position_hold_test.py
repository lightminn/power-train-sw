import time
import odrive
from odrive.enums import *

GEAR_RATIO = 5.0

drv = odrive.find_any()
ax = drv.axis0

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

# 검증된 게인
ax.motor.config.current_lim = 15.0
ax.controller.config.vel_limit = 5.0
ax.encoder.config.bandwidth = 50
ax.controller.config.pos_gain = 3.0
ax.controller.config.vel_gain = 0.04
ax.controller.config.vel_integrator_gain = 0.0

ax.controller.config.control_mode = ControlMode.POSITION_CONTROL
ax.controller.config.input_mode = InputMode.PASSTHROUGH

ax.requested_state = AxisState.CLOSED_LOOP_CONTROL
time.sleep(0.5)
if ax.current_state != 8:
    print(f"진입 실패: motor={ax.motor.error:#x}")
    exit()

# 현재 위치 = 홀딩 타겟
hold_pos = ax.encoder.pos_estimate
ax.controller.input_pos = hold_pos
time.sleep(0.5)
print(f"\n홀딩 타겟: {hold_pos:.3f} turns")
print("=" * 70)
print("외력 테스트: 출력단을 손으로 살짝 밀거나 돌려라. 자동 복귀해야 함.")
print("=" * 70)

# 통계 변수
max_dev = 0.0
max_iq = 0.0
samples = []

DURATION = 30.0   # 30초 모니터
t0 = time.time()
last_print = 0

while time.time() - t0 < DURATION:
    elapsed = time.time() - t0
    pos = ax.encoder.pos_estimate
    vel = ax.encoder.vel_estimate
    iq = ax.motor.current_control.Iq_measured
    dev = pos - hold_pos

    samples.append((elapsed, dev, vel, iq))
    if abs(dev) > abs(max_dev):
        max_dev = dev
    if abs(iq) > abs(max_iq):
        max_iq = iq

    # 0.2초마다 print
    if elapsed - last_print >= 0.2:
        bar_len = min(int(abs(dev) * 20), 30)
        bar = ('<' if dev < 0 else '>') * bar_len
        print(
            f"  t={elapsed:5.1f}s dev={dev:+6.3f} vel={vel:+5.2f} Iq={iq:+5.2f}A  {bar}")
        last_print = elapsed

    if ax.error != 0:
        print(
            f"\n트립! axis={ax.error:#x}, motor={ax.motor.error:#x}, ctrl={ax.controller.error:#x}")
        break
    if ax.current_state != 8:
        print(f"\n폐루프 이탈: state={ax.current_state}")
        break

    time.sleep(0.05)

# 결과 분석
final_pos = ax.encoder.pos_estimate
final_dev = final_pos - hold_pos
print("\n" + "=" * 70)
print(f"테스트 종료 ({DURATION}s)")
print(f"  최종 편차:    {final_dev:+.3f} turns")
print(f"  최대 편차:    {max_dev:+.3f} turns")
print(f"  최대 Iq:      {max_iq:+.2f} A")
print(f"  샘플 수:      {len(samples)}")

# 정상상태 평균/표준편차 (마지막 3초)
recent = [s for s in samples if s[0] > DURATION - 3]
if recent:
    devs = [s[1] for s in recent]
    iqs = [abs(s[3]) for s in recent]
    avg_dev = sum(devs) / len(devs)
    rms_dev = (sum((d - avg_dev) ** 2 for d in devs) / len(devs)) ** 0.5
    avg_iq = sum(iqs) / len(iqs)
    print(f"\n  마지막 3초 (정상상태):")
    print(f"    평균 편차:  {avg_dev:+.3f} turns")
    print(f"    RMS 편차:   {rms_dev:.3f} turns")
    print(f"    평균 |Iq|:  {avg_iq:.2f} A")

ax.requested_state = AxisState.IDLE
print("\nIDLE")
