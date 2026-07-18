"""⛔ DEPRECATED — 아카이브. 실행 금지. 정본은 ../bl70200_setup.py / ../can_drive_test.py.

이 스크립트는 BL70200 실측값과 다른 POLE_PAIRS=5 (cpr=6*5=30) 를 쓰고
encoder 설정을 NVM 에 저장한다 (실측 정본 pp=10 / cpr=60). 그 밖에도
캘리 단계마다 무제한 IDLE 대기, 폐루프 잔존, 정본 NVM 이 VELOCITY 모드인데
POSITION 모드로 바꾸지 않고 input_pos 를 쓴 뒤 무한 대기하는 결함이 있다.

2026-07-18 적대적 코드리뷰에서 확정되어 아카이브로 이동했다. 이력 참조용으로만 보존한다.
"""
import sys

sys.exit(
    "⛔ 이 스크립트는 아카이브됨 — 실행 금지.\n"
    "   pp=5 / cpr=30 을 NVM 에 저장해 BL70200 보드를 손상시킨다.\n"
    "   정본: bl70200_setup.py (셋업/캘리) · can_drive_test.py (다축 주행)"
)

import time
import odrive
from odrive.enums import *


def connect():
    print("ODrive 검색 중... (USB가 연결되어 있는지 확인하세요)")
    drv = odrive.find_any()
    print("ODrive 연결 성공! 보드 시리얼:", drv.serial_number)
    return drv


def dump_errors(drv):
    a = drv.axis1
    print(f"  axis.error:       {a.error}")
    print(f"  motor.error:      {a.motor.error}")
    print(f"  encoder.error:    {a.encoder.error}")
    print(f"  controller.error: {a.controller.error}")


my_drive = connect()
axis = my_drive.axis1

print(f"펌웨어: {my_drive.fw_version_major}.{my_drive.fw_version_minor}.{my_drive.fw_version_revision}")
print(f"하드웨어: v{my_drive.hw_version_major}.{my_drive.hw_version_minor} variant {my_drive.hw_version_variant}")

vbus = my_drive.vbus_voltage

print("gpio9:", my_drive.config.gpio9_mode)
print("gpio10:", my_drive.config.gpio10_mode)
print("gpio11:", my_drive.config.gpio11_mode)


print(f"DC 전원 전압: {vbus:.2f} V")
if vbus < 8.0:
    print("DC 전원이 연결되지 않았습니다.")
    exit()

POLE_PAIRS = 5

if axis.encoder.config.mode != EncoderMode.HALL:
    print("초기 설정 저장 중... (ODrive 재부팅)")
    axis.encoder.config.mode = EncoderMode.HALL
    axis.encoder.config.cpr = 6 * POLE_PAIRS
    axis.encoder.config.bandwidth = 100
    axis.motor.config.motor_type = MotorType.HIGH_CURRENT
    axis.motor.config.pole_pairs = POLE_PAIRS
    axis.motor.config.calibration_current = 4.0
    axis.motor.config.current_lim = 9.0
    my_drive.save_configuration()
    time.sleep(5)
    my_drive = connect()
    axis = my_drive.axis1
    print(
        f"재부팅 완료 — mode: {axis.encoder.config.mode}, cpr: {axis.encoder.config.cpr}")
else:
    print(f"인코더 모드: HALL, cpr: {axis.encoder.config.cpr}")

axis.error = 0
axis.motor.error = 0
axis.encoder.error = 0

# 1. 모터 캘리브레이션
print("모터 캘리브레이션 시작...")
axis.requested_state = AxisState.MOTOR_CALIBRATION
while axis.current_state != AxisState.IDLE:
    time.sleep(0.1)

if axis.motor.error != 0 or not axis.motor.is_calibrated:
    print(f"모터 캘리브레이션 실패! error: {axis.motor.error}")
    exit()
print("모터 캘리브레이션 완료")

# 2. 홀센서 폴라리티 캘리브레이션
print("홀센서 폴라리티 캘리브레이션 시작...")
axis.requested_state = AxisState.ENCODER_HALL_POLARITY_CALIBRATION
while axis.current_state != AxisState.IDLE:
    time.sleep(0.1)

if axis.encoder.error != 0:
    print(f"폴라리티 캘리브레이션 실패! error: {axis.encoder.error}")
    exit()
print("폴라리티 캘리브레이션 완료")

# 3. 오프셋 캘리브레이션 (calib_range 넓혀서 CPR 불일치 통과)
axis.encoder.config.calib_scan_distance = 150.0
axis.encoder.config.calib_range = 0.5   # 50% 허용 → CPR_POLEPAIRS_MISMATCH 억제
axis.encoder.error = 0
print("오프셋 캘리브레이션 시작... (모터 회전)")
axis.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
while axis.current_state != AxisState.IDLE:
    time.sleep(0.1)

print(
    f"오프셋 완료 — is_ready: {axis.encoder.is_ready}, error: {axis.encoder.error}, hall_state: {axis.encoder.hall_state}")
dump_errors(my_drive)

if not axis.encoder.is_ready:
    print("is_ready가 여전히 False입니다.")
    exit()

# 4. 폐루프 제어 진입
print("폐루프 제어 모드 진입")
axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
time.sleep(0.5)

if axis.current_state != AxisState.CLOSED_LOOP_CONTROL:
    print("폐루프 진입 실패!")
    dump_errors(my_drive)
    exit()

axis.controller.input_pos = 0.0
time.sleep(1.0)

# 5. 10바퀴 회전 테스트
print("10바퀴 회전 시작...")
axis.controller.input_pos = 10.0

while True:
    current_pos = axis.encoder.pos_estimate
    print(f"현재 위치: {current_pos:.2f} turns")
    if abs(current_pos - 10.0) < 0.3:
        break
    time.sleep(0.2)

print("10바퀴 회전 완료")
time.sleep(1.0)

print("모터 정지")
axis.requested_state = AxisState.IDLE
print("테스트 완료")
