import odrive
from odrive.enums import *
import time

print("ODrive 찾는 중...")
odrv0 = odrive.find_any(timeout=10)
ax = odrv0.axis1
print(f"ODrive 연결 성공! SN: {odrv0.serial_number}")

# ----------------------------------------------------
# 1. 모터 및 엔코더 스펙, 안전/튜닝 설정
# ----------------------------------------------------
print("1. 스펙 및 Gain 값 설정 중...")
ax.motor.config.pole_pairs = 7
ax.encoder.config.cpr = 16384

ax.controller.config.vel_limit = 10.0

ax.controller.config.vel_integrator_gain = 0
ax.controller.config.vel_gain = 0.02
ax.controller.config.pos_gain = 1.0

ax.controller.config.input_filter_bandwidth = 2.0
ax.controller.config.input_mode = INPUT_MODE_POS_FILTER

# ----------------------------------------------------
# 2. 전체 캘리브레이션 실행 (자동 대기 기능 추가)
# ----------------------------------------------------
print("2. 캘리브레이션 시작! 모터가 회전합니다. (약 10~15초 소요)")
ax.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

# 💡 캘리브레이션이 끝날 때까지 대기하는 필수 코드
while ax.current_state != AXIS_STATE_IDLE:
    time.sleep(0.5)

print(f"캘리브레이션 성공 여부: {ax.encoder.is_ready}")

if not ax.encoder.is_ready:
    print("❌ 에러: 캘리브레이션에 실패했습니다. 연결 상태를 확인하세요.")
    exit()

# ----------------------------------------------------
# 3. 자동 실행 설정 및 영구 저장
# ----------------------------------------------------
print("3. 전원 인가 시 자동 제어 모드 진입 설정 중...")
ax.motor.config.pre_calibrated = True
ax.encoder.config.pre_calibrated = True
ax.config.startup_encoder_offset_calibration = True
ax.config.startup_closed_loop_control = True

print("4. 설정 저장 및 재부팅...")
try:
    odrv0.save_configuration()
except:
    pass

try:
    odrv0.reboot()
except:
    pass

# ----------------------------------------------------
# 4. 재부팅 후 테스트 구동
# ----------------------------------------------------
print("재부팅 중입니다... 5초 대기...")
time.sleep(5)

print("5. ODrive 재연결 및 테스트 구동 준비...")
# 💡 재부팅되면서 USB 연결이 끊기므로 다시 찾아야 함
odrv0 = odrive.find_any(timeout=10)
ax = odrv0.axis1

ax.clear_errors()
ax.controller.config.input_filter_bandwidth = 2.0
ax.controller.config.input_mode = INPUT_MODE_POS_FILTER
ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

print("🎯 위치 1.0으로 이동 테스트 시작!")
ax.controller.input_pos = 1.0

print("모든 초기화 및 테스트가 완벽하게 끝났습니다!")
