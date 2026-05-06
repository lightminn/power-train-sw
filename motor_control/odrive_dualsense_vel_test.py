import time
import pygame

# --- ODrive 라이브러리 및 상태값 임포트 ---
import odrive
from odrive.enums import *

# ==========================================
# 1. 조이스틱(듀얼센스) 초기화 및 연결
# ==========================================
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("❌ 에러: 연결된 조이스틱/게임패드가 없습니다.")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"🎮 게임패드 연결 성공: {joystick.get_name()}")

# ==========================================
# 2. ODrive 연결 및 초기화 (M1 / axis1 기준)
# ==========================================
print("🤖 ODrive 검색 중... (USB 연결 확인)")
my_drive = odrive.find_any()
print("✅ ODrive 연결 성공! 보드 시리얼:", my_drive.serial_number)

print("⚙️ M1(axis1) 캘리브레이션 시작...")
my_drive.axis1.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE

while my_drive.axis1.current_state != AxisState.IDLE:
    time.sleep(0.1)

# 💡 [핵심 수정 사항] 제어 모드와 입력 모드를 세트로 맞춰줍니다!
print("🔄 제어 모드를 '속도 제어(Velocity Control)'로 세팅합니다.")
my_drive.axis1.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
# 기존에 저장된 POS_FILTER를 무시하고 다이렉트로 속도 값을 꽂아넣는 모드
my_drive.axis1.controller.config.input_mode = InputMode.PASSTHROUGH

print("🔒 폐루프 제어(Closed Loop Control) 모드 진입")
my_drive.axis1.requested_state = AxisState.CLOSED_LOOP_CONTROL

# 모터 초기 속도를 0으로 맞춰 정지 상태 유지
my_drive.axis1.controller.input_vel = 0.0

# ==========================================
# 3. 실시간 게임패드 제어 루프
# ==========================================
# 트리거를 끝까지 꾹 눌렀을 때의 최대 회전 속도 (초당 바퀴 수, Turns/sec)
# 이 값을 올리면 모터의 최고 속도가 빨라집니다.
MAX_VELOCITY = 5.0

# 데드존
DEADZONE = 0.05

print("🚀 듀얼센스 [속도 제어] 시작!")
print(" - [RT] 시계 방향 회전 (엑셀)")
print(" - [LT] 반시계 방향 회전 (후진)")
print(" - [손 떼면] 정지")
print(" - [O 버튼] 또는 창 닫기 시 종료")

running = True
while running:
    pygame.event.pump()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.JOYBUTTONDOWN:
            if event.button == 1:  # O 버튼
                running = False

    # ----------------------------------------------------
    # 아날로그 트리거 값 읽기
    # ----------------------------------------------------
    raw_lt = joystick.get_axis(2)  # 유저 환경에 맞춘 인덱스 유지
    raw_rt = joystick.get_axis(5)

    # -1.0 ~ 1.0을 0.0 ~ 1.0 으로 정규화
    lt_val = (raw_lt + 1.0) / 2.0
    rt_val = (raw_rt + 1.0) / 2.0

    if lt_val < DEADZONE:
        lt_val = 0.0
    if rt_val < DEADZONE:
        rt_val = 0.0

    # 양쪽 트리거의 차이 (-1.0 ~ 1.0)
    trigger_delta = (rt_val - lt_val)

    # 💡 [핵심 변경] 트리거 깊이 비례 목표 속도 계산
    target_vel = trigger_delta * MAX_VELOCITY

    # ODrive로 속도 명령 전송 (input_pos -> input_vel 로 변경)
    my_drive.axis1.controller.input_vel = target_vel

    print(
        f"\r🎮 LT: {lt_val:.2f} | RT: {rt_val:.2f} ➡️ 💨 속도: {target_vel:5.2f} Turns/s    ", end="")

    time.sleep(0.01)

# ==========================================
# 4. 뒷정리
# ==========================================
print("\n🛑 모터 정지 및 제어 종료")
my_drive.axis1.controller.input_vel = 0.0

# 💡 다음 번에 위치 제어 코드를 돌릴 때를 대비해 보드 상태를 원래대로 복구
my_drive.axis1.controller.config.control_mode = ControlMode.POSITION_CONTROL
my_drive.axis1.controller.config.input_mode = InputMode.POS_FILTER

my_drive.axis1.requested_state = AxisState.IDLE
pygame.quit()
print("✅ 프로그램이 정상적으로 종료되었습니다.")
