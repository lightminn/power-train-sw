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
    print("❌ 에러: 연결된 조이스틱/게임패드가 없습니다. USB나 블루투스 연결을 확인하세요.")
    exit()

# 첫 번째 컨트롤러 가져오기
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"🎮 게임패드 연결 성공: {joystick.get_name()}")

# ==========================================
# 2. ODrive 연결 및 초기화 (M1 / axis1 기준)
# ==========================================
print("🤖 ODrive 검색 중... (USB가 연결되어 있는지 확인하세요)")
my_drive = odrive.find_any()
print("✅ ODrive 연결 성공! 보드 시리얼:", my_drive.serial_number)

print("⚙️ M1(axis1) 캘리브레이션 시작... (모터가 움직입니다)")
my_drive.axis1.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE

while my_drive.axis1.current_state != AxisState.IDLE:
    time.sleep(0.1)

print("🔒 폐루프 제어(Closed Loop Control) 모드 진입")
my_drive.axis1.requested_state = AxisState.CLOSED_LOOP_CONTROL

# 모터 초기 위치를 0으로 맞춤
current_target_pos = 0.0
my_drive.axis1.controller.input_pos = current_target_pos

# ==========================================
# 3. 실시간 게임패드 제어 루프
# ==========================================
# 트리거를 끝까지 눌렀을 때 한 루프당 증가할 모터 회전량 (속도/민감도 조절용)
# 값이 클수록 트리거를 당겼을 때 모터가 더 빠르게 휙휙 돕니다.
SENSITIVITY = 0.15

# 데드존 (트리거를 아주 살짝 건드렸을 때 모터가 미세하게 떠는 것을 방지)
DEADZONE = 0.05

print("🚀 듀얼센스 모터 제어 시작!")
print(" - [RT] 시계 방향 회전 (아날로그)")
print(" - [LT] 반시계 방향 회전 (아날로그)")
print(" - [O 버튼] 또는 [B 버튼] 누르면 프로그램 종료")

running = True
while running:
    # Pygame 이벤트 펌핑 (입력 갱신)
    pygame.event.pump()

    # 종료 조건 (컨트롤러의 O 버튼/B 버튼을 누르거나 창을 닫을 때)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.JOYBUTTONDOWN:
            # 듀얼센스의 'O' 버튼은 보통 인덱스 1 (또는 2)입니다.
            if event.button == 1:
                running = False

    # ----------------------------------------------------
    # 아날로그 트리거 값 읽기 (SDL2 표준 맵핑 기준)
    # 듀얼센스/Xbox 패드: LT는 Axis 4, RT는 Axis 5
    # Pygame에서 트리거의 원본 값은 -1.0(안 누름) ~ 1.0(끝까지 누름) 입니다.
    # 이를 제어하기 쉽게 0.0 ~ 1.0 범위로 변환합니다.
    # ----------------------------------------------------
    raw_lt = joystick.get_axis(2)
    raw_rt = joystick.get_axis(5)

    # -1.0 ~ 1.0을 0.0 ~ 1.0 으로 정규화
    lt_val = (raw_lt + 1.0) / 2.0
    rt_val = (raw_rt + 1.0) / 2.0

    # 데드존 적용 (손가락만 올려놓았을 때 값이 튀는 것 방지)
    if lt_val < DEADZONE:
        lt_val = 0.0
    if rt_val < DEADZONE:
        rt_val = 0.0

    # 양쪽 트리거의 차이를 계산하여 최종 이동량 결정
    # RT를 누르면 양수(+), LT를 누르면 음수(-)
    trigger_delta = (rt_val - lt_val)

    # 현재 목표 위치에 입력량만큼 누적 (Incremental Position Update)
    # 살짝 누르면 조금씩 더해져서 천천히 돌고, 꾹 누르면 많이 더해져서 빨리 돕니다.
    current_target_pos += trigger_delta * SENSITIVITY

    # ODrive로 위치 명령 전송
    my_drive.axis1.controller.input_pos = current_target_pos

    # 현재 상태를 터미널에 출력 (디버깅용, 터미널 도배 방지를 위해 소수점 둘째 자리까지만)
    print(
        f"\r🎮 LT: {lt_val:.2f} | RT: {rt_val:.2f} ➡️ ⚙️ Motor Target: {current_target_pos:.2f} Turns    ", end="")

    # 루프 속도 조절 (약 100Hz = 초당 100번 제어)
    # 너무 빨리 명령을 보내면 USB 병목 현상이 생길 수 있습니다.
    time.sleep(0.01)

# ==========================================
# 4. 뒷정리
# ==========================================
print("\n🛑 모터 정지 및 제어 종료")
my_drive.axis1.requested_state = AxisState.IDLE
pygame.quit()
print("✅ 프로그램이 정상적으로 종료되었습니다.")
