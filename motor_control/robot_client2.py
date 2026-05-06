"""
노트북에서 실행: python3 robot_client2.py
DualSense 트리거 → TCP로 Pi에 속도값 전송
"""
import socket
import time
import pygame

PI_HOST      = '192.168.1.91'   # Pi IP 주소
COMMAND_PORT = 9000
MAX_VELOCITY = 5.0
DEADZONE     = 0.05
SEND_HZ      = 20               # 전송 주기

LT_AXIS = 2
RT_AXIS = 5

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("❌ 연결된 게임패드가 없습니다.")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"🎮 게임패드 연결 성공: {joystick.get_name()}")

print(f"📡 Pi({PI_HOST}:{COMMAND_PORT}) 연결 중...")
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((PI_HOST, COMMAND_PORT))
print("✅ 연결 성공!")
print("  [RT] 시계 방향  [LT] 반시계 방향  [O 버튼] 종료")

dt = 1.0 / SEND_HZ
running = True
try:
    while running:
        pygame.event.pump()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 1:   # O 버튼
                    running = False

        raw_lt = joystick.get_axis(LT_AXIS)
        raw_rt = joystick.get_axis(RT_AXIS)

        lt_val = (raw_lt + 1.0) / 2.0
        rt_val = (raw_rt + 1.0) / 2.0

        if lt_val < DEADZONE: lt_val = 0.0
        if rt_val < DEADZONE: rt_val = 0.0

        target_vel = (rt_val - lt_val) * MAX_VELOCITY

        sock.sendall(f"{target_vel:.4f}\n".encode())
        print(f"\r🎮 LT:{lt_val:.2f}  RT:{rt_val:.2f}  →  속도:{target_vel:+.2f} T/s    ", end="")

        time.sleep(dt)

except (BrokenPipeError, ConnectionResetError):
    print("\n❌ Pi 연결 끊김")
except KeyboardInterrupt:
    print("\n🛑 종료")
finally:
    try:
        sock.sendall(b"0.0\n")
        sock.close()
    except Exception:
        pass
    pygame.quit()
    print("✅ 종료")
