"""
노트북에서 실행: python3 robot_laptop.py
  - pygame:    DualSense 트리거 읽어 Pi로 속도 전송 (포트 9000)
  - GStreamer: 소켓 연결 성공 후 영상 수신 시작 (포트 5000)
"""
import socket
import subprocess
import time
import pygame

try:
    from laptop.socket_options import configure_command_socket
except ModuleNotFoundError:  # direct script execution
    from socket_options import configure_command_socket

PI_HOST      = '192.168.1.91'   # Pi IP 주소
COMMAND_PORT = 9000
VIDEO_PORT   = 5000
MAX_VELOCITY = 5.0
DEADZONE     = 0.05
SEND_HZ      = 20

LT_AXIS = 2
RT_AXIS = 5

GST_CMD = (
    f"gst-launch-1.0 tcpclientsrc host={PI_HOST} port={VIDEO_PORT} "
    "! jpegdec ! autovideosink sync=false"
)

# ── pygame DualSense 초기화 ───────────────────────
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("❌ 연결된 게임패드가 없습니다.")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"🎮 게임패드 연결: {joystick.get_name()}")

# ── Pi 명령 소켓 연결 (Pi가 준비될 때까지 대기) ──
print(f"📡 Pi({PI_HOST}:{COMMAND_PORT}) 연결 대기 중... (캘리브레이션 완료까지 약 20초)")
while True:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((PI_HOST, COMMAND_PORT))
        configure_command_socket(sock)
        break
    except ConnectionRefusedError:
        sock.close()
        time.sleep(1)

print("✅ Pi 연결 성공!")

# ── GStreamer 영상 수신 시작 (Pi 준비 후) ─────────
print(f"📺 영상 수신 시작 ({PI_HOST}:{VIDEO_PORT})...")
gst_proc = subprocess.Popen(GST_CMD, shell=True)

print("  [RT] 시계 방향  [LT] 반시계 방향  [O 버튼 또는 Ctrl+C] 종료")

# ── 제어 루프 ─────────────────────────────────────
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
    print("\n🛑 종료 중...")
finally:
    try:
        sock.sendall(b"0.0\n")
        sock.close()
    except Exception:
        pass
    pygame.quit()
    gst_proc.terminate()
    gst_proc.wait()
    print("✅ 종료")
