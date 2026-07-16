"""
Laptop에서 실행: DualSense → Pi 명령 전송
실행: python3 robot_client.py

필요 패키지: pip install pygame
"""
import pygame
import socket
import time
import sys

try:
    from laptop.socket_options import configure_command_socket
except ModuleNotFoundError:  # direct script execution
    from socket_options import configure_command_socket

# ── 설정 ──────────────────────────────────────────
PI_HOST      = '192.168.1.91'   # Pi IP 주소
COMMAND_PORT = 9000
MAX_VEL      = 4.0             # rev/s 최대 속도

# DualSense 트리거 축 — dualsense_axis_finder.py 로 실측(USB/BT·SDL 버전마다 다름)
LT_AXIS = 2   # L2
RT_AXIS = 5   # R2

SEND_HZ  = 20   # 명령 전송 주기
DEADZONE = 0.02 # 트리거 데드존
# ──────────────────────────────────────────────────


def trigger_to_ratio(raw):
    """pygame 트리거 값 -1.0~1.0 → 비율 0.0~1.0"""
    ratio = (raw + 1.0) / 2.0
    return ratio if ratio > DEADZONE else 0.0


def detect_axes(joy):
    """축 인덱스 확인용 — 트리거를 눌러보며 확인"""
    print("=== 축 감지 모드 ===")
    print("트리거를 천천히 눌러보세요. Ctrl+C로 종료.\n")
    try:
        while True:
            pygame.event.pump()
            axes = [joy.get_axis(i) for i in range(joy.get_numaxes())]
            line = "  ".join(f"[{i}]{v:+.2f}" for i, v in enumerate(axes))
            print(f"\r{line}", end='', flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()


def connect_to_pi(host, port, retries=5):
    for i in range(retries):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3.0)
            sock.connect((host, port))
            configure_command_socket(sock)
            sock.settimeout(None)
            print(f"Pi 연결됨: {host}:{port}")
            return sock
        except OSError as e:
            print(f"연결 실패 ({i+1}/{retries}): {e}")
            time.sleep(1.0)
    return None


def main():
    pygame.init()
    pygame.joystick.init()

    if '--detect' in sys.argv:
        if pygame.joystick.get_count() == 0:
            print("컨트롤러가 없습니다.")
            return
        joy = pygame.joystick.Joystick(0)
        joy.init()
        print(f"컨트롤러: {joy.get_name()}")
        detect_axes(joy)
        return

    if pygame.joystick.get_count() == 0:
        print("DualSense 컨트롤러를 연결하세요.")
        return

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"컨트롤러: {joy.get_name()}")
    print(f"  축 수: {joy.get_numaxes()}  (LT=axis{LT_AXIS}, RT=axis{RT_AXIS})")

    sock = connect_to_pi(PI_HOST, COMMAND_PORT)
    if sock is None:
        print("Pi에 연결할 수 없습니다.")
        return

    print("\nRT: 시계방향  |  LT: 반시계방향  |  Ctrl+C: 종료\n")

    interval = 1.0 / SEND_HZ
    try:
        while True:
            t_start = time.monotonic()
            pygame.event.pump()

            rt = trigger_to_ratio(joy.get_axis(RT_AXIS))
            lt = trigger_to_ratio(joy.get_axis(LT_AXIS))

            # RT 우선 (동시 입력 시)
            if rt > 0:
                vel = rt * MAX_VEL
            elif lt > 0:
                vel = -lt * MAX_VEL
            else:
                vel = 0.0

            try:
                sock.send(f"{vel:.3f}\n".encode())
            except OSError:
                print("\nPi 연결 끊김 — 재연결 시도...")
                sock.close()
                sock = connect_to_pi(PI_HOST, COMMAND_PORT)
                if sock is None:
                    break

            print(f"\rLT={lt:.2f}  RT={rt:.2f}  →  vel={vel:+6.2f} rev/s   ", end='', flush=True)

            elapsed = time.monotonic() - t_start
            sleep_t = interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n정지 명령 전송 중...")

    finally:
        try:
            sock.send(b"0.000\n")
            time.sleep(0.1)
            sock.close()
        except OSError:
            pass
        pygame.quit()
        print("종료")


if __name__ == '__main__':
    main()
