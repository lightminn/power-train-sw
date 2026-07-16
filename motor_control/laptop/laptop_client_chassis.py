"""노트북에서 실행: DualSense raw 입력 → 젯슨(chassis.teleop_server) TCP 전송.

무선 차체 4WS 텔레옵의 노트북쪽. DualSense 는 노트북에 USB/BT 로 붙이고, 노트북↔젯슨은
무선(라우터). 이 클라는 **매핑 안 하고 raw 입력만** 보냄 — 속도한계·min_drive·피벗 등
로봇 튜닝은 전부 서버(젯슨)에 있음. 프로토콜: `"left_x rt lt sq ci\n"`.

필요: pip install pygame.  실행:
  python3 laptop/laptop_client_chassis.py --host 192.168.8.106      # 젯슨 IP
  python3 laptop/laptop_client_chassis.py --detect                 # 축/버튼 인덱스 확인
젯슨: python3 -m chassis.teleop_server --no-us100

조작: □=arm/disarm · ○=estop · RT/LT=전/후진 · 좌스틱X=회전 · (트리거0+스틱=피벗)
⚠️ 축/버튼 번호는 OS/드라이버마다 다를 수 있음 — 안 맞으면 --detect 로 확인 후 상수 수정.
"""
import argparse
import socket
import time

import pygame

DEFAULT_HOST = "192.168.8.106"    # 젯슨(라우터 고정예약) 기본 IP
DEFAULT_PORT = 9000
SEND_HZ = 30
DEADZONE = 0.03

# DualSense 축/버튼 — dualsense_axis_finder.py 로 실측(USB/BT·SDL 버전마다 다름, 안 맞으면 재실행)
LX_AXIS = 0      # 좌스틱 X
RT_AXIS = 5      # R2 (전진)
LT_AXIS = 2      # L2 (후진)
SQ_BTN = 3       # □ arm/disarm
CI_BTN = 1       # ○ estop


def trig(raw):
    r = (raw + 1.0) / 2.0        # -1..1 → 0..1
    return r if r > DEADZONE else 0.0


def connect(host, port, retries=0):
    """retries=0 → 성공할 때까지 무한 재시도 (Ctrl-C 로만 중단)."""
    i = 0
    while retries == 0 or i < retries:
        i += 1
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(3.0)
            s.connect((host, port))
            # 무음 끊김(WiFi 블립) 을 send 에러로 드러나게 — 자동 재연결 트리거
            s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Nagle 뭉침 방지
            if hasattr(socket, "TCP_USER_TIMEOUT"):
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_USER_TIMEOUT, 5000)
            s.setblocking(False)          # 상태회신(recv) 논블로킹 폴링용
            print("서버 연결됨: %s:%d" % (host, port))
            return s
        except OSError as e:
            print("연결 실패 (%d%s): %s" % (i, "/%d" % retries if retries else "", e))
            time.sleep(1.0)
    return None


def detect(joy):
    print("=== 축/버튼 감지 — 스틱/트리거 움직이고 버튼 눌러보세요 (Ctrl-C 종료) ===")
    try:
        while True:
            pygame.event.pump()
            ax = "  ".join("[%d]%+.2f" % (i, joy.get_axis(i)) for i in range(joy.get_numaxes()))
            bt = "".join(str(joy.get_button(i)) for i in range(joy.get_numbuttons()))
            print("\r축 %s | 버튼 %s" % (ax, bt), end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()


def main():
    ap = argparse.ArgumentParser(description="차체 4WS 무선 텔레옵 — 노트북 클라")
    ap.add_argument("--host", default=DEFAULT_HOST, help="젯슨 IP (기본 192.168.8.106)")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--detect", action="store_true", help="축/버튼 인덱스 확인 모드")
    args = ap.parse_args()

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("DualSense 컨트롤러를 연결하세요.")
        return
    joy = pygame.joystick.Joystick(0)
    joy.init()
    print("컨트롤러: %s (축 %d · 버튼 %d)" % (joy.get_name(), joy.get_numaxes(), joy.get_numbuttons()))

    if args.detect:
        detect(joy)
        return

    sock = connect(args.host, args.port)
    if sock is None:
        print("서버에 연결할 수 없습니다.")
        return
    print("□:arm/disarm · ○:estop · RT/LT:전/후진 · 좌스틱X:회전 · Ctrl-C:종료")

    interval = 1.0 / SEND_HZ
    srv_mode, srv_v, srv_w = "?", "?", "?"      # 서버 상태 회신 ("S mode v ω")
    last_srv_ts = None
    try:
        while True:
            t0 = time.monotonic()
            pygame.event.pump()
            lx = joy.get_axis(LX_AXIS)
            rt = trig(joy.get_axis(RT_AXIS))
            lt = trig(joy.get_axis(LT_AXIS))
            sq = joy.get_button(SQ_BTN)
            ci = joy.get_button(CI_BTN)
            try:
                sock.send(("%.4f %.4f %.4f %d %d\n" % (lx, rt, lt, sq, ci)).encode())
            except BlockingIOError:
                pass                              # 송신버퍼 일시 포화 — 이 프레임만 스킵
            except OSError:
                print("\n서버 연결 끊김 — 재연결...")
                sock.close()
                sock = connect(args.host, args.port)
                if sock is None:
                    break
                srv_mode, last_srv_ts = "?", None
                continue
            # 서버 상태 회신 수신 (논블로킹)
            try:
                rx = sock.recv(1024)
                for ln in rx.decode(errors="ignore").splitlines():
                    p = ln.split()
                    if len(p) >= 4 and p[0] == "S":
                        srv_mode, srv_v, srv_w = p[1], p[2], p[3]
                        last_srv_ts = time.monotonic()
            except BlockingIOError:
                pass
            except OSError:
                pass                              # 수신 오류는 send 쪽 재연결 로직에 맡김
            srv = "%s v=%s ω=%s" % (srv_mode, srv_v, srv_w)
            if last_srv_ts is not None and time.monotonic() - last_srv_ts > 2.0:
                srv = "응답없음 %.0fs" % (time.monotonic() - last_srv_ts)
            print("\rlx=%+.2f rt=%.2f lt=%.2f □%d ○%d | 서버[%s]   "
                  % (lx, rt, lt, sq, ci, srv), end="", flush=True)
            dt = time.monotonic() - t0
            if interval - dt > 0:
                time.sleep(interval - dt)
    except KeyboardInterrupt:
        pass
    finally:
        if sock is not None:                 # 재연결 실패로 None 일 수 있음
            try:
                sock.send(b"0 0 0 0 0\n")
                time.sleep(0.1)
                sock.close()
            except OSError:
                pass
        pygame.quit()
        print("\n종료")


if __name__ == "__main__":
    main()
