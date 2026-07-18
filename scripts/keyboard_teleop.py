#!/usr/bin/env python3
"""키보드 텔레옵 (DualSense 없을 때 대체) — /cmd_vel 로 발행.

    i : 전진      , : 후진
    j : 좌회전(ω+)  l : 우회전(ω-)
    k 또는 space : 즉시 정지
    u : 속도 10% 증가   m : 속도 10% 감소
    q : 종료

★ chassis_node 의 명령 워치독이 300ms 라 이 스크립트는 항상 20Hz 로 현재
  (v, w) 를 계속 발행한다 — 키를 안 눌러도 마지막 값을 유지해서 보낸다.
  안전을 위해 시작값은 반드시 0, 속도 상한은 v_step 이하로 제한한다.
"""
import sys
import termios
import tty
import select
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

V_STEP = 0.4     # m/s, 한 번 누르면 이 속도로 고정 (min_rev 플로어 위)
W_STEP = 0.4     # rad/s
V_MAX = 1.0      # 안전 상한 (v_max=1.5 보다 낮게)


def getch_nonblocking():
    dr, _, _ = select.select([sys.stdin], [], [], 0.0)
    if dr:
        return sys.stdin.read(1)
    return None


def main():
    rclpy.init()
    node = Node("keyboard_teleop")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)

    v, w, scale = 0.0, 0.0, 1.0
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    print(__doc__)
    print(f"현재: v={v:+.2f} w={w:+.2f} scale={scale:.1f}  (Ctrl+C 로도 종료)")

    try:
        last_pub = 0.0
        while True:
            ch = getch_nonblocking()
            if ch:
                if ch == "i":
                    v, w = V_STEP * scale, 0.0
                elif ch == ",":
                    v, w = -V_STEP * scale, 0.0
                elif ch == "j":
                    v, w = 0.0, W_STEP * scale
                elif ch == "l":
                    v, w = 0.0, -W_STEP * scale
                elif ch in ("k", " "):
                    v, w = 0.0, 0.0
                elif ch == "u":
                    scale = min(scale + 0.1, V_MAX / V_STEP)
                elif ch == "m":
                    scale = max(scale - 0.1, 0.0)
                elif ch == "q":
                    break
                print(f"\r현재: v={v:+.2f} w={w:+.2f} scale={scale:.1f}   ", end="", flush=True)

            now = time.monotonic()
            if now - last_pub >= 0.05:      # 20 Hz
                msg = Twist()
                msg.linear.x = v
                msg.angular.z = w
                pub.publish(msg)
                last_pub = now
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        for _ in range(5):
            pub.publish(stop)
            time.sleep(0.05)
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()
        print("\n정지 명령 발행 후 종료")


if __name__ == "__main__":
    main()
