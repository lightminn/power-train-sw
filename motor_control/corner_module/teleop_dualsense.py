"""DualSense 텔레옵 데모: 게임패드 입력 → CornerModule.set().

map_input 은 순수 함수로 분리해 단위 테스트한다. 실행 진입점(main)은 Task 9.

축 인덱스 (Linux pygame, DualSense):
  좌스틱 X  = axis 0
  LT(L2)    = axis 4   (laptop_client_velocity.py 기준; 구형 odrive_dualsense_vel_test.py 는 axis 2 사용)
  RT(R2)    = axis 5
  □(Square) = button 2
  ○(Circle) = button 1
"""
import sys
import time

from corner_module.config import CornerConfig


def map_input(left_x: float, rt: float, lt: float, cfg: CornerConfig,
              deadzone: float = 0.05):
    """좌스틱 X → 조향각, (RT−LT) → 구동속도. 데드존 적용.

    조향은 좌우 대칭(±steer_max_deg)으로 매핑한다(CornerModule 이 다시 clamp).
    """
    sx = 0.0 if abs(left_x) < deadzone else left_x
    steer_deg = sx * cfg.steer_max_deg

    trig = rt - lt
    if abs(trig) < deadzone:
        trig = 0.0
    drive_vel = trig * cfg.drive_vel_limit
    return steer_deg, drive_vel


def main():
    import os
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")  # 헤드리스(컨테이너)에서 pygame 조이스틱
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    import pygame  # 게임패드 입력 (기존 odrive_dualsense_* 와 동일 방식)

    from corner_module.corner_module import CornerModule
    from corner_module.steer_ak40 import SteerAk40
    from corner_module.drive_odrive_usb import DriveOdriveUsb

    cfg = CornerConfig()
    ak_id = int(os.environ.get("AK_MOTOR_ID", "10"))  # 실 조향모터 CAN id (HIL 검증: 10)
    cm = CornerModule(SteerAk40(motor_id=ak_id), DriveOdriveUsb(), cfg)

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("게임패드 미연결"); sys.exit(1)
    js = pygame.joystick.Joystick(0)
    js.init()

    cm.connect()
    armed = False
    prev_sq = prev_ci = False   # 버튼 상승엣지 검출용
    period = 1.0 / cfg.loop_hz
    last_print = 0.0
    print("□: arm/disarm 토글, ○: estop, Ctrl-C: 종료")
    try:
        while True:
            pygame.event.pump()
            left_x = js.get_axis(0)              # 좌스틱 X (HIL 검증)
            rt = (js.get_axis(4) + 1.0) / 2.0   # R2=axis4, [-1,1]→[0,1] (DualSense HIL 검증)
            lt = (js.get_axis(3) + 1.0) / 2.0   # L2=axis3 (DualSense HIL 검증)
            sq = js.get_button(0)                 # □ Square=btn0 — arm/disarm 토글 (HIL 검증)
            ci = js.get_button(2)                 # ○ Circle=btn2 — estop (HIL 검증)
            if sq and not prev_sq:                # 상승엣지에서만 토글 (블로킹 sleep 디바운스 제거 — tick 끊김→stale 방지)
                if armed:
                    cm.disarm(); armed = False
                else:
                    cm.arm(); armed = True
            if ci and not prev_ci:
                cm.estop(); armed = False
            prev_sq, prev_ci = sq, ci

            if armed and cm.mode == "ARMED":
                steer, drive = map_input(left_x, rt, lt, cfg)
                cm.set(steer, drive)
            cm.tick()

            now = time.monotonic()
            if now - last_print > 1.0:            # 1Hz 상태 출력
                print(cm.state())
                last_print = now
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        cm.disarm()
        cm.close()
        print("종료")


if __name__ == "__main__":
    sys.path.insert(0, ".")
    main()
