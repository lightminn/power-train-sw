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
    import pygame  # 게임패드 입력 (기존 odrive_dualsense_* 와 동일 방식)

    from corner_module.corner_module import CornerModule
    from corner_module.steer_ak40 import SteerAk40
    from corner_module.drive_odrive_usb import DriveOdriveUsb

    cfg = CornerConfig()
    cm = CornerModule(SteerAk40(motor_id=1), DriveOdriveUsb(), cfg)

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("게임패드 미연결"); sys.exit(1)
    js = pygame.joystick.Joystick(0)
    js.init()

    cm.connect()
    armed = False
    period = 1.0 / cfg.loop_hz
    last_print = 0.0
    print("□: arm/disarm 토글, ○: estop, Ctrl-C: 종료")
    try:
        while True:
            pygame.event.pump()
            left_x = js.get_axis(0)
            rt = (js.get_axis(5) + 1.0) / 2.0   # 트리거 [-1,1] → [0,1]
            lt = (js.get_axis(4) + 1.0) / 2.0   # axis 4 = L2 (laptop_client_velocity.py 기준)
            if js.get_button(2):                  # □ Square — arm/disarm 토글
                if armed:
                    cm.disarm(); armed = False
                else:
                    cm.arm(); armed = True
                time.sleep(0.3)                   # 디바운스
            if js.get_button(1):                  # ○ Circle — estop
                cm.estop(); armed = False

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
