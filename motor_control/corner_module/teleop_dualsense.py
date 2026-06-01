"""DualSense 텔레옵 데모: 게임패드 입력 → CornerModule.set().
map_input 은 순수 함수로 분리해 단위 테스트한다. 실행 진입점(main)은 Task 9.
축 인덱스 (Linux pygame, DualSense):
  좌스틱 X  = axis 0
  LT(L2)    = axis 4
  RT(R2)    = axis 5
  □(Square) = button 2
  ○(Circle) = button 1
"""
import sys
import time
from corner_module.config import CornerConfig


def map_input(left_x: float, rt: float, lt: float, cfg: CornerConfig,
              deadzone: float = 0.05):
    sx = 0.0 if abs(left_x) < deadzone else left_x
    steer_deg = sx * cfg.steer_max_deg
    trig = rt - lt
    if abs(trig) < deadzone:
        trig = 0.0
    drive_vel = trig * cfg.drive_vel_limit
    return steer_deg, drive_vel


def main():
    import os
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

    import pygame
    from corner_module.corner_module import CornerModule
    from corner_module.steer_ak40 import SteerAk40
    from corner_module.drive_odrive_usb import DriveOdriveUsb

    # ── 추가: safety_monitor 임포트 ──
    from safety_us100.us100 import Us100Sensor
    from safety_us100.safety_monitor import SafetyMonitor
    from safety_us100.config import SafetyConfig

    cfg = CornerConfig()
    ak_id = int(os.environ.get("AK_MOTOR_ID", "10"))
    cm = CornerModule(SteerAk40(motor_id=ak_id), DriveOdriveUsb(), cfg)

    # ── 추가: 센서 + 감시 본체 초기화 ──
    safety_cfg = SafetyConfig()
    sensor = Us100Sensor(port=safety_cfg.port, baud=safety_cfg.baud)
    sensor.open()
    monitor = SafetyMonitor(sensor, safety_cfg)

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("게임패드 미연결"); sys.exit(1)
    js = pygame.joystick.Joystick(0)
    js.init()

    cm.connect()
    armed = False
    prev_sq = prev_ci = False
    period = 1.0 / cfg.loop_hz
    last_print = 0.0

    print("□: arm/disarm 토글, ○: estop, Ctrl-C: 종료")
    print("US-100 충돌방지 활성화 (stop 기준: 200mm, warn 기준: 400mm)")

    try:
        while True:
            pygame.event.pump()
            left_x = js.get_axis(0)
            rt = (js.get_axis(4) + 1.0) / 2.0
            lt = (js.get_axis(3) + 1.0) / 2.0
            sq = js.get_button(0)
            ci = js.get_button(2)

            if sq and not prev_sq:
                if armed:
                    cm.disarm(); armed = False
                else:
                    cm.arm(); armed = True
            if ci and not prev_ci:
                cm.estop(); armed = False
            prev_sq, prev_ci = sq, ci

            # ── 추가: 거리 판정 ──
            monitor.tick()
            verdict = monitor.verdict()

            if armed and cm.mode == "ARMED":
                if verdict.level == "stop":
                    # 장애물 너무 가까움 → 모터 멈춤
                    cm.set(0.0, 0.0)
                else:
                    steer, drive = map_input(left_x, rt, lt, cfg)
                    cm.set(steer, drive)

            cm.tick()

            now = time.monotonic()
            if now - last_print > 1.0:
                dist_str = "(없음)" if verdict.distance_mm is None else f"{int(verdict.distance_mm)}mm"
                print(f"{cm.state()} | 거리: {dist_str} 판정: {verdict.level}")
                last_print = now

            time.sleep(period)

    except KeyboardInterrupt:
        pass
    finally:
        cm.disarm()
        cm.close()
        sensor.close()
        print("종료")


if __name__ == "__main__":
    sys.path.insert(0, ".")
    main()
