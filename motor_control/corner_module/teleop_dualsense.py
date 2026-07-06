"""DualSense 텔레옵 데모: 게임패드 입력 → CornerModule.set().
map_input 은 순수 함수로 분리해 단위 테스트한다.

축/버튼 (DualSense, dualsense_axis_finder.py 실측 — USB/BT·SDL 버전마다 다름, 안 맞으면 재실행):
  좌스틱 X  = axis 0
  RT(R2)    = axis 5
  LT(L2)    = axis 2
  □(Square) = button 3
  ○(Circle) = button 1

실행 — 두 경로:
  python3 -m corner_module.teleop_dualsense              # US-100 충돌방지 ON (기본)
  python3 -m corner_module.teleop_dualsense --no-us100   # US-100 없이 (구동 게이팅 OFF)
조향 AK id 변경: --ak-id N  (또는 AK_MOTOR_ID 환경변수, 기본 1)
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


def main(argv=None):
    import argparse
    import os

    parser = argparse.ArgumentParser(description="코너모듈 DualSense 텔레옵")
    parser.add_argument("--no-us100", action="store_true",
                        help="US-100 충돌방지 없이 실행 (구동 게이팅 OFF — 장애물 자동정지 안 함)")
    parser.add_argument("--ak-id", type=int,
                        default=int(os.environ.get("AK_MOTOR_ID", "1")),
                        help="조향 AK CAN id (기본 1, 또는 AK_MOTOR_ID 환경변수)")
    args = parser.parse_args(argv)
    use_us100 = not args.no_us100

    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

    import pygame
    from corner_module.corner_module import CornerModule
    from corner_module.steer_ak40 import SteerAk40
    from corner_module.drive_odrive_usb import DriveOdriveUsb
    from corner_module.can_watchdog import CanWatchdog

    CanWatchdog("can0").start()          # mttcan TX 웻지 자가복구 (데몬 스레드)

    cfg = CornerConfig()
    cm = CornerModule(SteerAk40(motor_id=args.ak_id), DriveOdriveUsb(), cfg)

    # US-100 충돌방지 (옵션). --no-us100 이면 센서·게이팅 전부 건너뜀.
    monitor = None
    sensor = None
    if use_us100:
        from safety_us100.us100 import Us100Sensor
        from safety_us100.safety_monitor import SafetyMonitor
        from safety_us100.config import SafetyConfig
        safety_cfg = SafetyConfig()
        sensor = Us100Sensor(port=safety_cfg.port, baud=safety_cfg.baud)
        sensor.open()
        monitor = SafetyMonitor(sensor, safety_cfg)

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("게임패드 미연결")
        sys.exit(1)
    js = pygame.joystick.Joystick(0)
    js.init()

    cm.connect()
    armed = False
    prev_sq = prev_ci = False
    period = 1.0 / cfg.loop_hz
    last_print = 0.0

    print("□: arm/disarm 토글, ○: estop, Ctrl-C: 종료")
    if use_us100:
        print("US-100 충돌방지 ON (stop 200mm / warn 400mm)")
    else:
        print("⚠️ US-100 충돌방지 OFF — 구동 게이팅 없음 (장애물 자동정지 안 함)")

    try:
        while True:
            pygame.event.pump()
            left_x = js.get_axis(0)
            rt = (js.get_axis(5) + 1.0) / 2.0
            lt = (js.get_axis(2) + 1.0) / 2.0
            sq = js.get_button(3)
            ci = js.get_button(1)

            if sq and not prev_sq:
                if armed:
                    cm.disarm(); armed = False
                else:
                    cm.arm(); armed = True
            if ci and not prev_ci:
                cm.estop(); armed = False
            prev_sq, prev_ci = sq, ci

            # 거리 판정 (US-100 사용 시만)
            verdict = None
            if monitor is not None:
                monitor.tick()
                verdict = monitor.verdict()

            if armed and cm.mode == "ARMED":
                if verdict is not None and verdict.level == "stop":
                    cm.set(0.0, 0.0)   # 장애물 가까움 → 구동 0
                else:
                    steer, drive = map_input(left_x, rt, lt, cfg)
                    cm.set(steer, drive)

            cm.tick()

            now = time.monotonic()
            if now - last_print > 1.0:
                if verdict is not None:
                    dist_str = "(없음)" if verdict.distance_mm is None else f"{int(verdict.distance_mm)}mm"
                    print(f"{cm.state()} | 거리: {dist_str} 판정: {verdict.level}")
                else:
                    print(f"{cm.state()} | US-100 OFF")
                last_print = now

            time.sleep(period)

    except KeyboardInterrupt:
        pass
    finally:
        cm.disarm()
        cm.close()
        if sensor is not None:
            sensor.close()
        print("종료")


if __name__ == "__main__":
    sys.path.insert(0, ".")
    main()
