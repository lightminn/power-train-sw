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


def handle_corner_square(corner):
    """Apply one square-button edge without combining fault reset and arm."""
    if corner.mode == "FAULT":
        return bool(corner.reset_fault())
    if corner.mode == "ARMED":
        corner.disarm()
        return True
    if corner.mode == "IDLE":
        corner.arm()
        return corner.mode == "ARMED"
    return False


def cleanup_corner_resources(corner, background, sensor, pygame_module):
    errors = []
    if corner is not None:
        try:
            corner.estop()
        except BaseException as exc:
            errors.append(exc)
    background_stopped = True
    if background is not None:
        try:
            background_stopped = background.close() is not False
            if not background_stopped:
                errors.append(
                    RuntimeError("US-100 background worker still running")
                )
        except BaseException as exc:
            errors.append(exc)
            background_stopped = False
    if sensor is not None and background_stopped:
        try:
            sensor.close()
        except BaseException as exc:
            errors.append(exc)
    if corner is not None:
        for actuator in (corner.steer, corner.drive):
            try:
                actuator.close()
            except BaseException as exc:
                errors.append(exc)
        corner.mode = "DISCONNECTED"
    try:
        pygame_module.quit()
    except BaseException as exc:
        errors.append(exc)
    return errors


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
    cm = None
    background = None
    sensor = None
    try:
        cm = CornerModule(
            SteerAk40(motor_id=args.ak_id),
            DriveOdriveUsb(),
            cfg,
        )
        if use_us100:
            from safety_us100.background_monitor import BackgroundSafetyMonitor
            from safety_us100.us100 import Us100Sensor
            from safety_us100.safety_monitor import SafetyMonitor
            from safety_us100.config import SafetyConfig
            safety_cfg = SafetyConfig()
            sensor = Us100Sensor(port=safety_cfg.port, baud=safety_cfg.baud)
            sensor.open()
            background = BackgroundSafetyMonitor(
                SafetyMonitor(sensor, safety_cfg),
            )
            background.start()

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            print("게임패드 미연결")
            raise SystemExit(1)
        js = pygame.joystick.Joystick(0)
        js.init()
        cm.connect()
    except BaseException:
        cleanup_corner_resources(cm, background, sensor, pygame)
        raise
    prev_sq = prev_ci = False
    period = 1.0 / cfg.loop_hz
    last_print = 0.0
    verdict = None

    print("□: fault reset/arm/disarm, ○: estop, Ctrl-C: 종료")
    if use_us100:
        print("US-100 충돌방지 ON (E-stop 200mm 미만)")
    else:
        print("⚠️ US-100 충돌방지 OFF — 구동 게이팅 없음 (장애물 자동정지 안 함)")

    try:
        while True:
            try:
                pygame.event.pump()
                left_x = js.get_axis(0)
                rt = (js.get_axis(5) + 1.0) / 2.0
                lt = (js.get_axis(2) + 1.0) / 2.0
                sq = js.get_button(3)
                ci = js.get_button(1)

                if background is not None:
                    verdict = background.verdict()
                    if verdict.estop_required and cm.mode != "FAULT":
                        cm.estop()

                if sq and not prev_sq:
                    hazard_active = (
                        verdict is not None and verdict.estop_required
                    )
                    if hazard_active:
                        print("활성 hazard 중 fault reset 거부")
                    elif not handle_corner_square(cm):
                        print("□ 요청 거부 (mode=%s)" % cm.mode)
                if ci and not prev_ci:
                    cm.estop()
                prev_sq, prev_ci = sq, ci

                if cm.mode == "ARMED":
                    steer, drive = map_input(left_x, rt, lt, cfg)
                    if verdict is not None and verdict.status == "CHECKING":
                        drive = 0.0
                    cm.set(steer, drive)

                cm.tick()

                now = time.monotonic()
                if now - last_print > 1.0:
                    if verdict is not None:
                        dist_str = "(없음)" if verdict.distance_mm is None else f"{int(verdict.distance_mm)}mm"
                        print(f"{cm.state()} | 거리: {dist_str} status: {verdict.status} estop: {verdict.estop_required}")
                    else:
                        print(f"{cm.state()} | US-100 OFF")
                    last_print = now
            except Exception as exc:
                try:
                    cm.estop()
                finally:
                    print("제어 예외→FAULT: %s" % exc)

            time.sleep(period)

    except KeyboardInterrupt:
        pass
    finally:
        cleanup_corner_resources(cm, background, sensor, pygame)
        print("종료")


if __name__ == "__main__":
    sys.path.insert(0, ".")
    main()
