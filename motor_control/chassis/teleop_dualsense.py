"""차체 DualSense 텔레옵: 게임패드 → (v, ω) → ChassisManager (10모터 4WS 수동주행).

corner_module.teleop_dualsense(코너 1개)의 차체 확장판. 조향 AK ×4 + 구동 ODrive ×6
을 컨트롤러 하나로 동시 제어한다. 입력을 (전진속도 v, 요레이트 ω)로 매핑하면
ChassisManager 가 4WS 키네마틱스로 10모터에 분배 — 애커만 선회·뒤축 역위상·차동이
자동으로 나온다. map_chassis_input 은 순수 함수라 단위 테스트한다.

조작 (DualSense, dualsense_axis_finder.py 실측 — USB/BT·SDL 버전마다 다름, 안 맞으면 재실행):
  RT(R2, axis5) / LT(L2, axis2)  = 전진 / 후진 속도 v
  좌스틱 X (axis0)               = 회전 ω  (오른쪽=우회전)
  □(Square, btn3)                = E-stop reset / arm / disarm
  ○(Circle, btn1)                = estop (전 코너 정지)
  ▸ 트리거 없이(v=0) 좌스틱만 = **제자리 회전(피벗)**.

▸ 저속 HALL 코깅존(<~1 rev/s) 회피: `--min-rev`(기본 1.0 turns/s) 로 **최저 구동속도
  플로어**를 둔다 — 트리거를 살짝만 당겨도 각 바퀴가 즉시 1 rev/s 이상으로 붙어(부호
  유지) 툭툭 끊김·기동지연 제각각이 사라진다. 0 으로 두면 off(선형). 조향은 링키지 범위 내.

실행 (컨테이너 안, motor_control/ 에서 · can0 UP · DualSense 연결):
  python3 -m chassis.teleop_dualsense --no-us100      # US-100 없이 (구동 게이팅 OFF)
  python3 -m chassis.teleop_dualsense                 # US-100 충돌방지 ON (기본)
  옵션: --v-max 1.5  --omega-max 1.2  --min-rev 1.0  --channel can0
"""
import sys
import time


def map_chassis_input(left_x: float, rt: float, lt: float,
                      v_max: float = 0.6, omega_max: float = 1.2,
                      deadzone: float = 0.05):
    """게임패드 입력 → (v_mps, omega_rad_s).

    v = (RT−LT)·v_max  (전진 +, 후진 −),  ω = −좌스틱X·omega_max
    (REP-103: ω>0=좌회전이라 스틱 오른쪽(+x)=우회전=ω<0). deadzone 이하는 0.
    """
    trig = rt - lt
    if abs(trig) < deadzone:
        trig = 0.0
    v = trig * v_max
    sx = 0.0 if abs(left_x) < deadzone else left_x
    omega = -sx * omega_max
    return v, omega


def handle_chassis_square(manager):
    """Apply one square-button edge without combining reset and arm."""
    if manager.mode == "ESTOP":
        return bool(manager.reset_estop())
    if manager.mode == "ARMED":
        manager.disarm()
        return True
    if manager.mode == "IDLE":
        return bool(manager.arm())
    return False


def cleanup_chassis_resources(
    manager,
    background=None,
    sensor=None,
    pygame_module=None,
):
    """Best-effort shutdown; the sampling worker stops before its sensor."""
    errors = []
    if manager is not None:
        try:
            manager.estop("teleop_shutdown", "teleop cleanup")
        except BaseException as exc:
            errors.append(exc)
    background_stopped = True
    if background is not None:
        try:
            background_stopped = background.close() is True
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
    if manager is not None:
        for corner in manager.corners.values():
            try:
                corner.close()
            except BaseException as exc:
                errors.append(exc)
        manager.mode = "DISCONNECTED"
    if pygame_module is not None:
        try:
            pygame_module.quit()
        except BaseException as exc:
            errors.append(exc)
    return errors


def main(argv=None):
    import argparse
    import os

    parser = argparse.ArgumentParser(description="차체(10모터 4WS) DualSense 텔레옵")
    parser.add_argument("--no-us100", action="store_true",
                        help="US-100 충돌방지 없이 (구동 게이팅 OFF — 장애물 자동정지 안 함)")
    parser.add_argument("--channel", default="can0", help="socketcan 채널 (기본 can0)")
    parser.add_argument("--v-max", type=float, default=1.5, help="최대 전진속도 m/s (기본 1.5 ≈ 바퀴 2.4 rev/s)")
    parser.add_argument("--omega-max", type=float, default=1.2, help="최대 요레이트 rad/s (기본 1.2)")
    parser.add_argument("--min-rev", type=float, default=1.0,
                        help="최저 구동속도 turns/s (기본 1.0 — 저속 코깅존 회피, 0=off)")
    args = parser.parse_args(argv)
    use_us100 = not args.no_us100

    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

    import pygame
    from chassis.chassis_manager import ChassisManager, ChassisConfig, build_real_corners
    from corner_module.can_watchdog import CanWatchdog

    CanWatchdog(args.channel).start()    # mttcan TX 웻지 자가복구 (데몬 스레드)

    # US-100 blocking I/O는 배경 작업자만 수행한다.
    background = None
    sensor = None
    cm = None
    try:
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

        corners = build_real_corners(args.channel)
        cfg = ChassisConfig(min_drive_turns_per_s=args.min_rev)
        # v_max 만큼 낼 수 있게 kinematics 속도상한도 함께 올림.
        cfg.geometry.drive_limit_mps = max(
            args.v_max,
            cfg.geometry.drive_limit_mps,
        )
        cm = ChassisManager(corners, cfg)

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            print("게임패드 미연결")
            raise SystemExit(1)
        js = pygame.joystick.Joystick(0)
        js.init()
        cm.connect()
    except BaseException as exc:
        if cm is not None and not isinstance(exc, (KeyboardInterrupt, SystemExit)):
            cm.estop("control_exception", str(exc))
        cleanup_chassis_resources(cm, background, sensor, pygame)
        raise
    prev_sq = prev_ci = False
    period = 1.0 / cfg.loop_hz
    last_print = 0.0
    verdict = None

    print("=== 차체 4WS 텔레옵 (10모터) ===")
    print("□: reset/arm/disarm · ○: estop · RT/LT: 전/후진 · 좌스틱X: 회전 · (트리거0+스틱=피벗) · Ctrl-C: 종료")
    print("v_max=%.2f m/s · omega_max=%.2f rad/s · 최저구동 %.1f rev/s(코깅존 회피) · %s"
          % (args.v_max, args.omega_max, args.min_rev,
             "US-100 ON (E-stop <200mm)" if use_us100 else "⚠️ US-100 OFF"))

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
                    cm.update_external_safety(
                        verdict.status,
                        verdict.estop_required,
                        verdict.detail,
                    )

                if sq and not prev_sq:
                    if not handle_chassis_square(cm):
                        print("□ 요청 거부 (mode=%s)" % cm.mode)
                if ci and not prev_ci:
                    cm.estop("manual", "dualsense")
                prev_sq, prev_ci = sq, ci

                if cm.mode == "ARMED":
                    v, omega = map_chassis_input(
                        left_x,
                        rt,
                        lt,
                        args.v_max,
                        args.omega_max,
                    )
                    cm.set(v, omega)

                cm.tick()

                now = time.monotonic()
                if now - last_print > 1.0:
                    st = cm.state()
                    status = verdict.status if verdict is not None else "DISABLED"
                    print("mode=%s v=%+.2f ω=%+.2f safety=%s status=%s"
                          % (st["mode"], st["v"], st["omega"],
                             st["safety"].state, status))
                    last_print = now
            except Exception as exc:
                cm.estop("control_exception", str(exc))
                print("제어 예외→ESTOP: %s" % exc)

            time.sleep(period)

    except KeyboardInterrupt:
        pass
    finally:
        errors = cleanup_chassis_resources(
            cm,
            background,
            sensor,
            pygame,
        )
        if errors:
            print("정리 예외 %d건: %s" % (len(errors), errors[0]))
        print("종료")


if __name__ == "__main__":
    sys.path.insert(0, ".")
    main()
