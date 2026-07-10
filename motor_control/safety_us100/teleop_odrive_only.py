"""ODrive USB 단독 텔레옵 + US100 충돌방지.
AK 조향 모터 없이 ODrive 구동만 테스트할 때 사용.
RT → 전진, LT → 후진, □ → estop reset/arm/disarm, ○ → estop
"""
import sys
import time
import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")


def confirm_odrive_closed_loop(
    axis,
    closed_loop_state,
    timeout_s=0.5,
    clock=time.monotonic,
    sleeper=time.sleep,
):
    axis.requested_state = closed_loop_state
    deadline = clock() + timeout_s
    while True:
        if axis.current_state == closed_loop_state:
            return True
        remaining = deadline - clock()
        if remaining <= 0.0:
            return False
        sleeper(min(0.01, remaining))


def handle_odrive_square(armed, estop_latched, hazard_active, arm, disarm):
    """Apply one square edge; clearing an E-stop never arms in the same edge."""
    if hazard_active:
        disarm()
        return False, True
    if armed:
        disarm()
        return False, estop_latched
    if estop_latched:
        disarm()
        return False, False
    if arm():
        return True, False
    disarm()
    return False, True


def cleanup_odrive_resources(disarm, background, sensor, pygame_module):
    errors = []
    if disarm is not None:
        try:
            disarm()
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
    try:
        pygame_module.quit()
    except BaseException as exc:
        errors.append(exc)
    return errors


def main():
    import pygame
    import odrive
    # enums 전부 숫자로 대체
    AXIS_STATE_IDLE = 1
    AXIS_STATE_CLOSED_LOOP = 8
    from safety_us100.background_monitor import BackgroundSafetyMonitor
    from safety_us100.us100 import Us100Sensor
    from safety_us100.safety_monitor import SafetyMonitor
    from safety_us100.config import SafetyConfig

    axis = None
    sensor = None
    background = None

    def stop_motor():
        if axis is not None:
            axis.controller.input_vel = 0.0

    def disarm():
        stop_motor()
        if axis is not None:
            axis.requested_state = AXIS_STATE_IDLE

    def arm():
        return confirm_odrive_closed_loop(
            axis,
            AXIS_STATE_CLOSED_LOOP,
        )

    try:
        # ── ODrive 연결 ──
        print("ODrive 연결 중...")
        odrv = odrive.find_any(timeout=10)
        axis = odrv.axis1
        print("ODrive 연결 완료!")

        axis.controller.config.control_mode = 2
        axis.controller.config.input_mode = 2

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
    except BaseException:
        cleanup_odrive_resources(
            disarm,
            background,
            sensor,
            pygame,
        )
        raise

    armed = False
    estop_latched = False
    prev_sq = prev_ci = False
    last_print = 0.0
    VEL_LIMIT = 5.0  # turns/s, 필요시 조절

    print("□: estop reset/arm/disarm, ○: estop, Ctrl-C: 종료")
    print(f"US-100 충돌방지 활성화 (E-stop: {safety_cfg.stop_mm}mm 미만)")

    try:
        while True:
            try:
                pygame.event.pump()

                rt = (js.get_axis(5) + 1.0) / 2.0   # 전진 (R2, dualsense_axis_finder 실측)
                lt = (js.get_axis(2) + 1.0) / 2.0   # 후진 (L2)
                sq = js.get_button(3)                 # reset/arm/disarm (□)
                ci = js.get_button(1)                 # estop (○)

                verdict = background.verdict()
                hazard_active = verdict.estop_required
                if hazard_active:
                    if not estop_latched:
                        print("충돌방지 estop!")
                    disarm()
                    armed = False
                    estop_latched = True

                # 버튼 상승엣지
                if sq and not prev_sq:
                    was_latched = estop_latched
                    armed, estop_latched = handle_odrive_square(
                        armed,
                        estop_latched,
                        hazard_active,
                        arm,
                        disarm,
                    )
                    if hazard_active:
                        print("활성 hazard 중 estop reset 거부")
                    elif was_latched and not estop_latched:
                        print("estop reset — 다음 □에서 arm")
                    elif armed:
                        print("armed")
                    elif estop_latched:
                        print("arm 확인 실패→estop")
                    else:
                        print("disarmed")
                if ci and not prev_ci:
                    disarm()
                    armed = False
                    estop_latched = True
                    print("manual estop!")
                prev_sq, prev_ci = sq, ci

                if armed:
                    if verdict.status == "CHECKING":
                        stop_motor()
                    else:
                        trig = rt - lt
                        if abs(trig) < 0.05:
                            trig = 0.0
                        axis.controller.input_vel = trig * VEL_LIMIT

                now = time.monotonic()
                if now - last_print > 1.0:
                    dist_str = "(없음)" if verdict.distance_mm is None else f"{int(verdict.distance_mm)}mm"
                    state = "ESTOP" if estop_latched else ("ARMED" if armed else "IDLE")
                    vel = axis.encoder.vel_estimate if armed else 0.0
                    print(f"[{state}] 속도: {vel:.2f} turns/s | 거리: {dist_str} | status: {verdict.status}")
                    last_print = now
            except Exception as exc:
                try:
                    disarm()
                finally:
                    armed = False
                    estop_latched = True
                    print("제어 예외→ESTOP(control_exception): %s" % exc)

            time.sleep(0.02)  # 50Hz

    except KeyboardInterrupt:
        pass
    finally:
        cleanup_odrive_resources(
            disarm,
            background,
            sensor,
            pygame,
        )
        print("종료")


if __name__ == "__main__":
    sys.path.insert(0, ".")
    main()
