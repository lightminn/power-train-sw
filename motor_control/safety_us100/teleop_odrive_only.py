"""ODrive USB 단독 텔레옵 + US100 충돌방지.
AK 조향 모터 없이 ODrive 구동만 테스트할 때 사용.
RT → 전진, LT → 후진, □ → arm/disarm, ○ → estop
"""
import sys
import time
import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")


def main():
    import pygame
    import odrive
    # enums 전부 숫자로 대체
    AXIS_STATE_IDLE = 1
    AXIS_STATE_CLOSED_LOOP = 8
    from safety_us100.us100 import Us100Sensor
    from safety_us100.safety_monitor import SafetyMonitor
    from safety_us100.config import SafetyConfig

    # ── ODrive 연결 ──
    print("ODrive 연결 중...")
    odrv = odrive.find_any(timeout=10)
    axis = odrv.axis1
    print("ODrive 연결 완료!")

    # ── 속도 제어 모드 설정 ──
    axis.controller.config.control_mode = 2
    axis.controller.config.input_mode = 2

    # ── Safety Monitor 초기화 ──
    safety_cfg = SafetyConfig()
    sensor = Us100Sensor(port=safety_cfg.port, baud=safety_cfg.baud)
    sensor.open()
    monitor = SafetyMonitor(sensor, safety_cfg)

    # ── pygame 조이스틱 초기화 ──
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("게임패드 미연결")
        sensor.close()
        sys.exit(1)
    js = pygame.joystick.Joystick(0)
    js.init()

    armed = False
    prev_sq = prev_ci = False
    last_print = 0.0
    VEL_LIMIT = 5.0  # turns/s, 필요시 조절

    def stop_motor():
        axis.controller.input_vel = 0.0

    def disarm():
        stop_motor()
        axis.requested_state = AXIS_STATE_IDLE

    def arm():
        axis.requested_state = AXIS_STATE_CLOSED_LOOP

    print("□: arm/disarm, ○: estop, Ctrl-C: 종료")
    print(f"US-100 충돌방지 활성화 (stop: {safety_cfg.stop_mm}mm, warn: {safety_cfg.warn_mm}mm)")

    try:
        while True:
            pygame.event.pump()

            rt = (js.get_axis(4) + 1.0) / 2.0   # 전진
            lt = (js.get_axis(3) + 1.0) / 2.0   # 후진
            sq = js.get_button(0)                 # arm/disarm
            ci = js.get_button(2)                 # estop

            # 버튼 상승엣지
            if sq and not prev_sq:
                if armed:
                    disarm(); armed = False; print("disarmed")
                else:
                    arm(); armed = True; print("armed")
            if ci and not prev_ci:
                disarm(); armed = False; print("estop!")
            prev_sq, prev_ci = sq, ci

            # 거리 판정
            monitor.tick()
            verdict = monitor.verdict()

            if armed:
                if verdict.level == "stop":
                    # 장애물 감지 → 비상정지
                    disarm(); armed = False; print("충돌방지 estop!")
                else:
                    trig = rt - lt
                    if abs(trig) < 0.05:
                        trig = 0.0
                    axis.controller.input_vel = trig * VEL_LIMIT

            now = time.monotonic()
            if now - last_print > 1.0:
                dist_str = "(없음)" if verdict.distance_mm is None else f"{int(verdict.distance_mm)}mm"
                state = "ARMED" if armed else "IDLE"
                vel = axis.encoder.vel_estimate if armed else 0.0
                print(f"[{state}] 속도: {vel:.2f} turns/s | 거리: {dist_str} | 판정: {verdict.level}")
                last_print = now

            time.sleep(0.02)  # 50Hz

    except KeyboardInterrupt:
        pass
    finally:
        disarm()
        sensor.close()
        pygame.quit()
        print("종료")


if __name__ == "__main__":
    sys.path.insert(0, ".")
    main()
