"""차체 DualSense 텔레옵: 게임패드 → (v, ω) → ChassisManager (10모터 4WS 수동주행).

corner_module.teleop_dualsense(코너 1개)의 차체 확장판. 조향 AK ×4 + 구동 ODrive ×6
을 컨트롤러 하나로 동시 제어한다. 입력을 (전진속도 v, 요레이트 ω)로 매핑하면
ChassisManager 가 4WS 키네마틱스로 10모터에 분배 — 애커만 선회·뒤축 역위상·차동이
자동으로 나온다. map_chassis_input 은 순수 함수라 단위 테스트한다.

조작 (Linux pygame, DualSense — corner 텔레옵과 동일 축/버튼):
  RT(R2, axis4) / LT(L2, axis3)  = 전진 / 후진 속도 v
  좌스틱 X (axis0)               = 회전 ω  (오른쪽=우회전)
  □(Square, btn0)                = arm / disarm 토글
  ○(Circle, btn2)                = estop (전 코너 정지)
  ▸ 트리거 없이(v=0) 좌스틱만 = **제자리 회전(피벗)**.

⚠️ 바퀴 지령 <0.3 rev/s(≈0.19 m/s)는 HALL 저속 코깅존이라 실제로 안 돎 — 살짝만
   당기면 버징만, 반쯤 이상 당겨야 굴러간다(--v-max 로 조절). 조향은 링키지 범위 내.

실행 (컨테이너 안, motor_control/ 에서 · can0 UP · DualSense 연결):
  python3 -m chassis.teleop_dualsense                 # US-100 충돌방지 ON (기본)
  python3 -m chassis.teleop_dualsense --no-us100      # US-100 없이 (구동 게이팅 OFF)
  옵션: --v-max 0.6  --omega-max 1.2  --channel can0
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


def main(argv=None):
    import argparse
    import os

    parser = argparse.ArgumentParser(description="차체(10모터 4WS) DualSense 텔레옵")
    parser.add_argument("--no-us100", action="store_true",
                        help="US-100 충돌방지 없이 (구동 게이팅 OFF — 장애물 자동정지 안 함)")
    parser.add_argument("--channel", default="can0", help="socketcan 채널 (기본 can0)")
    parser.add_argument("--v-max", type=float, default=0.6, help="최대 전진속도 m/s (기본 0.6)")
    parser.add_argument("--omega-max", type=float, default=1.2, help="최대 요레이트 rad/s (기본 1.2)")
    args = parser.parse_args(argv)
    use_us100 = not args.no_us100

    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

    import pygame
    from chassis.chassis_manager import ChassisManager, ChassisConfig, build_real_corners

    # US-100 충돌방지 (옵션) — ChassisManager 가 tick 내부에서 stop 판정 시 구동 게이팅.
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

    corners = build_real_corners(args.channel)
    cfg = ChassisConfig()
    cm = ChassisManager(corners, cfg, monitor=monitor)

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

    print("=== 차체 4WS 텔레옵 (10모터) ===")
    print("□: arm/disarm · ○: estop · RT/LT: 전/후진 · 좌스틱X: 회전 · (트리거0+스틱=피벗) · Ctrl-C: 종료")
    print("v_max=%.2f m/s · omega_max=%.2f rad/s · %s"
          % (args.v_max, args.omega_max,
             "US-100 ON (stop 200/warn 400mm)" if use_us100 else "⚠️ US-100 OFF"))

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

            if armed and cm.mode == "ARMED":
                v, omega = map_chassis_input(left_x, rt, lt, args.v_max, args.omega_max)
                cm.set(v, omega)

            cm.tick()                       # US-100 게이팅·estop 전파는 여기서 총괄
            if cm.mode == "FAULT":          # 코너 트립으로 자동 estop 되면 arm 해제
                armed = False

            now = time.monotonic()
            if now - last_print > 1.0:
                st = cm.state()
                print("mode=%s v=%+.2f ω=%+.2f verdict=%s"
                      % (st["mode"], st["v"], st["omega"], st["verdict"]))
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
