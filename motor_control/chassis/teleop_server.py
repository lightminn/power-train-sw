"""차체 4WS 무선 텔레옵 서버 (젯슨쪽). teleop_dualsense.py(유선)의 무선 분리판.

노트북 클라이언트(`laptop/laptop_client_chassis.py`)가 DualSense **raw 입력**을 TCP 로
보내면, 여기서 `map_chassis_input`→(v, ω)→`ChassisManager` 로 10모터 4WS 를 구동한다.
매핑·속도한계·min_drive 플로어·US-100 게이팅 등 로봇 튜닝은 전부 **서버쪽**에 있어
클라이언트는 범용(어느 소스든 raw 입력만 보내면 됨).

프로토콜 (클라→서버, newline-delimited): `"left_x rt lt sq ci\n"`
  left_x ∈[-1,1] 좌스틱X · rt/lt ∈[0,1] 트리거 · sq/ci ∈{0,1} □/○ 현재 버튼상태.
  서버가 □ rising→arm 토글, ○ rising→estop. 클라 끊기면 구동 0(+차체 워치독).
상태 회신 (서버→클라, ~4Hz): `"S <mode> <v> <ω>\n"` (예 `"S ARMED +1.50 -0.00"`) —
  클라가 상태줄에 표시해 FAULT/disarm 을 조종자가 즉시 알게 함. 옛 클라(수신 안 함)는
  소켓버퍼가 차면 회신만 중단하고 조종은 계속(하위호환).

구조: 수신 스레드(소켓)가 최신입력만 공유상태에 갱신 → 제어 스레드(50Hz)가 그걸 읽어
edge판정·map·ChassisManager.tick. ChassisManager 는 제어 스레드만 만짐(스레드 안전).

실행 (젯슨 컨테이너, motor_control/ 에서 · can0 UP · 6축 캘리):
  python3 -m chassis.teleop_server --no-us100
  옵션: --port 9000 --v-max 1.5 --omega-max 1.2 --min-rev 1.0 --channel can0
노트북: python3 laptop/laptop_client_chassis.py --host <젯슨IP>
"""
import socket
import threading
import time

from chassis.teleop_dualsense import map_chassis_input


def make_status_line(mode, v, omega):
    """서버→클라 상태 회신 한 줄: 'S ARMED +1.50 -0.72\\n'."""
    return "S %s %+.2f %+.2f\n" % (mode, v, omega)


def parse_input_line(text):
    """"left_x rt lt sq ci" → (left_x, rt, lt, sq, ci) 클램프됨, 또는 None."""
    parts = text.split()
    if len(parts) != 5:
        return None
    try:
        lx, rt, lt = float(parts[0]), float(parts[1]), float(parts[2])
        sq, ci = int(parts[3]), int(parts[4])
    except ValueError:
        return None
    lx = max(-1.0, min(1.0, lx))
    rt = max(0.0, min(1.0, rt))
    lt = max(0.0, min(1.0, lt))
    return lx, rt, lt, (1 if sq else 0), (1 if ci else 0)


def main(argv=None):
    import argparse

    p = argparse.ArgumentParser(description="차체 4WS 무선 텔레옵 서버")
    p.add_argument("--port", type=int, default=9000)
    p.add_argument("--no-us100", action="store_true",
                   help="US-100 충돌방지 없이 (구동 게이팅 OFF)")
    p.add_argument("--channel", default="can0")
    p.add_argument("--v-max", type=float, default=1.5)
    p.add_argument("--omega-max", type=float, default=1.2)
    p.add_argument("--min-rev", type=float, default=1.0,
                   help="최저 구동속도 turns/s (저속 코깅존 회피, 0=off)")
    args = p.parse_args(argv)
    use_us100 = not args.no_us100

    from chassis.chassis_manager import ChassisManager, ChassisConfig, build_real_corners
    from corner_module.can_watchdog import CanWatchdog

    CanWatchdog(args.channel).start()    # mttcan TX 웻지 자가복구 (데몬 스레드)

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
    cfg = ChassisConfig(min_drive_turns_per_s=args.min_rev)
    cfg.geometry.drive_limit_mps = max(args.v_max, cfg.geometry.drive_limit_mps)
    cm = ChassisManager(corners, cfg, monitor=monitor)
    cm.connect()

    shared = {"lx": 0.0, "rt": 0.0, "lt": 0.0, "sq": 0, "ci": 0, "rx_ms": None,
              "st_mode": "IDLE", "st_v": 0.0, "st_w": 0.0}   # ← 클라 상태회신용 스냅샷
    lock = threading.Lock()
    running = [True]

    def control_loop():
        prev_sq = prev_ci = 0
        armed = False
        period = 1.0 / cfg.loop_hz
        last_print = 0.0
        errs = 0
        while running[0]:
            t0 = time.monotonic()
            with lock:
                lx, rt, lt, sq, ci = shared["lx"], shared["rt"], shared["lt"], shared["sq"], shared["ci"]
            v_cmd = w_cmd = 0.0
            try:                                       # 버스 에러 등으로 스레드가 죽지 않게 감쌈
                if sq and not prev_sq:                # □ rising = arm/disarm 토글
                    if armed:
                        cm.disarm(); armed = False
                    else:
                        cm.arm(); armed = True
                if ci and not prev_ci:                # ○ rising = estop
                    cm.estop(); armed = False
                prev_sq, prev_ci = sq, ci
                if armed and cm.mode == "ARMED":
                    v_cmd, w_cmd = map_chassis_input(lx, rt, lt, args.v_max, args.omega_max)
                    cm.set(v_cmd, w_cmd)
                cm.tick()
                if cm.mode == "FAULT":
                    armed = False
            except Exception as e:                     # 제어루프 유지 — 주기적으로만 경고
                errs += 1
                if errs % 50 == 1:
                    print("[server] 제어 예외(%d회): %s" % (errs, e), flush=True)
                time.sleep(0.05)
            # 상태 스냅샷 (수신 스레드가 클라로 회신 — cm 은 이 스레드만 만짐)
            with lock:
                shared["st_mode"], shared["st_v"], shared["st_w"] = cm.mode, v_cmd, w_cmd
            now = time.monotonic()
            if now - last_print > 1.0:
                st = cm.state()
                print("[server] mode=%s v=%+.2f ω=%+.2f verdict=%s"
                      % (st["mode"], st["v"], st["omega"], st["verdict"]), flush=True)
                last_print = now
            dt = time.monotonic() - t0
            if period - dt > 0:
                time.sleep(period - dt)

    ctrl = threading.Thread(target=control_loop, daemon=True)
    ctrl.start()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", args.port))
    server.listen(1)
    print("=== 차체 4WS 무선 텔레옵 서버 — 포트 %d 대기 (%s) ===" % (args.port,
          "US-100 ON" if use_us100 else "US-100 OFF"), flush=True)
    print("노트북: python3 laptop/laptop_client_chassis.py --host <이 젯슨 IP>", flush=True)
    try:
        while True:
            conn, addr = server.accept()
            print("[server] 클라이언트 연결: %s" % (addr,), flush=True)
            conn.settimeout(0.25)                     # recv 를 짧게 끊어 상태회신 주기 확보
            buf = b""
            last_status = 0.0
            status_ok = True                          # 옛 클라(수신 안 함) 버퍼 차면 회신만 중단
            try:
                while True:
                    try:
                        data = conn.recv(256)
                        if not data:
                            break
                        buf += data
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            r = parse_input_line(line.decode(errors="ignore").strip())
                            if r:
                                with lock:
                                    shared["lx"], shared["rt"], shared["lt"], shared["sq"], shared["ci"] = r
                                    shared["rx_ms"] = time.monotonic() * 1000.0
                    except socket.timeout:
                        pass
                    now = time.monotonic()
                    if status_ok and now - last_status >= 0.25:   # ~4Hz 상태 회신
                        with lock:
                            st_line = make_status_line(shared["st_mode"], shared["st_v"], shared["st_w"])
                        try:
                            conn.send(st_line.encode())
                        except (socket.timeout, OSError):
                            status_ok = False
                        last_status = now
            except OSError:
                pass
            finally:
                with lock:                            # 끊김 → 입력 중립(구동 0). arm 유지, 차체 워치독도 보조.
                    shared["lx"] = shared["rt"] = shared["lt"] = 0.0
                    shared["sq"] = shared["ci"] = 0
                conn.close()
                print("[server] 클라이언트 해제 — 구동 0", flush=True)
    except KeyboardInterrupt:
        print("\n[server] 종료(Ctrl-C)", flush=True)
    finally:
        running[0] = False
        time.sleep(0.2)
        cm.disarm()
        cm.close()
        if sensor is not None:
            sensor.close()
        server.close()
        print("[server] IDLE — 종료", flush=True)


if __name__ == "__main__":
    import sys
    sys.path.insert(0, ".")
    main()
