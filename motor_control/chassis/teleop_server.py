"""차체 4WS 무선 텔레옵 서버 (젯슨쪽). teleop_dualsense.py(유선)의 무선 분리판.

노트북 클라이언트(`laptop/laptop_client_chassis.py`)가 DualSense **raw 입력**을 TCP 로
보내면, 여기서 `map_chassis_input`→(v, ω)→`ChassisManager` 로 10모터 4WS 를 구동한다.
매핑·속도한계·min_drive 플로어·US-100 게이팅 등 로봇 튜닝은 전부 **서버쪽**에 있어
클라이언트는 범용(어느 소스든 raw 입력만 보내면 됨).

프로토콜 (클라→서버, newline-delimited): `"left_x rt lt sq ci\n"`
  left_x ∈[-1,1] 좌스틱X · rt/lt ∈[0,1] 트리거 · sq/ci ∈{0,1} □/○ 현재 버튼상태.
  서버가 □ rising→reset/arm/disarm, ○ rising→estop. 클라 끊기면 구동 0(+차체 워치독).
상태 회신 (서버→클라, ~4Hz): `"S <mode> <v> <ω>\n"` (예 `"S ARMED +1.50 -0.00"`) —
  클라가 상태줄에 표시해 ESTOP/disarm 을 조종자가 즉시 알게 함. 옛 클라(수신 안 함)는
  소켓버퍼가 차면 회신만 중단하고 조종은 계속(하위호환).

구조: 수신 스레드(소켓)가 최신입력만 공유상태에 갱신 → 제어 스레드(50Hz)가 그걸 읽어
edge판정·map·ChassisManager.tick. ChassisManager 는 제어 스레드만 만짐(스레드 안전).

실행 (젯슨 컨테이너, motor_control/ 에서 · can0 UP · 6축 캘리):
  python3 -m chassis.teleop_server --diagnostic-direct-can --no-us100
  비대화형 확인: 위 명령에 --confirm-arm-stowed 추가
  옵션: --port 9000 --v-max 1.5 --omega-max 1.2 --min-rev 1.0 --channel can0
노트북: python3 laptop/laptop_client_chassis.py --host <젯슨IP>
"""
import math
import socket
import threading
import time

from chassis.teleop_dualsense import (
    cleanup_chassis_resources,
    handle_chassis_square,
    map_chassis_input,
    require_diagnostic_direct_can,
)

WIRELESS_RX_TIMEOUT_MS = 300.0
WIRELESS_NEUTRAL_DEADZONE = 0.05


def reset_wireless_input(state):
    state.update({
        "lx": 0.0,
        "rt": 0.0,
        "lt": 0.0,
        "sq": 0,
        "ci": 0,
        "rx_ms": None,
        "neutral_seen": False,
        "rx_seq": 0,
        "consumed_seq": 0,
    })


def _wireless_sample_is_neutral(sample):
    lx, rt, lt, sq, ci = sample
    return (
        abs(lx) < WIRELESS_NEUTRAL_DEADZONE
        and abs(rt) < WIRELESS_NEUTRAL_DEADZONE
        and abs(lt) < WIRELESS_NEUTRAL_DEADZONE
        and not sq
        and not ci
    )


def update_wireless_input(
    state,
    sample,
    now_ms,
    timeout_ms=WIRELESS_RX_TIMEOUT_MS,
):
    last_rx_ms = state.get("rx_ms")
    if (
        state.get("neutral_seen", False)
        and last_rx_ms is not None
        and now_ms - last_rx_ms > timeout_ms
    ):
        reset_wireless_input(state)
    lx, rt, lt, sq, ci = sample
    if not state.get("neutral_seen", False):
        state["rx_ms"] = now_ms
        if not _wireless_sample_is_neutral(sample):
            state.update({"lx": 0.0, "rt": 0.0, "lt": 0.0,
                          "sq": 0, "ci": 0})
            return False
        state["neutral_seen"] = True
    state.update({
        "lx": lx,
        "rt": rt,
        "lt": lt,
        "sq": sq,
        "ci": ci,
        "rx_ms": now_ms,
        "rx_seq": state.get("rx_seq", 0) + 1,
    })
    return True


def wireless_input_active(state, now_ms, timeout_ms=WIRELESS_RX_TIMEOUT_MS):
    rx_ms = state.get("rx_ms")
    if not state.get("neutral_seen", False) or rx_ms is None:
        return False
    if now_ms - rx_ms > timeout_ms:
        return False
    return True


def fresh_wireless_input(state, now_ms, timeout_ms=WIRELESS_RX_TIMEOUT_MS):
    rx_ms = state.get("rx_ms")
    if (
        state.get("neutral_seen", False)
        and rx_ms is not None
        and now_ms - rx_ms > timeout_ms
    ):
        reset_wireless_input(state)
        return None
    if not wireless_input_active(state, now_ms, timeout_ms):
        return None
    rx_seq = state.get("rx_seq", 0)
    if rx_seq == state.get("consumed_seq", 0):
        return None
    state["consumed_seq"] = rx_seq
    return tuple(state[name] for name in ("lx", "rt", "lt", "sq", "ci"))


def apply_wireless_command(manager, sample, v_max, omega_max):
    if sample is None or manager.mode != "ARMED":
        return 0.0, 0.0, False
    lx, rt, lt, _sq, _ci = sample
    v_cmd, w_cmd = map_chassis_input(lx, rt, lt, v_max, omega_max)
    manager.set(v_cmd, w_cmd)
    return v_cmd, w_cmd, True


def _safe_exception_detail(exc):
    try:
        message = str(exc)
    except BaseException:
        message = "<unprintable>"
    return f"{type(exc).__name__}: {message}"


def _best_effort_estop(manager, source, detail):
    try:
        manager.estop(source, detail)
        return None
    except BaseException as exc:
        return exc


def run_control_thread(step, stop_event, manager, failure, failed_event):
    try:
        while not stop_event.is_set():
            step()
    except BaseException as exc:
        detail = _safe_exception_detail(exc)
        failure["exception"] = exc
        failure["detail"] = detail
        stop_event.set()
        _best_effort_estop(manager, "control_exception", detail)
        failed_event.set()
    finally:
        _best_effort_estop(
            manager,
            "control_thread_exit",
            "wireless control thread stopped",
        )


def control_thread_failure(thread, failure):
    detail = failure.get("detail")
    if detail is not None:
        return detail
    if thread is None or thread.is_alive():
        return None
    return "control thread stopped unexpectedly"


def shutdown_control_resources(
    thread,
    stop_event,
    manager,
    background,
    sensor,
    join_timeout_s=1.0,
):
    stop_event.set()
    if thread is not None and thread.ident is not None:
        thread.join(timeout=join_timeout_s)
    if thread is not None and thread.is_alive():
        _best_effort_estop(
            manager,
            "shutdown_timeout",
            "wireless control thread did not stop",
        )
        return False, [RuntimeError("wireless control thread still running")]
    return True, cleanup_chassis_resources(manager, background, sensor)


def shutdown_control_resources_eventually(
    thread,
    stop_event,
    manager,
    background,
    sensor,
    join_timeout_s=1.0,
):
    stopped, errors = shutdown_control_resources(
        thread,
        stop_event,
        manager,
        background,
        sensor,
        join_timeout_s=join_timeout_s,
    )
    if stopped:
        return True, errors

    # No shared resource is closed above while the control owner is alive.
    # The non-daemon owner must eventually exit before one cleanup retry.
    thread.join()
    stopped, retry_errors = shutdown_control_resources(
        thread,
        stop_event,
        manager,
        background,
        sensor,
        join_timeout_s=0.0,
    )
    return stopped, errors + retry_errors


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
    if not all(math.isfinite(value) for value in (lx, rt, lt, sq, ci)):
        return None
    lx = max(-1.0, min(1.0, lx))
    rt = max(0.0, min(1.0, rt))
    lt = max(0.0, min(1.0, lt))
    return lx, rt, lt, (1 if sq else 0), (1 if ci else 0)


def _parse_args(argv=None, input_fn=None):
    import argparse

    p = argparse.ArgumentParser(description="차체 4WS 무선 텔레옵 서버")
    p.add_argument(
        "--diagnostic-direct-can",
        action="store_true",
        help="legacy direct-CAN diagnostic tool임을 명시",
    )
    p.add_argument(
        "--confirm-arm-stowed",
        action="store_true",
        help="비대화형 환경에서 로봇팔의 기계적 접힘·고정을 확인",
    )
    p.add_argument("--port", type=int, default=9000)
    p.add_argument("--no-us100", action="store_true",
                   help="US-100 충돌방지 없이 (구동 게이팅 OFF)")
    p.add_argument("--channel", default="can0")
    p.add_argument("--v-max", type=float, default=1.5)
    p.add_argument("--omega-max", type=float, default=1.2)
    p.add_argument("--four-wheel", action="store_true",
                   help="🛠️ 중륜 2개(ODrive node 13/14) 없이 4륜만으로 돌린다. "
                        "중간 보드를 부하모터(다이나모)에 쓸 때. "
                        "⚠️ 임시 구성 — 바퀴 띄운 벤치용")
    p.add_argument("--min-rev", type=float, default=1.0,
                   help="최저 구동속도 turns/s (저속 코깅존 회피, 0=off)")
    p.add_argument("--friction-ff", type=float, default=0.0,
                   help="저속 마찰/코깅 보상 torque_ff (raw 단위, 0=off — 스펙 r6 §2.2b)")
    p.add_argument("--v-knee", type=float, default=0.5,
                   help="friction-ff 적용 상한 turns/s (기본 0.5)")
    args = p.parse_args(argv)
    require_diagnostic_direct_can(p, args, input_fn=input_fn)
    return args


def main(argv=None):
    args = _parse_args(argv)
    use_us100 = not args.no_us100

    from chassis.chassis_manager import ChassisManager, ChassisConfig, build_real_corners
    from chassis.runtime_lock import RealCanSession
    from corner_module.can_watchdog import CanWatchdog

    CanWatchdog(args.channel).start()    # mttcan TX 웻지 자가복구 (데몬 스레드)

    background = None
    sensor = None
    cm = None
    can_session = RealCanSession(
        channel=args.channel,
        owner="teleop_server",
    )
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

        can_session.__enter__()
        wheel_map = None
        if args.four_wheel:
            from chassis.chassis_manager import FOUR_WHEEL_MAP
            wheel_map = FOUR_WHEEL_MAP
            print("🛠️ 4륜 모드 — 중륜(node 13/14) 없이 앞뒤 4륜만 구동한다 (임시 구성)")
        corners = build_real_corners(
            args.channel, wheel_map=wheel_map,
            friction_ff=args.friction_ff, v_knee_turns_s=args.v_knee,
        )

        cfg = ChassisConfig(min_drive_turns_per_s=args.min_rev)
        if args.four_wheel:
            # ★ 기하와 매핑은 **반드시 짝**이어야 한다 (이름이 어긋나면 KeyError)
            from chassis.kinematics import four_wheel_geometry
            cfg.geometry = four_wheel_geometry()
        cfg.geometry.drive_limit_mps = max(
            args.v_max,
            cfg.geometry.drive_limit_mps,
        )
        cm = ChassisManager(corners, cfg)
        cm.connect()
    except BaseException as exc:
        if cm is not None and not isinstance(exc, KeyboardInterrupt):
            _best_effort_estop(
                cm,
                "control_exception",
                _safe_exception_detail(exc),
            )
        cleanup_chassis_resources(cm, background, sensor)
        can_session.close()
        raise

    shared = {"lx": 0.0, "rt": 0.0, "lt": 0.0, "sq": 0, "ci": 0, "rx_ms": None,
              "neutral_seen": False, "st_mode": "IDLE",
              "st_v": 0.0, "st_w": 0.0}   # ← 클라 상태회신용
    lock = threading.Lock()
    stop_event = threading.Event()
    control_failure = {}
    control_failed = threading.Event()

    prev_sq = prev_ci = 0
    period = 1.0 / cfg.loop_hz
    last_print = 0.0
    verdict = None

    def control_step():
        nonlocal prev_sq, prev_ci, last_print, verdict
        t0 = time.monotonic()
        with lock:
            input_active = wireless_input_active(
                shared,
                now_ms=t0 * 1000.0,
            )
            sample = fresh_wireless_input(
                shared,
                now_ms=t0 * 1000.0,
            )
        v_cmd = w_cmd = 0.0

        if background is not None:
            verdict = background.verdict()
            cm.update_external_safety(
                verdict.status,
                verdict.estop_required,
                verdict.detail,
            )

        if sample is None and not input_active:
            prev_sq = prev_ci = 0
        elif sample is not None:
            _lx, _rt, _lt, sq, ci = sample
            if sq and not prev_sq:
                if not handle_chassis_square(cm):
                    print("[server] □ 요청 거부 (mode=%s)" % cm.mode,
                          flush=True)
            if ci and not prev_ci:
                cm.estop("manual", "dualsense")
            prev_sq, prev_ci = sq, ci
            v_cmd, w_cmd, _applied = apply_wireless_command(
                cm,
                sample,
                args.v_max,
                args.omega_max,
            )

        # stale/no input intentionally skips set(); manager watchdog owns hold.
        cm.tick()
        with lock:
            shared["st_mode"], shared["st_v"], shared["st_w"] = (
                cm.mode,
                v_cmd,
                w_cmd,
            )
        now = time.monotonic()
        if now - last_print > 1.0:
            st = cm.state()
            status = verdict.status if verdict is not None else "DISABLED"
            print("[server] mode=%s v=%+.2f ω=%+.2f safety=%s status=%s"
                  % (st["mode"], st["v"], st["omega"],
                     st["safety"].state, status), flush=True)
            last_print = now
        remaining = period - (time.monotonic() - t0)
        if remaining > 0.0:
            stop_event.wait(remaining)

    def control_loop():
        run_control_thread(
            control_step,
            stop_event,
            cm,
            control_failure,
            control_failed,
        )

    ctrl = None
    server = None
    try:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", args.port))
        server.listen(1)
        server.settimeout(0.25)
        ctrl = threading.Thread(target=control_loop, daemon=False)
        ctrl.start()
    except BaseException as exc:
        if server is not None:
            try:
                server.close()
            except BaseException:
                pass
        if not isinstance(exc, KeyboardInterrupt):
            _best_effort_estop(
                cm,
                "control_exception",
                _safe_exception_detail(exc),
            )
        shutdown_control_resources_eventually(
            ctrl,
            stop_event,
            cm,
            background,
            sensor,
        )
        can_session.close()
        raise
    print("=== 차체 4WS 무선 텔레옵 서버 — 포트 %d 대기 (%s) ===" % (args.port,
          "US-100 ON" if use_us100 else "US-100 OFF"), flush=True)
    print("노트북: python3 laptop/laptop_client_chassis.py --host <이 젯슨 IP>", flush=True)
    try:
        while True:
            failure_detail = control_thread_failure(ctrl, control_failure)
            if failure_detail is not None:
                raise RuntimeError(
                    "wireless control unavailable: " + failure_detail
                )
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            print("[server] 클라이언트 연결: %s" % (addr,), flush=True)
            conn.settimeout(0.25)                     # recv 를 짧게 끊어 상태회신 주기 확보
            with lock:
                reset_wireless_input(shared)
            buf = b""
            last_status = 0.0
            status_ok = True                          # 옛 클라(수신 안 함) 버퍼 차면 회신만 중단
            try:
                while True:
                    failure_detail = control_thread_failure(
                        ctrl,
                        control_failure,
                    )
                    if failure_detail is not None:
                        raise RuntimeError(
                            "wireless control unavailable: " + failure_detail
                        )
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
                                    update_wireless_input(
                                        shared,
                                        r,
                                        time.monotonic() * 1000.0,
                                    )
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
                with lock:
                    reset_wireless_input(shared)
                conn.close()
                print("[server] 클라이언트 해제 — 구동 0", flush=True)
    except KeyboardInterrupt:
        print("\n[server] 종료(Ctrl-C)", flush=True)
    finally:
        try:
            server.close()
        except BaseException:
            pass
        stopped, errors = shutdown_control_resources_eventually(
            ctrl,
            stop_event,
            cm,
            background,
            sensor,
        )
        if not stopped:
            print("[server] 제어 스레드 생존 — 공유 자원 close 보류",
                  flush=True)
        if errors:
            print("[server] 정리 예외 %d건: %s" % (len(errors), errors[0]),
                  flush=True)
        can_session.close()
        print("[server] IDLE — 종료", flush=True)


if __name__ == "__main__":
    import sys
    sys.path.insert(0, ".")
    main()
