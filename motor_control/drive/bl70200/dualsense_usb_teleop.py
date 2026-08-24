#!/usr/bin/env python3
"""젯슨에서 실행: 노트북 DualSense RT/LT → BL70200 구동 (ODrive **USB** 직결, 인터록 없음).

⚠️ **US-100 충돌방지·차체 워치독·estop·안전토픽 게이팅이 전혀 없다.** 트리거가 그대로
바퀴 속도로 나간다. 반드시 **바퀴를 들어놓고** 쓸 것. 벤치 점검 전용 — 실차 주행용 아님.

can0 이 아니라 USB 직결이라 CAN 버스 상태(ERROR-PASSIVE·조향 부재)와 무관하게 돈다.
조향(AK)은 손대지 않는다. 구동 6축만.

배치: DualSense 는 노트북, ODrive 는 젯슨 USB. 노트북 클라는 기존 것을 그대로 쓴다
(프로토콜 `"lx rt lt sq ci\n"`, lx 는 이 서버가 무시).

  노트북: python3 motor_control/laptop/laptop_client_chassis.py --host 192.168.50.98 --port 9010
  젯슨  : docker exec -it powertrain_canwatchdog python3 \
            /workspace/motor_control/drive/bl70200/dualsense_usb_teleop.py

  전원 사이클 직후엔 캘리(RAM-only)부터:  ... dualsense_usb_teleop.py --calibrate
  벤치 전원이 약해 동시 arm 시 UV 트립 나면: ... --current-lim 2.0

조작: RT=전진 / LT=후진 (깊이 비례) · 손 떼면 정지 · □=disarm/재arm · ○=정지 후 종료.

링크 끊김·무입력 0.3s 는 속도 0 으로 떨군다 (안전 인터록이 아니라 폭주 방지 최소 장치).
"""
import argparse
import selectors
import socket
import sys
import time

GEAR_RATIO = 5.0          # 모터 5회전 = 바퀴 1회전
ODRIVE_VID = 0x1209
ODRIVE_PID = 0x0D32
DEFAULT_PORT = 9010       # 9000 은 teleop_command 노드가 쓰고 있음


# ---------------------------------------------------------------- ODrive USB

def discover_serials():
    """USB 에 붙은 ODrive 시리얼 목록. 실패하면 빈 리스트(→ find_any 폴백)."""
    try:
        import usb.core
        import usb.util
    except Exception as exc:
        print("pyusb 없음 (%s) — 단일 보드 폴백" % exc)
        return []
    serials = []
    try:
        for dev in usb.core.find(find_all=True, idVendor=ODRIVE_VID, idProduct=ODRIVE_PID):
            try:
                sn = usb.util.get_string(dev, dev.iSerialNumber)
            except Exception:
                sn = None
            if sn:
                serials.append(sn.strip())
    except Exception as exc:
        print("USB 열거 실패 (%s) — 단일 보드 폴백" % exc)
        return []
    return sorted(set(serials))


def connect(serials):
    import odrive
    boards = []
    if serials:
        for sn in serials:
            print("🔌 ODrive %s 연결 중..." % sn)
            boards.append((sn, odrive.find_any(serial_number=sn, timeout=20)))
    else:
        print("🔌 ODrive 검색 중 (아무 보드 1장)...")
        odrv = odrive.find_any(timeout=20)
        boards.append((str(odrv.serial_number), odrv))
    for sn, odrv in boards:
        print("✅ 보드 %s  vbus=%.1fV" % (sn, odrv.vbus_voltage))
    return boards


def collect_axes(boards, which, invert_axis1):
    """(라벨, axis, 부호). axis1 = 로봇 우측 = 전진 시 좌측과 반대 부호."""
    out = []
    for sn, odrv in boards:
        for idx in (0, 1):
            if which != "both" and int(which) != idx:
                continue
            sign = -1.0 if (idx == 1 and invert_axis1) else 1.0
            out.append(("%s/ax%d" % (sn[-6:], idx), getattr(odrv, "axis%d" % idx), sign))
    return out


def calibrate(ax, label, current_lim):
    from odrive.enums import AXIS_STATE_FULL_CALIBRATION_SEQUENCE, AXIS_STATE_IDLE
    ax.error = ax.motor.error = ax.encoder.error = ax.controller.error = 0
    ax.motor.config.calibration_current = 8.0
    ax.motor.config.current_lim = 20.0            # 캘리 헤드룸
    ax.encoder.config.calib_scan_omega = 6.0      # ★ 필수 (기본 12.566 은 offset 캘리 깨짐)
    ax.encoder.config.calib_scan_distance = 150
    ax.encoder.config.calib_range = 0.05
    print("⚙️  %s FULL_CAL... (~55s 회전, 출력축 자유로워야 함)" % label)
    ax.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    t0 = time.time()
    while ax.current_state != AXIS_STATE_IDLE:
        if time.time() - t0 > 120:
            print("   타임아웃")
            break
        time.sleep(0.5)
    ax.motor.config.current_lim = current_lim
    print("   %s motor=%s encoder=%s err=%s (%.0fs)"
          % (label, ax.motor.is_calibrated, ax.encoder.is_ready, hex(ax.error), time.time() - t0))


def arm(ax, label, current_lim):
    """속도 폐루프 진입. 점프 방지로 input_vel=0 을 먼저 박는다."""
    from odrive.enums import (AXIS_STATE_CLOSED_LOOP_CONTROL, CONTROL_MODE_VELOCITY_CONTROL,
                              INPUT_MODE_PASSTHROUGH)
    if not ax.motor.is_calibrated or not ax.encoder.is_ready:
        sys.exit("❌ %s 미캘리. 전원 사이클 후엔 --calibrate 를 먼저 돌려라 (캘리는 RAM-only)." % label)
    ax.error = ax.motor.error = ax.encoder.error = ax.controller.error = 0
    ax.motor.config.current_lim = current_lim
    ax.encoder.config.ignore_illegal_hall_state = True
    ax.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    ax.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    ax.controller.input_vel = 0.0
    ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.3)
    if ax.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        sys.exit("❌ %s 폐루프 진입 실패 (state=%d err=%s)" % (label, ax.current_state, hex(ax.error)))
    print("🔒 %s 폐루프 (current_lim %.1fA)" % (label, current_lim))


def set_all(axes, vel):
    for _, ax, sign in axes:
        try:
            ax.controller.input_vel = vel * sign
        except Exception as exc:
            print("\ninput_vel 실패: %s" % exc)


def idle_all(axes):
    from odrive.enums import AXIS_STATE_IDLE
    set_all(axes, 0.0)
    time.sleep(0.2)
    for lbl, ax, _ in axes:
        try:
            ax.requested_state = AXIS_STATE_IDLE
        except Exception as exc:
            print("%s IDLE 실패: %s" % (lbl, exc))


# ---------------------------------------------------------------- TCP 서버

def serve(axes, args):
    """클라 1대 접속 → RT/LT 수신 → 속도 지령. 끊기면 0 으로 떨구고 재대기."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", args.port))
    srv.listen(1)
    print("\n📡 TCP :%d 대기 — 노트북에서 laptop_client_chassis.py --host <젯슨IP> --port %d"
          % (args.port, args.port))

    sel = selectors.DefaultSelector()
    vel = 0.0
    dt = 0.02
    armed = not args.no_auto_arm
    neutral_seen = False       # 시작 직후 트리거 오독으로 튀는 것 방지
    prev_sq = 0

    while True:
        conn, addr = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn.setblocking(False)
        sel.register(conn, selectors.EVENT_READ)
        print("\n🎮 접속: %s:%d  (armed=%s)" % (addr[0], addr[1], armed))
        buf = b""
        rt = lt = 0.0
        last_rx = time.monotonic()
        neutral_seen = False
        try:
            while True:
                for _key, _ev in sel.select(timeout=dt):
                    try:
                        chunk = conn.recv(4096)
                    except (BlockingIOError, InterruptedError):
                        continue
                    except OSError:
                        chunk = b""
                    if not chunk:
                        raise ConnectionResetError
                    buf += chunk
                    *lines, buf = buf.split(b"\n")
                    for raw in lines[-4:]:              # 밀린 프레임은 최신 것만
                        parts = raw.decode(errors="ignore").split()
                        if len(parts) < 5:
                            continue
                        try:
                            _lx = float(parts[0])
                            rt, lt = float(parts[1]), float(parts[2])
                            sq, ci = int(float(parts[3])), int(float(parts[4]))
                        except ValueError:
                            continue
                        last_rx = time.monotonic()
                        if ci:
                            print("\n○ 정지·종료 요청")
                            return
                        if sq and not prev_sq:
                            armed = not armed
                            print("\n□ armed=%s" % armed)
                        prev_sq = sq

                rt = min(max(rt, 0.0), 1.0)
                lt = min(max(lt, 0.0), 1.0)
                if not neutral_seen:
                    if rt < 0.02 and lt < 0.02:
                        neutral_seen = True            # 중립 한 번 확인 후에야 명령 허용
                    target = 0.0
                elif time.monotonic() - last_rx > args.timeout:
                    target = 0.0                       # 링크 무입력 → 0
                elif not armed:
                    target = 0.0
                else:
                    target = (rt - lt) * args.max_vel

                step = args.accel * dt                 # 슬루 제한
                vel += max(-step, min(step, target - vel))
                set_all(axes, vel)

                try:
                    conn.send(("S %s %.2f 0\n" % ("RUN" if armed else "DISARM", vel)).encode())
                except (BlockingIOError, OSError):
                    pass
                print("\r🎮 RT %.2f LT %.2f | %s → 모터 %+5.2f t/s (바퀴 %+5.2f rev/s)   "
                      % (rt, lt, "ARMED " if armed else "DISARM", vel, vel / GEAR_RATIO), end="")
        except (ConnectionResetError, BrokenPipeError, OSError):
            print("\n🔌 링크 끊김 — 속도 0, 재접속 대기")
        finally:
            sel.unregister(conn)
            conn.close()
            vel = 0.0
            set_all(axes, 0.0)


def main():
    ap = argparse.ArgumentParser(description="DualSense(노트북) → BL70200 USB 구동 서버 (인터록 없음)")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help="수신 TCP 포트 (기본 9010)")
    ap.add_argument("--serial", action="append", default=[],
                    help="쓸 ODrive 시리얼 (여러 번). 생략 시 USB 에 붙은 보드 전부 자동")
    ap.add_argument("--axis", choices=["both", "0", "1"], default="both", help="쓸 축 (기본 both)")
    ap.add_argument("--max-vel", type=float, default=3.0,
                    help="트리거 만땅 시 모터 turns/s (기본 3.0 = 바퀴 0.6 rev/s)")
    ap.add_argument("--accel", type=float, default=8.0, help="속도 슬루 제한 turns/s^2")
    ap.add_argument("--current-lim", type=float, default=9.0,
                    help="축당 전류 제한 A (기본 9.0). 전원 약해 UV 트립 나면 2.0")
    ap.add_argument("--timeout", type=float, default=0.3, help="무입력 시 0 으로 떨구는 시간 (s)")
    ap.add_argument("--calibrate", action="store_true", help="구동 전 풀캘리 (전원 사이클마다 필요)")
    ap.add_argument("--no-invert-axis1", action="store_true",
                    help="axis1 부호 반전 끄기 (모터 프레임 그대로)")
    ap.add_argument("--no-auto-arm", action="store_true", help="시작 시 disarm 상태로 (□ 로 arm)")
    args = ap.parse_args()

    serials = args.serial or discover_serials()
    if serials:
        print("USB ODrive: %s" % ", ".join(serials))
    boards = connect(serials)
    axes = collect_axes(boards, args.axis, not args.no_invert_axis1)
    print("대상 축: " + ", ".join("%s(%+d)" % (l, s) for l, _, s in axes))

    if args.calibrate:
        for lbl, ax, _ in axes:
            calibrate(ax, lbl, args.current_lim)

    for lbl, ax, _ in axes:
        arm(ax, lbl, args.current_lim)

    print("\n⚠️  인터록 없음 — 바퀴 들렸는지 확인했나?")
    try:
        serve(axes, args)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n🛑 정지 후 IDLE 복귀")
        idle_all(axes)
        print("✅ 종료")


if __name__ == "__main__":
    main()
