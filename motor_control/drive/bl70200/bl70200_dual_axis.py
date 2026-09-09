#!/usr/bin/env python3
"""BL70200 듀얼축(M0=axis0, M1=axis1) — 양축 캘리 + 독립 위치 제어 데모.

한 ODrive v3.6 에 동일한 BL70200 2개. CAN node M0=11 / M1=12. 두 모터 완전 동일.
⚠️ HALL 캘리는 calib_scan_omega=6.0 필수(기본 12.566은 offset 캘리 깨짐). 캘리 반복 실패로
shadow_count 폭주(CPR_POLEPAIRS_MISMATCH)면 파라미터 말고 전원 OFF/ON + 식히기.

실행 (Jetson 컨테이너 /workspace, ODrive USB):
  python3 motor_control/drive/bl70200/bl70200_dual_axis.py --serial <SERIAL> --axis both --node 11 --calibrate  # 양축 풀캘리 (출력축 자유, 각 ~55s)
  python3 motor_control/drive/bl70200/bl70200_dual_axis.py --serial <SERIAL> --axis both --node 11 --demo       # 독립 위치 ±1바퀴 (단독/동시 반대)

문서: Notion "ODrive 듀얼축 — BL70200 모터 2개 동시 구동 (M0+M1)".
"""
import argparse
import time

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from chassis.usb_session import motor_session, validate_target, communication_snapshot


def calibrate(ax, name):
    from odrive.enums import AXIS_STATE_FULL_CALIBRATION_SEQUENCE, AXIS_STATE_IDLE
    ax.error = ax.motor.error = ax.encoder.error = ax.controller.error = 0
    ax.motor.config.calibration_current = 8.0
    ax.motor.config.current_lim = 20.0               # 캘리 헤드룸
    ax.encoder.config.calib_scan_omega = 6.0         # ★ 필수
    ax.encoder.config.calib_scan_distance = 150
    ax.encoder.config.calib_range = 0.05
    print("%s FULL_CAL... (~55s 회전)" % name)
    ax.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    t0 = time.time()
    while ax.current_state != AXIS_STATE_IDLE:
        if time.time() - t0 > 120:
            print("  타임아웃")
            break
        time.sleep(0.5)
    ax.motor.config.current_lim = 9.0                # 운용값 복귀
    print("  %s: motor=%s encoder=%s err=%s (%.0fs)"
          % (name, ax.motor.is_calibrated, ax.encoder.is_ready, hex(ax.error), time.time() - t0))


def demo(a0, a1):
    """독립 위치 제어: M0 단독 / M1 단독 / 동시 반대방향, 각 ±1바퀴(한 방향 ≤ 1바퀴)."""
    from odrive.enums import (CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_POS_FILTER,
                              AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE)
    for ax in (a0, a1):
        ax.encoder.config.ignore_illegal_hall_state = True   # 회전 중 간헐 트립 완화
        ax.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        ax.controller.config.input_mode = INPUT_MODE_POS_FILTER
    s0, s1 = a0.encoder.pos_estimate, a1.encoder.pos_estimate  # 점프 방지: 현재 위치로 시작
    a0.controller.input_pos = s0
    a1.controller.input_pos = s1
    a0.requested_state = a1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.3)
    print("state M0=%d M1=%d (8=폐루프)" % (a0.current_state, a1.current_state))

    def move(label, p0, p1):
        print(label)
        a0.controller.input_pos = p0
        a1.controller.input_pos = p1
        time.sleep(3)

    move("M0 단독 +1바퀴", s0 + 1.0, s1)
    move("  M0 복귀", s0, s1)
    move("M1 단독 +1바퀴", s0, s1 + 1.0)
    move("  M1 복귀", s0, s1)
    move("동시 M0 +1 / M1 -1 (반대방향)", s0 + 1.0, s1 - 1.0)
    move("  복귀", s0, s1)
    print("도달 M0=%.2f M1=%.2f | err M0=%s M1=%s"
          % (a0.encoder.pos_estimate, a1.encoder.pos_estimate, hex(a0.error), hex(a1.error)))
    a0.requested_state = a1.requested_state = AXIS_STATE_IDLE


def main(argv=None):
    ap = argparse.ArgumentParser(description="BL70200 듀얼축 벤치 (바퀴를 들고 실행)")
    ap.add_argument("--calibrate", action="store_true", help="양축 풀캘리 (출력축 자유)")
    ap.add_argument("--demo", action="store_true", help="독립 위치 제어 ±1바퀴")
    ap.add_argument("--serial", required=True, help="ODrive 보드 serial")
    ap.add_argument("--axis", choices=("both",), required=True, help="양축 선택 명시")
    ap.add_argument("--node", type=int, required=True, help="axis0 CAN node (axis1=node+1)")
    args = ap.parse_args(argv)
    try:
        validate_target(args.serial, args.axis, args.node)
    except ValueError as exc:
        ap.error(str(exc))
    with motor_session("bl70200_dual_axis"):
        import odrive
        odrv = odrive.find_any(serial_number=args.serial, timeout=20)
        snapshot = communication_snapshot(odrv, serial=args.serial, strict=True)
        if [snapshot[a]["node"] for a in ("axis0", "axis1")] != [args.node, args.node + 1]:
            raise ValueError("explicit nodes do not match board communication configuration")
        a0, a1 = odrv.axis0, odrv.axis1
        try:
            if args.calibrate:
                calibrate(a0, "M0/axis0")
                calibrate(a1, "M1/axis1")
            if args.demo:
                demo(a0, a1)
        finally:
            errors = []
            for ax in (a0, a1):
                try:
                    ax.requested_state = 1
                except Exception as exc:
                    errors.append(exc)
            if errors:
                raise RuntimeError(f"USB IDLE failed: {errors}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
