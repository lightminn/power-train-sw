#!/usr/bin/env python3
"""BL70200 + ODrive v3.6 (fw 0.5.1) 셋업 — 읽기 / 최적값 적용 / 캘리.

axis1, HALL, 48V, CAN node 11. 검증된 최적 NVM 설정을 한 곳에 모았다.
(motor_gui --track usb 로 X2212 게인이 박혀 오염될 수 있어, --read 로 대조 후 --apply 권장.)

실행 (Jetson 컨테이너 /workspace, ODrive USB 연결):
  python3 motor_control/drive/bl70200/bl70200_setup.py --read       # 현재 NVM 출력
  python3 motor_control/drive/bl70200/bl70200_setup.py --apply      # 최적값 적용 + NVM 저장(리부팅)
  python3 motor_control/drive/bl70200/bl70200_setup.py --calibrate  # 풀캘리 (출력축 자유, ~55s 회전)

여러 개 동시 가능: --apply --calibrate. 새 보드는 --node 13 처럼 node_id 지정(충돌 방지).
문서: Notion "ODrive(BL70200) 셋업".
"""
import argparse
import time

# 검증된 최적 NVM 설정 (2026-06-25 초기값 / 2026-07-04 vel_gain 재튜닝)
# vel_gain 0.06→0.12: 자유회전(무부하 연속) 상태 다중 시나리오 스윕으로 재측정.
# ±1바퀴 제약을 벗어나니 gain을 더 올릴 수 있었고, 정속 리플 ~2×↓·가감속
# 오버슈트 ~4×↓·방향전환 ~6×↓ 개선(bw30/vg0.12/vi0.20 = 무부하 최적, 실차 부하 시 재확인).
CFG = dict(
    pole_pairs=10, current_lim=9.0, calibration_current=8.0, resistance_calib_max_voltage=5.0,
    torque_constant=0.353,                          # 역기전력법 실측 (기본 0.04은 placeholder)
    cpr=60, bandwidth=30.0, calib_scan_omega=6.0, calib_scan_distance=150.0,
    pos_gain=2.0, vel_gain=0.12, vel_integrator_gain=0.2, input_filter_bandwidth=2.0,
    vel_limit=50.0, vel_ramp_rate=2.0, uv=40.0, ov=56.0, brake=2.0, node=11, baud=500000,
    # marginal HALL 이 폐루프 회전 중 순간 illegal state(000/111) 읽으면 axis 0x100
    # ENCODER_FAILED 로 트립 → 직전 유효상태 유지로 트립 방지(밴드에이드; 근본은 HALL 접지/
    # 필터캡 HW 보강). 2026-07-04 6축 CAN 주행에서 node12 가 이 플래그 없어 트립해 추가.
    ignore_illegal_hall_state=True,
)


def read(ax, odrv):
    m, e, c = ax.motor.config, ax.encoder.config, ax.controller.config
    print("fw %d.%d.%d  vbus=%.1f" % (odrv.fw_version_major, odrv.fw_version_minor,
                                      odrv.fw_version_revision, odrv.vbus_voltage))
    print("motor   type=%s pp=%s current_lim=%.1f torque_constant=%.4f"
          % (m.motor_type, m.pole_pairs, m.current_lim, m.torque_constant))
    print("encoder mode=%s cpr=%s bw=%.0f calib_scan_omega=%.1f ignore_illegal_hall=%s"
          % (e.mode, e.cpr, e.bandwidth, e.calib_scan_omega, e.ignore_illegal_hall_state))
    print("ctrl    pos=%.2f vel=%.4f vel_int=%.2f ifbw=%.1f vel_limit=%.0f"
          % (c.pos_gain, c.vel_gain, c.vel_integrator_gain, c.input_filter_bandwidth, c.vel_limit))
    print("board   UV=%.0f OV=%.0f brake=%.1f | cal: motor=%s encoder=%s"
          % (odrv.config.dc_bus_undervoltage_trip_level, odrv.config.dc_bus_overvoltage_trip_level,
             odrv.config.brake_resistance, ax.motor.is_calibrated, ax.encoder.is_ready))


def _load_odrive():
    import odrive

    return odrive


def _load_enums():
    from odrive import enums

    return enums


def find_odrive(odrive_module, *, serial=None, timeout=20):
    """Find one board, selecting an exact serial when one is supplied."""

    kwargs = {"timeout": timeout}
    if serial is not None:
        kwargs["serial_number"] = serial
    return odrive_module.find_any(**kwargs)


def apply(ax, odrv, *, node=None, enums=None, save=True):
    enums = enums or _load_enums()
    node = CFG["node"] if node is None else node
    m, e, c = ax.motor.config, ax.encoder.config, ax.controller.config
    m.motor_type = enums.MOTOR_TYPE_HIGH_CURRENT
    m.pole_pairs = CFG["pole_pairs"]
    m.current_lim = CFG["current_lim"]
    m.calibration_current = CFG["calibration_current"]
    m.resistance_calib_max_voltage = CFG["resistance_calib_max_voltage"]
    m.torque_constant = CFG["torque_constant"]
    e.mode = enums.ENCODER_MODE_HALL
    e.cpr = CFG["cpr"]
    e.bandwidth = CFG["bandwidth"]
    e.calib_scan_omega = CFG["calib_scan_omega"]
    e.calib_scan_distance = CFG["calib_scan_distance"]
    e.ignore_illegal_hall_state = CFG["ignore_illegal_hall_state"]
    c.pos_gain = CFG["pos_gain"]
    c.vel_gain = CFG["vel_gain"]
    c.vel_integrator_gain = CFG["vel_integrator_gain"]
    c.input_filter_bandwidth = CFG["input_filter_bandwidth"]
    c.vel_limit = CFG["vel_limit"]
    c.vel_ramp_rate = CFG["vel_ramp_rate"]
    c.control_mode = enums.CONTROL_MODE_VELOCITY_CONTROL
    c.input_mode = enums.INPUT_MODE_VEL_RAMP
    odrv.config.dc_bus_undervoltage_trip_level = CFG["uv"]
    odrv.config.dc_bus_overvoltage_trip_level = CFG["ov"]
    odrv.config.brake_resistance = CFG["brake"]
    odrv.can.set_baud_rate(CFG["baud"])              # config.baud_rate 직접쓰기 불가
    try:
        ax.config.can_node_id = node
    except AttributeError:
        ax.config.can.node_id = node                 # 이 빌드 폴백
    ax.config.startup_motor_calibration = False      # 부팅 자동진입 금지
    ax.config.startup_encoder_offset_calibration = False
    ax.config.startup_closed_loop_control = False
    if save:
        odrv.save_configuration()                    # 리부팅 → 캘리 소실
        print("적용 + NVM 저장 완료 (리부팅됨 → --calibrate 로 재캘리)")


def calibrate(ax, *, enums=None):
    enums = enums or _load_enums()
    ax.error = ax.motor.error = ax.encoder.error = ax.controller.error = 0
    ax.motor.config.current_lim = 20.0               # 캘리 헤드룸
    print("FULL_CAL... (⚠️ 출력축 자유, ~55s 양방향 회전)")
    ax.requested_state = enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    t0 = time.time()
    while ax.current_state != enums.AXIS_STATE_IDLE:
        if time.time() - t0 > 120:
            print("  타임아웃")
            break
        time.sleep(0.5)
    ax.motor.config.current_lim = CFG["current_lim"]  # 운용값 복귀
    print("캘리: motor=%s encoder=%s err=%s (%.0fs)"
          % (ax.motor.is_calibrated, ax.encoder.is_ready, hex(ax.error), time.time() - t0))


def _selected_axes(odrv, axis):
    if axis == "0":
        return ((0, odrv.axis0),)
    if axis == "1":
        return ((1, odrv.axis1),)
    return ((0, odrv.axis0), (1, odrv.axis1))


def _axis_calibration_ok(ax):
    return (
        bool(ax.motor.is_calibrated)
        and bool(ax.encoder.is_ready)
        and int(getattr(ax, "error", 0)) == 0
        and int(getattr(ax.motor, "error", 0)) == 0
        and int(getattr(ax.encoder, "error", 0)) == 0
    )


def persist_calibration(odrv):
    """Persist both calibrated axes as one fw 0.5.1 board transaction.

    This function never initiates calibration.  It only checks the completed
    state, sets the supported pre-calibrated flags, and saves once.  HALL
    polarity state from newer firmware is intentionally not used as evidence.
    """

    axes = ((0, odrv.axis0), (1, odrv.axis1))
    failed = [axis for axis, ax in axes if not _axis_calibration_ok(ax)]
    if failed:
        raise ValueError(f"calibration not successful on axes: {failed}")

    for _, ax in axes:
        ax.motor.config.pre_calibrated = True
        ax.encoder.config.pre_calibrated = True
    odrv.save_configuration()


def verify_persisted_calibration(odrv):
    """Verify the non-rotating calibration flags after board re-enumeration."""

    failed = []
    for axis, ax in ((0, odrv.axis0), (1, odrv.axis1)):
        if not (
            bool(ax.motor.config.pre_calibrated)
            and bool(ax.encoder.config.pre_calibrated)
            and _axis_calibration_ok(ax)
        ):
            failed.append(axis)
    if failed:
        raise ValueError(f"persisted calibration verification failed on axes: {failed}")


def build_parser():
    ap = argparse.ArgumentParser(description="BL70200 ODrive 셋업 (fw 0.5.1)")
    ap.add_argument("--read", action="store_true", help="현재 NVM 설정 출력")
    ap.add_argument("--apply", action="store_true", help="최적값 적용 + 저장 (리부팅)")
    ap.add_argument("--calibrate", action="store_true", help="풀캘리 (출력축 자유)")
    ap.add_argument("--node", type=int, default=CFG["node"],
                    help="CAN node_id (기본 11; both이면 axis1은 다음 번호)")
    ap.add_argument("--serial", help="ODrive 보드 시리얼 (정확히 이 보드만 연결)")
    ap.add_argument("--axis", choices=("0", "1", "both"), default="1",
                    help="대상 축 (기본 1; 양축은 both)")
    ap.add_argument(
        "--persist-calibration",
        action="store_true",
        help="양축 성공 확인 후 pre_calibrated 저장 + 재열거 대조",
    )
    return ap


def parse_args(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.persist_calibration and not args.serial:
        parser.error("--persist-calibration requires --serial")
    return args


def run(args, *, odrive_module=None, sleep_fn=time.sleep):
    odrive_module = odrive_module or _load_odrive()
    odrv = find_odrive(odrive_module, serial=args.serial)

    if args.apply:
        enums = _load_enums()
        selected = _selected_axes(odrv, args.axis)
        for offset, (_, ax) in enumerate(selected):
            apply(ax, odrv, node=args.node + offset, enums=enums, save=False)
        odrv.save_configuration()
        print("적용 + NVM 저장 완료 (리부팅됨 → --calibrate 로 재캘리)")
        sleep_fn(8)
        odrv = find_odrive(odrive_module, serial=args.serial)

    if args.calibrate:
        enums = _load_enums()
        for _, ax in _selected_axes(odrv, args.axis):
            calibrate(ax, enums=enums)

    if args.persist_calibration:
        persist_calibration(odrv)
        sleep_fn(8)
        odrv = find_odrive(odrive_module, serial=args.serial)
        verify_persisted_calibration(odrv)
        axes_to_read = _selected_axes(odrv, "both")
    else:
        axes_to_read = _selected_axes(odrv, args.axis)

    for axis, ax in axes_to_read:
        print(f"=== axis{axis} ===")
        read(ax, odrv)
    return 0


def main(argv=None, *, odrive_module=None):
    return run(parse_args(argv), odrive_module=odrive_module)


if __name__ == "__main__":
    raise SystemExit(main())
