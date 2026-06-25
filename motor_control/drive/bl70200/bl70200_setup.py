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

import odrive
from odrive.enums import *

# 검증된 최적 NVM 설정 (2026-06-25, axis1)
CFG = dict(
    pole_pairs=10, current_lim=9.0, calibration_current=8.0, resistance_calib_max_voltage=5.0,
    torque_constant=0.353,                          # 역기전력법 실측 (기본 0.04은 placeholder)
    cpr=60, bandwidth=30.0, calib_scan_omega=6.0, calib_scan_distance=150.0,
    pos_gain=2.0, vel_gain=0.06, vel_integrator_gain=0.2, input_filter_bandwidth=2.0,
    vel_limit=50.0, vel_ramp_rate=2.0, uv=40.0, ov=56.0, brake=2.0, node=11, baud=500000,
)


def read(ax, odrv):
    m, e, c = ax.motor.config, ax.encoder.config, ax.controller.config
    print("fw %d.%d.%d  vbus=%.1f" % (odrv.fw_version_major, odrv.fw_version_minor,
                                      odrv.fw_version_revision, odrv.vbus_voltage))
    print("motor   type=%s pp=%s current_lim=%.1f torque_constant=%.4f"
          % (m.motor_type, m.pole_pairs, m.current_lim, m.torque_constant))
    print("encoder mode=%s cpr=%s bw=%.0f calib_scan_omega=%.1f" % (e.mode, e.cpr, e.bandwidth, e.calib_scan_omega))
    print("ctrl    pos=%.2f vel=%.4f vel_int=%.2f ifbw=%.1f vel_limit=%.0f"
          % (c.pos_gain, c.vel_gain, c.vel_integrator_gain, c.input_filter_bandwidth, c.vel_limit))
    print("board   UV=%.0f OV=%.0f brake=%.1f | cal: motor=%s encoder=%s"
          % (odrv.config.dc_bus_undervoltage_trip_level, odrv.config.dc_bus_overvoltage_trip_level,
             odrv.config.brake_resistance, ax.motor.is_calibrated, ax.encoder.is_ready))


def apply(ax, odrv):
    m, e, c = ax.motor.config, ax.encoder.config, ax.controller.config
    m.motor_type = MOTOR_TYPE_HIGH_CURRENT
    m.pole_pairs = CFG["pole_pairs"]
    m.current_lim = CFG["current_lim"]
    m.calibration_current = CFG["calibration_current"]
    m.resistance_calib_max_voltage = CFG["resistance_calib_max_voltage"]
    m.torque_constant = CFG["torque_constant"]
    e.mode = ENCODER_MODE_HALL
    e.cpr = CFG["cpr"]
    e.bandwidth = CFG["bandwidth"]
    e.calib_scan_omega = CFG["calib_scan_omega"]
    e.calib_scan_distance = CFG["calib_scan_distance"]
    c.pos_gain = CFG["pos_gain"]
    c.vel_gain = CFG["vel_gain"]
    c.vel_integrator_gain = CFG["vel_integrator_gain"]
    c.input_filter_bandwidth = CFG["input_filter_bandwidth"]
    c.vel_limit = CFG["vel_limit"]
    c.vel_ramp_rate = CFG["vel_ramp_rate"]
    c.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    c.input_mode = INPUT_MODE_VEL_RAMP
    odrv.config.dc_bus_undervoltage_trip_level = CFG["uv"]
    odrv.config.dc_bus_overvoltage_trip_level = CFG["ov"]
    odrv.config.brake_resistance = CFG["brake"]
    odrv.can.set_baud_rate(CFG["baud"])              # config.baud_rate 직접쓰기 불가
    try:
        ax.config.can_node_id = CFG["node"]
    except AttributeError:
        ax.config.can.node_id = CFG["node"]          # 이 빌드 폴백
    ax.config.startup_motor_calibration = False      # 부팅 자동진입 금지
    ax.config.startup_encoder_offset_calibration = False
    ax.config.startup_closed_loop_control = False
    odrv.save_configuration()                        # 리부팅 → 캘리 소실
    print("적용 + NVM 저장 완료 (리부팅됨 → --calibrate 로 재캘리)")


def calibrate(ax):
    ax.error = ax.motor.error = ax.encoder.error = ax.controller.error = 0
    ax.motor.config.current_lim = 20.0               # 캘리 헤드룸
    print("FULL_CAL... (⚠️ 출력축 자유, ~55s 양방향 회전)")
    ax.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    t0 = time.time()
    while ax.current_state != AXIS_STATE_IDLE:
        if time.time() - t0 > 120:
            print("  타임아웃")
            break
        time.sleep(0.5)
    ax.motor.config.current_lim = CFG["current_lim"]  # 운용값 복귀
    print("캘리: motor=%s encoder=%s err=%s (%.0fs)"
          % (ax.motor.is_calibrated, ax.encoder.is_ready, hex(ax.error), time.time() - t0))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="BL70200 ODrive 셋업 (axis1)")
    ap.add_argument("--read", action="store_true", help="현재 NVM 설정 출력")
    ap.add_argument("--apply", action="store_true", help="최적값 적용 + 저장 (리부팅)")
    ap.add_argument("--calibrate", action="store_true", help="풀캘리 (출력축 자유)")
    ap.add_argument("--node", type=int, default=CFG["node"],
                    help="CAN node_id (기본 11; 새 보드는 13 등 충돌 안 나게 지정)")
    a = ap.parse_args()
    CFG["node"] = a.node

    odrv = odrive.find_any(timeout=20)
    ax = odrv.axis1
    if a.apply:
        apply(ax, odrv)
        time.sleep(8)                                # 리부팅 대기
        odrv = odrive.find_any(timeout=20)
        ax = odrv.axis1
    if a.calibrate:
        calibrate(ax)
    read(ax, odrv)
