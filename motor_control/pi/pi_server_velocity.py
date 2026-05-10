"""
Pi에서 실행: ODrive 모터 제어 + 명령 수신 서버
실행: python3 robot_server.py

3계층 속도 제어:
  Layer 1  target_vel  : 소켓으로 수신한 DualSense 목표 속도 (즉각 반영)
  Layer 2  setpoint_vel: Python 소프트 램프 — MAX_ACCEL 이내로 target 추종 (50 Hz)
  Layer 3  motor_vel   : ODrive PASSTHROUGH — setpoint 직접 추종 (전류 제어기가 내부 처리)
"""
import logging
import math
import os
import socket
import threading
import time
import odrive
from odrive.enums import *

# ── 설정 ──────────────────────────────────────────
COMMAND_PORT = 9000
AXIS_NUM     = 1
MAX_VEL      = 4.0      # rev/s 최대 속도
MAX_ACCEL    = 2.0      # rev/s² Layer-2 소프트 램프 가속도
CONTROL_HZ   = 50       # 제어 루프 주기
LOG_HZ       = 10       # ODrive 상태 로그 주기 (CONTROL_HZ의 약수)

# 모터/엔코더 스펙 (14극 BLDC + TLE5012B)
POLE_PAIRS       = 7
ENCODER_CPR      = 16384
MOTOR_TYPE       = MotorType.HIGH_CURRENT
BRAKE_RESISTANCE = 2.0   # Ω — 없으면 None

LOG_DIR  = os.path.dirname(os.path.abspath(__file__))
LOG_FILE = os.path.join(LOG_DIR, "robot_server.log")
# ──────────────────────────────────────────────────

# ── 로거 설정 ─────────────────────────────────────

def setup_logger():
    logger = logging.getLogger("robot")
    logger.setLevel(logging.DEBUG)
    fmt = logging.Formatter("%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s",
                            datefmt="%H:%M:%S")
    # 파일 핸들러 (DEBUG 이상 전부)
    fh = logging.FileHandler(LOG_FILE, encoding="utf-8")
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(fmt)
    # 콘솔 핸들러 (INFO 이상만)
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    ch.setFormatter(fmt)
    logger.addHandler(fh)
    logger.addHandler(ch)
    return logger

log = setup_logger()

# ── 공유 상태 (thread-safe) ───────────────────────
_target_vel  = 0.0
_target_lock = threading.Lock()
_running     = True


def set_target(vel):
    global _target_vel
    with _target_lock:
        _target_vel = max(-MAX_VEL, min(MAX_VEL, float(vel)))

def get_target():
    with _target_lock:
        return _target_vel


# ── ODrive 유틸 ───────────────────────────────────

def dump_errors(drv, ax):
    log.error(f"  vbus:             {drv.vbus_voltage:.2f} V")
    log.error(f"  brake_armed:      {drv.brake_resistor_armed}")
    log.error(f"  axis.error:       {ax.error:#x}")
    log.error(f"  motor.error:      {ax.motor.error:#x}")
    log.error(f"  encoder.error:    {ax.encoder.error:#x}")
    log.error(f"  controller.error: {ax.controller.error:#x}")
    log.error(f"  axis.state:       {ax.current_state}")


def clear_errors(ax):
    ax.error = 0
    ax.motor.error = 0
    ax.encoder.error = 0
    ax.controller.error = 0


def apply_dc_protection(drv):
    try:
        if BRAKE_RESISTANCE is not None:
            drv.config.enable_brake_resistor  = True
            drv.config.brake_resistance       = BRAKE_RESISTANCE
        drv.config.dc_max_negative_current         = -3.0  # 회생전류 제한 → vbus 급락 방지
        drv.config.dc_bus_overvoltage_trip_level   = 16.0  # 회생 스파이크 여유
        drv.config.dc_bus_undervoltage_trip_level  = 8.0
    except AttributeError:
        pass


def apply_control_gains(ax):
    # R=0.1031Ω 저저항 모터 → 전류 보수적으로 제한해 FET 과열 방지
    ax.motor.config.current_lim              = 6.0
    ax.controller.config.vel_limit           = 6.0   # MAX_VEL × 1.5
    ax.encoder.config.bandwidth              = 15    # 코깅 노이즈 필터링
    ax.controller.config.vel_gain            = 0.10  # 코깅 댐핑 강화
    ax.controller.config.vel_integrator_gain = 0.005 # 정상상태 보정 (와인드업 최소)
    ax.controller.config.pos_gain            = 1.0
    ax.controller.config.control_mode        = ControlMode.VELOCITY_CONTROL
    ax.controller.config.input_mode          = InputMode.VEL_RAMP   # ODrive 내부 램프 (Layer 2 보조)
    ax.controller.config.vel_ramp_rate       = 10.0  # 빠른 응답, Python 램프가 1차 제한
    ax.controller.input_vel                  = 0.0


# ── 시작 시퀀스 ───────────────────────────────────

def find_odrive_device():
    log.info("ODrive 검색 중...")
    drv = odrive.find_any(timeout=30)
    if drv is None:
        raise RuntimeError("ODrive를 찾을 수 없습니다.")
    return drv


def do_reboot(drv):
    log.info("ODrive 재부팅 중...")
    try:
        drv.reboot()
    except Exception:
        pass
    log.info("  재부팅 대기 (5초)...")
    time.sleep(5)
    log.info("  재연결 중...")
    drv = find_odrive_device()
    log.info(f"  재연결 완료 — vbus: {drv.vbus_voltage:.2f}V  "
             f"brake_armed: {drv.brake_resistor_armed}")
    return drv


def wait_for_idle(ax, timeout=20, label=""):
    t0 = time.time()
    while ax.current_state != AxisState.IDLE:
        elapsed = time.time() - t0
        if elapsed > timeout:
            log.error(f"  {label} 타임아웃!")
            return False
        log.debug(f"  {label} 진행 중... {elapsed:.0f}s (state={ax.current_state})")
        time.sleep(0.3)
    return True


def do_calibration(drv, ax):
    ax.motor.config.pole_pairs                   = POLE_PAIRS
    ax.motor.config.motor_type                   = MOTOR_TYPE
    ax.motor.config.calibration_current          = 2.0
    ax.motor.config.resistance_calib_max_voltage = 4.0
    ax.encoder.config.cpr                        = ENCODER_CPR
    ax.encoder.config.calib_range                = 0.5  # CPR_POLEPAIRS_MISMATCH 억제
    clear_errors(ax)

    log.info("  FULL_CALIBRATION_SEQUENCE 시작...")
    ax.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
    if not wait_for_idle(ax, timeout=40, label="캘리브레이션"):
        dump_errors(drv, ax); return False
    if ax.error != 0 or not ax.motor.is_calibrated or not ax.encoder.is_ready:
        log.error("  캘리 실패"); dump_errors(drv, ax); return False

    log.info(f"  캘리 완료 — R={ax.motor.config.phase_resistance:.4f}Ω  "
             f"L={ax.motor.config.phase_inductance*1e6:.1f}µH")
    return True


def enter_closed_loop(drv, ax):
    apply_dc_protection(drv)
    apply_control_gains(ax)
    clear_errors(ax)
    ax.requested_state = AxisState.CLOSED_LOOP_CONTROL
    time.sleep(0.5)
    if ax.current_state != 8:
        log.error("폐루프 진입 실패"); dump_errors(drv, ax); return False

    time.sleep(0.5)
    if ax.error != 0:
        log.error("폐루프 진입 후 즉시 에러"); dump_errors(drv, ax); return False

    log.info(f"ODrive 준비 완료  "
             f"[current_lim={ax.motor.config.current_lim}A  "
             f"vel_gain={ax.controller.config.vel_gain}  "
             f"vel_limit={ax.controller.config.vel_limit}]")
    return True


def full_startup():
    drv = find_odrive_device()
    ax  = drv.axis1 if AXIS_NUM == 1 else drv.axis0
    log.info(f"FW: {drv.fw_version_major}.{drv.fw_version_minor}.{drv.fw_version_revision}  "
             f"vbus: {drv.vbus_voltage:.2f}V")
    drv = do_reboot(drv)
    ax  = drv.axis1 if AXIS_NUM == 1 else drv.axis0
    if not do_calibration(drv, ax):
        raise RuntimeError("캘리브레이션 실패")
    if not enter_closed_loop(drv, ax):
        raise RuntimeError("폐루프 진입 실패")
    return drv, ax


# ── 3계층 제어 루프 (별도 스레드) ─────────────────

def control_loop(drv_box, ax_box):
    """
    Layer 2 소프트 램프: MAX_ACCEL 이내로 setpoint 조정
    Layer 3 ODrive 출력: setpoint를 PASSTHROUGH로 직접 전달
    brake_armed=False 감지 시 자동 재부팅·재캘리 수행
    """
    setpoint   = 0.0
    dt         = 1.0 / CONTROL_HZ
    max_step   = MAX_ACCEL * dt
    log_every  = max(1, round(CONTROL_HZ / LOG_HZ))  # 상태 로그 주기 (스텝 수)
    tick       = 0

    while _running:
        t_start = time.monotonic()
        tick   += 1

        drv    = drv_box[0]
        ax     = ax_box[0]
        target = get_target()

        try:
            # ── 브레이크 저항 비활성: 펌웨어 보호 발동, 재부팅만 가능 ──
            if not drv.brake_resistor_armed:
                log.error("[ctrl] 브레이크 저항 비활성 → 재부팅 필요")
                set_target(0.0); setpoint = 0.0
                drv = do_reboot(drv)
                ax  = drv.axis1 if AXIS_NUM == 1 else drv.axis0
                drv_box[0] = drv; ax_box[0] = ax
                if do_calibration(drv, ax) and enter_closed_loop(drv, ax):
                    log.info("[ctrl] 재부팅 복구 완료")
                else:
                    log.error("[ctrl] 재부팅 복구 실패 — 10초 대기")
                    time.sleep(10)
                continue

            # ── 축 에러 또는 폐루프 이탈 ──
            if ax.error != 0 or ax.current_state != 8:
                log.error(f"[ctrl] ODrive 에러 — axis:{ax.error:#x}  "
                          f"motor:{ax.motor.error:#x}  "
                          f"enc:{ax.encoder.error:#x}  "
                          f"ctrl:{ax.controller.error:#x}  "
                          f"state:{ax.current_state}  "
                          f"vbus:{drv.vbus_voltage:.2f}V  "
                          f"brake:{drv.brake_resistor_armed}")
                set_target(0.0); setpoint = 0.0
                clear_errors(ax)
                time.sleep(0.1)
                ax.controller.input_vel = 0.0
                ax.requested_state = AxisState.CLOSED_LOOP_CONTROL
                time.sleep(0.5)
                if ax.current_state != 8:
                    log.warning("[ctrl] 재진입 실패 → 재캘리 시도")
                    if do_calibration(drv, ax) and enter_closed_loop(drv, ax):
                        log.info("[ctrl] 재캘리 복구 완료")
                    else:
                        log.error("[ctrl] 재캘리 복구 실패 — 10초 대기")
                        time.sleep(10)
                else:
                    log.info("[ctrl] 재진입 성공")
                continue

            # ── Layer 2: 소프트 램프 ──
            diff = target - setpoint
            if abs(diff) > 1e-6:
                setpoint += math.copysign(min(abs(diff), max_step), diff)

            # ── Layer 3: ODrive PASSTHROUGH ──
            ax.controller.input_vel = setpoint

            # ── 주기적 상태 로그 (DEBUG → 파일에만 기록) ──
            if tick % log_every == 0:
                vel_est   = ax.encoder.vel_estimate
                vel_sp    = ax.controller.vel_setpoint
                iq        = ax.motor.current_control.Iq_measured
                vi_torque = ax.controller.vel_integrator_torque
                vbus      = drv.vbus_voltage
                log.debug(f"[state] target={target:+.3f}  setpoint={setpoint:+.3f}  "
                          f"vel_sp={vel_sp:+.3f}  vel_est={vel_est:+.3f}  "
                          f"Iq={iq:+.3f}A  vi_torque={vi_torque:+.4f}  "
                          f"vbus={vbus:.2f}V  state={ax.current_state}")

        except Exception as e:
            log.exception(f"[ctrl] 예외: {e}")
            setpoint = 0.0
            time.sleep(0.5)

        elapsed = time.monotonic() - t_start
        remain  = dt - elapsed
        if remain > 0:
            time.sleep(remain)
        elif elapsed > dt * 1.5:
            log.debug(f"[ctrl] 루프 지연 {elapsed*1000:.1f}ms (목표 {dt*1000:.0f}ms)")


# ── 소켓 수신 루프 ────────────────────────────────

def main():
    global _running

    log.info(f"=== robot_server 시작  로그파일: {LOG_FILE} ===")
    log.info(f"설정: MAX_VEL={MAX_VEL}  MAX_ACCEL={MAX_ACCEL}  CONTROL_HZ={CONTROL_HZ}")

    drv, ax = full_startup()

    drv_box = [drv]
    ax_box  = [ax]

    ctrl = threading.Thread(target=control_loop, args=(drv_box, ax_box), daemon=True)
    ctrl.start()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', COMMAND_PORT))
    server.listen(1)
    log.info(f"명령 서버 대기 중 (포트 {COMMAND_PORT}) ...")

    try:
        while True:
            conn, addr = server.accept()
            log.info(f"클라이언트 연결: {addr}")
            buf = b''
            try:
                while True:
                    data = conn.recv(64)
                    if not data:
                        break
                    buf += data
                    while b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        try:
                            vel = float(line.decode().strip())
                            log.debug(f"[recv] target_vel={vel:+.3f}")
                            set_target(vel)
                        except ValueError:
                            pass
            except OSError:
                pass
            finally:
                set_target(0.0)
                conn.close()
                log.info("클라이언트 연결 해제 — 목표 속도 0")

    except KeyboardInterrupt:
        log.info("서버 종료 (Ctrl+C)")
    finally:
        _running = False
        set_target(0.0)
        time.sleep(0.3)
        try:
            ax_box[0].controller.input_vel = 0.0
            time.sleep(0.3)
            ax_box[0].requested_state = AxisState.IDLE
        except Exception:
            pass
        server.close()
        log.info("IDLE — 종료")


if __name__ == '__main__':
    main()
