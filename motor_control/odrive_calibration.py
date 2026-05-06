import time
import odrive
from odrive.enums import *


def connect():
    print("ODrive 검색 중...")
    drv = odrive.find_any()
    print(f"연결 성공: {drv.serial_number}")
    return drv


def dump_errors(drv):
    a = drv.axis1
    print(f"  axis.error:       {a.error:#x}")
    print(f"  motor.error:      {a.motor.error:#x}")
    print(f"  encoder.error:    {a.encoder.error:#x}")
    print(f"  controller.error: {a.controller.error:#x}")


def clear_errors(axis):
    axis.error = 0
    axis.motor.error = 0
    axis.encoder.error = 0
    axis.controller.error = 0


def wait_idle(axis, timeout=120):
    t0 = time.time()
    while axis.current_state != AxisState.IDLE:
        if time.time() - t0 > timeout:
            return False
        time.sleep(0.1)
    return True


def safe_set(obj, attr, value):
    try:
        setattr(obj, attr, value)
        return True
    except AttributeError:
        return False


POLE_PAIRS = 5

my_drive = connect()
axis = my_drive.axis1

print(f"FW: {my_drive.fw_version_major}.{my_drive.fw_version_minor}.{my_drive.fw_version_revision}")
print(f"vbus: {my_drive.vbus_voltage:.2f} V")
print(
    f"brake_armed: {my_drive.brake_resistor_armed}, saturated: {my_drive.brake_resistor_saturated}")

if my_drive.vbus_voltage < 8.0:
    print("DC 전원 미연결")
    exit()

# === 핵심 값 강제 검증 (NVRAM 잔재 무시, 매번 갱신) ===
need_save = False
if axis.encoder.config.mode != EncoderMode.HALL:
    axis.encoder.config.mode = EncoderMode.HALL
    need_save = True
if axis.motor.config.pole_pairs != POLE_PAIRS:
    axis.motor.config.pole_pairs = POLE_PAIRS
    need_save = True
if axis.encoder.config.cpr != 6 * POLE_PAIRS:
    axis.encoder.config.cpr = 6 * POLE_PAIRS
    need_save = True
if axis.motor.config.motor_type != MotorType.HIGH_CURRENT:
    axis.motor.config.motor_type = MotorType.HIGH_CURRENT
    need_save = True

# brake / DC 보호
if my_drive.config.brake_resistance != 2.0:
    my_drive.config.brake_resistance = 2.0
    need_save = True
safe_set(my_drive.config, 'dc_max_negative_current', -8.0)
safe_set(my_drive.config, 'dc_bus_overvoltage_trip_level', 56.0)
safe_set(my_drive.config, 'dc_bus_undervoltage_trip_level', 8.0)

# 부팅 자동 실행 비활성 (axis0 + axis1)
for ax_n in (my_drive.axis0, my_drive.axis1):
    ax_n.config.startup_motor_calibration = False
    ax_n.config.startup_encoder_index_search = False
    ax_n.config.startup_encoder_offset_calibration = False
    ax_n.config.startup_closed_loop_control = False

if need_save:
    print("\n핵심 설정 갱신 → 저장 후 재부팅")
    try:
        my_drive.save_configuration()
    except Exception:
        pass
    time.sleep(5)
    my_drive = connect()
    axis = my_drive.axis1
    print(
        f"갱신 완료 — pp: {axis.motor.config.pole_pairs}, cpr: {axis.encoder.config.cpr}")
else:
    print(
        f"설정 OK — pp: {axis.motor.config.pole_pairs}, cpr: {axis.encoder.config.cpr}")

# === 캘리 파라미터 (성공한 값) ===
axis.motor.config.resistance_calib_max_voltage = 5.0
axis.motor.config.calibration_current = 8.0
axis.motor.config.current_lim = 20.0

axis.encoder.config.calib_scan_distance = 150.0
axis.encoder.config.calib_range = 0.05
axis.encoder.config.bandwidth = 100.0
safe_set(axis.encoder.config, 'ignore_illegal_hall_state', False)

# 제어기 게인 (HALL 저해상도 대응)
axis.controller.config.vel_limit = 10.0
axis.controller.config.vel_integrator_gain = 0.0
axis.controller.config.vel_gain = 0.05
axis.controller.config.pos_gain = 0.5
axis.controller.config.input_filter_bandwidth = 2.0
axis.controller.config.input_mode = InputMode.POS_FILTER

# pre_calibrated 해제 (RAM 캘리)
axis.motor.config.pre_calibrated = False
axis.encoder.config.pre_calibrated = False
my_drive.clear_errors() if hasattr(
    my_drive, 'clear_errors') else clear_errors(axis)

# === FULL_CALIBRATION_SEQUENCE ===
print("\n=== FULL_CALIBRATION_SEQUENCE 시작 ===")
print("(삐~ 소리 후 양방향 스캔, 10~15초)")
axis.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
if not wait_idle(axis, timeout=60):
    print("타임아웃")
    dump_errors(my_drive)
    exit()

print("\n=== 결과 ===")
dump_errors(my_drive)
print(f"  motor.is_calibrated: {axis.motor.is_calibrated}")
print(f"  encoder.is_ready:    {axis.encoder.is_ready}")
print(f"  phase_resistance:    {axis.motor.config.phase_resistance:.4f} Ω")
print(
    f"  phase_inductance:    {axis.motor.config.phase_inductance*1e6:.1f} µH")

if not (axis.motor.is_calibrated and axis.encoder.is_ready):
    print("\n캘리 실패. dump_errors 확인.")
    exit()

print("\n캘리 성공. 폐루프 진입 가능 상태.")
print("운용 시작하려면 다음 줄을 추가하거나 별도 스크립트로:")
print("  axis.requested_state = AxisState.CLOSED_LOOP_CONTROL")
print("  axis.controller.input_pos = 2.0   # 2바퀴 명령")
