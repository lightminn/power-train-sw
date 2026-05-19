from TMotorCANControl.servo_can import TMotorManager_servo_can
from TMotorCANControl.servo_can import Servo_Params
import time

Servo_Params['AK40-10'] = {
    'P_min': -32000.0,        # deg, int32 cmd 한계 (motor side)
    'P_max':  32000.0,
    'V_min': -32000,          # ERPM
    'V_max':  32000,
    'Curr_min': -60.0,        # A (대략, 펌웨어 한도 따라)
    'Curr_max':  60.0,
    'T_min': -18.0,           # Nm (출력축 기준 추정 — 데이터시트 확인)
    'T_max':  18.0,
    'Kt_TMotor': 0.16,        # Nm/A (대략)
    'GEAR_RATIO': 10.0,
    'NUM_POLE_PAIRS': 14,
    'Use_derived_torque_constants': False,
}


# CAN id 10, 모델명은 본인이 등록한 키
with TMotorManager_servo_can(motor_type='AK40-10', motor_ID=10) as m:
    m.set_zero_position()           # = 우리의 set_origin_here
    time.sleep(0.2)

    # 출력축 90도 (라디안 입력)
    m.set_output_angle_radians( 1.5708 )   # +90 deg
    for _ in range(60):                    # 3초 hold @ 20Hz
        m.update()
        time.sleep(0.05)

    m.set_output_angle_radians( 0.0 )      # 복귀
    for _ in range(60):
        m.update()
        time.sleep(0.05)

    # 50 RPM (출력축) = ω_out [rad/s] = 50*2π/60
    m.set_output_velocity_radians_per_second( 50*2*3.1416/60 )
    t_end = time.time()+5
    while time.time() < t_end:
        m.update()
        time.sleep(0.02)
