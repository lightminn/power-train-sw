import odrive
from odrive.enums import *
import time
import logging

# 로깅 설정
logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')

# --- 하드웨어 스펙 및 통신 상수 ---
NODE_ID = 1
POLE_PAIRS = 7
CPR = 16384
BAUDRATE = 250000
CALIB_TIMEOUT_SEC = 30.0

def main():
    logging.info("🔍 ODrive 연결 대기 중...")
    try:
        odrv0 = odrive.find_any(timeout=10)
    except Exception as e:
        logging.critical(f"ODrive를 찾을 수 없습니다: {e}")
        return

    logging.info("💥 공장 초기화 진행...")
    try:
        odrv0.erase_configuration()
    except Exception as e:
        # ObjectLostError 등 재부팅 시 발생하는 통신 끊김은 정상
        if "ObjectLostError" not in str(type(e)):
            logging.warning(f"초기화 중 예상치 못한 예외: {e}")
    
    time.sleep(3)
    try:
        odrv0 = odrive.find_any(timeout=10)
        logging.info("✅ 재연결 성공!")
    except Exception as e:
        logging.critical(f"초기화 후 ODrive 재연결 실패: {e}")
        return

    # 1. CAN 세팅 (에러 발생 시 중단)
    logging.info(f"⚙️ CAN 설정 (Baudrate: {BAUDRATE}, Node ID: {NODE_ID})")
    try:
        odrv0.can.config.baud_rate = BAUDRATE
    except Exception as e:
        logging.error(f"CAN Baudrate 설정 실패: {e}")
        
    try:
        odrv0.axis1.config.can.node_id = NODE_ID
    except AttributeError:
        try:
            odrv0.axis1.config.can_node_id = NODE_ID
        except Exception as e:
            logging.critical(f"Node ID 설정 완전 실패. 펌웨어 버전 확인 필요: {e}")
            return

    # 💡 [피드백 반영] RTR 폴링 제거를 위한 순환 전송(Cyclic) 설정 (10ms)
    try:
        odrv0.axis1.config.can.encoder_rate_ms = 10
    except AttributeError:
        logging.warning("현재 펌웨어에서 encoder_rate_ms 자동 전송을 지원하지 않을 수 있습니다.")

    # 2. 하드웨어 스펙 및 제어기 튜닝
    odrv0.axis1.motor.config.pole_pairs = POLE_PAIRS
    odrv0.axis1.encoder.config.cpr = CPR

    odrv0.axis1.controller.config.vel_limit = 5.0
    odrv0.axis1.controller.config.pos_gain = 2.0
    odrv0.axis1.controller.config.vel_gain = 0.015
    odrv0.axis1.controller.config.vel_integrator_gain = 0.25
    
    odrv0.axis1.controller.config.input_filter_bandwidth = 2.0
    odrv0.axis1.controller.config.input_mode = INPUT_MODE_POS_FILTER

    # 3. 캘리브레이션 및 타임아웃 처리
    logging.info("🚀 캘리브레이션 시작...")
    odrv0.axis1.clear_errors()
    odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    
    start_time = time.time()
    while odrv0.axis1.current_state != AXIS_STATE_IDLE:
        if time.time() - start_time > CALIB_TIMEOUT_SEC:
            logging.critical("🚨 캘리브레이션 타임아웃 (30초 초과)! 모터 Stall 또는 배선 불량 의심.")
            odrv0.axis1.requested_state = AXIS_STATE_IDLE
            return
        time.sleep(0.5)

    # 4. 결과 검증 및 Error Dump
    if odrv0.axis1.encoder.is_ready:
        logging.info("✅ 캘리브레이션 성공!")
        odrv0.axis1.motor.config.pre_calibrated = True
        odrv0.axis1.encoder.config.pre_calibrated = True
        
        # 💡 [피드백 반영] 안전 최우선: 부팅 시 영점만 잡고 제어 모드는 켜지 않음!
        odrv0.axis1.config.startup_encoder_offset_calibration = True
        odrv0.axis1.config.startup_closed_loop_control = False 
        
        logging.info("💾 설정 영구 저장 및 최종 재부팅...")
        try:
            odrv0.save_configuration()
            odrv0.reboot()
        except Exception as e:
            if "ObjectLostError" not in str(type(e)):
                logging.warning(f"저장 중 예외: {e}")
        logging.info("🎉 셋업 완료! 안전 모드(IDLE)로 부팅됩니다.")
    else:
        # 💡 [피드백 반영] 실패 시 원인 추적을 위한 명시적 Error Dump
        logging.error("❌ 캘리브레이션 실패! [Error Dump]")
        logging.error(f"  - Axis Error:       {hex(odrv0.axis1.error)}")
        logging.error(f"  - Motor Error:      {hex(odrv0.axis1.motor.error)}")
        logging.error(f"  - Encoder Error:    {hex(odrv0.axis1.encoder.error)}")
        logging.error(f"  - Controller Error: {hex(odrv0.axis1.controller.error)}")

if __name__ == "__main__":
    main()
