import can
import struct
import time
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')

BUS_CHANNEL = 'can0'
NODE_ID = 1  
MAX_POS = 10.0
MIN_POS = -10.0

CMD_HEARTBEAT = 0x001
CMD_SET_STATE = 0x007
CMD_SET_INPUT_POS = 0x00C
CMD_CLEAR_ERRORS = 0x018
CMD_GET_ENCODER_ESTIMATES = 0x009

AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

def send_can(bus, cmd_id, data=b''):
    arb_id = (NODE_ID << 5) | cmd_id
    msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
    try: bus.send(msg)
    except can.CanError as e: logging.error(f"CAN 전송 실패: {e}")

def request_data(bus, cmd_id):
    arb_id = (NODE_ID << 5) | cmd_id
    msg = can.Message(arbitration_id=arb_id, is_remote_frame=True, is_extended_id=False)
    try: bus.send(msg)
    except can.CanError: pass

def wait_for_boot(bus):
    """💡 ODrive가 부팅 및 영점 캘리브레이션을 마칠 때까지 기다립니다."""
    logging.info("⏳ ODrive 캘리브레이션 완료를 기다리는 중... (최대 10초)")
    start_wait = time.time()
    
    while time.time() - start_wait < 10.0:
        request_data(bus, CMD_HEARTBEAT)
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == ((NODE_ID << 5) | CMD_HEARTBEAT):
            state = msg.data[4]
            if state == AXIS_STATE_IDLE:
                logging.info("✅ ODrive 준비 완료! (IDLE 상태 확인)")
                return True
            elif state == AXIS_STATE_CLOSED_LOOP_CONTROL:
                return True
            else:
                print(f"   -> ODrive 캘리브레이션 진행 중... (상태 코드: {state})", end='\r')
                
    logging.error("\n❌ 대기 시간 초과! ODrive 상태를 확인하세요.")
    return False

def get_current_position(bus):
    """💡 제어 켤 때 튀는 걸 막기 위해 현재 위치를 읽어옵니다."""
    request_data(bus, CMD_GET_ENCODER_ESTIMATES)
    start = time.time()
    while time.time() - start < 1.0:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == ((NODE_ID << 5) | CMD_GET_ENCODER_ESTIMATES):
            pos, _ = struct.unpack('<ff', msg.data)
            return pos
    return 0.0

def wait_for_position(bus, target_pos, tolerance=0.05, timeout=5.0):
    start_time = time.time()
    last_req = 0
    
    while time.time() - start_time < timeout:
        if time.time() - last_req > 0.05:
            request_data(bus, CMD_GET_ENCODER_ESTIMATES)
            request_data(bus, CMD_HEARTBEAT)
            last_req = time.time()
            
        msg = bus.recv(timeout=0.05)
        if msg is None: continue
            
        if msg.arbitration_id == ((NODE_ID << 5) | CMD_HEARTBEAT):
            err = struct.unpack('<I', msg.data[:4])[0] 
            if err != 0:
                logging.error(f"🚨 하트비트 에러 감지! Error: {hex(err)}")
                return False
                
        elif msg.arbitration_id == ((NODE_ID << 5) | CMD_GET_ENCODER_ESTIMATES):
            pos, _ = struct.unpack('<ff', msg.data)
            print(f"   -> 실시간 위치: {pos:.3f} / 목표: {target_pos:.3f}   ", end='\r')
            if abs(pos - target_pos) <= tolerance:
                print()
                logging.info(f"✅ 목표 도달 완료! (최종 오차: {abs(pos - target_pos):.4f} turns)")
                return True

    print()
    logging.warning("⚠️ 타임아웃: 모터가 목표에 도달하지 못했습니다.")
    return False

def main():
    bus = None
    try:
        filters = [{"can_id": NODE_ID << 5, "can_mask": 0x7E0, "extended": False}]
        bus = can.interface.Bus(channel=BUS_CHANNEL, interface='socketcan', can_filters=filters)
        while bus.recv(timeout=0.01) is not None: pass

        logging.info(f"🔌 CAN 버스({BUS_CHANNEL}) 주행 시작...")

        # 💡 [핵심] ODrive가 완전히 준비될 때까지 알아서 대기!
        if not wait_for_boot(bus):
            return

        logging.info("🧹 에러 클리어")
        send_can(bus, CMD_CLEAR_ERRORS)
        time.sleep(0.1)

        # 💡 [핵심] 튀는 현상을 막기 위해 현재 위치를 목표 위치로 미리 동기화
        curr_pos = get_current_position(bus)
        logging.info(f"🔄 동기화: 내부 목표값을 현재 위치({curr_pos:.3f})로 락킹")
        send_can(bus, CMD_SET_INPUT_POS, struct.pack('<fhh', curr_pos, 0, 0))
        time.sleep(0.1)

        logging.info("🚀 제어 모드(CLOSED_LOOP) 진입")
        send_can(bus, CMD_SET_STATE, struct.pack('<i', AXIS_STATE_CLOSED_LOOP_CONTROL)) 
        time.sleep(0.5)

        target_positions = [5.0, 0.0]
        for pos in target_positions:
            clamped_pos = max(MIN_POS, min(MAX_POS, pos))
            logging.info(f"🎯 [명령 발송] 목표 위치: {clamped_pos} turns")
            send_can(bus, CMD_SET_INPUT_POS, struct.pack('<fhh', clamped_pos, 0, 0))
            
            success = wait_for_position(bus, target_pos=clamped_pos, tolerance=0.01, timeout=8.0)
            if not success:
                logging.error("❌ 주행 실패로 인해 후속 명령을 취소합니다.")
                break
            time.sleep(1.0)

    except KeyboardInterrupt:
        logging.warning("사용자 인터럽트(Ctrl+C) 수신!")
    except Exception as e:
        logging.critical(f"시스템 에러: {e}")
    finally:
        if bus is not None:
            logging.info("🛑 안전 모드(IDLE) 전환 및 버스 종료")
            try:
                send_can(bus, CMD_SET_STATE, struct.pack('<i', AXIS_STATE_IDLE))
                bus.shutdown()
            except: pass

if __name__ == "__main__":
    main()
