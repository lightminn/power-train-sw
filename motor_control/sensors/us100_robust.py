import serial
import time

UART_PORT = "/dev/ttyTHS1"
BAUD_RATE = 9600

def read_distance(ser):
    ser.reset_input_buffer()
    
    # 💡 [최종 핵심] 전압을 0V로 떨어뜨리는 \x00 대신, 
    # 3.3V를 유지하며 젯슨의 버그만 막아내는 \xFF 안전 방패 사용!
    payload = (b'\xFF' * 8) + bytes([0x55])
    
    ser.write(payload)
    ser.flush()

    # 센서가 진짜 초음파 쏘고 대답할 시간 대기
    time.sleep(0.1) 

    if ser.in_waiting >= 2:
        data = ser.read(ser.in_waiting)
        # 센서가 보낸 진짜 응답 2바이트 추출
        distance = data[-2] * 256 + data[-1] 
        
        if 1 < distance < 10000:
            return distance
    return None

def main():
    print(f"🔌 US100 센서 (안전 방패 모드) 시작 ({UART_PORT})")

    try:
        with serial.Serial(UART_PORT, BAUD_RATE, timeout=1) as ser:
            time.sleep(0.5) 

            while True:
                dist = read_distance(ser)
                if dist is not None:
                    print(f"🚀 거리: {dist} mm  ({dist / 10:.1f} cm)")
                else:
                    print("⚠️ 응답 없음 (센서 확인)")
                
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n종료합니다.")
    except Exception as e:
        print(f"🚨 에러 발생: {e}")

if __name__ == "__main__":
    main()
