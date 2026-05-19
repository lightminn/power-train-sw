import serial
import time

UART_PORT = "/dev/ttyTHS1"
BAUD_RATE = 9600

def read_distance(ser):
    """
    US100 UART 모드 거리 측정
    - 0x55 전송 → 2바이트 수신 (High, Low)
    - 거리(mm) = High * 256 + Low
    - 유효 범위: 1mm ~ 10000mm
    """
    ser.reset_input_buffer()
    ser.write(bytes([0x55]))
    time.sleep(0.5)

    if ser.in_waiting >= 2:
        high = ser.read(1)[0]
        low  = ser.read(1)[0]
        distance = high * 256 + low

        if 1 < distance < 10000:
            return distance
    return None

def main():
    print(f"[INFO] 포트:{UART_PORT}, baudrate:{BAUD_RATE}")

    with serial.Serial(UART_PORT, BAUD_RATE, timeout=1) as ser:
        print("[INFO] 거리 측정 시작 (Ctrl+C로 종료)\n")

        while True:
            distance = read_distance(ser)

            if distance is not None:
                print(f"거리:{distance} mm  ({distance / 10:.1f} cm)")
            else:
                print("측정 실패 (범위 초과 or 응답 없음)")

            time.sleep(0.1)

if __name__ == "__main__":
    main()
