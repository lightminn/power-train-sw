import cv2
import socket
import time

# 🚨 여기에 내 컴퓨터(아치 리눅스)의 IP 주소를 적으세요!
TARGET_IP = "192.168.50.203"
PORT = 5000

# 0번 웹캠 연결
cap = cv2.VideoCapture(0)

# UDP 패킷(최대 65KB) 안에 우겨넣기 위해 해상도 타협
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# UDP 소켓 생성
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"🚀 {TARGET_IP}:{PORT} 로 UDP 송출 시작... (종료: Ctrl+C)")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # JPEG 압축 (품질 70: 보통 20~30KB 나옴, UDP 최대치 65KB 이내로 컷)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 60]
        result, buffer = cv2.imencode('.jpg', frame, encode_param)

        if result:
            try:
                # 버퍼를 바이트로 변환해서 냅다 쏘기
                sock.sendto(buffer.tobytes(), (TARGET_IP, PORT))
            except Exception as e:
                # 65KB를 초과하면 에러 발생 (해상도나 품질을 낮춰야 함)
                print(f"패킷 전송 실패 (크기 초과): {e}")

        # 너무 미친듯이 쏴서 네트워크가 터지는 걸 방지 (약 30프레임 제한)
        time.sleep(0.03)

except KeyboardInterrupt:
    print("\n🛑 송출을 종료합니다.")
finally:
    cap.release()
    sock.close()
