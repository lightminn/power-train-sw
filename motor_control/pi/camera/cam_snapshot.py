import cv2
import sys
import time

def take_snapshot():
    # 0번 카메라를 엽니다. (보통 첫 번째 웹캠은 0번입니다.)
    # 만약 안 되면 1이나 2로 바꿔보세요.
    cap = cv2.VideoCapture(0)

    # 카메라가 정상적으로 열렸는지 확인
    if not cap.isOpened():
        print("❌ 카메라를 열 수 없습니다.")
        sys.exit()

    print("✅ 카메라 연결 성공!")

    # 카메라가 안정화될 때까지 잠시 대기 (1~2초)
    # Microsoft LifeCam 같은 UVC 카메라는 초기화 시간이 필요할 수 있습니다.
    time.sleep(1)

    # 해상도 설정 (LifeCam Studio는 FHD까지 지원하지만 테스트용으로 낮춥니다.)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # 프레임 읽기 (사진 찍기)
    ret, frame = cap.read()

    # 프레임을 정상적으로 읽었는지 확인
    if not ret:
        print("❌ 프레임을 읽어올 수 없습니다. (데이터 전송 실패)")
        cap.release()
        sys.exit()

    # 이미지 저장
    filename = 'webcam_snapshot.jpg'
    cv2.imwrite(filename, frame)
    print(f"📸 사진 저장 완료: {filename}")

    # 카메라 자원 해제
    cap.release()

if __name__ == "__main__":
    take_snapshot()
