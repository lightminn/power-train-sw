import cv2
from ultralytics import YOLO

# 1. 인텔 최적화 OpenVINO 모델 불러오기
print("🧠 인텔 전용 OpenVINO 모델 로드 중...")
model = YOLO("yolov8n_openvino_model", task="detect")

# 2. 카메라 연결 (V4L2 백엔드 강제 지정으로 리눅스 네이티브 최적화)
print("📷 카메라 연결 중 (MS LifeCam, 49번)...")
cap = cv2.VideoCapture(42, cv2.CAP_V4L2)

# 3. 하드웨어 가속 및 대역폭 최적화 세팅
# 이 세 줄을 아래처럼 변경
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))  # YUY2 = YUYV
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("❌ 에러: 카메라를 열 수 없습니다. 권한이나 연결 상태를 확인하세요.")
    exit()

# 4. 임의의 카메라 내부 파라미터 세팅 (640x480 해상도 기준 기본값 가정)
fx, fy = 500.0, 500.0  # 초점 거리 (Focal length)
cx, cy = 320.0, 240.0  # 화면 중심점 (Center point)

print("🚀 실시간 3D 좌표 추출 시작! (화면을 클릭하고 'q' 키를 누르면 종료됩니다)")

while cap.isOpened():
    # 카메라에서 프레임 읽기
    success, frame = cap.read()
    if not success:
        print("❌ 카메라 프레임 수신 실패")
        break

    # 5. YOLO 두뇌 가동
    results = model(frame, stream=True)

    # 기본 박스가 그려진 프레임 가져오기
    annotated_frame = frame  # 초기화 (results loop 안에서 갱신됨)

    # 6. 인식된 결과에서 박스 좌표 뽑고 3D로 변환
    for r in results:
        # YOLO가 기본적으로 제공하는 박스 그리기 함수 사용
        annotated_frame = r.plot()

        boxes = r.boxes
        for box in boxes:
            # 바운딩 박스 모서리 좌표 추출
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

            # 픽셀 좌표계에서의 중심점 (u, v) 계산
            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)

            # 거리 Z (일반 웹캠이므로 현재는 1.0 미터로 가짜 고정)
            # 나중에 RGB-D 카메라를 달면 이 부분을 Z = depth_frame[v, u] 로 변경
            Z = 1.0

            # 카메라 중심 기준의 3D 공간 좌표 계산 (단위: 미터)
            X_cam = (u - cx) * Z / fx
            Y_cam = (v - cy) * Z / fy

            # 콘솔에 출력 (디버깅 및 제어기 전송용)
            cls_name = r.names[int(box.cls[0])]
            print(
                f"🎯 [{cls_name}] 3D Pos -> X: {X_cam:.2f}m, Y: {Y_cam:.2f}m, Z: {Z:.2f}m")

            # 화면에 중심점(빨간 점)과 3D 좌표 텍스트 띄우기
            cv2.circle(annotated_frame, (u, v), 5, (0, 0, 255), -1)
            coord_text = f"X:{X_cam:.2f} Y:{Y_cam:.2f} Z:{Z:.2f}"
            cv2.putText(
                annotated_frame,
                coord_text,
                (u - 50, v - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2,
            )

    # 7. 완성된 화면 띄우기
    cv2.imshow("YOLOv8 3D Position Extraction", annotated_frame)

    # 'q' 누르면 탈출
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# 8. 뒷정리
cap.release()
cv2.destroyAllWindows()
print("✅ 프로그램이 정상적으로 종료되었습니다.")
