import cv2
import time
from ultralytics import YOLO

# --- ODrive 라이브러리 및 상태값 임포트 ---
import odrive
from odrive.enums import *

# 0. ODrive 연결 및 초기화 (M1 / axis1 기준)
print("🤖 ODrive 검색 중... (USB가 연결되어 있는지 확인하세요)")
my_drive = odrive.find_any()
print("✅ ODrive 연결 성공! 보드 시리얼:", my_drive.serial_number)

print("⚙️ M1(axis1) 캘리브레이션 시작... (모터가 움직입니다)")
# 💡 Pylance 경고를 없애기 위해 최신 Enum 문법(AxisState.)으로 수정
my_drive.axis1.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE

while my_drive.axis1.current_state != AxisState.IDLE:
    time.sleep(0.1)

print("🔒 폐루프 제어(Closed Loop Control) 모드 진입")
my_drive.axis1.requested_state = AxisState.CLOSED_LOOP_CONTROL

my_drive.axis1.controller.input_pos = 0.0
# --------------------------------------------------------

# 1. 인텔 최적화 OpenVINO 모델 불러오기
print("🧠 인텔 전용 OpenVINO 모델 로드 중...")
model = YOLO("yolov8n_openvino_model", task="detect")

# 2. 카메라 연결
print("📷 카메라 연결 중 (MS LifeCam, 49번)...")
cap = cv2.VideoCapture(42, cv2.CAP_V4L2)

# 3. 하드웨어 가속 및 대역폭 최적화 세팅
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("❌ 에러: 카메라를 열 수 없습니다.")
    exit()

# 4. 임의의 카메라 내부 파라미터 세팅
fx, fy = 500.0, 500.0
cx, cy = 320.0, 240.0

SCALE_FACTOR = 10.0

print("🚀 실시간 비전 + 모터 제어 시작! ('q' 키를 누르면 종료됩니다)")

# 💡 병(bottle)만 추적하기 위한 클래스 ID (COCO Dataset 기준 39번)
TARGET_CLASS_ID = 39

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break

    # 5. YOLO 두뇌 가동
    # 💡 classes=[TARGET_CLASS_ID] 를 추가하여 오직 'bottle'만 인식하도록 제한합니다.
    results = model(frame, classes=[TARGET_CLASS_ID], stream=True)
    annotated_frame = frame

    # 6. 인식된 결과에서 박스 좌표 뽑고 3D로 변환
    for r in results:
        annotated_frame = r.plot()  # 이제 화면에도 병에만 네모 박스가 그려집니다.
        boxes = r.boxes

        if len(boxes) > 0:
            # 화면에 병이 여러 개 잡힐 경우 가장 먼저 인식된 병(boxes[0])을 타겟으로 삼음
            box = boxes[0]
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)
            Z = 1.0

            X_cam = (u - cx) * Z / fx
            Y_cam = (v - cy) * Z / fy

            # 좌표를 모터 회전 명령으로 변환
            target_turns = X_cam * SCALE_FACTOR

            # 모터(M1)에 회전 명령 하달
            my_drive.axis1.controller.input_pos = target_turns

            cls_name = r.names[int(box.cls[0])]
            print(
                f"🎯 [{cls_name}] X: {X_cam:.2f}m -> ⚙️ Motor Target: {target_turns:.2f} Turns")

            cv2.circle(annotated_frame, (u, v), 5, (0, 0, 255), -1)
            coord_text = f"X:{X_cam:.2f} Z:{Z:.2f} Turn:{target_turns:.2f}"
            cv2.putText(annotated_frame, coord_text, (u - 50, v - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    cv2.imshow("YOLOv8 3D + ODrive Control", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# 8. 뒷정리
cap.release()
cv2.destroyAllWindows()

print("🛑 모터 정지 및 제어 종료")
my_drive.axis1.requested_state = AxisState.IDLE

print("✅ 프로그램이 정상적으로 종료되었습니다.")
