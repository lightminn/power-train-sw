import cv2
import time
import odrive
from odrive.enums import *
from ultralytics import YOLO

# === 설정 ===
GEAR_RATIO = 5.0
TARGET_CLASS_ID = 39          # COCO bottle
CAMERA_ID = 42
FRAME_W, FRAME_H = 1280, 720
fx, fy = 500.0, 500.0
cx, cy = FRAME_W / 2, FRAME_H / 2
SCALE_FACTOR = 5.0            # X_cam(m) → 모터 turns 변환. 너무 크면 발진
MAX_TURNS = 20.0              # 안전 한계 (모터단). 출력단 ±4 turns
POS_DEADZONE = 0.05           # 명령 변화 이 미만이면 무시 (떨림 방지)


# === ODrive 초기화 ===
print("🤖 ODrive 검색 중...")
drv = odrive.find_any()
ax = drv.axis1
print(f"✅ 연결: {drv.serial_number}, vbus: {drv.vbus_voltage:.2f}V")
print(f"   brake_armed: {drv.brake_resistor_armed}")

# 캘리 상태에 따라 분기
if ax.motor.is_calibrated and ax.encoder.is_ready:
    print("✓ 이미 캘리됨 — 스킵")
else:
    print("⚙️ FULL_CALIBRATION_SEQUENCE...")
    ax.error = 0; ax.motor.error = 0; ax.encoder.error = 0; ax.controller.error = 0
    ax.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
    while ax.current_state != AxisState.IDLE:
        time.sleep(0.1)
    if not (ax.motor.is_calibrated and ax.encoder.is_ready):
        print(f"❌ 캘리 실패. axis.error: {ax.error:#x}, motor: {ax.motor.error:#x}, enc: {ax.encoder.error:#x}")
        exit()
    print("✓ 캘리 완료")

# 에러 클리어
ax.error = 0; ax.motor.error = 0; ax.encoder.error = 0; ax.controller.error = 0

# 비전 추적용 위치 제어 게인 (떨림 방지 + 부드러운 추종)
ax.motor.config.current_lim = 15.0
ax.controller.config.vel_limit = 5.0
ax.encoder.config.bandwidth = 50
ax.controller.config.pos_gain = 2.0
ax.controller.config.vel_gain = 0.04
ax.controller.config.vel_integrator_gain = 0.0

# 명령 점프를 부드럽게 (POS_FILTER) — 카메라 노이즈로 input_pos가 점프해도 모터는 부드럽게
ax.controller.config.control_mode = ControlMode.POSITION_CONTROL
ax.controller.config.input_mode = InputMode.POS_FILTER
ax.controller.config.input_filter_bandwidth = 2.0   # 2Hz 저역 필터

print("🔒 폐루프 진입...")
ax.requested_state = AxisState.CLOSED_LOOP_CONTROL
time.sleep(0.5)
if ax.current_state != 8:
    print(f"❌ 폐루프 진입 실패. motor: {ax.motor.error:#x}")
    exit()

# 현재 위치 = 원점
origin = ax.encoder.pos_estimate
ax.controller.input_pos = origin
print(f"✓ 폐루프 진입. 원점: {origin:.2f} turns")


# === YOLO + 카메라 ===
print("🧠 OpenVINO 모델 로드...")
model = YOLO("yolov8n_openvino_model", task="detect")

print(f"📷 카메라 {CAMERA_ID}...")
cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

if not cap.isOpened():
    print("❌ 카메라 열기 실패")
    ax.requested_state = AxisState.IDLE
    exit()

print("🚀 추적 시작 — 'q' 종료\n")

last_target = origin
frame_count = 0
t_start = time.time()

try:
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            break

        results = model(frame, classes=[TARGET_CLASS_ID], stream=True, verbose=False)
        annotated = frame

        for r in results:
            annotated = r.plot()
            boxes = r.boxes

            if len(boxes) > 0:
                box = boxes[0]
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                u = int((x1 + x2) / 2)
                v = int((y1 + y2) / 2)
                Z = 1.0
                X_cam = (u - cx) * Z / fx
                Y_cam = (v - cy) * Z / fy

                # 모터단 turns. 안전 한계 클램프
                target_turns = X_cam * SCALE_FACTOR
                target_turns = max(-MAX_TURNS, min(MAX_TURNS, target_turns))
                target_pos = origin + target_turns

                # deadzone — 카메라 노이즈로 인한 미세 떨림 무시
                if abs(target_pos - last_target) > POS_DEADZONE:
                    ax.controller.input_pos = target_pos
                    last_target = target_pos

                cur = ax.encoder.pos_estimate - origin
                cls_name = r.names[int(box.cls[0])]
                print(f"🎯 [{cls_name}] X={X_cam:+.2f}m → tgt={target_turns:+.2f} cur={cur:+.2f} (출력 {cur/GEAR_RATIO:+.2f})")

                cv2.circle(annotated, (u, v), 5, (0, 0, 255), -1)
                cv2.putText(annotated, f"X:{X_cam:+.2f} Tgt:{target_turns:+.2f} Cur:{cur:+.2f}",
                            (u - 80, v - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # ODrive 에러 모니터
        if ax.error != 0:
            print(f"\n⚠️ 트립: axis={ax.error:#x}, motor={ax.motor.error:#x}")
            break
        if ax.current_state != 8:
            print(f"\n⚠️ 폐루프 이탈: state={ax.current_state}")
            break

        cv2.imshow("YOLO + ODrive HALL Tracking", annotated)
        frame_count += 1

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except KeyboardInterrupt:
    print("\nCtrl+C")

finally:
    elapsed = time.time() - t_start
    if elapsed > 0:
        print(f"\n📊 평균 FPS: {frame_count/elapsed:.1f}")

    # 원점 복귀
    print("🏠 원점 복귀...")
    ax.controller.input_pos = origin
    time.sleep(2)

    cap.release()
    cv2.destroyAllWindows()

    ax.requested_state = AxisState.IDLE
    print("🛑 IDLE")
    print("✅ 종료")
