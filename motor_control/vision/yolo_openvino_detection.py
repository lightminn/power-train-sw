"""OpenVINO webcam YOLO demo exposing 2D detections only.

Calibrated intrinsics and a real depth source are not available on this V4L2
path, so it must not publish or display fabricated XYZ coordinates.
"""

import cv2
from ultralytics import YOLO


def main() -> int:
    print("🧠 인텔 전용 OpenVINO 모델 로드 중...")
    model = YOLO("yolov8n_openvino_model", task="detect")

    print("📷 카메라 연결 중 (MS LifeCam, 42번)...")
    cap = cv2.VideoCapture(42, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        print("❌ 에러: 카메라를 열 수 없습니다. 권한이나 연결 상태를 확인하세요.")
        return 1

    print("🚀 실시간 2D detection 시작 ('q' 키로 종료)")
    try:
        while cap.isOpened():
            success, frame = cap.read()
            if not success:
                print("❌ 카메라 프레임 수신 실패")
                break

            annotated_frame = frame
            for result in model(frame, stream=True):
                annotated_frame = result.plot()
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    u = int((x1 + x2) / 2)
                    v = int((y1 + y2) / 2)
                    class_name = result.names[int(box.cls[0])]
                    print(f"🎯 [{class_name}] 2D detection center=({u}, {v})")
                    cv2.circle(annotated_frame, (u, v), 5, (0, 0, 255), -1)
                    cv2.putText(
                        annotated_frame,
                        f"2D center ({u}, {v})",
                        (u - 50, v - 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 255),
                        2,
                    )

            cv2.imshow("YOLOv8 2D Detection", annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
    print("✅ 프로그램이 정상적으로 종료되었습니다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
