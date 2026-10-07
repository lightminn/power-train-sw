"""OV5640(V4L2 USB 카메라) + YOLO Seg("박스") 테스트 전용 뷰어 노드 (2026-08-02 추가).

키보드 텔레옵 창과 별도 터미널에 띄워서, 로봇팔 조작과 무관하게 카메라+YOLO
세그멘테이션 파이프라인이 잘 도는지 화면으로 확인하기 위한 **순수 확인용** 노드다.

⚠️ **가짜(fake) 창이다** — 토픽은 실제로 발행하지만(/vision_test/detected_objects),
이 토픽을 구독하는 곳이 시스템 어디에도 없다. perception_node의 진짜 파이프라인
(/detected_objects, /pick_target — arm_fsm 이 구독)과는 이름부터 완전히 분리돼
있어서, 이 노드를 띄우거나 내려도 로봇팔 동작에는 어떤 영향도 없다.

perception_node(RealSense 전용, depth 기반 markerless pose)와 달리 이 노드는:
  - 카메라: 평범한 V4L2 USB 캠(OV5640 등) — cv2.VideoCapture(device, CAP_V4L2).
    depth 가 없으니 pose 는 항상 비어있다(orientation.w=1, position=0) — 애초에
    "인식이 되는지" 화면으로 보는 게 목적이라 markerless pose 정확도는 범위 밖.
  - 모델: model_presets.py 의 "box" preset(대회 상자 seg 모델, 1클래스
    box-segmentation)을 기본값으로 그대로 재사용 — 실제 대회 모델이 이 카메라에서도
    잘 잡히는지 확인할 수 있게.
  - 화면에 OpenCV 창으로 마스크 오버레이 + bbox + 라벨을 계속 띄운다(q 로 종료).

실행: `ros2 run robot_arm_perception vision_test` (workspace 루트에서 — model_path
기본값이 "src/robot_arm_perception/models/best.pt" 상대경로라 perception_node와
동일하게 CWD=~/ros2_ws 를 가정한다). 편의 스크립트는
`src/robot_arm_perception/scripts/run_vision_test.sh` 참고.
"""
import os

import cv2
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from cv_bridge import CvBridge

from geometry_msgs.msg import Pose
from sensor_msgs.msg import RegionOfInterest, Image as ImageMsg
from robot_arm_msgs.msg import DetectedObject, DetectedObjectArray
from robot_arm_perception.model_presets import get_preset

#: perception_node 의 진짜 토픽(/detected_objects, /pick_target)과 절대 안 겹치게
#: 이름부터 다르게 둔다 — 아무도 구독 안 하는 게 이 노드의 안전장치다.
TOPIC_OBJECTS = "/vision_test/detected_objects"
TOPIC_DEBUG_IMAGE = "/vision_test/debug_image"

WINDOW_TITLE = "YOLO Seg 테스트 (OV5640) - 로봇팔에 영향 없음 (q로 종료)"


class VisionTestNode(Node):
    def __init__(self):
        super().__init__("vision_test_node")

        preset = get_preset("box", self.get_logger())

        self.declare_parameter("camera_device", 0)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("model_path", preset["model_path"])
        self.declare_parameter("task", preset["task"])  # 'segment'
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("show_window", True)
        self.declare_parameter("publish_debug_image", True)

        camera_device = self.get_parameter("camera_device").value
        image_width = self.get_parameter("image_width").value
        image_height = self.get_parameter("image_height").value
        model_path = self.get_parameter("model_path").value
        task = self.get_parameter("task").value

        if not os.path.exists(model_path):
            self.get_logger().error(
                f"모델 파일을 못 찾습니다: {model_path} — 워크스페이스 루트(~/ros2_ws)에서 "
                "실행 중인지 확인하세요(상대경로 기준)."
            )
            raise FileNotFoundError(model_path)

        from ultralytics import YOLO
        # task 를 명시적으로 넘긴다 — perception_node와 같은 이유(TensorRT .engine
        # 은 task 메타데이터를 안 보존해서 seg 모델도 detect로 오판될 수 있음).
        self.model = YOLO(model_path, task=task)
        self.get_logger().info(f"YOLO 로드: {model_path} (task={self.model.task})")

        self.cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(
                f"카메라를 열 수 없습니다: {camera_device} — OV5640 이 다른 /dev/videoN "
                "번호로 잡혔을 수 있습니다. camera_device 파라미터로 지정하세요."
            )
            raise RuntimeError(f"Cannot open camera device: {camera_device}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_height)
        self._w = image_width
        self._h = image_height

        self.pub_objects = self.create_publisher(DetectedObjectArray, TOPIC_OBJECTS, 10)
        self.pub_debug = self.create_publisher(ImageMsg, TOPIC_DEBUG_IMAGE, 1)
        self.bridge = CvBridge()

        self.get_logger().info(
            f"vision_test_node 시작 — camera_device={camera_device}, "
            f"{image_width}x{image_height}, 발행 토픽={TOPIC_OBJECTS} "
            "(구독자 없음 — 로봇팔과 완전히 무관)"
        )

    def run(self):
        conf = self.get_parameter("conf_threshold").value
        show_window = self.get_parameter("show_window").value
        publish_debug = self.get_parameter("publish_debug_image").value

        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("카메라 프레임 읽기 실패")
                continue

            stamp = self.get_clock().now().to_msg()
            results = self.model.predict(frame, conf=conf, verbose=False)
            r0 = results[0]
            mask_polygons = None if r0.masks is None else r0.masks.xy

            array_msg = DetectedObjectArray()
            array_msg.header.stamp = stamp
            array_msg.header.frame_id = "vision_test_camera"

            display = frame.copy()
            for i, box in enumerate(r0.boxes):
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
                cls_id = int(box.cls[0])
                conf_val = float(box.conf[0])

                obj = DetectedObject()
                obj.class_id = cls_id
                obj.class_name = self.model.names[cls_id]
                obj.confidence = conf_val
                obj.pose = Pose()
                obj.pose.orientation.w = 1.0  # depth 없음 — pose 는 항상 비어있음
                roi = RegionOfInterest()
                roi.x_offset, roi.y_offset = max(x1, 0), max(y1, 0)
                roi.width, roi.height = max(x2 - x1, 0), max(y2 - y1, 0)
                obj.bbox = roi
                array_msg.objects.append(obj)

                self._draw_one(display, i, x1, y1, x2, y2, obj.class_name, conf_val, mask_polygons)

            # SIGTERM(예: `timeout`, docker stop)이 프레임 처리 중에 오면 rclpy 가
            # context 를 먼저 내려버려서, 여기서 그냥 publish 하면 RCLError
            # ("publisher's context is invalid") 트레이스백을 뱉고 죽는다.
            if not rclpy.ok():
                break

            self.pub_objects.publish(array_msg)
            if publish_debug and self.pub_debug.get_subscription_count() > 0:
                self.pub_debug.publish(self.bridge.cv2_to_imgmsg(display, encoding="bgr8"))

            if show_window:
                cv2.imshow(WINDOW_TITLE, display)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            rclpy.spin_once(self, timeout_sec=0.0)

    def _draw_one(self, img, index, x1, y1, x2, y2, class_name, conf, mask_polygons):
        color = (0, 200, 255)  # 대회 로직과 헷갈리지 않게 perception_node 와 다른 색(주황)
        if mask_polygons is not None and index < len(mask_polygons):
            polygon = np.asarray(mask_polygons[index], dtype=np.float32)
            if polygon.ndim == 2 and polygon.shape[0] >= 3:
                polygon[:, 0] = np.clip(polygon[:, 0], 0, self._w - 1)
                polygon[:, 1] = np.clip(polygon[:, 1], 0, self._h - 1)
                mask = np.zeros((self._h, self._w), dtype=np.uint8)
                cv2.fillPoly(mask, [np.rint(polygon).astype(np.int32)], 1)
                region = mask.astype(bool)
                img[region] = (img[region] * 0.5 + np.array(color, dtype=np.float32) * 0.5).astype(np.uint8)

        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            img, f"{class_name} {conf:.2f}", (x1, max(y1 - 8, 12)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA,
        )

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionTestNode()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
