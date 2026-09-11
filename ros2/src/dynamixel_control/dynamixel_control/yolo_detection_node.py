import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO


class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection')

        self.declare_parameter('camera_device', 0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('target_class', 'cell phone')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('show_window', True)

        camera_device = self.get_parameter('camera_device').value
        image_width = self.get_parameter('image_width').value
        image_height = self.get_parameter('image_height').value
        model_path = self.get_parameter('model_path').value

        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            raise

        self.cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera device: {camera_device}')
            raise RuntimeError(f'Cannot open camera device: {camera_device}')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_height)

        self.pub_center = self.create_publisher(Int32MultiArray, '/yolo/target_center', 10)
        self.pub_image = self.create_publisher(Image, '/yolo/detection_image', 10)
        self.bridge = CvBridge()

    def run(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn('Failed to read frame from camera')
                continue

            target_class = self.get_parameter('target_class').value
            conf_threshold = self.get_parameter('conf_threshold').value
            publish_debug = self.get_parameter('publish_debug_image').value
            show_window = self.get_parameter('show_window').value

            results = self.model.predict(frame, verbose=False)

            best_box = None
            best_conf = -1.0
            display_frame = frame.copy()

            for result in results:
                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    cls_name = self.model.names[cls_id]
                    conf = float(box.conf[0])

                    if conf < conf_threshold:
                        continue

                    x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]

                    if cls_name == target_class:
                        color = (0, 255, 0)  # 초록 — 추적 대상
                        if conf > best_conf:
                            best_conf = conf
                            best_box = box
                    else:
                        color = (255, 165, 0)  # 파랑 — 기타 객체

                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(display_frame, f'{cls_name} {conf:.2f}', (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            if best_box is not None:
                x1, y1, x2, y2 = [int(v) for v in best_box.xyxy[0]]
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                msg = Int32MultiArray()
                msg.data = [cx, cy]
                self.pub_center.publish(msg)

                self.get_logger().info(
                    f'{target_class} detected — center=({cx}, {cy}), conf={best_conf:.2f}'
                )

                cv2.circle(display_frame, (cx, cy), 5, (0, 0, 255), -1)

                if publish_debug:
                    img_msg = self.bridge.cv2_to_imgmsg(display_frame, encoding='bgr8')
                    self.pub_image.publish(img_msg)

            if show_window:
                cv2.imshow('YOLO Detection', display_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
