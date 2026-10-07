"""손목 카메라 노드 — 그리퍼에 고정된 USB 캠으로 근접·파지 상태를 관측한다 (2026-08-13 추가).

## 왜 `perception_node` 를 확장하지 않고 별도 노드인가

`perception_node` 는 **`/pick_target` 을 latched(TRANSIENT_LOCAL)로 발행하는 유일한
노드**이고, `arm_fsm` 은 그 값을 그대로 목표 좌표로 쓴다. 같은 노드를 손목 캠용
두 번째 인스턴스로 띄우면 토픽 5개를 전부 remap 해야 하는데, `/pick_target` remap 을
하나만 빠뜨리면 **손목 캠이 본 박스가 latched 로 박혀** 팔이 엉뚱한 좌표로 간다.
latched 라 손목 노드를 내려도 값이 남고, 증상은 "인식은 되는데 팔이 딴 데로 간다"라
원인 추적이 어렵다.

이 노드는 그 publisher 를 **아예 만들지 않는다.** launch 규율로 지키는 것보다 구조로
막는 쪽이 맞다고 판단했다(같은 이유로 `robot_arm_gui` 도 퍼블리셔를 0개로 유지한다).

## 무엇을 발행하나

    /wrist/raw_image        원본 (캡처 스레드가 게이트 없이 매 프레임)
    /wrist/debug_image      오버레이 (구독자가 있을 때만 — 없으면 그리지도 않는다)
    /wrist/detected_objects DetectedObjectArray. **pose 는 비어 있다**(depth 없음)
    /wrist/metrics          std_msgs/String(JSON) — wrist_metrics.summarize() 한 벌

`pose` 를 비워 두는 건 실수가 아니라 계약이다 — 단안 카메라는 3D 좌표를 줄 수 없고,
누군가 이 토픽의 pose 를 좌표로 오인하는 일이 없어야 한다. 거리는 `metrics` 의
`distance_m`(겉보기 크기 환산, `f_px` 실측 후에만 값이 채워짐)으로만 나간다.

## 추론 주기를 낮게 잡는 이유

전방 캠 YOLO 가 같은 Jetson GPU 를 쓴다. 손목 캠은 "지금 얼마나 가까운가"만 보면 되므로
`inference_rate_hz`(기본 5Hz)로 충분하다. 캡처는 그대로 30fps 로 돌아 raw 영상은
끊기지 않는다(`perception_node` 가 캡처/추론을 분리한 것과 같은 이유).

## USB 대역폭 함정

손목 캠과 RealSense 가 **같은 USB 2.0 허브**에 물려 있다(2026-08-13 실측: 허브·양 카메라
전부 480Mbps 링크). 무압축(YUYV)으로 열면 640x480@30 하나가 대역을 거의 다 먹어
**RealSense 스트림이 시작되지 않는다.** 그래서 `fourcc` 기본값이 `MJPG` 이고,
width/height **보다 먼저** 설정한다(순서를 바꾸면 드라이버가 무시하는 경우가 있다).
"""
import json
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image as ImageMsg, RegionOfInterest
from std_msgs.msg import String

from robot_arm_msgs.msg import DetectedObject, DetectedObjectArray
# ⚠️ 이 import 는 **상수와 순수 함수만** 가져온다 — 모듈 최상단에서 RealSense 를 열지
# 않으므로(파이프라인 생성은 `_init_realsense()` 안에 있다) 카메라 충돌이 없다.
# TensorRT 엔진 캐시(`_resolve_model`)를 그대로 재사용하려는 것이고, 같은 형식의 재사용
# 선례가 `arm_fsm_node.py` 의 `from ...moveit_dynamixel_bridge import JOINT_CONFIG` 다.
from robot_arm_perception.perception_node import _load_yolo
from robot_arm_perception.model_presets import get_preset
from robot_arm_perception import wrist_color_mask, wrist_metrics

TOPIC_RAW = '/wrist/raw_image'
TOPIC_DEBUG = '/wrist/debug_image'
TOPIC_OBJECTS = '/wrist/detected_objects'
TOPIC_METRICS = '/wrist/metrics'

#: 오버레이 색 — 전방 캠(초록/파랑)과 눈으로 구분되게 주황.
COLOR = (0, 165, 255)


def _open_device(device: str):
    """`/dev/v4l/by-id/...` 경로와 `2` 같은 번호를 모두 받는다.

    ⚠️ 번호(`/dev/video2`)는 **부팅·핫플러그마다 바뀐다** — RealSense 가 video4~9 를
    한꺼번에 잡기 때문에 순서가 조금만 달라져도 손목 캠 번호가 밀린다. 운용값으로는
    반드시 `by-id` 경로를 쓸 것(`ls /dev/v4l/by-id/`).
    """
    device = str(device).strip()
    if device.lstrip('-').isdigit():
        return cv2.VideoCapture(int(device), cv2.CAP_V4L2)
    return cv2.VideoCapture(device, cv2.CAP_V4L2)


class WristCameraNode(Node):
    """캡처 스레드(raw 발행) + 추론 스레드(저주기)로 나뉜 경량 노드."""

    def __init__(self):
        super().__init__('wrist_camera')

        preset = get_preset('box')
        # 'color' = HSV 색상 마스크(기본), 'yolo' = seg 모델.
        # ⚠️ 기본값이 color 인 근거는 실측이다 — wrist_color_mask 모듈 docstring 참고
        # (파지 거리에서 YOLO 가 대상 상자를 conf 0.20 에서도 못 봤다). color 는 GPU 를
        # 아예 쓰지 않아 전방 캠 추론 대역을 뺏지 않는 부수 효과도 있다.
        self.declare_parameter('mask_source', 'color')
        # 그리퍼가 보이는 화면 영역 (x0, x1, y0, y1) 비율 — 배경의 같은 색 물체를 배제한다.
        self.declare_parameter('roi', list(wrist_color_mask.DEFAULT_ROI))
        self.declare_parameter('min_blob_px', wrist_color_mask.DEFAULT_MIN_BLOB_PX)
        # 케이블 배제 — 상자와 같은 색이라 색으로는 못 가르고 두께로 가른다.
        # 0 으로 두면 끈다(멀리서 상자가 아주 작게 보이는 구간용).
        self.declare_parameter('thin_reject_px', wrist_color_mask.DEFAULT_THIN_REJECT_PX)
        self.declare_parameter('trim_frac', wrist_color_mask.DEFAULT_TRIM_FRAC)
        self.declare_parameter('camera_device', '')
        self.declare_parameter('fourcc', 'MJPG')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('capture_fps', 30)
        self.declare_parameter('inference_rate_hz', 5.0)
        self.declare_parameter('model_name', 'box')
        self.declare_parameter('model_path', preset['model_path'])
        self.declare_parameter('task', preset['task'])
        self.declare_parameter('backend', 'trt')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('frame_id', 'wrist_camera_optical_frame')
        # 캘리브 전 기본값 0 — 그러면 metrics 의 distance_m 만 None 이 되고 나머지는 나온다.
        # 실측(알려진 거리에 박스를 두고 픽셀 크기 측정) 후 launch 인자로 넘긴다.
        self.declare_parameter('f_px', 0.0)
        # ⚠️ '높이'가 아니라 '한 변'이다 — 거리는 bbox **가로 폭**으로만 낸다
        # (세로는 비스듬히 내려다본 단축 때문에 f_px 가 2.7배 다르다, wrist_metrics 참고).
        # 대상은 95mm 큐브라 어느 변이든 같은 값이지만, 직육면체로 바뀌면 **가로로 보이는
        # 변**의 실치수를 넣어야 한다.
        self.declare_parameter('box_size_m', 0.0)

        self._w = int(self.get_parameter('image_width').value)
        self._h = int(self.get_parameter('image_height').value)
        self._frame_id = self.get_parameter('frame_id').value
        self._conf = float(self.get_parameter('conf_threshold').value)
        self._f_px = float(self.get_parameter('f_px').value)
        self._box_size = float(self.get_parameter('box_size_m').value)

        self._bridge = CvBridge()
        self.pub_raw = self.create_publisher(ImageMsg, TOPIC_RAW, 1)
        self.pub_debug = self.create_publisher(ImageMsg, TOPIC_DEBUG, 1)
        self.pub_objects = self.create_publisher(DetectedObjectArray, TOPIC_OBJECTS, 10)
        self.pub_metrics = self.create_publisher(String, TOPIC_METRICS, 10)
        # ⚠️ /pick_target publisher 를 만들지 않는다 — 모듈 docstring 참고.

        self._mask_source = str(self.get_parameter('mask_source').value)
        self._roi = tuple(float(v) for v in self.get_parameter('roi').value)
        self._min_blob = int(self.get_parameter('min_blob_px').value)
        self._thin_px = int(self.get_parameter('thin_reject_px').value)
        self._trim_frac = float(self.get_parameter('trim_frac').value)

        self._cap = self._open_camera()
        # color 모드면 모델을 아예 로드하지 않는다 — GPU 도, 8분짜리 엔진 빌드도 없다.
        self._model = self._load_model() if self._mask_source == 'yolo' else None

        self._lock = threading.Lock()
        self._latest = None          # (seq, stamp, frame)
        self._seq = 0
        self._running = True
        self._capture = threading.Thread(target=self._capture_loop, daemon=True)
        self._inference = threading.Thread(target=self._inference_loop, daemon=True)
        self._capture.start()
        self._inference.start()

        self.get_logger().info(
            f'wrist_camera 시작 — {self._w}x{self._h}, 마스크={self._mask_source}, '
            f'{float(self.get_parameter("inference_rate_hz").value):.1f}Hz, '
            f'ROI={self._roi}, 가는구조물제거={self._thin_px}px, frame_id={self._frame_id}, '
            f'거리환산={"활성" if self._f_px > 0 and self._box_size > 0 else "미캘리브(distance_m=null)"}')

    # ── 초기화 ─────────────────────────────────

    def _open_camera(self):
        device = self.get_parameter('camera_device').value
        if not str(device).strip():
            raise RuntimeError(
                'camera_device 가 비어 있습니다 — `ls /dev/v4l/by-id/` 의 경로를 넘기세요. '
                '번호(/dev/video2)는 부팅마다 바뀝니다.')
        cap = _open_device(device)
        if not cap.isOpened():
            raise RuntimeError(f'카메라를 열 수 없습니다: {device}')
        # ⚠️ 순서 주의 — FOURCC 를 해상도보다 먼저. 모듈 docstring 의 대역폭 항목 참고.
        fourcc = str(self.get_parameter('fourcc').value)
        if fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._h)
        cap.set(cv2.CAP_PROP_FPS, int(self.get_parameter('capture_fps').value))

        actual = int(cap.get(cv2.CAP_PROP_FOURCC))
        actual_name = actual.to_bytes(4, 'little').decode(errors='replace') if actual else '?'
        if fourcc and actual_name != fourcc:
            self.get_logger().warn(
                f'요청한 FOURCC={fourcc} 인데 드라이버가 {actual_name} 로 열었습니다 — '
                'USB 2.0 허브를 RealSense 와 공유 중이면 대역폭이 모자라 '
                '한쪽 스트림이 안 뜰 수 있습니다.')
        return cap

    def _load_model(self):
        name = self.get_parameter('model_name').value
        preset = get_preset(name)
        path = self.get_parameter('model_path').value or preset['model_path']
        task = self.get_parameter('task').value or preset['task']
        backend = self.get_parameter('backend').value
        # ⚠️ task 를 반드시 명시한다 — .engine 은 task 메타데이터를 보존하지 않아
        # ultralytics 가 seg 모델을 detect 로 오판하고 masks 가 조용히 None 이 된다.
        model, resolved, seconds = _load_yolo(path, backend, task, self._h, self._w)
        self.get_logger().info(
            f'YOLO loaded: preset={name} path={resolved} task={task} ({seconds:.2f}s)')
        return model

    # ── 캡처 ───────────────────────────────────

    def _capture_loop(self):
        """`cap.read()` 가 블로킹이라 이 루프 자체가 페이싱이 된다(별도 슬립 불필요)."""
        while self._running and rclpy.ok():
            ok, frame = self._cap.read()
            if not ok:
                self.get_logger().warn('프레임 읽기 실패', throttle_duration_sec=5.0)
                time.sleep(0.05)
                continue
            stamp = self.get_clock().now().to_msg()
            if not rclpy.ok():
                break
            raw = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            raw.header.stamp = stamp
            raw.header.frame_id = self._frame_id
            self.pub_raw.publish(raw)
            with self._lock:
                self._seq += 1
                self._latest = (self._seq, stamp, frame)

    # ── 추론 ───────────────────────────────────

    def _inference_loop(self):
        last_seq = 0
        while self._running and rclpy.ok():
            rate = max(0.5, float(self.get_parameter('inference_rate_hz').value))
            time.sleep(1.0 / rate)
            with self._lock:
                latest = self._latest
            if latest is None or latest[0] == last_seq:
                continue
            last_seq = latest[0]
            try:
                self._process(latest[1], latest[2])
            except Exception as e:                      # noqa: BLE001 - 한 프레임 실패로 죽지 않는다
                self.get_logger().error(f'추론 실패: {e}', throttle_duration_sec=5.0)

    def _process(self, stamp, frame):
        if self._mask_source == 'color':
            self._process_color(stamp, frame)
        else:
            self._process_yolo(stamp, frame)

    def _process_color(self, stamp, frame):
        """HSV 색상 마스크 경로 — 파지 거리 기본값. GPU 를 쓰지 않는다."""
        blob, mask = wrist_color_mask.find_box(
            frame, roi=self._roi, min_area=self._min_blob,
            thin_px=self._thin_px, trim_frac=self._trim_frac)

        array = DetectedObjectArray()
        array.header.stamp = stamp
        array.header.frame_id = self._frame_id
        best = None
        if blob is not None:
            x1, y1, x2, y2 = blob['bbox']
            obj = DetectedObject()
            obj.class_id = 0
            obj.class_name = 'box-color'
            # 색상 경로에는 확신도 개념이 없다 — 있는 척하지 않고 1.0 고정으로 두고,
            # 신뢰 판단은 metrics 의 fill/occluded/border_contacts 로 한다.
            obj.confidence = 1.0
            obj.pose = Pose()
            obj.pose.orientation.w = 1.0
            roi_msg = RegionOfInterest()
            roi_msg.x_offset, roi_msg.y_offset = max(int(x1), 0), max(int(y1), 0)
            roi_msg.width, roi_msg.height = max(int(x2 - x1), 0), max(int(y2 - y1), 0)
            obj.bbox = roi_msg
            array.objects.append(obj)
            best = (1.0, 0, blob['bbox'], blob['area'], blob['centroid'])

        if not rclpy.ok():
            return
        self.pub_objects.publish(array)
        # `trimmed_px` 를 같이 실어 보낸다 — 0 이 아니면 그 프레임엔 케이블 같은 게 붙어
        # 있었다는 뜻이라, 실측 스크립트가 흔들린 표본을 사후에 분리할 수 있다.
        self._publish_metrics(
            stamp, best,
            None if blob is None else {'trimmed_px': blob['trimmed_px']})

        if self.pub_debug.get_subscription_count() > 0:
            display = frame.copy()
            region = mask > 0
            display[region] = (display[region] * 0.5
                               + np.array(COLOR, dtype=np.float32) * 0.5).astype(np.uint8)
            x0, y0, x1r, y1r = wrist_color_mask.roi_rect(self._w, self._h, self._roi)
            cv2.rectangle(display, (x0, y0), (x1r - 1, y1r - 1), (0, 255, 255), 1)
            if blob is not None:
                bx1, by1, bx2, by2 = (int(v) for v in blob['bbox'])
                cv2.rectangle(display, (bx1, by1), (bx2, by2), COLOR, 2)
                cv2.putText(display,
                            f'box-color {blob["area"]}px'
                            + (f' (-{blob["trimmed_px"]})' if blob['trimmed_px'] else ''),
                            (bx1, max(by1 - 8, 12)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, COLOR, 2, cv2.LINE_AA)
            debug = self._bridge.cv2_to_imgmsg(display, encoding='bgr8')
            debug.header.stamp = stamp
            debug.header.frame_id = self._frame_id
            self.pub_debug.publish(debug)

    def _process_yolo(self, stamp, frame):
        results = self._model.predict(frame, conf=self._conf, verbose=False)
        r0 = results[0]
        polygons = None if r0.masks is None else r0.masks.xy

        array = DetectedObjectArray()
        array.header.stamp = stamp
        array.header.frame_id = self._frame_id

        want_debug = self.pub_debug.get_subscription_count() > 0
        display = frame.copy() if want_debug else None

        best = None            # (confidence, index, bbox, mask_pixels, centroid)
        for i, box in enumerate(r0.boxes):
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])

            obj = DetectedObject()
            obj.class_id = cls_id
            obj.class_name = self._model.names[cls_id]
            obj.confidence = conf
            # depth 가 없으므로 pose 는 비운다 — 모듈 docstring 참고.
            obj.pose = Pose()
            obj.pose.orientation.w = 1.0
            roi = RegionOfInterest()
            roi.x_offset, roi.y_offset = max(int(x1), 0), max(int(y1), 0)
            roi.width, roi.height = max(int(x2 - x1), 0), max(int(y2 - y1), 0)
            obj.bbox = roi
            array.objects.append(obj)

            mask_px, centroid = self._mask_stats(polygons, i, (x1, y1, x2, y2))
            # ⚠️ ROI 밖 검출은 후보에서 뺀다. 2026-08-13 실측에서 배경 선반의 택배 상자가
            # confidence 0.97 로 이겨 **그리퍼에 물린 상자 대신 배경이** 지표로 나갔다.
            # 그리퍼는 화면에서 늘 같은 자리라, 위치가 confidence 보다 강한 근거다.
            x0r, y0r, x1r, y1r = wrist_color_mask.roi_rect(self._w, self._h, self._roi)
            inside = x0r <= centroid[0] < x1r and y0r <= centroid[1] < y1r
            if inside and (best is None or conf > best[0]):
                best = (conf, i, (x1, y1, x2, y2), mask_px, centroid)

            if want_debug:
                self._draw(display, i, (x1, y1, x2, y2), obj.class_name, conf, polygons)

        if not rclpy.ok():
            return
        self.pub_objects.publish(array)
        self._publish_metrics(stamp, best)
        if want_debug:
            debug = self._bridge.cv2_to_imgmsg(display, encoding='bgr8')
            debug.header.stamp = stamp
            debug.header.frame_id = self._frame_id
            self.pub_debug.publish(debug)

    def _mask_stats(self, polygons, index, bbox):
        """마스크 픽셀 수 + 중심. seg 모델이 아니면 bbox 로 폴백한다."""
        if polygons is not None and index < len(polygons):
            polygon = np.asarray(polygons[index], dtype=np.float32)
            if polygon.ndim == 2 and polygon.shape[0] >= 3:
                polygon[:, 0] = np.clip(polygon[:, 0], 0, self._w - 1)
                polygon[:, 1] = np.clip(polygon[:, 1], 0, self._h - 1)
                mask = np.zeros((self._h, self._w), dtype=np.uint8)
                cv2.fillPoly(mask, [np.rint(polygon).astype(np.int32)], 1)
                pixels = int(mask.sum())
                if pixels > 0:
                    ys, xs = np.nonzero(mask)
                    return pixels, (float(xs.mean()), float(ys.mean()))
        x1, y1, x2, y2 = bbox
        return (int(max(0.0, x2 - x1) * max(0.0, y2 - y1)),
                ((x1 + x2) / 2.0, (y1 + y2) / 2.0))

    def _publish_metrics(self, stamp, best, extra=None):
        payload = {'stamp': stamp.sec + stamp.nanosec * 1e-9, 'detected': best is not None}
        if best is not None:
            conf, _, bbox, mask_px, centroid = best
            payload.update(wrist_metrics.summarize(
                mask_pixels=mask_px, cx_px=centroid[0], cy_px=centroid[1],
                bbox=bbox, frame_w=self._w, frame_h=self._h,
                f_px=self._f_px, real_size_m=self._box_size))
            payload['confidence'] = conf
            if extra:
                payload.update(extra)
        self.pub_metrics.publish(String(data=json.dumps(payload)))

    def _draw(self, img, index, bbox, class_name, conf, polygons):
        x1, y1, x2, y2 = (int(v) for v in bbox)
        if polygons is not None and index < len(polygons):
            polygon = np.asarray(polygons[index], dtype=np.float32)
            if polygon.ndim == 2 and polygon.shape[0] >= 3:
                polygon[:, 0] = np.clip(polygon[:, 0], 0, self._w - 1)
                polygon[:, 1] = np.clip(polygon[:, 1], 0, self._h - 1)
                mask = np.zeros((self._h, self._w), dtype=np.uint8)
                cv2.fillPoly(mask, [np.rint(polygon).astype(np.int32)], 1)
                region = mask.astype(bool)
                img[region] = (img[region] * 0.5
                               + np.array(COLOR, dtype=np.float32) * 0.5).astype(np.uint8)
        cv2.rectangle(img, (x1, y1), (x2, y2), COLOR, 2)
        cv2.putText(img, f'{class_name} {conf:.2f}', (x1, max(y1 - 8, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, COLOR, 2, cv2.LINE_AA)

    # ── 종료 ───────────────────────────────────

    def destroy_node(self):
        self._running = False
        if self._cap is not None:
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WristCameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
