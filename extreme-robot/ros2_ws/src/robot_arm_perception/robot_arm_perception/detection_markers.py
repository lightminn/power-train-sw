"""`/detected_objects`·`/pick_target` → RViz MarkerArray 변환 (2026-08-07 추가).

## 왜 필요한가

`perception_node`가 내는 `DetectedObjectArray`는 커스텀 메시지라 **RViz가 그릴 줄
모른다.** 그래서 지금까지 "박스가 3D 어디로 검출됐는지"를 화면에서 확인할 방법이
`/perception/debug_image`(2D 오버레이)밖에 없었다.

카메라 TF 캘리브레이션에서 이게 특히 아프다 — 2026-08-07 실측에서, 박스 좌표를
`base_link`가 아닌 다른 기준으로 재는 바람에 카메라 위치가 19cm 어긋난 적이 있는데,
**이런 상수 오프셋은 최소자승 잔차에 원리적으로 안 나타난다**(모든 점이 같이 밀리므로).
반면 RViz에 로봇 모델과 함께 띄우면 "박스가 팔 옆구리에 박혀 있네" 하고 즉시 보인다.

## 무엇을 그리는가

- 검출 객체마다 **반투명 큐브**(pose 위치) + **텍스트**(클래스/신뢰도/거리)
- `/pick_target`으로 뽑힌 객체는 **초록**, 나머지는 **파랑** — `perception_node`의
  debug_image 색 규약과 같게 맞췄다(화면 둘을 나란히 봐도 헷갈리지 않게).

frame_id는 수신 메시지 헤더를 그대로 쓴다(보통 `camera_color_optical_frame`).
따라서 `base_link` 기준으로 보려면 카메라 TF가 떠 있어야 한다 —
`camera_tf_tuner`(드래그로 조정) 또는 `camera_tf.launch.py`(확정값) 중 하나.

⚠️ depth를 못 얻은 객체는 position이 (0,0,0)이라 전부 카메라 원점에 겹쳐 쌓인다.
그래서 기본값(`skip_zero_pose=True`)으로 걸러낸다 — 끄면 "검출은 됐는데 depth가
안 나온" 경우를 눈으로 확인할 수 있다.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from builtin_interfaces.msg import Duration
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker, MarkerArray
from robot_arm_msgs.msg import DetectedObject, DetectedObjectArray

TOPIC_OBJECTS = "/detected_objects"
TOPIC_PICK = "/pick_target"
TOPIC_MARKERS = "/perception/detection_markers"

NS_BOX = "detection"
NS_TEXT = "detection_label"

# perception_node._draw_debug 와 같은 색 규약 (pick=초록 / 나머지=파랑)
COLOR_PICK = (0.1, 0.9, 0.2)
COLOR_OTHER = (0.2, 0.5, 1.0)

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class DetectionMarkerNode(Node):
    def __init__(self):
        super().__init__("detection_markers")

        self.declare_parameter("marker_size", 0.08)      # 큐브 한 변 [m]
        self.declare_parameter("alpha", 0.45)            # 큐브 투명도
        self.declare_parameter("text_height", 0.035)     # 라벨 글자 크기 [m]
        self.declare_parameter("skip_zero_pose", True)   # depth 없는 검출 숨김
        self.declare_parameter("lifetime", 0.5)          # 마커 수명 [s] (0=영구)

        self._pick_key = None

        self.pub = self.create_publisher(MarkerArray, TOPIC_MARKERS, 1)
        self.create_subscription(
            DetectedObjectArray, TOPIC_OBJECTS, self._on_objects, 10)
        self.create_subscription(DetectedObject, TOPIC_PICK, self._on_pick, LATCHED)

        self.get_logger().info(
            f"detection_markers 시작 — {TOPIC_OBJECTS} → {TOPIC_MARKERS} "
            "(RViz에서 MarkerArray로 추가)")

    # ── 콜백 ───────────────────────────────────

    def _on_pick(self, msg):
        """pick 타겟은 latched 별도 토픽으로 오므로 '어느 객체가 pick인지'를 따로 기억한다.

        DetectedObject에 고유 id가 없어 (클래스 + 위치)로 대조한다. 위치는 프레임마다
        수 mm 흔들리므로 정확히 같길 기대할 수 없다 — 그래서 근접 판정을 쓴다.
        """
        p = msg.pose.position
        self._pick_key = (msg.class_name, p.x, p.y, p.z)

    def _is_pick(self, obj):
        if self._pick_key is None:
            return False
        name, px, py, pz = self._pick_key
        if obj.class_name != name:
            return False
        p = obj.pose.position
        return math.dist((p.x, p.y, p.z), (px, py, pz)) < 0.05

    def _on_objects(self, msg):
        size = self.get_parameter("marker_size").value
        alpha = self.get_parameter("alpha").value
        text_h = self.get_parameter("text_height").value
        skip_zero = self.get_parameter("skip_zero_pose").value
        lifetime = self.get_parameter("lifetime").value

        array = MarkerArray()
        # 지난 프레임 마커를 지운다 — 검출 개수가 줄면 남은 마커가 유령으로 떠 있게 된다.
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        for i, obj in enumerate(msg.objects):
            p = obj.pose.position
            if skip_zero and abs(p.x) < 1e-6 and abs(p.y) < 1e-6 and abs(p.z) < 1e-6:
                continue

            pick = self._is_pick(obj)
            r, g, b = COLOR_PICK if pick else COLOR_OTHER

            cube = Marker()
            cube.header = msg.header
            cube.ns = NS_BOX
            cube.id = i
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose = obj.pose
            cube.scale = Vector3(x=size, y=size, z=size)
            cube.color = ColorRGBA(r=r, g=g, b=b, a=float(alpha))
            cube.lifetime = _duration(lifetime)
            array.markers.append(cube)

            label = Marker()
            label.header = msg.header
            label.ns = NS_TEXT
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = p.x
            label.pose.position.y = p.y - size          # optical 좌표에서 Y=아래 → 위쪽에 표시
            label.pose.position.z = p.z
            label.pose.orientation.w = 1.0
            label.scale.z = float(text_h)
            label.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
            label.lifetime = _duration(lifetime)
            distance = math.sqrt(p.x ** 2 + p.y ** 2 + p.z ** 2)
            mark = " [PICK]" if pick else ""
            label.text = (f"{obj.class_name} {obj.confidence:.2f}{mark}\n"
                          f"{distance * 100:.0f}cm")
            array.markers.append(label)

        self.pub.publish(array)


def _duration(seconds: float) -> Duration:
    return Duration(sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9))


def main(args=None):
    rclpy.init(args=args)
    node = DetectionMarkerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
