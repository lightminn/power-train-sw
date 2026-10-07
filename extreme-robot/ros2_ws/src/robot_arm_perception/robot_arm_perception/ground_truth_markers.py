"""줄자로 잰 "박스 실제 위치"를 RViz에 와이어프레임으로 그린다 (2026-08-07 추가).

`camera_tf_tuner`로 카메라 TF를 끌 때 **맞았다는 기준**이 필요하다. 이 노드가 그
기준이다 — 박스를 실제로 놓은 자리를 `base_link` 기준으로 입력하면 그 위치에
와이어프레임 큐브를 띄운다. `detection_markers`가 그리는 검출 큐브(초록/파랑)가
이 와이어프레임 안에 들어오면 캘리브가 맞은 것이다.

좌표는 **cm**로 받는다 — `calibrate_camera_pose.py`의 입력 규약과 같게 맞췄다
(REP-103: x=앞(+), y=왼쪽(+)/오른쪽(-), z=위(+)).

    ros2 run robot_arm_perception ground_truth_markers --ros-args \\
        -p points:="30,0,3; 35,15,3; 38,-15,8"

실행 중에 바꿔도 된다:

    ros2 param set /ground_truth_markers points "40,0,3"

⚠️ 이 노드는 "정답"을 그리는 게 아니라 **당신이 잰 값**을 그린다. 2026-08-07 실측에서
측정 기준점이 `base_link`가 아니어서 19cm 어긋난 적이 있는데, 그런 경우 이 큐브 자체가
틀린 자리에 뜬다. 큐브가 로봇 모델 대비 엉뚱한 데 있으면 **카메라가 아니라 측정 기준을
먼저 의심할 것.**
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

TOPIC = "/perception/ground_truth_markers"
NS_CUBE = "ground_truth"
NS_TEXT = "ground_truth_label"

COLOR = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)   # 흰색 — 검출 큐브(초록/파랑)와 구분

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

#: 큐브 12개 모서리 (단위 큐브 꼭짓점 인덱스 쌍)
_EDGES = ((0, 1), (1, 3), (3, 2), (2, 0),
          (4, 5), (5, 7), (7, 6), (6, 4),
          (0, 4), (1, 5), (2, 6), (3, 7))


def parse_points(text: str):
    """'30,0,3; 35,15,3' → [(0.30, 0.0, 0.03), ...]  (cm 입력 → m 반환)"""
    points = []
    for chunk in text.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        values = [v for v in chunk.replace(",", " ").split() if v]
        if len(values) != 3:
            raise ValueError(f"'{chunk}' — 숫자 3개(x y z, cm)여야 합니다")
        points.append(tuple(float(v) / 100.0 for v in values))
    return points


class GroundTruthMarkers(Node):
    def __init__(self):
        super().__init__("ground_truth_markers")

        self.declare_parameter("points", "")
        self.declare_parameter("frame", "base_link")
        self.declare_parameter("size_cm", 8.0)
        self.declare_parameter("line_width", 0.004)

        self._points = self._read_points(self.get_parameter("points").value)

        self.pub = self.create_publisher(MarkerArray, TOPIC, LATCHED)
        self.add_on_set_parameters_callback(self._on_params)
        # latched 로 내지만, RViz를 나중에 켜는 경우를 대비해 느리게 재발행한다.
        self.create_timer(2.0, self._publish)
        self._publish()

        self.get_logger().info(
            f"ground_truth_markers 시작 — {len(self._points)}점, 토픽={TOPIC}"
            + ("" if self._points else
               "  ⚠️ points 가 비어 있습니다 — -p points:=\"30,0,3; 35,15,3\" 처럼 지정하세요"))

    def _read_points(self, text):
        try:
            return parse_points(text)
        except ValueError as e:
            self.get_logger().error(f"points 파싱 실패: {e}")
            return []

    def _on_params(self, params):
        for param in params:
            if param.name == "points":
                self._points = self._read_points(param.value)
                self.get_logger().info(f"points 갱신 — {len(self._points)}점")
        self._publish()
        return SetParametersResult(successful=True)

    def _publish(self):
        frame = self.get_parameter("frame").value
        half = float(self.get_parameter("size_cm").value) / 200.0   # cm → m 의 절반
        width = float(self.get_parameter("line_width").value)
        stamp = self.get_clock().now().to_msg()

        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        for i, (x, y, z) in enumerate(self._points):
            corners = [
                Point(x=x + sx * half, y=y + sy * half, z=z + sz * half)
                for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)
            ]
            cube = Marker()
            cube.header.frame_id = frame
            cube.header.stamp = stamp
            cube.ns = NS_CUBE
            cube.id = i
            cube.type = Marker.LINE_LIST
            cube.action = Marker.ADD
            cube.pose.orientation.w = 1.0
            cube.scale = Vector3(x=width, y=0.0, z=0.0)
            cube.color = COLOR
            for a, b in _EDGES:
                cube.points.append(corners[a])
                cube.points.append(corners[b])
            array.markers.append(cube)

            label = Marker()
            label.header.frame_id = frame
            label.header.stamp = stamp
            label.ns = NS_TEXT
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = z + half + 0.02
            label.pose.orientation.w = 1.0
            label.scale.z = 0.03
            label.color = COLOR
            # RViz(Ogre 기본 폰트)는 한글이 두부(□)로 깨진다 — 화면 문자열은 영어로.
            label.text = f"GT{i + 1} {x * 100:.0f},{y * 100:.0f},{z * 100:.0f}"
            array.markers.append(label)

        self.pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthMarkers()
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
