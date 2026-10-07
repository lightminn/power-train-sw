"""RViz에서 드래그로 카메라 TF를 맞추는 노드 (2026-08-07 추가).

## 무엇

`base_link → camera_link` 변환을 **런타임에 바꿀 수 있는** 형태로 발행한다. RViz의
6-DOF 인터랙티브 마커를 끌면 카메라 프레임이 따라 움직이고, 검출된 박스 마커
(`detection_markers`)가 같이 움직인다. 실제 박스 위치(`ground_truth_markers`의
와이어프레임)에 겹칠 때까지 끌면 그게 캘리브 값이다.

`camera_tf.launch.py`(확정값 static 발행)의 **조정용 대체품**이다. 값이 정해지면
여기서 나온 숫자를 그 launch의 기본값에 박아 넣고 이 노드는 안 쓴다.

⚠️ **`camera_tf.launch.py`와 동시에 띄우지 말 것** — 같은 TF를 둘이 발행하면 TF 트리가
깨진다(수신 측이 둘을 번갈아 보게 됨). 기동 시 `base_to_camera_link` 노드가 이미 떠
있으면 경고를 찍는다.

## 다점 최소자승(`calibrate_camera_pose.py`)과의 관계 — 대체가 아니라 보완이다

눈으로 6-DOF를 동시에 맞추는 건 수치 해보다 정밀하지 않다(특히 광축 방향 깊이 착시
때문에 좌우 정밀도가 잘 안 나오는데, 이 팔은 `arm_joint_1`에 모터가 없어 좌우 오차를
IK가 흡수하지 못한다). 반대로 수치 해는 **측정 기준점이 통째로 틀린 오차를 원리적으로
못 잡는다**(모든 점이 같이 밀리므로 잔차가 그대로다 — 2026-08-07 실측에서 19cm 어긋난
적 있음). 그래서 순서는:

    RViz로 대충 맞추고 눈으로 검증 → 다점 해로 확정 → 다시 RViz로 검증

## 조작 (RViz 안에서 전부 끝난다)

**드래그** — 화살표=이동, 링=회전. 축은 **base_link 기준으로 고정**
(`orientation_mode=FIXED`)이라 마커를 돌려놔도 "빨강 화살표=앞"이 유지된다.

**우클릭 메뉴** — 노란 카메라 몸통을 우클릭하면 다점 캘리브를 클릭만으로 진행한다:

    측정 ▶  1) 30,0,3 측정      ← 그 자리에 박스를 놓고 클릭하면 자동으로 여러 프레임 평균
            2) 35,15,3 측정
    ▶ 해 계산 (Kabsch)          ← 3점 이상 모이면 최소자승으로 풀어 카메라를 그 자리로 이동
    ↩ 마지막 측정 취소
    ✕ 측정 초기화
    🔍 품질 진단                ← 적용하지 않고 잔차/경고만
    💾 값 저장

측정할 좌표를 미리 `gt_points`로 넘겨두면(같은 값이 `ground_truth_markers`에도 가서
흰 와이어프레임으로 보인다) **캘리브 중에 숫자를 타이핑할 일이 없다** — 박스를 옮기고
메뉴에서 해당 번호를 누르는 게 전부다.

**수치 입력 GUI가 필요하면** `rqt_reconfigure`를 같이 띄운다(`camera_calib.launch.py`의
`rqt:=true`). RViz2는 파이썬 패널 플러그인을 지원하지 않아 슬라이더 위젯을 새로 만들려면
C++ 플러그인을 컴파일해야 하는데, 이 노드의 자세가 전부 ROS 파라미터라
`rqt_reconfigure`가 그대로 편집 GUI가 된다(cam_x/cam_y/… 를 숫자로 입력).

**안내문은 두 곳으로 나간다** — RViz 3D 텍스트(`/perception/calib_status`, Ogre 폰트가
한글을 못 그려 **영어**)와, 같은 내용의 한국어 판(`/perception/calib_guide`,
`std_msgs/String`)이다. 후자는 `calib_status_view` 가 별도 Tk 창에 큰 글씨로 띄운다
(`camera_calib.launch.py` 가 기본으로 같이 띄운다). 두 언어를 `_guide_lines(ko=)` 한
함수에서 만들므로 한쪽만 고쳐져 어긋나는 일이 없다.

- 미세조정은 파라미터로도 가능: `ros2 param set /camera_tf_tuner cam_x 0.305`
- 드래그를 놓을 때마다 현재 값을 `cam_x:=... ` 복붙 형태로 로그에 찍는다.
- 파일로 저장: `ros2 service call /camera_tf_tuner/save std_srvs/srv/Trigger {}`
"""
import math

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Pose, Quaternion, TransformStamped, Vector3
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import (
    InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker,
)
from interactive_markers import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from robot_arm_msgs.msg import DetectedObject
from robot_arm_perception import camera_calib_solver as solver
from robot_arm_perception.ground_truth_markers import parse_points

#: camera_link → optical frame 고정 회전 (REP-103). camera_tf.launch.py /
#: calibrate_camera_pose.py 와 반드시 같은 값이어야 한다.
OPTICAL_RPY = solver.OPTICAL_RPY

TOPIC_PICK = "/pick_target"
#: RViz 3D 뷰에 뜨는 텍스트 마커 — Ogre 폰트가 한글을 못 그려 **영어**다.
TOPIC_STATUS = "/perception/calib_status"
#: 같은 내용의 **한국어** 판. `calib_status_view`(Tk 창)가 이걸 크게 띄운다.
#: 3D 뷰 텍스트가 작고 한 덩어리라 읽기 힘든 문제 때문에 옆 창을 따로 둔다.
TOPIC_GUIDE = "/perception/calib_guide"

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

#: 한 점을 측정할 때 평균낼 프레임 수 / 그 안에 검출이 안 오면 포기할 시간.
DEFAULT_SAMPLES = 30
MEASURE_TIMEOUT_S = 15.0

#: camera_tf.launch.py 가 발행하는 static TF 노드 이름 — 중복 발행 감지용.
CONFLICTING_NODE = "base_to_camera_link"

MARKER_NAME = "camera"


def quat_from_rpy(roll, pitch, yaw) -> Quaternion:
    """ZYX(yaw-pitch-roll) — static_transform_publisher 규약과 같다."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def rpy_from_quat(q) -> tuple:
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                      1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


class CameraTfTuner(Node):
    def __init__(self):
        super().__init__("camera_tf_tuner")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("optical_frame", "camera_color_optical_frame")
        # 기본값은 camera_tf.launch.py 의 현재 기본값과 동일 — 거기서 이어서 조정한다.
        self.declare_parameter("cam_x", 0.123)
        self.declare_parameter("cam_y", 0.0)
        self.declare_parameter("cam_z", 0.082)
        self.declare_parameter("cam_roll", 0.0)
        self.declare_parameter("cam_pitch", -0.26)
        self.declare_parameter("cam_yaw", 0.0)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("interactive", True)
        self.declare_parameter("marker_scale", 0.15)
        self.declare_parameter("save_path", "camera_tf_calib.txt")
        # 다점 캘리브(우클릭 메뉴)용 — ground_truth_markers 와 같은 문자열을 받는다.
        self.declare_parameter("gt_points", "")
        self.declare_parameter("samples", DEFAULT_SAMPLES)
        # 3D 뷰에 단계별 가이드를 띄운다(끄면 결과/경고만 표시)
        self.declare_parameter("show_guide", True)
        self.declare_parameter("min_range", solver.DEFAULT_MIN_RANGE)
        self.declare_parameter("pair_tol", solver.DEFAULT_PAIR_TOL)

        self.base_frame = self.get_parameter("base_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.optical_frame = self.get_parameter("optical_frame").value

        self._pose = [self.get_parameter(n).value for n in
                      ("cam_x", "cam_y", "cam_z", "cam_roll", "cam_pitch", "cam_yaw")]
        self._applying = False   # 파라미터↔마커 갱신이 서로를 다시 부르는 것 방지

        self._warn_if_static_publisher_running()

        self._tf = TransformBroadcaster(self)
        self._static_tf = StaticTransformBroadcaster(self)
        self._publish_optical_tf()

        rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / rate, self._publish_tf)

        self.add_on_set_parameters_callback(self._on_params)
        self.create_service(Trigger, "~/save", self._on_save)

        # ── 다점 캘리브 상태 ──
        self._truths = self._read_gt_points()
        self._measured = {}     # gt 인덱스 → 관측 좌표(np.array, optical frame)
        self._order = []        # 측정 순서 (취소용)
        self._collecting = None
        self._status_lines = []
        self._status_lines_ko = []
        self._solved = False
        self._last_pick = None      # (수신 시각, confidence) — "박스가 지금 보이나" 표시용
        self.pub_status = self.create_publisher(Marker, TOPIC_STATUS, LATCHED)
        self.pub_guide = self.create_publisher(String, TOPIC_GUIDE, LATCHED)
        self.create_subscription(DetectedObject, TOPIC_PICK, self._on_pick, LATCHED)
        self.create_timer(1.0, self._check_measure_timeout)

        self._server = None
        self._menu = None
        if self.get_parameter("interactive").value:
            self._server = InteractiveMarkerServer(self, "camera_tf_tuner")
            self._server.insert(self._make_marker(), feedback_callback=self._on_feedback)
            self._menu = self._make_menu()
            self._menu.apply(self._server, MARKER_NAME)
            self._server.applyChanges()
        self._publish_status()

        self.get_logger().info(
            f"camera_tf_tuner 시작 — {self.base_frame}→{self.camera_frame} @ {rate:.0f}Hz, "
            f"인터랙티브={'on' if self._server else 'off'}")
        self._log_values("초기값")

    # ── TF 발행 ────────────────────────────────

    def _publish_optical_tf(self):
        """camera_link → optical frame 은 절대 안 변하므로 static 으로 한 번만."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.camera_frame
        t.child_frame_id = self.optical_frame
        t.transform.rotation = quat_from_rpy(*OPTICAL_RPY)
        self._static_tf.sendTransform(t)

    def _publish_tf(self):
        x, y, z, roll, pitch, yaw = self._pose
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.camera_frame
        t.transform.translation = Vector3(x=float(x), y=float(y), z=float(z))
        t.transform.rotation = quat_from_rpy(roll, pitch, yaw)
        self._tf.sendTransform(t)

    # ── 인터랙티브 마커 ────────────────────────

    def _make_marker(self) -> InteractiveMarker:
        scale = float(self.get_parameter("marker_scale").value)

        im = InteractiveMarker()
        im.header.frame_id = self.base_frame
        im.name = MARKER_NAME
        im.description = "camera_link"
        im.scale = scale
        im.pose = self._current_pose_msg()

        # 카메라 몸통 — 어디를 잡는지 보이게. D435 대략 크기(90×25×25mm).
        body = Marker()
        body.type = Marker.CUBE
        body.scale = Vector3(x=0.025, y=0.090, z=0.025)
        body.color = ColorRGBA(r=0.9, g=0.9, b=0.2, a=0.9)
        visual = InteractiveMarkerControl()
        visual.always_visible = True
        # MENU 로 둬야 이 몸통을 우클릭했을 때 다점 캘리브 메뉴가 열린다.
        visual.interaction_mode = InteractiveMarkerControl.MENU
        visual.markers.append(body)
        im.controls.append(visual)

        # 이동/회전 핸들 3축. orientation_mode=FIXED 라 마커를 돌려도 축이 base_link 에
        # 고정된다 — "빨강 화살표=앞(x)" 이 항상 유지돼 조작이 예측 가능하다.
        axes = (("x", (1.0, 0.0, 0.0)), ("y", (0.0, 0.0, 1.0)), ("z", (0.0, 1.0, 0.0)))
        for name, (qx, qy, qz) in axes:
            norm = math.sqrt(1.0 + qx * qx + qy * qy + qz * qz)
            quat = Quaternion(x=qx / norm, y=qy / norm, z=qz / norm, w=1.0 / norm)
            for mode, prefix in ((InteractiveMarkerControl.MOVE_AXIS, "move"),
                                 (InteractiveMarkerControl.ROTATE_AXIS, "rotate")):
                control = InteractiveMarkerControl()
                control.orientation = quat
                control.name = f"{prefix}_{name}"
                control.interaction_mode = mode
                control.orientation_mode = InteractiveMarkerControl.FIXED
                im.controls.append(control)
        return im

    def _current_pose_msg(self) -> Pose:
        x, y, z, roll, pitch, yaw = self._pose
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
        pose.orientation = quat_from_rpy(roll, pitch, yaw)
        return pose

    def _on_feedback(self, feedback):
        if feedback.event_type not in (InteractiveMarkerFeedback.POSE_UPDATE,
                                       InteractiveMarkerFeedback.MOUSE_UP):
            return
        p = feedback.pose
        roll, pitch, yaw = rpy_from_quat(p.orientation)
        self._pose = [p.position.x, p.position.y, p.position.z, roll, pitch, yaw]
        self._sync_params()
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self._log_values("드래그")

    # ── 우클릭 메뉴 (다점 캘리브 GUI) ──────────

    def _read_gt_points(self):
        try:
            return parse_points(self.get_parameter("gt_points").value)
        except ValueError as e:
            self.get_logger().error(f"gt_points 파싱 실패: {e}")
            return []

    def _make_menu(self) -> MenuHandler:
        # RViz(Ogre 기본 폰트)는 한글을 두부(□)로 그린다 — 화면에 뜨는 문자열은 영어로.
        menu = MenuHandler()
        if self._truths:
            measure = menu.insert("Measure point")
            for i, (x, y, z) in enumerate(self._truths):
                menu.insert(
                    f"{i + 1}) at {x * 100:.0f}, {y * 100:.0f}, {z * 100:.0f} cm",
                    parent=measure,
                    callback=(lambda feedback, index=i: self._menu_measure(index)))
        else:
            # 측정 대상이 없으면 메뉴에 이유를 띄운다 — 빈 메뉴는 고장처럼 보인다.
            menu.insert("(gt_points is empty - pass it as a launch argument)",
                        callback=lambda feedback: self._set_status(
                            'gt_points not set - e.g. gt_points:="30,0,3; 35,15,3"',
                            ko='gt_points 가 비어 있습니다 — launch 인자로 넘기세요. '
                               '예: gt_points:="30,0,3; 35,15,3" (cm, base_link 기준)'))
        menu.insert("Solve (Kabsch) and apply",
                    callback=lambda f: self._menu_solve(apply=True))
        menu.insert("Diagnose only (no apply)",
                    callback=lambda f: self._menu_solve(apply=False))
        menu.insert("Undo last measurement", callback=lambda f: self._menu_undo())
        menu.insert("Clear all measurements", callback=lambda f: self._menu_clear())
        menu.insert("Save values to file", callback=lambda f: self._menu_save())
        return menu

    def _menu_measure(self, index):
        if self._collecting is not None:
            self._set_status("already measuring - wait for it to finish.",
                             ko="이미 측정 중입니다 — 끝날 때까지 기다리세요.")
            return
        self._collecting = {
            "index": index,
            "samples": [],
            "last": None,
            "deadline": self._now() + MEASURE_TIMEOUT_S,
        }
        x, y, z = self._truths[index]
        self._set_status(
            f"measuring point {index + 1} (at {x * 100:.0f},{y * 100:.0f},{z * 100:.0f} cm) "
            "- the box must be visible to the camera",
            ko=f"{index + 1}번 점 측정 중 ({x * 100:.0f},{y * 100:.0f},{z * 100:.0f} cm) "
               "— 박스가 카메라에 보여야 합니다")

    def _on_pick(self, msg):
        """측정 중일 때만 샘플을 모은다 — 검출 지터를 여러 프레임 평균으로 눌러야 한다.

        측정 중이 아니어도 수신 시각은 기록한다 — 가이드에 "지금 박스가 보이는지"를
        띄우기 위해서다. 측정이 실패하는 원인의 대부분이 "검출이 아예 안 오고 있음"이라
        누르기 전에 알 수 있어야 한다.
        """
        self._last_pick = (self._now(), float(msg.confidence))
        if self._collecting is None:
            return
        p = msg.pose.position
        sample = np.array([p.x, p.y, p.z])
        if abs(p.x) < 1e-9 and abs(p.y) < 1e-9 and abs(p.z) < 1e-9:
            return                                  # depth 없는 검출
        last = self._collecting["last"]
        if last is not None and np.array_equal(sample, last):
            return                                  # latched 재전송분
        self._collecting["last"] = sample
        self._collecting["samples"].append(sample)

        need = int(self.get_parameter("samples").value)
        got = len(self._collecting["samples"])
        if got >= need:
            self._finish_measure()
        elif got % 10 == 0:
            self._set_status(f"measuring... {got}/{need}",
                             ko=f"측정 중... {got}/{need} 프레임")

    def _check_measure_timeout(self):
        # 가이드의 "box detected" 줄이 살아있어야 하므로 매 틱 다시 그린다.
        self._publish_status()
        if self._collecting is None or self._now() < self._collecting["deadline"]:
            return
        if len(self._collecting["samples"]) >= 3:
            self._finish_measure()                  # 느리지만 쓸 만큼은 모였다
            return
        index = self._collecting["index"]
        self._collecting = None
        self._set_status(
            f"point {index + 1} FAILED - no {TOPIC_PICK} received. Check that "
            "perception_node is running, the box is visible, and pick_classes / "
            "conf_threshold are not filtering it out.",
            ko=f"{index + 1}번 점 측정 실패 — {TOPIC_PICK} 이 한 건도 안 왔습니다. "
               "perception_node 가 떠 있는지, 박스가 화면에 보이는지, "
               "pick_classes / conf_threshold 가 걸러내고 있지 않은지 확인하세요.")

    def _finish_measure(self):
        index = self._collecting["index"]
        samples = np.array(self._collecting["samples"])
        self._collecting = None

        mean = samples.mean(axis=0)
        std = samples.std(axis=0)
        truth = np.array(self._truths[index])

        if index not in self._measured:
            self._order.append(index)
        self._measured[index] = mean

        measurement = (f"  observed {mean[0]:+.3f},{mean[1]:+.3f},{mean[2]:+.3f} m  "
                       f"jitter sd=({std[0] * 100:.1f},{std[1] * 100:.1f},"
                       f"{std[2] * 100:.1f})cm")
        lines = [f"point {index + 1} measured ({len(samples)} frames averaged)", measurement]
        lines_ko = [f"{index + 1}번 점 측정 완료 ({len(samples)} 프레임 평균)", measurement]

        # 가드 — 문제를 '지금' 알려야 다시 재는 삽질이 없다.
        # 경고문 자체는 solver(=터미널 도구와 공유하는 단일 출처)가 영어로 만든다.
        warnings = []
        warning = solver.check_range(mean, float(self.get_parameter("min_range").value))
        if warning:
            warnings.append(warning)
        others = [i for i in self._order if i != index]
        warnings.extend(solver.check_pair(
            mean, truth,
            [self._measured[i] for i in others],
            [np.array(self._truths[i]) for i in others],
            float(self.get_parameter("pair_tol").value)))
        lines.extend("⚠️ " + w for w in warnings)
        lines_ko.extend("⚠️ " + w for w in warnings)

        self._set_status("\n".join(lines), ko="\n".join(lines_ko))

    def _menu_solve(self, *, apply):
        indexes = [i for i in self._order if i in self._measured]
        result = solver.solve(
            [self._measured[i] for i in indexes],
            [self._truths[i] for i in indexes],
            min_range=float(self.get_parameter("min_range").value),
            pair_tol=float(self.get_parameter("pair_tol").value))
        if result is None:
            self._set_status(f"only {len(indexes)} point(s) - measure at least 3.",
                             ko=f"점이 {len(indexes)}개뿐입니다 — 최소 3개를 측정하세요.")
            return

        report = solver.format_report(result)
        if not apply:
            self._set_status("DIAGNOSE (not applied)\n" + report,
                             ko="진단만 함 (적용 안 됨)\n" + report)
            return

        self._pose = list(result["xyz"]) + list(result["rpy"])
        self._solved = True
        self._sync_params()
        if self._server is not None:
            self._server.setPose(MARKER_NAME, self._current_pose_msg())
            self._server.applyChanges()
        self._set_status("SOLVED and applied\n" + report,
                         ko="해를 구해 카메라 자세에 적용했습니다\n" + report)

    def _menu_undo(self):
        if not self._order:
            self._set_status("nothing to undo.", ko="취소할 측정이 없습니다.")
            return
        index = self._order.pop()
        self._measured.pop(index, None)
        self._set_status(f"undid point {index + 1} ({len(self._order)} left)",
                         ko=f"{index + 1}번 점 측정을 취소했습니다 (남은 점 {len(self._order)}개)")

    def _menu_clear(self):
        self._measured.clear()
        self._order.clear()
        self._collecting = None
        self._solved = False
        self._set_status("measurements cleared.", ko="측정값을 모두 지웠습니다.")

    def _menu_save(self):
        response = self._on_save(None, Trigger.Response())
        self._set_status(response.message,
                         ko=("저장됨: " + self.get_parameter("save_path").value
                             if response.success else response.message))

    # ── 상태 표시 (RViz 3D 텍스트) ─────────────

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _set_status(self, text, ko=None):
        """`text` 는 RViz 3D 텍스트용 영어, `ko` 는 Tk 창(`calib_status_view`)용 한국어.

        `ko` 를 안 주면 영어를 그대로 쓴다 — solver 리포트처럼 숫자 위주라 번역이
        무의미하거나, 터미널 도구(`calibrate_camera_pose.py`)와 문구를 맞춰야 하는 경우다.
        """
        self._status_lines = text.split("\n")
        self._status_lines_ko = (ko if ko is not None else text).split("\n")
        for line in self._status_lines_ko:
            self.get_logger().info(line)
        self._publish_status()

    def _current_step(self) -> int:
        """지금 어느 단계인지 — 가이드가 상태에 따라 바뀌어야 따라갈 수 있다."""
        if self._solved:
            return 4                                   # 검증 & 저장
        if len(self._order) >= 3:
            return 3                                   # 풀 준비 됨
        if self._order or self._collecting is not None:
            return 2                                   # 측정 중
        return 1                                       # 거친 정렬

    def _detection_line(self, ko=False) -> str:
        """'지금 박스가 보이나' — 측정 실패 원인 1순위라 누르기 전에 보여준다."""
        if self._last_pick is None:
            return ("박스 인식: 없음 (/pick_target 을 한 번도 못 받음)" if ko else
                    "box detected: NO (never received /pick_target)")
        age = self._now() - self._last_pick[0]
        if age > 2.0:
            return (f"박스 인식: 없음 ({age:.0f}초 전이 마지막)" if ko else
                    f"box detected: NO (last seen {age:.0f}s ago)")
        return (f"박스 인식: 있음 (확신도 {self._last_pick[1]:.2f})" if ko else
                f"box detected: YES (conf {self._last_pick[1]:.2f})")

    def _guide_lines(self, ko=False):
        """단계 체크리스트 + '지금 할 일' 한 줄.

        영어/한국어를 **한 함수에서** 만든다 — 두 벌로 나누면 한쪽만 고쳐져서 어긋난다.
        영어는 RViz 3D 텍스트(Ogre 폰트가 한글을 두부로 그림), 한국어는 Tk 창용이다.
        """
        def t(en, kr):
            return kr if ko else en

        step = self._current_step()
        done, total = len(self._order), len(self._truths)
        checks = "".join("[x]" if i in self._measured else "[ ]"
                         for i in range(total))

        steps = [
            (1, t("Coarse align   drag the yellow camera box",
                  "거친 정렬     노란 카메라 상자를 드래그")),
            (2, t(f"Measure points {done}/{total} {checks}",
                  f"점 측정       {done}/{total} {checks}")),
            (3, t("Solve          right-click > Solve (Kabsch)",
                  "해 계산       우클릭 > Solve (Kabsch)")),
            (4, t("Verify & Save  green cube inside white box",
                  "검증 & 저장   초록 큐브가 흰 와이어프레임 안에")),
        ]
        lines = [t("=== CAMERA TF CALIBRATION ===", "=== 카메라 TF 캘리브레이션 ===")]
        lines += [f"{'>>' if n == step else '  '} {n}. {text}" for n, text in steps]
        lines.append("-" * 30)

        now = t("NOW: ", "지금: ")
        if step == 1:
            lines.append(now + t("pick the 'Interact' tool in the top toolbar,",
                                 "RViz 상단 툴바에서 'Interact' 도구를 고른 뒤,"))
            lines.append(t("then drag the yellow box until the point cloud's",
                           "포인트클라우드의 책상/차체가 로봇 모델과 겹칠 때까지"))
            lines.append(t("table/chassis overlaps the robot model.",
                           "노란 상자를 드래그하세요."))
            lines.append(t("(do NOT align on the arm links - without servos",
                           "(팔 링크에 맞추지 말 것 — 서보가 없으면 모델의"))
            lines.append(t(" the model's joint angles are not the real ones)",
                           " 관절각이 실제 각도가 아닙니다)"))
        elif step == 2:
            nxt = next((i for i in range(total) if i not in self._measured), None)
            if self._collecting is not None:
                index = self._collecting["index"]
                lines.append(now + t(f"hold still, measuring point {index + 1}...",
                                     f"움직이지 말고 대기 — {index + 1}번 점 측정 중..."))
            elif nxt is None:
                lines.append(now + t("all points measured - right-click > Solve",
                                     "점을 다 쟀습니다 — 우클릭 > Solve (Kabsch)"))
            else:
                x, y, z = self._truths[nxt]
                spot = f"({x * 100:.0f}, {y * 100:.0f}, {z * 100:.0f} cm)"
                lines.append(now + t(f"put the box at GT{nxt + 1} {spot},",
                                     f"박스를 {nxt + 1}번 자리 {spot} 에 놓고,"))
                lines.append(t("then right-click the yellow box >",
                               "노란 상자를 우클릭 >"))
                lines.append(t(f"Measure point > {nxt + 1})",
                               f"Measure point > {nxt + 1}) 을 누르세요"))
        elif step == 3:
            lines.append(now + t("right-click > Solve (Kabsch) and apply",
                                 "우클릭 > Solve (Kabsch) and apply"))
            lines.append(t("(more points = better; 5-6 is ideal)",
                           "(점이 많을수록 좋습니다 — 5~6개가 이상적)"))
        else:
            lines.append(now + t("put the box at any GT spot and check the",
                                 "아무 기준점에나 박스를 놓고, 초록 큐브가"))
            lines.append(t("green cube lands inside the white wireframe.",
                           "흰 와이어프레임 안에 들어오는지 확인하세요."))
            lines.append(t("Then right-click > Save values to file,",
                           "맞으면 우클릭 > Save values to file 로 저장하고,"))
            lines.append(t("and copy them into camera_tf.launch.py",
                           "그 값을 camera_tf.launch.py 기본값에 넣으세요."))

        lines.append(self._detection_line(ko))
        return lines

    def _status_text(self, ko=False) -> str:
        lines = self._guide_lines(ko) if self.get_parameter("show_guide").value else []
        status = self._status_lines_ko if ko else self._status_lines
        if status:
            lines = lines + ["-" * 30] + status
        return "\n".join(lines)

    def _publish_status(self):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "calib_status"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.z = 0.75      # 로봇 모델 위로 띄워 가리지 않게
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.035
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.6, a=1.0)
        marker.text = self._status_text(ko=False)
        self.pub_status.publish(marker)
        # 같은 내용의 한국어 판 — 옆 창(calib_status_view)이 크게 띄운다.
        self.pub_guide.publish(String(data=self._status_text(ko=True)))

    # ── 파라미터 ↔ 마커 동기화 ─────────────────

    def _sync_params(self):
        """마커에서 바뀐 값을 파라미터에도 반영 — `ros2 param get` 이 실제 값을 보게."""
        from rclpy.parameter import Parameter
        self._applying = True
        try:
            self.set_parameters([
                Parameter(name, Parameter.Type.DOUBLE, float(value))
                for name, value in zip(
                    ("cam_x", "cam_y", "cam_z", "cam_roll", "cam_pitch", "cam_yaw"),
                    self._pose)
            ])
        finally:
            self._applying = False

    def _on_params(self, params):
        """`ros2 param set` 으로 mm 단위 미세조정 — 마커도 따라 움직인다."""
        if self._applying:
            return SetParametersResult(successful=True)
        index = {"cam_x": 0, "cam_y": 1, "cam_z": 2,
                 "cam_roll": 3, "cam_pitch": 4, "cam_yaw": 5}
        changed = False
        for param in params:
            if param.name in index:
                self._pose[index[param.name]] = float(param.value)
                changed = True
        if changed:
            if self._server is not None:
                self._server.setPose(MARKER_NAME, self._current_pose_msg())
                self._server.applyChanges()
            self._log_values("param")
        return SetParametersResult(successful=True)

    # ── 값 출력/저장 ───────────────────────────

    def _format_values(self) -> str:
        x, y, z, roll, pitch, yaw = self._pose
        return (f"cam_x:={x:.4f} cam_y:={y:.4f} cam_z:={z:.4f} "
                f"cam_roll:={roll:.4f} cam_pitch:={pitch:.4f} cam_yaw:={yaw:.4f}")

    def _log_values(self, tag: str):
        x, y, z, roll, pitch, yaw = self._pose
        self.get_logger().info(
            f"[{tag}] {self._format_values()}\n"
            f"         (앞 {x * 100:.1f}, 좌우 {y * 100:+.1f}, 높이 {z * 100:.1f} cm / "
            f"roll {math.degrees(roll):+.1f}° pitch {math.degrees(pitch):+.1f}° "
            f"yaw {math.degrees(yaw):+.1f}°)")

    def _on_save(self, request, response):
        path = self.get_parameter("save_path").value
        try:
            with open(path, "w") as handle:
                handle.write(
                    "# camera_tf_tuner 결과 — camera_tf.launch.py 기본값에 반영할 것\n"
                    f"ros2 launch robot_arm_description camera_tf.launch.py \\\n"
                    f"  {self._format_values()}\n")
        except OSError as e:
            response.success = False
            response.message = f"save failed: {e}"
            return response
        response.success = True
        # 이 메시지는 _menu_save 를 통해 RViz 상태 텍스트로도 나간다 → 영어.
        response.message = f"saved: {path}"
        self.get_logger().info(response.message)
        return response

    # ── 중복 발행 감지 ─────────────────────────

    def _warn_if_static_publisher_running(self):
        if CONFLICTING_NODE in self.get_node_names():
            self.get_logger().error(
                f"'{CONFLICTING_NODE}' 노드가 이미 떠 있습니다 — camera_tf.launch.py 가 "
                f"같은 {self.base_frame}→{self.camera_frame} TF를 발행 중입니다. "
                "둘이 같이 발행하면 TF 트리가 깨집니다. 한쪽을 끄세요.")


def main(args=None):
    rclpy.init(args=args)
    node = CameraTfTuner()
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
