"""RealSense D435i + YOLO(seg) → DetectedObjectArray ROS2 퍼블리셔 (markerless).

대회 규정상 타겟 객체에 마커 부착이 금지되어 pose 추정을 markerless 로 구현한다.

Phase 2 Step 1: YOLO segmentation 추론 → class_id / class_name / confidence / bbox / mask
Phase 2 Step 2 (markerless pose):
  - Translation: 마스크 centroid color 픽셀 → depth 픽셀 투영 → depth 패치 median
      → deproject (yolo_depth_3d.py 로직 포팅). 정렬(align) 생략, 검출별 투영만.
  - Orientation: 마스크 (u,v) 픽셀에 2D PCA → 주축 각도를 카메라 광축(Z) 기준
      yaw 로 근사 → quaternion. depth 노이즈와 무관하게 안정적인 1축 근사.

좌표계(camera color optical frame): X=오른쪽, Y=아래, Z=전방 [m] (REP-103 optical).
yaw 는 optical Z 축 회전 → quaternion (0, 0, sin(θ/2), cos(θ/2)).

camera_mode 파라미터:
  realsense  실제 RealSense D435i (기본값) — depth 기반 translation 활성
  test       test_image_path 정지 이미지 반복 (하드웨어 없이 검증) — translation 0,
             orientation(2D PCA)만 채워짐

2026-07-13 파워트레인 "계약 v2"(Notion `자율주행 SW 개발계획` §3.2/§15) 반영:
D435i는 로봇팔 전용 camera owner — 원격 조종용 raw 영상은 YOLO 추론·구독자 유무를
기다리면 안 됨("raw sender는 YOLO/debug image를 기다리지 않는다"). `/perception/raw_image`를
YOLO 추론과 무관하게 발행한다(`stream_node`가 SRT :5002로 송출). 기존 `/perception/debug_image`
(오버레이, 구독자 있을 때만)는 로컬 디버그용으로 그대로 둠.

2026-07-15: "구독자 게이트 없음"만으로는 YOLO 추론이 느려지면 raw_image 발행 주기까지 같이
밀리는 문제가 있어(같은 스레드에서 캡처→추론→발행 순차 실행), 캡처 스레드(`_capture_loop`)/
추론 스레드(`_inference_loop`)로 분리. 캡처는 프레임을 받는 즉시(추론과 무관하게) raw_image를
발행하고 최신 프레임 1장만 락으로 보호된 공유 슬롯에 남기며, 추론은 그 최신 프레임만
시퀀스 번호로 소비(지연 큐 누적 없음). 같은 세션에서 RealSense 멀티 디바이스 오인식 방지용
`D435_SERIAL` 하드코딩과, depth_frame은 있는데 depth_img가 없을 때 deproject를 호출하던
버그도 수정.

2026-07-18: 대회 5개 구간이 하나의 연속 주행 안에서 순차 진행되지만 비전(카메라+YOLO)은
이 노드 하나로 통합돼 있어, 구간이 바뀔 때마다 재시작하며 `model_name` 파라미터로
`model_presets.MODEL_PRESETS`의 model_path/classes/pick_classes 세트를 한 번에 갈아
끼운다(그리퍼 preset과 동일 패턴). seg 모델(box)은 markerless pose 전체가, detect 전용
모델(traffic_light)은 bbox 중심 depth translation만 활성화되는 기존 `_get_binmask` 폴백
구조가 그대로 이 다중 모델 전환을 뒷받침한다.

2026-08-11: 그 "재시작"을 없앴다 — `model_name`/`model_path`/`task`/`backend` 를
`ros2 param set`(또는 관제 GUI)으로 바꾸면 **프로세스를 살린 채** 모델이 교체된다.
파라미터 콜백은 검증만 하고(존재하지 않는 경로는 여기서 거절 — 예전엔 그대로
프로세스가 죽었다), 실제 로드는 `self.model` 의 유일한 소비자인 추론 스레드가
프레임 사이에서 한다. 결과와 소요 시간은 `/perception/model_status`(JSON, latched)로
나간다. ⚠️ `/pick_target` 은 latched 라 교체 후에도 **이전 모델의 타깃이 남는다** —
그 사실도 model_status 에 실어 보낸다.
"""
import math
import os
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import Pose, Point, Quaternion
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import (
    RegionOfInterest, Image as ImageMsg, PointCloud2, PointField)
from std_msgs.msg import String
from cv_bridge import CvBridge
from robot_arm_msgs.msg import DetectedObject, DetectedObjectArray
from robot_arm_perception.model_presets import DEFAULT_MODEL, MODEL_PRESETS, get_preset
from .perception_quality import is_usable_bbox, robust_depth_m

D435_SERIAL = "250222071245"

# ──────────────────────────────────────────────
# TensorRT 엔진 캐시 (yolo_depth_3d.py resolve_model 포팅)
# ──────────────────────────────────────────────

def _weight_stamp(path: str) -> str:
    """가중치 파일의 내용을 대표하는 짧은 지문(크기+mtime).

    ⚠️ 엔진 캐시 이름에 이게 없으면 **같은 경로에 새 `best.pt` 를 덮어써도 낡은
    `.engine` 이 조용히 재사용된다.** 모델을 갈아끼우며 결과를 비교하는 실습에서
    가장 찾기 어려운 실패 모드라, 캐시 키에 파일 지문을 포함시킨다.
    """
    try:
        st = os.stat(path)
    except OSError:
        return 'nofile'
    return f'{st.st_size:x}-{int(st.st_mtime):x}'


def _resolve_model(path: str, backend: str, height: int, width: int) -> str:
    if backend == 'pt' or path.endswith('.engine'):
        return path
    h32 = ((height + 31) // 32) * 32
    w32 = ((width + 31) // 32) * 32
    base = os.path.splitext(path)[0]
    cached = f'{base}_{h32}x{w32}_{_weight_stamp(path)}_fp16.engine'
    if os.path.exists(cached):
        print(f'[perception] reusing cached engine: {cached}')
        return cached
    print(f'[perception] exporting TensorRT FP16 engine (imgsz={h32}x{w32}) — 5~10분 소요')
    from ultralytics import YOLO as _YOLO
    engine = _YOLO(path).export(format='engine', half=True, imgsz=(h32, w32))
    if engine != cached and os.path.exists(str(engine)):
        os.rename(str(engine), cached)
    return cached


def _load_yolo(path: str, backend: str, task: str, height: int, width: int):
    """`(model, 실제로 연 경로, 로드 초)`. 로드 시간은 화면에 그대로 표시된다."""
    from ultralytics import YOLO
    t0 = time.time()
    resolved = _resolve_model(path, backend, height, width)
    model = YOLO(resolved, task=task)
    return model, resolved, time.time() - t0


# ──────────────────────────────────────────────
# RealSense 헬퍼 (yolo_depth_3d.py latest_frames 포팅)
# ──────────────────────────────────────────────

def _latest_frames(pipe):
    frames = pipe.wait_for_frames()
    while True:
        nxt = pipe.poll_for_frames()
        if nxt.size() == 0:
            return frames
        frames = nxt


# ──────────────────────────────────────────────
# color↔depth 투영 파라미터 (yolo_depth_3d.py DepthCal 포팅)
# 전체 정렬(rs.align) 대신 검출별 color→depth 픽셀 투영만 수행 → Orin Nano
# 기준 5.5fps→30fps, 좌표 오차 평균 11mm (센서 노이즈 이내).
# ──────────────────────────────────────────────

class DepthCal:
    def __init__(self, rs, profile, dmin: float = 0.1, dmax: float = 10.0):
        dprof = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        cprof = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self.di = dprof.get_intrinsics()
        self.ci = cprof.get_intrinsics()
        self.c2d = cprof.get_extrinsics_to(dprof)
        self.d2c = dprof.get_extrinsics_to(cprof)
        self.scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.dmin, self.dmax = dmin, dmax


def _deproject_mask(rs, depth_frame, depth_img: np.ndarray,
                    binmask: np.ndarray | None,
                    cu: float, cv_: float, cal: DepthCal,
                    *, minimum_pixels: int = 20):
    """마스크 픽셀 여러 점을 투영해 robust median depth로 (X,Y,Z)[m] 산출.

    정렬(align) 없는 검출별 투영 구조는 유지하되, 이전의 단일 centroid 패치
    방식은 letterboxed 마스크 좌표가 depth와 어긋나면 배경 depth를 섞어
    측정하는 문제가 있었다(2026-07-28 D435 근접 타겟 오측정 수정). 마스크
    전역에서 다수 점을 샘플링하고 MAD 기반 이상치 제거를 거친다.
    """
    h, w = depth_img.shape
    color_points = [(cu, cv_)]
    if binmask is not None and binmask.any():
        eroded = cv2.erode(
            binmask.astype(np.uint8), np.ones((5, 5), np.uint8),
            iterations=1,
        )
        ys, xs = np.nonzero(eroded)
        if xs.size:
            indexes = np.linspace(
                0, xs.size - 1, num=min(25, xs.size), dtype=np.int64,
            )
            color_points = [
                (float(xs[index]), float(ys[index])) for index in indexes
            ]

    projected: list[tuple[int, int]] = []
    raw_values: list[np.ndarray] = []
    for color_x, color_y in color_points:
        dpx = rs.rs2_project_color_pixel_to_depth_pixel(
            depth_frame.get_data(), cal.scale, cal.dmin, cal.dmax,
            cal.di, cal.ci, cal.c2d, cal.d2c, [color_x, color_y],
        )
        dx, dy = int(round(dpx[0])), int(round(dpx[1]))
        if not (0 <= dx < w and 0 <= dy < h):
            continue
        patch = depth_img[
            max(dy - 1, 0):min(dy + 2, h),
            max(dx - 1, 0):min(dx + 2, w),
        ]
        raw_values.append(patch.reshape(-1))
        projected.append((dx, dy))
    if not raw_values:
        return None
    z = robust_depth_m(
        np.concatenate(raw_values),
        depth_scale=cal.scale,
        minimum_pixels=minimum_pixels,
        minimum_m=cal.dmin,
        maximum_m=cal.dmax,
    )
    if z is None:
        return None
    dx = float(np.median([point[0] for point in projected]))
    dy = float(np.median([point[1] for point in projected]))
    pt_d = rs.rs2_deproject_pixel_to_point(cal.di, [dx, dy], z)
    return tuple(rs.rs2_transform_point_to_point(cal.d2c, pt_d))


# ──────────────────────────────────────────────
# 2D 마스크 PCA → yaw 쿼터니언
# ──────────────────────────────────────────────

def _mask_pca_yaw_quat(xs: np.ndarray, ys: np.ndarray):
    """마스크 픽셀 (xs=u, ys=v) 에 PCA → 주축 각도 → optical Z 회전 quaternion.

    이미지 좌표 x=오른쪽, y=아래 는 optical frame X,Y 와 같은 방향이므로 주축
    각도 θ=atan2(dy,dx) 가 곧 광축(Z) 기준 회전이 된다. 주축 방향은 ±180°
    모호성이 있으나(긴 축은 양방향 동일) 그리퍼 접근각엔 영향 없다.
    반환: (qx, qy, qz, qw). 점이 부족하면 None.
    """
    if xs.size < 10:
        return None
    pts = np.column_stack([xs.astype(np.float64), ys.astype(np.float64)])
    pts -= pts.mean(axis=0)
    cov = np.cov(pts.T)
    evals, evecs = np.linalg.eigh(cov)         # 오름차순 고유값
    major = evecs[:, int(np.argmax(evals))]    # 최대 분산 방향 = 객체 긴 축
    theta = math.atan2(float(major[1]), float(major[0]))
    return (0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0))


# ──────────────────────────────────────────────
# 노드
# ──────────────────────────────────────────────

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # model_name 이 model_presets.MODEL_PRESETS 의 기본값을 고르고,
        # 이후 개별 파라미터는 여전히 CLI/launch로 override 가능(gripper_type과 동일 패턴).
        self.declare_parameter('model_name', DEFAULT_MODEL)  # 'box' | 'traffic_light' | ...
        self._model_name = self.get_parameter('model_name').value
        preset = get_preset(self._model_name, self.get_logger())

        self.declare_parameter('model_path', preset['model_path'])
        self.declare_parameter('task', preset['task'])    # 'segment' | 'detect'
        self.declare_parameter('backend', 'pt')          # 'pt' | 'trt'
        self.declare_parameter('conf_threshold', 0.55)
        self.declare_parameter('classes', preset['classes'])  # 쉼표구분 필터, 빈값=전체
        self.declare_parameter('width', 848)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('camera_mode', 'realsense')  # 'realsense' | 'test'
        self.declare_parameter('test_image_path', '')

        # depth 포인트클라우드 (카메라 TF 캘리브 시각화용, 기본 off)
        # RealSense는 프로세스 하나만 장치를 열 수 있어 realsense2_camera 드라이버를
        # 따로 띄울 수 없다 — 클라우드가 필요하면 이 노드가 직접 내야 한다.
        self.declare_parameter('publish_cloud', False)
        self.declare_parameter('cloud_rate_hz', 5.0)   # 추론과 대역폭을 뺏지 않게 낮게
        self.declare_parameter('cloud_decimation', 4)  # N픽셀마다 1점

        # 픽 대상 선별 (Phase 2 Step 3)
        self.declare_parameter('pick_classes', preset['pick_classes'])  # 쉼표구분 화이트리스트. 빈값=후보없음(관찰 전용 구간)
        self.declare_parameter('pick_min_conf', 0.5)    # 픽 후보 최소 confidence
        self.declare_parameter('require_depth', True)   # True=depth pose(z!=0) 필수, False=conf만(test용)
        self._warned_no_pick_classes = False

        model_path = self.get_parameter('model_path').value
        backend = self.get_parameter('backend').value
        self._w = self.get_parameter('width').value
        self._h = self.get_parameter('height').value
        self._fps = self.get_parameter('fps').value
        self._camera_mode = self.get_parameter('camera_mode').value

        # YOLO 모델 로드 (segmentation)
        # task를 명시적으로 넘긴다 — TensorRT .engine은 task 메타데이터를 보존하지 않아
        # ultralytics가 자동 추정 시 seg 모델도 'detect'로 오판하고 r0.masks가 조용히
        # None이 되는 문제가 있었다(2026-07-22 실측, model_presets.py 모듈 docstring 참고).
        task = self.get_parameter('task').value
        self.model, resolved, load_s = _load_yolo(model_path, backend, task,
                                                  self._h, self._w)
        self.get_logger().info(
            f"YOLO loaded: model_name={self._model_name} path={resolved} "
            f"task={self.model.task} ({load_s:.2f}s)")

        # ── 런타임 모델 교체 (실습/구간 전환용) ──
        # 파라미터 콜백은 spin 스레드에서 돌기 때문에 **여기서 로드하지 않는다**
        # (로드가 길면 파라미터 서비스가 그 동안 통째로 막힌다). 검증만 하고
        # 요청을 적어 두면, self.model 의 유일한 소비자인 추론 스레드가 프레임
        # 사이에서 교체한다 — in-flight predict() 와 경합할 수 없다.
        self._pending_lock = threading.Lock()
        self._pending_model = None
        self._applying_params = False    # set_parameters 재진입 가드
        self._pick_target_published = False
        self.pub_model_status = self.create_publisher(
            String, '/perception/model_status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._publish_model_status('loaded', name=self._model_name, path=resolved,
                                   task=str(self.model.task), seconds=round(load_s, 2))
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # 카메라 초기화 (realsense 모드면 DepthCal 생성)
        self._rs = None
        self._pipe = None
        self._depth_cal = None
        self._test_img = None
        if self._camera_mode == 'realsense':
            self._init_realsense()
        else:
            self._init_test_image()
            self.get_logger().warn(
                'camera_mode=test: depth 없음 → translation 0, orientation(PCA)만 채워짐')

        # 퍼블리셔
        self.pub_objects = self.create_publisher(
            DetectedObjectArray, '/detected_objects', 10)
        # /pick_target: transient_local(latched) — "도착→집어" 타이밍에 최신 타깃 유실 방지
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_pick = self.create_publisher(
            DetectedObject, '/pick_target', latched_qos)
        self.pub_debug = self.create_publisher(ImageMsg, '/perception/debug_image', 1)
        # YOLO/오버레이와 무관한 raw 스트림 — 캡처 스레드가 매 프레임 발행(§3.2)
        self.pub_raw = self.create_publisher(ImageMsg, '/perception/raw_image', 1)
        self.pub_cloud = self.create_publisher(PointCloud2, '/perception/depth_cloud', 1)
        self._last_cloud_t = 0.0
        self._bridge = CvBridge()

        # 캡처 스레드는 카메라를 전담하며 추론을 기다리지 않는다. 추론 스레드는
        # 공유 슬롯의 최신 프레임 1장만 소비하므로 지연 큐가 쌓이지 않는다.
        self._running = True
        self._latest_lock = threading.Lock()
        self._latest_frame = None
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._inference_thread = threading.Thread(target=self._inference_loop, daemon=True)
        self._capture_thread.start()
        self._inference_thread.start()

    # ── 카메라 초기화 ──────────────────────────

    def _init_realsense(self):
        try:
            import pyrealsense2 as rs
            self._rs = rs
            pipe = rs.pipeline()
            cfg = rs.config()
            cfg.enable_device(D435_SERIAL)
            cfg.enable_stream(rs.stream.depth, self._w, self._h, rs.format.z16, self._fps)
            cfg.enable_stream(rs.stream.color, self._w, self._h, rs.format.bgr8, self._fps)
            profile = pipe.start(cfg)
            self._pipe = pipe
            # color↔depth 투영 파라미터 (RealSense 내부파라미터 직접 사용)
            self._depth_cal = DepthCal(rs, profile)
            self.get_logger().info(
                f'RealSense started ({self._w}x{self._h} @ {self._fps}fps), depth pose 활성')
        except Exception as e:
            self.get_logger().error(f'RealSense init failed: {e}')
            raise

    def _init_test_image(self):
        test_path = self.get_parameter('test_image_path').value
        if test_path and os.path.exists(test_path):
            img = cv2.imread(test_path)
            if img is not None:
                self._test_img = cv2.resize(img, (self._w, self._h))
                self.get_logger().info(f'Test image loaded: {test_path}')
                return
        self._test_img = np.zeros((self._h, self._w, 3), dtype=np.uint8)
        self.get_logger().warn('camera_mode=test: 빈 프레임 사용 (test_image_path 미지정)')

    # ── 런타임 모델 교체 ───────────────────────

    def _on_set_parameters(self, params):
        """모델 관련 파라미터 변경 요청을 **검증만** 하고 예약한다.

        지금까지는 모델이 생성자에서 한 번만 로드돼 구간이 바뀔 때마다 프로세스를
        재시작해야 했다. 실습에서 `best.pt` 를 바꿔가며 결과를 비교하려면 그 재시작이
        그대로 시연의 병목이 된다.

        ⚠️ 여기서 `YOLO()` 를 부르면 안 된다 — 이 콜백은 rclpy spin 스레드에서 돌고,
        로드가 걸리는 동안 이 노드의 파라미터 서비스 전체가 응답을 멈춘다.
        """
        if self._applying_params:
            return SetParametersResult(successful=True)

        watched = {p.name: p.value for p in params
                   if p.name in ('model_name', 'model_path', 'task', 'backend')}
        if not watched:
            return SetParametersResult(successful=True)

        name = watched.get('model_name', self._model_name)
        derived = 'model_name' in watched and 'model_path' not in watched
        if derived and name not in MODEL_PRESETS:
            # 경로를 preset 에서 뽑아야 하는데 그 preset 이 없다. get_preset 은 이걸
            # 조용히 box 로 폴백하는데, 화면에서 고른 값이 말없이 다른 모델로 바뀌면
            # "왜 결과가 그대로지?"가 된다 — 여기서는 거절한다.
            # (model_path 를 같이 주면 name 은 표시용 라벨일 뿐이라 자유롭게 쓴다 —
            #  프리셋에 없는 `best.pt` 를 실습 중에 얹는 경로가 그렇다.)
            return SetParametersResult(
                successful=False,
                reason=f'모르는 model_name: {name} (가능: {", ".join(sorted(MODEL_PRESETS))})')

        preset = MODEL_PRESETS.get(name, {})
        path = watched.get('model_path')
        if path is None:
            path = preset.get('model_path') if derived \
                else self.get_parameter('model_path').value
        task = watched.get('task')
        if task is None:
            task = preset.get('task') if derived else self.get_parameter('task').value
        backend = watched.get('backend', self.get_parameter('backend').value)

        if task not in ('segment', 'detect'):
            return SetParametersResult(
                successful=False, reason=f'task 는 segment|detect 만 됩니다: {task!r}')
        if backend not in ('pt', 'trt'):
            return SetParametersResult(
                successful=False, reason=f'backend 는 pt|trt 만 됩니다: {backend!r}')
        if not path or not os.path.exists(path):
            # 지금까지 이 검증이 없어서 잘못된 경로 하나로 **프로세스가 죽었다**
            # (vision_test_node 는 이미 존재 확인을 한다 — 그 패턴을 가져왔다).
            return SetParametersResult(
                successful=False,
                reason=(f'모델 파일이 없습니다: {path!r} '
                        f'(상대경로면 노드의 CWD 기준 — 워크스페이스 루트에서 띄웠는지 확인)'))

        request = {'name': name, 'path': path, 'task': task, 'backend': backend}
        if derived:
            # model_name 만 바꿨으면 클래스 필터도 그 preset 것으로 따라가야 한다.
            # 안 그러면 _get_cls_filter 가 프레임마다 'Unknown class' 경고를 뿜고
            # 필터가 조용히 "전체 통과"로 열화된다.
            request['classes'] = preset.get('classes', '')
            request['pick_classes'] = preset.get('pick_classes', '')
        with self._pending_lock:
            self._pending_model = request
        self._publish_model_status('loading', name=name, path=path, task=task)
        return SetParametersResult(successful=True)

    def _apply_pending_model(self):
        """추론 스레드 전용 — 프레임과 프레임 사이에서만 교체한다."""
        with self._pending_lock:
            request = self._pending_model
            self._pending_model = None
        if request is None:
            return

        try:
            model, resolved, load_s = _load_yolo(
                request['path'], request['backend'], request['task'],
                self._h, self._w)
        except Exception as exc:                              # noqa: BLE001
            self.get_logger().error(f'모델 교체 실패: {exc}')
            self._publish_model_status('error', name=request['name'],
                                       path=request['path'], task=request['task'],
                                       detail=f'{type(exc).__name__}: {exc}')
            return

        self.model = model
        self._model_name = request['name']
        self._warned_no_pick_classes = False
        if 'classes' in request:
            self._mirror_params({'classes': request['classes'],
                                 'pick_classes': request['pick_classes']})
        self.get_logger().info(
            f'모델 교체 완료: {request["name"]} path={resolved} '
            f'task={model.task} ({load_s:.2f}s)')
        self._publish_model_status('loaded', name=request['name'], path=resolved,
                                   task=str(model.task), seconds=round(load_s, 2))

    def _mirror_params(self, values):
        """preset 에서 파생된 값을 파라미터에도 반영(재진입 가드 필수)."""
        from rclpy.parameter import Parameter
        self._applying_params = True
        try:
            self.set_parameters([Parameter(k, Parameter.Type.STRING, v)
                                 for k, v in values.items()])
        finally:
            self._applying_params = False

    def _publish_model_status(self, state, *, name, path, task,
                              seconds=None, detail=''):
        """`/perception/model_status` (JSON, latched).

        `stale_pick_target` 이 핵심이다 — `/pick_target` 은 latched 라 새 모델이
        아직 아무것도 못 찾았어도 **이전 모델의 마지막 타깃이 그대로 남아 있다.**
        지울 방법이 없으므로 화면이 경고할 수 있게 사실을 실어 보낸다.
        """
        import json
        payload = {
            'state': state,
            'name': name,
            'path': path,
            'task': task,
            'seconds': seconds,
            'detail': detail,
            'stale_pick_target': self._pick_target_published,
        }
        self.pub_model_status.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    # ── 클래스 필터 ────────────────────────────

    def _get_cls_filter(self, model):
        classes_str = self.get_parameter('classes').value
        if not classes_str.strip():
            return None
        name_to_id = {v: k for k, v in model.names.items()}
        ids = []
        for name in classes_str.split(','):
            name = name.strip()
            if name in name_to_id:
                ids.append(name_to_id[name])
            else:
                self.get_logger().warn(f'Unknown class "{name}" — 무시',
                                       throttle_duration_sec=5.0)
        return ids or None

    # ── capture / latest-only 추론 ──────────────────────────────

    def _capture_loop(self):
        """카메라 전담 — 프레임을 받는 즉시 raw_image 발행, 최신 프레임만 공유 슬롯에 남김."""
        sequence = 0
        interval = 1.0 / self._fps
        frame_id = self.get_parameter('frame_id').value

        while self._running and rclpy.ok():
            t0 = time.time()
            color_img, depth_frame, depth_img = self._grab_frame()
            if color_img is None:
                time.sleep(0.01)
                continue

            stamp = self.get_clock().now().to_msg()
            raw = self._bridge.cv2_to_imgmsg(color_img, encoding='bgr8')
            raw.header.stamp = stamp
            raw.header.frame_id = frame_id
            self.pub_raw.publish(raw)

            if depth_img is not None:
                self._maybe_publish_cloud(depth_img, stamp, frame_id)

            # RealSense SDK 프레임 객체는 스레드 간 공유가 안전하지 않음 — keep()으로
            # 버퍼를 유지한 채 참조만 공유 슬롯에 넘긴다(색상/깊이 배열은 복사).
            if depth_frame is not None:
                depth_frame.keep()
            with self._latest_lock:
                sequence += 1
                self._latest_frame = (sequence, stamp, color_img.copy(), depth_frame,
                                      None if depth_img is None else depth_img.copy())

            if self._camera_mode == 'test':
                elapsed = time.time() - t0
                if interval - elapsed > 0:
                    time.sleep(interval - elapsed)

    def _maybe_publish_cloud(self, depth_img, stamp, frame_id):
        """depth → PointCloud2 (카메라 TF 캘리브 시각화용, 2026-08-07 추가).

        RViz에서 이 점구름이 URDF 로봇 모델·책상면과 겹치도록 카메라 TF를 맞추면,
        박스 다점 측정 없이도 roll/pitch/높이를 눈으로 잡을 수 있다 —
        `camera_tf_tuner`와 짝으로 쓴다. 로봇 형상을 따로 YOLO로 학습할 필요가
        없는 이유이기도 하다(형상은 이미 depth 안에 있다).

        기본 off + 구독자 게이트 + 저속(5Hz) — 추론 스레드의 대역폭을 뺏지 않는다.
        """
        if not self.get_parameter('publish_cloud').value:
            return
        if self.pub_cloud.get_subscription_count() == 0:
            return
        cal = self._depth_cal
        if cal is None:
            return
        rate = float(self.get_parameter('cloud_rate_hz').value)
        now = time.time()
        if rate <= 0.0 or now - self._last_cloud_t < 1.0 / rate:
            return
        self._last_cloud_t = now

        step = max(int(self.get_parameter('cloud_decimation').value), 1)
        z = depth_img[::step, ::step].astype(np.float32) * cal.scale
        rows, cols = np.nonzero((z > cal.dmin) & (z < cal.dmax))
        if rows.size == 0:
            return
        zz = z[rows, cols]
        # 픽셀 → depth optical 3D. depth 스트림은 왜곡계수가 0이라 핀홀 식이
        # rs2_deproject_pixel_to_point 와 같은 결과를 주고, 벡터화되어 수만 점도 1ms 안쪽이다.
        xx = (cols * step - cal.di.ppx) / cal.di.fx * zz
        yy = (rows * step - cal.di.ppy) / cal.di.fy * zz
        points = np.stack((xx, yy, zz), axis=1)

        # depth optical → color optical. 헤더 frame_id 가 color optical 기준이므로 필수다
        # (D435 는 두 센서가 물리적으로 ~15mm 떨어져 있어 생략하면 그만큼 통째로 밀린다).
        matrix = np.array(cal.d2c.rotation, dtype=np.float32).reshape(3, 3).T  # rs는 열 우선
        translation = np.array(cal.d2c.translation, dtype=np.float32)
        points = points @ matrix.T + translation

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = int(points.shape[0])
        msg.fields = [
            PointField(name=name, offset=4 * i,
                       datatype=PointField.FLOAT32, count=1)
            for i, name in enumerate(('x', 'y', 'z'))
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
        self.pub_cloud.publish(msg)

    def _inference_loop(self):
        """공유 슬롯의 최신 프레임만 소비 — 지연 프레임 큐가 쌓이지 않는다.

        모델 교체도 여기서 한다(프레임과 프레임 사이). 이 루프가 `self.model` 의
        유일한 소비자라, 교체가 진행 중인 `predict()` 와 경합할 수 없다.
        """
        last_sequence = -1
        while self._running and rclpy.ok():
            self._apply_pending_model()
            with self._latest_lock:
                frame = self._latest_frame
            if frame is None or frame[0] == last_sequence:
                time.sleep(0.002)
                continue
            last_sequence, stamp, color_img, depth_frame, depth_img = frame
            try:
                self._publish_detections(color_img, depth_frame, depth_img, stamp)
            except Exception as exc:                          # noqa: BLE001
                # 이 스레드가 예외로 죽으면 프로세스는 멀쩡한데 검출만 영원히
                # 멈춘다(가장 진단하기 나쁜 실패). 잡아서 로그로 남기고 계속 돈다.
                self.get_logger().error(f'추론 루프 예외: {type(exc).__name__}: {exc}',
                                        throttle_duration_sec=2.0)
                time.sleep(0.05)

    def _publish_detections(self, color_img, depth_frame, depth_img, stamp):
        # ⚠️ 모델 참조를 **한 번만** 스냅샷한다. self.model 을 두 번 따로 읽으면
        # (predict 할 때 / names 를 볼 때) 그 사이에 교체가 끼어들어 A 모델의
        # class_id 로 B 모델의 names 를 인덱싱하고 KeyError 가 난다.
        model = self.model
        conf = self.get_parameter('conf_threshold').value
        cls_filter = self._get_cls_filter(model)
        frame_id = self.get_parameter('frame_id').value

        # ① YOLO segmentation 추론
        results = model.predict(
            color_img, conf=conf, classes=cls_filter, verbose=False)
        r0 = results[0]
        # ``masks.data``는 모델/letterbox 래스터 좌표라 848x480 리사이즈 시
        # 패딩이 있으면 depth 샘플이 어긋난다. ``masks.xy``는 이미 원본
        # color-frame 좌표로 de-letterbox된 폴리곤이라 이쪽을 사용한다
        # (2026-07-28 D435 근접 타겟 오측정 수정).
        mask_polygons = None if r0.masks is None else r0.masks.xy

        array_msg = DetectedObjectArray()
        array_msg.header.stamp = stamp
        array_msg.header.frame_id = frame_id

        for i, box in enumerate(r0.boxes):
            x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
            if not is_usable_bbox(
                (x1, y1, x2, y2),
                frame_width=self._w,
                frame_height=self._h,
            ):
                continue

            obj = DetectedObject()
            obj.class_id = int(box.cls[0])
            obj.class_name = model.names[obj.class_id]
            obj.confidence = float(box.conf[0])
            obj.pose = Pose()  # 기본값(position 0)
            obj.pose.orientation.w = 1.0  # 유효 단위 쿼터니언

            roi = RegionOfInterest()
            roi.x_offset = x1
            roi.y_offset = y1
            roi.width = x2 - x1
            roi.height = y2 - y1
            obj.bbox = roi

            # ② markerless pose: 마스크 기반 translation(depth) + orientation(2D PCA)
            binmask = self._get_binmask(
                mask_polygons, i, color_img, (x1, y1, x2, y2),
            )
            self._fill_markerless_pose(
                obj, binmask, (x1, y1, x2, y2), depth_frame, depth_img)

            array_msg.objects.append(obj)

        self.pub_objects.publish(array_msg)

        # ③ 픽 대상 선별 → /pick_target (후보 없으면 publish 안 함 = latched 유지)
        pick = self._select_pick_target(array_msg.objects)
        if pick is not None:
            self.pub_pick.publish(pick)
            # latched 라 모델을 바꿔도 이 값이 남는다 — model_status 가 그 사실을
            # 실어 보내 화면이 "이전 모델의 타깃" 경고를 띄울 수 있게 한다.
            self._pick_target_published = True

        # ④ debug 이미지 발행
        if self.pub_debug.get_subscription_count() > 0:
            debug_img = self._draw_debug(
                color_img.copy(), array_msg.objects, mask_polygons, pick,
            )
            self.pub_debug.publish(
                self._bridge.cv2_to_imgmsg(debug_img, encoding='bgr8'))

    def _select_pick_target(self, objects):
        """픽 후보 중 confidence 최고 1개 선택.

        후보 조건: class_name ∈ pick_classes(화이트리스트) AND confidence ≥ pick_min_conf
        AND (require_depth=False 또는 depth pose 채워짐 z!=0). 후보 없으면 None.
        """
        pick_classes = self.get_parameter('pick_classes').value
        whitelist = {c.strip() for c in pick_classes.split(',') if c.strip()}
        if not whitelist:
            if not self._warned_no_pick_classes:
                self.get_logger().warn(
                    'pick_classes 미설정 → /pick_target 후보 없음. 집을 클래스를 지정하라.')
                self._warned_no_pick_classes = True
            return None

        min_conf = self.get_parameter('pick_min_conf').value
        require_depth = self.get_parameter('require_depth').value

        best = None
        for obj in objects:
            if obj.class_name not in whitelist:
                continue
            if obj.confidence < min_conf:
                continue
            if require_depth and obj.pose.position.z == 0.0:
                continue
            if best is None or obj.confidence > best.confidence:
                best = obj
        return best

    def _get_binmask(self, mask_polygons, i, color_img=None, box=None):
        """i번째 객체의 이진 마스크(원본 해상도).

        seg 모델이면 YOLO 마스크(``masks.xy``, 이미 원본 좌표로 de-letterbox된
        폴리곤)를 fillPoly로 래스터화해 사용. detect 전용 모델(마스크 없음)이면
        bbox 안 red/blue HSV 색상 마스크로 대체(대회 타겟 클래스 'red and blue box'
        전용 근사) — PCA 주축 계산엔 정확한 세그멘테이션과 동일하게 넘길 수 있다.
        """
        if mask_polygons is not None and i < len(mask_polygons):
            polygon = np.asarray(mask_polygons[i], dtype=np.float32)
            if polygon.ndim == 2 and polygon.shape[0] >= 3:
                polygon[:, 0] = np.clip(polygon[:, 0], 0, self._w - 1)
                polygon[:, 1] = np.clip(polygon[:, 1], 0, self._h - 1)
                original_mask = np.zeros((self._h, self._w), dtype=np.uint8)
                cv2.fillPoly(
                    original_mask,
                    [np.rint(polygon).astype(np.int32)],
                    1,
                )
                return original_mask.astype(bool)
        if color_img is not None and box is not None:
            return self._color_mask_in_box(color_img, box)
        return None

    def _color_mask_in_box(self, color_img, box):
        """bbox 내부에서 red/blue HSV 마스크 추출 → 전체 프레임 크기 bool 배열로 반환.

        빨강은 HSV hue가 0 근처와 180 근처 양쪽에 걸쳐 있어 두 구간을 합친다.
        """
        x1, y1, x2, y2 = box
        x1, y1 = max(x1, 0), max(y1, 0)
        x2, y2 = min(x2, self._w), min(y2, self._h)
        full = np.zeros((self._h, self._w), dtype=bool)
        if x2 <= x1 or y2 <= y1:
            return full
        hsv = cv2.cvtColor(color_img[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
        red = (cv2.inRange(hsv, (0, 70, 50), (10, 255, 255))
               | cv2.inRange(hsv, (170, 70, 50), (180, 255, 255)))
        blue = cv2.inRange(hsv, (100, 70, 50), (130, 255, 255))
        mask = cv2.morphologyEx(red | blue, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        full[y1:y2, x1:x2] = mask > 0
        return full

    def _fill_markerless_pose(self, obj, binmask, box, depth_frame, depth_img):
        """translation = 마스크 centroid depth median deproject, orientation = 2D PCA yaw."""
        x1, y1, x2, y2 = box

        # 마스크 픽셀 좌표 (없으면 bbox 중심으로 폴백)
        if binmask is not None and binmask.any():
            ys, xs = np.nonzero(binmask)
            cu, cv_ = float(xs.mean()), float(ys.mean())
            quat = _mask_pca_yaw_quat(xs, ys)
            if quat is not None:
                qx, qy, qz, qw = quat
                obj.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        else:
            cu, cv_ = (x1 + x2) / 2.0, (y1 + y2) / 2.0

        # translation: realsense 모드에서만 (depth 필요)
        if depth_frame is None or depth_img is None or self._depth_cal is None:
            return
        xyz = _deproject_mask(
            self._rs, depth_frame, depth_img, binmask, cu, cv_,
            self._depth_cal,
        )
        if xyz is not None:
            obj.pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))

    def _draw_debug(self, img, objects, mask_polygons, pick):
        """감지 결과를 img에 그려 반환. pick 타겟은 초록, 나머지는 파란색."""
        overlay = img.copy()
        for i, obj in enumerate(objects):
            is_pick = (pick is not None and obj is pick)
            color = (0, 255, 0) if is_pick else (255, 100, 0)

            # 바운딩박스
            x1 = obj.bbox.x_offset
            y1 = obj.bbox.y_offset
            x2 = x1 + obj.bbox.width
            y2 = y1 + obj.bbox.height

            # 마스크 반투명 오버레이
            binmask = self._get_binmask(
                mask_polygons, i, img, (x1, y1, x2, y2),
            )
            if binmask is not None:
                overlay[binmask] = (
                    overlay[binmask] * 0.5 + np.array(color, dtype=np.float32) * 0.5
                ).astype(np.uint8)

            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)

            # 텍스트: 클래스명 + confidence + 거리 + yaw(PCA)
            z = obj.pose.position.z
            depth_str = f' {z:.2f}m' if z != 0.0 else ''
            q = obj.pose.orientation
            has_yaw = not (q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 1.0)
            yaw_str = ''
            if has_yaw:
                yaw_rad = 2.0 * math.atan2(q.z, q.w)
                yaw_deg = math.degrees(yaw_rad)
                yaw_str = f' yaw={yaw_deg:.0f}deg'
            label = f'{obj.class_name} {obj.confidence:.2f}{depth_str}{yaw_str}'
            cv2.putText(overlay, label, (x1, max(y1 - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)

            # PCA 주축 방향 시각화 (centroid에서 yaw 방향으로 선 긋기, ±180° 모호성 있어 양방향 표시)
            if has_yaw and binmask is not None and binmask.any():
                ys_m, xs_m = np.nonzero(binmask)
                cu, cvv = float(xs_m.mean()), float(ys_m.mean())
                length = 0.6 * max(x2 - x1, y2 - y1)
                dx = length * math.cos(yaw_rad)
                dy = length * math.sin(yaw_rad)
                p1 = (int(cu - dx), int(cvv - dy))
                p2 = (int(cu + dx), int(cvv + dy))
                cv2.line(overlay, p1, p2, (0, 0, 255), 2, cv2.LINE_AA)
                cv2.circle(overlay, (int(cu), int(cvv)), 3, (0, 0, 255), -1)
        return overlay

    def _grab_frame(self):
        """(color_img, depth_frame, depth_img) 반환. test 모드는 depth None."""
        if self._camera_mode == 'realsense':
            try:
                frames = _latest_frames(self._pipe)
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame:
                    return None, None, None
                color_img = np.asanyarray(color_frame.get_data())
                depth_img = (np.asanyarray(depth_frame.get_data())
                             if depth_frame else None)
                return color_img, depth_frame, depth_img
            except Exception as e:
                self.get_logger().error(f'Frame grab error: {e}')
                return None, None, None
        else:
            return self._test_img.copy(), None, None

    # ── 정리 ───────────────────────────────────

    def destroy_node(self):
        self._running = False
        self._capture_thread.join(timeout=2.0)
        self._inference_thread.join(timeout=2.0)
        if self._pipe is not None:
            self._pipe.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
