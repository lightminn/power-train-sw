# 운용 콘솔 PiP·거리 정본·라이브 결함 구현 계획

> **For agentic workers:** 이 계획은 태스크 단위로 실행한다. 각 태스크는
> 실패하는 테스트 → 최소 구현 → 통과 확인 → 커밋 순서다. 체크박스로 진행을 추적한다.

**Goal:** 작업 카메라 PiP 가 스트림 해상도와 무관하게 고정 크기를 유지하고,
거리 표시가 D435i SDK depth 를 정본으로 삼으며, 라이브에서 관측된 결함이
제거된 운용 콘솔.

**Architecture:** 순수 파이썬 뷰 모델(`metadata.py`·`status_view.py`)의 판정
로직을 먼저 고치고, GTK 위젯 계층(`app.py`)에서 레이아웃·입력 정책을 바꾼다.
GStreamer/네트워크 계약(SRT·UDP·ops TCP)은 건드리지 않는다. 젯슨측 결함(C1)은
콘솔 커밋과 분리한다.

**Tech Stack:** Python 3.14(시스템 `/usr/bin/python3`), PyGObject/GTK 3,
GStreamer, pytest.

**Spec:** `docs/superpowers/specs/2026-07-29-operator-console-pip-distance-live-design.md`

## Global Constraints

- 작업 브랜치: `pr3`(= GitHub PR #3 `agent/operator-console-live-telemetry`,
  베이스 커밋 `00126f0`). 워크트리: `/home/light/ZETIN/robotics/power-train-sw-pr3`.
- 테스트 실행 인터프리터는 **`/usr/bin/python3`** 하나뿐이다(conda 파이썬에는
  `gi` 가 없다). 명령: `/usr/bin/python3 -m pytest operator_console/tests -q`.
- 기준선(수정 전): `operator_console/tests` **215 passed**,
  `/usr/bin/python3 -m operator_console.runtime_smoke` **PASS**.
- 수신 계약 불변: SRT :5000/:5002, UDP :5003/:5004/:5005/:5007, ops TCP :9001,
  metadata `schema_version: 1`. 필드 **추가**는 선택적(optional)으로만 한다.
- 콘솔은 수신 전용이다. 어떤 태스크도 센서·모터·CAN·ROS 노드를 기동하거나
  중지시키는 코드를 추가하지 않는다.
- 결측·정체 데이터를 정상값(0 포함)으로 렌더하지 않는다.
- 한국어 UI 문구는 기존 어휘를 따른다: `정상`, `확인 필요`, `정보 없음`,
  `연결 중`, `대상 탐지 대기`, `거리 정보 없음`, `거리 정보 지연`.
- 새 검증 게이트를 만들면 **음성 대조**(결함 재주입 → FAIL 확인)까지 수행한다.
- 매 태스크 끝에서 전체 스위트 + runtime smoke 를 돌리고 커밋한다.

---

### Task 1: 거리 정본을 SDK depth 로 전환 (B0)

**Files:**
- Modify: `operator_console/metadata.py:79-85` (`target_distance_m`),
  `operator_console/metadata.py:254-329` (`parse_metadata`),
  `operator_console/metadata.py:21-29` (`Detection`)
- Test: `operator_console/tests/test_app.py` (기존 거리 테스트가 여기 있다)

**Interfaces:**
- Consumes: `Detection.position_m: tuple[float,float,float] | None`
- Produces: `target_distance_m(detection) -> float | None` 이 **depth(Z)** 를
  반환한다. `Detection.depth_m: float | None = None` 필드가 추가되고,
  값이 있으면 그 값이 우선한다.

- [ ] **Step 1: 실패하는 테스트를 쓴다**

```python
def test_distance_uses_sdk_depth_not_3d_range():
    """실기 대조값: position (0.0854,0.0704,0.2882) 의 SDK depth 는 0.2882 m,
    3D magnitude 는 0.3199 m. 콘솔 정본은 depth 다."""
    detection = Detection(
        "box-segmentation", 0.96, (503, 293, 219, 187),
        (0.0854, 0.0704, 0.2882),
    )
    assert target_distance_m(detection) == pytest.approx(0.2882, abs=1e-4)


def test_explicit_depth_field_overrides_position_z():
    detection = Detection(
        "box-segmentation", 0.96, (0, 0, 10, 10), (0.1, 0.1, 0.30),
        depth_m=0.2884,
    )
    assert target_distance_m(detection) == pytest.approx(0.2884, abs=1e-4)


def test_parse_metadata_reads_optional_depth_m():
    frame = parse_metadata(
        b'{"schema_version":1,"capture_sequence":1,"frame_width":848,'
        b'"frame_height":480,"detections":[{"class_name":"box-segmentation",'
        b'"confidence":0.9,"bbox_xywh":[1,2,3,4],'
        b'"position_m":[0.1,0.1,0.3],"depth_m":0.288}]}'
    )
    assert frame.detections[0].depth_m == pytest.approx(0.288)


def test_non_positive_depth_is_unavailable():
    assert target_distance_m(
        Detection("x", 0.9, (0, 0, 1, 1), (0.1, 0.1, 0.0))
    ) is None
```

`Detection` 을 `test_app.py` 의 import 목록에 추가한다.

- [ ] **Step 2: 실패를 확인한다**

Run: `/usr/bin/python3 -m pytest operator_console/tests/test_app.py -q -k "depth or distance"`
Expected: FAIL — `target_distance_m` 이 0.3087 을 돌려주고, `Detection` 에
`depth_m` 인자가 없다.

- [ ] **Step 3: 최소 구현**

`Detection` 에 필드를 추가한다(기존 위치 인자 순서를 깨지 않도록 **맨 뒤**에).

```python
@dataclass(frozen=True)
class Detection:
    class_name: str
    confidence: float
    bbox_xywh: tuple[int, int, int, int]
    position_m: tuple[float, float, float] | None
    yaw_rad: float | None = None
    is_pick_target: bool = False
    depth_m: float | None = None
```

`target_distance_m` 을 규약이 드러나는 형태로 바꾼다.

```python
def target_distance_m(detection: Detection | None) -> float | None:
    """대상까지의 **D435i SDK depth**(광축 Z, m).

    3D 직선거리(√(x²+y²+z²))가 아니다.  magnitude 는 대상이 광축에서 벗어날수록
    depth/cos θ 로 커져 SDK 값과 어긋난다 — 2026-07-29 실기 대조에서 29 cm 대상
    기준 +3.16 cm(약 11%) 차이가 확인되어 정본을 depth 로 고정했다.  송신부가
    명시적 `depth_m` 을 실어 주면 그 값이 우선한다.
    """
    if detection is None:
        return None
    if detection.depth_m is not None:
        depth_m = detection.depth_m
    elif detection.position_m is not None:
        depth_m = detection.position_m[2]
    else:
        return None
    return depth_m if math.isfinite(depth_m) and depth_m > 0.0 else None
```

`parse_metadata` 의 검출 루프에서 `depth_m` 을 선택적으로 읽는다
(`position_m` 파싱 직후).

```python
        raw_depth = item.get("depth_m")
        try:
            depth_m = None if raw_depth is None else float(raw_depth)
        except (TypeError, OverflowError) as exc:
            raise ValueError("invalid depth") from exc
        if depth_m is not None and (
            not math.isfinite(depth_m) or depth_m <= 0.0
        ):
            depth_m = None
```

`Detection(...)` 생성에 `depth_m=depth_m` 을 넘긴다.

- [ ] **Step 4: 통과를 확인한다**

Run: `/usr/bin/python3 -m pytest operator_console/tests -q`
Expected: 전건 PASS. 기존 테스트 중 3D magnitude 를 기대하던 것이 있으면
**테스트가 아니라 기대값을 정본(depth)으로 고친다** — 단, 그 테스트가 검증하던
다른 성질(None 처리 등)은 유지한다.

- [ ] **Step 5: 음성 대조**

`target_distance_m` 을 잠시 magnitude 로 되돌려
`test_distance_uses_sdk_depth_not_3d_range` 가 FAIL 하는지 확인하고 되돌린다.

- [ ] **Step 6: 커밋**

```bash
git add operator_console/metadata.py operator_console/tests/test_app.py
git commit -m "fix(console): distance authority is D435i SDK depth, not 3D range"
```

---

### Task 2: 비-타깃 폴백 제거 (B1)

**Files:**
- Modify: `operator_console/metadata.py:176-182` (`DisplayTargetTracker.update`)
- Test: `operator_console/tests/test_app.py`

**Interfaces:**
- Produces: pick target 이 없고 IoU 연속 추적도 끊긴 프레임에서
  `DisplayTargetView.detection is None`, `distance_m is None`,
  `distance_state == "대상 탐지 대기"`.

- [ ] **Step 1: 실패하는 테스트**

```python
def _frame(detections, sequence=1, now_s=100.0):
    return MetadataFrame(
        sequence=sequence, width=848, height=480,
        detections=tuple(detections), received_monotonic_s=now_s,
        frame_id="camera_color_optical_frame",
    )


def test_no_pick_target_means_no_distance_number():
    """송신자가 지정하지 않은 물체를 '대상'으로 승격하지 않는다."""
    tracker = DisplayTargetTracker()
    high_conf = Detection(
        "box-segmentation", 0.96, (503, 293, 219, 187),
        (0.09, 0.07, 0.288), is_pick_target=False,
    )
    view = tracker.update(_frame([high_conf]), now_s=100.0)
    assert view.detection is None
    assert view.distance_m is None
    assert view.distance_state == "대상 탐지 대기"


def test_designated_pick_target_still_reports_depth():
    tracker = DisplayTargetTracker()
    target = Detection(
        "box-segmentation", 0.96, (503, 293, 219, 187),
        (0.09, 0.07, 0.288), is_pick_target=True,
    )
    view = tracker.update(_frame([target]), now_s=100.0)
    assert view.distance_m == pytest.approx(0.288, abs=1e-3)


def test_iou_continuity_survives_a_frame_without_the_flag():
    """한 번 지정된 대상은 다음 프레임에서 플래그가 빠져도 같은 상자면 유지된다."""
    tracker = DisplayTargetTracker()
    first = Detection(
        "box-segmentation", 0.96, (500, 290, 220, 190),
        (0.09, 0.07, 0.288), is_pick_target=True,
    )
    tracker.update(_frame([first], sequence=1), now_s=100.0)
    drifted = Detection(
        "box-segmentation", 0.95, (503, 293, 219, 187),
        (0.09, 0.07, 0.290), is_pick_target=False,
    )
    view = tracker.update(_frame([drifted], sequence=2, now_s=100.1), now_s=100.1)
    assert view.detection is not None
    assert view.distance_m == pytest.approx(0.290, abs=2e-2)
```

`MetadataFrame` 을 import 목록에 추가한다.

- [ ] **Step 2: 실패 확인**

Run: `/usr/bin/python3 -m pytest operator_console/tests/test_app.py -q -k "pick_target or continuity"`
Expected: `test_no_pick_target_means_no_distance_number` FAIL (폴백이 대상을 만든다).

- [ ] **Step 3: 최소 구현**

`update()` 의 선택 로직에서 최고-confidence 폴백을 제거하고, 대상이 없을 때의
상태 문구를 맞춘다.

```python
        # 표시 대상은 송신자가 지정한 pick target 과 그 IoU 연속 추적분뿐이다.
        # 임의의 고신뢰 검출을 '대상'으로 승격하지 않는다(콘솔이 대상을 만들지
        # 않는다는 pick_display_target 의 정책과 일치).
        selected = explicit or continuous
        if selected is None:
            if (
                self._cached.detection is not None
                and self._last_valid_received_s is not None
                and (now_s - self._last_valid_received_s) * 1000.0
                <= self.config.dropout_hold_ms
            ):
                self._cached = DisplayTargetView(
                    self._cached.detection,
                    self._cached.distance_m,
                    self._cached.distance_state,
                    held=True,
                )
                return self._cached
            self._reset_filter()
            self._cached = DisplayTargetView(None, None, "대상 탐지 대기")
            return self._cached
```

`_reset_filter` 는 Task 4 에서 도입한다. 이 태스크에서는 아래 최소 형태로
먼저 추가한다.

```python
    def _reset_filter(self) -> None:
        self._distances.clear()
        self._ema = None
        self._last_bbox = None
        self._last_class = None
```

- [ ] **Step 4: 통과 확인**

Run: `/usr/bin/python3 -m pytest operator_console/tests -q`
Expected: 전건 PASS. 폴백을 전제로 쓰인 기존 테스트가 있으면 **spec B1 에 맞춰
기대값을 고친다**(폴백 복원 금지).

- [ ] **Step 5: 커밋**

```bash
git add operator_console/metadata.py operator_console/tests/test_app.py
git commit -m "fix(console): show distance only for the sender-designated pick target"
```

---

### Task 3: frame_id 화이트리스트 (B2)

**Files:**
- Modify: `operator_console/metadata.py` (모듈 상수 + `DisplayTargetTracker.update`)
- Test: `operator_console/tests/test_app.py`

**Interfaces:**
- Produces: 상수 `SUPPORTED_DISTANCE_FRAME_IDS = ("camera_color_optical_frame",)`,
  불일치 프레임에 대해 `distance_state == "거리 기준 불일치"`, `distance_m is None`.

- [ ] **Step 1: 실패하는 테스트**

```python
def test_unknown_frame_id_refuses_to_report_distance():
    """좌표계가 바뀌면 depth 의 의미가 달라진다 — 숫자를 내지 않는다."""
    tracker = DisplayTargetTracker()
    target = Detection(
        "box-segmentation", 0.96, (503, 293, 219, 187),
        (0.09, 0.07, 0.288), is_pick_target=True,
    )
    frame = MetadataFrame(
        sequence=1, width=848, height=480, detections=(target,),
        received_monotonic_s=100.0, frame_id="base_link",
    )
    view = tracker.update(frame, now_s=100.0)
    assert view.distance_m is None
    assert view.distance_state == "거리 기준 불일치"


def test_missing_frame_id_is_accepted_for_backward_compatibility():
    tracker = DisplayTargetTracker()
    target = Detection(
        "box-segmentation", 0.96, (503, 293, 219, 187),
        (0.09, 0.07, 0.288), is_pick_target=True,
    )
    frame = MetadataFrame(
        sequence=1, width=848, height=480, detections=(target,),
        received_monotonic_s=100.0, frame_id=None,
    )
    assert tracker.update(frame, now_s=100.0).distance_m == pytest.approx(0.288)
```

- [ ] **Step 2: 실패 확인**

Run: `/usr/bin/python3 -m pytest operator_console/tests/test_app.py -q -k frame_id`
Expected: FAIL — 현재는 frame_id 를 보지 않아 0.288 을 낸다.

- [ ] **Step 3: 최소 구현**

```python
SUPPORTED_DISTANCE_FRAME_IDS = ("camera_color_optical_frame",)
```

`update()` 에서 프레임 신선도 검사 직후에 넣는다.

```python
        if (
            frame.frame_id is not None
            and frame.frame_id not in SUPPORTED_DISTANCE_FRAME_IDS
        ):
            self._reset_filter()
            self._cached = DisplayTargetView(None, None, "거리 기준 불일치")
            return self._cached
```

- [ ] **Step 4: 통과 확인 + 커밋**

Run: `/usr/bin/python3 -m pytest operator_console/tests -q`

```bash
git add operator_console/metadata.py operator_console/tests/test_app.py
git commit -m "fix(console): reject distance from an unsupported metadata frame"
```

---

### Task 4: dropout 이후 필터 상태 리셋 (B4)

**Files:**
- Modify: `operator_console/metadata.py` (`DisplayTargetTracker.update`)
- Test: `operator_console/tests/test_app.py`

**Interfaces:**
- Produces: `update(None)` 및 `dropout_hold_ms` 초과 공백 뒤 첫 유효 프레임의
  표시 거리는 **공백 이전 값과 혼합되지 않는다**(EMA/median 초기화).

- [ ] **Step 1: 실패하는 테스트**

```python
def test_long_dropout_clears_the_distance_filter():
    """10초 공백 뒤 재획득한 첫 값이 과거 EMA 와 섞이면 안 된다."""
    tracker = DisplayTargetTracker()
    near = Detection(
        "box-segmentation", 0.96, (500, 290, 220, 190),
        (0.0, 0.0, 0.30), is_pick_target=True,
    )
    for sequence in range(1, 6):
        tracker.update(
            _frame([near], sequence=sequence, now_s=100.0 + sequence * 0.1),
            now_s=100.0 + sequence * 0.1,
        )
    tracker.update(None, now_s=110.0)
    far = Detection(
        "box-segmentation", 0.96, (500, 290, 220, 190),
        (0.0, 0.0, 1.20), is_pick_target=True,
    )
    view = tracker.update(_frame([far], sequence=99, now_s=110.5), now_s=110.5)
    assert view.distance_m == pytest.approx(1.20, abs=1e-3)
```

- [ ] **Step 2: 실패 확인**

Run: `/usr/bin/python3 -m pytest operator_console/tests/test_app.py -q -k dropout`
Expected: FAIL — 같은 bbox 라 IoU 연속으로 판정되고 EMA 가 살아 있어
0.30 과 1.20 이 혼합된 값(예: 0.66)이 나온다. 또는 jump rejection 에 걸려 None.

- [ ] **Step 3: 최소 구현**

`update()` 의 `frame is None` 분기에서 필터를 함께 리셋한다.

```python
        if frame is None:
            self._reset_filter()
            self._last_valid_received_s = None
            self._cached = DisplayTargetView(None, None, "거리 정보 없음")
            return self._cached
```

그리고 프레임이 있어도 마지막 유효 수신으로부터 `dropout_hold_ms` 를 넘겼으면
연속성을 끊는다 — `changed` 계산 직전에 넣는다.

```python
        stale_gap = (
            self._last_valid_received_s is not None
            and (frame.received_monotonic_s - self._last_valid_received_s)
            * 1000.0 > self.config.dropout_hold_ms
        )
        if stale_gap:
            self._reset_filter()
```

- [ ] **Step 4: 통과 확인 + 커밋**

Run: `/usr/bin/python3 -m pytest operator_console/tests -q`

```bash
git add operator_console/metadata.py operator_console/tests/test_app.py
git commit -m "fix(console): reset the distance filter across metadata dropouts"
```

---

### Task 5: draw 핸들러가 필터 상태를 바꾸지 않게 (B5)

**Files:**
- Modify: `operator_console/app.py:434-500` (`MetadataCanvas._on_draw`),
  `operator_console/app.py` (`MetadataCanvas` 생성자), `_sync_overlay_rail`
- Test: `operator_console/tests/test_interactions.py`

**Interfaces:**
- Produces: `DisplayTargetTracker.view()` — 상태를 바꾸지 않고 마지막
  `DisplayTargetView` 를 돌려주는 읽기 전용 접근자. `MetadataCanvas._on_draw`
  는 `update()` 를 호출하지 않는다.

- [ ] **Step 1: 실패하는 테스트**

```python
def test_draw_does_not_advance_the_distance_filter():
    """렌더 횟수가 필터 결과를 바꾸면 화면 가림 여부에 따라 거리가 달라진다."""
    tracker = DisplayTargetTracker()
    target = Detection(
        "box-segmentation", 0.96, (500, 290, 220, 190),
        (0.0, 0.0, 0.30), is_pick_target=True,
    )
    frame = MetadataFrame(
        sequence=1, width=848, height=480, detections=(target,),
        received_monotonic_s=100.0, frame_id="camera_color_optical_frame",
    )
    tracker.update(frame, now_s=100.0)
    before = tracker.view()
    for _ in range(5):
        assert tracker.view() == before
```

- [ ] **Step 2: 실패 확인**

Run: `/usr/bin/python3 -m pytest operator_console/tests/test_interactions.py -q -k draw`
Expected: FAIL — `view()` 가 없다(AttributeError).

- [ ] **Step 3: 최소 구현**

`metadata.py` 에 읽기 전용 접근자를 추가한다.

```python
    def view(self) -> DisplayTargetView:
        """마지막 판정 결과를 상태 변경 없이 돌려준다(렌더 경로 전용)."""
        return self._cached
```

`app.py` `MetadataCanvas._on_draw` 에서 `self._target_tracker.update(frame)` 를
`self._target_tracker.view()` 로 바꾼다. 갱신은 `_sync_overlay_rail` 의
타이머 경로가 단독으로 수행한다(현행 유지). 갱신 소유자가 하나임을 주석으로
남긴다.

```python
        # 갱신 소유자는 _sync_overlay_rail 타이머 하나다.  draw 는 읽기만 한다 —
        # 그리기 횟수가 필터(EMA/median/연속성) 진행을 바꾸면 창이 가려졌을 때
        # 거리값이 달라진다.
        target_view = self._target_tracker.view()
```

- [ ] **Step 4: 통과 확인**

Run: `/usr/bin/python3 -m pytest operator_console/tests -q`
그리고 `/usr/bin/python3 -m operator_console.runtime_smoke` 로 오버레이 경로가
살아 있는지(traceback 0) 확인한다.

- [ ] **Step 5: 커밋**

```bash
git add operator_console/metadata.py operator_console/app.py operator_console/tests/test_interactions.py
git commit -m "fix(console): make the draw path read-only for the distance filter"
```

---

### Task 6: metadata 프레임 크기 불일치 방어 (B6)

**Files:**
- Modify: `operator_console/app.py:434-500` (`MetadataCanvas._on_draw`),
  `operator_console/app.py` (`VideoPanel` 이 실제 영상 해상도를 캔버스에 알려줌)
- Test: `operator_console/tests/test_app.py`

**Interfaces:**
- Produces: `fit_overlay_transform` 은 그대로. `MetadataCanvas.set_video_size(width, height)`
  가 추가되고, metadata 프레임 크기와 실제 영상 크기가 다르면 오버레이를 그리지
  않는다.

- [ ] **Step 1: 실패하는 테스트**

```python
def test_overlay_is_suppressed_when_metadata_size_differs_from_video():
    from operator_console.app import overlay_size_matches
    assert overlay_size_matches(848, 480, 848, 480) is True
    assert overlay_size_matches(848, 480, None, None) is True   # 영상 크기 미상
    assert overlay_size_matches(848, 480, 1280, 720) is False
```

- [ ] **Step 2: 실패 확인**

Run: `/usr/bin/python3 -m pytest operator_console/tests/test_app.py -q -k overlay_size`
Expected: FAIL — ImportError.

- [ ] **Step 3: 최소 구현**

`app.py` 에 순수 함수를 추가한다(테스트 용이).

```python
def overlay_size_matches(
    metadata_width: int, metadata_height: int,
    video_width: int | None, video_height: int | None,
) -> bool:
    """metadata 좌표를 영상 위에 겹쳐도 되는지.

    송신부는 frame_width/height 를 파라미터 기본값으로 싣기 때문에 인식 해상도가
    바뀌면 bbox 가 조용히 어긋난다.  영상 해상도를 알 수 없으면(협상 전) 허용한다.
    """
    if video_width is None or video_height is None:
        return True
    return (metadata_width, metadata_height) == (video_width, video_height)
```

`VideoPanel._on_video_buffer`(또는 sink caps 를 얻는 지점)에서 협상된 caps 의
width/height 를 읽어 `self._metadata_canvas.set_video_size(w, h)` 로 전달하고,
`MetadataCanvas._on_draw` 첫머리에서 다음을 확인한다.

```python
        if not overlay_size_matches(
            frame.width, frame.height, self._video_width, self._video_height,
        ):
            return False
```

`set_video_size` 는 값이 바뀔 때만 `queue_draw()` 한다.

- [ ] **Step 4: 통과 확인 + 커밋**

Run: `/usr/bin/python3 -m pytest operator_console/tests -q` +
`/usr/bin/python3 -m operator_console.runtime_smoke`

```bash
git add operator_console/app.py operator_console/tests/test_app.py
git commit -m "fix(console): suppress the overlay when metadata size and video size disagree"
```

---

### Task 7: PiP 고정 크기 슬롯 (A-1)

**Files:**
- Modify: `operator_console/app.py:2140-2200` (PiP 프레임 구성),
  `operator_console/app.py:2865-2876` (`_on_video_area_allocated`)
- Test: `operator_console/tests/test_interactions.py`

**Interfaces:**
- Produces: `class FixedSizeSlot(Gtk.Bin)` — 자식의 natural size 를 전파하지 않고
  `set_slot_size(width, height)` 로 지정된 크기만 요구하는 컨테이너.
  PiP 프레임의 자식은 항상 이 슬롯이다.

- [ ] **Step 1: 실패하는 테스트**

```python
def test_pip_slot_ignores_the_child_natural_size():
    """PiP 는 스트림 해상도(848x480/1280x720)와 무관하게 지정 크기를 요구한다."""
    from operator_console.app import FixedSizeSlot

    big = Gtk.DrawingArea()
    big.set_size_request(1280, 720)          # gtksink 가 영상 해상도를 요구하는 상황
    slot = FixedSizeSlot()
    slot.add(big)
    slot.set_slot_size(360, 204)

    minimum_w, natural_w = slot.get_preferred_width()
    minimum_h, natural_h = slot.get_preferred_height()
    assert (minimum_w, natural_w) == (360, 360)
    assert (minimum_h, natural_h) == (204, 204)
```

- [ ] **Step 2: 실패 확인**

Run: `/usr/bin/python3 -m pytest operator_console/tests/test_interactions.py -q -k pip_slot`
Expected: FAIL — ImportError.

- [ ] **Step 3: 최소 구현**

```python
class FixedSizeSlot(Gtk.Bin):
    """자식의 natural size 를 전파하지 않는 고정 크기 컨테이너.

    Gtk 의 size request 는 '최소'라서, gtksink 가 영상 해상도를 natural size 로
    보고하면 Overlay 가 그 크기로 할당해 PiP 가 스테이지를 덮어버린다
    (2026-07-29 실기 관측).  이 슬롯은 지정된 크기만 요구한다.
    """

    def __init__(self) -> None:
        super().__init__()
        self._slot_width = 1
        self._slot_height = 1

    def set_slot_size(self, width: int, height: int) -> None:
        width, height = max(1, int(width)), max(1, int(height))
        if (width, height) == (self._slot_width, self._slot_height):
            return
        self._slot_width, self._slot_height = width, height
        self.queue_resize()

    def do_get_preferred_width(self) -> tuple[int, int]:
        return self._slot_width, self._slot_width

    def do_get_preferred_height(self) -> tuple[int, int]:
        return self._slot_height, self._slot_height

    def do_get_preferred_width_for_height(self, _height: int) -> tuple[int, int]:
        return self._slot_width, self._slot_width

    def do_get_preferred_height_for_width(self, _width: int) -> tuple[int, int]:
        return self._slot_height, self._slot_height
```

PiP 구성에서 프레임의 자식으로 슬롯을 끼운다.

```python
        self._pip_slot = FixedSizeSlot()
        self._pip_slot.add(self._d435)
        pip_frame.add(self._pip_slot)
```

`_on_video_area_allocated` 는 프레임이 아니라 슬롯에 크기를 준다.

```python
    def _on_video_area_allocated(
        self, _widget: Gtk.Overlay, allocation: Gdk.Rectangle,
    ) -> None:
        if allocation.width < 1:
            return
        pip_width = min(430, max(300, int(allocation.width * 0.30)))
        # D435i transport is 848x480.  Preserve the exact native ratio instead
        # of the close-but-not-identical 16:9 approximation.
        pip_height = int(round(pip_width * 480 / 848))
        self._pip_slot.set_slot_size(pip_width, pip_height)
```

`swap_camera_views` 는 `self._pip_frame` 대신 `self._pip_slot` 에서
자식을 넣고 뺀다(Task 8 에서 함께 정리해도 되지만, 이 태스크에서 바로 맞춘다).

- [ ] **Step 4: 통과 확인**

Run: `/usr/bin/python3 -m pytest operator_console/tests -q` +
`/usr/bin/python3 -m operator_console.runtime_smoke`

- [ ] **Step 5: 실기 확인(중요)**

젯슨의 전방 :5000 + 작업 :5002 가 살아 있는 상태로 콘솔을 띄우고,
PiP 가 계산 크기를 유지하는지 스크린샷으로 확인한다.

```bash
env -u WAYLAND_DISPLAY GDK_BACKEND=x11 DISPLAY=:77 /usr/bin/python3 -m operator_console.app
```

수정 전 증거는 `live_full/shot_04.png`(PiP 가 전방 화면을 덮음)다.

- [ ] **Step 6: 커밋**

```bash
git add operator_console/app.py operator_console/tests/test_interactions.py
git commit -m "fix(console): clamp the PiP slot so the work camera cannot swallow the stage"
```

---

### Task 8: 클릭 스왑 제거 + 명시 조작 (A-2, A-3)

**Files:**
- Modify: `operator_console/app.py:1007-1018` (헤더 EventBox·스왑 힌트),
  `operator_console/app.py:1129-1133` (`_on_swap_click`),
  `operator_console/app.py:2190-2200` (`set_swap_handler` 배선),
  `operator_console/app.py` (화면 표시 패널, 키 입력 핸들러)
- Test: `operator_console/tests/test_interactions.py`

**Interfaces:**
- Produces: 영상/헤더 클릭은 MAIN 을 바꾸지 않는다. `OperatorConsole` 에
  `_swap_button`(라벨 `주/보조 화면 교체`)이 생기고, 단축키 `V` 로도 같은
  동작을 한다. `swap_camera_views(panel, user_initiated=True)` 시그니처는 유지.

- [ ] **Step 1: 실패하는 테스트**

```python
def test_video_header_click_does_not_swap_views():
    """오조작 방지: 화면을 눌러도 주/보조가 바뀌지 않는다."""
    from operator_console.app import VideoPanel

    panel = VideoPanel("작업 카메라", "127.0.0.1", 15002, 60)
    assert not hasattr(panel, "_swap_handler") or panel._swap_handler is None
    assert panel._header_click.get_events() is not None
    # 헤더 EventBox 에 button-press 스왑 콜백이 남아 있으면 안 된다
    assert getattr(panel, "_swap_click_connected", False) is False
```

`OperatorConsole` 전체 기동 테스트는 무겁다. 대신 아래 계약 테스트를 둔다.

```python
def test_swap_is_reachable_only_from_the_explicit_control():
    import inspect
    from operator_console import app as console_app

    source = inspect.getsource(console_app.VideoPanel)
    assert "_on_swap_click" not in source, (
        "영상 패널에서 클릭 스왑 경로가 남아 있다"
    )
    console_source = inspect.getsource(console_app.OperatorConsole)
    assert "화면 교체" in console_source, "명시 스왑 버튼이 없다"
```

- [ ] **Step 2: 실패 확인**

Run: `/usr/bin/python3 -m pytest operator_console/tests/test_interactions.py -q -k swap`
Expected: FAIL — 아직 클릭 경로가 있다.

- [ ] **Step 3: 최소 구현**

1. `VideoPanel` 에서 `_on_swap_click` 메서드와
   `header_click.connect("button-press-event", ...)` 를 제거한다.
   `set_swap_handler` 와 `self._swap_handler` 도 제거한다.
   헤더 EventBox 자체는 배경 스타일 때문에 유지하되, 라벨
   `swap_hint` 텍스트는 `set_role` 에서 항상 빈 문자열로 둔다
   ("클릭하여 크게 보기" 문구 삭제).
2. `OperatorConsole` 의 `_l515.set_swap_handler(...)`/`_d435.set_swap_handler(...)`
   호출을 제거한다.
3. "화면 표시" 패널(`display_options`)에 버튼을 추가한다.

```python
        self._swap_button = Gtk.Button(label="주/보조 화면 교체")
        _style(self._swap_button, "status-view-option")
        self._swap_button.set_tooltip_text(
            "큰 화면과 작은 화면의 카메라를 맞바꿉니다 (단축키 V)"
        )
        self._swap_button.connect(
            "clicked",
            lambda _button: self.swap_camera_views(
                self._d435 if self._main_video is self._l515 else self._l515,
                user_initiated=True,
            ),
        )
        display_options.pack_start(self._swap_button, False, False, 0)
```

4. `_on_key_press` 에 `V` 를 추가한다(기존 F11 처리와 같은 자리).

```python
        if event.keyval in (Gdk.KEY_v, Gdk.KEY_V):
            self.swap_camera_views(
                self._d435 if self._main_video is self._l515 else self._l515,
                user_initiated=True,
            )
            return True
```

- [ ] **Step 4: 통과 확인**

Run: `/usr/bin/python3 -m pytest operator_console/tests -q` +
`/usr/bin/python3 -m operator_console.runtime_smoke`
(스모크의 "자동 카메라 스왑 0" 단언이 계속 통과해야 한다.)

- [ ] **Step 5: 실기 확인**

Xvfb 에서 xdotool 로 PiP 헤더를 클릭해 **스왑이 일어나지 않음**을,
버튼 클릭으로는 **스왑되고 PiP 크기가 유지됨**을 스크린샷으로 확인한다.

- [ ] **Step 6: 커밋**

```bash
git add operator_console/app.py operator_console/tests/test_interactions.py
git commit -m "fix(console): swap views only from an explicit control, never a video click"
```

---

### Task 9: E-STOP 가용성 동적화 + 전송 실패 경고 (C2)

**Files:**
- Modify: `operator_console/app.py:2239-2290` (전역 E-STOP 버튼),
  `operator_console/app.py` (`OpsPanel` 상태 콜백), `_refresh_health`
- Test: `operator_console/tests/test_ops_panel.py`

**Interfaces:**
- Produces: `OpsPanel.link_ready() -> bool` (토큰 존재 ∧ ops 연결 생존),
  `OperatorConsole._refresh_estop_availability()` 가 250 ms 주기로 버튼
  민감도와 사유 문구를 갱신한다. 전송 실패·`OUTCOME_UNKNOWN` 은 상단 경고로
  올라온다.

- [ ] **Step 1: 실패하는 테스트**

```python
def test_link_ready_requires_both_token_and_live_connection(tmp_path):
    token = tmp_path / "ops.token"
    token.write_text("t")
    panel = OpsPanel("127.0.0.1", 19001, str(token), event_sink=lambda *_: None)
    assert panel.ops_available() is True      # 토큰은 있다
    assert panel.link_ready() is False        # 브로커가 없으면 준비 아님


def test_link_ready_is_false_without_a_token(tmp_path):
    panel = OpsPanel(
        "127.0.0.1", 19001, str(tmp_path / "missing.token"),
        event_sink=lambda *_: None,
    )
    assert panel.ops_available() is False
    assert panel.link_ready() is False
```

- [ ] **Step 2: 실패 확인**

Run: `/usr/bin/python3 -m pytest operator_console/tests/test_ops_panel.py -q -k link_ready`
Expected: FAIL — `link_ready` 가 없다.

- [ ] **Step 3: 최소 구현**

`OpsPanel` 에 링크 생존 판정을 추가한다. 판정 근거는 `ops_client` 가 이미
가지고 있는 연결 상태(마지막 성공 수신 시각/연결 플래그)를 쓴다. 없으면
`_client.connected` 를 노출시키고 그걸 쓴다.

```python
    def link_ready(self) -> bool:
        """토큰이 있고 ops 링크가 살아 있을 때만 True."""
        if not self.ops_available():
            return False
        return bool(getattr(self._client, "connected", False))
```

`OperatorConsole` 에서 주기 갱신한다(`_refresh_health` 안, 250 ms).

```python
        ready = self._ops_panel.link_ready()
        self._global_estop.set_sensitive(ready)
        self._global_estop.set_tooltip_text(
            "확인 없이 즉시 토큰 인증 비상정지 명령을 전송합니다" if ready
            else "조작 채널이 연결되지 않아 비상정지를 전송할 수 없습니다"
            if self._ops_panel.ops_available()
            else "조작 토큰이 없어 비상정지 명령을 전송할 수 없습니다"
        )
```

전송 결과가 실패/`OUTCOME_UNKNOWN` 이면 상단 상태줄에 경고를 띄운다
(`_on_submit_response` 에서 이벤트만 남기지 말고 `self._show_alert(...)` 호출).
`_show_alert` 는 topbar 아래 한 줄짜리 라벨을 8초간 보이게 하는 최소 구현으로
충분하다.

- [ ] **Step 4: 통과 확인 + 커밋**

Run: `/usr/bin/python3 -m pytest operator_console/tests -q` +
`/usr/bin/python3 -m operator_console.runtime_smoke`

```bash
git add operator_console/app.py operator_console/tests/test_ops_panel.py
git commit -m "fix(console): gate E-STOP on live ops link and surface send failures"
```

---

### Task 10: freshness 를 health 로 오표시하지 않기 (C3)

**Files:**
- Modify: `operator_console/status_view.py:687-700` (전원 카드 판정)
- Test: `operator_console/tests/test_status_view.py`

**Interfaces:**
- Produces: `rs485_state` 가 정상이 아니거나 전압·SOC 가 모두 결측이면
  전원 카드 상태는 `"확인 필요"`이고 필수 준비 카운트에서 제외된다.

- [ ] **Step 1: 실패하는 테스트**

```python
def test_power_card_is_not_normal_when_rs485_link_is_dead():
    """실기 관측: rs485_state=ERROR, 전압·SOC 전부 None 인데 '정상' 이었다."""
    snapshot = parse_telemetry(json.dumps({
        "schema_version": 1, "sequence": 1,
        "voltage_v": None, "current_a": None, "power_w": None,
        "drive_state": "PDIST unavailable", "can_state": "unavailable",
        "pdist_soc_percent": None, "pdist_battery_flags": None,
        "pdist_protection_flags": None, "rs485_state": "ERROR",
        "rs485_consecutive_failures": 3691,
    }).encode("utf-8"))
    state, reason = power_card_state(snapshot, fresh=True)
    assert state == "확인 필요"
    assert "ERROR" in reason or "전원" in reason
```

`power_card_state` 는 이 태스크에서 새로 뽑아내는 순수 함수다.

- [ ] **Step 2: 실패 확인**

Run: `/usr/bin/python3 -m pytest operator_console/tests/test_status_view.py -q -k power_card`
Expected: FAIL — ImportError.

- [ ] **Step 3: 최소 구현**

`status_view.py` 에 순수 함수를 추가하고 `update()` 가 그것을 쓰게 한다.

```python
def power_card_state(power: object | None, *, fresh: bool) -> tuple[str, str]:
    """수신 신선도와 전원 장치 건강을 분리해 판정한다.

    datagram 이 제때 와도 PDIST 링크가 죽어 있으면 전원은 '정상'이 아니다
    (2026-07-29 실기: rs485_state=ERROR, 연속 실패 3691, 전압·SOC 전부 결측인데
    카드가 '정상'이었다).
    """
    if power is None:
        return "정보 없음", "전원 장치 정보 수신 대기"
    if not fresh:
        return "확인 필요", "전원 정보 수신 지연"
    rs485 = (getattr(power, "rs485_state", "") or "").upper()
    if rs485 and rs485 not in ("OK", "VALID", "NORMAL"):
        return "확인 필요", f"전원 링크 {rs485}"
    if (
        getattr(power, "voltage_v", None) is None
        and getattr(power, "pdist_soc_percent", None) is None
    ):
        return "확인 필요", "전원 계측값 없음"
    if (
        getattr(power, "pdist_battery_flags", None) not in (None, 0)
        or getattr(power, "pdist_protection_flags", None) not in (None, 0)
    ):
        return "확인 필요", "보호 상태 확인 필요"
    return "정상", getattr(power, "rs485_state", "") or "정상"
```

`update()` 의 기존 `power_state`/`power_reason` 계산을 이 함수 호출로 교체한다.
`required_ready` 계산은 그대로 두면 자동으로 준비 카운트에서 빠진다.

- [ ] **Step 4: 통과 확인 + 커밋**

Run: `/usr/bin/python3 -m pytest operator_console/tests -q`

```bash
git add operator_console/status_view.py operator_console/tests/test_status_view.py
git commit -m "fix(console): a dead PDIST link is not a healthy power system"
```

---

### Task 11: runtime_smoke 의 화면 격리 실제화 (C5)

**Files:**
- Modify: `operator_console/runtime_smoke.py:1-14`(docstring),
  `operator_console/runtime_smoke.py:197-212`(Popen)
- Test: `operator_console/tests/test_runtime_smoke.py`

**Interfaces:**
- Produces: 스모크가 자식 프로세스 환경에서 `WAYLAND_DISPLAY` 를 제거하고
  `GDK_BACKEND=x11` 을 넣어 Xvfb 로 강제한다.

- [ ] **Step 1: 실패하는 테스트**

```python
def test_smoke_env_forces_x11_backend():
    """Wayland 세션에서는 GDK 가 DISPLAY 를 무시해 사용자 화면에 창이 뜬다."""
    from operator_console.runtime_smoke import smoke_child_env

    env = smoke_child_env({"WAYLAND_DISPLAY": "wayland-0", "PATH": "/usr/bin"})
    assert "WAYLAND_DISPLAY" not in env
    assert env["GDK_BACKEND"] == "x11"
    assert env["PATH"] == "/usr/bin"
```

- [ ] **Step 2: 실패 확인**

Run: `/usr/bin/python3 -m pytest operator_console/tests/test_runtime_smoke.py -q -k x11`
Expected: FAIL — ImportError.

- [ ] **Step 3: 최소 구현**

```python
def smoke_child_env(base: dict[str, str] | None = None) -> dict[str, str]:
    """xvfb-run 이 실제로 격리되도록 강제하는 자식 프로세스 환경.

    Wayland 세션에서 GDK 는 DISPLAY 보다 WAYLAND_DISPLAY 를 우선해 잡는다.
    그대로 두면 '사용자 화면에 창을 띄우지 않는다'는 이 하니스의 주장이 거짓이 되고
    X11 렌더 경로 결함도 놓친다(2026-07-29 확인).
    """
    env = dict(os.environ if base is None else base)
    env.pop("WAYLAND_DISPLAY", None)
    env["GDK_BACKEND"] = "x11"
    return env
```

`Popen(...)` 에 `env=smoke_child_env()` 를 추가하고, 모듈 docstring 12~13행의
설명을 실제 동작에 맞게 고친다.

- [ ] **Step 4: 음성 대조**

`env=` 인자를 잠시 빼고 Wayland 세션에서 스모크를 돌려 사용자 화면에 창이
뜨는지 눈으로 확인한 뒤 되돌린다. 확인 결과를 커밋 메시지에 한 줄로 남긴다.

- [ ] **Step 5: 통과 확인 + 커밋**

Run: `/usr/bin/python3 -m pytest operator_console/tests -q` +
`/usr/bin/python3 -m operator_console.runtime_smoke`

```bash
git add operator_console/runtime_smoke.py operator_console/tests/test_runtime_smoke.py
git commit -m "fix(console): make the runtime smoke actually isolate the display"
```

---

### Task 12: 젯슨 ops_broker 복구 (C1) — 콘솔과 별도

**Files:**
- 조사 대상(젯슨): `ros2/launch/control.launch.py`,
  `ros2/src/powertrain_ros/setup.py`, `docker/docker-compose.jetson.yml`,
  컨테이너 `powertrain_control` 의 오버레이 소싱 경로
- Test: 젯슨 실기 판정(아래 Step 4)

**Interfaces:**
- Produces: `powertrain_control` 컨테이너에서 `ops_broker` 가 살아 있고
  TCP :9001 이 LISTEN 이며 healthcheck 가 healthy.

- [ ] **Step 1: 원인 확정**

```bash
ssh zetin@192.168.8.106 'docker exec powertrain_control bash -lc "
  ls -d /workspace/ros2/install/powertrain_msgs 2>&1;
  python3 -c \"import powertrain_msgs\" 2>&1 | tail -2;
  echo \$AMENT_PREFIX_PATH; echo \$PYTHONPATH"'
```

`teleop_command` 는 같은 런치에서 정상 기동하므로, 차이는 `ops_broker_node.py`
가 `powertrain_msgs.msg.WheelStates` 를 import 한다는 점 하나다. 위 출력으로
① 패키지 미설치 ② 오버레이 미소싱 ③ PYTHONPATH 누락 중 무엇인지 가른다.

- [ ] **Step 2: 재현 가능한 실패를 남긴다**

원인이 확정되면 그 상태를 그대로 기록한다(컨테이너 로그 원문 + 위 출력).
이것이 수정 전 증거다.

- [ ] **Step 3: 수정**

원인에 따라 하나만 고친다. 임시 우회(PYTHONPATH 수동 주입 등)로 끝내지 말고
**컨테이너 재시작 후에도 유지되는 경로**(compose 환경 또는 빌드/설치 단계)에
반영한다.

- [ ] **Step 4: 판정**

```bash
ssh zetin@192.168.8.106 '
  docker compose -f ~/power-train-sw/docker/docker-compose.jetson.yml restart powertrain_control;
  sleep 25;
  ss -ltn | grep ":9001";
  docker inspect --format "{{.State.Health.Status}}" powertrain_control;
  docker logs --tail 20 powertrain_control | grep -i ops_broker'
```

Expected: `:9001` LISTEN, `healthy`, ops_broker traceback 없음.

- [ ] **Step 5: 커밋 (콘솔 커밋과 분리)**

```bash
git commit -m "fix(deploy): restore ops_broker on the Jetson control stack"
```

---

### Task 12b: 젯슨에 arm_console_bridge 수정본 배포 (B1-a) — Task 2 의 전제

**왜 필수인가.** Task 2 가 폴백을 없애면 표시 거리는 **송신자가 지정한 pick
target 에서만** 나온다. 그런데 젯슨에 배포된 `arm_console_bridge` 는 PR #3
이전 버전이라 이전 프레임 bbox 와 **완전 일치**를 요구하고, 실측에서
`/pick_target` [506,293,215,187] 과 `/detected_objects` [503,293,219,187] 이
어긋나 `is_pick_target: false` 만 보낸다. 배포를 안 하면 **거리가 영영
표시되지 않는다.**

**Files:**
- Deploy: 젯슨 `~/power-train-sw` 의 `ros2/src/powertrain_ros/powertrain_ros/arm_console_mirror.py`,
  `arm_console_bridge_node.py` (PR #3 변경분)

- [ ] **Step 1: 배포 전 증거 저장**

```bash
timeout 12 /usr/bin/python3 -c "
import socket,json
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.bind(('0.0.0.0',5003)); s.settimeout(8)
d,_=s.recvfrom(65535); p=json.loads(d)
print([(x['bbox_xywh'], x['is_pick_target']) for x in p['detections']])"
```

Expected(수정 전): `is_pick_target` 이 False.

- [ ] **Step 2: 배포**

젯슨 체크아웃에 PR #3 의 브리지 변경분을 반영하고 `powertrain_ros` 를 빌드한
뒤 브리지를 재기동한다. 젯슨 체크아웃에는 팀원의 미커밋 작업이 있으므로
**절대 `git checkout`/`reset` 으로 덮지 말고** 해당 두 파일만 반영한다.

- [ ] **Step 3: 판정**

Step 1 명령을 다시 돌려 같은 상자에 대해 `is_pick_target: true` 가 오는지
확인한다. `/pick_target` 이 비어 있으면 `perception_node` 의
`pick_classes:=box-segmentation` 이 설정됐는지 먼저 본다(미설정이면 후보 자체가
없다 — B1-b).

- [ ] **Step 4: 커밋 (젯슨 배포 기록)**

```bash
git commit -m "deploy(jetson): arm_console_bridge IoU pick-target matching"
```

---

### Task 13: 운용 호스트 IP 정리 (C4) — 결정 필요 항목

**Files:**
- Create: `docs/reports/2026-07-29-operator-console-live-review.md`
- Modify: 없음(코드 변경은 사용자 결정 후)

**Interfaces:**
- Produces: 현상·영향·후보안이 정리된 보고서 한 편.

- [ ] **Step 1: 보고서 작성**

다음을 기록한다.

- 젯슨의 텔레메트리 송신기들이 `--operator-host 192.168.8.206` 로 고정되어
  있고 그 IP 는 현재 사용되지 않는다. 운용 노트북은 192.168.8.163 이다.
- 이번 검증에서는 노트북에 `192.168.8.206/24` 를 임시로 부여해 우회했고
  세션 종료 시 제거했다.
- 후보 ①: 공유기(GL-SFT1200)에서 운용 노트북에 DHCP 예약을 걸어 .206 고정.
  후보 ②: 송신기 파라미터를 배포 설정으로 빼서 현장에서 바꾸기 쉽게.
- 권고: ①이 즉효이고 코드 변경이 없다. ②는 대회 현장처럼 네트워크가 바뀌는
  상황에 필요하다. 둘 다 해도 충돌하지 않는다.

- [ ] **Step 2: 커밋**

```bash
git add docs/reports/2026-07-29-operator-console-live-review.md
git commit -m "docs(report): operator console live review findings and IP follow-up"
```

---

### Task 14: 최종 통합 검증

- [ ] **Step 1: 전체 스위트**

Run: `/usr/bin/python3 -m pytest operator_console/tests -q`
Expected: 215 + 신규 테스트 전건 PASS.

- [ ] **Step 2: runtime smoke**

Run: `/usr/bin/python3 -m operator_console.runtime_smoke`
Expected: PASS, traceback 0, 자동 카메라 스왑 0.

- [ ] **Step 3: 실기 라이브 (젯슨)**

전방 :5000 + 팔팀 작업 :5002 + 브리지 :5003 이 살아 있는 상태에서 콘솔을 띄우고
다음을 캡처로 남긴다.

1. PiP 가 계산 크기를 유지한다(전방 화면을 덮지 않는다).
2. 헤더 클릭으로 스왑되지 않고, 버튼/`V` 로는 스왑되며 그때도 PiP 크기가 같다.
3. 거리 표시가 **SDK depth 와 ±5 mm 이내**로 일치한다.
   `/detected_objects` 의 z, `depth_frame.get_distance()`, 콘솔 표시값을 나란히
   기록한다(오늘 기준선 0.2882 / 0.2884 / 0.3199).
4. 10분 소크에서 RSS·fd·thread 안정, traceback 0.

- [ ] **Step 4: 보고서 갱신 후 커밋**

`docs/reports/2026-07-29-operator-console-live-review.md` 에 수정 전/후 수치와
캡처 경로를 채우고 커밋한다.
