"""Versioned D435 YOLO metadata validation and latest-only UDP reception."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import math
import socket
import threading
import time
from typing import Any

from .udp_source import SourceSequenceGate

MIN_DISPLAY_CONFIDENCE = 0.5
DISPLAY_BORDER_MARGIN_PX = 4
DISPLAY_MAX_BORDER_CONTACTS = 2
OVERLAY_STALE_AFTER_S = 0.50
SUPPORTED_DISTANCE_FRAME_IDS = ("camera_color_optical_frame",)


@dataclass(frozen=True)
class Detection:
    class_name: str
    confidence: float
    bbox_xywh: tuple[int, int, int, int]
    position_m: tuple[float, float, float] | None
    yaw_rad: float | None = None
    is_pick_target: bool = False
    depth_m: float | None = None


@dataclass(frozen=True)
class MetadataFrame:
    sequence: int
    width: int
    height: int
    detections: tuple[Detection, ...]
    received_monotonic_s: float
    # v1 :5003 is contractually D435i/work-camera metadata.
    source_camera_id: str = "work"
    capture_stamp_ns: int | None = None
    frame_id: str | None = None


def displayable_detections(
    frame: MetadataFrame | None,
) -> tuple[Detection, ...]:
    """Apply display-quality gates without mutating receiver data."""
    if frame is None:
        return ()

    def sufficiently_visible(detection: Detection) -> bool:
        x, y, width, height = detection.bbox_xywh
        contacts = sum((
            x <= DISPLAY_BORDER_MARGIN_PX,
            y <= DISPLAY_BORDER_MARGIN_PX,
            x + width >= frame.width - DISPLAY_BORDER_MARGIN_PX,
            y + height >= frame.height - DISPLAY_BORDER_MARGIN_PX,
        ))
        return contacts <= DISPLAY_MAX_BORDER_CONTACTS

    return tuple(
        detection for detection in frame.detections
        if detection.confidence >= MIN_DISPLAY_CONFIDENCE
        and sufficiently_visible(detection)
    )


def pick_display_target(frame: MetadataFrame | None) -> Detection | None:
    """Return only the sender-designated target; never invent one in the UI."""
    return next(
        (
            detection for detection in displayable_detections(frame)
            if detection.is_pick_target
        ),
        None,
    )


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


@dataclass(frozen=True)
class DistanceFilterConfig:
    window_size: int = 3
    ema_alpha: float = 0.60
    max_frame_age_ms: float = OVERLAY_STALE_AFTER_S * 1000.0
    dropout_hold_ms: float = 350.0
    jump_rejection_m: float = 0.50
    jump_rejection_ratio: float = 2.5
    minimum_distance_m: float = 0.10
    maximum_distance_m: float = 10.0
    continuity_iou: float = 0.20


@dataclass(frozen=True)
class DisplayTargetView:
    detection: Detection | None
    distance_m: float | None
    distance_state: str
    held: bool = False


def _bbox_iou(
    first: tuple[int, int, int, int],
    second: tuple[int, int, int, int],
) -> float:
    ax, ay, aw, ah = first
    bx, by, bw, bh = second
    left, top = max(ax, bx), max(ay, by)
    right, bottom = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    intersection = max(0, right - left) * max(0, bottom - top)
    union = aw * ah + bw * bh - intersection
    return 0.0 if union <= 0 else intersection / union


class DisplayTargetTracker:
    """View-only target continuity and bounded distance stabilization."""

    def __init__(self, config: DistanceFilterConfig | None = None) -> None:
        self.config = config or DistanceFilterConfig()
        self._distances: deque[float] = deque(maxlen=self.config.window_size)
        self._last_bbox: tuple[int, int, int, int] | None = None
        self._last_class: str | None = None
        self._ema: float | None = None
        self._last_sequence: int | None = None
        self._last_valid_received_s: float | None = None
        self._cached = DisplayTargetView(None, None, "거리 확인 중")

    def _reset_filter(self) -> None:
        self._distances.clear()
        self._ema = None
        self._last_bbox = None
        self._last_class = None

    def view(self) -> DisplayTargetView:
        """마지막 판정 결과를 상태 변경 없이 돌려준다(렌더 경로 전용)."""
        return self._cached

    def update(
        self, frame: MetadataFrame | None, *, now_s: float | None = None,
    ) -> DisplayTargetView:
        if frame is None:
            self._reset_filter()
            self._last_valid_received_s = None
            self._cached = DisplayTargetView(None, None, "거리 정보 없음")
            return self._cached
        now_s = time.monotonic() if now_s is None else now_s
        if (
            (now_s - frame.received_monotonic_s) * 1000.0
            > self.config.max_frame_age_ms
        ):
            self._cached = DisplayTargetView(None, None, "거리 정보 지연")
            return self._cached
        if (
            frame.frame_id is not None
            and frame.frame_id not in SUPPORTED_DISTANCE_FRAME_IDS
        ):
            self._reset_filter()
            self._cached = DisplayTargetView(None, None, "거리 기준 불일치")
            return self._cached
        if frame.sequence == self._last_sequence:
            if (
                self._cached.detection is not None
                and self._last_valid_received_s is not None
                and (now_s - self._last_valid_received_s) * 1000.0
                > self.config.dropout_hold_ms
            ):
                self._cached = DisplayTargetView(
                    None, None, "대상 탐지 대기",
                )
            return self._cached
        self._last_sequence = frame.sequence

        detections = displayable_detections(frame)
        explicit = next(
            (item for item in detections if item.is_pick_target), None,
        )
        continuous = None
        if explicit is None and self._last_bbox is not None:
            candidates = [
                item for item in detections
                if item.class_name == self._last_class
                and _bbox_iou(item.bbox_xywh, self._last_bbox)
                >= self.config.continuity_iou
            ]
            continuous = max(
                candidates, key=lambda item: item.confidence, default=None,
            )
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

        stale_gap = (
            self._last_valid_received_s is not None
            and (frame.received_monotonic_s - self._last_valid_received_s)
            * 1000.0 > self.config.dropout_hold_ms
        )
        if stale_gap:
            self._reset_filter()
        self._last_valid_received_s = frame.received_monotonic_s
        changed = (
            self._last_bbox is None
            or selected.class_name != self._last_class
            or _bbox_iou(selected.bbox_xywh, self._last_bbox)
            < self.config.continuity_iou
        )
        if changed:
            self._distances.clear()
            self._ema = None
        self._last_bbox = selected.bbox_xywh
        self._last_class = selected.class_name
        raw_distance = target_distance_m(selected)
        if raw_distance is None:
            self._cached = DisplayTargetView(
                selected, None, "거리 확인 중",
            )
            return self._cached
        if not (
            self.config.minimum_distance_m
            <= raw_distance
            <= self.config.maximum_distance_m
        ):
            self._cached = DisplayTargetView(
                selected, None, "거리 확인 중",
            )
            return self._cached

        if self._distances:
            baseline = sorted(self._distances)[len(self._distances) // 2]
            ratio = max(raw_distance, baseline) / max(
                min(raw_distance, baseline), 1e-6,
            )
            if (
                abs(raw_distance - baseline)
                > self.config.jump_rejection_m
                and ratio > self.config.jump_rejection_ratio
            ):
                self._cached = DisplayTargetView(
                    selected, None, "거리 갱신 중",
                )
                return self._cached
        self._distances.append(raw_distance)
        ordered = sorted(self._distances)
        median = ordered[len(ordered) // 2]
        self._ema = (
            median if self._ema is None
            else self.config.ema_alpha * median
            + (1.0 - self.config.ema_alpha) * self._ema
        )
        self._cached = DisplayTargetView(selected, self._ema, "정상")
        return self._cached


def parse_metadata(raw: bytes, received_monotonic_s: float | None = None) -> MetadataFrame:
    """Validate the small v1 JSON datagram; reject malformed sender input."""
    if len(raw) > 2048:
        raise ValueError("oversize metadata")
    payload: dict[str, Any] = json.loads(raw.decode("utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("invalid metadata")
    if payload.get("schema_version") != 1:
        raise ValueError("unsupported schema")
    try:
        width = int(payload["frame_width"])
        height = int(payload["frame_height"])
        sequence = int(payload["capture_sequence"])
    except (KeyError, TypeError, OverflowError) as exc:
        raise ValueError("invalid metadata structure") from exc
    if width < 1 or height < 1:
        raise ValueError("invalid frame dimensions")
    raw_detections = payload.get("detections", [])
    if not isinstance(raw_detections, list):
        raise ValueError("invalid detections")
    detections: list[Detection] = []
    for item in raw_detections:
        if not isinstance(item, dict):
            raise ValueError("invalid detection")
        try:
            box = tuple(int(value) for value in item["bbox_xywh"])
            class_name = str(item["class_name"])
            confidence = float(item["confidence"])
        except (KeyError, TypeError, OverflowError) as exc:
            raise ValueError("invalid detection structure") from exc
        if len(box) != 4:
            raise ValueError("invalid bbox")
        if box[2] < 1 or box[3] < 1:
            continue
        xyz = item.get("position_m")
        try:
            position = (
                None if xyz is None else tuple(float(value) for value in xyz)
            )
        except (TypeError, OverflowError) as exc:
            raise ValueError("invalid position") from exc
        if position is not None and len(position) != 3:
            raise ValueError("invalid position")
        if position is not None and (
            not all(math.isfinite(value) for value in position)
            or position[2] <= 0.0
        ):
            position = None
        raw_depth = item.get("depth_m")
        try:
            depth_m = None if raw_depth is None else float(raw_depth)
        except (TypeError, OverflowError) as exc:
            raise ValueError("invalid depth") from exc
        if depth_m is not None and (
            not math.isfinite(depth_m) or depth_m <= 0.0
        ):
            depth_m = None
        raw_yaw = item.get("yaw_rad")
        try:
            yaw_rad = None if raw_yaw is None else float(raw_yaw)
        except (TypeError, OverflowError) as exc:
            raise ValueError("invalid yaw") from exc
        if yaw_rad is not None and not math.isfinite(yaw_rad):
            raise ValueError("invalid yaw")
        is_pick_target = item.get("is_pick_target", False)
        if not isinstance(is_pick_target, bool):
            raise ValueError("invalid is_pick_target")
        detections.append(Detection(
            class_name, confidence, box, position,
            yaw_rad, is_pick_target, depth_m,
        ))
    capture_stamp_ns = payload.get("capture_stamp_ns")
    if capture_stamp_ns is not None:
        if not isinstance(capture_stamp_ns, int) or capture_stamp_ns < 0:
            raise ValueError("invalid capture timestamp")
    frame_id = payload.get("frame_id")
    if frame_id is not None and not isinstance(frame_id, str):
        raise ValueError("invalid frame id")
    return MetadataFrame(
        sequence=sequence, width=width, height=height,
        detections=tuple(detections),
        received_monotonic_s=time.monotonic() if received_monotonic_s is None else received_monotonic_s,
        capture_stamp_ns=capture_stamp_ns,
        frame_id=frame_id,
    )


class LatestMetadataReceiver:
    """Non-blocking latest-only UDP receiver; GUI consumers never block on it."""
    def __init__(self, port: int) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind(("0.0.0.0", port))
        self._latest: MetadataFrame | None = None
        self._lock = threading.Lock()
        self._stopping = threading.Event()
        self._source_gate = SourceSequenceGate(stale_after_s=2.0)
        self._invalid_packet_count = 0
        self._thread = threading.Thread(target=self._run, name="d435-metadata", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        self._socket.settimeout(0.2)
        while not self._stopping.is_set():
            try:
                raw, address = self._socket.recvfrom(4096)
            except OSError:
                continue
            received_s = time.monotonic()
            try:
                frame = parse_metadata(raw, received_monotonic_s=received_s)
                accepted = self._source_gate.accept(
                    address,
                    frame.sequence,
                    now_s=received_s,
                )
            except Exception:
                self._invalid_packet_count += 1
                continue
            if not accepted:
                continue
            with self._lock:
                self._latest = frame

    @property
    def invalid_packet_count(self) -> int:
        return self._invalid_packet_count

    def latest(self) -> MetadataFrame | None:
        with self._lock:
            return self._latest

    def close(self) -> None:
        self._stopping.set()
        self._socket.close()
        self._thread.join(timeout=1.0)
