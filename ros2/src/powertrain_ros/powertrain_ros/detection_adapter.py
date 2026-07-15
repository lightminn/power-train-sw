"""ROS-independent perception frame qualification and marker dedup core.

ROS adapters must convert ``DetectedObjectArray`` messages into the primitive
values below and inject a timestamped TF lookup.  This module owns neither a
ROS clock nor a TF buffer, which keeps frame, freshness, and marker identity
policy directly unit-testable.
"""

from dataclasses import dataclass
import math
from typing import Mapping, Sequence


BASE_FRAME_ID = "base_link"


@dataclass(frozen=True)
class DetectionHeader:
    frame_id: str
    stamp_s: float


@dataclass(frozen=True)
class DetectionPose:
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class DetectionBBox:
    x_offset: int
    y_offset: int
    width: int
    height: int
    do_rectify: bool = False


@dataclass(frozen=True)
class DetectedObjectValue:
    class_id: int
    class_name: str
    confidence: float
    pose: DetectionPose
    bbox: DetectionBBox


@dataclass(frozen=True)
class DetectedObjectArrayValue:
    header: DetectionHeader
    objects: Sequence[DetectedObjectValue]


@dataclass(frozen=True)
class DetectionAdapterConfig:
    timeout_s: float
    cluster_radius_m: float
    min_reobservation_s: float
    confidence_min: float
    unique_class_id_contract: bool = False


@dataclass(frozen=True)
class BaseLinkObservation:
    marker_key: str
    frame_id: str
    source_frame_id: str
    stamp_s: float
    class_id: int
    class_name: str
    confidence: float
    x: float
    y: float
    z: float
    yaw: float
    bbox: DetectionBBox


@dataclass(frozen=True)
class DetectionResult:
    observations: tuple = ()
    hold_reason: str = ""


@dataclass(frozen=True)
class MarkerAggregate:
    marker_key: str
    class_id: int
    class_name: str
    position: tuple
    first_seen_s: float
    last_seen_s: float
    accepted_observations: int


@dataclass(frozen=True)
class DedupState:
    total_markers: int
    markers: tuple
    counts_by_class: Mapping[str, int]


@dataclass
class _MarkerRecord:
    marker_key: str
    class_id: int
    class_name: str
    position: tuple
    first_seen_s: float
    last_seen_s: float
    accepted_observations: int = 1

    def snapshot(self):
        return MarkerAggregate(
            marker_key=self.marker_key,
            class_id=self.class_id,
            class_name=self.class_name,
            position=self.position,
            first_seen_s=self.first_seen_s,
            last_seen_s=self.last_seen_s,
            accepted_observations=self.accepted_observations,
        )


def _finite_float(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError("%s must be numeric" % label) from None
    if not math.isfinite(result):
        raise ValueError("%s must be finite" % label)
    return result


def _matrix(value, rows, columns, label):
    try:
        if len(value) != rows:
            raise ValueError
        result = tuple(
            tuple(_finite_float(value[row][column], label)
                  for column in range(columns))
            for row in range(rows)
        )
        if any(len(value[row]) != columns for row in range(rows)):
            raise ValueError
    except (TypeError, IndexError, ValueError):
        raise ValueError("%s has an invalid shape or value" % label) from None
    return result


def _transform_parts(value):
    """Normalize a homogeneous 4x4 or ``(rotation_3x3, translation_3)``."""
    try:
        length = len(value)
    except TypeError:
        raise ValueError("TF value is not a supported transform") from None

    if length == 4:
        homogeneous = _matrix(value, 4, 4, "4x4 TF")
        bottom = homogeneous[3]
        if not all(
            math.isclose(actual, expected, abs_tol=1e-9)
            for actual, expected in zip(bottom, (0.0, 0.0, 0.0, 1.0))
        ):
            raise ValueError("4x4 TF bottom row must be [0, 0, 0, 1]")
        rotation = tuple(row[:3] for row in homogeneous[:3])
        translation = tuple(row[3] for row in homogeneous[:3])
        return rotation, translation

    if length == 2:
        rotation = _matrix(value[0], 3, 3, "TF rotation")
        translation_matrix = _matrix((value[1],), 1, 3, "TF translation")
        return rotation, translation_matrix[0]

    raise ValueError("TF value is not a 4x4 or (R, t) transform")


def _rotate(rotation, vector):
    return tuple(
        sum(rotation[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )


def _transform_pose(rotation, translation, pose):
    source_position = (
        _finite_float(pose.x, "pose.x"),
        _finite_float(pose.y, "pose.y"),
        _finite_float(pose.z, "pose.z"),
    )
    yaw = _finite_float(pose.yaw, "pose.yaw")
    rotated_position = _rotate(rotation, source_position)
    position = tuple(
        rotated_position[index] + translation[index] for index in range(3)
    )

    source_heading = (math.cos(yaw), math.sin(yaw), 0.0)
    base_heading = _rotate(rotation, source_heading)
    if math.hypot(base_heading[0], base_heading[1]) <= 1e-12:
        raise ValueError("transformed yaw axis is vertical")
    base_yaw = math.atan2(base_heading[1], base_heading[0])
    return position, base_yaw


class DetectionAdapter:
    """Qualify timestamped frames, transform detections, and deduplicate markers."""

    def __init__(self, tf_lookup, config: DetectionAdapterConfig):
        if not callable(tf_lookup):
            raise ValueError("tf_lookup must be callable")
        self.tf_lookup = tf_lookup
        self.config = config
        self._validate_config()
        self._records = {}
        self._next_cluster_id = 1

    def _validate_config(self):
        timeout_s = _finite_float(self.config.timeout_s, "timeout_s")
        radius_m = _finite_float(
            self.config.cluster_radius_m,
            "cluster_radius_m",
        )
        minimum_s = _finite_float(
            self.config.min_reobservation_s,
            "min_reobservation_s",
        )
        confidence = _finite_float(
            self.config.confidence_min,
            "confidence_min",
        )
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        if radius_m < 0.0:
            raise ValueError("cluster_radius_m must be non-negative")
        if minimum_s < 0.0:
            raise ValueError("min_reobservation_s must be non-negative")
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence_min must be in [0, 1]")
        if not isinstance(self.config.unique_class_id_contract, bool):
            raise ValueError("unique_class_id_contract must be boolean")

        self.timeout_s = timeout_s
        self.cluster_radius_m = radius_m
        self.min_reobservation_s = minimum_s
        self.confidence_min = confidence

    @staticmethod
    def _hold(reason):
        return DetectionResult(observations=(), hold_reason=reason)

    def process(self, detection_array, *, now_s):
        """Return accepted ``base_link`` observations for one array value."""
        frame_id = detection_array.header.frame_id
        if not isinstance(frame_id, str) or not frame_id.strip():
            return self._hold("frame_id_empty")
        frame_id = frame_id.strip()

        try:
            stamp_s = _finite_float(detection_array.header.stamp_s, "stamp_s")
            now_s = _finite_float(now_s, "now_s")
        except ValueError:
            return self._hold("stamp_invalid")
        if stamp_s == 0.0:
            return self._hold("stamp_zero")
        if stamp_s < 0.0:
            return self._hold("stamp_invalid")
        age_s = now_s - stamp_s
        if age_s < 0.0:
            return self._hold("stamp_future")
        if age_s > self.timeout_s:
            return self._hold("stamp_stale")

        try:
            transform = self.tf_lookup(frame_id, stamp_s)
        except Exception:
            return self._hold("tf_lookup_failed")
        if transform is None:
            return self._hold("tf_unavailable")
        try:
            rotation, translation = _transform_parts(transform)
        except ValueError:
            return self._hold("tf_invalid")

        observations = []
        for detected in detection_array.objects:
            observation = self._adapt_object(
                detected,
                source_frame_id=frame_id,
                stamp_s=stamp_s,
                rotation=rotation,
                translation=translation,
            )
            if observation is not None:
                observations.append(observation)
        return DetectionResult(observations=tuple(observations), hold_reason="")

    def _adapt_object(
        self,
        detected,
        *,
        source_frame_id,
        stamp_s,
        rotation,
        translation,
    ):
        try:
            confidence = _finite_float(detected.confidence, "confidence")
            if not 0.0 <= confidence <= 1.0:
                return None
            if confidence < self.confidence_min:
                return None
            class_id = int(detected.class_id)
            class_name = str(detected.class_name).strip()
            if not class_name:
                return None
            position, yaw = _transform_pose(
                rotation,
                translation,
                detected.pose,
            )
        except (TypeError, ValueError):
            return None

        marker_key = self._accept_marker(
            class_id=class_id,
            class_name=class_name,
            position=position,
            stamp_s=stamp_s,
        )
        if marker_key is None:
            return None
        return BaseLinkObservation(
            marker_key=marker_key,
            frame_id=BASE_FRAME_ID,
            source_frame_id=source_frame_id,
            stamp_s=stamp_s,
            class_id=class_id,
            class_name=class_name,
            confidence=confidence,
            x=position[0],
            y=position[1],
            z=position[2],
            yaw=yaw,
            bbox=detected.bbox,
        )

    def _accept_marker(self, *, class_id, class_name, position, stamp_s):
        record = self._matching_record(class_id, class_name, position)
        if record is not None:
            elapsed_s = stamp_s - record.last_seen_s
            if elapsed_s < self.min_reobservation_s:
                return None
            record.last_seen_s = stamp_s
            record.accepted_observations += 1
            return record.marker_key

        if self.config.unique_class_id_contract:
            marker_key = "class_id:%d" % class_id
        else:
            marker_key = "class_name:%s:%d" % (
                class_name,
                self._next_cluster_id,
            )
            self._next_cluster_id += 1
        self._records[marker_key] = _MarkerRecord(
            marker_key=marker_key,
            class_id=class_id,
            class_name=class_name,
            position=position,
            first_seen_s=stamp_s,
            last_seen_s=stamp_s,
        )
        return marker_key

    def _matching_record(self, class_id, class_name, position):
        if self.config.unique_class_id_contract:
            return self._records.get("class_id:%d" % class_id)

        radius_squared = self.cluster_radius_m * self.cluster_radius_m
        nearest = None
        nearest_squared = None
        for record in self._records.values():
            if record.class_name != class_name:
                continue
            distance_squared = sum(
                (position[index] - record.position[index]) ** 2
                for index in range(3)
            )
            if distance_squared > radius_squared:
                continue
            if nearest_squared is None or distance_squared < nearest_squared:
                nearest = record
                nearest_squared = distance_squared
        return nearest

    def dedup_state(self):
        """Return a detached aggregate for three-segment/five-type counting."""
        markers = tuple(record.snapshot() for record in self._records.values())
        counts = {}
        for marker in markers:
            counts[marker.class_name] = counts.get(marker.class_name, 0) + 1
        return DedupState(
            total_markers=len(markers),
            markers=markers,
            counts_by_class=dict(sorted(counts.items())),
        )
