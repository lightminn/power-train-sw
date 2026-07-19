"""Pure validation and JSON encoding for the arm-to-console UDP mirror."""
from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import json
import math
from typing import Any


DYNAMIXEL_FIELDS_PER_MOTOR = 5
MAX_MOTORS = 8
MAX_TELEMETRY_BYTES = 4096
MAX_METADATA_BYTES = 2048
MAX_JOINTS = 16

_INT32_MIN = -(2**31)
_INT32_MAX = 2**31 - 1
_SOURCE_NAMES = ("dynamixel", "joints", "detections")


@dataclass(frozen=True)
class DynamixelMotor:
    id: int
    position_raw: int
    position_deg: float
    velocity: int
    current: int
    temperature_c: int


def position_raw_to_deg(raw: int) -> float:
    return (raw - 2048) * 360.0 / 4096.0


def parse_dynamixel_state(data) -> tuple[DynamixelMotor, ...] | None:
    """Parse one flat Int32MultiArray snapshot, rejecting it as a unit."""
    try:
        values = tuple(data)
    except TypeError:
        return None
    if (
        not values
        or len(values) % DYNAMIXEL_FIELDS_PER_MOTOR != 0
        or len(values) // DYNAMIXEL_FIELDS_PER_MOTOR > MAX_MOTORS
    ):
        return None
    if any(
        isinstance(value, bool) or not isinstance(value, int)
        for value in values
    ):
        return None

    motors = []
    for offset in range(0, len(values), DYNAMIXEL_FIELDS_PER_MOTOR):
        motor_id, position_raw, velocity, current, temperature_c = values[
            offset:offset + DYNAMIXEL_FIELDS_PER_MOTOR
        ]
        if not 0 <= motor_id <= 252:
            return None
        if not 0 <= position_raw <= 4095:
            return None
        if not _INT32_MIN <= velocity <= _INT32_MAX:
            return None
        if not _INT32_MIN <= current <= _INT32_MAX:
            return None
        if not 0 <= temperature_c <= 150:
            return None
        motors.append(
            DynamixelMotor(
                id=motor_id,
                position_raw=position_raw,
                position_deg=position_raw_to_deg(position_raw),
                velocity=velocity,
                current=current,
                temperature_c=temperature_c,
            )
        )
    return tuple(motors)


def yaw_from_quaternion(z: float, w: float) -> float:
    """Recover planar yaw and normalize it to the interval (-pi, pi]."""
    yaw = 2.0 * math.atan2(z, w)
    normalized = (yaw + math.pi) % (2.0 * math.pi) - math.pi
    return normalized + 2.0 * math.pi if normalized <= -math.pi else normalized


def _encode(payload: Mapping[str, Any]) -> bytes:
    return json.dumps(
        payload,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")


def _motor_payload(motor: DynamixelMotor) -> dict[str, int | float]:
    return {
        "id": motor.id,
        "position_raw": motor.position_raw,
        "position_deg": motor.position_deg,
        "velocity": motor.velocity,
        "current": motor.current,
        "temperature_c": motor.temperature_c,
    }


def _joint_payload(
    joints: Mapping[str, Sequence[Any]],
) -> tuple[dict[str, list[Any]], bool]:
    try:
        names = list(joints["names"])
        position = list(joints["position_rad"])
        velocity = list(joints["velocity"])
    except (KeyError, TypeError) as exc:
        raise ValueError("joint arrays are invalid") from exc
    if len(names) != len(position) or len(names) != len(velocity):
        raise ValueError("joint array lengths do not match")

    truncated = len(names) > MAX_JOINTS
    position_values = [float(value) for value in position[:MAX_JOINTS]]
    velocity_values = [float(value) for value in velocity[:MAX_JOINTS]]
    # NaN 은 여기서 잡는다 — 전체 payload _encode(allow_nan=False) 단계까지
    # 가면 dynamixel 온도까지 함께 버려진다.
    if not all(
        math.isfinite(value)
        for value in position_values + velocity_values
    ):
        raise ValueError("joint values must be finite")
    return {
        "names": [str(name) for name in names[:MAX_JOINTS]],
        "position_rad": position_values,
        "velocity": velocity_values,
    }, truncated


def build_arm_telemetry_payload(
    *,
    sequence,
    stamp_s,
    motors,
    joints,
    source_age_s,
) -> bytes:
    """Encode one bounded telemetry snapshot for UDP :5007."""
    joint_payload = None
    truncated = False
    if joints is not None:
        try:
            joint_payload, truncated = _joint_payload(joints)
        except ValueError:
            # 합법 JointState 변형(velocity 생략)이나 NaN 이 datagram 전체
            # (모터 온도 포함)를 침묵시키지 않도록 joints 만 강등한다
            # (2026-07-19 콘솔 E2E 리뷰 A#2).
            joint_payload = None
            truncated = False
    payload = {
        "schema_version": 1,
        "sequence": int(sequence),
        # Correlation/logging only; receivers must use local arrival freshness.
        "stamp_s": float(stamp_s),
        "dynamixel": (
            None
            if motors is None
            else [_motor_payload(motor) for motor in motors]
        ),
        "joints": joint_payload,
        "source_age_s": {
            name: source_age_s.get(name) for name in _SOURCE_NAMES
        },
        "truncated": truncated,
    }
    encoded = _encode(payload)
    if len(encoded) <= MAX_TELEMETRY_BYTES:
        return encoded

    payload["joints"] = None
    payload["truncated"] = True
    encoded = _encode(payload)
    if len(encoded) > MAX_TELEMETRY_BYTES:
        raise ValueError("arm telemetry exceeds 4096 bytes")
    return encoded


def _metadata_detection(
    detection,
    pick_target,
) -> dict[str, Any] | None:
    (
        class_id,
        class_name,
        confidence,
        bbox_xywh,
        position_m,
        yaw_rad,
    ) = detection
    confidence = float(confidence)
    yaw_rad = float(yaw_rad)
    if not math.isfinite(confidence) or not math.isfinite(yaw_rad):
        return None
    bbox = tuple(int(value) for value in bbox_xywh)
    if len(bbox) != 4:
        raise ValueError("detection bbox must contain four values")

    position = None
    if position_m is not None:
        position_values = tuple(float(value) for value in position_m)
        if len(position_values) != 3:
            raise ValueError("detection position must contain three values")
        if not all(math.isfinite(value) for value in position_values):
            return None
        if position_values[2] > 0.0:
            position = list(position_values)

    class_id = int(class_id)
    # The latched pick target is best effort.  Exact matching prevents an old
    # target from marking a new detection after the arm stack has moved on.
    is_pick_target = (
        pick_target is not None
        and class_id == int(pick_target[0])
        and bbox == tuple(pick_target[1])
    )
    return {
        "class_id": class_id,
        "class_name": str(class_name),
        "confidence": confidence,
        "bbox_xywh": list(bbox),
        "position_m": position,
        "yaw_rad": yaw_rad,
        "is_pick_target": is_pick_target,
    }


def build_detection_metadata_payload(
    *,
    capture_stamp_ns,
    frame_id,
    frame_width,
    frame_height,
    detections,
    pick_target,
    capture_sequence=None,
) -> bytes:
    """Encode the arm metadata schema superset accepted by the console.

    ``capture_sequence`` 는 콘솔 SourceSequenceGate 가 단조 증가를 요구한다.
    header.stamp 미설정(0) 스택에서 stamp 겸용은 첫 프레임 이후 전부
    기각되므로, 브리지는 자체 카운터를 명시로 넘긴다 (리뷰 A#6).
    """
    encoded_detections = []
    for detection in detections:
        encoded = _metadata_detection(detection, pick_target)
        if encoded is not None:
            encoded_detections.append(encoded)
    payload = {
        "schema_version": 1,
        "capture_sequence": int(
            capture_stamp_ns if capture_sequence is None else capture_sequence
        ),
        "capture_stamp_ns": int(capture_stamp_ns),
        "frame_width": int(frame_width),
        "frame_height": int(frame_height),
        "frame_id": str(frame_id),
        "detections": encoded_detections,
    }
    encoded = _encode(payload)
    while len(encoded) > MAX_METADATA_BYTES and encoded_detections:
        lowest = min(
            range(len(encoded_detections)),
            key=lambda index: encoded_detections[index]["confidence"],
        )
        encoded_detections.pop(lowest)
        encoded = _encode(payload)
    if len(encoded) > MAX_METADATA_BYTES:
        raise ValueError("detection metadata exceeds 2048 bytes")
    return encoded
