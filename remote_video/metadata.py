"""Pure validation and latest-only tracking for D435i detection metadata."""

from dataclasses import dataclass
import json
import math
from numbers import Real

from .contract import MAX_METADATA_BYTES, METADATA_SCHEMA_VERSION


_PACKET_FIELDS = {
    "schema_version",
    "session_id",
    "sequence",
    "source_frame_sequence",
    "capture_stamp_ns",
    "detections",
}
_DETECTION_FIELDS = {"bbox", "class_name", "class_id", "confidence"}


class MetadataError(ValueError):
    """Raised when D435i metadata violates the v1 receiver contract."""


@dataclass(frozen=True)
class Detection:
    bbox: tuple[float, float, float, float]
    class_name: str
    class_id: int
    confidence: float


@dataclass(frozen=True)
class MetadataPacket:
    schema_version: int
    session_id: str
    sequence: int
    source_frame_sequence: int
    # Correlation/logging only. Never compare this sender clock with the
    # notebook clock to decide freshness.
    capture_stamp_ns: int
    detections: tuple[Detection, ...]


def _integer(value, name: str, *, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise MetadataError(f"{name} must be an integer >= {minimum}")
    return value


def _finite(value, name: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, Real)
        or not math.isfinite(value)
    ):
        raise MetadataError(f"{name} must be a finite number")
    return float(value)


def _parse_detection(value, index: int) -> Detection:
    label = f"detections[{index}]"
    if not isinstance(value, dict) or set(value) != _DETECTION_FIELDS:
        raise MetadataError(f"{label} has invalid fields")
    bbox_value = value["bbox"]
    if not isinstance(bbox_value, (list, tuple)) or len(bbox_value) != 4:
        raise MetadataError(f"{label}.bbox must contain four coordinates")
    bbox = tuple(
        _finite(coordinate, f"{label}.bbox") for coordinate in bbox_value
    )
    x1, y1, x2, y2 = bbox
    if x1 >= x2 or y1 >= y2:
        raise MetadataError(f"{label}.bbox must not be inverted or empty")
    class_name = value["class_name"]
    if not isinstance(class_name, str) or not class_name:
        raise MetadataError(f"{label}.class_name must be a non-empty string")
    class_id = _integer(value["class_id"], f"{label}.class_id")
    confidence = _finite(value["confidence"], f"{label}.confidence")
    if not 0.0 <= confidence <= 1.0:
        raise MetadataError(f"{label}.confidence must be in [0, 1]")
    return Detection(
        bbox=(x1, y1, x2, y2),
        class_name=class_name,
        class_id=class_id,
        confidence=confidence,
    )


def parse_metadata(
    data: bytes, *, now_monotonic_ns: int
) -> tuple[MetadataPacket, int]:
    """Parse bounded v1 JSON and return it with local monotonic receive time."""

    if not isinstance(data, bytes) or not data or len(data) > MAX_METADATA_BYTES:
        raise MetadataError("metadata exceeds size limit or is empty")
    received_ns = _integer(now_monotonic_ns, "now_monotonic_ns")
    try:
        message = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise MetadataError("invalid JSON") from exc
    if not isinstance(message, dict):
        raise MetadataError("metadata must be a JSON object")
    if set(message) != _PACKET_FIELDS:
        raise MetadataError("metadata packet has invalid fields")
    if (
        isinstance(message["schema_version"], bool)
        or message["schema_version"] != METADATA_SCHEMA_VERSION
    ):
        raise MetadataError("unsupported schema version")
    session_id = message["session_id"]
    if not isinstance(session_id, str) or not session_id:
        raise MetadataError("session_id must be a non-empty string")
    detection_values = message["detections"]
    if not isinstance(detection_values, list):
        raise MetadataError("detections must be a JSON array")
    detections = tuple(
        _parse_detection(value, index)
        for index, value in enumerate(detection_values)
    )
    packet = MetadataPacket(
        schema_version=METADATA_SCHEMA_VERSION,
        session_id=session_id,
        sequence=_integer(message["sequence"], "sequence"),
        source_frame_sequence=_integer(
            message["source_frame_sequence"], "source_frame_sequence"
        ),
        capture_stamp_ns=_integer(
            message["capture_stamp_ns"], "capture_stamp_ns"
        ),
        detections=detections,
    )
    return packet, received_ns


class MetadataTracker:
    """Retain one latest packet and hide only stale overlay metadata."""

    def __init__(self, ttl_s: float = 0.5):
        if (
            isinstance(ttl_s, bool)
            or not isinstance(ttl_s, Real)
            or not math.isfinite(ttl_s)
            or ttl_s <= 0
        ):
            raise ValueError("ttl_s must be a finite positive number")
        self._ttl_ns = int(float(ttl_s) * 1_000_000_000)
        self._latest: MetadataPacket | None = None
        self._received_monotonic_ns: int | None = None

    @property
    def latest(self) -> MetadataPacket | None:
        return self._latest

    def update(
        self, packet: MetadataPacket, *, received_monotonic_ns: int
    ) -> bool:
        if not isinstance(packet, MetadataPacket):
            raise TypeError("packet must be MetadataPacket")
        received_ns = _integer(received_monotonic_ns, "received_monotonic_ns")
        previous = self._latest
        if (
            previous is not None
            and packet.session_id == previous.session_id
            and packet.sequence <= previous.sequence
        ):
            return False
        if (
            self._received_monotonic_ns is not None
            and received_ns < self._received_monotonic_ns
        ):
            raise ValueError("received_monotonic_ns must not go backwards")
        self._latest = packet
        self._received_monotonic_ns = received_ns
        return True

    def overlay_state(self, now_monotonic_ns: int) -> str:
        """Return overlay freshness without blocking or degrading raw video."""

        now_ns = _integer(now_monotonic_ns, "now_monotonic_ns")
        if self._latest is None or self._received_monotonic_ns is None:
            return "OVERLAY_STALE"
        if now_ns < self._received_monotonic_ns:
            raise ValueError("now_monotonic_ns precedes the latest local receive time")
        # Freshness is based only on the receiver's local monotonic TTL.
        if now_ns - self._received_monotonic_ns > self._ttl_ns:
            return "OVERLAY_STALE"
        return "FRESH"
