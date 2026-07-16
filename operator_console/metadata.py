"""Versioned D435 YOLO metadata validation and latest-only UDP reception."""
from __future__ import annotations

from dataclasses import dataclass
import json
import socket
import threading
import time
from typing import Any

from .udp_source import SourceSequenceGate


@dataclass(frozen=True)
class Detection:
    class_name: str
    confidence: float
    bbox_xywh: tuple[int, int, int, int]
    position_m: tuple[float, float, float] | None


@dataclass(frozen=True)
class MetadataFrame:
    sequence: int
    width: int
    height: int
    detections: tuple[Detection, ...]
    received_monotonic_s: float


def parse_metadata(raw: bytes, received_monotonic_s: float | None = None) -> MetadataFrame:
    """Validate the small v1 JSON datagram; reject malformed sender input."""
    if len(raw) > 2048:
        raise ValueError("oversize metadata")
    payload: dict[str, Any] = json.loads(raw.decode("utf-8"))
    if payload.get("schema_version") != 1:
        raise ValueError("unsupported schema")
    width, height = int(payload["frame_width"]), int(payload["frame_height"])
    if width < 1 or height < 1:
        raise ValueError("invalid frame dimensions")
    detections: list[Detection] = []
    for item in payload.get("detections", []):
        box = tuple(int(value) for value in item["bbox_xywh"])
        if len(box) != 4 or box[2] < 1 or box[3] < 1:
            raise ValueError("invalid bbox")
        xyz = item.get("position_m")
        position = None if xyz is None else tuple(float(value) for value in xyz)
        if position is not None and len(position) != 3:
            raise ValueError("invalid position")
        detections.append(Detection(str(item["class_name"]), float(item["confidence"]), box, position))
    return MetadataFrame(
        sequence=int(payload["capture_sequence"]), width=width, height=height,
        detections=tuple(detections),
        received_monotonic_s=time.monotonic() if received_monotonic_s is None else received_monotonic_s,
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
        self._thread = threading.Thread(target=self._run, name="d435-metadata", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        self._socket.settimeout(0.2)
        while not self._stopping.is_set():
            try:
                raw, address = self._socket.recvfrom(4096)
                received_s = time.monotonic()
                frame = parse_metadata(raw, received_monotonic_s=received_s)
            except (OSError, ValueError, UnicodeDecodeError, json.JSONDecodeError):
                continue
            if not self._source_gate.accept(
                address,
                frame.sequence,
                now_s=received_s,
            ):
                continue
            with self._lock:
                self._latest = frame

    def latest(self) -> MetadataFrame | None:
        with self._lock:
            return self._latest

    def close(self) -> None:
        self._stopping.set()
        self._socket.close()
        self._thread.join(timeout=1.0)
