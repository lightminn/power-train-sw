"""L515 terrain overlay reception: session gate, display delay, view decision.

Spec §4.6 (docs/specs/2026-10-17-l515-terrain-overlay-design.md).  Pure
Python — no GTK — so the decisions are testable without a display; the GTK
``TerrainCanvas`` in app.py only draws what ``terrain_view`` returns.
Operator visual aid only: nothing here feeds control or safety.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import socket
import threading
import time

from terrain_overlay.contract import (
    MAX_PACKET_BYTES, TerrainFrame, TerrainObstacle, parse_packet,
)

BOOT_SWITCH_SILENCE_S = 1.0
LINK_STALE_S = 0.5
DEPTH_STALE_S = 0.6
MAX_DISPLAY_DELAY_S = 0.4
BUFFER_MARGIN_S = 0.5
REFERENCE_NOTE = "참고용 · 낮은 장애물/구덩이 미표시"


class TerrainSessionGate:
    """Order senders by session, never by address (spec §4.6 세션 게이트)."""

    def __init__(self) -> None:
        self._session: tuple[str, int, str] | None = None
        self._last_sequence: int | None = None
        self._last_accepted_s: float | None = None

    def accept(self, frame: TerrainFrame, now_s: float) -> tuple[bool, bool]:
        """Return ``(accepted, session_changed)``."""
        key = frame.session_key
        current = self._session
        if current is None:
            changed = True
        elif key == current:
            if frame.sequence <= self._last_sequence:
                return False, False
            changed = False
        elif key[0] == current[0]:
            if key[1:] <= current[1:]:
                return False, False  # an older run of this boot, forever
            changed = True
        else:
            if now_s - self._last_accepted_s <= BOOT_SWITCH_SILENCE_S:
                return False, False
            changed = True
        self._session = key
        self._last_sequence = frame.sequence
        self._last_accepted_s = now_s
        return True, changed


def validate_display_delay_s(value: float) -> float:
    value = float(value)
    if not math.isfinite(value) or not 0.0 <= value <= MAX_DISPLAY_DELAY_S:
        raise ValueError("terrain display delay must be within 0-400 ms")
    return value


class LatestTerrainReceiver:
    """Daemon UDP receiver keeping a short, delay-aligned frame buffer."""

    def __init__(self, port: int, display_delay_s: float = 0.0) -> None:
        self.display_delay_s = validate_display_delay_s(display_delay_s)
        self._gate = TerrainSessionGate()
        self._buffer: deque[TerrainFrame] = deque()
        self._last_rx_s: float | None = None
        self._lock = threading.Lock()
        self._invalid_packet_count = 0
        self._rejected_packet_count = 0
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind(("0.0.0.0", port))
        self._stopping = threading.Event()
        self._thread = threading.Thread(target=self._run, name="l515-terrain", daemon=True)
        self._thread.start()

    @property
    def port(self) -> int:
        return self._socket.getsockname()[1]

    def _run(self) -> None:
        self._socket.settimeout(0.2)
        while not self._stopping.is_set():
            try:
                raw, _address = self._socket.recvfrom(MAX_PACKET_BYTES + 1)
            except OSError:
                continue
            try:
                self.ingest(raw, time.monotonic())
            except Exception:  # noqa: BLE001 — never let one datagram kill reception
                self._invalid_packet_count += 1

    def ingest(self, raw: bytes, received_s: float) -> bool:
        try:
            frame = parse_packet(raw, received_monotonic_s=received_s)
        except ValueError:
            self._invalid_packet_count += 1
            return False
        with self._lock:
            accepted, changed = self._gate.accept(frame, received_s)
            if not accepted:
                self._rejected_packet_count += 1
                return False
            if changed:
                self._buffer.clear()
            self._buffer.append(frame)
            self._last_rx_s = received_s
            horizon = received_s - (self.display_delay_s + BUFFER_MARGIN_S)
            while len(self._buffer) > 1 and self._buffer[0].received_monotonic_s < horizon:
                self._buffer.popleft()
        return True

    @property
    def invalid_packet_count(self) -> int:
        return self._invalid_packet_count

    @property
    def rejected_packet_count(self) -> int:
        return self._rejected_packet_count

    @property
    def last_rx_monotonic(self) -> float | None:
        with self._lock:
            return self._last_rx_s

    def latest(self) -> TerrainFrame | None:
        with self._lock:
            return self._buffer[-1] if self._buffer else None

    def display_frame(self, now_s: float) -> TerrainFrame | None:
        return self.snapshot(now_s)[1]

    def snapshot(self, now_s: float):
        """``(latest, display_frame, last_rx)`` read under one lock.

        Separate reads can straddle a session switch and pair an old frame
        with the new session's receive time (Codex 2026-10-17 interleaving).
        """
        cutoff = now_s - self.display_delay_s
        with self._lock:
            display = next(
                (f for f in reversed(self._buffer) if f.received_monotonic_s <= cutoff),
                None,
            )
            latest = self._buffer[-1] if self._buffer else None
            return latest, display, self._last_rx_s

    def close(self) -> None:
        self._stopping.set()
        self._socket.close()
        self._thread.join(timeout=1.0)


@dataclass(frozen=True)
class TerrainView:
    badge: str
    tone: str  # muted / ok / warn / error
    note: str
    obstacles: tuple[TerrainObstacle, ...]
    probe_state: str  # WAITING / LIVE / STALE (runtime_smoke)


def _basis(frame: TerrainFrame) -> str:
    return "차체 앞 기준" if frame.distance_origin == "rover_front" else "카메라 기준"


def _note(frame: TerrainFrame | None) -> str:
    if frame is not None and not frame.alignment_verified:
        return f"정합 미검증 · {REFERENCE_NOTE}"
    return REFERENCE_NOTE


def terrain_view(
    latest: TerrainFrame | None, display: TerrainFrame | None,
    last_rx_s: float | None, now_s: float, display_delay_s: float,
    video_size: tuple[int, int] | None,
    *, video_live: bool = True,
) -> TerrainView:
    if last_rx_s is None or latest is None:
        return TerrainView("지형: 수신 없음", "muted", REFERENCE_NOTE, (), "WAITING")
    link_age = now_s - last_rx_s
    if link_age > LINK_STALE_S:
        return TerrainView("지형: 수신 없음", "muted", REFERENCE_NOTE, (), "STALE")
    shown = display or latest
    aged = max(0.0, now_s - display_delay_s - shown.received_monotonic_s)
    shown_depth_age = (None if shown.depth_age_ms is None
                       else shown.depth_age_ms / 1000.0 + aged)
    if shown.status == "OK" and (shown_depth_age is None or shown_depth_age > DEPTH_STALE_S):
        # An OK result from depth older than the outline limit is not a current
        # reading: withdraw the distance / "no obstacle" summary too.
        age_text = "" if shown_depth_age is None else f" {shown_depth_age:.1f} s"
        badge, tone = f"지형: 깊이 지연{age_text}", "warn"
    elif shown.status == "OK":
        if shown.ground.state == "HELD":
            held_s = shown.ground.age_ms / 1000.0 + aged
            badge, tone = f"지형: 바닥 유지 {held_s:.1f} s", "warn"
        else:
            center = shown.zones["center"]
            path = "경로 돌출 장애물 없음" if center is None else f"경로 {center:.2f} m"
            badge, tone = f"지형 OK · {path} ({_basis(shown)})", "ok"
    elif shown.status == "NO_GROUND":
        badge, tone = "지형: 바닥 미검출", "warn"
    else:
        badge, tone = f"지형: 센서 오류({shown.reason})", "error"
    obstacles: tuple[TerrainObstacle, ...] = ()
    note = _note(shown)
    fresh_ok = (
        display is not None
        and display.status == "OK"
        and display.depth_age_ms is not None
        and display.depth_age_ms / 1000.0 + aged <= DEPTH_STALE_S
    )
    if fresh_ok:
        if not video_live:
            note = f"윤곽 숨김: 영상 끊김 · {note}"
        elif video_size is None:
            note = f"윤곽 숨김: 영상 크기 미확인 · {note}"
        elif tuple(video_size) != display.frame_size:
            note = f"윤곽 숨김: 영상 {video_size[0]}×{video_size[1]} ≠ 1280×720 · {note}"
        else:
            obstacles = display.obstacles
    return TerrainView(badge, tone, note, obstacles, "LIVE")
