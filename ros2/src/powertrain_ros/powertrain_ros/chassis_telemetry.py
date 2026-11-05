"""ROS-free encoding and polling helpers for chassis console telemetry."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import threading
import time
from collections.abc import Callable, Mapping


MAX_TELEMETRY_BYTES = 4096
MAX_TEXT_CHARS = 256
_UNSHRINKABLE_TEXT = frozenset({"suspension_source", "suspension_frame"})


def parse_component_mask_state(raw: str) -> dict[str, bool]:
    """Extract a strict bool-valued component mask from safety-state JSON."""
    payload = json.loads(raw)
    if not isinstance(payload, dict):
        raise ValueError("safety state must be an object")
    raw_mask = payload.get("component_mask")
    if not isinstance(raw_mask, dict):
        raise ValueError("component_mask must be an object")
    component_mask: dict[str, bool] = {}
    for component, enabled in raw_mask.items():
        if not isinstance(component, str) or not isinstance(enabled, bool):
            raise ValueError("component_mask values must be bool")
        component_mask[component] = enabled
    return component_mask


def component_mask_payload_value(
    component_mask: Mapping[str, bool] | None,
    *,
    updated_s: float | None,
    now_s: float,
) -> dict[str, bool] | None:
    """Mirror a defensive copy only while the safety-state source is fresh."""
    if updated_s is None or component_mask is None or now_s - updated_s > 1.0:
        return None
    return dict(component_mask)


@dataclass(frozen=True)
class PollCache:
    value: object | None
    error: str | None
    updated_monotonic_s: float | None


class LatestPollWorker:
    """Run one possibly blocking poll in a daemon and expose only its latest result."""

    def __init__(
        self,
        poll: Callable[[], object | None],
        *,
        error: Callable[[], str | None] | None = None,
        period_s: float = 1.0,
        name: str = "telemetry-poll",
    ) -> None:
        period_s = float(period_s)
        if not math.isfinite(period_s) or period_s <= 0.0:
            raise ValueError("period_s must be finite and positive")
        self._poll = poll
        self._error = error
        self._period_s = period_s
        self._cache = PollCache(None, "not polled", None)
        self._lock = threading.Lock()
        self._stopping = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name=name,
            daemon=True,
        )
        self._thread.start()

    def _run(self) -> None:
        while not self._stopping.is_set():
            started_s = time.monotonic()
            try:
                value = self._poll()
                error = None
                if value is None:
                    error = self._error() if self._error is not None else None
                    error = error or "no response"
            except Exception as exc:
                value = None
                error = f"{type(exc).__name__}: {exc}"
            cache = PollCache(value, error, time.monotonic())
            with self._lock:
                self._cache = cache
            remaining_s = self._period_s - (time.monotonic() - started_s)
            if self._stopping.wait(max(0.0, remaining_s)):
                return

    def latest(self) -> PollCache:
        with self._lock:
            return self._cache

    def close(self, *, join_timeout_s: float = 1.0) -> bool:
        self._stopping.set()
        self._thread.join(timeout=join_timeout_s)
        return not self._thread.is_alive()


def _thawed(value: object) -> object:
    # GatewayClient/ObservabilityClient hand out read-only MappingProxyType
    # payloads; json.dumps only accepts plain containers.
    if isinstance(value, Mapping):
        return {key: _thawed(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_thawed(item) for item in value]
    return value


def _encoded(payload: dict[str, object]) -> bytes:
    return json.dumps(_thawed(payload), separators=(",", ":")).encode("utf-8")


def _bounded_top_level_strings(payload: dict[str, object]) -> dict[str, object]:
    return {
        key: value[:MAX_TEXT_CHARS] if isinstance(value, str) else value
        for key, value in payload.items()
    }


def encode_telemetry_payload(payload: dict[str, object]) -> bytes:
    """Encode a bounded datagram, retaining a summary if wheel rows overflow."""
    bounded = _bounded_top_level_strings(dict(payload))
    raw = _encoded(bounded)
    if len(raw) <= MAX_TELEMETRY_BYTES:
        return raw

    bounded.pop("wheel_statuses", None)
    bounded["truncated"] = True
    raw = _encoded(bounded)
    if len(raw) <= MAX_TELEMETRY_BYTES:
        return raw

    # Defensive final reduction for an unexpectedly large structured field.
    # Keep the console populated with the scalar operational summary.
    summary_keys = (
        "schema_version", "sequence", "odometry_source", "x_m", "y_m",
        "yaw_rad", "roll_rad", "pitch_rad", "drive_state", "can_state",
        "l515_state", "l515_detail",
        "l515_mode", "safety_status", "safety_distance_mm",
        "safety_estop_required", "safety_consecutive_failures",
        "safety_detail", "component_mask", "wheel_count", "wheel_fault_count",
        "wheel_stale_count", "wheel_axis_error_count",
        "wheel_steer_fault_count", "terrain_path_available",
        "terrain_path_offset_m", "terrain_heading_error_rad",
        "terrain_support_m", "terrain_bank_rad", "terrain_slope_rad",
        "terrain_roughness_m", "terrain_confidence",
        "terrain_reject_reasons", "controller_fsm_state",
        "controller_fsm_reasons", "degradation_state",
        "degradation_reasons", "degradation_speed_scale",
        "mission_fsm_state", "mission_fsm_reason", "section_fsm_section",
        "section_fsm_phase", "section_fsm_notices", "slip_candidate",
        "stuck_candidate", "suspension_joints_rad", "suspension_age_ms",
        "suspension_source", "suspension_frame",
    )
    summary = {key: bounded[key] for key in summary_keys if key in bounded}
    summary["truncated"] = True
    raw = _encoded(summary)
    while len(raw) > MAX_TELEMETRY_BYTES:
        text_fields = [
            (len(_encoded({key: value})), key, value)
            for key, value in summary.items()
            # Console rejects empty suspension label strings; they are <=80 chars.
            if isinstance(value, str) and value and key not in _UNSHRINKABLE_TEXT
        ]
        if not text_fields:
            break
        _encoded_size, key, value = max(text_fields)
        summary[key] = value[: len(value) // 2]
        raw = _encoded(summary)
    if len(raw) <= MAX_TELEMETRY_BYTES:
        return raw
    return _encoded({
        "schema_version": bounded.get("schema_version", 1),
        "sequence": bounded.get("sequence", 0),
        "truncated": True,
    })


class SuspensionTelemetry:
    """Optional AS5048B rocker/bogie angles for the chassis datagram.

    The blocking I2C reads run in a LatestPollWorker daemon thread; the send
    timer only reads the cached latest sample.  Any import/config problem
    disables the fields (never fake zeros) without stopping the sender.
    """

    def __init__(self, config_path: str | None = None, *, reader_factory=None) -> None:
        self.status = "disabled"
        self._worker: LatestPollWorker | None = None
        self._reader = None
        self._config = None
        try:
            from suspension_encoders.config import load_config
            from suspension_encoders.reader import (
                SuspensionEncoderReader,
                telemetry_fields,
            )
            config = load_config(config_path or None)
            if not config.enabled:
                self.status = f"disabled in {config.source_path}"
                return
            factory = reader_factory or SuspensionEncoderReader
            reader = factory(config)
            try:
                worker = LatestPollWorker(
                    reader.read_once,
                    period_s=1.0 / config.read_hz,
                    name="suspension-encoder-poll",
                )
            except BaseException:
                reader.close()
                raise
        except Exception as exc:  # optional sensor: report, never stop the sender
            self.status = f"unavailable: {type(exc).__name__}: {exc}"
            return
        self._telemetry_fields = telemetry_fields
        self._config = config
        self._reader = reader
        self._worker = worker
        self.status = (
            f"polling /dev/i2c-{config.i2c_bus} at {config.read_hz:g} Hz "
            f"({config.source_path})"
        )

    @property
    def active(self) -> bool:
        return self._worker is not None

    def latest_errors(self) -> list[str]:
        """Per-channel or whole-poll errors of the latest poll, for logging."""
        if self._worker is None:
            return []
        cached = self._worker.latest()
        if cached.updated_monotonic_s is None:
            return []  # first poll still pending
        if cached.value is None:
            return [cached.error or "no sample"]
        return [
            f"{reading.channel.joint}: {reading.error}"
            for reading in cached.value.readings
            if reading.error is not None
        ]

    def fields(self, now_s: float | None = None) -> dict[str, object]:
        """Fields for this datagram; {} on any problem (never breaks the send).

        The cache is read BEFORE the clock so a sample published by the
        worker in between can never get a negative age.
        """
        if self._worker is None:
            return {}
        cached = self._worker.latest()
        if now_s is None:
            now_s = time.monotonic()
        try:
            return self._telemetry_fields(
                cached.value,
                now_s=now_s,
                max_age_s=self._config.max_age_s,
                calibrated_frame=self._config.calibrated_frame,
                publish_unverified=self._config.publish_unverified,
            )
        except Exception as exc:
            self.status = f"field error: {type(exc).__name__}: {exc}"
            return {}

    def close(self, *, join_timeout_s: float = 1.0) -> bool:
        if self._worker is None:
            return True
        stopped = self._worker.close(join_timeout_s=join_timeout_s)
        if stopped:
            self._reader.close()
        return stopped
