"""RX-only telemetry observation; commands use only the gated ops channel."""
from __future__ import annotations

from dataclasses import dataclass
from collections.abc import Mapping
import json
import socket
import threading
import time
from typing import Any

from .udp_source import SourceSequenceGate


@dataclass(frozen=True)
class WheelStatus:
    """Read-only per-wheel summary mirrored from `/wheel_states`."""
    name: str
    mode: str
    drive_turns_per_s: float | None
    steer_deg: float | None
    stale: bool
    drive_axis_error: int
    steer_fault: int


@dataclass(frozen=True)
class TelemetrySnapshot:
    sequence: int
    odometry_source: str
    x_m: float | None
    y_m: float | None
    yaw_rad: float | None
    voltage_v: float | None
    current_a: float | None
    power_w: float | None
    drive_state: str
    can_state: str
    l515_state: str
    l515_detail: str
    l515_mode: str
    l515_color_hz: float | None
    l515_depth_hz: float | None
    l515_submitted_hz: float | None
    l515_sent_hz: float | None
    l515_drop_hz: float | None
    l515_ros_topic_rates_hz: tuple[tuple[str, float], ...]
    l515_aligned_depth_age_ms: float | None
    l515_process_cpu_percent: float | None
    l515_process_rss_bytes: int | None
    pdist_soc_percent: int | None
    pdist_battery_flags: int | None
    pdist_protection_flags: int | None
    pdist_charge_current_a: float | None
    rs485_state: str
    rs485_consecutive_failures: int | None
    rs485_detail: str
    unit_status: tuple[tuple[str, str], ...]
    compose_status: tuple[tuple[str, str], ...]
    journal_tail: tuple[str, ...]
    safety_status: str
    safety_distance_mm: float | None
    safety_estop_required: bool | None
    safety_consecutive_failures: int | None
    safety_detail: str
    component_mask: dict[str, bool] | None
    wheel_count: int | None
    wheel_fault_count: int | None
    wheel_stale_count: int | None
    wheel_axis_error_count: int | None
    wheel_steer_fault_count: int | None
    wheel_statuses: tuple[WheelStatus, ...]
    truncated: bool
    received_monotonic_s: float


def _optional_number(payload: dict[str, Any], name: str) -> float | None:
    value = payload.get(name)
    return None if value is None else float(value)


def _optional_int(payload: dict[str, Any], name: str) -> int | None:
    value = payload.get(name)
    return None if value is None else int(value)


def _wheel_statuses(payload: dict[str, Any]) -> tuple[WheelStatus, ...]:
    raw_statuses = payload.get("wheel_statuses", [])
    if not isinstance(raw_statuses, list) or len(raw_statuses) > 6:
        raise ValueError("invalid wheel_statuses")
    statuses: list[WheelStatus] = []
    for raw in raw_statuses:
        if not isinstance(raw, dict):
            raise ValueError("invalid wheel status")
        statuses.append(WheelStatus(
            name=str(raw["name"]), mode=str(raw["mode"]),
            drive_turns_per_s=_optional_number(raw, "drive_turns_per_s"),
            steer_deg=_optional_number(raw, "steer_deg"), stale=bool(raw.get("stale", False)),
            drive_axis_error=int(raw.get("drive_axis_error", 0)),
            steer_fault=int(raw.get("steer_fault", 0)),
        ))
    return tuple(statuses)


def _l515_ros_topic_rates(payload: dict[str, Any]) -> tuple[tuple[str, float], ...]:
    """Keep the Gateway's per-topic rates scalar and bounded for UDP v1."""
    raw_rates = payload.get("l515_ros_topic_rates_hz", {})
    if not isinstance(raw_rates, dict) or len(raw_rates) > 6:
        raise ValueError("invalid l515_ros_topic_rates_hz")
    return tuple(sorted((str(topic), float(rate)) for topic, rate in raw_rates.items()))


def _status_mapping(payload: dict[str, Any], name: str) -> tuple[tuple[str, str], ...]:
    raw_status = payload.get(name)
    if raw_status is None:
        return ()
    if not isinstance(raw_status, dict) or len(raw_status) > 16:
        raise ValueError(f"invalid {name}")
    return tuple(sorted((str(key), str(value)) for key, value in raw_status.items()))


def _journal_tail(payload: dict[str, Any]) -> tuple[str, ...]:
    raw_tail = payload.get("journal_tail")
    if raw_tail is None:
        return ()
    if not isinstance(raw_tail, list) or len(raw_tail) > 100:
        raise ValueError("invalid journal_tail")
    return tuple(str(line) for line in raw_tail)


def _optional_component_mask(payload: dict[str, Any]) -> dict[str, bool] | None:
    raw_mask = payload.get("component_mask")
    if raw_mask is None:
        return None
    if not isinstance(raw_mask, dict):
        raise ValueError("invalid component_mask")
    component_mask: dict[str, bool] = {}
    for component, enabled in raw_mask.items():
        if not isinstance(component, str) or not isinstance(enabled, bool):
            raise ValueError("invalid component_mask")
        component_mask[component] = enabled
    return component_mask


def parse_telemetry(raw: bytes, received_monotonic_s: float | None = None) -> TelemetrySnapshot:
    """Validate telemetry v1; a missing physical source remains explicit None."""
    if len(raw) > 8192:
        raise ValueError("oversize telemetry")
    payload: dict[str, Any] = json.loads(raw.decode("utf-8"))
    if payload.get("schema_version") != 1:
        raise ValueError("unsupported schema")
    return TelemetrySnapshot(
        sequence=int(payload["sequence"]),
        odometry_source=str(payload.get("odometry_source", "unavailable")),
        x_m=_optional_number(payload, "x_m"), y_m=_optional_number(payload, "y_m"),
        yaw_rad=_optional_number(payload, "yaw_rad"), voltage_v=_optional_number(payload, "voltage_v"),
        current_a=_optional_number(payload, "current_a"), power_w=_optional_number(payload, "power_w"),
        drive_state=str(payload.get("drive_state", "unavailable")),
        can_state=str(payload.get("can_state", "unavailable")),
        l515_state=str(payload.get("l515_state", "unavailable")),
        l515_detail=str(payload.get("l515_detail", "")),
        l515_mode=str(payload.get("l515_mode", "-")),
        l515_color_hz=_optional_number(payload, "l515_color_hz"),
        l515_depth_hz=_optional_number(payload, "l515_depth_hz"),
        l515_submitted_hz=_optional_number(payload, "l515_submitted_hz"),
        l515_sent_hz=_optional_number(payload, "l515_sent_hz"),
        l515_drop_hz=_optional_number(payload, "l515_drop_hz"),
        l515_ros_topic_rates_hz=_l515_ros_topic_rates(payload),
        l515_aligned_depth_age_ms=_optional_number(payload, "l515_aligned_depth_age_ms"),
        l515_process_cpu_percent=_optional_number(payload, "l515_process_cpu_percent"),
        l515_process_rss_bytes=_optional_int(payload, "l515_process_rss_bytes"),
        pdist_soc_percent=_optional_int(payload, "pdist_soc_percent"),
        pdist_battery_flags=_optional_int(payload, "pdist_battery_flags"),
        pdist_protection_flags=_optional_int(payload, "pdist_protection_flags"),
        pdist_charge_current_a=_optional_number(payload, "pdist_charge_current_a"),
        rs485_state=str(payload.get("rs485_state", "unavailable")),
        rs485_consecutive_failures=_optional_int(payload, "rs485_consecutive_failures"),
        rs485_detail=str(payload.get("rs485_detail", "")),
        unit_status=_status_mapping(payload, "unit_status"),
        compose_status=_status_mapping(payload, "compose_status"),
        journal_tail=_journal_tail(payload),
        safety_status=str(payload.get("safety_status", "unavailable")),
        safety_distance_mm=_optional_number(payload, "safety_distance_mm"),
        safety_estop_required=(None if payload.get("safety_estop_required") is None
                               else bool(payload["safety_estop_required"])),
        safety_consecutive_failures=_optional_int(payload, "safety_consecutive_failures"),
        safety_detail=str(payload.get("safety_detail", "")),
        component_mask=_optional_component_mask(payload),
        wheel_count=_optional_int(payload, "wheel_count"),
        wheel_fault_count=_optional_int(payload, "wheel_fault_count"),
        wheel_stale_count=_optional_int(payload, "wheel_stale_count"),
        wheel_axis_error_count=_optional_int(payload, "wheel_axis_error_count"),
        wheel_steer_fault_count=_optional_int(payload, "wheel_steer_fault_count"),
        wheel_statuses=_wheel_statuses(payload),
        truncated=payload.get("truncated") is True,
        received_monotonic_s=time.monotonic() if received_monotonic_s is None else received_monotonic_s,
    )


_COMPONENT_BANNER_LABELS = (
    ("drive", "DRIVE"),
    ("steer", "STEER"),
    ("us100", "US-100"),
    ("robot_arm", "ARM"),
)


def mask_banner_text(component_mask: Mapping[str, bool] | None) -> str | None:
    """Render disabled component names in one stable operator-facing order."""
    if component_mask is None:
        return None
    disabled = [
        label
        for component, label in _COMPONENT_BANNER_LABELS
        if component_mask.get(component) is False
    ]
    return None if not disabled else "MASK: " + "·".join(disabled) + " OFF"


def safety_banner_state(
    snapshot: TelemetrySnapshot | None,
    *,
    component_mask: Mapping[str, bool] | None,
    telemetry_live: bool,
) -> tuple[str, str]:
    """Return safety banner copy/color, with US-100 masking taking priority."""
    if component_mask is not None and component_mask.get("us100") is False:
        return "SAFETY DISABLED (US-100 OFF)", "#d97706"
    if not telemetry_live or snapshot is None:
        return "SAFETY UNAVAILABLE", "#d97706"
    if snapshot.safety_estop_required:
        detail = snapshot.safety_detail or "no detail"
        return f"SAFETY ESTOP · {snapshot.safety_status} · {detail}", "#dc2626"
    return f"SAFETY CLEAR · {snapshot.safety_status}", "#16a34a"


def chassis_component_states(
    snapshot: TelemetrySnapshot | None,
    *,
    now_s: float | None = None,
) -> tuple[str, str, str]:
    """Return ODOM/DRIVE/CAN health with the shared 1 s receive-age gate."""
    if snapshot is None:
        return "UNAVAILABLE", "UNAVAILABLE", "UNAVAILABLE"
    current_s = time.monotonic() if now_s is None else float(now_s)
    if current_s - snapshot.received_monotonic_s > 1.0:
        return "STALE", "STALE", "STALE"
    return (
        "LIVE" if snapshot.odometry_source != "unavailable" else "UNAVAILABLE",
        "LIVE" if snapshot.drive_state != "unavailable" else "UNAVAILABLE",
        "LIVE" if snapshot.can_state != "unavailable" else "UNAVAILABLE",
    )


class LatestTelemetryReceiver:
    """Latest-only UDP receiver. It cannot affect robot control."""
    def __init__(self, port: int) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind(("0.0.0.0", port))
        self._latest: TelemetrySnapshot | None = None
        self._lock = threading.Lock()
        self._stopping = threading.Event()
        self._source_gate = SourceSequenceGate(stale_after_s=2.0)
        self._thread = threading.Thread(target=self._run, name="robot-telemetry", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        self._socket.settimeout(0.2)
        while not self._stopping.is_set():
            try:
                raw, address = self._socket.recvfrom(8192)
                received_s = time.monotonic()
                snapshot = parse_telemetry(raw, received_monotonic_s=received_s)
            except (OSError, ValueError, UnicodeDecodeError, json.JSONDecodeError):
                continue
            if not self._source_gate.accept(
                address,
                snapshot.sequence,
                now_s=received_s,
            ):
                continue
            with self._lock:
                self._latest = snapshot

    def latest(self) -> TelemetrySnapshot | None:
        with self._lock:
            return self._latest

    def close(self) -> None:
        self._stopping.set()
        self._socket.close()
        self._thread.join(timeout=1.0)
