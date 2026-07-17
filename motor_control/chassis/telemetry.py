from dataclasses import dataclass
import re
import subprocess
import threading
import time
from typing import Optional, Tuple

from chassis.wheel_consistency import WheelConsistencyResult


@dataclass(frozen=True)
class AkNodeHealth:
    can_id: int
    physical_wheel: str
    last_feedback_age_ms: Optional[float]
    feedback_rate_hz: float
    steer_fault: int
    stale: bool
    recovery_count: int


@dataclass(frozen=True)
class OdriveNodeHealth:
    node_id: int
    physical_wheel: str
    last_heartbeat_age_ms: Optional[float]
    last_encoder_age_ms: Optional[float]
    axis_state: int
    axis_error: int
    stale: bool
    recovery_count: int


@dataclass(frozen=True)
class CanBusHealth:
    rx_packet_delta: int = 0
    tx_packet_delta: int = 0
    error_warning: bool = False
    error_passive: bool = False
    bus_off_delta: int = 0
    restart_count: int = 0


@dataclass(frozen=True)
class _CanBusTotals:
    rx_packets: int
    tx_packets: int
    state: str
    bus_off: int
    restarted: int


class CanBusStatsSampler:
    """Sample Linux CAN link statistics away from the 50 Hz control callback."""

    def __init__(self, channel: str, *, period_s: float = 1.0):
        self.channel = str(channel)
        self.period_s = float(period_s)
        self._health = CanBusHealth()
        self._previous = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    def update_from_text(self, text: str) -> CanBusHealth:
        current = self._parse(text)
        with self._lock:
            previous = self._previous
            self._previous = current
            rx_delta = 0 if previous is None else max(
                0, current.rx_packets - previous.rx_packets
            )
            tx_delta = 0 if previous is None else max(
                0, current.tx_packets - previous.tx_packets
            )
            bus_off_delta = 0 if previous is None else max(
                0, current.bus_off - previous.bus_off
            )
            self._health = CanBusHealth(
                rx_packet_delta=rx_delta,
                tx_packet_delta=tx_delta,
                error_warning=current.state in {
                    "ERROR-WARNING", "ERROR-PASSIVE", "BUS-OFF",
                },
                error_passive=current.state in {"ERROR-PASSIVE", "BUS-OFF"},
                bus_off_delta=bus_off_delta,
                restart_count=current.restarted,
            )
            return self._health

    def snapshot(self) -> CanBusHealth:
        with self._lock:
            return self._health

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
            name=f"{self.channel}-health",
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        self._thread = None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                completed = subprocess.run(
                    ["ip", "-details", "-statistics", "link", "show", self.channel],
                    check=True,
                    capture_output=True,
                    text=True,
                    timeout=0.5,
                )
                self.update_from_text(completed.stdout)
            except (OSError, subprocess.SubprocessError, ValueError):
                pass
            self._stop.wait(self.period_s)

    @staticmethod
    def _parse(text: str) -> _CanBusTotals:
        state_match = re.search(r"\bcan state ([A-Z-]+)", text)
        if state_match is None:
            raise ValueError("CAN state is missing from ip statistics")
        lines = [line.strip() for line in text.splitlines() if line.strip()]

        def table_value(header_prefix, column):
            for index, line in enumerate(lines[:-1]):
                if line.startswith(header_prefix):
                    headers = line.split()
                    values = lines[index + 1].split()
                    if headers and headers[0].endswith(":"):
                        headers = headers[1:]
                    try:
                        return int(values[headers.index(column)])
                    except (ValueError, IndexError) as exc:
                        raise ValueError(
                            f"invalid {header_prefix} statistics"
                        ) from exc
            raise ValueError(f"missing {header_prefix} statistics")

        return _CanBusTotals(
            rx_packets=table_value("RX:", "packets"),
            tx_packets=table_value("TX:", "packets"),
            state=state_match.group(1),
            bus_off=table_value("re-started", "bus-off"),
            restarted=table_value("re-started", "re-started"),
        )


@dataclass(frozen=True)
class CanOwnerHealth:
    pid: int
    process_name: str
    lock_path: str
    acquisition_time: str


@dataclass(frozen=True)
class InterlockHealth:
    motion_hold_sources: Tuple[str, ...]
    latched_estop_sources: Tuple[str, ...]
    reset_required: bool


@dataclass(frozen=True)
class WheelSnapshot:
    name: str
    corner_mode: str
    drive_turns_per_s: float
    steer_deg: float
    drive_current_a: float
    steer_current_a: float
    drive_stale: bool
    steer_stale: bool
    drive_axis_error: int
    steer_fault: int


@dataclass(frozen=True)
class ChassisSnapshot:
    chassis_mode: str
    stop_state: str
    healthy: bool
    wheels: Tuple[WheelSnapshot, ...]
    ak_nodes: Tuple[AkNodeHealth, ...] = ()
    odrive_nodes: Tuple[OdriveNodeHealth, ...] = ()
    bus: CanBusHealth = CanBusHealth()
    owner: Optional[CanOwnerHealth] = None
    interlock: InterlockHealth = InterlockHealth((), (), False)
    wheel_consistency: WheelConsistencyResult = WheelConsistencyResult(
        (), 1.0, None, None
    )
    extraction_active: bool = False
    extraction_remaining_s: float = 0.0
    extraction_budget_left_m: float = 0.0
    extraction_grants_left: int = 0
    last_extraction_reject: str = ""


def build_can_health_event(
    snapshot: ChassisSnapshot,
    *,
    source: str = "chassis_node",
    wall_time_ns: Optional[int] = None,
    monotonic_ns: Optional[int] = None,
):
    """Build the existing Task-1/2 datagram event contract from one snapshot."""
    owner = None
    if snapshot.owner is not None:
        owner = {
            "pid": snapshot.owner.pid,
            "process_name": snapshot.owner.process_name,
            "lock_path": snapshot.owner.lock_path,
            "acquisition_time": snapshot.owner.acquisition_time,
        }
    warnings = [
        {
            "severity": "WARN",
            "code": warning.code,
            "wheels": list(warning.wheels),
            "value": warning.value,
            "threshold": warning.threshold,
        }
        for warning in snapshot.wheel_consistency.warnings
    ]
    payload = {
        "healthy": snapshot.healthy,
        "ak_nodes": [
            {
                "can_id": node.can_id,
                "physical_wheel": node.physical_wheel,
                "last_feedback_age_ms": node.last_feedback_age_ms,
                "feedback_rate_hz": node.feedback_rate_hz,
                "steer_fault": node.steer_fault,
                "stale": node.stale,
                "recovery_count": node.recovery_count,
            }
            for node in snapshot.ak_nodes
        ],
        "odrive_nodes": [
            {
                "node_id": node.node_id,
                "physical_wheel": node.physical_wheel,
                "last_heartbeat_age_ms": node.last_heartbeat_age_ms,
                "last_encoder_age_ms": node.last_encoder_age_ms,
                "axis_state": node.axis_state,
                "axis_error": node.axis_error,
                "stale": node.stale,
                "recovery_count": node.recovery_count,
            }
            for node in snapshot.odrive_nodes
        ],
        "bus": {
            "rx_packet_delta": snapshot.bus.rx_packet_delta,
            "tx_packet_delta": snapshot.bus.tx_packet_delta,
            "error_warning": snapshot.bus.error_warning,
            "error_passive": snapshot.bus.error_passive,
            "bus_off_delta": snapshot.bus.bus_off_delta,
            "restart_count": snapshot.bus.restart_count,
        },
        "owner": owner,
        "interlock": {
            "motion_hold_sources": list(
                snapshot.interlock.motion_hold_sources
            ),
            "latched_estop_sources": list(
                snapshot.interlock.latched_estop_sources
            ),
            "reset_required": snapshot.interlock.reset_required,
        },
        "wheel_consistency": {
            "warnings": warnings,
            "terrain_speed_cap": snapshot.wheel_consistency.terrain_speed_cap,
            "wheel_yaw_rate_rad_s": (
                snapshot.wheel_consistency.wheel_yaw_rate_rad_s
            ),
            "imu_yaw_rate_rad_s": (
                snapshot.wheel_consistency.imu_yaw_rate_rad_s
            ),
        },
    }
    warn = (
        not snapshot.healthy
        or bool(warnings)
        or snapshot.bus.error_warning
        or snapshot.bus.error_passive
        or snapshot.bus.bus_off_delta > 0
    )
    return {
        "schema_version": 1,
        "wall_time_ns": time.time_ns() if wall_time_ns is None else int(wall_time_ns),
        "monotonic_ns": (
            time.monotonic_ns()
            if monotonic_ns is None
            else int(monotonic_ns)
        ),
        "source": str(source),
        "event_type": "CAN_HEALTH",
        "severity": "WARN" if warn else "INFO",
        "payload": payload,
    }
