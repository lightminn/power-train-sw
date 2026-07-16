"""Pure parsing and availability tracking for remote receiver feedback."""

from dataclasses import dataclass
import json
import math
from numbers import Real

from remote_video.contract import RECEIVER_FEEDBACK_SCHEMA_VERSION


# Compatibility name retained for existing dashboard callers.  The shared
# remote-video contract is the schema authority used by the laptop sender.
SCHEMA_VERSION = RECEIVER_FEEDBACK_SCHEMA_VERSION
CHANNELS = ("l515_rgb", "d435i_rgb")
L515_UNAVAILABLE_VERDICT = "REMOTE_DRIVE_VIDEO_UNAVAILABLE"
D435I_UNAVAILABLE_VERDICT = "REMOTE_ARM_VIDEO_UNAVAILABLE"

_VERDICTS = {
    "l515_rgb": L515_UNAVAILABLE_VERDICT,
    "d435i_rgb": D435I_UNAVAILABLE_VERDICT,
}
_REPORT_FIELDS = {
    "schema_version",
    "channel",
    "session_id",
    "sequence",
    "decode_fps",
    "display_fps",
    "frame_age_ms",
    "sequence_gap",
    "rtt_ms",
    "loss_percent",
}


class FeedbackError(ValueError):
    """Raised when a receiver feedback report violates the v1 contract."""


@dataclass(frozen=True)
class ReceiverReport:
    channel: str
    session_id: str
    sequence: int
    decode_fps: float
    display_fps: float
    frame_age_ms: float
    sequence_gap: int
    rtt_ms: float
    loss_percent: float
    received_monotonic_ns: int


@dataclass(frozen=True)
class FeedbackConfig:
    report_ttl_s: float = 2.0
    enter_fps: float = 29.0
    exit_fps: float = 29.5
    enter_dwell_s: float = 1.0
    exit_dwell_s: float = 2.0

    def __post_init__(self) -> None:
        for name in (
            "report_ttl_s",
            "enter_fps",
            "exit_fps",
            "enter_dwell_s",
            "exit_dwell_s",
        ):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, Real)
                or not math.isfinite(value)
            ):
                raise ValueError(f"{name} must be a finite number")
        if self.report_ttl_s <= 0:
            raise ValueError("report_ttl_s must be positive")
        if self.enter_fps < 0 or self.exit_fps < 0:
            raise ValueError("fps thresholds must be non-negative")
        if self.exit_fps <= self.enter_fps:
            raise ValueError("exit_fps must be greater than enter_fps")
        if self.enter_dwell_s < 0 or self.exit_dwell_s < 0:
            raise ValueError("dwell values must be non-negative")


@dataclass(frozen=True)
class ChannelAvailability:
    available: bool
    verdict: str | None
    reason: str
    # Optional receiver evidence needed by the pure profile state machine.
    loss_percent: float | None = None


@dataclass
class _ChannelState:
    latest: ReceiverReport | None = None
    available: bool = False
    low_since_ns: int | None = None
    high_since_ns: int | None = None
    reason: str = "no_receiver_report"


def _integer(value, name: str, *, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise FeedbackError(f"{name} must be an integer >= {minimum}")
    return value


def _finite_number(value, name: str, *, minimum: float = 0.0) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, Real)
        or not math.isfinite(value)
        or value < minimum
    ):
        raise FeedbackError(f"{name} must be a finite number >= {minimum}")
    return float(value)


def parse_report(
    data: bytes, *, now_monotonic_ns: int, max_bytes: int = 4096
) -> ReceiverReport:
    """Parse one bounded v1 JSON report and attach local receive time."""

    if isinstance(max_bytes, bool) or not isinstance(max_bytes, int) or max_bytes <= 0:
        raise FeedbackError("max_bytes must be a positive integer")
    if not isinstance(data, bytes) or not data or len(data) > max_bytes:
        raise FeedbackError("feedback report exceeds size limit or is empty")
    received_ns = _integer(now_monotonic_ns, "now_monotonic_ns")
    try:
        message = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise FeedbackError("invalid JSON") from exc
    if not isinstance(message, dict):
        raise FeedbackError("feedback report must be a JSON object")
    if set(message) != _REPORT_FIELDS:
        raise FeedbackError("feedback report has invalid fields")
    if (
        isinstance(message["schema_version"], bool)
        or message["schema_version"] != SCHEMA_VERSION
    ):
        raise FeedbackError("unsupported schema version")
    channel = message["channel"]
    if channel not in CHANNELS:
        raise FeedbackError("unknown channel")
    session_id = message["session_id"]
    if not isinstance(session_id, str) or not session_id:
        raise FeedbackError("session_id must be a non-empty string")

    loss_percent = _finite_number(message["loss_percent"], "loss_percent")
    if loss_percent > 100.0:
        raise FeedbackError("loss_percent must be <= 100")
    return ReceiverReport(
        channel=channel,
        session_id=session_id,
        sequence=_integer(message["sequence"], "sequence"),
        decode_fps=_finite_number(message["decode_fps"], "decode_fps"),
        display_fps=_finite_number(message["display_fps"], "display_fps"),
        frame_age_ms=_finite_number(message["frame_age_ms"], "frame_age_ms"),
        sequence_gap=_integer(message["sequence_gap"], "sequence_gap"),
        rtt_ms=_finite_number(message["rtt_ms"], "rtt_ms"),
        loss_percent=loss_percent,
        received_monotonic_ns=received_ns,
    )


class FeedbackTracker:
    """Keep latest-only reports and apply fail-closed FPS hysteresis."""

    def __init__(self, config: FeedbackConfig):
        if not isinstance(config, FeedbackConfig):
            raise TypeError("config must be FeedbackConfig")
        self._config = config
        self._states = {channel: _ChannelState() for channel in CHANNELS}

    def update(self, report: ReceiverReport) -> bool:
        """Accept a newer report, returning false for duplicates or reordering."""

        if not isinstance(report, ReceiverReport):
            raise TypeError("report must be ReceiverReport")
        if report.channel not in self._states:
            raise ValueError("unknown channel")
        state = self._states[report.channel]
        previous = state.latest
        if (
            previous is not None
            and report.session_id == previous.session_id
            and report.sequence <= previous.sequence
        ):
            return False
        if previous is not None and report.session_id != previous.session_id:
            state.available = False
            state.low_since_ns = None
            state.high_since_ns = None
            state.reason = "receiver_session_changed"
        state.latest = report
        self._apply_fps_state(state, report.received_monotonic_ns)
        return True

    def availability(
        self, channel: str, now_monotonic_ns: int
    ) -> ChannelAvailability:
        if channel not in self._states:
            raise ValueError(f"unknown channel: {channel!r}")
        now_ns = _integer(now_monotonic_ns, "now_monotonic_ns")
        state = self._states[channel]
        report = state.latest
        if report is None:
            return ChannelAvailability(
                False, _VERDICTS[channel], "no_receiver_report", None
            )
        if now_ns < report.received_monotonic_ns:
            raise ValueError("now_monotonic_ns precedes the latest report")
        ttl_ns = int(self._config.report_ttl_s * 1_000_000_000)
        if now_ns - report.received_monotonic_ns > ttl_ns:
            state.available = False
            state.low_since_ns = None
            state.high_since_ns = None
            state.reason = "receiver_report_stale"
        else:
            self._apply_fps_state(state, now_ns)
        return ChannelAvailability(
            available=state.available,
            verdict=None if state.available else _VERDICTS[channel],
            reason=state.reason,
            loss_percent=report.loss_percent,
        )

    def _apply_fps_state(self, state: _ChannelState, now_ns: int) -> None:
        report = state.latest
        if report is None:
            return
        if state.available:
            state.high_since_ns = None
            if report.display_fps < self._config.enter_fps:
                if state.low_since_ns is None:
                    state.low_since_ns = report.received_monotonic_ns
                dwell_ns = int(self._config.enter_dwell_s * 1_000_000_000)
                if now_ns - state.low_since_ns >= dwell_ns:
                    state.available = False
                    state.low_since_ns = None
                    state.reason = "display_fps_below_enter"
                else:
                    state.reason = "display_fps_entry_dwell"
            else:
                state.low_since_ns = None
                state.reason = "receiver_report_healthy"
            return

        state.low_since_ns = None
        if report.display_fps >= self._config.exit_fps:
            if state.high_since_ns is None:
                state.high_since_ns = report.received_monotonic_ns
            dwell_ns = int(self._config.exit_dwell_s * 1_000_000_000)
            if now_ns - state.high_since_ns >= dwell_ns:
                state.available = True
                state.high_since_ns = None
                state.reason = "receiver_report_healthy"
            else:
                state.reason = "display_fps_recovery_dwell"
        else:
            state.high_since_ns = None
            state.reason = "display_fps_below_exit"
