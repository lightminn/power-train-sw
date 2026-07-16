"""Pure static video profiles and receiver-authoritative transitions.

Profile changes must only be applied at the boundary that performs supervised
replacement of the existing x264 process. Runtime selection of a new encoder
kind or an arbitrary pipeline string is not allowed here.
"""

from dataclasses import dataclass
import math
from numbers import Real

from .receiver_feedback import ChannelAvailability


@dataclass(frozen=True)
class ChannelSpec:
    width: int
    height: int
    fps: int
    bitrate_kbps: int


@dataclass(frozen=True)
class VideoProfile:
    name: str
    l515: ChannelSpec
    d435i: ChannelSpec
    metadata_best_effort: bool
    l515_rgb_locked: bool


NORMAL = VideoProfile(
    name="NORMAL",
    l515=ChannelSpec(1280, 720, 30, 3000),
    # 2000 kbps is provisional until the D435i channel is qualified.
    d435i=ChannelSpec(848, 480, 30, 2000),
    metadata_best_effort=True,
    l515_rgb_locked=False,
)
CONGESTED = VideoProfile(
    name="CONGESTED",
    # Both reduced bitrate steps are provisional until qualification.
    l515=ChannelSpec(1280, 720, 30, 1800),
    d435i=ChannelSpec(848, 480, 30, 1200),
    metadata_best_effort=True,
    l515_rgb_locked=False,
)
EMERGENCY_REMOTE = VideoProfile(
    name="EMERGENCY_REMOTE",
    l515=ChannelSpec(1280, 720, 30, 1800),
    d435i=ChannelSpec(848, 480, 30, 1200),
    metadata_best_effort=True,
    l515_rgb_locked=True,
)
PROFILES = {
    profile.name: profile
    for profile in (NORMAL, CONGESTED, EMERGENCY_REMOTE)
}


def validate_profiles() -> None:
    """Reject hidden resolution/FPS degradation or bitrate increases."""

    ordered = (NORMAL, CONGESTED, EMERGENCY_REMOTE)
    for channel_name in ("l515", "d435i"):
        specs = [getattr(profile, channel_name) for profile in ordered]
        geometry = {(spec.width, spec.height, spec.fps) for spec in specs}
        if len(geometry) != 1:
            raise ValueError(
                f"{channel_name} resolution and fps must stay fixed across profiles"
            )
        bitrates = [spec.bitrate_kbps for spec in specs]
        if any(left < right for left, right in zip(bitrates, bitrates[1:])):
            raise ValueError(
                f"{channel_name} bitrate must be monotonically non-increasing"
            )


validate_profiles()


@dataclass(frozen=True)
class ProfileConfig:
    loss_enter_percent: float = 5.0
    loss_exit_percent: float = 1.0
    congested_enter_dwell_s: float = 1.0
    normal_exit_dwell_s: float = 2.0
    min_dwell_s: float = 3.0

    def __post_init__(self) -> None:
        for name in (
            "loss_enter_percent",
            "loss_exit_percent",
            "congested_enter_dwell_s",
            "normal_exit_dwell_s",
            "min_dwell_s",
        ):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, Real)
                or not math.isfinite(value)
                or value < 0
            ):
                raise ValueError(f"{name} must be a finite non-negative number")
        if self.loss_enter_percent > 100 or self.loss_exit_percent > 100:
            raise ValueError("loss thresholds must not exceed 100 percent")
        if self.loss_exit_percent >= self.loss_enter_percent:
            raise ValueError(
                "loss_exit_percent must be less than loss_enter_percent"
            )


@dataclass(frozen=True)
class ProfileDecision:
    profile: VideoProfile
    changed: bool
    reason: str
    entered_monotonic_ns: int


class ProfileStateMachine:
    """Choose a static profile from receiver evidence and operator override."""

    def __init__(self, config: ProfileConfig):
        if not isinstance(config, ProfileConfig):
            raise TypeError("config must be ProfileConfig")
        self._config = config
        self._profile = NORMAL
        self._entered_ns: int | None = None
        self._last_tick_ns: int | None = None
        self._candidate: str | None = None
        self._candidate_since_ns: int | None = None
        self._override_active = False

    def tick(
        self,
        now_monotonic_ns: int,
        *,
        l515: ChannelAvailability,
        d435i: ChannelAvailability,
        sender_drop_hint: bool,
        operator_override: str | None,
    ) -> ProfileDecision:
        now_ns = self._validate_tick_inputs(
            now_monotonic_ns, l515, d435i, sender_drop_hint, operator_override
        )
        if self._entered_ns is None:
            self._entered_ns = now_ns

        if operator_override is not None:
            self._override_active = True
            self._clear_candidate()
            return self._set_profile(
                PROFILES[operator_override], now_ns, "operator_override"
            )

        if self._override_active:
            self._override_active = False
            self._clear_candidate()

        unavailable_channel = None
        if not l515.available:
            unavailable_channel = "l515_rgb"
        elif not d435i.available:
            unavailable_channel = "d435i_rgb"
        if unavailable_channel is not None:
            self._clear_candidate()
            return self._set_profile(
                EMERGENCY_REMOTE,
                now_ns,
                f"receiver_unavailable:{unavailable_channel}",
            )

        losses = [
            availability.loss_percent
            for availability in (l515, d435i)
            if availability.loss_percent is not None
        ]
        maximum_loss = max(losses, default=0.0)
        high_loss = maximum_loss >= self._config.loss_enter_percent
        hinted_loss = (
            sender_drop_hint
            and maximum_loss > self._config.loss_exit_percent
        )
        fully_recovered = maximum_loss <= self._config.loss_exit_percent

        if self._profile is NORMAL:
            if high_loss or hinted_loss:
                reason = (
                    "receiver_loss_high"
                    if high_loss
                    else "receiver_loss_with_sender_hint"
                )
                if self._candidate_ready(
                    "congested", now_ns, self._config.congested_enter_dwell_s
                ) and self._minimum_dwell_met(now_ns):
                    return self._set_profile(CONGESTED, now_ns, reason)
            else:
                self._clear_candidate()
            return self._decision(False, "profile_stable")

        if self._profile is CONGESTED:
            if fully_recovered:
                if self._candidate_ready(
                    "normal", now_ns, self._config.normal_exit_dwell_s
                ) and self._minimum_dwell_met(now_ns):
                    return self._set_profile(
                        NORMAL, now_ns, "receiver_loss_recovered"
                    )
            else:
                self._clear_candidate()
            return self._decision(False, "profile_stable")

        if fully_recovered:
            if self._candidate_ready(
                "emergency_recovery", now_ns, self._config.normal_exit_dwell_s
            ) and self._minimum_dwell_met(now_ns):
                return self._set_profile(NORMAL, now_ns, "receiver_recovered")
        elif high_loss or hinted_loss:
            reason = (
                "receiver_loss_high"
                if high_loss
                else "receiver_loss_with_sender_hint"
            )
            if self._candidate_ready(
                "emergency_to_congested",
                now_ns,
                self._config.congested_enter_dwell_s,
            ) and self._minimum_dwell_met(now_ns):
                return self._set_profile(CONGESTED, now_ns, reason)
        else:
            self._clear_candidate()
        return self._decision(False, "profile_stable")

    def _validate_tick_inputs(
        self,
        now_monotonic_ns,
        l515,
        d435i,
        sender_drop_hint,
        operator_override,
    ) -> int:
        if (
            isinstance(now_monotonic_ns, bool)
            or not isinstance(now_monotonic_ns, int)
            or now_monotonic_ns < 0
        ):
            raise ValueError("now_monotonic_ns must be a non-negative integer")
        if self._last_tick_ns is not None and now_monotonic_ns < self._last_tick_ns:
            raise ValueError("monotonic tick time must not go backwards")
        if not isinstance(l515, ChannelAvailability) or not isinstance(
            d435i, ChannelAvailability
        ):
            raise TypeError("channel inputs must be ChannelAvailability")
        if type(sender_drop_hint) is not bool:
            raise TypeError("sender_drop_hint must be boolean")
        if operator_override is not None and operator_override not in PROFILES:
            raise ValueError("operator_override must name a static profile")
        self._last_tick_ns = now_monotonic_ns
        return now_monotonic_ns

    def _candidate_ready(self, name: str, now_ns: int, dwell_s: float) -> bool:
        if self._candidate != name:
            self._candidate = name
            self._candidate_since_ns = now_ns
        assert self._candidate_since_ns is not None
        return now_ns - self._candidate_since_ns >= int(dwell_s * 1_000_000_000)

    def _minimum_dwell_met(self, now_ns: int) -> bool:
        assert self._entered_ns is not None
        return now_ns - self._entered_ns >= int(
            self._config.min_dwell_s * 1_000_000_000
        )

    def _clear_candidate(self) -> None:
        self._candidate = None
        self._candidate_since_ns = None

    def _set_profile(
        self, profile: VideoProfile, now_ns: int, reason: str
    ) -> ProfileDecision:
        changed = profile is not self._profile
        if changed:
            self._profile = profile
            self._entered_ns = now_ns
        self._clear_candidate()
        return self._decision(changed, reason)

    def _decision(self, changed: bool, reason: str) -> ProfileDecision:
        assert self._entered_ns is not None
        return ProfileDecision(
            profile=self._profile,
            changed=changed,
            reason=reason,
            entered_monotonic_ns=self._entered_ns,
        )
