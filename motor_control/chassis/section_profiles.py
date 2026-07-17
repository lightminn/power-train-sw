"""WP8 section profiles above the chassis-owned mission supervisor.

This module is a pure, standard-library-only progress layer.  It never calls
``MissionSupervisor``, changes command authority, or creates motor commands;
it only returns hints for a future cross-team adapter.

The event names and the SMOG ``payload["arm_result"]`` values are an explicit
fake bridge contract.  The real perception/arm topics, ``MISSION_STOP``
unlock ordering, and recovery policy remain unconfirmed.  In particular,
SMOG currently produces a conservative speed hint without a drive hold; the
competition rule for continuing through smoke must be confirmed before this
hint is connected to any authority path.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field, replace
import math
from types import MappingProxyType
from typing import Any


# Temporary fake section-event constants.  Unknown strings remain recordable
# so a future team topic cannot silently mutate section progress.
SMOG_ENTER = "SMOG_ENTER"
SMOG_EXIT = "SMOG_EXIT"
LIGHT_RED = "LIGHT_RED"
LIGHT_GREEN = "LIGHT_GREEN"
MARKER_DETECTED = "MARKER_DETECTED"
STUCK_DETECTED = "STUCK_DETECTED"
LEAD_LOST = "LEAD_LOST"
LEAD_FOUND = "LEAD_FOUND"
ARRIVAL_REACHED = "ARRIVAL_REACHED"
OPERATOR_HOLD = "OPERATOR_HOLD"
OPERATOR_RESUME = "OPERATOR_RESUME"
SECTION_ENTER = "SECTION_ENTER"
SECTION_EXIT = "SECTION_EXIT"

EVENT_TYPES = (
    SMOG_ENTER,
    SMOG_EXIT,
    LIGHT_RED,
    LIGHT_GREEN,
    MARKER_DETECTED,
    STUCK_DETECTED,
    LEAD_LOST,
    LEAD_FOUND,
    ARRIVAL_REACHED,
    OPERATOR_HOLD,
    OPERATOR_RESUME,
    SECTION_ENTER,
    SECTION_EXIT,
)

SMOG = "SMOG"
RELIEF = "RELIEF"
MARKERS = "MARKERS"
ICE = "ICE"
FOLLOW = "FOLLOW"


def _finite(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{label} must be numeric") from None
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def _frozen_mapping(value: Mapping) -> Mapping:
    return MappingProxyType(dict(value))


@dataclass(frozen=True)
class SectionEvent:
    type: str
    stamp_s: float
    payload: Mapping

    def __post_init__(self) -> None:
        if not isinstance(self.type, str) or not self.type.strip():
            raise ValueError("event type must be a non-empty string")
        stamp_s = _finite(self.stamp_s, "stamp_s")
        if stamp_s < 0.0:
            raise ValueError("stamp_s must be non-negative")
        if not isinstance(self.payload, Mapping):
            raise ValueError("payload must be a mapping")
        object.__setattr__(self, "type", self.type.strip())
        object.__setattr__(self, "stamp_s", stamp_s)
        object.__setattr__(self, "payload", _frozen_mapping(self.payload))


@dataclass(frozen=True)
class MarkerObservationRecord:
    marker_key: str
    class_id: int | None
    class_name: str
    position: tuple
    confidence: float
    stamp_s: float
    accepted: bool
    reason: str
    stage: str


@dataclass(frozen=True)
class _MarkerIdentity:
    marker_key: str
    class_id: int | None
    class_name: str
    position: tuple
    first_seen_s: float
    last_seen_s: float
    observations: int
    stage: str


@dataclass(frozen=True)
class MarkerDedup:
    """Confirm novel markers using ID first, then class/3-D clustering.

    ``observe`` returns ``True`` when an observation creates or advances a
    candidate.  Only records that reach ``marker_confirm_observations`` count
    as unique.  Duplicate and rejected observations return ``False`` and are
    preserved in ``failures``; candidate/confirmation progress is preserved in
    ``successes``.
    """

    cluster_m: float = 1.0
    min_reobserve_s: float = 1.0
    min_confidence: float = 0.5
    marker_confirm_observations: int = 2
    marker_candidate_ttl_s: float = 5.0
    marker_max_candidates: int = 16
    _records: dict = field(default_factory=dict, init=False, repr=False)
    _successes: list = field(default_factory=list, init=False, repr=False)
    _failures: list = field(default_factory=list, init=False, repr=False)
    _last_observation_stamp_s: float | None = field(
        default=None,
        init=False,
        repr=False,
    )
    _next_marker_index: int = field(default=1, init=False, repr=False)

    def __post_init__(self) -> None:
        cluster_m = _finite(self.cluster_m, "cluster_m")
        min_reobserve_s = _finite(self.min_reobserve_s, "min_reobserve_s")
        min_confidence = _finite(self.min_confidence, "min_confidence")
        candidate_ttl_s = _finite(
            self.marker_candidate_ttl_s,
            "marker_candidate_ttl_s",
        )
        if cluster_m < 0.0:
            raise ValueError("cluster_m must be non-negative")
        if min_reobserve_s < 0.0:
            raise ValueError("min_reobserve_s must be non-negative")
        if not 0.0 <= min_confidence <= 1.0:
            raise ValueError("min_confidence must be in [0, 1]")
        if (
            isinstance(self.marker_confirm_observations, bool)
            or not isinstance(self.marker_confirm_observations, int)
            or self.marker_confirm_observations < 2
        ):
            raise ValueError(
                "marker_confirm_observations must be at least 2"
            )
        if candidate_ttl_s < min_reobserve_s:
            raise ValueError(
                "marker_candidate_ttl_s must be at least min_reobserve_s"
            )
        if (
            isinstance(self.marker_max_candidates, bool)
            or not isinstance(self.marker_max_candidates, int)
            or self.marker_max_candidates <= 0
        ):
            raise ValueError("marker_max_candidates must be positive")
        object.__setattr__(self, "cluster_m", cluster_m)
        object.__setattr__(self, "min_reobserve_s", min_reobserve_s)
        object.__setattr__(self, "min_confidence", min_confidence)
        object.__setattr__(
            self,
            "marker_candidate_ttl_s",
            candidate_ttl_s,
        )

    @property
    def unique_count(self) -> int:
        return sum(
            record.stage == "confirmed" for record in self._records.values()
        )

    @property
    def successes(self) -> tuple:
        return tuple(self._successes)

    @property
    def failures(self) -> tuple:
        return tuple(self._failures)

    def observe(
        self,
        *,
        class_id,
        class_name,
        position,
        confidence,
        stamp_s,
    ) -> bool:
        try:
            confidence = _finite(confidence, "confidence")
            stamp_s = _finite(stamp_s, "stamp_s")
            if stamp_s < 0.0:
                raise ValueError("stamp_s must be non-negative")
            class_name = str(class_name).strip()
            if not class_name:
                raise ValueError("class_name must be non-empty")
            if len(position) != 3:
                raise ValueError("position must have three coordinates")
            position = tuple(
                _finite(value, f"position[{index}]")
                for index, value in enumerate(position)
            )
            normalized_id = self._normalize_class_id(class_id)
        except (TypeError, ValueError) as exc:
            self._record(
                marker_key="",
                class_id=None,
                class_name=str(class_name).strip(),
                position=(),
                confidence=(
                    float(confidence)
                    if isinstance(confidence, (int, float))
                    else 0.0
                ),
                stamp_s=(
                    float(stamp_s)
                    if isinstance(stamp_s, (int, float))
                    else 0.0
                ),
                accepted=False,
                reason=f"invalid_observation:{exc}",
                stage="candidate",
            )
            return False

        if not 0.0 <= confidence <= 1.0:
            return self._reject(
                "invalid_confidence",
                normalized_id,
                class_name,
                position,
                confidence,
                stamp_s,
            )
        if confidence < self.min_confidence:
            return self._reject(
                "low_confidence",
                normalized_id,
                class_name,
                position,
                confidence,
                stamp_s,
            )

        last_stamp_s = self._last_observation_stamp_s
        if last_stamp_s is not None and stamp_s < last_stamp_s:
            return self._reject(
                "stale_observation",
                normalized_id,
                class_name,
                position,
                confidence,
                stamp_s,
            )
        object.__setattr__(self, "_last_observation_stamp_s", stamp_s)
        self._expire_candidates(stamp_s)

        matched, duplicate_reason = self._match(
            normalized_id,
            class_name,
            position,
        )
        if matched is not None:
            if stamp_s - matched.last_seen_s < self.min_reobserve_s:
                return self._reject(
                    "min_reobserve",
                    normalized_id,
                    class_name,
                    position,
                    confidence,
                    stamp_s,
                    marker_key=matched.marker_key,
                    stage=matched.stage,
                )
            if matched.stage == "candidate":
                observations = matched.observations + 1
                stage = (
                    "confirmed"
                    if observations >= self.marker_confirm_observations
                    else "candidate"
                )
                self._records[matched.marker_key] = replace(
                    matched,
                    last_seen_s=stamp_s,
                    observations=observations,
                    stage=stage,
                )
                self._record(
                    marker_key=matched.marker_key,
                    class_id=normalized_id,
                    class_name=class_name,
                    position=position,
                    confidence=confidence,
                    stamp_s=stamp_s,
                    accepted=True,
                    reason=(
                        "marker_confirmed"
                        if stage == "confirmed"
                        else "marker_candidate_progress"
                    ),
                    stage=stage,
                )
                return True
            self._records[matched.marker_key] = replace(
                matched,
                last_seen_s=stamp_s,
            )
            return self._reject(
                duplicate_reason,
                normalized_id,
                class_name,
                position,
                confidence,
                stamp_s,
                marker_key=matched.marker_key,
                stage="confirmed",
            )

        self._evict_oldest_candidate_if_full()
        if normalized_id is not None:
            marker_key = f"class_id:{normalized_id}"
        else:
            marker_key = (
                f"class_name:{class_name}:{self._next_marker_index}"
            )
            object.__setattr__(
                self,
                "_next_marker_index",
                self._next_marker_index + 1,
            )
        identity = _MarkerIdentity(
            marker_key=marker_key,
            class_id=normalized_id,
            class_name=class_name,
            position=position,
            first_seen_s=stamp_s,
            last_seen_s=stamp_s,
            observations=1,
            stage="candidate",
        )
        self._records[marker_key] = identity
        self._record(
            marker_key=marker_key,
            class_id=normalized_id,
            class_name=class_name,
            position=position,
            confidence=confidence,
            stamp_s=stamp_s,
            accepted=True,
            reason="marker_candidate",
            stage="candidate",
        )
        return True

    def _expire_candidates(self, stamp_s) -> None:
        expired = [
            marker_key
            for marker_key, record in self._records.items()
            if record.stage == "candidate"
            and stamp_s - record.last_seen_s > self.marker_candidate_ttl_s
        ]
        for marker_key in expired:
            del self._records[marker_key]

    def _evict_oldest_candidate_if_full(self) -> None:
        candidates = [
            record
            for record in self._records.values()
            if record.stage == "candidate"
        ]
        if len(candidates) < self.marker_max_candidates:
            return
        oldest = min(
            candidates,
            key=lambda record: (record.first_seen_s, record.marker_key),
        )
        del self._records[oldest.marker_key]

    @staticmethod
    def _normalize_class_id(class_id) -> int | None:
        if class_id is None or class_id == "":
            return None
        if isinstance(class_id, bool):
            raise ValueError("class_id must not be boolean")
        normalized = int(class_id)
        return normalized if normalized > 0 else None

    def _match(self, class_id, class_name, position):
        if class_id is not None:
            matched = self._records.get(f"class_id:{class_id}")
            return matched, "duplicate_class_id"

        nearest = None
        nearest_squared = None
        radius_squared = self.cluster_m * self.cluster_m
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
        return nearest, "duplicate_cluster"

    def _reject(
        self,
        reason,
        class_id,
        class_name,
        position,
        confidence,
        stamp_s,
        *,
        marker_key="",
        stage="candidate",
    ) -> bool:
        self._record(
            marker_key=marker_key,
            class_id=class_id,
            class_name=class_name,
            position=position,
            confidence=confidence,
            stamp_s=stamp_s,
            accepted=False,
            reason=reason,
            stage=stage,
        )
        return False

    def _record(self, **values) -> None:
        record = MarkerObservationRecord(**values)
        target = self._successes if record.accepted else self._failures
        target.append(record)


@dataclass(frozen=True)
class SectionProfile:
    section: str


SMOG_PROFILE = SectionProfile(SMOG)
RELIEF_PROFILE = SectionProfile(RELIEF)
MARKERS_PROFILE = SectionProfile(MARKERS)
ICE_PROFILE = SectionProfile(ICE)
FOLLOW_PROFILE = SectionProfile(FOLLOW)
SECTION_PROFILES = MappingProxyType(
    {
        profile.section: profile
        for profile in (
            SMOG_PROFILE,
            RELIEF_PROFILE,
            MARKERS_PROFILE,
            ICE_PROFILE,
            FOLLOW_PROFILE,
        )
    }
)


@dataclass(frozen=True)
class SectionConfig:
    cluster_m: float = 1.0
    min_reobserve_s: float = 1.0
    min_confidence: float = 0.5
    marker_target_count: int = 5
    marker_confirm_observations: int = 2
    marker_candidate_ttl_s: float = 5.0
    marker_max_candidates: int = 16
    smog_arm_result_count: int = 2
    smog_speed_hint: float = 0.25
    ice_speed_hint: float = 0.15
    relief_work_request: str = "ARRIVED_PICKUP"
    odom_gate_m: float | None = None
    state_ttl_s: float = 0.6

    def __post_init__(self) -> None:
        if (
            isinstance(self.marker_target_count, bool)
            or self.marker_target_count <= 0
        ):
            raise ValueError("marker_target_count must be positive")
        if (
            isinstance(self.marker_confirm_observations, bool)
            or not isinstance(self.marker_confirm_observations, int)
            or self.marker_confirm_observations < 2
        ):
            raise ValueError(
                "marker_confirm_observations must be at least 2"
            )
        if (
            isinstance(self.marker_max_candidates, bool)
            or not isinstance(self.marker_max_candidates, int)
            or self.marker_max_candidates <= 0
        ):
            raise ValueError("marker_max_candidates must be positive")
        if (
            isinstance(self.smog_arm_result_count, bool)
            or self.smog_arm_result_count <= 0
        ):
            raise ValueError("smog_arm_result_count must be positive")
        if (
            not isinstance(self.relief_work_request, str)
            or not self.relief_work_request
        ):
            raise ValueError("relief_work_request must be non-empty")
        for name in (
            "cluster_m",
            "min_reobserve_s",
            "min_confidence",
            "marker_candidate_ttl_s",
            "smog_speed_hint",
            "ice_speed_hint",
            "state_ttl_s",
        ):
            _finite(getattr(self, name), name)
        if (
            float(self.marker_candidate_ttl_s)
            < float(self.min_reobserve_s)
        ):
            raise ValueError(
                "marker_candidate_ttl_s must be at least min_reobserve_s"
            )
        if self.odom_gate_m is not None:
            gate = _finite(self.odom_gate_m, "odom_gate_m")
            if gate < 0.0:
                raise ValueError("odom_gate_m must be non-negative")


@dataclass(frozen=True)
class SectionJournalEvent:
    event_type: str
    stamp_s: float
    accepted: bool
    reason: str
    payload: Mapping

    def __post_init__(self) -> None:
        object.__setattr__(self, "payload", _frozen_mapping(self.payload))


@dataclass(frozen=True)
class SectionState:
    section: str
    phase: str
    drive_hold_hint: bool
    speed_hint: float | None
    work_request: str | None
    notices: tuple
    journal_events: tuple
    complete: bool


class SectionSupervisor:
    """Deterministic section progress evaluator with an injected tick time."""

    def __init__(self, profile, config=None):
        if isinstance(profile, str):
            try:
                profile = SECTION_PROFILES[profile]
            except KeyError:
                raise ValueError(
                    f"invalid section profile: {profile}"
                ) from None
        if not isinstance(profile, SectionProfile):
            raise ValueError(
                "profile must be a SectionProfile or section name"
            )
        self.profile = profile
        self.config = config or SectionConfig()
        if not isinstance(self.config, SectionConfig):
            raise ValueError("config must be SectionConfig")
        self.marker_dedup = MarkerDedup(
            cluster_m=self.config.cluster_m,
            min_reobserve_s=self.config.min_reobserve_s,
            min_confidence=self.config.min_confidence,
            marker_confirm_observations=(
                self.config.marker_confirm_observations
            ),
            marker_candidate_ttl_s=self.config.marker_candidate_ttl_s,
            marker_max_candidates=self.config.marker_max_candidates,
        )
        self._queue = []
        self._journal = []
        self._notices = []
        self._base_phase = "READY"
        self._profile_hold = False
        self._operator_hold = False
        self._recovery_requested = False
        self._complete = False
        self._speed_hint = None
        self._pending_work_request = None
        self._relief_work_requested = False
        self._smog_arm_results = set()
        self._odom_distance_m = None
        self._last_event_stamp_s = None

    @property
    def unique_markers(self) -> int:
        return self.marker_dedup.unique_count

    def submit(self, event) -> None:
        if not isinstance(event, SectionEvent):
            raise ValueError("submit requires SectionEvent")
        self._queue.append(event)

    def update_odom(self, distance_m) -> None:
        distance_m = _finite(distance_m, "distance_m")
        if distance_m < 0.0:
            raise ValueError("distance_m must be non-negative")
        self._odom_distance_m = distance_m

    def tick(self, now_s) -> SectionState:
        now_s = _finite(now_s, "now_s")
        if now_s < 0.0:
            raise ValueError("now_s must be non-negative")
        ready = [event for event in self._queue if event.stamp_s <= now_s]
        self._queue = [event for event in self._queue if event.stamp_s > now_s]
        for event in ready:
            if (
                self._last_event_stamp_s is not None
                and event.stamp_s < self._last_event_stamp_s
            ):
                accepted, reason = False, "stale_event_ignored"
            else:
                accepted, reason = self._process(event)
                self._last_event_stamp_s = event.stamp_s
            self._journal.append(
                SectionJournalEvent(
                    event_type=event.type,
                    stamp_s=event.stamp_s,
                    accepted=accepted,
                    reason=reason,
                    payload=event.payload,
                )
            )

        state = SectionState(
            section=self.profile.section,
            phase=self._phase(),
            drive_hold_hint=(
                False
                if self._complete
                else self._operator_hold or self._profile_hold
            ),
            speed_hint=self._speed_hint,
            work_request=self._pending_work_request,
            notices=tuple(self._notices),
            journal_events=tuple(self._journal),
            complete=self._complete,
        )
        self._journal.clear()
        self._pending_work_request = None
        return state

    def _phase(self) -> str:
        if self._complete:
            return "COMPLETE"
        if self._recovery_requested:
            return "RECOVERY_REQUESTED"
        if self._operator_hold or self._profile_hold:
            if self._base_phase == "STOP_REQUESTED":
                return "STOP_REQUESTED"
            return "EVENT_HOLD"
        return self._base_phase

    def _process(self, event):
        if event.type not in EVENT_TYPES:
            return False, "unknown_event"

        if event.type == OPERATOR_HOLD:
            self._operator_hold = True
            self._add_notice("operator_hold")
            return True, "operator_hold"
        if event.type == OPERATOR_RESUME:
            self._operator_hold = False
            self._remove_notice("operator_hold")
            return True, "operator_resume"
        if event.type == SECTION_ENTER:
            self._base_phase = "DRIVE"
            if self.profile.section == ICE:
                self._speed_hint = float(self.config.ice_speed_hint)
            return True, "section_enter"
        if event.type == SECTION_EXIT:
            if self._operator_hold:
                self._add_notice("section_exit_blocked:operator_hold")
                return False, "operator_hold_active"
            if self._recovery_requested:
                self._add_notice("section_exit_blocked:recovery_requested")
                return False, "recovery_requested"
            if self._profile_hold:
                self._add_notice("section_exit_blocked:profile_hold")
                return False, "profile_hold_active"
            return self._finish_section()

        section = self.profile.section
        if section == SMOG:
            return self._process_smog(event)
        if section == RELIEF:
            return self._process_relief(event)
        if section == MARKERS:
            return self._process_markers(event)
        if section == ICE:
            return self._process_ice(event)
        if section == FOLLOW:
            return self._process_follow(event)
        return False, "event_not_used_by_profile"

    def _process_smog(self, event):
        if event.type == SMOG_ENTER:
            self._base_phase = "DRIVE"
            self._speed_hint = float(self.config.smog_speed_hint)
            self._add_notice("smog_hold_policy_unconfirmed")
            return True, "speed_hint_only"
        if event.type == SMOG_EXIT:
            self._speed_hint = None
            self._remove_notice("smog_hold_policy_unconfirmed")
            return True, "smog_exit"
        if event.type == ARRIVAL_REACHED:
            result = str(event.payload.get("arm_result", "")).strip()
            if not result:
                return False, "smog_arm_result_missing"
            self._smog_arm_results.add(result)
            self._remove_notice_prefix("smog_arm_results_required:")
            return True, "smog_arm_result"
        return False, "event_not_used_by_profile"

    def _process_relief(self, event):
        if event.type == LIGHT_RED:
            self._profile_hold = True
            self._base_phase = "DRIVE"
            return True, "red_hold_hint"
        if event.type == LIGHT_GREEN:
            self._profile_hold = False
            self._base_phase = "DRIVE"
            return True, "green_resume_hint"
        if event.type == ARRIVAL_REACHED:
            request = str(
                event.payload.get(
                    "work_request",
                    self.config.relief_work_request,
                )
            ).strip()
            if not request:
                return False, "work_request_empty"
            self._pending_work_request = request
            self._relief_work_requested = True
            self._profile_hold = True
            self._base_phase = "STOP_REQUESTED"
            return True, "work_request_hint"
        return False, "event_not_used_by_profile"

    def _process_markers(self, event):
        if event.type != MARKER_DETECTED:
            return False, "event_not_used_by_profile"
        payload = event.payload
        accepted = self.marker_dedup.observe(
            class_id=payload.get("class_id"),
            class_name=payload.get("class_name", ""),
            position=payload.get("position", ()),
            confidence=payload.get("confidence", 0.0),
            stamp_s=event.stamp_s,
        )
        if accepted:
            return True, self.marker_dedup.successes[-1].reason
        return False, self.marker_dedup.failures[-1].reason

    def _process_ice(self, event):
        if event.type == STUCK_DETECTED:
            self._recovery_requested = True
            self._profile_hold = True
            self._add_notice("operator_alert:stuck_recovery_policy_pending")
            return True, "recovery_requested"
        if event.type == ARRIVAL_REACHED:
            return True, "arrival_progress"
        return False, "event_not_used_by_profile"

    def _process_follow(self, event):
        if event.type == LEAD_LOST:
            self._profile_hold = True
            self._base_phase = "DRIVE"
            return True, "lead_lost_hold_hint"
        if event.type == LEAD_FOUND:
            self._profile_hold = False
            self._base_phase = "DRIVE"
            return True, "lead_found_resume_hint"
        if event.type == ARRIVAL_REACHED:
            return True, "arrival_progress"
        return False, "event_not_used_by_profile"

    def _finish_section(self):
        section = self.profile.section
        if section == SMOG:
            count = len(self._smog_arm_results)
            required = self.config.smog_arm_result_count
            if count < required:
                self._remove_notice_prefix("smog_arm_results_required:")
                self._add_notice(
                    f"smog_arm_results_required:{count}/{required}"
                )
                return False, "smog_arm_results_pending"
        if section == RELIEF and not self._relief_work_requested:
            self._add_notice("relief_work_request_required")
            return False, "relief_work_request_pending"
        if (
            section == MARKERS
            and self.unique_markers < self.config.marker_target_count
        ):
            self._remove_notice_prefix("unique_markers_required:")
            self._add_notice(
                f"unique_markers_required:{self.unique_markers}/"
                f"{self.config.marker_target_count}"
            )
            return False, "marker_target_pending"

        self._complete = True
        self._profile_hold = False
        self._operator_hold = False
        self._recovery_requested = False
        self._speed_hint = None
        self._remove_notice_prefix("smog_arm_results_required:")
        self._remove_notice_prefix("unique_markers_required:")
        self._remove_notice("relief_work_request_required")
        self._remove_notice("operator_hold")
        if (
            self.config.odom_gate_m is not None
            and (
                self._odom_distance_m is None
                or self._odom_distance_m < self.config.odom_gate_m
            )
        ):
            self._add_notice("odom_aux_gate_unmet")
        return True, "section_complete"

    def _add_notice(self, notice):
        if notice not in self._notices:
            self._notices.append(notice)

    def _remove_notice(self, notice):
        if notice in self._notices:
            self._notices.remove(notice)

    def _remove_notice_prefix(self, prefix):
        self._notices[:] = [
            notice for notice in self._notices if not notice.startswith(prefix)
        ]
