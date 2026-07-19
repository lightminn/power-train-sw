"""Pure verdicts for the WP8 chassis-to-arm handshake scenarios."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from powertrain_ros import contract


TOPIC_CHASSIS_MODE = "chassis_mode"
TOPIC_ARRIVAL = "arrival"
TOPIC_ARM_STATUS = "arm_status"

FAULT_SCENARIOS = {"no_response", "late_done", "failed_latch", "dup_done"}
PICKUP_BRANCHES = {"work_accepted", "fail_closed", "violation"}

# MISSION_STOP 모드와 ArrivalStatus는 같은 supervisor tick에서 함께 발행되며
# 토픽 간 DDS 전달 순서는 보장되지 않는다(07-19 실기에서 24 ms skew 실측).
# 이 허용치 안에서 주행(LOCK) 모드가 끼지 않으면 동시 활성으로 인정한다.
STOP_ARRIVAL_SKEW_TOL_S = 0.5


@dataclass(frozen=True)
class Event:
    t: float
    topic: str
    value: str
    mission_id: int | None = None


@dataclass(frozen=True)
class Finding:
    check: str
    ok: bool
    detail: str


def _ordered(events: Iterable[Event]) -> list[Event]:
    return sorted(events, key=lambda event: event.t)


def _first(events: Iterable[Event], predicate) -> Event | None:
    return next((event for event in events if predicate(event)), None)


def _drive_modes_after(events: Iterable[Event], t: float) -> list[Event]:
    return [
        event
        for event in events
        if event.t > t
        and event.topic == TOPIC_CHASSIS_MODE
        and event.value in contract.LOCK_MODES
    ]


def _resume_transition_count(events: Iterable[Event], *, after_t: float) -> int:
    count = 0
    previous_mode = None
    for event in events:
        if event.t <= after_t or event.topic != TOPIC_CHASSIS_MODE:
            continue
        is_drive = event.value in contract.LOCK_MODES
        if is_drive and previous_mode not in contract.LOCK_MODES:
            count += 1
        previous_mode = event.value
    return count


def judge_baseline(
    events: Iterable[Event],
    *,
    window_s: float,
    min_heartbeat_hz: float = 5.0,
) -> list[Finding]:
    """Check idle heartbeat, pre-arrival lock, and absence of early work."""
    window_s = float(window_s)
    min_heartbeat_hz = float(min_heartbeat_hz)
    if (
        not math.isfinite(window_s)
        or window_s <= 0.0
        or not math.isfinite(min_heartbeat_hz)
        or min_heartbeat_hz < 0.0
    ):
        raise ValueError("window_s must be positive and heartbeat rate nonnegative")

    ordered = _ordered(events)
    window_end = ordered[-1].t if ordered else window_s
    window_start = window_end - window_s
    window = [event for event in ordered if window_start <= event.t <= window_end]
    arm_events = [event for event in window if event.topic == TOPIC_ARM_STATUS]
    observed_hz = len(arm_events) / window_s
    premature = [
        event
        for event in arm_events
        if event.value in contract.WORK_ACCEPTED_STATUSES
    ]
    observed_modes = [
        event
        for event in window
        if event.topic == TOPIC_CHASSIS_MODE
    ]
    non_work_modes = contract.LOCK_MODES | {contract.MODE_STOW_REQUEST}
    has_non_work_mode = any(
        event.value in non_work_modes for event in observed_modes
    )
    has_mission_stop = any(
        event.value == contract.MODE_MISSION_STOP for event in observed_modes
    )
    return [
        Finding(
            "arm_heartbeat",
            observed_hz >= min_heartbeat_hz,
            "observed=%.3fHz required=%.3fHz samples=%d"
            % (observed_hz, min_heartbeat_hz, len(arm_events)),
        ),
        Finding(
            "no_premature_work",
            not premature,
            "work_accepted_before_arrival=%d" % len(premature),
        ),
        Finding(
            "non_work_mode",
            has_non_work_mode and not has_mission_stop,
            "observed=%s"
            % ([event.value for event in observed_modes] or "none"),
        ),
    ]


def _pickup_evidence(events: Iterable[Event]) -> dict[str, object]:
    ordered = _ordered(events)
    stop = _first(
        ordered,
        lambda event: event.topic == TOPIC_CHASSIS_MODE
        and event.value == contract.MODE_MISSION_STOP,
    )
    arrivals = [
        event
        for event in ordered
        if event.topic == TOPIC_ARRIVAL
        and event.value == contract.ARRIVED_PICKUP
    ]
    arrival = arrivals[0] if arrivals else None
    # MISSION_STOP 모드와 arrival은 같은 supervisor tick에서 함께 발행되고,
    # 서로 다른 토픽 간 DDS 전달 순서는 보장되지 않는다(07-19 실기 24 ms skew).
    # 강제 가능한 계약은 "동시 활성"이므로, 사이에 주행(LOCK) 모드가 관측되지
    # 않는 작은 skew는 허용한다.
    stop_covers_arrival = False
    if stop is not None and arrival is not None:
        if stop.t < arrival.t:
            stop_covers_arrival = True
        elif stop.t - arrival.t <= STOP_ARRIVAL_SKEW_TOL_S:
            drive_between = any(
                event.topic == TOPIC_CHASSIS_MODE
                and event.value in contract.LOCK_MODES
                and arrival.t <= event.t <= stop.t
                for event in ordered
            )
            stop_covers_arrival = not drive_between
    arrival_ids = {event.mission_id for event in arrivals}
    ids_consistent = (
        bool(arrivals)
        and None not in arrival_ids
        and len(arrival_ids) == 1
    )
    mission_id = arrival.mission_id if arrival is not None else None
    accepted = [
        event
        for event in ordered
        if arrival is not None
        and event.t > arrival.t
        and event.topic == TOPIC_ARM_STATUS
        and event.value in contract.WORK_ACCEPTED_STATUSES
        and event.mission_id == mission_id
    ]
    any_work_accepted = any(
        event.topic == TOPIC_ARM_STATUS
        and event.value in contract.WORK_ACCEPTED_STATUSES
        for event in ordered
    )
    rejection_markers = [
        event
        for event in ordered
        if event.topic == "marker"
        and (
            event.value.startswith("arm_rejected:")
            or event.value.startswith("arrival_rejected:")
        )
    ]
    fail_closed = (
        not any_work_accepted
        and not any(event.topic == TOPIC_ARRIVAL for event in ordered)
        and stop is None
        and bool(rejection_markers)
    )
    work_accepted = stop_covers_arrival and ids_consistent and bool(accepted)
    return {
        "stop": stop,
        "arrival": arrival,
        "arrival_ids": arrival_ids,
        "mission_id": mission_id,
        "accepted": accepted,
        "stop_covers_arrival": stop_covers_arrival,
        "ids_consistent": ids_consistent,
        "any_work_accepted": any_work_accepted,
        "rejection_markers": rejection_markers,
        "fail_closed": fail_closed,
        "work_accepted": work_accepted,
    }


def pickup_branch(events: Iterable[Event]) -> str:
    """Classify pickup evidence as accepted work, accepted fail-closed, or violation."""
    evidence = _pickup_evidence(events)
    if evidence["work_accepted"]:
        return "work_accepted"
    if evidence["fail_closed"]:
        return "fail_closed"
    return "violation"


def judge_pickup_conjunction(events: Iterable[Event]) -> list[Finding]:
    """Accept either the work conjunction or an explicit fail-closed rejection."""
    evidence = _pickup_evidence(events)
    if evidence["fail_closed"]:
        rejection_markers = evidence["rejection_markers"]
        return [
            Finding(
                "no_work_accepted",
                not evidence["any_work_accepted"],
                "work_accepted=none",
            ),
            Finding(
                "no_arrival_status",
                True,
                "arrival_messages=0",
            ),
            Finding(
                "no_mission_stop",
                True,
                "mission_stop=none",
            ),
            Finding(
                "service_rejected",
                bool(rejection_markers),
                "markers=%s"
                % ([event.value for event in rejection_markers] or "none"),
            ),
        ]

    stop = evidence["stop"]
    arrival = evidence["arrival"]
    accepted = evidence["accepted"]
    mission_id = evidence["mission_id"]
    arrival_ids = evidence["arrival_ids"]
    return [
        Finding(
            "stop_covers_arrival",
            bool(evidence["stop_covers_arrival"]),
            "stop_t=%s arrival_t=%s"
            % (
                "missing" if stop is None else "%.6f" % stop.t,
                "missing" if arrival is None else "%.6f" % arrival.t,
            ),
        ),
        Finding(
            "work_accepted",
            bool(accepted),
            "mission_id=%s statuses=%s"
            % (mission_id, [event.value for event in accepted] or "none"),
        ),
        Finding(
            "arrival_mission_id",
            bool(evidence["ids_consistent"]),
            "mission_ids=%s" % sorted(str(value) for value in arrival_ids),
        ),
    ]


def judge_resume(
    events: Iterable[Event],
    *,
    resume_t: float,
) -> list[Finding]:
    """Use only post-SIGCONT evidence for conjunction re-acceptance."""
    accepted = [
        event
        for event in _ordered(events)
        if event.t > resume_t
        and event.topic == TOPIC_ARM_STATUS
        and event.value in contract.WORK_ACCEPTED_STATUSES
    ]
    return [
        Finding(
            "work_reaccepted",
            bool(accepted),
            "resume_t=%.6f first_accept_t=%s"
            % (
                resume_t,
                "missing" if not accepted else "%.6f" % accepted[0].t,
            ),
        )
    ]


def _next_event(events: list[Event], start: int, predicate) -> tuple[int, Event | None]:
    for index in range(start, len(events)):
        if predicate(events[index]):
            return index, events[index]
    return len(events), None


def judge_full_cycle(events: Iterable[Event]) -> list[Finding]:
    """Check the two authoritative completion cycles and mission ordering."""
    ordered = _ordered(events)
    cursor, pickup_stop = _next_event(
        ordered,
        0,
        lambda event: event.topic == TOPIC_CHASSIS_MODE
        and event.value == contract.MODE_MISSION_STOP,
    )
    cursor, pickup_arrival = _next_event(
        ordered,
        cursor + 1,
        lambda event: event.topic == TOPIC_ARRIVAL
        and event.value == contract.ARRIVED_PICKUP,
    )
    pickup_id = None if pickup_arrival is None else pickup_arrival.mission_id
    cursor, pickup_done = _next_event(
        ordered,
        cursor + 1,
        lambda event: event.topic == TOPIC_ARM_STATUS
        and event.value == contract.ARM_CARRYING_LOCKED
        and event.mission_id == pickup_id,
    )
    cursor, pickup_resume = _next_event(
        ordered,
        cursor + 1,
        lambda event: event.topic == TOPIC_CHASSIS_MODE
        and event.value in contract.LOCK_MODES,
    )
    pickup_ok = all(
        event is not None
        for event in (pickup_stop, pickup_arrival, pickup_done, pickup_resume)
    )

    cursor, drop_stop = _next_event(
        ordered,
        cursor + 1,
        lambda event: event.topic == TOPIC_CHASSIS_MODE
        and event.value == contract.MODE_MISSION_STOP,
    )
    cursor, drop_arrival = _next_event(
        ordered,
        cursor + 1,
        lambda event: event.topic == TOPIC_ARRIVAL
        and event.value == contract.ARRIVED_DROP,
    )
    drop_id = None if drop_arrival is None else drop_arrival.mission_id
    cursor, drop_done = _next_event(
        ordered,
        cursor + 1,
        lambda event: event.topic == TOPIC_ARM_STATUS
        and event.value == contract.ARM_STOWED_LOCKED
        and event.mission_id == drop_id,
    )
    cursor, drop_resume = _next_event(
        ordered,
        cursor + 1,
        lambda event: event.topic == TOPIC_CHASSIS_MODE
        and event.value in contract.LOCK_MODES,
    )
    drop_ok = all(
        event is not None
        for event in (drop_stop, drop_arrival, drop_done, drop_resume)
    )
    ids_monotonic = (
        isinstance(pickup_id, int)
        and not isinstance(pickup_id, bool)
        and isinstance(drop_id, int)
        and not isinstance(drop_id, bool)
        and drop_id > pickup_id
    )
    return [
        Finding(
            "pickup_cycle",
            pickup_ok,
            "stop=%s arrival=%s carrying_locked=%s resume=%s"
            % tuple(
                "yes" if event is not None else "no"
                for event in (pickup_stop, pickup_arrival, pickup_done, pickup_resume)
            ),
        ),
        Finding(
            "drop_cycle",
            drop_ok,
            "stop=%s arrival=%s stowed_locked=%s resume=%s"
            % tuple(
                "yes" if event is not None else "no"
                for event in (drop_stop, drop_arrival, drop_done, drop_resume)
            ),
        ),
        Finding(
            "mission_id_monotonic",
            ids_monotonic,
            "pickup=%s drop=%s" % (pickup_id, drop_id),
        ),
    ]


def judge_fault(
    events: Iterable[Event],
    *,
    scenario: str,
) -> list[Finding]:
    """Judge one of the four contract-v2 fault injections."""
    if scenario not in FAULT_SCENARIOS:
        raise ValueError("unsupported fault scenario: %s" % scenario)
    ordered = _ordered(events)

    if scenario == "no_response":
        arrivals = [
            event for event in ordered if event.topic == TOPIC_ARRIVAL
        ]
        first_arrival_t = arrivals[0].t if arrivals else math.inf
        modes_after = [
            event
            for event in ordered
            if event.t > first_arrival_t and event.topic == TOPIC_CHASSIS_MODE
        ]
        last_mode = modes_after[-1].value if modes_after else None
        no_resume = not _drive_modes_after(ordered, first_arrival_t)
        return [
            Finding(
                "arrival_republish",
                len(arrivals) >= 2,
                "count=%d" % len(arrivals),
            ),
            Finding(
                "no_resume",
                no_resume and last_mode == contract.MODE_MISSION_STOP,
                "drive_modes=%d last_mode=%s"
                % (len(_drive_modes_after(ordered, first_arrival_t)), last_mode),
            ),
        ]

    arrival = _first(ordered, lambda event: event.topic == TOPIC_ARRIVAL)
    active_id = None if arrival is None else arrival.mission_id

    if scenario == "late_done":
        late = _first(
            ordered,
            lambda event: arrival is not None
            and event.t > arrival.t
            and event.topic == TOPIC_ARM_STATUS
            and event.value in contract.DRIVE_READY_STATUSES
            and event.mission_id != active_id,
        )
        late_t = math.inf if late is None else late.t
        return [
            Finding(
                "previous_mission_completion",
                late is not None,
                "active=%s received=%s"
                % (active_id, None if late is None else late.mission_id),
            ),
            Finding(
                "no_resume",
                late is not None and not _drive_modes_after(ordered, late_t),
                "drive_modes=%d" % len(_drive_modes_after(ordered, late_t)),
            ),
        ]

    if scenario == "failed_latch":
        failed = _first(
            ordered,
            lambda event: event.topic == TOPIC_ARM_STATUS
            and event.value == contract.ARM_FAILED,
        )
        failed_t = math.inf if failed is None else failed.t
        return [
            Finding(
                "failed_observed",
                failed is not None,
                "mission_id=%s" % (None if failed is None else failed.mission_id),
            ),
            Finding(
                "no_resume",
                failed is not None and not _drive_modes_after(ordered, failed_t),
                "drive_modes=%d" % len(_drive_modes_after(ordered, failed_t)),
            ),
        ]

    completions = [
        event
        for event in ordered
        if event.topic == TOPIC_ARM_STATUS
        and event.value in contract.DRIVE_READY_STATUSES
    ]
    first_completion_t = completions[0].t if completions else math.inf
    duplicate = (
        len(completions) >= 2
        and completions[0].value == completions[1].value
        and completions[0].mission_id == completions[1].mission_id
    )
    resume_count = _resume_transition_count(
        ordered,
        after_t=first_completion_t,
    )
    return [
        Finding(
            "duplicate_completion",
            duplicate,
            "count=%d" % len(completions),
        ),
        Finding(
            "single_resume",
            resume_count == 1,
            "resume_transitions=%d" % resume_count,
        ),
    ]


def summarize(findings: Iterable[Finding]) -> tuple[bool, str]:
    """Return the aggregate verdict and a compact human-readable table."""
    rows = list(findings)
    passed = all(finding.ok for finding in rows)
    lines = ["판정 | 결과 | 상세", "--- | --- | ---"]
    lines.extend(
        "%s | %s | %s"
        % (finding.check, "PASS" if finding.ok else "FAIL", finding.detail)
        for finding in rows
    )
    return passed, "\n".join(lines)
