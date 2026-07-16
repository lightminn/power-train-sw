"""Pure formatting for chassis-owner CAN health shown on the console."""
from __future__ import annotations

from collections.abc import Mapping


_STALE_AFTER_NS = 3_000_000_000
_MALFORMED = "UNAVAILABLE · malformed CAN_HEALTH"


def _node_availability(nodes: object) -> tuple[int, int]:
    if not isinstance(nodes, (list, tuple)):
        raise TypeError("CAN node health must be a list")
    available = 0
    for node in nodes:
        if not isinstance(node, Mapping):
            raise TypeError("CAN node health entry must be a mapping")
        stale = node["stale"]
        if type(stale) is not bool:
            raise TypeError("CAN node stale flag must be boolean")
        available += not stale
    return available, len(nodes)


def can_status_text(
    record: Mapping[str, object] | None,
    now_monotonic_ns: int,
) -> str:
    """Render one daemon-cached CAN health event without ROS."""
    if record is None:
        return "UNAVAILABLE · no CAN_HEALTH from chassis owner"
    if not isinstance(record, Mapping):
        return _MALFORMED

    try:
        monotonic_ns = record["monotonic_ns"]
        if type(monotonic_ns) is not int or type(now_monotonic_ns) is not int:
            raise TypeError("monotonic timestamps must be integers")
        age_ns = now_monotonic_ns - monotonic_ns
    except (KeyError, TypeError):
        return _MALFORMED

    if age_ns < 0 or age_ns > _STALE_AFTER_NS:
        return f"UNAVAILABLE · CAN_HEALTH stale {age_ns / 1_000_000_000:.1f}s"

    try:
        payload = record["payload"]
        if not isinstance(payload, Mapping):
            raise TypeError("CAN_HEALTH payload must be a mapping")
        healthy = payload["healthy"]
        if type(healthy) is not bool:
            raise TypeError("CAN_HEALTH healthy flag must be boolean")
        ak_ok, ak_total = _node_availability(payload["ak_nodes"])
        odrive_ok, odrive_total = _node_availability(payload["odrive_nodes"])

        bus = payload["bus"]
        if not isinstance(bus, Mapping):
            raise TypeError("CAN_HEALTH bus field must be a mapping")
        error_warning = bus["error_warning"]
        error_passive = bus["error_passive"]
        bus_off_delta = bus["bus_off_delta"]
        if type(error_warning) is not bool or type(error_passive) is not bool:
            raise TypeError("CAN bus flags must be boolean")
        if type(bus_off_delta) is not int or bus_off_delta < 0:
            raise TypeError("CAN bus-off delta must be a non-negative integer")
    except (KeyError, TypeError):
        return _MALFORMED

    state = "HEALTHY" if healthy else "UNHEALTHY"
    text = (
        f"{state} · AK {ak_ok}/{ak_total}"
        f" · ODrive {odrive_ok}/{odrive_total}"
    )
    flags = []
    if error_warning:
        flags.append("WARN")
    if error_passive:
        flags.append("PASSIVE")
    if bus_off_delta > 0:
        flags.append(f"BUSOFF+{bus_off_delta}")
    if flags:
        text += f" · bus {','.join(flags)}"
    return text
