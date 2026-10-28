import json

import gi
import pytest

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk

from operator_console.mobility_view import MobilityDashboard
from operator_console.obstacle_telemetry import parse_obstacle_telemetry
from operator_console.telemetry import parse_telemetry
from operator_console.tests.test_interactions import _chassis_payload


requires_gtk = pytest.mark.skipif(
    not Gtk.init_check()[0], reason="GTK display required",
)


def obstacle_payload(state="OK", *, zones=None):
    if zones is None:
        zones = {
            "left": {"valid": True, "detected": False, "distance_m": None},
            "center": {"valid": True, "detected": True, "distance_m": 1.42},
            "right": {"valid": True, "detected": False, "distance_m": None},
        }
    return {
        "schema_version": 1,
        "sequence": 12,
        "capture_timestamp_s": 123.0,
        "state": state,
        "zones": zones,
        "processing_ms": 17.0,
        "detail": "",
    }


def parsed_chassis():
    return parse_telemetry(
        json.dumps(_chassis_payload(1)).encode(), received_monotonic_s=10.0,
    )


@requires_gtk
def test_obstacle_zones_replace_legacy_summary_when_live():
    dashboard = MobilityDashboard(lambda _action: None)
    obstacle = parse_obstacle_telemetry(
        json.dumps(obstacle_payload()).encode(), received_monotonic_s=10.0,
    )
    dashboard.update(parsed_chassis(), ops_state=None, obstacle=obstacle, now_s=10.1)
    labels = dashboard._terrain[1]
    assert labels["path"].get_text() == "최근접 장애물 1.42 m"
    assert "좌 · 관측 범위 내 미검출" in labels["geometry"].get_text()
    assert "중앙 · 1.42 m" in labels["geometry"].get_text()


@requires_gtk
@pytest.mark.parametrize(
    ("state", "expected"),
    [("NO_GROUND", "바닥 인지 불가"), ("SENSOR_ERROR", "L515 인지 오류")],
)
def test_failure_is_never_presented_as_no_obstacle(state, expected):
    zones = {
        name: {"valid": False, "detected": None, "distance_m": None}
        for name in ("left", "center", "right")
    }
    obstacle = parse_obstacle_telemetry(
        json.dumps(obstacle_payload(state, zones=zones)).encode(),
        received_monotonic_s=10.0,
    )
    dashboard = MobilityDashboard(lambda _action: None)
    dashboard.update(parsed_chassis(), ops_state=None, obstacle=obstacle, now_s=10.1)
    assert expected in dashboard._terrain[1]["path"].get_text()
    assert "인지 불가" in dashboard._terrain[1]["geometry"].get_text()


@requires_gtk
def test_stale_result_clears_previous_distance():
    obstacle = parse_obstacle_telemetry(
        json.dumps(obstacle_payload()).encode(), received_monotonic_s=10.0,
    )
    dashboard = MobilityDashboard(lambda _action: None)
    dashboard.update(parsed_chassis(), ops_state=None, obstacle=obstacle, now_s=11.0)
    labels = dashboard._terrain[1]
    assert labels["path"].get_text() == "장애물 인지 갱신 지연"
    assert "1.42" not in labels["geometry"].get_text()
