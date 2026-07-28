from pathlib import Path

from operator_console.status_view import (
    GRAPH_WINDOW_S,
    MAX_GRAPH_SAMPLES,
    RobotStatusDashboard,
    TimedSeries,
)


def test_timed_series_is_bounded_by_count_and_time_window():
    series = TimedSeries()
    for index in range(MAX_GRAPH_SAMPLES + 50):
        series.append(float(index) / 5.0, float(index))

    samples = series.samples()
    assert len(samples) <= MAX_GRAPH_SAMPLES
    assert samples[-1].timestamp_s - samples[0].timestamp_s <= GRAPH_WINDOW_S


def test_timed_series_keeps_explicit_stale_gap():
    series = TimedSeries()
    series.append(1.0, 2.0)
    series.mark_gap(2.0)
    series.mark_gap(3.0)

    assert [sample.value for sample in series.samples()] == [2.0, None]


def test_status_view_defaults_match_operator_view_options():
    assert RobotStatusDashboard.DEFAULT_VISIBLE == {
        "drive": True,
        "power": True,
        "arm": False,
        "safety": False,
        "ai": False,
        "network": False,
    }


def test_status_dashboard_has_no_control_or_transport_send_surface():
    source = (
        Path(__file__).resolve().parents[1] / "status_view.py"
    ).read_text(encoding="utf-8")

    assert "OpsClient" not in source
    assert "submit(" not in source
    assert "sendto(" not in source
    assert "component_mask[" not in source


def test_mission_display_options_only_change_overlay_visibility():
    source = (
        Path(__file__).resolve().parents[1] / "app.py"
    ).read_text(encoding="utf-8")

    assert "def set_display_options(" in source
    assert "self._show_objects" in source
    assert "self._show_distance" in source
    assert "set_metadata_display_options(" in source
    assert "YOLO Node" not in source
    assert "metadata_receiver.close()" not in source[
        source.index("def _on_display_option_toggled"):
        source.index("def _on_video_area_allocated")
    ]
