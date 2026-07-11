from dataclasses import FrozenInstanceError

import pytest

from l515_dashboard.config import DashboardConfig


def test_defaults_match_l515_stream_contract():
    config = DashboardConfig()

    assert (config.port, config.latency_ms, config.encoder) == (5000, 60, "x264")
    assert (config.width, config.height, config.fps) == (1280, 720, 30)
    assert config.bitrate_kbps == 3000
    assert config.startup_timeout_s == 10.0
    assert config.graceful_timeout_s == 3.0
    assert config.termination_timeout_s == 2.0
    assert config.socket_path == "/run/powertrain/l515-gateway.sock"
    assert config.lock_path == "/run/powertrain/l515-gateway.lock"
    assert (config.color_width, config.color_height) == (1280, 720)
    assert (config.depth_width, config.depth_height) == (640, 480)
    assert config.overlay_alpha == 0.5
    assert config.reconnect_interval_s == 2.0
    assert config.max_message_bytes == 65536


def test_config_is_immutable():
    config = DashboardConfig()

    with pytest.raises(FrozenInstanceError):
        config.port = 5001


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"port": 0}, "port"),
        ({"encoder": "unknown"}, "encoder"),
        ({"startup_timeout_s": 0}, "startup_timeout_s"),
        ({"graceful_timeout_s": -1}, "graceful_timeout_s"),
        ({"termination_timeout_s": 0}, "termination_timeout_s"),
        ({"overlay_alpha": 0}, "overlay_alpha"),
        ({"overlay_alpha": 1.1}, "overlay_alpha"),
        ({"reconnect_interval_s": 0}, "reconnect_interval_s"),
        ({"max_message_bytes": 0}, "max_message_bytes"),
        ({"socket_path": ""}, "socket_path"),
    ],
)
def test_rejects_invalid_runtime_configuration(kwargs, message):
    with pytest.raises(ValueError, match=message):
        DashboardConfig(**kwargs)


@pytest.mark.parametrize(
    "field",
    ["port", "latency_ms", "width", "height", "fps", "bitrate_kbps",
     "color_width", "color_height", "depth_width", "depth_height",
     "max_message_bytes"],
)
@pytest.mark.parametrize("value", [1.5, True, "1"])
def test_rejects_noninteger_fields(field, value):
    with pytest.raises(ValueError, match=field):
        DashboardConfig(**{field: value})


@pytest.mark.parametrize(
    "field",
    [
        "latency_ms",
        "width",
        "height",
        "fps",
        "bitrate_kbps",
        "startup_timeout_s",
        "graceful_timeout_s",
        "termination_timeout_s",
        "overlay_alpha",
        "reconnect_interval_s",
    ],
)
@pytest.mark.parametrize("value", [float("nan"), float("inf")])
def test_rejects_nonfinite_positive_numeric_fields(field, value):
    with pytest.raises(ValueError, match=field):
        DashboardConfig(**{field: value})


@pytest.mark.parametrize("field", ["socket_path", "lock_path"])
@pytest.mark.parametrize("value", [None, 1, True])
def test_rejects_nonstring_paths(field, value):
    with pytest.raises(ValueError, match=field):
        DashboardConfig(**{field: value})


def test_rejects_nonfixed_stream_profiles():
    with pytest.raises(ValueError, match="color"):
        DashboardConfig(color_width=640)
    with pytest.raises(ValueError, match="depth"):
        DashboardConfig(depth_height=720)
