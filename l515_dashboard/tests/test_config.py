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
    ],
)
def test_rejects_invalid_runtime_configuration(kwargs, message):
    with pytest.raises(ValueError, match=message):
        DashboardConfig(**kwargs)


@pytest.mark.parametrize(
    "field",
    ["port", "latency_ms", "width", "height", "fps", "bitrate_kbps"],
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
    ],
)
@pytest.mark.parametrize("value", [float("nan"), float("inf")])
def test_rejects_nonfinite_positive_numeric_fields(field, value):
    with pytest.raises(ValueError, match=field):
        DashboardConfig(**{field: value})
