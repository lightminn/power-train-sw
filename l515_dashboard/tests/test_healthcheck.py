import pytest

from l515_dashboard.diagnostics import ALL_TOPICS, FRESHNESS_S
from l515_dashboard.healthcheck import GatewayHealthError, validate_gateway_health


def healthy_status():
    return {
        "state": "RUNNING",
        "sdk": {"source_state": "streaming"},
        "diagnostics": {
            topic: {"age_s": FRESHNESS_S[topic]}
            for topic in ALL_TOPICS
        },
    }


def test_gateway_health_requires_running_streaming_and_every_fresh_topic():
    validate_gateway_health(healthy_status())


def test_gateway_health_rejects_disconnected_source():
    status = healthy_status()
    status["sdk"]["source_state"] = "disconnected"

    with pytest.raises(GatewayHealthError, match="source"):
        validate_gateway_health(status)


def test_gateway_health_rejects_one_stale_stream():
    status = healthy_status()
    topic = "/l515/depth/image_rect_raw"
    status["diagnostics"][topic]["age_s"] = FRESHNESS_S[topic] + 0.001

    with pytest.raises(GatewayHealthError, match="depth"):
        validate_gateway_health(status)
