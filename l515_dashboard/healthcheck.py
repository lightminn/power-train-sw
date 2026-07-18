"""Container healthcheck for the live L515 Gateway production path."""

from collections.abc import Mapping
import math
from numbers import Real
import sys

from .client import GatewayClient
from .config import DashboardConfig
from .diagnostics import ALL_TOPICS, FRESHNESS_S


class GatewayHealthError(RuntimeError):
    pass


def validate_gateway_health(status) -> None:
    if not isinstance(status, Mapping):
        raise GatewayHealthError("Gateway status is not an object")
    if status.get("state") != "RUNNING":
        raise GatewayHealthError(f"Gateway state is {status.get('state')!r}")
    sdk = status.get("sdk")
    if not isinstance(sdk, Mapping) or sdk.get("source_state") != "streaming":
        source_state = sdk.get("source_state") if isinstance(sdk, Mapping) else None
        raise GatewayHealthError(f"L515 source state is {source_state!r}")
    diagnostics = status.get("diagnostics")
    if not isinstance(diagnostics, Mapping):
        raise GatewayHealthError("L515 diagnostics are missing")
    for topic in ALL_TOPICS:
        metric = diagnostics.get(topic)
        age_s = metric.get("age_s") if isinstance(metric, Mapping) else None
        if (
            isinstance(age_s, bool)
            or not isinstance(age_s, Real)
            or not math.isfinite(age_s)
            or age_s < 0
            or age_s > FRESHNESS_S[topic]
        ):
            raise GatewayHealthError(
                f"L515 stream {topic} is stale: age_s={age_s!r}"
            )


def main() -> int:
    config = DashboardConfig()
    try:
        snapshot = GatewayClient(
            config.socket_path,
            max_message_bytes=config.max_message_bytes,
            request_timeout_s=1.0,
        ).request("get_status")
        validate_gateway_health(snapshot.payload)
    except Exception as exc:
        print(f"L515 Gateway unhealthy: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
