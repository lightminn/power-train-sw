"""A2a ops 채널 와이어 계약 — 스펙 r6 §3.1의 권위 구현.

hello는 안정 client_id와 클라이언트 단조시각 stamp_s를 포함하며 서버가 인증
토큰+client_id 신원과 연결별 시각 오프셋을 고정한다. 요청 stamp_s는 같은
클라이언트 단조시각 축을 사용한다. 요청 =
newline-JSON {schema_version, token, request_id, sequence, action, params,
stamp_s [, expected_state_revision, phase]}. 응답 = {request_id,
status(PENDING/FINAL_SUCCESS/FINAL_REJECTED/OUTCOME_UNKNOWN), state_revision,
detail}. 역할 인가는 서버의 토큰→역할 매핑이 유일 근거다(client_type 없음).
"""
from dataclasses import dataclass, field
import json
import math

SCHEMA_VERSION = 1
DEFAULT_PORT = 9001
MAX_RECORD_BYTES = 4 * 1024
ROLE_CONSOLE = "console"
ROLE_CONTROLLER = "controller"
STATUS_PENDING = "PENDING"
STATUS_FINAL_SUCCESS = "FINAL_SUCCESS"
STATUS_FINAL_REJECTED = "FINAL_REJECTED"
STATUS_OUTCOME_UNKNOWN = "OUTCOME_UNKNOWN"
RETRANSMIT_INTERVAL_S = 0.25
REQUEST_DEADLINE_S = 2.0
REQUEST_FUTURE_SKEW_S = 0.25
SERVICE_CALL_TIMEOUT_S = 1.0
SERVICE_UNAVAILABLE_ABANDON_S = 3.0
SERVICE_ORDER_ABANDON_S = 10.0
assert (
    SERVICE_CALL_TIMEOUT_S
    < SERVICE_UNAVAILABLE_ABANDON_S
    < SERVICE_ORDER_ABANDON_S
)
# ⚠️ recovery-v1-initial-candidate — HIL·운전자 피드백 후 변경 전제(임시).
EMERGENCY_HOLD_S = {"estop_reset": 5.0, "arm": 3.0}
OPS_STATE_STALE_S = 0.5
_PHASES = {"begin", "execute"}
_REQUIRED = (
    "schema_version", "token", "request_id", "sequence", "action",
    "params", "stamp_s",
)
_OPTIONAL = ("expected_state_revision", "phase")


def service_abandon_timeout_s(*, service_was_ready):
    if service_was_ready:
        return SERVICE_ORDER_ABANDON_S
    return SERVICE_UNAVAILABLE_ABANDON_S


@dataclass(frozen=True)
class ActionSpec:
    roles: frozenset
    kind: str                      # composite | service | publish | local
    target: tuple = ()
    emergency_roles: frozenset = field(default_factory=frozenset)


_BOTH = frozenset({ROLE_CONSOLE, ROLE_CONTROLLER})
_CONSOLE = frozenset({ROLE_CONSOLE})
_CTRL_EMERGENCY = frozenset({ROLE_CONTROLLER})
_MISSIONS = (
    "mission_arrive_pickup", "mission_arrive_drop", "mission_skip",
    "mission_retry", "mission_regrasp_confirmed",
)

ACTIONS = {
    "clear_transient_hold": ActionSpec(_BOTH, "composite", (
        "/teleop_command/clear_hold", "/chassis_node/authority_clear_hold",
    )),
    "authority_manual": ActionSpec(
        _BOTH, "service", ("/chassis_node/authority_manual",)
    ),
    "authority_auto": ActionSpec(
        _BOTH, "service", ("/chassis_node/authority_auto",)
    ),
    "authority_idle": ActionSpec(
        _CONSOLE, "service", ("/chassis_node/authority_idle",)
    ),
    "estop_reset": ActionSpec(
        _CONSOLE, "service", ("/chassis_node/reset_estop",),
        emergency_roles=_CTRL_EMERGENCY,
    ),
    "estop": ActionSpec(
        _CONSOLE, "service", ("/chassis_node/estop",),
    ),
    "arm": ActionSpec(
        _CONSOLE, "service", ("/chassis_node/arm",),
        emergency_roles=_CTRL_EMERGENCY,
    ),
    "disarm": ActionSpec(_CONSOLE, "service", ("/chassis_node/disarm",)),
    "arm_lock_override": ActionSpec(
        _CONSOLE, "service_setbool",
        ("/chassis_node/arm_lock_override",),
    ),
    "mission_clear_grip_lost": ActionSpec(
        _CONSOLE, "service_setbool",
        ("/chassis_node/mission_clear_grip_lost",),
    ),
    "drive_enable": ActionSpec(
        _CONSOLE, "service_setbool",
        ("/chassis_node/component_enable_drive",),
    ),
    "steer_enable": ActionSpec(
        _CONSOLE, "service_setbool",
        ("/chassis_node/component_enable_steer",),
    ),
    "us100_enable": ActionSpec(
        _CONSOLE, "service_setbool",
        ("/chassis_node/component_enable_us100",),
    ),
    "robot_arm_enable": ActionSpec(
        _CONSOLE, "service_setbool",
        ("/chassis_node/component_enable_robot_arm",),
    ),
    "extraction_grant": ActionSpec(
        _CONSOLE, "service", ("/chassis_node/extraction_grant",)
    ),
    "operator_hold": ActionSpec(_CONSOLE, "publish", ("/section_events",)),
    "operator_resume": ActionSpec(_CONSOLE, "publish", ("/section_events",)),
    "status_query": ActionSpec(_BOTH, "local"),
    # A2c: calibration_*
}
for _name in _MISSIONS:
    ACTIONS[_name] = ActionSpec(
        _CONSOLE, "service", ("/chassis_node/%s" % _name,)
    )


def decode_request(line):
    if len(line.encode("utf-8", errors="replace")) > MAX_RECORD_BYTES:
        raise ValueError("record exceeds %d bytes" % MAX_RECORD_BYTES)
    try:
        payload = json.loads(line)
    except (TypeError, ValueError) as exc:
        raise ValueError("invalid JSON: %s" % exc) from exc
    if not isinstance(payload, dict):
        raise ValueError("request must be a JSON object")
    unknown = set(payload) - set(_REQUIRED) - set(_OPTIONAL)
    if unknown:
        raise ValueError("unknown fields: %s" % sorted(unknown))
    missing = [key for key in _REQUIRED if key not in payload]
    if missing:
        raise ValueError("missing fields: %s" % missing)
    if payload["schema_version"] != SCHEMA_VERSION:
        raise ValueError(
            "unrecognized schema_version: %r" % payload["schema_version"]
        )
    if payload["action"] not in ACTIONS:
        raise ValueError("unknown action: %r" % payload["action"])
    if not isinstance(payload["request_id"], str) or not payload["request_id"]:
        raise ValueError("request_id must be a non-empty string")
    sequence = payload["sequence"]
    if not isinstance(sequence, int) or isinstance(sequence, bool) \
            or sequence < 0:
        raise ValueError("sequence must be a non-negative integer")
    if not isinstance(payload["token"], str) or not payload["token"]:
        raise ValueError("token must be a non-empty string")
    if not isinstance(payload["params"], dict):
        raise ValueError("params must be an object")
    try:
        stamp_s = float(payload["stamp_s"])
    except (TypeError, ValueError) as exc:
        raise ValueError("stamp_s must be finite") from exc
    if not math.isfinite(stamp_s):
        raise ValueError("stamp_s must be finite")
    payload["stamp_s"] = stamp_s
    if "phase" in payload and payload["phase"] not in _PHASES:
        raise ValueError("phase must be 'begin' or 'execute'")
    if "expected_state_revision" in payload:
        revision = payload["expected_state_revision"]
        if not isinstance(revision, int) or isinstance(revision, bool) \
                or revision < 0:
            raise ValueError("expected_state_revision must be >= 0 int")
    return payload


def encode_response(*, request_id, status, state_revision, detail=""):
    return (
        json.dumps(
            {
                "request_id": str(request_id),
                "status": str(status),
                "state_revision": int(state_revision),
                "detail": str(detail),
            },
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        )
        + "\n"
    ).encode("utf-8")
