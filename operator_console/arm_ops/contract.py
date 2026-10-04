"""Console-side request/response contract for robot-arm ops commands.

Scope of this file
------------------
It fixes **what the console may ask for**: the action names, their parameter
shapes, and the validation every arm request passes before it can exist as an
object.  It does not fix, and must not be read as fixing, the arm side: which
ROS topic, service or action carries each command, what the payload looks like
on that side, or what a jog actually does to a joint.  Those live in
:data:`BACKEND_REQUIREMENTS` as explicitly unresolved obligations.

Why the actions are not registered yet
--------------------------------------
Real commands leave this console through exactly one door: the token-gated ops
channel, whose ``ops_contract.ACTIONS`` is a closed dict — the broker rejects
any action not in it.  None of the arm actions below are in it, and this task
deliberately does not add them.  The consequence is a useful safety property
rather than an inconvenience: an arm request can be *built and validated* here,
but :class:`~operator_console.arm_ops.transport.ContractGatedTransport` refuses
to hand it to the ops client until the server-side registration exists.
:data:`PLANNED_ACTIONS` records exactly what that registration must say, and
``test_arm_ops_contract`` asserts the actions are still unregistered so the
"not wired yet" state cannot rot into an unnoticed "silently wired".

Naming
------
Every action is prefixed ``robot_arm_``.  The existing ops vocabulary already
uses ``arm`` for *chassis 시동* and ``robot_arm_enable`` for the arm component
toggle; an unprefixed ``arm_stop`` would read as a chassis command at exactly
the moment that mistake is most expensive.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Mapping


# Version of the *params* payloads described in this file.  Independent of the
# ops channel's own SCHEMA_VERSION, which versions the envelope.
ARM_PARAMS_VERSION = 1

# The ops channel caps a record at 4 KiB, and the envelope (token, request id,
# action name, timestamps) takes a few hundred bytes of that.  This cap is the
# params' share, checked at build time so a future free-text field cannot
# quietly produce a record the broker will drop.
#
# It must be large enough for the longest params the per-field checks below
# actually permit — 80-character strings JSON-escape to six bytes each — or the
# two limits would contradict each other and a legal command would be
# unbuildable.  ``test_every_action_fits_the_params_budget_at_maximum_length``
# pins that relationship.
MAX_PARAMS_BYTES = 1200


# --- action names -----------------------------------------------------------
ACTION_TOOL_CHANGE = "robot_arm_tool_change"
ACTION_MODE_REQUEST = "robot_arm_mode_request"
ACTION_MODE_RELEASE = "robot_arm_mode_release"
ACTION_JOG = "robot_arm_jog"
ACTION_JOG_STOP = "robot_arm_jog_stop"
ACTION_STOP = "robot_arm_stop"
ACTION_RESUME_HOLD = "robot_arm_resume_hold"
ACTION_HOME = "robot_arm_home"
ACTION_POSE_SAVE = "robot_arm_pose_save"
ACTION_POSE_MOVE = "robot_arm_pose_move"
ACTION_POSE_DELETE = "robot_arm_pose_delete"
ACTION_TOOL_COMMAND = "robot_arm_tool_command"
ACTION_TOOL_ENABLE = "robot_arm_tool_enable"
ACTION_TOOL_JOG = "robot_arm_tool_jog"
ACTION_TOOL_JOG_STOP = "robot_arm_tool_jog_stop"
ACTION_CALIBRATION = "robot_arm_calibration"
ACTION_CALIBRATION_JOG = "robot_arm_calibration_jog"
ACTION_CALIBRATION_JOG_STOP = "robot_arm_calibration_jog_stop"


# Actions that command motion and therefore require an approved MANUAL grant.
MOTION_ACTIONS = frozenset({
    ACTION_JOG,
    ACTION_JOG_STOP,
    ACTION_RESUME_HOLD,
    ACTION_HOME,
    ACTION_POSE_MOVE,
    ACTION_TOOL_COMMAND,
    ACTION_TOOL_ENABLE,
    ACTION_TOOL_JOG,
    ACTION_TOOL_JOG_STOP,
    ACTION_CALIBRATION_JOG,
    ACTION_CALIBRATION_JOG_STOP,
})

# Actions that end motion.  These are the one category that must stay reachable
# when everything else is blocked: a stop that needs permission is not a stop.
STOP_ACTIONS = frozenset({
    ACTION_STOP,
    ACTION_JOG_STOP,
    ACTION_TOOL_JOG_STOP,
    ACTION_CALIBRATION_JOG_STOP,
})

# Actions scoped to whichever tool is currently attached.  Each carries the
# tool identity it was issued against so a command cannot land on a tool that
# was swapped in between.
TOOL_SCOPED_ACTIONS = frozenset({
    ACTION_TOOL_COMMAND,
    ACTION_TOOL_ENABLE,
    ACTION_TOOL_JOG,
    ACTION_TOOL_JOG_STOP,
    ACTION_CALIBRATION_JOG,
    ACTION_CALIBRATION_JOG_STOP,
})

# Actions belonging to a calibration session, allowed to run while one is
# active.  Everything else is blocked for the duration.
CALIBRATION_ACTIONS = frozenset({
    ACTION_CALIBRATION,
    ACTION_CALIBRATION_JOG,
    ACTION_CALIBRATION_JOG_STOP,
})


# --- enumerated parameter values -------------------------------------------
MODE_MANUAL = "MANUAL"
MODE_FSM = "FSM"
MODES = frozenset({MODE_MANUAL, MODE_FSM})

TARGET_SINGLE = "single"
TARGET_LEFT = "left"
TARGET_RIGHT = "right"
TARGET_BOTH = "both"
TOOL_TARGETS = frozenset({
    TARGET_SINGLE, TARGET_LEFT, TARGET_RIGHT, TARGET_BOTH,
})

COMMAND_OPEN = "open"
COMMAND_CLOSE = "close"
COMMAND_STOP = "stop"
COMMAND_LEFT = "left"
COMMAND_RIGHT = "right"
TOOL_COMMANDS = frozenset({
    COMMAND_OPEN, COMMAND_CLOSE, COMMAND_STOP, COMMAND_LEFT, COMMAND_RIGHT,
})

SCOPE_ARM = "arm"
SCOPE_TOOL = "tool"
CALIBRATION_SCOPES = frozenset({SCOPE_ARM, SCOPE_TOOL})

CALIBRATION_ACTION_TOKENS = frozenset({
    "measure", "verify", "apply_temporary", "save",
    "start", "cancel", "capture_open", "capture_close",
})

# Why a jog ended.  The reason travels with the stop so the backend and the
# operation log can tell a deliberate release from a safety-driven one.
STOP_REASON_EXPLICIT = "explicit"          # 키/버튼을 놓음
STOP_REASON_EXPIRED = "expired"            # TTL 만료 — 갱신이 끊김
STOP_REASON_AUTHORITY_LOST = "authority_lost"
STOP_REASON_LINK_LOST = "link_lost"
STOP_REASON_TOOL_CHANGED = "tool_changed"
STOP_REASON_FOCUS_LOST = "focus_lost"      # 탭 이탈·포커스 상실·창 닫힘
STOP_REASON_CALIBRATION = "calibration_started"
STOP_REASONS = frozenset({
    STOP_REASON_EXPLICIT, STOP_REASON_EXPIRED, STOP_REASON_AUTHORITY_LOST,
    STOP_REASON_LINK_LOST, STOP_REASON_TOOL_CHANGED, STOP_REASON_FOCUS_LOST,
    STOP_REASON_CALIBRATION,
})

STOP_REASON_KOREAN = {
    STOP_REASON_EXPLICIT: "명시적 정지",
    STOP_REASON_EXPIRED: "조그 유효시간 만료",
    STOP_REASON_AUTHORITY_LOST: "제어권 상실",
    STOP_REASON_LINK_LOST: "통신 단절",
    STOP_REASON_TOOL_CHANGED: "도구 교체",
    STOP_REASON_FOCUS_LOST: "화면 이탈",
    STOP_REASON_CALIBRATION: "캘리브레이션 시작",
}

DIRECTIONS = frozenset({-1, 1})


# --- parameter specification ------------------------------------------------
@dataclass(frozen=True)
class ParamSpec:
    name: str
    required: bool
    check: Callable[[Any], bool]
    note: str = ""


def _is_str(value: Any) -> bool:
    return isinstance(value, str) and bool(value) and len(value) <= 80


def _is_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _in(allowed) -> Callable[[Any], bool]:
    return lambda value: value in allowed


def _is_axis(value: Any) -> bool:
    return _is_int(value) and 1 <= value <= 6


def _is_speed_level(value: Any) -> bool:
    return _is_int(value) and 0 <= value <= 10


def _is_generation(value: Any) -> bool:
    return _is_int(value) and value >= 0


def _is_deadline(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) \
        and value > 0.0


def _spec(name: str, check, *, required: bool = True, note: str = "") -> ParamSpec:
    return ParamSpec(name=name, required=required, check=check, note=note)


# Every tool-scoped action carries these two so a late command cannot land on a
# tool that has since been swapped.  The backend must reject a mismatch too:
# this is a console-side guard, not a substitute for the arm's own check.
_TOOL_IDENTITY = (
    _spec("tool_id", _is_str, note="명령 발행 시점의 활성 도구 ID"),
    _spec("tool_generation", _is_generation,
          note="활성 도구 세대. 백엔드 현재 세대와 다르면 거부되어야 한다"),
)

# A jog is a deadman: it is valid only until ``expires_at_s`` and must be
# re-sent to stay alive.  ``sequence`` lets the backend drop an out-of-order
# duplicate without inspecting the ops envelope.
_JOG_LIFETIME = (
    _spec("expires_at_s", _is_deadline,
          note="이 시각(초, 단조시각)을 넘기면 백엔드가 스스로 정지해야 한다"),
    _spec("sequence", _is_generation, note="같은 조그 세션 내 단조 증가 번호"),
)


ACTION_PARAMS: dict[str, tuple[ParamSpec, ...]] = {
    ACTION_TOOL_CHANGE: (
        _spec("requested_kind", _is_str, note="변경 요청 대상 도구 종류"),
        _spec("observed_tool_generation", _is_generation,
              note="요청 시점에 콘솔이 보고 있던 활성 도구 세대"),
    ),
    ACTION_MODE_REQUEST: (
        _spec("mode", _in(MODES)),
    ),
    ACTION_MODE_RELEASE: (),
    ACTION_JOG: (
        _spec("axis", _is_axis),
        _spec("direction", _in(DIRECTIONS)),
        _spec("speed_level", _is_speed_level, required=False),
        *_JOG_LIFETIME,
    ),
    ACTION_JOG_STOP: (
        _spec("axis", _is_axis),
        _spec("reason", _in(STOP_REASONS)),
        _spec("sequence", _is_generation),
    ),
    ACTION_STOP: (),
    ACTION_RESUME_HOLD: (),
    ACTION_HOME: (),
    ACTION_POSE_SAVE: (_spec("name", _is_str),),
    ACTION_POSE_MOVE: (_spec("name", _is_str),),
    ACTION_POSE_DELETE: (_spec("name", _is_str),),
    ACTION_TOOL_COMMAND: (
        _spec("target", _in(TOOL_TARGETS)),
        _spec("command", _in(TOOL_COMMANDS)),
        *_TOOL_IDENTITY,
    ),
    ACTION_TOOL_ENABLE: (
        _spec("enabled", lambda value: isinstance(value, bool)),
        *_TOOL_IDENTITY,
    ),
    ACTION_TOOL_JOG: (
        _spec("target", _in(TOOL_TARGETS)),
        _spec("direction", _in(DIRECTIONS)),
        *_TOOL_IDENTITY,
        *_JOG_LIFETIME,
    ),
    ACTION_TOOL_JOG_STOP: (
        _spec("target", _in(TOOL_TARGETS)),
        _spec("reason", _in(STOP_REASONS)),
        _spec("sequence", _is_generation),
        *_TOOL_IDENTITY,
    ),
    ACTION_CALIBRATION: (
        _spec("scope", _in(CALIBRATION_SCOPES)),
        _spec("step", _is_str, note="arm: gear_ratio|zero|range, tool: 대상 끝점"),
        _spec("action", _in(CALIBRATION_ACTION_TOKENS)),
        _spec("tool_id", _is_str, required=False),
        _spec("tool_generation", _is_generation, required=False),
    ),
    ACTION_CALIBRATION_JOG: (
        _spec("target", _in(TOOL_TARGETS)),
        _spec("direction", _in(DIRECTIONS)),
        *_TOOL_IDENTITY,
        *_JOG_LIFETIME,
    ),
    ACTION_CALIBRATION_JOG_STOP: (
        _spec("target", _in(TOOL_TARGETS)),
        _spec("reason", _in(STOP_REASONS)),
        _spec("sequence", _is_generation),
        *_TOOL_IDENTITY,
    ),
}

ARM_ACTIONS = frozenset(ACTION_PARAMS)


class ArmParamsError(ValueError):
    """A request whose parameters do not satisfy this contract."""


def validate_params(action: str, params: Mapping[str, Any]) -> dict[str, Any]:
    """Return a normalised copy of ``params``, or raise :class:`ArmParamsError`.

    Unknown keys are an error rather than something to ignore: a typo'd field
    that is silently dropped becomes a command that means something other than
    what the caller wrote.
    """
    if action not in ACTION_PARAMS:
        raise ArmParamsError(f"unknown arm action: {action!r}")
    specs = ACTION_PARAMS[action]
    known = {spec.name: spec for spec in specs}
    unknown = sorted(set(params) - set(known))
    if unknown:
        raise ArmParamsError(f"{action}: unknown params {unknown}")
    result: dict[str, Any] = {}
    for spec in specs:
        if spec.name not in params:
            if spec.required:
                raise ArmParamsError(f"{action}: missing param {spec.name!r}")
            continue
        value = params[spec.name]
        if not spec.check(value):
            raise ArmParamsError(
                f"{action}: invalid param {spec.name!r}={value!r}"
            )
        result[spec.name] = value
    _assert_size(action, result)
    return result


def _assert_size(action: str, params: Mapping[str, Any]) -> None:
    import json

    try:
        encoded = json.dumps(params, allow_nan=False, separators=(",", ":"))
    except (TypeError, ValueError) as exc:
        raise ArmParamsError(f"{action}: params are not JSON encodable") from exc
    if len(encoded.encode("utf-8")) > MAX_PARAMS_BYTES:
        raise ArmParamsError(
            f"{action}: params exceed {MAX_PARAMS_BYTES} bytes"
        )


@dataclass(frozen=True)
class ArmOpsRequest:
    """One validated, ready-to-submit arm command.

    Constructing this object is itself the authorization record: nothing builds
    one except :mod:`operator_console.arm_ops.session`, and only after
    :mod:`operator_console.arm_ops.policy` has allowed it.  It carries no
    socket, no transport and no ROS knowledge — just the action name, the
    validated params, and the ops-level revision guard.
    """

    action: str
    params: dict[str, Any] = field(default_factory=dict)
    expected_state_revision: int | None = None
    issued_s: float = 0.0
    # Free-form label for the operation log; never sent on the wire.
    intent: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "params", validate_params(self.action, self.params))

    @property
    def is_stop(self) -> bool:
        return self.action in STOP_ACTIONS


@dataclass(frozen=True)
class ArmOpsAck:
    """The console-side view of an ops response for an arm request.

    ``status`` reuses the ops channel's own vocabulary
    (PENDING / FINAL_SUCCESS / FINAL_REJECTED / OUTCOME_UNKNOWN) unchanged, so
    ``operator_console.labels.ack_korean`` renders an arm ack exactly like a
    chassis one.  In particular OUTCOME_UNKNOWN stays OUTCOME_UNKNOWN — it is
    never collapsed into success or failure.
    """

    request_id: str
    status: str
    detail: str = ""
    action: str = ""


# --- what still has to be decided elsewhere ---------------------------------
@dataclass(frozen=True)
class PlannedAction:
    """One ops action that must be registered before anything can be sent.

    ``target`` is deliberately empty.  Filling it in means naming the ROS
    service or topic that executes the command, which is the arm side's call
    and is not guessed here.
    """

    action: str
    roles: tuple[str, ...]
    kind: str = "UNRESOLVED"
    target: tuple[str, ...] = ()
    note: str = ""

    @property
    def resolved(self) -> bool:
        return bool(self.target) and self.kind != "UNRESOLVED"


_CONSOLE_ONLY = ("console",)

PLANNED_ACTIONS: tuple[PlannedAction, ...] = tuple(
    PlannedAction(action=action, roles=_CONSOLE_ONLY, note=note)
    for action, note in (
        (ACTION_TOOL_CHANGE, "명시적 도구 변경. 완료는 백엔드의 새 활성 도구 상태로 확인한다"),
        (ACTION_MODE_REQUEST, "MANUAL/FSM 제어권 요청. 승인은 별도 상태로 회신되어야 한다"),
        (ACTION_MODE_RELEASE, "제어권 반납"),
        (ACTION_JOG, "누름 조그의 시작과 최신값 갱신. deadman — expires_at_s 경과 시 정지"),
        (ACTION_JOG_STOP, "명시적 조그 종료. 사유를 포함한다"),
        (ACTION_STOP, "정지. 기존 stop 의미(토크 해제)를 보존해야 한다"),
        (ACTION_RESUME_HOLD, "복귀. 현재 자세 유지 의미를 보존해야 한다"),
        (ACTION_HOME, "home. 저장된 자세 의미를 보존해야 한다"),
        (ACTION_POSE_SAVE, "현재 자세 저장"),
        (ACTION_POSE_MOVE, "저장 자세로 이동"),
        (ACTION_POSE_DELETE, "저장 자세 삭제"),
        (ACTION_TOOL_COMMAND, "그리퍼 open/close/stop. 백엔드의 정상 FSM 경로를 사용해야 한다"),
        (ACTION_TOOL_ENABLE, "현재 감지 도구의 프로필에 고정된 actuator 토크 활성화/해제"),
        (ACTION_TOOL_JOG, "그리퍼 누름 조그. deadman"),
        (ACTION_TOOL_JOG_STOP, "그리퍼 조그 종료"),
        (ACTION_CALIBRATION, "팔/도구 보정 단계 명령. 측정·검증·임시적용·저장이 구분되어야 한다"),
        (ACTION_CALIBRATION_JOG, "보정용 조그. deadman"),
        (ACTION_CALIBRATION_JOG_STOP, "보정용 조그 종료"),
    )
)

assert {planned.action for planned in PLANNED_ACTIONS} == ARM_ACTIONS


@dataclass(frozen=True)
class BackendRequirement:
    """An obligation on the arm side that this console does not get to decide."""

    key: str
    requirement: str
    resolved: bool = False


BACKEND_REQUIREMENTS: tuple[BackendRequirement, ...] = (
    BackendRequirement(
        "action_registration",
        "ops_contract.ACTIONS 에 robot_arm_* 액션과 각각의 ROS 대상 등록. "
        "등록 전까지 콘솔은 어떤 팔 명령도 전송할 수 없다.",
    ),
    BackendRequirement(
        "grant_report",
        "MANUAL/FSM 승인 상태를 ops 상태 push 또는 팔 텔레메트리로 회신. "
        "요청 성공 ACK는 승인이 아니다.",
    ),
    BackendRequirement(
        "jog_deadman",
        "expires_at_s 경과 시 백엔드가 스스로 조그를 종료. "
        "콘솔의 명시적 stop 도착 여부와 무관해야 한다.",
    ),
    BackendRequirement(
        "jog_rate_and_serialization",
        "ops 브로커는 변경 명령을 직렬화한다(busy: mutation in flight). "
        "조그 갱신 주기·rate limit·최신값 병합 정책을 실측으로 확정해야 한다.",
    ),
    BackendRequirement(
        "tool_generation_authority",
        "활성 도구 세대의 정본과 증가 규칙. 불일치 명령은 백엔드도 거부해야 한다.",
    ),
    BackendRequirement(
        "single_ownership",
        "키보드 텔레옵 경로와 MoveIt/팔 FSM 경로의 모터 단일 소유권 확정. "
        "두 경로 병렬 실행은 금지된다.",
    ),
    BackendRequirement(
        "stop_semantics",
        "stop(토크 해제) · resume(현재 자세 유지) · home(저장 자세)의 기존 의미 보존 확인.",
    ),
    BackendRequirement(
        "calibration_persistence",
        "팔 기어비/영점 임시 적용, 가동범위 저장, 도구 YAML 저장의 구분. "
        "하나의 '저장'으로 합치지 않는다.",
    ),
    BackendRequirement(
        "token_role",
        "팔 명령에 필요한 역할(console 전용 여부)과 토큰 발급 정책.",
    ),
)


def unresolved_requirements() -> tuple[BackendRequirement, ...]:
    return tuple(item for item in BACKEND_REQUIREMENTS if not item.resolved)


def unregistered_actions(registered: Mapping[str, Any] | set[str]) -> tuple[str, ...]:
    """Arm actions the ops contract does not yet accept.

    Pass ``ops_contract.ACTIONS``.  Anything returned here cannot be sent, and
    the transport gate refuses it rather than letting it reach the socket.
    """
    known = set(registered)
    return tuple(sorted(ARM_ACTIONS - known))
