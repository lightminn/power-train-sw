"""Injected state and callback contracts for the robot-arm console tabs.

This module is the boundary between the arm UI skeleton and everything that
owns real data.  The tabs in this package **never** open a socket, import
``rclpy``, or touch a motor: they render an :class:`ArmUiState` that somebody
else hands them and they report operator intent through :class:`ArmUiCallbacks`.

Two rules shape every default in this file.

1. **A missing source is never drawn as a good source.**  Every default here is
   "not received" / "not granted" / "not saved".  There is no default tool, no
   default approval, and no default calibration result.
2. **This file does not fix the external schema.**  The field names below are
   the console-side view model.  Which ROS topic, UDP field, or unit feeds each
   one is decided by the telemetry-adapter work, not here.  Adapters translate
   into these dataclasses; the widgets only read them.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Callable


# --- source freshness -------------------------------------------------------
# The four states match ``operator_console.labels.freshness_korean`` so the arm
# tabs read the same way as every other console panel.
LINK_WAITING = "WAITING"
LINK_LIVE = "LIVE"
LINK_STALE = "STALE"
LINK_UNAVAILABLE = "UNAVAILABLE"


# --- tool kinds -------------------------------------------------------------
# Kinds, not product names: the panel layout depends on the mechanism (one
# actuator vs. two vs. none), and the operator-facing name arrives with the
# state.  ``UNKNOWN`` is the production default -- no tool is assumed attached.
TOOL_UNKNOWN = "unknown"
TOOL_SINGLE_GRIPPER = "single_gripper"
TOOL_DUAL_GRIPPER = "dual_gripper"
TOOL_CLEANER = "cleaner"
TOOL_ENVIRONMENT_SENSOR = "environment_sensor"

TOOL_KIND_KOREAN = {
    TOOL_UNKNOWN: "감지 대기",
    TOOL_SINGLE_GRIPPER: "단일 그리퍼",
    TOOL_DUAL_GRIPPER: "듀얼 그리퍼",
    TOOL_CLEANER: "청소 모듈",
    TOOL_ENVIRONMENT_SENSOR: "환경 센서 모듈",
}

# Kinds an operator may nominate as a change candidate.  ``UNKNOWN`` is an
# observation, never a request.
SELECTABLE_TOOL_KINDS = (
    TOOL_SINGLE_GRIPPER,
    TOOL_DUAL_GRIPPER,
    TOOL_CLEANER,
    TOOL_ENVIRONMENT_SENSOR,
)

# Kinds that carry an existing end-effector FSM control panel.  The cleaner is
# limited to its existing LEFT/RIGHT/STOP FSM path; raw motor drive remains
# deliberately absent from this UI.
GRIPPER_TOOL_KINDS = frozenset({TOOL_SINGLE_GRIPPER, TOOL_DUAL_GRIPPER})


# --- capabilities -----------------------------------------------------------
# A capability token means "the backend that feeds this UI says this action
# exists".  Absent token => the control is drawn disabled with a reason.  The
# skeleton never infers a capability from the presence of a callback.
CAP_TOOL_CHANGE = "tool_change"
CAP_CONTROL_MODE = "control_mode"
CAP_TELEOP_JOG = "teleop_jog"
CAP_TELEOP_POSE = "teleop_pose"
CAP_GRIPPER_COMMAND = "gripper_command"
CAP_TOOL_ENABLE = "tool_enable"
CAP_DUAL_SYNC = "dual_sync"
CAP_ARM_CALIBRATION = "arm_calibration"
CAP_TOOL_CALIBRATION = "tool_calibration"
CAP_DUAL_TOOL_CALIBRATION = "dual_tool_calibration"

# Actions that additionally require an approved MANUAL grant.  Requesting the
# grant itself must stay available, or the operator can never get one.
MANUAL_GRANT_REQUIRED = frozenset({
    CAP_TELEOP_JOG,
    CAP_TELEOP_POSE,
    CAP_GRIPPER_COMMAND,
    CAP_DUAL_SYNC,
    CAP_TOOL_ENABLE,
})

# Actions that belong to a calibration session and therefore stay live while
# one runs -- everything else is blocked for the duration.
CALIBRATION_CAPS = frozenset({
    CAP_ARM_CALIBRATION,
    CAP_TOOL_CALIBRATION,
    CAP_DUAL_TOOL_CALIBRATION,
})


# --- request / session status vocabularies ---------------------------------
REQUEST_IDLE = "IDLE"
REQUEST_PENDING = "PENDING"
REQUEST_IN_PROGRESS = "IN_PROGRESS"
REQUEST_COMPLETED = "COMPLETED"
REQUEST_REJECTED = "REJECTED"
REQUEST_TIMEOUT = "TIMEOUT"
REQUEST_DISCONNECTED = "DISCONNECTED"

REQUEST_KOREAN = {
    REQUEST_IDLE: "요청 없음",
    REQUEST_PENDING: "요청 대기",
    REQUEST_IN_PROGRESS: "진행 중",
    REQUEST_COMPLETED: "완료",
    REQUEST_REJECTED: "거부됨",
    REQUEST_TIMEOUT: "시간 초과",
    REQUEST_DISCONNECTED: "연결 끊김",
}

# Request states that still hold the tool-change path open.
REQUEST_ACTIVE = frozenset({REQUEST_PENDING, REQUEST_IN_PROGRESS})

AUTHORITY_UNKNOWN = "UNKNOWN"
AUTHORITY_NONE = "NONE"
AUTHORITY_REQUESTED = "REQUESTED"
AUTHORITY_GRANTED = "GRANTED"
AUTHORITY_REJECTED = "REJECTED"
AUTHORITY_TIMEOUT = "TIMEOUT"

AUTHORITY_KOREAN = {
    AUTHORITY_UNKNOWN: "확인 불가",
    AUTHORITY_NONE: "미보유",
    AUTHORITY_REQUESTED: "요청 후 승인 대기",
    AUTHORITY_GRANTED: "승인됨",
    AUTHORITY_REJECTED: "거부됨",
    AUTHORITY_TIMEOUT: "응답 없음",
}

MODE_MANUAL = "MANUAL"
MODE_FSM = "FSM"

SYNC_UNKNOWN = "UNKNOWN"
SYNC_SYNCED = "SYNCED"
SYNC_DRIFT = "DRIFT"
SYNC_FAULT = "FAULT"

SYNC_KOREAN = {
    SYNC_UNKNOWN: "판정 없음",
    SYNC_SYNCED: "동기화됨",
    SYNC_DRIFT: "편차 있음",
    SYNC_FAULT: "동기화 오류",
}

STEP_IDLE = "IDLE"
STEP_RUNNING = "RUNNING"
STEP_DONE = "DONE"
STEP_FAILED = "FAILED"
STEP_ABORTED = "ABORTED"
STEP_UNSUPPORTED = "UNSUPPORTED"

STEP_KOREAN = {
    STEP_IDLE: "미실시",
    STEP_RUNNING: "진행 중",
    STEP_DONE: "완료",
    STEP_FAILED: "실패",
    STEP_ABORTED: "중단됨",
    STEP_UNSUPPORTED: "지원하지 않음",
}


@dataclass(frozen=True)
class SourceLink:
    """Freshness of the arm state feed itself.

    ``UNAVAILABLE`` is the production default: until an adapter proves
    otherwise the console has no arm data, and a stale last-known value is
    never redrawn as a current one.
    """

    state: str = LINK_UNAVAILABLE
    age_s: float | None = None
    detail: str = ""


@dataclass(frozen=True)
class ToolIdentity:
    """What the backend reports as physically present.

    ``generation`` increments whenever the backend confirms a different active
    tool.  Panels compare it against the diagnostics payload so a late reply
    carrying an older generation is recognised and dropped instead of being
    painted onto the new tool's panel.
    """

    kind: str = TOOL_UNKNOWN
    tool_id: str = ""
    display_name: str = ""
    interface: str = ""
    attached: bool | None = None
    actuator_ids: tuple[int, ...] = ()
    generation: int = 0

    def label(self) -> str:
        base = self.display_name or TOOL_KIND_KOREAN.get(self.kind, self.kind)
        return f"{base} ({self.tool_id})" if self.tool_id else base


@dataclass(frozen=True)
class ToolChangeRequest:
    """The explicit change request, kept apart from the detected tool."""

    status: str = REQUEST_IDLE
    requested_kind: str = ""
    detail: str = ""


@dataclass(frozen=True)
class ControlAuthority:
    """MANUAL/FSM authority: what was asked for vs. what the arm granted.

    ``requested_mode`` is only a record of the ask.  Nothing in this UI may key
    an enable off it -- only ``granted_mode`` unlocks motion controls.
    """

    requested_mode: str = ""
    granted_mode: str = ""
    status: str = AUTHORITY_UNKNOWN
    detail: str = ""


@dataclass(frozen=True)
class FsmState:
    """One state machine's reading: the backend's own token plus a label."""

    raw: str = ""
    korean: str = ""

    def text(self) -> str:
        if not self.raw and not self.korean:
            return "정보 없음"
        if self.korean and self.raw:
            return f"{self.korean} ({self.raw})"
        return self.korean or self.raw


@dataclass(frozen=True)
class BlockReason:
    """A concrete, backend-sourced reason motion is unavailable."""

    code: str
    korean: str


@dataclass(frozen=True)
class AxisState:
    index: int
    name: str = ""
    angle_deg: float | None = None
    moving: bool = False


@dataclass(frozen=True)
class TeleopState:
    """Keyboard-teleop presentation state.

    The skeleton renders these values and reports intent; it does not decide
    what a jog command looks like on the wire.
    """

    selected_axis: int | None = None
    axes: tuple[AxisState, ...] = ()
    speed_level: int | None = None
    speed_ratio: float | None = None
    jogging: bool = False
    poses: tuple[str, ...] = ()
    active_pose: str = ""


@dataclass(frozen=True)
class ToolMotorReading:
    """One actuator of the **currently active** tool.

    Readings for any other tool must not be handed to the UI: the diagnostics
    panel renders exactly the rows it is given, so filtering belongs upstream
    and is asserted by :func:`readings_match_active_tool`.
    """

    actuator_id: int
    role: str = ""
    position_deg: float | None = None
    opening_ratio: float | None = None
    load_percent: float | None = None
    temperature_c: float | None = None
    torque_on: bool | None = None
    online: bool | None = None
    operating_mode: str = ""
    error: str = ""


@dataclass(frozen=True)
class ToolDiagnostics:
    motors: tuple[ToolMotorReading, ...] = ()
    sync_state: str = SYNC_UNKNOWN
    sync_error_deg: float | None = None
    generation: int = 0


@dataclass(frozen=True)
class CalibrationStep:
    """One measure -> verify -> temporary-apply -> save step.

    The four outcomes are deliberately separate fields.  A measured value is
    not a verified one, a temporary parameter is not a saved one, and
    ``supported=False`` renders as "지원하지 않음" rather than an idle step the
    operator might sit and wait on.
    """

    key: str
    title: str
    supported: bool = True
    status: str = STEP_IDLE
    measured: str = ""
    verified: str = ""
    applied_temporarily: bool = False
    saved: bool = False
    detail: str = ""


@dataclass(frozen=True)
class CalibrationState:
    session_active: bool = False
    session_target: str = ""
    arm_steps: tuple[CalibrationStep, ...] = ()
    tool_steps: tuple[CalibrationStep, ...] = ()
    unsaved_results: bool = False
    detail: str = ""


@dataclass(frozen=True)
class ArmUiState:
    """Everything the two arm tabs draw.  Replace it wholesale on each update."""

    link: SourceLink = field(default_factory=SourceLink)
    detected_tool: ToolIdentity = field(default_factory=ToolIdentity)
    tool_change: ToolChangeRequest = field(default_factory=ToolChangeRequest)
    authority: ControlAuthority = field(default_factory=ControlAuthority)
    arm_fsm: FsmState = field(default_factory=FsmState)
    arm_status: FsmState = field(default_factory=FsmState)
    tool_fsm: FsmState = field(default_factory=FsmState)
    block_reasons: tuple[BlockReason, ...] = ()
    teleop: TeleopState = field(default_factory=TeleopState)
    diagnostics: ToolDiagnostics = field(default_factory=ToolDiagnostics)
    calibration: CalibrationState = field(default_factory=CalibrationState)
    capabilities: frozenset[str] = frozenset()
    # True only for the preview fixtures.  Panels render a visible banner so a
    # screenshot of fake data can never be mistaken for a live arm.
    is_fixture: bool = False
    # Explicit local bench mode.  It bypasses presentation capability/grant
    # gates but never invents an active tool or disables the stop path.
    developer_mode: bool = False


def default_state() -> ArmUiState:
    """The production default: nothing received, nothing granted, nothing saved."""
    return ArmUiState()


Callback = Callable[..., object] | None


@dataclass(frozen=True)
class ArmUiCallbacks:
    """Operator intent leaves the UI through these and nowhere else.

    Every field defaults to ``None``, and a ``None`` callback disables its
    control.  The signatures name the intent; they are not a wire format.  The
    integration layer decides how -- and whether -- each becomes an ops command.
    """

    # 도구
    request_tool_change: Callback = None          # (kind: str)
    # 제어권
    request_control_mode: Callback = None         # (mode: str)  MANUAL | FSM
    release_control_mode: Callback = None         # ()
    # 키보드 텔레옵
    select_axis: Callback = None                  # (axis_index: int)
    jog_start: Callback = None                    # (axis_index: int, direction: int)
    jog_stop: Callback = None                     # (axis_index: int)
    change_speed: Callback = None                 # (delta: int)  -1 = '[', +1 = ']'
    stop_motion: Callback = None                  # ()  space -- 기존 stop 의미 보존
    resume_hold: Callback = None                  # ()  t
    go_home: Callback = None                      # ()  h
    save_pose: Callback = None                    # (name: str)
    move_to_pose: Callback = None                 # (name: str)
    delete_pose: Callback = None                  # (name: str)
    # 도구 조작
    set_tool_enabled: Callback = None             # (enabled: bool) active tool only
    tool_command: Callback = None                 # (target: str, command: str)
    tool_jog_start: Callback = None               # (target: str, direction: int)
    tool_jog_stop: Callback = None                # (target: str)
    # 보정
    arm_calibration_command: Callback = None      # (step_key: str, action: str)
    tool_calibration_command: Callback = None     # (target: str, action: str)
    tool_calibration_jog_start: Callback = None   # (target: str, direction: int)
    tool_calibration_jog_stop: Callback = None    # (target: str)


# Calibration / tool-command action tokens used by the panels.
ACTION_MEASURE = "measure"
ACTION_VERIFY = "verify"
ACTION_APPLY_TEMPORARY = "apply_temporary"
ACTION_SAVE = "save"
ACTION_CANCEL = "cancel"
ACTION_START = "start"
ACTION_CAPTURE_OPEN = "capture_open"
ACTION_CAPTURE_CLOSE = "capture_close"

COMMAND_OPEN = "open"
COMMAND_CLOSE = "close"
COMMAND_STOP = "stop"
COMMAND_LEFT = "left"
COMMAND_RIGHT = "right"

TARGET_SINGLE = "single"
TARGET_LEFT = "left"
TARGET_RIGHT = "right"
TARGET_BOTH = "both"

TARGET_KOREAN = {
    TARGET_SINGLE: "그리퍼",
    TARGET_LEFT: "좌측",
    TARGET_RIGHT: "우측",
    TARGET_BOTH: "동기",
}


def is_live(state: ArmUiState) -> bool:
    """Only a genuinely LIVE feed counts.  STALE is not "still fine"."""
    return state.link.state == LINK_LIVE


def has_manual_grant(state: ArmUiState) -> bool:
    return (
        state.authority.status == AUTHORITY_GRANTED
        and state.authority.granted_mode == MODE_MANUAL
    )


def gate(
    state: ArmUiState,
    callback: Callback,
    capability: str,
) -> tuple[bool, str]:
    """Decide whether one control is operable, and say why when it is not.

    The order matters: the *most fundamental* obstacle wins, so the operator is
    told "no command path" rather than "no control authority" when both are
    true.  Returning the reason -- not just a bool -- is what lets every
    disabled control carry a tooltip that explains itself.
    """
    if callback is None:
        return False, "미연결 — 명령 경로가 연결되지 않음"
    if capability and capability not in state.capabilities and not state.developer_mode:
        return False, "지원하지 않음 — 백엔드가 이 기능을 보고하지 않음"
    if not is_live(state):
        if state.link.state == LINK_STALE:
            return False, "상태 지연(STALE) — 최신 상태 없이 조작 불가"
        return False, "상태 미수신 — 조작 불가"
    if state.calibration.session_active and capability not in CALIBRATION_CAPS:
        return False, "캘리브레이션 진행 중 — 일반 조작 차단"
    if state.tool_change.status in REQUEST_ACTIVE and capability != CAP_TOOL_CHANGE:
        return False, "도구 교체 중 — 조작 차단"
    if (capability in MANUAL_GRANT_REQUIRED
            and not has_manual_grant(state)
            and not state.developer_mode):
        return False, "수동 제어권 미승인"
    if state.block_reasons:
        return False, state.block_reasons[0].korean
    return True, ""


def readings_match_active_tool(state: ArmUiState) -> bool:
    """True when the diagnostics rows belong to the currently detected tool.

    Two independent leaks are caught here: a row whose actuator id is not in
    the active profile, and a payload stamped with a superseded generation.
    """
    tool = state.detected_tool
    diagnostics = state.diagnostics
    if not diagnostics.motors:
        return True
    if diagnostics.generation != tool.generation:
        return False
    if not tool.actuator_ids:
        return False
    allowed = set(tool.actuator_ids)
    return all(motor.actuator_id in allowed for motor in diagnostics.motors)


def cleared_for_tool(state: ArmUiState) -> ArmUiState:
    """Drop diagnostics that do not belong to the active tool.

    Panels call this immediately before rendering, so a late or mismatched
    payload blanks the panel instead of leaving the previous tool's numbers on
    screen.
    """
    if readings_match_active_tool(state):
        return state
    return replace(
        state,
        diagnostics=ToolDiagnostics(generation=state.detected_tool.generation),
    )
