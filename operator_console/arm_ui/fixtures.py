"""Preview-only fixture states.  Never import these from production paths.

Every state built here carries ``is_fixture=True``, which makes both tabs draw
a visible banner.  That flag is the whole point: a screenshot of this data must
not be mistakable for a live arm, and the production default
(:func:`contracts.default_state`) stays entirely separate — it is the
disconnected state, and nothing in this module can reach it.

The numbers are plausible-looking placeholders for layout review.  They are not
measurements, not a schema, and not a claim about what the arm publishes.
"""
from __future__ import annotations

from dataclasses import replace

from . import contracts as C


ALL_CAPABILITIES = frozenset({
    C.CAP_TOOL_CHANGE,
    C.CAP_CONTROL_MODE,
    C.CAP_TELEOP_JOG,
    C.CAP_TELEOP_POSE,
    C.CAP_GRIPPER_COMMAND,
    C.CAP_DUAL_SYNC,
    C.CAP_ARM_CALIBRATION,
    C.CAP_TOOL_CALIBRATION,
    C.CAP_DUAL_TOOL_CALIBRATION,
})


def _axes() -> tuple[C.AxisState, ...]:
    return tuple(
        C.AxisState(
            index=index,
            name=name,
            angle_deg=angle,
        )
        for index, (name, angle) in enumerate(
            (
                ("베이스", 12.4), ("숄더", -38.1), ("엘보", 74.9),
                ("손목 피치", -21.0), ("손목 롤", 5.2),
            ),
            start=1,
        )
    )


def _teleop(selected: int | None, poses: tuple[str, ...]) -> C.TeleopState:
    return C.TeleopState(
        selected_axis=selected,
        axes=_axes(),
        speed_level=3,
        speed_ratio=0.45,
        jogging=False,
        poses=poses,
    )


def disconnected() -> C.ArmUiState:
    """Nothing received.  Identical in shape to the production default."""
    return replace(C.default_state(), is_fixture=True)


def waiting() -> C.ArmUiState:
    """A source is configured but has not delivered a first snapshot."""
    return replace(
        C.default_state(),
        link=C.SourceLink(state=C.LINK_WAITING, detail="첫 스냅샷 대기"),
        is_fixture=True,
    )


def stale_single() -> C.ArmUiState:
    """A gripper was seen, then the feed went quiet — must not read as normal."""
    live = single_gripper()
    return replace(
        live,
        link=C.SourceLink(state=C.LINK_STALE, age_s=4.7, detail="수신 지연"),
        is_fixture=True,
    )


def single_gripper() -> C.ArmUiState:
    return C.ArmUiState(
        link=C.SourceLink(state=C.LINK_LIVE, age_s=0.08),
        detected_tool=C.ToolIdentity(
            kind=C.TOOL_SINGLE_GRIPPER,
            tool_id="TOOL-S1",
            display_name="단일 그리퍼",
            interface="Dynamixel",
            attached=True,
            actuator_ids=(5,),
            generation=7,
        ),
        tool_change=C.ToolChangeRequest(status=C.REQUEST_IDLE),
        authority=C.ControlAuthority(
            requested_mode=C.MODE_MANUAL,
            granted_mode=C.MODE_MANUAL,
            status=C.AUTHORITY_GRANTED,
        ),
        arm_fsm=C.FsmState(raw="IDLE", korean="대기"),
        arm_status=C.FsmState(raw="READY", korean="준비"),
        tool_fsm=C.FsmState(raw="READY", korean="준비"),
        teleop=_teleop(2, ("작업 준비", "이송", "홈")),
        diagnostics=C.ToolDiagnostics(
            motors=(
                C.ToolMotorReading(
                    actuator_id=5,
                    role=C.TARGET_SINGLE,
                    position_deg=41.6,
                    opening_ratio=0.62,
                    load_percent=18.0,
                    temperature_c=37.0,
                    torque_on=True,
                    online=True,
                    operating_mode="position",
                ),
            ),
            generation=7,
        ),
        calibration=C.CalibrationState(
            arm_steps=(
                C.CalibrationStep(
                    key="gear_ratio", title="기어비", status=C.STEP_DONE,
                    measured="축1 1:36.0 / 축2 1:36.0", verified="오차 0.3%",
                    applied_temporarily=True,
                ),
                C.CalibrationStep(key="zero", title="영점", status=C.STEP_IDLE),
                C.CalibrationStep(key="range", title="가동범위", status=C.STEP_IDLE),
            ),
            tool_steps=(
                C.CalibrationStep(
                    key=C.TARGET_SINGLE, title="단일 끝점", status=C.STEP_IDLE,
                ),
            ),
        ),
        capabilities=ALL_CAPABILITIES,
        is_fixture=True,
    )


def dual_gripper() -> C.ArmUiState:
    return C.ArmUiState(
        link=C.SourceLink(state=C.LINK_LIVE, age_s=0.05),
        detected_tool=C.ToolIdentity(
            kind=C.TOOL_DUAL_GRIPPER,
            tool_id="TOOL-D1",
            display_name="듀얼 그리퍼",
            interface="Dynamixel",
            attached=True,
            actuator_ids=(3, 4),
            generation=8,
        ),
        tool_change=C.ToolChangeRequest(
            status=C.REQUEST_COMPLETED,
            requested_kind=C.TOOL_DUAL_GRIPPER,
            detail="백엔드가 새 활성 도구를 확인함",
        ),
        authority=C.ControlAuthority(
            requested_mode=C.MODE_MANUAL,
            granted_mode=C.MODE_MANUAL,
            status=C.AUTHORITY_GRANTED,
        ),
        arm_fsm=C.FsmState(raw="MANUAL", korean="수동"),
        arm_status=C.FsmState(raw="READY", korean="준비"),
        tool_fsm=C.FsmState(raw="OPENING", korean="열리는 중"),
        teleop=_teleop(1, ("작업 준비", "이송")),
        diagnostics=C.ToolDiagnostics(
            motors=(
                C.ToolMotorReading(
                    actuator_id=3, role=C.TARGET_LEFT, position_deg=33.2,
                    opening_ratio=0.55, load_percent=22.0, temperature_c=39.0,
                    torque_on=True, online=True, operating_mode="position",
                ),
                C.ToolMotorReading(
                    actuator_id=4, role=C.TARGET_RIGHT, position_deg=34.9,
                    opening_ratio=0.58, load_percent=25.0, temperature_c=40.0,
                    torque_on=True, online=True, operating_mode="position",
                ),
            ),
            sync_state=C.SYNC_DRIFT,
            sync_error_deg=1.7,
            generation=8,
        ),
        calibration=C.CalibrationState(
            tool_steps=(
                C.CalibrationStep(
                    key=C.TARGET_LEFT, title="좌 끝점", status=C.STEP_DONE,
                    measured="열림 12.0° / 닫힘 78.5°", verified="반복오차 0.4°",
                ),
                C.CalibrationStep(
                    key=C.TARGET_RIGHT, title="우 끝점", status=C.STEP_RUNNING,
                    measured="열림 12.4°",
                ),
            ),
            session_active=True,
            session_target="듀얼 끝점",
            unsaved_results=True,
        ),
        capabilities=ALL_CAPABILITIES,
        is_fixture=True,
    )


def cleaner_no_grant() -> C.ArmUiState:
    """A non-gripper tool, MANUAL requested but not yet granted."""
    return C.ArmUiState(
        link=C.SourceLink(state=C.LINK_LIVE, age_s=0.11),
        detected_tool=C.ToolIdentity(
            kind=C.TOOL_CLEANER,
            tool_id="TOOL-C1",
            display_name="청소 모듈",
            interface="Dynamixel",
            attached=True,
            actuator_ids=(),
            generation=9,
        ),
        tool_change=C.ToolChangeRequest(
            status=C.REQUEST_REJECTED,
            requested_kind=C.TOOL_SINGLE_GRIPPER,
            detail="도구 체결 확인 실패",
        ),
        authority=C.ControlAuthority(
            requested_mode=C.MODE_MANUAL,
            granted_mode="",
            status=C.AUTHORITY_REQUESTED,
        ),
        arm_fsm=C.FsmState(raw="IDLE", korean="대기"),
        arm_status=C.FsmState(raw="READY", korean="준비"),
        tool_fsm=C.FsmState(raw="STOPPED", korean="정지"),
        block_reasons=(
            C.BlockReason("mode_not_granted", "수동 제어권이 승인되지 않았습니다."),
        ),
        teleop=_teleop(None, ()),
        capabilities=frozenset({
            C.CAP_TOOL_CHANGE, C.CAP_CONTROL_MODE, C.CAP_TELEOP_JOG, C.CAP_TELEOP_POSE,
        }),
        is_fixture=True,
    )


SCENARIOS: dict[str, tuple[str, object]] = {
    "disconnected": ("미연결", disconnected),
    "waiting": ("수신 대기", waiting),
    "single": ("단일 그리퍼 (LIVE)", single_gripper),
    "dual": ("듀얼 그리퍼 (LIVE·보정 중)", dual_gripper),
    "cleaner": ("청소 모듈 (제어권 미승인)", cleaner_no_grant),
    "stale": ("단일 그리퍼 (STALE)", stale_single),
}
