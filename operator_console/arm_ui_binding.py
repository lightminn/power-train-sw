"""Read-only projection from console arm telemetry into the new tab view model."""
from __future__ import annotations

import math
import time

from .arm_ui import contracts as C


_TOOL_KINDS = {
    "spur_1motor_gripper": C.TOOL_SINGLE_GRIPPER,
    "dual_motor_gripper": C.TOOL_DUAL_GRIPPER,
    "cleaner": C.TOOL_CLEANER,
}


class ArmUiTelemetryBinding:
    """Keep tool generations stable while only observing the UDP snapshot.

    This deliberately supplies no capabilities or callbacks.  The new tabs can
    show the same live observations as the existing UI, but cannot issue an arm
    command until a separately authenticated ops adapter exists.
    """

    def __init__(self) -> None:
        self._tool_key = None
        self._tool_generation = 0

    @staticmethod
    def _age(snapshot, source_age_s):
        if snapshot is None or source_age_s is None:
            return None
        return source_age_s + max(0.0, time.monotonic() - snapshot.received_monotonic_s)

    def state(self, snapshot, *, ops_link_ready=False):
        if snapshot is None:
            return C.default_state()
        receive_age = max(0.0, time.monotonic() - snapshot.received_monotonic_s)
        link_state = C.LINK_LIVE if receive_age <= 1.0 else C.LINK_STALE
        runtime = snapshot.tool_runtime or {}
        tool_age = self._age(snapshot, runtime.get("source_age_s"))
        tool = runtime.get("tool") if tool_age is not None and tool_age <= 1.0 else None
        if tool is None:
            identity = C.ToolIdentity(generation=self._tool_generation)
            tool_key = None
            diagnostics = C.ToolDiagnostics(generation=self._tool_generation)
        else:
            kind = _TOOL_KINDS.get(tool["tool_type"], C.TOOL_UNKNOWN)
            ids = tuple(tool["actuator_ids"])
            tool_key = (kind, ids)
            if tool_key != self._tool_key:
                self._tool_generation += 1
            # The arm status has no hardware serial number.  This is therefore
            # an observed active-profile fingerprint, not an invented serial.
            profile_id = f"{tool['tool_type']}:{','.join(map(str, ids))}"
            identity = C.ToolIdentity(
                kind=kind,
                tool_id=profile_id,
                display_name=C.TOOL_KIND_KOREAN[kind],
                actuator_ids=ids,
                attached=(False if tool.get("tool_detached") or tool.get("physical_tool_detached")
                          else None),
                generation=self._tool_generation,
            )
            readings = tuple(
                C.ToolMotorReading(
                    actuator_id=row["id"],
                    torque_on=(True if row.get("torque_state") == "ON" else
                               False if row.get("torque_state") == "OFF" else None),
                    online=row.get("online"),
                    operating_mode=("" if row.get("operating_mode") is None
                                    else str(row["operating_mode"])),
                    error=("" if row.get("hardware_error") is None
                           else str(row["hardware_error"])),
                )
                for row in tool["actuators"]
            )
            diagnostics = C.ToolDiagnostics(
                motors=readings, generation=self._tool_generation,
            )
        self._tool_key = tool_key

        joints_age = self._age(snapshot, snapshot.joints_age_s)
        axes = ()
        if joints_age is not None and joints_age <= 1.0:
            axes = tuple(
                C.AxisState(index=index + 1, name=name,
                            angle_deg=math.degrees(position))
                for index, (name, position) in enumerate(
                    zip(snapshot.joint_names, snapshot.joint_position_rad)
                )
            )
        arm_runtime = snapshot.arm_runtime or {}
        runtime_ages = arm_runtime.get("source_age_s", {})

        def runtime_state(key):
            value = arm_runtime.get(key)
            age = self._age(snapshot, runtime_ages.get(key))
            return C.FsmState(raw=value) if value and age is not None and age <= 1.0 else C.FsmState()

        manual = arm_runtime.get("control_mode")
        manual_age = self._age(snapshot, runtime_ages.get("control_mode"))
        granted = manual == "MANUAL" and manual_age is not None and manual_age <= 1.0
        capabilities = set()
        if link_state == C.LINK_LIVE and ops_link_ready:
            capabilities.add(C.CAP_CONTROL_MODE)
        if (link_state == C.LINK_LIVE and ops_link_ready
                and tool is not None and tool.get("motion_allowed") is True
                and tool.get("read_only") is not True
                and tool.get("emergency_stop") is not True):
            capabilities.add(C.CAP_GRIPPER_COMMAND)
        return C.ArmUiState(
            link=C.SourceLink(state=link_state, age_s=receive_age),
            detected_tool=identity,
            arm_fsm=runtime_state("fsm_state"),
            arm_status=runtime_state("arm_status"),
            authority=C.ControlAuthority(
                granted_mode="MANUAL" if granted else "",
                status=C.AUTHORITY_GRANTED if granted else C.AUTHORITY_UNKNOWN,
            ),
            teleop=C.TeleopState(axes=axes),
            diagnostics=diagnostics,
            capabilities=frozenset(capabilities),
        )
