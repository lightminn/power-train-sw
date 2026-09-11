"""Robot-arm console tabs: UI skeleton only.

Scope
-----
This package builds two GTK tabs — 로봇팔 수동조작 and 로봇팔·도구 캘리브레이션 —
and nothing else.  It does not open sockets, import ``rclpy``, drive motors,
compute a calibration, or persist a result.  Buttons report operator intent
through injected callbacks, the screen is refreshed from injected state, and a
control with no callback or no reported capability is disabled with a reason.

Boundary
--------
* **In:** widget construction, layout, enable/disable logic, hold lifecycle,
  Korean operator labels, arm-scoped CSS, teardown.
* **Out:** telemetry transport and schema, ops command formats, ROS, motor
  motion, calibration arithmetic and storage, and registering these tabs in
  ``app.py`` — that integration is the connecting work's call.

Nothing here fixes the external telemetry schema.  :mod:`contracts` is the
console-side view model; adapters translate into it.

Usage
-----
    from operator_console.arm_ui import ArmManualTab, ArmUiCallbacks, default_state

    tab = ArmManualTab(ArmUiCallbacks(request_tool_change=my_handler))
    stack.add_titled(tab.widget, ARM_MANUAL_TAB_NAME, ARM_MANUAL_TAB_TITLE)
    tab.attach_keys(window)      # optional: window-level teleop keys
    tab.update_state(state)      # on every refresh
    tab.dispose()                # on shutdown

Preview:  ``/usr/bin/python3 -m operator_console.arm_ui``
"""
from __future__ import annotations

from .calibration_tab import TAB_NAME as ARM_CALIBRATION_TAB_NAME
from .calibration_tab import TAB_TITLE as ARM_CALIBRATION_TAB_TITLE
from .calibration_tab import ArmCalibrationTab
from .contracts import (
    ArmUiCallbacks,
    ArmUiState,
    AxisState,
    BlockReason,
    CalibrationState,
    CalibrationStep,
    ControlAuthority,
    FsmState,
    SourceLink,
    TeleopState,
    ToolChangeRequest,
    ToolDiagnostics,
    ToolIdentity,
    ToolMotorReading,
    default_state,
    gate,
)
from .manual_tab import TAB_NAME as ARM_MANUAL_TAB_NAME
from .manual_tab import TAB_TITLE as ARM_MANUAL_TAB_TITLE
from .manual_tab import ArmManualTab
from .styling import install_arm_ui_css

__all__ = [
    "ARM_CALIBRATION_TAB_NAME",
    "ARM_CALIBRATION_TAB_TITLE",
    "ARM_MANUAL_TAB_NAME",
    "ARM_MANUAL_TAB_TITLE",
    "ArmCalibrationTab",
    "ArmManualTab",
    "ArmUiCallbacks",
    "ArmUiState",
    "AxisState",
    "BlockReason",
    "CalibrationState",
    "CalibrationStep",
    "ControlAuthority",
    "FsmState",
    "SourceLink",
    "TeleopState",
    "ToolChangeRequest",
    "ToolDiagnostics",
    "ToolIdentity",
    "ToolMotorReading",
    "default_state",
    "gate",
    "install_arm_ui_css",
]
