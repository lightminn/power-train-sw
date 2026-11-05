"""Pure DRIVE/ARM remote-input safety gateway.

The client sends a requested mode.  The state in this module is the only ACK.
All physical evidence is injected so this core remains deterministic and can
be tested without ROS or hardware.
"""

from dataclasses import dataclass
import math


DISCONNECTED = "DISCONNECTED"
DRIVE = "DRIVE"
STOPPING_FOR_ARM = "STOPPING_FOR_ARM"
ARM = "ARM"
STOPPING_FOR_DRIVE = "STOPPING_FOR_DRIVE"
MOTION_HOLD = "MOTION_HOLD"

JOINT_NAMES = tuple("joint_%d" % index for index in range(1, 6))


def validate_manual_command_format(command_format, *, assist_enabled=False):
    """Reject unsupported startup combinations before opening control inputs."""
    if command_format not in ("steering", "twist"):
        raise ValueError("manual_command_format must be steering or twist")
    if command_format == "steering" and assist_enabled:
        raise ValueError(
            "assist_enabled=true requires manual_command_format=twist; "
            "steering assist is not qualified")


@dataclass(frozen=True)
class GatewayConfig:
    input_timeout_s: float = 0.20
    stopping_timeout_s: float = 2.0
    max_linear: float = 1.0
    max_angular: float = 1.0
    max_joint_velocity: float = 1.0
    manual_command_format: str = "steering"
    # False (field relaxation): stale input outputs zero without latching
    # MOTION_HOLD, and driving resumes as soon as fresh input returns.
    latch_stale_input: bool = True


@dataclass(frozen=True)
class DriveOutput:
    linear: float = 0.0
    angular: float = 0.0
    steering: float = 0.0
    source_received_s: float = None
    connection_session_id: str = ""


@dataclass(frozen=True)
class ArmOutput:
    joint_name: str = "joint_1"
    joint_velocity: float = 0.0
    gripper: float = 0.0


@dataclass(frozen=True)
class GatewayOutput:
    state: str
    drive: DriveOutput
    arm: ArmOutput
    reason: str = ""
    input_fresh: bool = False
    assist_bypass: bool = False


def gated_arm_output(output, *, enabled=False):
    """Production-default gate shared by the ROS JointJog adapter."""
    if enabled:
        return output
    return ArmOutput(joint_name=output.joint_name)


def frame_is_neutral(frame):
    """Return whether a decoded input frame is fully neutral."""
    axes = frame.axes
    return (
        not frame.deadman
        and not frame.mode_chord
        and not frame.estop_edge
        and axes.left_x == 0.0
        and axes.right_y == 0.0
        and axes.left_trigger == 0.0
        and axes.right_trigger == 0.0
        and frame.dpad.x == 0
        and frame.dpad.y == 0
    )


class RemoteInputGateway:
    """Enforce zero-separated mode transitions and hold-to-run input."""

    def __init__(
        self,
        cfg=None,
        *,
        arm_output_enabled=False,
        wheel_stopped=None,
        wheel_stop_qualified=None,
        arm_stationary_ack=None,
        stow_confirmed=None,
    ):
        self.cfg = cfg or GatewayConfig()
        validate_manual_command_format(self.cfg.manual_command_format)
        for name in (
            "input_timeout_s",
            "stopping_timeout_s",
            "max_linear",
            "max_angular",
            "max_joint_velocity",
        ):
            value = float(getattr(self.cfg, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("%s must be finite and positive" % name)

        self.arm_output_enabled = bool(arm_output_enabled)
        self._wheel_stopped = wheel_stopped or (lambda: False)
        self._wheel_stop_qualified = wheel_stop_qualified or (lambda: False)
        self._arm_stationary_ack = arm_stationary_ack or (lambda: False)
        self._stow_confirmed = stow_confirmed or (lambda: False)

        self.state = DISCONNECTED
        self._connected = False
        self._connection_session_id = ""
        self._connection_ready = False
        self._session_id = None
        self._frame = None
        self._stopping_started_s = None
        self._selected_joint = 0
        self._last_dpad_x = 0
        self._last_reason = "waiting for TCP connection"
        # Input epochs prove a record was made after a stall or HOLD clear: the
        # server issues an epoch, the client echoes the latest one it received.
        self.input_epoch = 0
        self._epoch_issued_s = {}
        self._required_epoch = None
        self._recovery_pending = False
        self._last_submit_received_s = None

    @staticmethod
    def _flag(callback):
        return bool(callback())

    def _zero(self, reason=None, *, fresh=False):
        return GatewayOutput(
            state=self.state,
            drive=DriveOutput(source_received_s=(
                self._frame.received_monotonic_s if fresh and self._frame is not None else None),
                connection_session_id=self._connection_session_id),
            arm=ArmOutput(joint_name=JOINT_NAMES[self._selected_joint]),
            reason=self._last_reason if reason is None else reason,
            input_fresh=fresh,
            assist_bypass=(
                bool(self._frame.assist_bypass)
                if fresh and self._frame is not None
                else False
            ),
        )

    def begin_connection(self, connection_session_id=""):
        self._connected = True
        self._connection_session_id = str(connection_session_id)
        self._connection_ready = False
        self._session_id = None
        self._frame = None
        self._last_dpad_x = 0
        # A new TCP connection carries no pre-stall records; neutral is enough.
        self._required_epoch = None
        self._recovery_pending = False
        self._last_submit_received_s = None
        if self.state != MOTION_HOLD:
            self.state = DISCONNECTED
            self._last_reason = "new TCP connection requires neutral input"

    def end_connection(self):
        self._connected = False
        self._connection_session_id = ""
        self._connection_ready = False
        self._session_id = None
        self._frame = None
        self._last_dpad_x = 0
        self._last_submit_received_s = None
        if self.state != MOTION_HOLD:
            self.state = DISCONNECTED
            self._last_reason = "TCP disconnected"

    def submit(self, frame):
        if not self._connected:
            return False
        if self._session_id is None:
            self._session_id = frame.session_id
        elif frame.session_id != self._session_id:
            self.contract_violation(
                "CONTRACT_VIOLATION: gateway session changed"
            )
            return False
        previous_s = self._last_submit_received_s
        self._last_submit_received_s = frame.received_monotonic_s
        if (
            previous_s is not None
            and frame.received_monotonic_s - previous_s > self.cfg.input_timeout_s
        ):
            # The input expired between ticks; apply the same stale policy the
            # tick would have, before this (possibly pre-stall) record is used.
            if self.cfg.latch_stale_input:
                self._enter_hold("remote input stale")
            elif self._required_epoch is None:
                # Already recovering means a tick saw this stall and issued the
                # epoch the client may now be echoing; keep that requirement.
                self._enter_recovery(previous_s + self.cfg.input_timeout_s)
        self._frame = frame
        return True

    def contract_violation(self, reason):
        self.state = MOTION_HOLD
        self._connection_ready = False
        self._stopping_started_s = None
        self._last_reason = str(reason)

    def clear_hold(self):
        if self.state != MOTION_HOLD:
            return False
        # Records already buffered on this connection predate the clear.
        self._recovery_pending = True
        # The gap that caused the HOLD is handled; measure gaps afresh.
        self._last_submit_received_s = None
        self.state = DISCONNECTED
        self._connection_ready = False
        self._frame = None
        self._session_id = None
        self._stopping_started_s = None
        self._last_reason = "MOTION_HOLD cleared; neutral input required"
        return True

    @staticmethod
    def _neutral(frame):
        return frame_is_neutral(frame)

    def _frame_is_fresh(self, now_s):
        if self._frame is None:
            return False
        age = float(now_s) - self._frame.received_monotonic_s
        return (
            math.isfinite(age)
            and age >= -1e-12
            and age <= self.cfg.input_timeout_s + 1e-12
        )

    def _issue_epoch(self, now_s):
        self.input_epoch += 1
        self._epoch_issued_s[self.input_epoch] = float(now_s)
        for old in [e for e in self._epoch_issued_s if e < self.input_epoch - 8]:
            del self._epoch_issued_s[old]

    def _enter_recovery(self, now_s):
        self._issue_epoch(now_s)
        self._required_epoch = self.input_epoch

    def _epoch_is_valid(self, frame, now_s):
        epoch = frame.input_epoch
        issued = self._epoch_issued_s.get(epoch)
        return (
            epoch is not None
            and epoch >= self._required_epoch
            and issued is not None
            and float(now_s) - issued <= self.cfg.input_timeout_s + 1e-12
        )

    def _enter_hold(self, reason, *, fresh=False):
        self.state = MOTION_HOLD
        self._connection_ready = False
        self._stopping_started_s = None
        self._last_reason = reason
        return self._zero(reason, fresh=fresh)

    def _qualified(self):
        try:
            return self._flag(self._wheel_stop_qualified), ""
        except Exception as exc:
            return False, "wheel-stop qualification error: %s" % exc

    def _request_transition(self, target, now_s):
        if target == ARM and not self.arm_output_enabled:
            self._last_reason = "ARM output disabled"
            return self._zero(self._last_reason, fresh=True)
        qualified, error = self._qualified()
        if error:
            self._last_reason = error
            return self._zero(error, fresh=True)
        if not qualified:
            self._last_reason = "wheel-stop predicate unqualified"
            return self._zero(self._last_reason, fresh=True)

        self.state = (
            STOPPING_FOR_ARM if target == ARM else STOPPING_FOR_DRIVE
        )
        self._stopping_started_s = float(now_s)
        self._last_reason = "zero commanded before %s" % target
        return self._zero(self._last_reason, fresh=True)

    def _tick_stopping(self, now_s):
        elapsed = float(now_s) - self._stopping_started_s
        if not math.isfinite(elapsed) or elapsed < 0.0:
            return self._enter_hold(
                "clock rollback during mode transition",
                fresh=True,
            )
        if elapsed + 1e-12 >= self.cfg.stopping_timeout_s:
            return self._enter_hold("mode transition timeout", fresh=True)

        qualified, error = self._qualified()
        if error:
            return self._enter_hold(error, fresh=True)
        if not qualified:
            return self._enter_hold(
                "wheel-stop qualification lost",
                fresh=True,
            )

        if self.state == STOPPING_FOR_ARM:
            try:
                wheels_stopped = self._flag(self._wheel_stopped)
                arm_stationary = self._flag(self._arm_stationary_ack)
            except Exception as exc:
                return self._enter_hold(
                    "DRIVE to ARM ACK callback error: %s" % exc,
                    fresh=True,
                )
            if not wheels_stopped:
                return self._zero("wheel-stop ACK pending", fresh=True)
            if not arm_stationary:
                return self._zero("arm stationary ACK pending", fresh=True)
            self.state = ARM
            self._stopping_started_s = None
            self._last_reason = "ARM mode ACK"
            return self._zero(self._last_reason, fresh=True)

        try:
            stowed = self._flag(self._stow_confirmed)
        except Exception as exc:
            return self._enter_hold(
                "stow-confirm callback error: %s" % exc,
                fresh=True,
            )
        if not stowed:
            return self._zero("STOW_REQUEST ACK pending", fresh=True)
        self.state = DRIVE
        self._stopping_started_s = None
        self._last_reason = "DRIVE mode ACK after stow-confirm"
        return self._zero(self._last_reason, fresh=True)

    def _drive_output(self, frame):
        if not frame.deadman:
            self._last_reason = "drive deadman released"
            return self._zero(self._last_reason, fresh=True)
        # 2026-11-06 operator request: LT drives forward, RT reverses (the arm
        # gripper below keeps its RT-open/LT-close mapping).
        linear = (
            frame.axes.left_trigger - frame.axes.right_trigger
        ) * self.cfg.max_linear
        # Steering is independent of speed, including at zero and in reverse.
        # SDL stick-right is positive; our wheel-frame steering is left-positive.
        steering = -frame.axes.left_x
        angular = 0.0
        if self.cfg.manual_command_format == "twist":
            angular = steering * self.cfg.max_angular
            steering = 0.0
        self._last_reason = "DRIVE input"
        return GatewayOutput(
            state=self.state,
            drive=DriveOutput(linear, angular, steering, frame.received_monotonic_s,
                              self._connection_session_id),
            arm=ArmOutput(joint_name=JOINT_NAMES[self._selected_joint]),
            reason=self._last_reason,
            input_fresh=True,
            assist_bypass=bool(frame.assist_bypass),
        )

    def _arm_output(self, frame):
        dpad_x = frame.dpad.x
        if dpad_x != 0 and self._last_dpad_x == 0:
            self._selected_joint = (
                self._selected_joint + (1 if dpad_x > 0 else -1)
            ) % len(JOINT_NAMES)
        self._last_dpad_x = dpad_x

        if not frame.deadman:
            self._last_reason = "arm deadman released; pose hold"
            return self._zero(self._last_reason, fresh=True)
        if (
            frame.axes.left_trigger > 0.0
            and frame.axes.right_trigger > 0.0
        ):
            self._last_reason = "conflicting gripper triggers; arm hold"
            return self._zero(self._last_reason, fresh=True)

        joint_velocity = (
            frame.axes.right_y * self.cfg.max_joint_velocity
        )
        gripper = frame.axes.right_trigger - frame.axes.left_trigger
        self._last_reason = "ARM input"
        return GatewayOutput(
            state=self.state,
            drive=DriveOutput(),
            arm=ArmOutput(
                joint_name=JOINT_NAMES[self._selected_joint],
                joint_velocity=joint_velocity,
                gripper=gripper,
            ),
            reason=self._last_reason,
            input_fresh=True,
            assist_bypass=bool(frame.assist_bypass),
        )

    def tick(self, now_s):
        try:
            now_s = float(now_s)
        except (TypeError, ValueError):
            return self._enter_hold("invalid local monotonic time")
        if not math.isfinite(now_s):
            return self._enter_hold("invalid local monotonic time")
        if self.state == MOTION_HOLD:
            return self._zero(self._last_reason)
        if self._recovery_pending:
            self._recovery_pending = False
            self._enter_recovery(now_s)
        if (
            self._required_epoch is not None
            and now_s - self._epoch_issued_s[self.input_epoch]
            > self.cfg.input_timeout_s / 2.0
        ):
            # Keep a recent epoch in flight so post-stall input can qualify.
            self._issue_epoch(now_s)
        if not self._connected or self._frame is None:
            return self._zero(self._last_reason)
        if not self._frame_is_fresh(now_s):
            if not self.cfg.latch_stale_input:
                if self._required_epoch is None:
                    self._enter_recovery(now_s)
                return self._zero("remote input stale; zero until fresh input")
            return self._enter_hold("remote input stale")

        frame = self._frame
        if frame.estop_edge:
            return self._enter_hold("remote E-stop edge", fresh=True)

        if self._required_epoch is not None:
            if not self._epoch_is_valid(frame, now_s):
                return self._zero("waiting for input made after the stall")
            self._required_epoch = None

        if not self._connection_ready:
            if not self._neutral(frame):
                self._last_reason = "neutral input required after connection"
                return self._zero(self._last_reason, fresh=True)
            self._connection_ready = True
            self.state = DRIVE
            self._last_reason = "DRIVE mode ACK after neutral input"
            return self._zero(self._last_reason, fresh=True)

        if self.state in (STOPPING_FOR_ARM, STOPPING_FOR_DRIVE):
            return self._tick_stopping(now_s)

        requested_state = frame.mode
        if requested_state != self.state:
            if not frame.mode_chord:
                self._last_reason = "mode change requires mode_chord"
                return self._zero(self._last_reason, fresh=True)
            return self._request_transition(requested_state, now_s)

        if self.state == DRIVE:
            return self._drive_output(frame)
        if self.state == ARM:
            return self._arm_output(frame)
        return self._enter_hold("invalid gateway state: %s" % self.state)
