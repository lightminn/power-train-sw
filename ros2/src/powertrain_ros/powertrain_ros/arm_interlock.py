"""ROS-independent contract-v2 safety gate for robot-arm collaboration."""

from dataclasses import dataclass
from math import isfinite

from powertrain_ros import contract


@dataclass(frozen=True)
class ArmSnapshot:
    """Last accepted arm heartbeat."""

    status: str
    mission_id: int
    stamp_s: float


class ArmInterlock:
    """Fail-closed arm heartbeat, drive-profile, and work-ACK state core."""

    def __init__(self, timeout_s=0.5, future_tolerance_s=0.1):
        self.timeout_s = float(timeout_s)
        self.future_tolerance_s = float(future_tolerance_s)
        self._sample = None
        self._last_seen_stamp_s = 0.0
        self._last_now_s = 0.0
        self._grip_lost_latched = False
        self._contract_violation = None
        self._stamp_domain_latched = False

    def update(self, status, mission_id, stamp_s, now_s):
        stamp_s = float(stamp_s)
        now_s = float(now_s)

        if now_s < self._last_now_s:
            self._sample = None
            self._last_seen_stamp_s = 0.0
            self._last_now_s = now_s
            return False
        self._last_now_s = now_s

        if self._stamp_domain_latched:
            return False
        if not isfinite(stamp_s):
            return False
        if stamp_s <= 0.0 or stamp_s > now_s + self.future_tolerance_s:
            return False
        if stamp_s <= self._last_seen_stamp_s:
            regression_s = self._last_seen_stamp_s - stamp_s
            if regression_s > max(self.timeout_s, self.future_tolerance_s):
                self._sample = None
                # Keep the monotonic baseline: only a process restart may recover.
                self._stamp_domain_latched = True
                self._contract_violation = (
                    "stamp_domain:"
                    f"stamp={stamp_s:.9f},last={self._last_seen_stamp_s:.9f}"
                )
            return False
        if status not in contract.ARM_STATUSES:
            self._contract_violation = str(status)
            return False

        self._last_seen_stamp_s = stamp_s
        self._contract_violation = None
        self._sample = ArmSnapshot(str(status), int(mission_id), stamp_s)
        if status == contract.ARM_GRIP_LOST:
            self._grip_lost_latched = True
        return True

    def fresh(self, now_s):
        now_s = float(now_s)
        if now_s < self._last_now_s:
            self._sample = None
            self._last_seen_stamp_s = 0.0
            self._last_now_s = now_s
            return False
        self._last_now_s = now_s
        age = now_s - self._sample.stamp_s if self._sample is not None else float("inf")
        return 0.0 <= age <= self.timeout_s

    def drive_allowed(self, profile, now_s, manual_override=False):
        if self._grip_lost_latched or self._contract_violation is not None:
            return False
        if manual_override:
            return profile == "REMOTE_ARM_OVERRIDE" and not self.fresh(now_s)
        if not self.fresh(now_s):
            return False
        if profile == "EMPTY_STOWED":
            expected = contract.ARM_STOWED_LOCKED
        elif profile == "CARRYING_LOCKED":
            expected = contract.ARM_CARRYING_LOCKED
        else:
            return False
        return self._sample.status == expected

    def work_acknowledged(self, mission_id, now_s):
        return (
            self._contract_violation is None
            and self.fresh(now_s)
            and self._sample.status in contract.WORK_ACCEPTED_STATUSES
            and self._sample.mission_id == int(mission_id)
        )

    def clear_grip_lost(self, authorized=False):
        if not authorized:
            return False
        self._grip_lost_latched = False
        return True

    @property
    def last_contract_violation(self):
        return self._contract_violation

    def hold_reason(self, profile, now_s, manual_override=False):
        if self._grip_lost_latched:
            return "grip_lost_latched"
        if self._contract_violation is not None:
            return f"arm_contract_violation:{self._contract_violation}"
        if manual_override:
            if profile != "REMOTE_ARM_OVERRIDE":
                return "operator_override_profile_invalid"
            if self.fresh(now_s):
                return "operator_override_inhibited_by_fresh_arm"
            return ""
        if not self.fresh(now_s):
            return "arm_status_stale"
        if not self.drive_allowed(profile, now_s, manual_override=False):
            return "arm_not_drive_ready"
        return ""

    def operator_status(self, profile, now_s, manual_override=False):
        if manual_override and profile == "REMOTE_ARM_OVERRIDE" and not self.fresh(now_s):
            return "operator_override_active"
        return ""
