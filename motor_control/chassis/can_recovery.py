"""Bounded CAN recovery evidence; no transport or actuator side effects."""
import math


class CanRecovery:
    def __init__(self):
        self.state = "IDLE"
        self.reason = ""
        self.count = 0
        self.started_s = None
        self.stable_s = None
        self.rearmed_s = None
        self.generations = {}
        self.required_nodes = set()
        self.pending_command = None
        self.connection_session_id = None

    @property
    def pending(self):
        return self.state in {"WAIT_STOP", "WAIT_INPUT"}

    def reset(self):
        self.state, self.reason = "IDLE", "explicit arm"
        self.started_s = self.stable_s = self.rearmed_s = None
        self.generations = {}
        self.required_nodes = set()
        self.pending_command = None
        self.connection_session_id = None

    def begin(self, now_s, reason):
        if not self.pending:
            self.started_s = now_s
            self.generations = {}
            self.required_nodes = set()
        self.state, self.reason = "WAIT_STOP", reason
        self.stable_s = self.rearmed_s = None
        self.pending_command = None

    def cancel(self, reason):
        self.state, self.reason = "CANCELLED", reason
        self.stable_s = None
        self.pending_command = None

    def snapshot(self):
        return {"state": self.state, "reason": self.reason, "count": self.count,
                "required_nodes": sorted(self.required_nodes)}

    @staticmethod
    def fresh(state):
        ages = (state.get("last_heartbeat_age_ms"), state.get("last_encoder_age_ms"))
        return all(isinstance(age, (int, float)) and math.isfinite(age)
                   and 0 <= age <= 200.0 for age in ages)

    @staticmethod
    def eligible(state):
        generation = state.get("can_recovery_generation")
        return (state.get("can_recovery_supported") is True
                and state.get("can_recovery_latched") is True
                and state.get("can_auto_resume_eligible") is True
                and type(generation) is int and 0 <= generation <= 255
                and state.get("axis_error") == 0x4000
                and state.get("axis_state") == 1)

    def stopped_evidence(self, states, now_s):
        """Return (ready, fatal_reason); stale evidence can only wait."""
        stable = True
        found = False
        for state in states:
            if not self.fresh(state):
                self.required_nodes.add(state["node_id"])
                stable = False
                continue
            node = state["node_id"]
            error = state.get("axis_error")
            eligible = self.eligible(state)
            if error != 0 and not eligible:
                return False, "ineligible_axis_error:%s:%s" % (node, error)
            if node in self.generations and not eligible:
                return False, "recovery_eligibility_revoked:%s" % node
            if eligible:
                self.required_nodes.add(node)
                found = True
                generation = state["can_recovery_generation"]
                if self.generations.get(node) != generation:
                    self.stable_s = None
                self.generations[node] = generation
            if node in self.required_nodes and not eligible:
                stable = False
            velocity = state.get("actual_vel")
            stable = stable and (
                state.get("axis_state") == 1
                and not state.get("can_recovery_in_progress", False)
                and isinstance(velocity, (int, float)) and math.isfinite(velocity)
                and abs(velocity) < .1)
        if not stable or not found:
            self.stable_s = None
            return False, ""
        if self.stable_s is None:
            self.stable_s = now_s
        return now_s - self.stable_s >= .200 - 1e-9, ""

    def observe_interruption(self, states):
        """Capture origins before zero/IDLE writes can change the next sample."""
        for state in states:
            node = state["node_id"]
            if not self.fresh(state):
                self.required_nodes.add(node)
            if self.eligible(state):
                self.required_nodes.add(node)
                self.generations[node] = state["can_recovery_generation"]

    @classmethod
    def arm_guard(cls, expected):
        """Recheck the latest drained sample immediately before destructive clear."""
        def valid(current):
            velocity = current.get("actual_vel", float("nan"))
            if (not cls.fresh(current) or current.get("node_id") != expected["node_id"]
                    or current.get("axis_state") != 1 or not math.isfinite(velocity)
                    or abs(velocity) >= .1 or current.get("can_recovery_in_progress")):
                return False
            if cls.eligible(expected):
                return (cls.eligible(current) and current.get("can_recovery_generation")
                        == expected["can_recovery_generation"])
            return (current.get("axis_error") == 0
                    and not current.get("can_recovery_latched")
                    and current.get("can_recovery_generation")
                    == expected.get("can_recovery_generation"))
        return valid
