"""Tool selection policy separated from future physical detection sources."""

from dataclasses import dataclass

from dynamixel_control.tool_profiles import (
    ToolProfileError, get_profile, validate_profile)


@dataclass(frozen=True)
class ToolSelection:
    tool_type: str
    profile: dict
    valid: bool
    reason: str


class ToolIdentityProvider:
    """Interface for a future lock/tool-ID sensor implementation."""

    def detected_tool_type(self):
        return None


class ParameterToolIdentityProvider(ToolIdentityProvider):
    def __init__(self, tool_type):
        self._tool_type = tool_type

    def detected_tool_type(self):
        return self._tool_type


class BusToolIdentityProvider(ToolIdentityProvider):
    """Identify one exact, non-arm actuator signature from profile IDs."""

    def __init__(self, profiles, probe, excluded_ids=()):
        excluded = {int(item) for item in excluded_ids}
        self._probe = probe
        self._signatures = {}
        for tool_type, profile in profiles.items():
            signature = frozenset(
                int(item) for item in profile.get('actuator_ids', []))
            if signature and not signature & excluded:
                self._signatures[tool_type] = signature
        self.last_present_ids = frozenset()
        self.last_reason = 'not scanned'

    @property
    def candidate_ids(self):
        return frozenset().union(*self._signatures.values()) \
            if self._signatures else frozenset()

    @property
    def supported_tool_types(self):
        return frozenset(self._signatures)

    def detected_tool_type(self):
        present = set()
        for actuator_id in sorted(self.candidate_ids):
            try:
                if self._probe(actuator_id):
                    present.add(actuator_id)
            except Exception:
                continue
        self.last_present_ids = frozenset(present)
        matches = [name for name, signature in self._signatures.items()
                   if signature == self.last_present_ids]
        if len(matches) == 1:
            self.last_reason = ''
            return matches[0]
        self.last_reason = ('no supported tool actuator detected' if not present
                            else f'ambiguous actuator IDs: {sorted(present)}')
        return None


class ToolManager:
    """Resolve profiles and enforce that selection changes occur while safe."""

    SAFE_CHANGE_STATES = frozenset({'IDLE', 'STOWED', 'STOWED_LOCKED'})

    def __init__(self, profiles, identity_provider, mock_mode=False):
        self._profiles = profiles
        self._provider = identity_provider
        self._mock_mode = mock_mode
        self._selection = None

    @property
    def selection(self):
        return self._selection

    def refresh(self, state):
        requested = self._provider.detected_tool_type()
        if self._selection and requested != self._selection.tool_type \
                and state not in self.SAFE_CHANGE_STATES:
            raise ToolProfileError(
                f'tool change {self._selection.tool_type}->{requested} denied in {state}')
        profile = get_profile(self._profiles, requested)
        errors = validate_profile(requested, profile, self._mock_mode)
        self._selection = ToolSelection(
            requested, profile, not errors, '; '.join(errors))
        return self._selection

    def create_fsm(self, bridge, state='IDLE'):
        """Select the mechanism FSM without making a Dynamixel write.

        An uncalibrated profile still receives its FSM so it can enter the
        explicit CALIBRATION_REQUIRED state; it is not motion-valid.
        """
        from dynamixel_control.tool_fsm.registry import create_tool_fsm
        selection = self.refresh(state)
        return create_tool_fsm(selection.tool_type, selection.profile, bridge)
