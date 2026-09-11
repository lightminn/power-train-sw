"""Bounded, operator-triggered recovery jogs for the legacy dual gripper."""


class DualManualRecoveryError(RuntimeError):
    pass


class DualManualRecovery:
    """One click reads one actual motor and writes one bounded relative goal."""

    IDS = frozenset((3, 4))
    DELTAS_DEG = frozenset((-0.5, 0.5))
    TICKS_PER_REVOLUTION = 4096

    def __init__(self, bridge):
        self.bridge = bridge

    def jog(self, dxl_id, delta_deg, allowed_degrees=None, goal_writer=None):
        dxl_id = int(dxl_id)
        delta_deg = float(delta_deg)
        if dxl_id not in self.IDS:
            raise DualManualRecoveryError('manual recovery allows only ID3 or ID4')
        allowed = self.DELTAS_DEG if allowed_degrees is None else frozenset(
            float(value) for value in allowed_degrees)
        if delta_deg not in allowed:
            raise DualManualRecoveryError('manual recovery delta is not allowed')
        position = self.bridge.read_manual_position(dxl_id)
        torque = self.bridge.read_manual_torque(dxl_id)
        hardware_error = self.bridge.read_manual_hardware_error(dxl_id)
        if position is None:
            raise DualManualRecoveryError(f'ID{dxl_id} position unavailable')
        if torque != 1:
            raise DualManualRecoveryError(f'ID{dxl_id} actual Torque Enable is OFF')
        if hardware_error != 0:
            raise DualManualRecoveryError(
                f'ID{dxl_id} hardware error: {hardware_error}')
        delta_tick = int(round(delta_deg * self.TICKS_PER_REVOLUTION / 360.0))
        if delta_tick == 0:
            raise DualManualRecoveryError('0.5° conversion produced zero ticks')
        target = int(position) + delta_tick
        (goal_writer or self.bridge.manual_goal_position)(dxl_id, target)
        return target
