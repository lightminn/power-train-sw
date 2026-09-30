"""Read-only startup validation for tool FSMs."""


def validate_single_motor_startup(profile, bridge):
    """Return a snapshot after reads only; raise on an unsafe observation."""
    ids = profile.get('actuator_ids')
    if ids != [5]:
        raise ValueError('spur_1motor_gripper requires actuator_ids == [5]')
    # The bridge allowlist is policy state, not a register write.
    bridge.set_allowlist([5])
    model = bridge.read_model(5) if hasattr(bridge, 'read_model') else None
    position = bridge.read_position(5)
    torque = bridge.read_torque(5)
    hardware_error = bridge.read_hardware_error(5)
    if position is None or torque not in (0, 1):
        raise RuntimeError('ID5 feedback unavailable')
    if hardware_error != 0:
        raise RuntimeError(f'ID5 hardware error: {hardware_error}')
    expected = profile.get('motor_model')
    if expected is not None and model is not None and model != expected:
        raise RuntimeError(f'ID5 model {model!r} incompatible with {expected!r}')
    return {'id': 5, 'model': model, 'position': int(position),
            'torque': int(torque), 'hardware_error': int(hardware_error)}


def validate_dual_motor_startup(profile, bridge):
    """Read and validate both dual motors without changing any register."""
    ids = profile.get('actuator_ids')
    if ids != [3, 4]:
        raise ValueError('dual_motor_gripper requires actuator_ids == [3, 4]')
    bridge.set_allowlist(ids)
    if not profile.get('calibrated', False):
        raise RuntimeError('dual motor profile is not calibrated')
    if not profile.get('endpoint_calibration_verified', False):
        raise RuntimeError('dual endpoint calibration is not verified')
    endpoints = profile.get('motor_endpoints') or {}
    expected_models = profile.get('endpoint_calibration_models') or {}
    snapshots = {}
    for dxl_id in ids:
        endpoint = endpoints.get(dxl_id, endpoints.get(str(dxl_id)))
        if not endpoint or endpoint.get('open') == endpoint.get('close'):
            raise RuntimeError(f'ID{dxl_id} calibrated endpoints are invalid')
        model = bridge.read_model(dxl_id) if hasattr(bridge, 'read_model') else None
        position = bridge.read_position(dxl_id)
        torque = bridge.read_torque(dxl_id)
        hardware_error = bridge.read_hardware_error(dxl_id)
        if position is None or torque not in (0, 1):
            raise RuntimeError(f'ID{dxl_id} feedback unavailable')
        if hardware_error != 0:
            raise RuntimeError(f'ID{dxl_id} hardware error: {hardware_error}')
        expected = expected_models.get(dxl_id, expected_models.get(str(dxl_id)))
        if expected is not None and model is not None and model != expected:
            raise RuntimeError(
                f'ID{dxl_id} model {model!r} incompatible with {expected!r}')
        snapshots[dxl_id] = {
            'id': dxl_id, 'model': model, 'position': int(position),
            'torque': int(torque), 'hardware_error': int(hardware_error)}
    return snapshots
