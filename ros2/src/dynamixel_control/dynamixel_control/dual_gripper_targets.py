"""Shared target calculation for the calibrated dual-motor gripper."""


def dual_relative_targets(profile, actuator_ids, positions, command, step_ticks):
    """Return endpoint-directed relative targets using profile endpoint signs."""
    endpoint_command = str(command).strip().upper()
    if endpoint_command not in ('OPEN', 'CLOSE'):
        raise ValueError(f'unsupported dual target direction {command!r}')
    endpoints = profile.get('motor_endpoints') or {}
    selected_endpoints = {}
    spans = {}
    fractions = {}
    for dxl_id in actuator_ids:
        endpoint = endpoints.get(dxl_id, endpoints.get(str(dxl_id)))
        if not endpoint:
            raise ValueError(f'missing dual endpoint for ID{dxl_id}')
        selected_endpoints[dxl_id] = endpoint
        span = int(endpoint['open']) - int(endpoint['close'])
        if span == 0:
            raise ValueError(f'zero dual endpoint span for ID{dxl_id}')
        spans[dxl_id] = span
        fractions[dxl_id] = (
            (float(positions[dxl_id]) - int(endpoint['close'])) / span)
    if any(not 0 <= value <= 1 for value in fractions.values()):
        raise ValueError('jog position outside motor endpoints')
    if max(fractions.values()) - min(fractions.values()) > 0.05:
        raise ValueError('dual jog synchronization fault')
    step = float(step_ticks) / max(abs(span) for span in spans.values())
    opening = endpoint_command == 'OPEN'
    remaining = min((1 - value if opening else value)
                    for value in fractions.values())
    delta = min(step, remaining) * (1 if opening else -1)
    return {
        dxl_id: max(
            min(int(selected_endpoints[dxl_id]['open']),
                int(selected_endpoints[dxl_id]['close'])),
            min(
                max(int(selected_endpoints[dxl_id]['open']),
                    int(selected_endpoints[dxl_id]['close'])),
                round(float(positions[dxl_id]) + delta * spans[dxl_id])))
        for dxl_id in actuator_ids
    }
