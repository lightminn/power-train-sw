"""Read current patch1 flash twice; no erase, write, configuration save or arm."""
import hashlib
import json
import odrive
from chassis.usb_session import motor_session, read_path
from tools.mks_dfu import BASE, FLASH_SIZE, APP_SIZE, validate_image
from flash_board import SERIAL, OUT, check_idle, enter_dfu, return_to_app, get_config, record

result = {'serial': SERIAL, 'operations': record['operations']}

def save():
    (OUT / 'current-backup-result.json').write_text(json.dumps(result, indent=2) + '\n')

def save_config(label, cfg):
    text = json.dumps(cfg, indent=2, sort_keys=True) + '\n'
    with (OUT / (SERIAL + '-' + label + '-configuration.json')).open('x') as f:
        f.write(text)
    return hashlib.sha256(text.encode()).hexdigest()

def differences(a, b, path=''):
    if isinstance(a, dict) and isinstance(b, dict):
        assert a.keys() == b.keys(), path
        return [item for k in a for item in differences(a[k], b[k], path + '.' + k)]
    return [] if a == b else [(path.lstrip('.'), a, b)]

def backup_stopped(board):
    # Read-only ROM backup may preserve an observed non-CAN fault. This helper
    # cannot arm or program. The application updater keeps its strict check.
    assert format(board.serial_number, 'X') == SERIAL
    assert (board.hw_version_major, board.hw_version_minor, board.hw_version_variant) == (3, 6, 56)
    assert [board.axis0.config.can_node_id, board.axis1.config.can_node_id] == [13, 14]
    states = []
    for axis in (board.axis0, board.axis1):
        assert axis.current_state == 1 and axis.requested_state == 0
        assert axis.controller.input_vel == axis.controller.input_torque == 0
        assert abs(axis.encoder.vel_estimate) < 0.01
        assert not axis.can_recovery_in_progress
        assert axis.encoder.error == axis.controller.error == 0
        assert (axis.error, axis.motor.error) in {(0, 0), (0x4000, 0), (0x40, 0x1000)}
        assert axis.motor.is_calibrated and axis.encoder.is_ready
        states.append({p: read_path(axis, p) for p in ('current_state', 'requested_state',
            'error', 'motor.error', 'encoder.error', 'controller.error', 'encoder.vel_estimate',
            'controller.input_vel', 'controller.input_torque', 'can_recovery_latched')})
    return states

with motor_session('mks_patch1_current_backup'):
    board = odrive.find_any(serial_number=SERIAL, timeout=15)
    result['before_axes'] = backup_stopped(board)
    assert board.can.reliability_patch == 1
    result['can_before'] = {p: read_path(board.can, p) for p in (
        'reliability_patch', 'error', 'hal_error', 'last_hal_error', 'hal_error_history',
        'recovery_count', 'recovery_failure_count', 'tx_drop_count', 'tx_error_count')}
    before = get_config(board)
    assert before == get_config(board), 'configuration changed between reads'
    result['before_config_sha256'] = save_config('before', before)
    for name in ('axis0', 'axis1'):
        assert not any(v for k, v in before[name]['config'].items() if k.startswith('startup_'))
    transport = enter_dfu(board)
    try:
        one = transport.read(BASE, FLASH_SIZE)
        two = transport.read(BASE, FLASH_SIZE)
        assert one == two, 'two full ROM reads differ'
        validate_image(one[:APP_SIZE])
        with (OUT / (SERIAL + '-original-1m.bin')).open('xb') as f:
            f.write(one)
        result.update(bytes=len(one), reads_equal=True,
                      sha256=hashlib.sha256(one).hexdigest(),
                      application_sha256=hashlib.sha256(one[:APP_SIZE]).hexdigest(),
                      nvm_sha256=hashlib.sha256(one[APP_SIZE:]).hexdigest())
    except Exception as error:
        result['error'] = repr(error)
        raise
    finally:
        save()
        board = return_to_app(transport)
        result['after_axes'] = check_idle(board)
        assert board.can.reliability_patch == 1
        after = get_config(board)
        assert after == get_config(board)
        result['config_diff'] = differences(before, after)
        allowed = {'axis0.controller.config.input_mode', 'axis1.controller.config.input_mode'}
        assert all(path in allowed and old == 1 and new == 2
                   for path, old, new in result['config_diff']), result['config_diff']
        result['saved_config_sha256'] = save_config('after-original-reboot', after)
        result['configuration_preserved'] = True
        save()
print(json.dumps({k: v for k, v in result.items() if k != 'operations'}, indent=2), flush=True)
