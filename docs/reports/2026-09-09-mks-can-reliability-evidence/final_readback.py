"""Final read-only acceptance after the sustained CAN query trial."""
import collections
import datetime
import json
import pathlib
import time
import can
import odrive
import odrive.configuration
from chassis.usb_session import motor_session

out = pathlib.Path('/evidence/mks-reliability')
record = {'started_at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
          'boot_id': pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip()}
with motor_session('mks_final_readback'):
    board = odrive.find_any(serial_number='336A33523235', timeout=15)
    record['patch'] = board.can.reliability_patch
    assert record['patch'] == 1
    expected = json.loads((out / '336A33523235-configuration.json').read_text())
    record['configuration_unchanged'] = odrive.configuration.get_dict(board, board, False) == expected
    assert record['configuration_unchanged']
    record['can_diagnostics'] = {name: getattr(board.can, name) for name in (
        'hal_error', 'last_hal_error', 'hal_error_history', 'tx_drop_count', 'tx_error_count',
        'recovery_count', 'recovery_failure_count', 'reinit_requested', 'reinitializing')}
    assert record['can_diagnostics']['hal_error'] == 0
    record['axes'] = [{'node': a.config.can_node_id, 'state': a.current_state, 'error': a.error,
        'motor_error': a.motor.error, 'encoder_error': a.encoder.error,
        'calibrated': a.motor.is_calibrated, 'ready': a.encoder.is_ready,
        'motor_pre_calibrated': a.motor.config.pre_calibrated,
        'encoder_pre_calibrated': a.encoder.config.pre_calibrated,
        'input_vel': a.controller.input_vel, 'velocity': a.encoder.vel_estimate,
        'watchdog_enabled': a.config.enable_watchdog, 'watchdog_timeout': a.config.watchdog_timeout}
        for a in (board.axis0, board.axis1)]
    record['vbus_voltage'] = board.vbus_voltage
    (out / 'final-hardware-result.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(record, indent=2), flush=True)
    assert all(a['state'] == 1 and a['error'] in (0, 0x4000) and a['motor_error'] == a['encoder_error'] == 0
        and a['calibrated'] and a['ready'] and a['input_vel'] == 0 and abs(a['velocity']) < .01
        for a in record['axes'])
    bus = can.Bus(interface='socketcan', channel='can0')
    counts = collections.Counter()
    states = {}
    until = time.monotonic() + 6
    try:
        while time.monotonic() < until:
            m = bus.recv(.05)
            if m is None:
                continue
            assert not m.is_error_frame
            if m.is_remote_frame:
                continue
            if not m.is_extended_id and m.arbitration_id & 31 == 1 and m.arbitration_id >> 5 in range(11, 17):
                node = m.arbitration_id >> 5
                states[node] = {'error': int.from_bytes(m.data[:4], 'little'), 'state': m.data[4]}
                counts[str(node)] += 1
            elif m.is_extended_id and m.arbitration_id >> 8 == 41 and m.arbitration_id & 255 in (1, 2, 3, 4):
                counts['ak' + str(m.arbitration_id & 255)] += 1
    finally:
        bus.shutdown()
    record['heartbeat_counts'] = counts
    record['states'] = states
    assert len(counts) == 10 and min(counts.values()) >= 250
    assert len(states) == 6 and all(s['state'] == 1 and s['error'] in (
        (0, 0x4000) if node in (13, 14) else (0,)) for node, s in states.items())
    assert board.can.hal_error == 0 and not board.can.reinit_requested and not board.can.reinitializing
    assert board.can.recovery_count == record['can_diagnostics']['recovery_count']
    record['recovery_count_stable_during_six_seconds'] = True
    record['axes_after_check'] = [{'state': a.current_state, 'error': a.error,
        'latched': a.can_recovery_latched, 'velocity': a.encoder.vel_estimate}
        for a in (board.axis0, board.axis1)]
    assert all(a['state'] == 1 and a['error'] == 0 and not a['latched'] and abs(a['velocity']) < .01
               for a in record['axes_after_check'])
    assert odrive.configuration.get_dict(board, board, False) == expected
    record['result'] = 'PASS'
    (out / 'final-hardware-result.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(record, indent=2), flush=True)
