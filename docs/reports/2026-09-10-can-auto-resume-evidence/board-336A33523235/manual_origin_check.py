"""Physical patch2 provenance check, with both axes remaining IDLE throughout.

One same-baud manual CAN reinit; explicit clear afterward. No arm, setpoint,
NVM save, gain change or calibration. Production motor writers must be stopped.
"""
import collections
import json
import time
import can
import odrive
from chassis.usb_session import motor_session
from flash_board import SERIAL, OUT, check_idle, get_config

record = {'serial': SERIAL, 'method': 'IDLE manual same-baud reinit, passive CAN, explicit clear; no arm'}

def snapshot(board):
    return [{'node': a.config.can_node_id, 'state': a.current_state, 'error': a.error,
             'motor_error': a.motor.error, 'encoder_error': a.encoder.error,
             'controller_error': a.controller.error, 'input_vel': a.controller.input_vel,
             'input_torque': a.controller.input_torque, 'velocity': a.encoder.vel_estimate,
             'latched': a.can_recovery_latched, 'in_progress': a.can_recovery_in_progress,
             'eligible': a.can_recovery_auto_resume_eligible,
             'generation': a.can_recovery_generation} for a in (board.axis0, board.axis1)]

def passive(bus, seconds):
    counts = collections.Counter()
    states = {}
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        m = bus.recv(.05)
        if m is None:
            continue
        assert not m.is_error_frame
        if m.is_remote_frame:
            continue
        if not m.is_extended_id and m.arbitration_id & 31 == 1:
            node = m.arbitration_id >> 5
            if node in range(11, 17):
                states[node] = {'error': int.from_bytes(m.data[:4], 'little'),
                                'state': m.data[4], 'data_hex': bytes(m.data).hex()}
                counts[str(node)] += 1
        elif m.is_extended_id and m.arbitration_id >> 8 == 41:
            node = m.arbitration_id & 255
            if node in range(1, 5):
                counts['ak' + str(node)] += 1
    return {'counts': dict(counts), 'states': states}

with motor_session('mks_patch2_manual_origin_idle_check'):
    board = odrive.find_any(serial_number=SERIAL, timeout=15)
    check_idle(board)
    assert board.can.reliability_patch == 2
    expected = json.loads((OUT / (SERIAL + '-after-original-reboot-configuration.json')).read_text())
    assert get_config(board) == expected
    bus = can.Bus(interface='socketcan', channel='can0')
    try:
        record['before'] = snapshot(board)
        previous = board.can.recovery_count
        board.can.set_baud_rate(500000)
        record['manual_recovery_can'] = passive(bus, 2)
        record['latched'] = snapshot(board)
        assert board.can.recovery_count > previous
        for a in record['latched']:
            assert a['state'] == 1 and a['error'] == 0x4000
            assert a['latched'] and not a['in_progress'] and not a['eligible']
            assert a['input_vel'] == a['input_torque'] == 0 and abs(a['velocity']) < .01
            assert a['motor_error'] == a['encoder_error'] == a['controller_error'] == 0
        for node in (13, 14):
            raw = bytes.fromhex(record['manual_recovery_can']['states'][node]['data_hex'])
            assert raw[5] == 0xA2 and raw[6] == 1 and raw[4] == 1
        for axis in (board.axis0, board.axis1):
            axis.clear_errors()
        record['final_can'] = passive(bus, 5)
        record['after'] = snapshot(board)
        check_idle(board)
        assert all(not a['eligible'] and not a['latched'] for a in record['after'])
        assert len(record['final_can']['counts']) == 10
        assert min(record['final_can']['counts'].values()) >= 150
        assert len(record['final_can']['states']) == 6
        assert all(s['state'] == 1 and s['error'] == 0 for s in record['final_can']['states'].values())
        assert get_config(board) == expected
        record['configuration_unchanged'] = True
        record['result'] = 'PASS'
    except Exception as error:
        record['error'] = repr(error)
        raise
    finally:
        bus.shutdown()
        (OUT / 'manual-origin-idle-result.json').write_text(json.dumps(record, indent=2) + '\n')
        print(json.dumps(record, indent=2), flush=True)
