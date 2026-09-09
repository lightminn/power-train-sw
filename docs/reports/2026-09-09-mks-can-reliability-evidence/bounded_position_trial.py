"""One loaded-ground trial: node13 +0.25, node14 -0.25 motor revolutions.

User gate: safe ground area and independent power cut ready; motor shaft <1 turn.
Only this patched board is armed. No calibration, gain/current change or NVM save.
"""
import datetime
import gzip
import json
import math
import pathlib
import struct
import threading
import time

import can
import odrive
import odrive.configuration
from chassis.usb_session import motor_session

OUT = pathlib.Path('/evidence/mks-reliability')
SERIAL = '336A33523235'
record = {'serial': SERIAL, 'started_at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
          'requested_motor_turns': {13: .25, 14: -.25}, 'samples': [], 'errors': []}
end = threading.Event()
abort = threading.Event()
feed = threading.Event()
phase = ['initial']
heartbeat = {}
feedback = {}
targets = {}
initial = {}
previous = {}
travel = {13: 0., 14: 0.}
max_displacement = {13: 0., 14: 0.}
deadline = [0.]


def save():
    (OUT / 'bounded-position-result.json').write_text(json.dumps(record, indent=2) + '\n')


with motor_session('mks_bounded_ground_position_trial'):
    board = odrive.find_any(serial_number=SERIAL, timeout=15)
    assert board.can.reliability_patch == 1
    expected = json.loads((OUT / (SERIAL + '-configuration.json')).read_text())
    assert odrive.configuration.get_dict(board, board, False) == expected
    axes = (board.axis0, board.axis1)
    assert [a.config.can_node_id for a in axes] == [13, 14]
    for n, a in zip((13, 14), axes):
        assert a.current_state == 1 and a.error == a.motor.error == a.encoder.error == 0
        assert a.motor.config.current_lim == 9 and a.motor.is_calibrated and a.encoder.is_ready
        initial[n] = a.encoder.pos_estimate
        assert math.isfinite(initial[n])
        previous[n] = targets[n] = initial[n]
    record['initial_motor_positions'] = dict(initial)
    tx = can.Bus(interface='socketcan', channel='can0')
    rx = can.Bus(interface='socketcan', channel='can0')

    def send(n, cmd, data=b'', rtr=False):
        tx.send(can.Message(arbitration_id=(n << 5) | cmd, is_extended_id=False,
                            is_remote_frame=rtr, data=data), timeout=.02)

    def idle():
        errors = []
        for n in (13, 14):
            try:
                send(n, 7, struct.pack('<II', 1, 0))
            except Exception as error:
                errors.append(repr(error))
        if errors:
            raise RuntimeError(errors)

    def fail(error):
        message = str(error)
        if message not in record['errors']:
            record['errors'].append(message)
        abort.set()
        feed.clear()

    def receive():
        try:
            with gzip.open(OUT / 'bounded-position-frames.jsonl.gz', 'wt') as stream:
                while not end.is_set():
                    m = rx.recv(.01)
                    if m is None:
                        continue
                    now = time.monotonic()
                    stream.write(json.dumps({'t': now, 'phase': phase[0], 'id': m.arbitration_id,
                        'rx': m.is_rx, 'rtr': m.is_remote_frame, 'ext': m.is_extended_id,
                        'error': m.is_error_frame, 'data': bytes(m.data).hex()}) + '\n')
                    if m.is_error_frame:
                        raise RuntimeError('CAN error frame')
                    n, cmd = m.arbitration_id >> 5, m.arbitration_id & 31
                    if m.is_extended_id or m.is_remote_frame or not m.is_rx or n not in range(11, 17):
                        continue
                    if cmd == 1 and len(m.data) >= 5:
                        error, state = int.from_bytes(m.data[:4], 'little'), m.data[4]
                        heartbeat[n] = (now, error, state)
                        if error or (n not in (13, 14) and state != 1):
                            raise RuntimeError('unexpected motor state/error: ' + repr((n, error, state)))
                    elif n in (13, 14) and cmd == 9 and len(m.data) == 8:
                        position, velocity = struct.unpack('<ff', m.data)
                        if not math.isfinite(position) or not math.isfinite(velocity):
                            raise RuntimeError('nonfinite encoder feedback')
                        travel[n] += abs(position - previous[n])
                        previous[n] = position
                        max_displacement[n] = max(max_displacement[n], abs(position - initial[n]))
                        feedback[n] = (now, position, velocity)
                        record['samples'].append({'t': now, 'node': n, 'position': position, 'velocity': velocity})
                        if max_displacement[n] > .40 or travel[n] > .45 or abs(velocity) > 1.1:
                            raise RuntimeError('motor movement guard: ' + repr((n, max_displacement[n], travel[n], velocity)))
        except Exception as error:
            fail(error)

    def write():
        try:
            while not end.is_set():
                now = time.monotonic()
                if not abort.is_set() and now >= deadline[0]:
                    fail('independent eight second trial deadline')
                if not abort.is_set() and now > deadline[0] - 7.8:
                    if any(now - feedback.get(n, (0,))[0] > .060 for n in (13, 14)):
                        fail('sender observed encoder older than 60 ms')
                    if any(now - heartbeat.get(n, (0,))[0] > .060 for n in range(11, 17)):
                        fail('sender observed heartbeat older than 60 ms')
                    if phase[0] == 'quarter_motor_turn' and any(
                            heartbeat.get(n, (0, -1, -1))[1:] != (0, 8) for n in (13, 14)):
                        fail('sender observed loss of healthy closed-loop state')
                if abort.is_set():
                    idle()  # No watchdog-feeding control frame after abort.
                elif feed.is_set():
                    for n in (13, 14):
                        send(n, 12, struct.pack('<fhh', targets[n], 0, 0))
                for n in (13, 14):
                    send(n, 9, rtr=True)
                end.wait(max(0., .02 - (time.monotonic() - now)))
        except Exception as error:
            fail(error)
            try:
                idle()
            except Exception:
                pass

    def healthy():
        if abort.is_set():
            raise RuntimeError(record['errors'])
        now = time.monotonic()
        if any(now - heartbeat.get(n, (0,))[0] > .060 for n in range(11, 17)):
            raise RuntimeError('stale heartbeat')
        if any(now - feedback.get(n, (0,))[0] > .060 for n in (13, 14)):
            raise RuntimeError('stale encoder')

    threads = [threading.Thread(target=receive), threading.Thread(target=write)]
    try:
        for n, a in zip((13, 14), axes):
            a.controller.input_vel = 0
            a.controller.input_torque = 0
            a.controller.input_pos = initial[n]
            a.controller.config.control_mode = 3
            a.controller.config.input_mode = 5
            a.controller.config.vel_limit = 1.
            a.controller.config.enable_vel_limit = True
            a.controller.config.enable_overspeed_error = True
            a.controller.config.vel_limit_tolerance = 1.2
            a.trap_traj.config.vel_limit = 1.
            a.trap_traj.config.accel_limit = 2.
            a.trap_traj.config.decel_limit = 2.
            a.config.watchdog_timeout = .3
            a.config.enable_watchdog = True
            send(n, 13, struct.pack('<ff', 0., 0.))
        deadline[0] = time.monotonic() + 8
        for thread in threads:
            thread.start()
        feed.set()
        time.sleep(.12)
        healthy()
        phase[0] = 'arm_current_position'
        for n in (13, 14):
            send(n, 7, struct.pack('<II', 8, 0))
        arm_deadline = time.monotonic() + .5
        while not all(heartbeat.get(n, (0, -1, -1))[1:] == (0, 8) for n in (13, 14)):
            healthy()
            if time.monotonic() > arm_deadline:
                raise RuntimeError('closed-loop confirmation timeout')
            time.sleep(.01)
        phase[0] = 'quarter_motor_turn'
        targets.update({13: initial[13] + .25, 14: initial[14] - .25})
        record['target_motor_positions'] = dict(targets)
        finish = time.monotonic() + 4
        settled_since = None
        while time.monotonic() < finish:
            healthy()
            reached = all(abs(feedback[n][1] - targets[n]) < .06
                          and abs(feedback[n][2]) < .08 for n in (13, 14))
            settled_since = (settled_since or time.monotonic()) if reached else None
            if settled_since is not None and time.monotonic() - settled_since >= .15:
                record['target_reached'] = True
                break
            time.sleep(.01)
        if not record.get('target_reached'):
            raise AssertionError('bounded target not reached within four seconds')
    except Exception as error:
        fail(error)
    finally:
        phase[0] = 'stop'
        feed.clear()
        abort.set()
        try:
            idle()
        except Exception as error:
            record['can_stop_error'] = repr(error)
        for n, a in zip((13, 14), axes):
            try:
                a.requested_state = 1
            except Exception as error:
                record.setdefault('usb_stop_errors', {})[n] = repr(error)
        try:
            time.sleep(.4)
            assert all(a.current_state == 1 for a in axes)
            for name, a in zip(('axis0', 'axis1'), axes):
                a.config.enable_watchdog = expected[name]['config']['enable_watchdog']
                a.config.watchdog_timeout = expected[name]['config']['watchdog_timeout']
                for field in ('control_mode', 'input_mode', 'vel_limit', 'enable_vel_limit',
                              'enable_overspeed_error', 'vel_limit_tolerance'):
                    setattr(a.controller.config, field, expected[name]['controller']['config'][field])
                for field in ('vel_limit', 'accel_limit', 'decel_limit'):
                    setattr(a.trap_traj.config, field, expected[name]['trap_traj']['config'][field])
                a.controller.input_vel = 0
                a.controller.input_torque = 0
                a.controller.input_pos = a.encoder.pos_estimate
                a.clear_errors()
            record['configuration_restored'] = odrive.configuration.get_dict(board, board, False) == expected
            record['final_axes'] = [{'state': a.current_state, 'error': a.error,
                'position': a.encoder.pos_estimate, 'velocity': a.encoder.vel_estimate} for a in axes]
        except Exception as error:
            record['cleanup_error'] = repr(error)
        end.set()
        for thread in threads:
            if thread.ident is not None:
                thread.join(3)
        tx.shutdown()
        rx.shutdown()
        record['max_displacement_motor_turns'] = max_displacement
        record['cumulative_motor_travel_turns'] = travel
        record['finished_at'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        save()
        print(json.dumps({k: v for k, v in record.items() if k != 'samples'}, indent=2), flush=True)
    assert record.get('target_reached') and record.get('configuration_restored') and not record['errors'] and not record.get('cleanup_error')
    assert all(math.isfinite(value) and value <= .40 for value in max_displacement.values())
    assert all(math.isfinite(value) and value <= .45 for value in travel.values())
