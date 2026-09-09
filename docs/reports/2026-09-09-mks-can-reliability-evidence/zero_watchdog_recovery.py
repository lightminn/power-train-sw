"""Loaded-ground zero-setpoint validation, serial 336A33523235 only.

No nonzero setpoints or NVM writes. The user has confirmed a safe area and power
cut readiness. Encoder displacement guard and 300 ms device watchdog remain active.
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
          'samples': [], 'checks': {}, 'events': []}
end = threading.Event()
control = threading.Event()
abort = threading.Event()
faults = []
heartbeats = {}
positions = {}
initial_positions = {}
max_travel = {13: 0.0, 14: 0.0}
phase = ['setup']
last_control = [None]
test_deadline = [0.0]


def save():
    (OUT / 'zero-watchdog-recovery-result.json').write_text(json.dumps(record, indent=2) + '\n')


with motor_session('mks_zero_watchdog_recovery'):
    board = odrive.find_any(serial_number=SERIAL, timeout=15)
    assert board.can.reliability_patch == 1
    axes = (board.axis0, board.axis1)
    assert [a.config.can_node_id for a in axes] == [13, 14]
    expected = json.loads((OUT / (SERIAL + '-configuration.json')).read_text())
    assert odrive.configuration.get_dict(board, board, False) == expected
    for n, axis in zip((13, 14), axes):
        assert axis.current_state == 1 and axis.error == axis.motor.error == axis.encoder.error == 0
        initial_positions[n] = axis.encoder.pos_estimate
        assert math.isfinite(initial_positions[n])
    tx = can.Bus(interface='socketcan', channel='can0')
    observer = can.Bus(interface='socketcan', channel='can0')

    def send(node, command, payload=b'', rtr=False):
        tx.send(can.Message(arbitration_id=(node << 5) | command, is_extended_id=False,
                            is_remote_frame=rtr, data=payload), timeout=.02)

    def idle():
        errors = []
        for node in (13, 14):
            try:
                # No setpoints after abort: do not extend the local watchdog
                # if a subsequent IDLE request is lost or rejected.
                send(node, 7, struct.pack('<II', 1, 0))
            except Exception as error:
                errors.append(repr(error))
        if errors:
            raise RuntimeError(errors)

    def receive():
        try:
            with gzip.open(OUT / 'zero-watchdog-recovery-frames.jsonl.gz', 'wt') as stream:
                while not end.is_set():
                    message = observer.recv(.02)
                    if message is None:
                        continue
                    now = time.monotonic()
                    stream.write(json.dumps({'t': now, 'phase': phase[0], 'id': message.arbitration_id,
                        'rx': message.is_rx, 'rtr': message.is_remote_frame, 'ext': message.is_extended_id,
                        'error': message.is_error_frame, 'data': bytes(message.data).hex()}) + '\n')
                    if message.is_error_frame:
                        raise RuntimeError('CAN error frame')
                    node, command = message.arbitration_id >> 5, message.arbitration_id & 31
                    if message.is_extended_id or message.is_remote_frame or not message.is_rx or node not in (13, 14):
                        continue
                    if command == 1 and len(message.data) >= 5:
                        heartbeats[node] = (now, int.from_bytes(message.data[:4], 'little'), message.data[4])
                    elif command == 9 and len(message.data) == 8:
                        position, velocity = struct.unpack('<ff', message.data)
                        if not math.isfinite(position) or not math.isfinite(velocity):
                            raise RuntimeError('nonfinite encoder feedback')
                        positions[node] = (now, position, velocity)
                        max_travel[node] = max(max_travel[node], abs(position - initial_positions[node]))
                        if max_travel[node] > .50:
                            raise RuntimeError('zero-command displacement exceeded half motor revolution')
        except Exception as error:
            faults.append(repr(error))
            abort.set()

    def write():
        try:
            tick = 0
            while not end.is_set():
                started = time.monotonic()
                if started >= test_deadline[0] and not abort.is_set():
                    faults.append('independent 30 second test deadline expired')
                    abort.set()
                if abort.is_set():
                    idle()
                elif control.is_set():
                    for node in (13, 14):
                        send(node, 13, struct.pack('<ff', 0., 0.))
                    last_control[0] = time.monotonic()
                for node in (13, 14):
                    if tick % 3 == node - 13:
                        send(node, 9, rtr=True)
                    if tick % 60 == (node - 13) * 30:
                        send(node, 20, rtr=True)
                tick += 1
                end.wait(max(0, .02 - (time.monotonic() - started)))
        except Exception as error:
            faults.append(repr(error))
            abort.set()
            try:
                idle()
            except Exception:
                pass  # Device watchdog still expires without valid controls.

    def check():
        if faults:
            raise RuntimeError(faults)
        now = time.monotonic()
        if any(now - heartbeats.get(n, (0,))[0] > .4 for n in (13, 14)):
            raise RuntimeError('missing board heartbeat')
        if any(now - positions.get(n, (0,))[0] > .2 for n in (13, 14)):
            raise RuntimeError('stale board encoder')

    def wait_until(predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            check()
            if predicate():
                record['events'].append({'t': time.monotonic(), 'event': label})
                return
            time.sleep(.01)
        raise AssertionError(label + ' timeout: ' + repr(heartbeats))

    def hold(duration):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            check()
            time.sleep(.02)

    def states(state, error):
        return all(heartbeats.get(n, (0, -1, -1))[1:] == (error, state) for n in (13, 14))

    def snapshot(label):
        values = [{'state': a.current_state, 'error': a.error, 'motor_error': a.motor.error,
                   'encoder_error': a.encoder.error, 'position': a.encoder.pos_estimate,
                   'velocity': a.encoder.vel_estimate, 'iq': a.motor.current_control.Iq_measured,
                   'latched': a.can_recovery_latched} for a in axes]
        record['samples'].append({'phase': label, 't': time.monotonic(), 'axes': values})
        return values

    threads = [threading.Thread(target=receive), threading.Thread(target=write)]
    try:
        for a in axes:
            a.controller.input_vel = 0
            a.controller.input_torque = 0
            a.controller.config.control_mode = 2
            a.controller.config.input_mode = 1
            a.config.watchdog_timeout = .3
            a.config.enable_watchdog = True
        test_deadline[0] = time.monotonic() + 30
        for thread in threads:
            thread.start()
        time.sleep(.2)
        phase[0] = 'idle_watchdog'
        hold(1)
        assert states(1, 0)
        record['checks']['idle_does_not_timeout'] = True

        def arm_zero(label):
            phase[0] = label
            control.set()
            hold(.1)
            for n in (13, 14):
                send(n, 7, struct.pack('<II', 8, 0))
            wait_until(lambda: states(8, 0), .5, label + '_closed_loop')
            hold(1)
            assert states(8, 0)
            snapshot(label)

        arm_zero('zero_valid_controls')
        phase[0] = 'queries_only_watchdog'
        control.clear()
        wait_until(lambda: states(1, 0x800), .6, 'watchdog_expired_to_idle')
        record['watchdog_last_control_t'] = last_control[0]
        record['watchdog_idle_heartbeat_t'] = {n: heartbeats[n][0] for n in (13, 14)}
        hold(.5)
        assert states(1, 0x800)
        record['checks']['queries_do_not_feed_watchdog'] = True
        snapshot('watchdog_expired')
        for a in axes:
            a.clear_errors()
        hold(.1)
        assert states(1, 0)

        arm_zero('zero_before_can_reset')
        phase[0] = 'same_baud_recovery'
        previous = board.can.recovery_count
        board.can.set_baud_rate(500000)
        wait_until(lambda: states(1, 0x4000), .8, 'can_recovery_latched_idle')
        hold(.5)
        assert states(1, 0x4000)
        assert board.can.recovery_count == previous + 1
        assert not board.can.reinit_requested and not board.can.reinitializing
        assert board.can.hal_error == 0
        assert all(a.can_recovery_latched for a in axes)
        record['checks']['can_reset_latches_idle_despite_zero_controls'] = True
        snapshot('can_recovery_latched')
        control.clear()
        phase[0] = 'manual_clear_only'
        for a in axes:
            a.clear_errors()
        hold(.4)
        assert states(1, 0)
        record['checks']['manual_clear_does_not_arm'] = True
        arm_zero('manual_zero_rearm')
        record['checks']['explicit_zero_rearm_works'] = True
    except Exception as error:
        record['error'] = repr(error)
        raise
    finally:
        control.clear()
        abort.set()
        try:
            idle()
        except Exception as error:
            record['can_stop_error'] = repr(error)
        # USB stop attempts are independent of CAN and of the other axis.
        for n, a in zip((13, 14), axes):
            try:
                a.requested_state = 1
            except Exception as error:
                record.setdefault('usb_stop_errors', {})[n] = repr(error)
        try:
            time.sleep(.35)
            assert all(a.current_state == 1 for a in axes)
            for name, a in zip(('axis0', 'axis1'), axes):
                a.config.enable_watchdog = expected[name]['config']['enable_watchdog']
                a.config.watchdog_timeout = expected[name]['config']['watchdog_timeout']
                a.controller.config.control_mode = expected[name]['controller']['config']['control_mode']
                a.controller.config.input_mode = expected[name]['controller']['config']['input_mode']
                a.controller.input_vel = 0
                a.controller.input_torque = 0
                a.clear_errors()
            record['configuration_restored'] = odrive.configuration.get_dict(board, board, False) == expected
            record['final_axes'] = snapshot('final_idle')
        except Exception as error:
            record['cleanup_error'] = repr(error)
        end.set()
        for thread in threads:
            if thread.ident is not None:
                thread.join(3)
        tx.shutdown()
        observer.shutdown()
        record['faults'] = faults
        record['max_motor_displacement_turns'] = max_travel
        record['finished_at'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        save()
        print(json.dumps({k: v for k, v in record.items() if k != 'events'}, indent=2), flush=True)
    assert record.get('configuration_restored') and not record.get('cleanup_error') and not faults
