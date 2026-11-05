"""patch4 one-board update for 336A33523235 (node 13/14); run only under the documented gate.

Adapted from docs/reports/2026-09-10-can-auto-resume-evidence/board-336A33523235-patch3/
{backup_current,flash_board}.py. Differences (2026-10-21), all on the pre-flash check only:
  * the running patch3 board loops CAN recovery while the bus is disconnected, so
    can_recovery_in_progress may be true at any instant and is not required false;
  * axis.error may carry DC_BUS_UNDER_VOLTAGE (0x2) beside the recovery latch (0x4000).
Safety conditions are unchanged: IDLE, zero input, |vel|<0.01, motor/encoder/controller
errors 0, calibration kept. Never arms, calibrates, saves configuration or erases NVM.

  backup: two full 1 MiB ROM reads, compared, saved; returns to the application.
  flash:  application-only guarded DFU of the patch4 image with rollback to the backup.
"""
import datetime
import hashlib
import json
import pathlib
import sys
import time

import odrive
import odrive.configuration
import usb.core
import usb.util

from chassis.usb_session import motor_session, read_path
from tools.mks_dfu import APP_SIZE, BASE, FLASH_SIZE, DfuTransport, replace_application, validate_image

SERIAL = '336A33523235'
NODES = [13, 14]
IMAGE_SHA = '15462f7f64745b03d1347fccda8091cd34b795056fefee64f87e794dc8a4b3fc'
OUT = pathlib.Path('/evidence')
ORIGINAL = OUT / (SERIAL + '-patch3-original-1m.bin')
ALLOWED_AXIS_ERRORS = {0, 0x4000, 0x4002}
# Running (RAM) vs saved configuration may differ only here, as recorded on 2026-09-10.
ALLOWED_CONFIG_DIFF = {'axis0.controller.config.input_mode', 'axis1.controller.config.input_mode'}
record = {'serial': SERIAL, 'started_at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
          'operations': [], 'candidate_sha256': IMAGE_SHA}


def save(name):
    (OUT / name).write_text(json.dumps(record, indent=2, default=str) + '\n')


def digest(data):
    return hashlib.sha256(data).hexdigest()


def get_config(board):
    return odrive.configuration.get_dict(board, board, False)


def differences(a, b, path=''):
    if isinstance(a, dict) and isinstance(b, dict):
        assert a.keys() == b.keys(), path
        return [item for k in a for item in differences(a[k], b[k], path + '.' + k)]
    return [] if a == b else [(path.lstrip('.'), a, b)]


def check_stopped(board, patch):
    """Identity and a stopped, calibrated board; recovery latch/under-voltage tolerated."""
    assert format(board.serial_number, 'X') == SERIAL
    assert (board.hw_version_major, board.hw_version_minor, board.hw_version_variant) == (3, 6, 56)
    assert board.can.reliability_patch == patch, board.can.reliability_patch
    axes = [board.axis0, board.axis1]
    assert [a.config.can_node_id for a in axes] == NODES
    states = []
    for axis in axes:
        assert axis.current_state == 1, axis.current_state
        assert axis.controller.input_vel == 0 and axis.controller.input_torque == 0
        assert abs(axis.encoder.vel_estimate) < 0.01
        assert axis.motor.error == axis.encoder.error == axis.controller.error == 0
        assert axis.error in ALLOWED_AXIS_ERRORS, hex(axis.error)
        assert axis.motor.is_calibrated and axis.encoder.is_ready
        assert axis.motor.config.pre_calibrated and axis.encoder.config.pre_calibrated
        states.append({p: read_path(axis, p) for p in (
            'current_state', 'requested_state', 'error', 'encoder.pos_estimate',
            'encoder.vel_estimate', 'can_recovery_latched', 'can_recovery_in_progress')})
    return states


def check_patch4_clean(board):
    """Strict post-flash state: no relaxation, no latch, no recovery in progress."""
    states = check_stopped(board, 4)
    for axis in (board.axis0, board.axis1):
        assert axis.error == 0, hex(axis.error)
        assert axis.requested_state in (0, 1), axis.requested_state
        assert not axis.can_recovery_latched and not axis.can_recovery_in_progress
        assert not axis.can_recovery_auto_resume_eligible
    assert not board.can.reinitializing and not board.can.reinit_requested
    return states


def can_counters(board):
    names = ['reliability_patch', 'hal_error', 'last_hal_error', 'hal_error_history',
             'recovery_count', 'recovery_failure_count', 'tx_drop_count', 'tx_error_count',
             'lec_error_count', 'rx_overrun_count', 'bus_off_count', 'error_passive_count',
             'error_warning_count']
    return {n: read_path(board.can, n) for n in names}


def enter_dfu(board):
    assert not list(usb.core.find(find_all=True, idVendor=0x0483, idProduct=0xDF11))
    try:
        board.enter_dfu_mode()
    except Exception as error:
        record.setdefault('enter_disconnects', []).append(type(error).__name__)
    return attach_dfu(15)


def attach_dfu(timeout_s):
    """The ROM DFU device whose USB serial is this board's (STM32 unique ID)."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        devices = list(usb.core.find(find_all=True, idVendor=0x0483, idProduct=0xDF11))
        matches = [d for d in devices if usb.util.get_string(d, d.iSerialNumber).upper() == SERIAL]
        if len(matches) == 1:
            dev = matches[0]
            dev.set_configuration()
            interface = dev.get_active_configuration()[(0, 0)]
            assert usb.util.get_string(dev, interface.iInterface).strip() == (
                '@Internal Flash  /0x08000000/04*016Kg,01*064Kg,07*128Kg')
            interface.set_altsetting()
            return DfuTransport(dev, record['operations'].append)
        time.sleep(0.2)
    raise RuntimeError('matching STM32F405 ROM DFU did not enumerate')


def return_to_app(transport):
    try:
        transport.jump_to_application()
    except usb.core.USBError as error:
        record.setdefault('jump_disconnects', []).append(str(error))
    return odrive.find_any(serial_number=SERIAL, timeout=25)


def backup():
    with motor_session('mks_patch4_backup_336A33523235'):
        board = odrive.find_any(serial_number=SERIAL, timeout=15)
        record['before_axes'] = check_stopped(board, 3)
        record['can_before'] = can_counters(board)
        before = get_config(board)
        assert before == get_config(board), 'configuration changed between reads'
        for name in ('axis0', 'axis1'):
            assert not any(v for k, v in before[name]['config'].items() if k.startswith('startup_'))
        (OUT / (SERIAL + '-patch3-running-configuration.json')).write_text(
            json.dumps(before, indent=2, sort_keys=True) + '\n')
        transport = enter_dfu(board)
        try:
            one = transport.read(BASE, FLASH_SIZE)
            two = transport.read(BASE, FLASH_SIZE)
            assert one == two, 'two full ROM reads differ'
            validate_image(one[:APP_SIZE])
            with ORIGINAL.open('xb') as stream:
                stream.write(one)
            record.update(bytes=len(one), reads_equal=True, sha256=digest(one),
                          application_sha256=digest(one[:APP_SIZE]), nvm_sha256=digest(one[APP_SIZE:]))
        finally:
            save('backup-result.json')
            board = return_to_app(transport)
            record['after_axes'] = check_stopped(board, 3)
            after = get_config(board)
            record['config_diff'] = differences(before, after)
            assert all(path in ALLOWED_CONFIG_DIFF for path, _, _ in record['config_diff']), record['config_diff']
            (OUT / (SERIAL + '-patch3-after-reboot-configuration.json')).write_text(
                json.dumps(after, indent=2, sort_keys=True) + '\n')
            save('backup-result.json')


def backup_from_dfu():
    """Board already in ROM DFU (an earlier enter_dfu outlived its caller): same backup."""
    before = json.loads((OUT / (SERIAL + '-patch3-running-configuration.json')).read_text())
    with motor_session('mks_patch4_backup_dfu_336A33523235'):
        transport = attach_dfu(5)
        board = None
        try:
            one = transport.read(BASE, FLASH_SIZE)
            two = transport.read(BASE, FLASH_SIZE)
            assert one == two, 'two full ROM reads differ'
            validate_image(one[:APP_SIZE])
            with ORIGINAL.open('xb') as stream:
                stream.write(one)
            record.update(bytes=len(one), reads_equal=True, sha256=digest(one),
                          application_sha256=digest(one[:APP_SIZE]), nvm_sha256=digest(one[APP_SIZE:]))
        finally:
            save('backup-result.json')
            board = return_to_app(transport)
            record['after_axes'] = check_stopped(board, 3)
            after = get_config(board)
            record['config_diff'] = differences(before, after)
            assert all(path in ALLOWED_CONFIG_DIFF for path, _, _ in record['config_diff']), record['config_diff']
            (OUT / (SERIAL + '-patch3-after-reboot-configuration.json')).write_text(
                json.dumps(after, indent=2, sort_keys=True) + '\n')
            save('backup-result.json')
    print(json.dumps({k: v for k, v in record.items() if k != 'operations'}, indent=2, default=str))


def load_inputs():
    original = ORIGINAL.read_bytes()
    image = (OUT / 'ODriveFirmware.bin').read_bytes()
    expected = json.loads((OUT / (SERIAL + '-patch3-after-reboot-configuration.json')).read_text()
                          if (OUT / (SERIAL + '-patch3-after-reboot-configuration.json')).exists()
                          else (OUT / (SERIAL + '-patch3-running-configuration.json')).read_text())
    assert digest(image) == IMAGE_SHA
    assert len(original) == FLASH_SIZE
    validate_image(image)
    record['original_sha256'] = digest(original)
    return original, image, expected


# Each step is its own process: re-discovering the app with fibre in the same process
# that used pyusb ROM DFU hung on 2026-10-21 (backup step), while a fresh process found it.
def program():
    """Check the stopped patch3 app, write+verify the application in ROM DFU, jump, exit."""
    original, image, expected = load_inputs()
    with motor_session('mks_patch4_program_336A33523235'):
        board = odrive.find_any(serial_number=SERIAL, timeout=15)
        record['before_axes'] = check_stopped(board, 3)
        record['before_config_diff'] = differences(expected, get_config(board))
        assert all(p in ALLOWED_CONFIG_DIFF for p, _, _ in record['before_config_diff'])
        transport = enter_dfu(board)
        save('program-result.json')
        try:
            replace_application(transport, image, original)
        except Exception as error:
            record['program_error'] = repr(error)
            if transport.read(BASE, FLASH_SIZE) == original:
                record['original_intact'] = True
                transport.jump_to_application()
            save('program-result.json')
            raise
        record['program_verified'] = True  # application and NVM read back inside replace_application
        save('program-result.json')
        try:
            transport.jump_to_application()
        except usb.core.USBError as error:
            record.setdefault('jump_disconnects', []).append(str(error))
        record['finished_at'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        save('program-result.json')
    print(json.dumps({k: v for k, v in record.items() if k != 'operations'}, indent=2, default=str))


def program_from_dfu():
    """Fresh process: the program step put the checked, stopped board into ROM DFU but could
    not see the re-enumerated device (as backup attempt 1, 2026-10-21). Attach that device
    and write; replace_application still refuses unless the full flash equals the backup."""
    original, image, _ = load_inputs()
    earlier = json.loads((OUT / 'program-result.json').read_text())
    assert earlier['before_config_diff'] == [] or all(
        p in ALLOWED_CONFIG_DIFF for p, _, _ in earlier['before_config_diff'])
    assert all(a['current_state'] == 1 and a['encoder.vel_estimate'] == 0.0
               for a in earlier['before_axes'])
    assert 'program_verified' not in earlier and 'ROM DFU did not enumerate' in earlier['error']
    record['earlier_program'] = {k: earlier[k] for k in ('started_at', 'before_axes', 'error')}
    with motor_session('mks_patch4_program_dfu_336A33523235'):
        transport = attach_dfu(5)
        save('program-dfu-result.json')
        try:
            replace_application(transport, image, original)
        except Exception as error:
            record['program_error'] = repr(error)
            if transport.read(BASE, FLASH_SIZE) == original:
                record['original_intact'] = True
                transport.jump_to_application()
            save('program-dfu-result.json')
            raise
        record['program_verified'] = True
        save('program-dfu-result.json')
        try:
            transport.jump_to_application()
        except usb.core.USBError as error:
            record.setdefault('jump_disconnects', []).append(str(error))
        record['finished_at'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        save('program-dfu-result.json')
    print(json.dumps({k: v for k, v in record.items() if k != 'operations'}, indent=2, default=str))


def verify():
    """Fresh process: strict patch4 acceptance; a failure leads to the rollback step."""
    _, _, expected = load_inputs()
    with motor_session('mks_patch4_verify_336A33523235'):
        board = odrive.find_any(serial_number=SERIAL, timeout=30)
        record['after_axes'] = check_patch4_clean(board)
        assert (board.fw_version_major, board.fw_version_minor, board.fw_version_revision) == (0, 5, 1)
        assert board.fw_version_unreleased == 1
        record['after_config_diff'] = differences(expected, get_config(board))
        assert all(p in ALLOWED_CONFIG_DIFF for p, _, _ in record['after_config_diff'])
        record['can_after_boot'] = can_counters(board)
        time.sleep(3)  # bus is disconnected: patch4 must count LEC, not latch or reinit
        record['can_after_3s'] = can_counters(board)
        record['axes_after_3s'] = check_patch4_clean(board)
        record['deadline_counts'] = [a.control_deadline_count for a in (board.axis0, board.axis1)]
        assert record['can_after_3s']['recovery_count'] == record['can_after_boot']['recovery_count']
        assert record['can_after_3s']['recovery_failure_count'] == 0
        assert board.can.hal_error & ~0x7FF == 0
        record['accepted'] = True
        save('verify-result.json')
    print(json.dumps({k: v for k, v in record.items() if k != 'operations'}, indent=2, default=str))


def rollback():
    """Fresh process: restore the backed-up patch3 application (NVM untouched), jump, exit.

    Refuses unless both axes are IDLE and still. Operating rule (2026-10-21): if verify or
    this step refuses or fails, powertrain_chassis stays stopped, the robot is made safe on
    site, and the board is restored through the physical DFU switch; the failure is reported.
    """
    original, image, _ = load_inputs()
    with motor_session('mks_patch4_rollback_336A33523235'):
        board = odrive.find_any(serial_number=SERIAL, timeout=30)
        assert format(board.serial_number, 'X') == SERIAL
        for axis in (board.axis0, board.axis1):
            assert axis.current_state == 1 and abs(axis.encoder.vel_estimate) < 0.01
        transport = enter_dfu(board)
        current = image.ljust(APP_SIZE, b'\xff') + original[APP_SIZE:]
        replace_application(transport, original[:APP_SIZE], current)
        assert transport.read(BASE, FLASH_SIZE) == original
        record['original_restored'] = True
        save('rollback-result.json')
        try:
            transport.jump_to_application()
        except usb.core.USBError as error:
            record.setdefault('jump_disconnects', []).append(str(error))
        save('rollback-result.json')


if __name__ == '__main__':
    step = sys.argv[1]
    try:
        {'backup': backup, 'backup-from-dfu': backup_from_dfu, 'program': program, 'program-from-dfu': program_from_dfu,
         'verify': verify, 'rollback': rollback}[step]()
    except Exception as error:
        record['error'] = repr(error)
        save(step.replace('-from-dfu', '') + '-result.json')
        raise
