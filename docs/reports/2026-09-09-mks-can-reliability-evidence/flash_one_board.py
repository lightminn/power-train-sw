"""Task-specific one-board update; run only under the documented physical gate.

This intentionally pins serial, original image and reviewed candidate hashes.
It never arms, calibrates, saves configuration, erases NVM, or touches other boards.
"""
import datetime
import hashlib
import json
import pathlib
import time

import odrive
import odrive.configuration
import usb.core
import usb.util

from chassis.usb_session import motor_session
from tools.mks_dfu import APP_SIZE, BASE, FLASH_SIZE, DfuTransport, replace_application, validate_image

SERIAL = '336A33523235'
ORIGINAL_SHA = '794ed7b812282cffae5297adb683a29dff5fc4f9f5af3207ea6c5c18404ab9c7'
IMAGE_SHA = '7567809465b9646d39b1f9595f8a8e037ed758344f90269ba49038f8c6bfc710'
OUT = pathlib.Path('/evidence/mks-reliability')
record = {'serial': SERIAL, 'started_at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
          'operations': [], 'original_sha256': ORIGINAL_SHA, 'candidate_sha256': IMAGE_SHA}


def save():
    (OUT / 'flash-update-result.json').write_text(json.dumps(record, indent=2) + '\n')


def digest(data):
    return hashlib.sha256(data).hexdigest()


def get_config(board):
    return odrive.configuration.get_dict(board, board, False)


def check_idle(board):
    assert format(board.serial_number, 'X') == SERIAL
    assert (board.hw_version_major, board.hw_version_minor, board.hw_version_variant) == (3, 6, 56)
    axes = [board.axis0, board.axis1]
    assert [a.config.can_node_id for a in axes] == [13, 14]
    for axis in axes:
        assert axis.current_state == 1
        assert axis.controller.input_vel == 0 and axis.controller.input_torque == 0
        assert abs(axis.encoder.vel_estimate) < 0.01
        assert axis.error == axis.motor.error == axis.encoder.error == axis.controller.error == 0
        assert axis.motor.is_calibrated and axis.encoder.is_ready
        assert axis.motor.config.pre_calibrated and axis.encoder.config.pre_calibrated
    return [{'state': a.current_state, 'position': a.encoder.pos_estimate,
             'velocity': a.encoder.vel_estimate, 'error': a.error,
             'calibrated': a.motor.is_calibrated, 'encoder_ready': a.encoder.is_ready} for a in axes]


def enter_dfu(board):
    assert not list(usb.core.find(find_all=True, idVendor=0x0483, idProduct=0xDF11))
    try:
        board.enter_dfu_mode()
    except Exception as error:
        # Disconnection is expected; only a matching ROM device makes this successful.
        record.setdefault('enter_disconnects', []).append(type(error).__name__)
    deadline = time.monotonic() + 15
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


def run():
    original = (OUT / (SERIAL + '-original-1m.bin')).read_bytes()
    image = (OUT / 'ODriveFirmware.bin').read_bytes()
    expected_config = json.loads((OUT / (SERIAL + '-configuration.json')).read_text())
    assert digest(original) == ORIGINAL_SHA and digest(image) == IMAGE_SHA
    validate_image(image)
    with motor_session('mks_reviewed_single_board_flash'):
        board = odrive.find_any(serial_number=SERIAL, timeout=15)
        record['before_axes'] = check_idle(board)
        assert get_config(board) == expected_config
        for name in ('axis0', 'axis1'):
            assert not any(value for key, value in expected_config[name]['config'].items()
                           if key.startswith('startup_'))
        transport = enter_dfu(board)
        save()
        try:
            replace_application(transport, image, original)
        except Exception as error:
            record['program_error'] = repr(error)
            # Jump only after an independent full original readback proves rollback.
            if transport.read(BASE, FLASH_SIZE) == original:
                old = return_to_app(transport)
                record['original_restored'] = get_config(old) == expected_config
                record['restored_axes'] = check_idle(old)
            save()
            raise
        record['program_verified'] = True
        save()
        board = None
        try:
            board = return_to_app(transport)
            record['after_axes'] = check_idle(board)
            record['patch'] = board.can.reliability_patch
            assert record['patch'] == 1
            record['config_unchanged'] = get_config(board) == expected_config
            assert record['config_unchanged']
            assert (board.fw_version_major, board.fw_version_minor, board.fw_version_revision) == (0, 5, 1)
            assert board.fw_version_unreleased == 1
            assert board.can.hal_error == 0
        except Exception as error:
            # The running app failed acceptance; restore the original via ROM DFU.
            record['application_error'] = repr(error)
            if board is None:
                record['physical_rom_entry_required'] = True
                save()
                raise RuntimeError('verified image did not enumerate; physical ROM DFU entry '
                                   'is required to restore the preserved original image') from error
            transport = enter_dfu(board)
            current = image.ljust(APP_SIZE, b'\xff') + original[APP_SIZE:]
            replace_application(transport, original[:APP_SIZE], current)
            assert transport.read(BASE, FLASH_SIZE) == original
            old = return_to_app(transport)
            record['original_restored'] = get_config(old) == expected_config
            record['restored_axes'] = check_idle(old)
            save()
            raise
        record['finished_at'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        save()
        print(json.dumps({k: v for k, v in record.items() if k != 'operations'}, indent=2), flush=True)


if __name__ == '__main__':
    try:
        run()
    except Exception as error:
        record['error'] = repr(error)
        save()
        raise
