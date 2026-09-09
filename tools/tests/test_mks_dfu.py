"""Exercise flash range, verification and rollback against independent flash memory."""
import struct

import pytest

from tools.mks_dfu import replace_application

BASE = 0x08000000
APP = 0xC0000
SIZE = 0x100000
SECTORS = [(i * 0x4000, 0x4000) for i in range(4)] + [(0x10000, 0x10000)] + [
    (i, 0x20000) for i in range(0x20000, SIZE, 0x20000)]


def firmware(fill):
    return struct.pack('<II', 0x20020000, BASE + 0x101) + bytes([fill]) * 8192


class Memory:
    def __init__(self):
        self.initial = firmware(0x11).ljust(APP, b'\xff') + bytes([0x5A]) * (SIZE - APP)
        self.data = bytearray(self.initial)
        self.writes = []
        self.erases = []
        self.fail_write_once = False
        self.corrupt_verification_once = False

    def read(self, address, length):
        data = bytes(self.data[address - BASE:address - BASE + length])
        if self.corrupt_verification_once and self.writes and length == APP:
            self.corrupt_verification_once = False
            return data[:99] + bytes([data[99] ^ 1]) + data[100:]
        return data

    def erase(self, address):
        offset, size = next(s for s in SECTORS if BASE + s[0] == address)
        self.erases.append(address)
        self.data[offset:offset + size] = b'\xff' * size

    def write(self, address, data):
        self.writes.append((address, len(data)))
        if self.fail_write_once:
            self.fail_write_once = False
            raise OSError('injected USB transfer failure')
        offset = address - BASE
        # Real flash programming can only clear bits after sector erase.
        for i, value in enumerate(data):
            self.data[offset + i] &= value


def test_only_application_changes_and_all_configuration_bytes_survive():
    m = Memory()
    replace_application(m, firmware(0x22), m.initial)
    assert bytes(m.data[:APP]) == firmware(0x22).ljust(APP, b'\xff')
    assert bytes(m.data[APP:]) == bytes([0x5A]) * (SIZE - APP)
    assert all(address < BASE + APP for address in m.erases)
    assert all(BASE <= address and address + length <= BASE + APP for address, length in m.writes)


@pytest.mark.parametrize('image', [b'', firmware(0x22).ljust(APP + 1, b'\x00'),
                                  b'\x00' * 8192, struct.pack('<II', 0x20020000, BASE + APP + 1) + b'x' * 1024],
                         ids=['empty', 'oversize', 'invalid-vector', 'reset-outside-app'])
def test_bad_image_is_rejected_before_any_erase(image):
    m = Memory()
    with pytest.raises(ValueError):
        replace_application(m, image, m.initial)
    assert not m.erases and not m.writes and bytes(m.data) == m.initial


def test_wrong_original_backup_is_rejected_before_any_erase():
    m = Memory()
    wrong = m.initial[:100] + b'wrong' + m.initial[105:]
    with pytest.raises(ValueError):
        replace_application(m, firmware(0x22), wrong)
    assert not m.erases and not m.writes


@pytest.mark.parametrize('failure', ['fail_write_once', 'corrupt_verification_once'])
def test_failed_write_or_verify_restores_the_original_before_reporting_failure(failure):
    m = Memory()
    setattr(m, failure, True)
    with pytest.raises(RuntimeError, match='original application restored'):
        replace_application(m, firmware(0x22), m.initial)
    assert m.writes, 'test must reach actual programming before injected failure'
    assert bytes(m.data) == m.initial


class Clock:
    def __init__(self):
        self.now = 0.0
        self.sleeps = []

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.now += seconds


class Rom:
    """DFUse boundary: actual host transport runs; ROM address/state are independent."""
    def __init__(self, clock):
        self.clock = clock
        self.state = 2
        self.pointer = BASE
        self.poll_ms = 500
        self.ready_at = 0.0
        self.calls = []
        self.memory = bytes(range(256)) * (SIZE // 256)
        self.short_out = False
        self.short_upload = False
        self.jump_error = False

    def ctrl_transfer(self, direction, request, value, index, data, timeout):
        self.calls.append((self.clock.now, direction, request, value, data, timeout))
        assert index == 0 and 0 < timeout <= 2000
        if direction == 0x21:
            if self.short_out and request == 1 and data and (self.short_out != 'write' or value == 2):
                return len(data) - 1
            if request == 6:
                assert self.state in {2, 3, 5, 9}, 'ABORT during BUSY is invalid'
                self.state = 2
            elif request == 4:
                assert self.state == 10
                self.state = 2
            elif request == 1:
                assert self.state in {2, 5}
                if not data:
                    self.state = 6
                else:
                    if value == 0 and data[0] == 0x21:
                        self.pointer = struct.unpack_from('<I', data, 1)[0]
                    self.state = 3
            return len(data)
        if request == 3:
            delay = 0
            if self.state == 3:
                self.state = 4
                delay = self.poll_ms
                self.ready_at = self.clock.now + delay / 1000
            elif self.state == 4:
                assert self.clock.now + 1e-9 >= self.ready_at, 'GETSTATUS before bwPollTimeout'
                self.state = 5
            elif self.state == 6:
                self.state = 10 if self.jump_error else 7
            return bytes([3 if self.state == 10 else 0]) + delay.to_bytes(3, 'little') + bytes([self.state, 0])
        assert request == 2 and self.state in {2, 9}
        self.state = 9
        start = self.pointer - BASE + (value - 2) * data
        result = self.memory[start:start + data]
        return result[:-1] if self.short_upload else result


@pytest.fixture
def rom_transport(monkeypatch):
    from tools import mks_dfu
    clock = Clock()
    monkeypatch.setattr(mks_dfu, 'time', clock)
    rom = Rom(clock)
    return mks_dfu.DfuTransport(rom), rom, clock


def test_transport_observes_full_rom_poll_timeout(rom_transport):
    transport, rom, clock = rom_transport
    transport.erase(BASE)
    assert 0.5 in clock.sleeps
    assert rom.state == 2


def test_transport_timeout_does_not_poll_early_or_exceed_deadline(rom_transport):
    transport, rom, clock = rom_transport
    rom.poll_ms = 6000
    with pytest.raises(TimeoutError):
        transport.erase(BASE)
    assert clock.now <= 5.0
    assert sum(call[2] == 3 and call[0] > 0 for call in rom.calls) == 0


@pytest.mark.parametrize('length', [1, 2048, 2049, 2050, 4099])
def test_transport_partial_final_upload_has_correct_absolute_address(rom_transport, length):
    transport, rom, _ = rom_transport
    rom.poll_ms = 0
    assert transport.read(BASE, length) == rom.memory[:length]


def test_transport_jump_propagates_device_error(rom_transport):
    transport, rom, _ = rom_transport
    rom.poll_ms = 0
    rom.jump_error = True
    with pytest.raises(OSError, match='DFU status error'):
        transport.jump_to_application()


def test_transport_jump_requires_manifest_state(rom_transport):
    transport, rom, _ = rom_transport
    rom.poll_ms = 0
    transport.jump_to_application()
    assert rom.state == 7


@pytest.mark.parametrize('operation', ['address', 'erase', 'write'])
def test_transport_rejects_short_out_transfer(rom_transport, operation):
    transport, rom, _ = rom_transport
    rom.poll_ms = 0
    rom.short_out = operation
    with pytest.raises(OSError, match='short DFU download'):
        if operation == 'address':
            transport.address(BASE)
        elif operation == 'erase':
            transport.erase(BASE)
        else:
            transport.write(BASE, b'\x22' * 4)


def test_transport_read_rejects_short_in_transfer(rom_transport):
    transport, rom, _ = rom_transport
    rom.poll_ms = 0
    rom.short_upload = True
    with pytest.raises(OSError, match='short DFU upload'):
        transport.read(BASE, 2048)


def test_persistent_update_and_restore_failure_never_jump():
    m = Memory()
    jumps = []
    m.jump_to_application = lambda: jumps.append(True)
    def fail(*_args):
        raise OSError('persistent transfer failure')
    m.write = fail
    with pytest.raises(RuntimeError, match='ORIGINAL RESTORE FAILED'):
        replace_application(m, firmware(0x22), m.initial)
    assert not jumps
    assert bytes(m.data[APP:]) == m.initial[APP:]


@pytest.mark.parametrize('operation', ['erase-nvm', 'write-nvm', 'write-straddle', 'read-after-flash'])
def test_transport_rejects_out_of_range_before_usb(rom_transport, operation):
    transport, rom, _ = rom_transport
    with pytest.raises(ValueError):
        if operation == 'erase-nvm':
            transport.erase(BASE + APP)
        elif operation == 'write-nvm':
            transport.write(BASE + APP, b'xx')
        elif operation == 'write-straddle':
            transport.write(BASE + APP - 4, b'x' * 8)
        else:
            transport.read(BASE + SIZE - 1, 2)
    assert not rom.calls


def test_abort_after_timeout_honors_outstanding_busy_poll(rom_transport):
    transport, rom, clock = rom_transport
    rom.poll_ms = 6000
    with pytest.raises(TimeoutError):
        transport.erase(BASE)
    transport.abort()
    assert clock.now >= 6.0 and rom.state == 2


def test_abort_waits_for_busy_rom_before_sending_abort(rom_transport):
    transport, rom, _ = rom_transport
    rom.state = 3
    transport.abort()
    assert rom.state == 2


def test_transport_one_byte_flash_end_read_stays_inside_flash(rom_transport):
    transport, rom, _ = rom_transport
    rom.poll_ms = 0
    assert transport.read(BASE + SIZE - 1, 1) == rom.memory[-1:]
    assert rom.pointer == BASE + SIZE - 2


def test_transport_final_write_is_padded_only_to_next_application_word(rom_transport):
    transport, rom, _ = rom_transport
    rom.poll_ms = 0
    transport.write(BASE + APP - 4, b'x')
    payloads = [call[4] for call in rom.calls if call[1:4] == (0x21, 1, 2)]
    assert payloads == [b'x\xff\xff\xff']


def test_transport_abort_clears_rom_error_before_aborting(rom_transport):
    transport, rom, _ = rom_transport
    rom.state = 10
    transport.abort()
    assert rom.state == 2
    commands = [call[2] for call in rom.calls if call[1] == 0x21]
    assert commands == [4, 6]


def test_transport_wrong_manifest_state_is_not_success(rom_transport):
    transport, rom, _ = rom_transport
    rom.poll_ms = 0
    transport.address = lambda _: None
    original = rom.ctrl_transfer
    def control(*args, **kwargs):
        result = original(*args, **kwargs)
        if args[1] == 3:
            return bytes([0, 0, 0, 0, 2, 0])
        return result
    rom.ctrl_transfer = control
    with pytest.raises(OSError, match='manifestation did not start'):
        transport.jump_to_application()
