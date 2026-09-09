"""Guarded STM32F405 1 MiB DFU update, preserving the last two NVM sectors.

The caller owns the motor maintenance lock, verifies board identity/IDLE/config,
enters ROM DFU and decides whether to return to the application after readback.
No mass erase, option-byte write, calibration or automatic arm is provided.
"""
import struct
import time

BASE = 0x08000000
APP_SIZE = 0xC0000
FLASH_SIZE = 0x100000
BLOCK_SIZE = 2048
APP_SECTORS = (0, 0x4000, 0x8000, 0xC000, 0x10000,
               0x20000, 0x40000, 0x60000, 0x80000, 0xA0000)


def validate_image(image):
    if not 8 <= len(image) <= APP_SIZE:
        raise ValueError('image size outside application region')
    sp, pc = struct.unpack_from('<II', image)
    if not (0x20000000 < sp <= 0x20020000 and sp % 8 == 0
            and pc & 1 and BASE + 8 <= (pc & ~1) < BASE + len(image)):
        raise ValueError('invalid STM32F405 application vector')


def replace_application(transport, image, original):
    """Program and verify application only; restore original application on error.

    Transport implements read(address, length), erase(sector_address), and
    write(address, bytes). A current full-flash match is required before erase.
    A rollback failure leaves ROM DFU available and must never trigger an app jump.
    """
    validate_image(image)
    if len(original) != FLASH_SIZE:
        raise ValueError('original backup must contain exactly 1 MiB')
    validate_image(original[:APP_SIZE])
    if transport.read(BASE, FLASH_SIZE) != original:
        raise ValueError('current flash differs from original backup; no erase performed')
    nvm = original[APP_SIZE:]

    def program(application):
        for offset in APP_SECTORS:
            transport.erase(BASE + offset)
        for offset in range(0, len(application), BLOCK_SIZE):
            chunk = application[offset:offset + BLOCK_SIZE]
            if chunk != b'\xff' * len(chunk):
                transport.write(BASE + offset, chunk)
        if transport.read(BASE, APP_SIZE) != application.ljust(APP_SIZE, b'\xff'):
            raise OSError('application readback differs')
        if transport.read(BASE + APP_SIZE, FLASH_SIZE - APP_SIZE) != nvm:
            raise OSError('configuration readback differs')

    try:
        program(image)
    except Exception as update_error:
        try:
            program(original[:APP_SIZE])
        except Exception as restore_error:
            raise RuntimeError(
                f'update failed ({update_error!r}); ORIGINAL RESTORE FAILED '
                f'({restore_error!r}); remain in ROM DFU') from restore_error
        raise RuntimeError(
            f'update failed ({update_error!r}); original application restored') from update_error


class DfuTransport:
    """STM32 ROM DFUse transport with bounded transfers and explicit flash ranges.

    Device must already be matched to the intended serial and Internal Flash
    alternate interface. Only interface 0, alternate 0 on STM32F405 is supported.
    The optional recorder receives metadata, never complete firmware contents.
    """

    def __init__(self, device, recorder=None):
        self.device = device
        self.recorder = recorder or (lambda event: None)
        self._poll_not_before = 0.0

    def control(self, direction, request, value, data, timeout=2000):
        self.recorder({'direction': direction, 'request': request, 'value': value,
                       'length': len(data) if isinstance(data, bytes) else data})
        result = self.device.ctrl_transfer(direction, request, value, 0, data, timeout=timeout)
        if not direction & 0x80 and result != len(data):
            raise OSError(f'short DFU download: expected {len(data)}, got {result}')
        return result

    def status(self, deadline=None):
        deadline = time.monotonic() + 5 if deadline is None else deadline
        # bwPollTimeout is a minimum, including after a previous wait timed out.
        # Never replace it with a shorter USB timeout or a capped polling sleep.
        now = time.monotonic()
        if self._poll_not_before > now:
            time.sleep(max(0.0, min(self._poll_not_before, deadline) - now))
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('DFU state timeout')
        timeout = max(1, min(2000, int(remaining * 1000)))
        raw = bytes(self.control(0xA1, 3, 0, 6, timeout=timeout))
        if len(raw) != 6:
            raise OSError('short DFU status')
        delay = int.from_bytes(raw[1:4], 'little')
        self._poll_not_before = time.monotonic() + delay / 1000
        if time.monotonic() > deadline:
            raise TimeoutError('DFU state timeout')
        return raw[0], raw[4], delay

    def wait(self, states):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            status, state, _ = self.status(deadline)
            if status:
                raise OSError(f'DFU status error {status}, state {state}')
            if state in states:
                return
            # A zero timeout from a transient state must not cause a busy loop.
            self._poll_not_before = max(self._poll_not_before, time.monotonic() + 0.001)
        raise TimeoutError('DFU state timeout')

    def abort(self):
        # A failed/ambiguous transfer can still be busy in ROM. ABORT is not
        # legal in dfuDNBUSY: finish the bounded status handshake first.
        deadline = time.monotonic() + 5
        while True:
            status, state, _ = self.status(deadline)
            if state not in {3, 4}:
                break
            self._poll_not_before = max(self._poll_not_before, time.monotonic() + 0.001)
        if state == 10:
            self.control(0x21, 4, 0, b'')
            self.wait({2})
        elif status or state not in {2, 5, 9}:
            raise OSError(f'cannot abort DFU status {status}, state {state}')
        self.control(0x21, 6, 0, b'')
        self.wait({2})

    def address(self, address):
        self.abort()
        self.control(0x21, 1, 0, b'\x21' + struct.pack('<I', address))
        self.wait({5})
        self.abort()

    def read(self, address, length):
        if not (length > 0 and BASE <= address and address + length <= BASE + FLASH_SIZE):
            raise ValueError('read outside flash')
        self.address(address)
        data = bytearray()
        for offset in range(0, length, BLOCK_SIZE):
            count = min(BLOCK_SIZE, length - offset)
            block = 2 + offset // BLOCK_SIZE
            skip = 0
            transfer_count = count
            if count < BLOCK_SIZE:
                # DFUse multiplies block number by this request's length, not
                # the previous block size. Rebase a short tail and use block 2.
                # ROM memory transfers require at least two bytes (AN3156).
                if count == 1:
                    skip = int(address + offset > BASE)
                    transfer_count = 2
                self.address(address + offset - skip)
                block = 2
            part = bytes(self.control(0xA1, 2, block, transfer_count))
            if len(part) != transfer_count:
                raise OSError('short DFU upload')
            data.extend(part[skip:skip + count])
        self.abort()
        return bytes(data)

    def erase(self, address):
        if address - BASE not in APP_SECTORS:
            raise ValueError('erase outside application sectors')
        self.abort()
        self.control(0x21, 1, 0, b'\x41' + struct.pack('<I', address))
        self.wait({5})
        self.abort()

    def write(self, address, data):
        if not (0 < len(data) <= BLOCK_SIZE and address % 4 == 0
                and BASE <= address and address + len(data) <= BASE + APP_SIZE):
            raise ValueError('write outside aligned application region')
        # F405 ROM writes at least two bytes; pad only erased application bytes
        # to a word boundary so any final image tail is handled deterministically.
        data = bytes(data).ljust((len(data) + 3) & ~3, b'\xff')
        self.address(address)
        self.control(0x21, 1, 2, data)
        self.wait({5})
        self.abort()

    def jump_to_application(self):
        self.address(BASE)
        self.control(0x21, 1, 0, b'')
        # GETSTATUS initiates manifestation; USB disconnect is expected. The
        # caller must separately verify the returning application and config.
        status, state, _ = self.status()
        if status:
            raise OSError(f'DFU status error {status}, state {state}')
        if state != 7:
            raise OSError(f'DFU manifestation did not start: state {state}')
