"""AS5048B register decoding and a stdlib-only Linux I2C transport.

Register map: ams AS5048A/AS5048B datasheet DS000298 v1-11 (2018-01-29),
Figure 31 "Register Map I2C" (pp. 25-26), Figure 32 slave address (p. 27),
"Diagnostic Functions" (p. 30):

    0x15  I2C slave address, OTP bits <4:0> (MSB internally inverted)
    0x16  OTP zero position <13:6>          0x17  zero position <5:0>
    0xFA  AGC (0 = strong field, 255 = weak field)
    0xFB  diagnostics: bit3 Comp High (weak field), bit2 Comp Low (strong
          field), bit1 COF (CORDIC overflow -> angle/magnitude invalid),
          bit0 OCF (offset compensation finished, high after power-up)
    0xFC  magnitude <13:6>                  0xFD  magnitude <5:0>
    0xFE  angle <13:6>                      0xFF  angle <5:0>

The angle register already includes the chip's zero-position correction.
This module never writes any register; software zero offsets live in the
suspension encoder config file instead of sensor OTP.  The datasheet supports
random/sequential reads and asks for STOP only after a complete sequence, so
each sensor is read with one combined write(reg) + repeated-start read.
"""
from __future__ import annotations

import ctypes
from dataclasses import dataclass
import errno
import fcntl
import math
import os


COUNTS_PER_TURN = 1 << 14
RAD_PER_COUNT = 2.0 * math.pi / COUNTS_PER_TURN

REG_SLAVE_ADDRESS = 0x15
REG_ZERO_HIGH = 0x16
REG_AGC = 0xFA
READOUT_LENGTH = 6  # 0xFA..0xFF: AGC, DIAG, MAG_H, MAG_L, ANGLE_H, ANGLE_L

DIAG_OCF = 0x01
DIAG_COF = 0x02
DIAG_COMP_LOW = 0x04
DIAG_COMP_HIGH = 0x08

# linux/i2c-dev.h, linux/i2c.h
I2C_FUNCS = 0x0705
I2C_RDWR = 0x0707
I2C_M_RD = 0x0001
I2C_FUNC_I2C = 0x00000001
# Errors meaning the device node itself went away: drop the fd and reopen on
# the next poll.  NACK (EREMOTEIO/ENXIO) is per-address and keeps the fd.
_REOPEN_ERRNOS = frozenset({errno.ENODEV, errno.EBADF, errno.ENOENT})


def _join14(high: int, low: int) -> int:
    return ((high & 0xFF) << 6) | (low & 0x3F)


@dataclass(frozen=True)
class As5048bReadout:
    angle_counts: int
    magnitude: int
    agc: int
    diag: int

    @property
    def ocf(self) -> bool:
        return bool(self.diag & DIAG_OCF)

    @property
    def cof(self) -> bool:
        return bool(self.diag & DIAG_COF)

    @property
    def comp_low(self) -> bool:
        return bool(self.diag & DIAG_COMP_LOW)

    @property
    def comp_high(self) -> bool:
        return bool(self.diag & DIAG_COMP_HIGH)

    @property
    def invalid_reason(self) -> str | None:
        """Angle is unusable unless offset compensation finished without overflow."""
        if self.cof:
            return "CORDIC overflow (COF)"
        if not self.ocf:
            return "offset compensation not finished (OCF=0)"
        return None

    @property
    def warnings(self) -> tuple[str, ...]:
        found = []
        if self.comp_high:
            found.append("weak field (Comp High)")
        if self.comp_low:
            found.append("strong field (Comp Low)")
        return tuple(found)

    @property
    def angle_rad(self) -> float:
        return self.angle_counts * RAD_PER_COUNT


def decode_readout(block: bytes) -> As5048bReadout:
    """Decode the 6-byte 0xFA..0xFF readout block."""
    if len(block) != READOUT_LENGTH:
        raise ValueError(f"expected {READOUT_LENGTH} bytes, got {len(block)}")
    agc, diag, mag_h, mag_l, ang_h, ang_l = block
    if diag & 0xF0 or mag_l & 0xC0 or ang_l & 0xC0:
        # Unused bits read as zero on a healthy device; anything else is a
        # corrupted transfer (e.g. a different device answering the address).
        raise ValueError(f"unused register bits set: {bytes(block).hex()}")
    return As5048bReadout(
        angle_counts=_join14(ang_h, ang_l),
        magnitude=_join14(mag_h, mag_l),
        agc=agc,
        diag=diag,
    )


@dataclass(frozen=True)
class As5048bOtpInfo:
    slave_address_bits: int  # register 0x15 bits <4:0> as read (MSB reads 0)
    zero_position_counts: int


def decode_otp_info(block: bytes) -> As5048bOtpInfo:
    """Decode the 3-byte 0x15..0x17 block (slave address + OTP zero)."""
    if len(block) != 3:
        raise ValueError(f"expected 3 bytes, got {len(block)}")
    return As5048bOtpInfo(
        slave_address_bits=block[0] & 0x1F,
        zero_position_counts=_join14(block[1], block[2]),
    )


class _I2cMsg(ctypes.Structure):
    _fields_ = [
        ("addr", ctypes.c_uint16),
        ("flags", ctypes.c_uint16),
        ("len", ctypes.c_uint16),
        ("buf", ctypes.POINTER(ctypes.c_uint8)),
    ]


class _I2cRdwrIoctlData(ctypes.Structure):
    _fields_ = [
        ("msgs", ctypes.POINTER(_I2cMsg)),
        ("nmsgs", ctypes.c_uint32),
    ]


class LinuxI2cBus:
    """Minimal /dev/i2c-N register reader using the I2C_RDWR ioctl."""

    def __init__(self, bus: int) -> None:
        self.bus = int(bus)
        self.path = f"/dev/i2c-{self.bus}"
        self._fd: int | None = None

    def _ensure_open(self) -> int:
        if self._fd is None:
            fd = os.open(self.path, os.O_RDWR | os.O_CLOEXEC)
            try:
                funcs = ctypes.c_ulong(0)
                fcntl.ioctl(fd, I2C_FUNCS, funcs, True)
                if not funcs.value & I2C_FUNC_I2C:
                    raise OSError(errno.EOPNOTSUPP, f"{self.path} has no plain-I2C (I2C_RDWR) support")
            except BaseException:
                os.close(fd)
                raise
            self._fd = fd
        return self._fd

    def read_registers(self, address: int, register: int, length: int) -> bytes:
        if not 0x03 <= address <= 0x77:
            raise ValueError(f"invalid 7-bit I2C address {address:#x}")
        if not 1 <= length <= 32:
            raise ValueError("length must be 1..32")
        fd = self._ensure_open()
        write_buf = (ctypes.c_uint8 * 1)(register & 0xFF)
        read_buf = (ctypes.c_uint8 * length)()
        msgs = (_I2cMsg * 2)(
            _I2cMsg(address, 0, 1, ctypes.cast(write_buf, ctypes.POINTER(ctypes.c_uint8))),
            _I2cMsg(address, I2C_M_RD, length, ctypes.cast(read_buf, ctypes.POINTER(ctypes.c_uint8))),
        )
        data = _I2cRdwrIoctlData(ctypes.cast(msgs, ctypes.POINTER(_I2cMsg)), 2)
        # The kernel follows msgs[].buf directly; write_buf/read_buf/msgs stay
        # referenced by locals until the ioctl returns.  I2C_RDWR returns the
        # number of messages transferred.
        try:
            transferred = fcntl.ioctl(fd, I2C_RDWR, data, True)
        except OSError as exc:
            if exc.errno in _REOPEN_ERRNOS:
                self.close()
            raise
        if transferred != 2:
            raise OSError(errno.EIO, f"I2C_RDWR transferred {transferred}/2 messages")
        return bytes(read_buf)

    def close(self) -> None:
        if self._fd is not None:
            fd, self._fd = self._fd, None
            os.close(fd)


def read_readout(bus, address: int) -> As5048bReadout:
    return decode_readout(bus.read_registers(address, REG_AGC, READOUT_LENGTH))


def read_otp_info(bus, address: int) -> As5048bOtpInfo:
    return decode_otp_info(bus.read_registers(address, REG_SLAVE_ADDRESS, 3))
