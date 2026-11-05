import ctypes
import errno
import math

import pytest

from suspension_encoders import as5048b
from suspension_encoders.as5048b import (
    COUNTS_PER_TURN,
    I2C_M_RD,
    I2C_RDWR,
    LinuxI2cBus,
    decode_otp_info,
    decode_readout,
)
from suspension_encoders.tests.fake_bus import readout_block


@pytest.mark.parametrize("counts", [0, 1, 63, 64, 8192, COUNTS_PER_TURN - 1])
def test_angle_is_high_byte_bits_13_6_plus_low_byte_bits_5_0(counts):
    readout = decode_readout(readout_block(counts, magnitude=0x2ABC & 0x3FFF))
    assert readout.angle_counts == counts
    assert readout.magnitude == 0x2ABC & 0x3FFF
    assert readout.angle_rad == pytest.approx(2 * math.pi * counts / COUNTS_PER_TURN)


def test_register_block_order_matches_datasheet_fa_to_ff():
    block = bytes([0x55, 0x01, 0xFF, 0x3F, 0x80, 0x01])
    readout = decode_readout(block)
    assert readout.agc == 0x55
    assert readout.diag == 0x01
    assert readout.magnitude == 0x3FFF
    assert readout.angle_counts == (0x80 << 6) | 0x01


def test_cof_or_missing_ocf_invalidates_angle_but_field_warnings_do_not():
    assert decode_readout(readout_block(5, diag=0x01)).invalid_reason is None
    assert "COF" in decode_readout(readout_block(5, diag=0x03)).invalid_reason
    assert "OCF" in decode_readout(readout_block(5, diag=0x00)).invalid_reason
    weak = decode_readout(readout_block(5, diag=0x09))
    assert weak.invalid_reason is None
    assert weak.warnings == ("weak field (Comp High)",)
    strong = decode_readout(readout_block(5, diag=0x05))
    assert strong.warnings == ("strong field (Comp Low)",)


@pytest.mark.parametrize("block", [
    bytes([0, 0x11, 0, 0, 0, 0]),   # diag bit 4 is unused
    bytes([0, 0x01, 0, 0x40, 0, 0]),  # magnitude low bits 7:6 unused
    bytes([0, 0x01, 0, 0, 0, 0x80]),  # angle low bits 7:6 unused
    bytes(5),
])
def test_corrupt_or_short_blocks_are_rejected(block):
    with pytest.raises(ValueError):
        decode_readout(block)


def test_otp_block_decodes_slave_bits_and_zero_position():
    info = decode_otp_info(bytes([0x0F, 0x12, 0x34]))
    assert info.slave_address_bits == 0x0F
    assert info.zero_position_counts == (0x12 << 6) | 0x34


def test_ctypes_structs_match_kernel_layout_on_64bit():
    if ctypes.sizeof(ctypes.c_void_p) != 8:
        pytest.skip("layout assertion is for LP64 targets (Jetson aarch64, x86_64)")
    assert ctypes.sizeof(as5048b._I2cMsg) == 16
    assert as5048b._I2cMsg.buf.offset == 8
    assert ctypes.sizeof(as5048b._I2cRdwrIoctlData) == 16


def _fake_linux(monkeypatch, calls, *, funcs=as5048b.I2C_FUNC_I2C, rdwr=None):
    def fake_open(path, flags):
        calls.append(("open", path))
        return 42

    def fake_ioctl(fd, request, data, mutate=False):
        assert fd == 42 and mutate is True
        if request == as5048b.I2C_FUNCS:
            data.value = funcs
            return 0
        assert request == I2C_RDWR
        calls.append("ioctl")
        return rdwr(data)

    monkeypatch.setattr(as5048b.os, "open", fake_open)
    monkeypatch.setattr(as5048b.os, "close", lambda fd: calls.append(("close", fd)))
    monkeypatch.setattr(as5048b.fcntl, "ioctl", fake_ioctl)


def test_linux_bus_issues_one_combined_write_then_repeated_start_read(monkeypatch):
    calls = []

    def rdwr(data):
        assert data.nmsgs == 2
        write, read = data.msgs[0], data.msgs[1]
        assert (write.addr, write.flags, write.len, write.buf[0]) == (0x41, 0, 1, 0xFA)
        assert (read.addr, read.flags, read.len) == (0x41, I2C_M_RD, 6)
        for index, value in enumerate(readout_block(1234)):
            read.buf[index] = value
        return 2

    _fake_linux(monkeypatch, calls, rdwr=rdwr)
    bus = LinuxI2cBus(7)
    assert as5048b.read_readout(bus, 0x41).angle_counts == 1234
    assert as5048b.read_readout(bus, 0x41).angle_counts == 1234
    bus.close()
    assert calls == [("open", "/dev/i2c-7"), "ioctl", "ioctl", ("close", 42)]


def test_linux_bus_rejects_adapter_without_plain_i2c(monkeypatch):
    calls = []
    _fake_linux(monkeypatch, calls, funcs=0, rdwr=lambda data: 2)
    with pytest.raises(OSError, match="I2C_RDWR"):
        LinuxI2cBus(7).read_registers(0x40, 0xFA, 6)
    assert calls == [("open", "/dev/i2c-7"), ("close", 42)]


def test_linux_bus_short_transfer_is_an_error(monkeypatch):
    _fake_linux(monkeypatch, [], rdwr=lambda data: 1)
    with pytest.raises(OSError, match="1/2"):
        LinuxI2cBus(7).read_registers(0x40, 0xFA, 6)


def test_linux_bus_reopens_after_device_loss_but_not_after_nack(monkeypatch):
    calls = []
    outcomes = iter([errno.EREMOTEIO, errno.ENODEV, None])

    def rdwr(data):
        code = next(outcomes)
        if code is not None:
            raise OSError(code, "x")
        return 2

    _fake_linux(monkeypatch, calls, rdwr=rdwr)
    bus = LinuxI2cBus(7)
    for _ in range(2):
        with pytest.raises(OSError):
            bus.read_registers(0x40, 0xFA, 6)
    bus.read_registers(0x40, 0xFA, 6)
    assert calls == [("open", "/dev/i2c-7"), "ioctl", "ioctl", ("close", 42),
                     ("open", "/dev/i2c-7"), "ioctl"]


def test_linux_bus_rejects_bad_address_before_opening(monkeypatch):
    monkeypatch.setattr(as5048b.os, "open", lambda *a: pytest.fail("opened"))
    with pytest.raises(ValueError):
        LinuxI2cBus(7).read_registers(0x80, 0xFA, 6)


def test_module_never_issues_register_writes():
    source = as5048b.__file__
    text = open(source, encoding="utf-8").read()
    # Only the register-pointer byte of a combined read is ever sent.
    assert "write_registers" not in text
    assert "I2C_SLAVE" not in text
