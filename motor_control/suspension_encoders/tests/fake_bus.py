"""In-memory AS5048B bus double: register files per address."""
from __future__ import annotations

import errno


def readout_block(counts: int, *, agc: int = 120, diag: int = 0x01, magnitude: int = 4000) -> bytes:
    return bytes([
        agc, diag,
        (magnitude >> 6) & 0xFF, magnitude & 0x3F,
        (counts >> 6) & 0xFF, counts & 0x3F,
    ])


class FakeAs5048bBus:
    def __init__(self, devices: dict[int, int] | None = None) -> None:
        self.registers: dict[int, dict[int, int]] = {}
        self.reads: list[tuple[int, int, int]] = []
        self.writes = 0
        self.closed = False
        for address, counts in (devices or {}).items():
            self.set_angle(address, counts)

    def set_angle(self, address: int, counts: int, *, diag: int = 0x01, agc: int = 120,
                  magnitude: int = 4000, zero_reg: int = 0) -> None:
        regs = self.registers.setdefault(address, {})
        for offset, value in enumerate(readout_block(counts, agc=agc, diag=diag, magnitude=magnitude)):
            regs[0xFA + offset] = value
        regs[0x15] = 0x00
        regs[0x16] = (zero_reg >> 6) & 0xFF
        regs[0x17] = zero_reg & 0x3F

    def remove(self, address: int) -> None:
        self.registers.pop(address, None)

    def read_registers(self, address: int, register: int, length: int) -> bytes:
        self.reads.append((address, register, length))
        regs = self.registers.get(address)
        if regs is None:
            raise OSError(errno.EREMOTEIO, "Remote I/O error")
        return bytes(regs.get(register + i, 0) for i in range(length))

    def close(self) -> None:
        self.closed = True
