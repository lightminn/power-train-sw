#!/usr/bin/env python3
"""Reboot arm servos latched in a hardware error so the bridge can use them again.

Run with the bridge stopped: this opens the Dynamixel bus directly. Only servos
reporting an error are rebooted; a latched servo already has torque off, so the
reboot does not drop a joint that was holding. Goal PWM and other RAM values
reset, and the bridge writes them again at startup.

A reboot also forgets the multi-turn count of Extended Position axes (2 and 3),
so their angles can shift by whole servo turns. Check the arm before moving to a
saved pose after one of those was rebooted.
"""
import sys
import time

ARM_IDS = (11, 14, 13, 12, 16, 1)
EXTENDED_IDS = (14, 13)
ADDR_HARDWARE_ERROR = 70
ADDR_TORQUE = 64
PORT, BAUD, PROTOCOL = '/dev/ttyUSB0', 1000000, 2.0


def clear_errors(handler, port, ids=ARM_IDS, settle=time.sleep):
    """Return (rebooted ids, ids still in error). Absent servos are skipped."""
    rebooted = []
    for dxl_id in ids:
        error, result, _ = handler.read1ByteTxRx(port, dxl_id, ADDR_HARDWARE_ERROR)
        if result != 0 or error == 0:
            continue
        print(f'ID {dxl_id}: hardware error {error:#04x}, rebooting')
        handler.reboot(port, dxl_id)
        rebooted.append(dxl_id)
    if rebooted:
        settle(1.0)
    failed = []
    for dxl_id in rebooted:
        error, result, _ = handler.read1ByteTxRx(port, dxl_id, ADDR_HARDWARE_ERROR)
        if result != 0 or error != 0:
            failed.append(dxl_id)
    return rebooted, failed


def main():
    from dynamixel_sdk import PacketHandler, PortHandler
    port = PortHandler(PORT)
    if not (port.openPort() and port.setBaudRate(BAUD)):
        print(f'cannot open {PORT}')
        return 1
    try:
        rebooted, failed = clear_errors(PacketHandler(PROTOCOL), port)
    finally:
        port.closePort()
    print(f'rebooted {rebooted}, still failing {failed}')
    for dxl_id in set(rebooted) & set(EXTENDED_IDS):
        print(f'WARNING ID {dxl_id}: multi-turn count reset, verify the joint angle')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
