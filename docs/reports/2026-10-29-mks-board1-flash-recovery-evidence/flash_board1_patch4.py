"""Board 1 (node 11/12, STM32 UID 335233643235): patch4 application over unstable flash.

The board does not boot. 30 ROM DFU reads of the application flip bits at 14 addresses,
all inside application sector 1 (0x08004000-0x08007FFF); sectors 0 and 2-9 and the NVM
(last 256 KiB) read identically every time. tools/mks_dfu.replace_application
requires an exact full-flash match before erasing, which this board cannot satisfy, so
this script applies the same application-only update with one relaxed precondition:

  * NVM must equal every backup read exactly (it is preserved, never erased);
  * the application may differ from backup read 1 only inside the degraded sector 1.

No mass erase, option bytes, NVM erase, calibration or arm. The board is never jumped to
the application: the operator returns the BOOT0 switch and resets it.

usage (Jetson, fresh container per step, CAN owner stopped):
  python3 flash_board1_patch4.py check   OUT_DIR   # read-only preconditions
  python3 flash_board1_patch4.py program OUT_DIR   # erase app sectors 0-9, write, verify
"""
import datetime
import gzip
import hashlib
import json
import pathlib
import sys

import usb.core
import usb.util

sys.path.insert(0, "/workspace")
sys.path.insert(0, "/workspace/motor_control")
from chassis.usb_session import motor_session
from tools.mks_dfu import (APP_SECTORS, APP_SIZE, BASE, BLOCK_SIZE, FLASH_SIZE,
                           DfuTransport, validate_image)

SERIAL = "335233643235"
LAYOUT = "@Internal Flash  /0x08000000/04*016Kg,01*064Kg,07*128Kg"
IMAGE_GZ = pathlib.Path("/workspace/docs/reports/2026-10-21-mks-can-patch4-evidence/"
                        "build-patch4/ODriveFirmware.bin.gz")
IMAGE_SHA256 = "15462f7f64745b03d1347fccda8091cd34b795056fefee64f87e794dc8a4b3fc"
DEGRADED = range(0x4000, 0x8000)        # sector 1 offsets
STABILITY_READS = 30

step, out = sys.argv[1], pathlib.Path(sys.argv[2])
record = {"step": step, "serial": SERIAL,
          "started_at": datetime.datetime.now(datetime.timezone.utc).isoformat()}


def save():
    (out / f"{SERIAL}-{step}-result.json").write_text(json.dumps(record, indent=2, default=str))


def attach():
    devices = list(usb.core.find(find_all=True, idVendor=0x0483, idProduct=0xDF11))
    matches = [d for d in devices if usb.util.get_string(d, d.iSerialNumber).upper() == SERIAL]
    if len(matches) != 1:
        raise RuntimeError(f"expected one ROM DFU device {SERIAL}, found {len(matches)}")
    dev = matches[0]
    dev.set_configuration()
    interface = dev.get_active_configuration()[(0, 0)]
    if usb.util.get_string(dev, interface.iInterface).strip() != LAYOUT:
        raise RuntimeError("unexpected flash layout")
    interface.set_altsetting()
    return DfuTransport(dev)


def load_backups():
    reads = [(out / f"{SERIAL}-read{i}.bin").read_bytes() for i in (1, 2, 3)]
    if any(len(r) != FLASH_SIZE for r in reads):
        raise ValueError("backup reads must be 1 MiB")
    nvm = reads[0][APP_SIZE:]
    if any(r[APP_SIZE:] != nvm for r in reads):
        raise ValueError("backup NVM reads differ")
    return reads[0], nvm


def load_image():
    image = gzip.decompress(IMAGE_GZ.read_bytes())
    if hashlib.sha256(image).hexdigest() != IMAGE_SHA256:
        raise ValueError("patch4 image hash mismatch")
    validate_image(image)
    return image


def precondition(transport, reference, nvm):
    current = transport.read(BASE, FLASH_SIZE)
    if current[APP_SIZE:] != nvm:
        raise ValueError("current NVM differs from backup; no erase performed")
    unexpected = [BASE + i for i in range(APP_SIZE)
                  if current[i] != reference[i] and i not in DEGRADED]
    if unexpected:
        raise ValueError(f"application differs outside degraded sector 1: "
                         f"{[hex(a) for a in unexpected[:8]]}; no erase performed")
    return [hex(BASE + i) for i in range(APP_SIZE) if current[i] != reference[i]]


def program(transport, image, nvm):
    expected = image.ljust(APP_SIZE, b"\xff")
    for offset in APP_SECTORS:
        transport.erase(BASE + offset)
    for offset in range(0, len(image), BLOCK_SIZE):
        chunk = image[offset:offset + BLOCK_SIZE]
        if chunk != b"\xff" * len(chunk):
            transport.write(BASE + offset, chunk)
    unstable = set()
    for _ in range(STABILITY_READS):
        app = transport.read(BASE, APP_SIZE)
        unstable.update(BASE + i for i in range(APP_SIZE) if app[i] != expected[i])
    record["readback_unstable_addresses"] = sorted(hex(a) for a in unstable)
    if unstable:
        raise OSError(f"application readback differs at {len(unstable)} addresses")
    if transport.read(BASE + APP_SIZE, FLASH_SIZE - APP_SIZE) != nvm:
        raise OSError("NVM readback differs after programming")


reference, nvm = load_backups()
image = load_image()
record.update(image_sha256=IMAGE_SHA256, image_bytes=len(image),
              backup_nvm_sha256=hashlib.sha256(nvm).hexdigest())
with motor_session(f"mks_board1_patch4_{step}"):
    transport = attach()
    try:
        record["precondition_diff"] = precondition(transport, reference, nvm)
        if step == "check":
            record["ready"] = True
        elif step == "program":
            program(transport, image, nvm)
            record["program_verified"] = True
        else:
            raise SystemExit(f"unknown step {step}")
    except Exception as error:
        record["error"] = repr(error)
        record["note"] = "board left in ROM DFU; no application jump"
        try:  # evidence only; never a restore write
            record["nvm_intact_after_error"] = (
                transport.read(BASE + APP_SIZE, FLASH_SIZE - APP_SIZE) == nvm)
        except Exception as read_error:
            record["nvm_check_error"] = repr(read_error)
        save()
        raise
    record["finished_at"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
    save()
print(json.dumps(record, indent=2, default=str))
