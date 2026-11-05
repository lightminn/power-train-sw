"""Read-only ROM DFU backup of one STM32F405 (MKS ODrive) — no erase/write/jump.

usage: python3 dfu_backup_readonly.py OUT_DIR
"""
import hashlib
import json
import pathlib
import sys
import time

import usb.core
import usb.util

sys.path.insert(0, "/workspace")
from tools.mks_dfu import APP_SIZE, BASE, FLASH_SIZE, DfuTransport, validate_image

out = pathlib.Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=True)
devices = list(usb.core.find(find_all=True, idVendor=0x0483, idProduct=0xDF11))
assert len(devices) == 1, f"expected exactly one ROM DFU device, found {len(devices)}"
dev = devices[0]
serial = usb.util.get_string(dev, dev.iSerialNumber).upper()
dev.set_configuration()
interface = dev.get_active_configuration()[(0, 0)]
layout = usb.util.get_string(dev, interface.iInterface).strip()
assert layout == "@Internal Flash  /0x08000000/04*016Kg,01*064Kg,07*128Kg", layout
interface.set_altsetting()
transport = DfuTransport(dev)

reads = []
for attempt in (1, 2):
    t0 = time.monotonic()
    reads.append(transport.read(BASE, FLASH_SIZE))
    print(f"read {attempt}: {len(reads[-1])} bytes in {time.monotonic() - t0:.1f}s", flush=True)
assert reads[0] == reads[1], "two full-flash reads differ"
image = reads[0]
sha = hashlib.sha256(image).hexdigest()
(out / f"{serial}-original-1m.bin").write_bytes(image)

app, nvm = image[:APP_SIZE], image[APP_SIZE:]
try:
    validate_image(app.rstrip(b"\xff") or app[:8])
    app_vector = "valid"
except Exception as error:  # report, do not act
    app_vector = f"invalid: {error}"
report = dict(
    serial=serial, flash_sha256=sha,
    app_sha256=hashlib.sha256(app).hexdigest(), nvm_sha256=hashlib.sha256(nvm).hexdigest(),
    app_vector=app_vector,
    app_used_bytes=len(app.rstrip(b"\xff")),
    app_all_erased=app.count(0xFF) == len(app),
    nvm_non_erased_bytes=len(nvm) - nvm.count(0xFF),
    nvm_sector10_non_erased=len(nvm[:0x20000]) - nvm[:0x20000].count(0xFF),
    nvm_sector11_non_erased=len(nvm[0x20000:]) - nvm[0x20000:].count(0xFF),
)
(out / f"{serial}-backup-report.json").write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
