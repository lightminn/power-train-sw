"""Read-only post-program check: 20 full reads vs patch4 image + backup NVM."""
import gzip, hashlib, pathlib, sys
import usb.core, usb.util
sys.path.insert(0, "/workspace")
from tools.mks_dfu import BASE, APP_SIZE, FLASH_SIZE, DfuTransport
out = pathlib.Path("/out")
img = gzip.decompress(pathlib.Path("/workspace/docs/reports/2026-10-21-mks-can-patch4-evidence/build-patch4/ODriveFirmware.bin.gz").read_bytes()).ljust(APP_SIZE, b"\xff")
nvm = (out / "335233643235-read1.bin").read_bytes()[APP_SIZE:]
dev = usb.core.find(idVendor=0x0483, idProduct=0xDF11)
assert usb.util.get_string(dev, dev.iSerialNumber).upper() == "335233643235"
dev.set_configuration(); i = dev.get_active_configuration()[(0, 0)]; i.set_altsetting()
t = DfuTransport(dev)
bad_app, bad_nvm = set(), 0
for _ in range(20):
    f = t.read(BASE, FLASH_SIZE)
    bad_app.update(k for k in range(APP_SIZE) if f[k] != img[k])
    bad_nvm += f[APP_SIZE:] != nvm
print(f"20 reads: app mismatched bytes={len(bad_app)} {[hex(BASE+k) for k in sorted(bad_app)][:10]} nvm mismatched reads={bad_nvm}")
