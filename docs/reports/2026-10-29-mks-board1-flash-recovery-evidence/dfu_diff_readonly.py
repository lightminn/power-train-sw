"""Read-only: read full flash 3x, report where reads differ. No erase/write/jump."""
import hashlib, pathlib, sys
import usb.core, usb.util
sys.path.insert(0, "/workspace")
from tools.mks_dfu import BASE, FLASH_SIZE, DfuTransport
out = pathlib.Path(sys.argv[1])
dev = usb.core.find(idVendor=0x0483, idProduct=0xDF11)
serial = usb.util.get_string(dev, dev.iSerialNumber).upper()
dev.set_configuration(); intf = dev.get_active_configuration()[(0, 0)]; intf.set_altsetting()
t = DfuTransport(dev)
reads = [t.read(BASE, FLASH_SIZE) for _ in range(3)]
for i, r in enumerate(reads):
    (out / f"{serial}-read{i+1}.bin").write_bytes(r)
    print(f"read{i+1} sha={hashlib.sha256(r).hexdigest()[:16]} ff={r.count(0xff)} zero={r.count(0)}")
def ranges(a, b):
    diff = [i for i in range(len(a)) if a[i] != b[i]]
    if not diff: return []
    out, start, prev = [], diff[0], diff[0]
    for i in diff[1:]:
        if i > prev + 1: out.append((start, prev)); start = i
        prev = i
    out.append((start, prev)); return out, len(diff)
for x, y in ((0, 1), (1, 2), (0, 2)):
    res = ranges(reads[x], reads[y])
    if not res: print(f"read{x+1} == read{y+1}"); continue
    rg, n = res
    print(f"read{x+1} vs read{y+1}: {n} bytes differ in {len(rg)} ranges; first: " +
          ", ".join(f"0x{BASE+s:08X}-0x{BASE+e:08X}" for s, e in rg[:6]))
print("serial", serial)
