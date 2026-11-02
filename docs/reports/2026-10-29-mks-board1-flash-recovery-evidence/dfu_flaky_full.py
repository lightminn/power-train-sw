"""Read-only: 30 reads of the application region; unstable bytes by sector and bit. No write."""
import collections, json, sys
import usb.core, usb.util
sys.path.insert(0, "/workspace")
from tools.mks_dfu import BASE, APP_SIZE, APP_SECTORS, DfuTransport
dev = usb.core.find(idVendor=0x0483, idProduct=0xDF11)
dev.set_configuration(); i = dev.get_active_configuration()[(0, 0)]; i.set_altsetting()
t = DfuTransport(dev)
first = t.read(BASE, APP_SIZE)
seen = collections.defaultdict(set)
for _ in range(29):
    r = t.read(BASE, APP_SIZE)
    for k in (j for j in range(APP_SIZE) if r[j] != first[j]):
        seen[k].add(r[k])
def sector(off):
    return max(i for i, s in enumerate(APP_SECTORS) if off >= s)
by_sector = collections.Counter(sector(k) for k in seen)
bits = collections.Counter()
for k, vals in seen.items():
    for v in vals:
        bits[bin(v ^ first[k]).count("1")] += 1
print("unstable bytes:", len(seen), "by sector:", dict(sorted(by_sector.items())))
print("bit-flip widths:", dict(bits))
print("addresses:", [hex(BASE + k) for k in sorted(seen)])
json.dump({"unstable": [hex(BASE + k) for k in sorted(seen)]}, open("/out/unstable30.json", "w"))
