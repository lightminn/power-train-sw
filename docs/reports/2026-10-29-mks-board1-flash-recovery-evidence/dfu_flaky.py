"""Read-only: repeat reads of sector 1 and NVM; tally unstable bytes. No erase/write/jump."""
import collections, hashlib, sys
import usb.core, usb.util
sys.path.insert(0, "/workspace")
from tools.mks_dfu import BASE, APP_SIZE, FLASH_SIZE, DfuTransport
dev = usb.core.find(idVendor=0x0483, idProduct=0xDF11)
dev.set_configuration(); i = dev.get_active_configuration()[(0, 0)]; i.set_altsetting()
t = DfuTransport(dev)
S1, S1_LEN = BASE + 0x4000, 0x4000
tallies = collections.defaultdict(collections.Counter)
nvm_hashes = set(); app_hashes = set()
for n in range(10):
    s1 = t.read(S1, S1_LEN)
    if n == 0: base = s1
    for k in range(S1_LEN):
        tallies[k][s1[k]] += 1
    if n < 3:
        full = t.read(BASE, FLASH_SIZE)
        nvm_hashes.add(hashlib.sha256(full[APP_SIZE:]).hexdigest()[:16])
flaky = {k: c for k, c in tallies.items() if len(c) > 1}
for k, c in sorted(flaky.items()):
    vals = ", ".join(f"0x{v:02X}({v:08b})x{cnt}" for v, cnt in c.most_common())
    print(f"0x{S1+k:08X}: {vals}")
print(f"sector1 unstable bytes over 10 reads: {len(flaky)}")
print(f"NVM sha over 3 full reads: {sorted(nvm_hashes)}")
