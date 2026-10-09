# scan_terrain.py   사용법: python scan_terrain.py
import os, glob, collections
import numpy as np, cv2
from terrain import load_detector

for f in sorted(glob.glob("data/*/")):
    f = f.rstrip("/")
    det = load_detector(f)
    st, zs, h = collections.Counter(), {"left": [], "center": [], "right": []}, []
    for p in sorted(glob.glob(os.path.join(f, "depth", "*.png"))):
        r = det.process(cv2.imread(p, cv2.IMREAD_UNCHANGED))
        st[r["status"]] += 1
        for k, v in r["zones"].items():
            if v is not None:
                zs[k].append(v)
        if r["camera_height_m"]:
            h.append(r["camera_height_m"])
    n = sum(st.values())
    z = "  ".join(
        f"{k[0].upper()}:{np.median(v):.2f}m({len(v)}/{n})"
        if v
        else f"{k[0].upper()}:-"
        for k, v in zs.items()
    )
    print(
        f"{os.path.basename(f):12s} {dict(st)}  {z}  높이 {np.median(h) if h else float('nan'):.3f}m"
    )
