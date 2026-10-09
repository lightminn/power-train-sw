# overlay.py   사용법: python overlay.py data/obj_50cm
import os, sys, glob
import numpy as np, cv2
from terrain import load_detector


def render(depth, det, res):
    gray = (np.clip(depth.astype(np.float32) * det.scale / 5.0, 0, 1) * 255).astype(
        np.uint8
    )
    img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if res["status"] == "OK":
        img[det.floor_mask] = (
            0.5 * img[det.floor_mask] + 0.5 * np.array([0, 200, 0])
        ).astype(np.uint8)
        img[det.obst_mask] = (0, 0, 255)
    h, w = img.shape[:2]
    z = res["zones"]
    f = lambda v: "-" if v is None else f"{v:.2f}m"
    ok = res["status"] == "OK"
    cv2.rectangle(img, (0, 0), (w, 30), (0, 120, 0) if ok else (0, 0, 200), -1)
    cv2.putText(
        img,
        f"{res['status']}   L {f(z['left'])}   C {f(z['center'])}   R {f(z['right'])}",
        (8, 21),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
    )
    for x in (w // 2 - 1,):  # 화면 중앙선 표시용
        cv2.line(img, (x, 30), (x, h), (255, 255, 255), 1)
    return img


if __name__ == "__main__":
    folder = sys.argv[1]
    det = load_detector(folder)
    out = os.path.join("out_overlay", os.path.basename(folder.rstrip("/")))
    os.makedirs(out, exist_ok=True)
    for p in sorted(glob.glob(os.path.join(folder, "depth", "*.png"))):
        depth = cv2.imread(p, cv2.IMREAD_UNCHANGED)
        res = det.process(depth)
        cv2.imwrite(os.path.join(out, os.path.basename(p)), render(depth, det, res))
    print("저장:", out)
