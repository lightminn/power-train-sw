"""폴더별 장면 확인용 썸네일 시트 만들기.
사용: python contact_sheet.py <기록들이 든 폴더> [--n 6]
각 기록 폴더(color/ 와 depth/ 가 있는 폴더)마다 컬러 프레임 n장을 균등 간격으로 뽑아
out_sheets/<폴더이름>.jpg 로 저장하고, 프레임 수와 깊이 유효 비율을 표로 출력한다."""
import argparse, glob, os
import cv2
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("root")
ap.add_argument("--n", type=int, default=6)
a = ap.parse_args()

TW, TH = 320, 180
os.makedirs("out_sheets", exist_ok=True)
print(f"{'폴더':<18}{'프레임':>6}  {'깊이 유효%':>9}  시트")
for d in sorted(glob.glob(os.path.join(a.root, "*"))):
    cdir = os.path.join(d, "color")
    if not os.path.isdir(cdir):
        continue
    cf = sorted(glob.glob(os.path.join(cdir, "*.jpg")) + glob.glob(os.path.join(cdir, "*.png")))
    df = sorted(glob.glob(os.path.join(d, "depth", "*.png")))
    if not cf:
        continue
    idx = np.unique(np.linspace(0, len(cf) - 1, min(a.n, len(cf))).astype(int))
    tiles = []
    for i in idx:
        im = cv2.imread(cf[i])
        if im is None:
            continue
        im = cv2.resize(im, (TW, TH))
        cv2.putText(im, f"#{i}", (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        tiles.append(im)
    cols = 3
    while len(tiles) % cols:
        tiles.append(np.zeros((TH, TW, 3), np.uint8))
    rows = [np.hstack(tiles[r:r + cols]) for r in range(0, len(tiles), cols)]
    sheet = np.vstack(rows)
    name = os.path.basename(d)
    cv2.putText(sheet, name, (6, sheet.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    out = os.path.join("out_sheets", name + ".jpg")
    cv2.imwrite(out, sheet)
    valid = ""
    if df:
        dm = cv2.imread(df[len(df) // 2], cv2.IMREAD_UNCHANGED)
        if dm is not None:
            valid = f"{100.0 * np.count_nonzero(dm) / dm.size:.0f}"
    print(f"{name:<18}{len(cf):>6}  {valid:>9}  {out}")
