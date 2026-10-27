# scan_seq.py — 이동 녹화 폴더를 프레임마다 판정해 CSV·그래프·요약표를 만든다.
# 사용법 (lidar_dev 폴더에서, terrain.py 와 같은 위치):
#   python scan_seq.py                      # data/ 아래 모든 폴더
#   python scan_seq.py data/20261023_180428 # 폴더 하나
#   python scan_seq.py --overlay-every 20   # 20프레임마다 깊이 화면에 바닥·장애물을 칠해 저장
# 결과: out_seq/<폴더>.csv, out_seq/<폴더>.png(그래프), out_seq/summary.csv, (옵션) out_seq/<폴더>_overlay/
import argparse, csv, glob, json, os, time
import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from terrain import load_detector

plt.rcParams["axes.unicode_minus"] = False
for fam in ("AppleGothic", "Apple SD Gothic Neo", "Noto Sans CJK JP", "NanumGothic"):  # 한글 글꼴(없으면 기본값)
    try:
        from matplotlib import font_manager as fm
        if any(fam == f.name for f in fm.fontManager.ttflist):
            plt.rcParams["font.family"] = fam
            break
    except Exception:
        pass

STATUS_COLOR = {"OK": "#16a34a", "HELD": "#eab308", "NO_GROUND": "#f97316", "SENSOR_ERROR": "#dc2626", "HOLD": "#6b7280"}


def paint(depth, det, res, scale):
    """깊이(회색) 위에 바닥 초록·장애물 빨강. 맨 위 줄에 상태와 구역 거리."""
    z = depth.astype(np.float32) * scale
    g = np.clip(z / 4.0 * 255, 0, 255).astype(np.uint8)
    img = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    if det.floor_mask is not None and res["status"] == "OK":
        img[det.floor_mask] = (0.5 * img[det.floor_mask] + 0.5 * np.array([60, 160, 60])).astype(np.uint8)
        img[det.obst_mask] = (40, 40, 230)
    zs = res["zones"]
    f = lambda v: "-" if v is None else f"{v:.2f}m"
    txt = f'{res["status"]} L{f(zs["left"])} C{f(zs["center"])} R{f(zs["right"])}'
    bar = np.zeros((28, img.shape[1], 3), np.uint8)
    cv2.putText(bar, txt, (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return np.vstack([bar, img])


def run_folder(folder, overlay_every, out, near_below=0.0, near_max=30):
    name = os.path.basename(folder.rstrip("/"))
    det = load_detector(folder)
    files = sorted(glob.glob(os.path.join(folder, "depth", "*.png")))
    rows = []
    ov_dir = os.path.join(out, name + "_overlay")
    if overlay_every or near_below:
        os.makedirs(ov_dir, exist_ok=True)
    near_saved = 0
    for i, p in enumerate(files):
        d = cv2.imread(p, cv2.IMREAD_UNCHANGED)
        t0 = time.time()
        r = det.process(d)
        ms = (time.time() - t0) * 1000
        status = r["status"]
        if status == "OK" and not r["ground_plane_ok"]:
            status = "HELD"          # 바닥을 이번 프레임에 못 찾아 직전 평면으로 판단한 프레임
        z = r["zones"]
        rows.append({"frame": i, "file": os.path.basename(p), "status": status,
                     "camera_height_m": r["camera_height_m"], "camera_tilt_deg": r["camera_tilt_deg"],
                     "left": z["left"], "center": z["center"], "right": z["right"],
                     "nearest": r["nearest_obstacle_m"], "proc_ms": round(ms, 1)})
        if overlay_every and i % overlay_every == 0 and d is not None:
            cv2.imwrite(os.path.join(ov_dir, f"{i:06d}.png"), paint(d, det, r, det.scale))
        elif (near_below and d is not None and r["nearest_obstacle_m"] is not None
              and r["nearest_obstacle_m"] < near_below and near_saved < near_max):
            cv2.imwrite(os.path.join(ov_dir, f"near_{i:06d}.png"), paint(d, det, r, det.scale))  # 가까운 값이 나온 프레임
            near_saved += 1
    # CSV
    keys = list(rows[0].keys()) if rows else []
    with open(os.path.join(out, name + ".csv"), "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    plot(name, rows, out)
    return name, summarize(rows)


def plot(name, rows, out):
    if not rows:
        return
    x = [r["frame"] for r in rows]
    fig, ax = plt.subplots(3, 1, figsize=(9, 7), sharex=True, gridspec_kw={"height_ratios": [3, 0.6, 1.4]})
    for key, c in (("left", "#2563eb"), ("center", "#dc2626"), ("right", "#16a34a")):
        ax[0].plot(x, [np.nan if r[key] is None else r[key] for r in rows], ".-", ms=3, lw=0.8, color=c, label=key)
    ax[0].set_ylabel("nearest obstacle (m)"); ax[0].legend(loc="upper right"); ax[0].set_title(name)
    ax[0].set_ylim(0, 3.2); ax[0].grid(alpha=0.3)
    for r in rows:
        ax[1].axvspan(r["frame"] - 0.5, r["frame"] + 0.5, color=STATUS_COLOR.get(r["status"], "#999"))
    ax[1].set_yticks([]); ax[1].set_ylabel("status")
    ax[1].text(0.01, -0.9, "green OK / yellow HELD / orange NO_GROUND / red SENSOR_ERROR", transform=ax[1].transAxes, fontsize=8)
    ax[2].plot(x, [np.nan if r["camera_height_m"] is None else r["camera_height_m"] for r in rows], color="#6b7280")
    ax[2].set_ylabel("camera height (m)"); ax[2].set_xlabel("frame"); ax[2].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out, name + ".png"), dpi=110)
    plt.close(fig)


def summarize(rows):
    n = len(rows)
    if not n:
        return {}
    cnt = lambda s: sum(r["status"] == s for r in rows)
    near = [r["nearest"] for r in rows if r["nearest"] is not None]
    cen = [r["center"] for r in rows]
    jumps = [abs(cen[i] - cen[i - 1]) for i in range(1, n) if cen[i] is not None and cen[i - 1] is not None]
    hs = [r["camera_height_m"] for r in rows if r["camera_height_m"] is not None]
    first = next((r for r in rows if r["nearest"] is not None), None)
    return {"frames": n, "OK%": round(100 * cnt("OK") / n), "HELD%": round(100 * cnt("HELD") / n),
            "NO_GROUND%": round(100 * cnt("NO_GROUND") / n), "SENSOR_ERROR%": round(100 * cnt("SENSOR_ERROR") / n),
            "obstacle_frames%": round(100 * len(near) / n),
            "first_detect_frame": first["frame"] if first else "",
            "first_detect_m": first["nearest"] if first else "",
            "min_nearest_m": min(near) if near else "",
            "center_jump_median_m": round(float(np.median(jumps)), 3) if jumps else "",
            "center_jump_max_m": round(float(np.max(jumps)), 3) if jumps else "",
            "cam_height_median_m": round(float(np.median(hs)), 3) if hs else "",
            "cam_height_std_m": round(float(np.std(hs)), 3) if hs else "",
            "proc_ms_mean": round(float(np.mean([r["proc_ms"] for r in rows])), 1)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folders", nargs="*")
    ap.add_argument("--overlay-every", type=int, default=0)
    ap.add_argument("--near-below", type=float, default=0.0, help="가장 가까운 장애물이 이 값(m)보다 작은 프레임을 near_<번호>.png 로 저장(최대 30장)")
    ap.add_argument("--out", default="out_seq")
    a = ap.parse_args()
    folders = a.folders or sorted(d for d in glob.glob("data/*/") if os.path.isdir(os.path.join(d, "depth")))
    os.makedirs(a.out, exist_ok=True)
    summ = []
    for f in folders:
        print("처리 중:", f, flush=True)
        name, s = run_folder(f, a.overlay_every, a.out, a.near_below)
        summ.append((name, s))
    if summ and summ[0][1]:
        cols = ["folder"] + list(summ[0][1].keys())
        with open(os.path.join(a.out, "summary.csv"), "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(cols)
            for n, s in summ:
                w.writerow([n] + [s.get(k, "") for k in cols[1:]])
        print("\n" + "  ".join(f"{c:>12}" for c in cols))
        for n, s in summ:
            print("  ".join(f"{str(v):>12}" for v in [n] + [s.get(k, "") for k in cols[1:]]))
        print(f"\n저장: {a.out}/summary.csv, 폴더별 .csv 와 .png")


if __name__ == "__main__":
    main()
