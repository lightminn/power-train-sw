# imu_calib.py — L515 내장 가속도계 다자세 정지 보정(축별 편향·배율, 타원체 맞춤). 기록마다 한 자세로 5 s 가만히 둔다(9자세 이상).
# v2(10-18, 교차검토 4차 반영): 축에 정확히 맞춰 놓을 필요가 없다 — 정지 평균 a_i에 대해 Σ(|(a_i − b)/s| − g)²를 최소화하는
#   b(편향)·s(배율)를 구한다(가우스–뉴턴). L515는 원통형이라 옆면 자세를 축에 맞추기 어렵다.
#   거부 조건: 자료 부족·비유한, 정지 아님(가속·자이로 표준편차, 자세별 자이로 평균이 다른 자세와 다름 = 일정한 회전),
#   자세 분포 부족(각 축이 ±방향 모두 중력과 60° 안으로 놓인 적 없음, 맞춤 조건수가 큼 = 식별 불가), 배율·편향 범위 밖,
#   보정 후 잔차·한 자세 빼고 맞춘 잔차(교차 확인)가 큼. 하나라도 걸리면 결과를 저장하지 않는다(--force면 valid=false로 저장).
# 모델 한계: 대각 배율 + 편향만이다. 축간 비직교·교차축 감도, IMU↔깊이↔차체 회전은 보정하지 않는다 → 크기가 g에 맞아도 방향 정확도는 입증되지 않는다.
#   바닥 법선과 비교할 때는 meta의 depth_to_accel 회전을 적용하고, 전복 계산기에는 차체 좌표 변환까지 적용한다. 움직이는 동안의 가속도는 중력이 아니다.
# 사용: python imu_calib.py <기록1> ... <기록N≥9> [--out imu_calib.json] [--force]   (record_seq.py 기록, imu.csv 필요)
import argparse, csv, json, os, sys
import numpy as np

G = 9.80665
Q = {"min_samples": 100, "min_window_s": 2.0, "trim_frac": 0.10,     # 가속·자이로 공통 구간의 앞뒤 10%를 버린 뒤
     "accel_std_max": 0.08, "gyro_std_max": 0.02, "gyro_dev_max": 0.08,  # 정지 판정(m/s², rad/s)
     "min_poses": 9,                                                   # 미지수 6개 — 6자세는 딱 맞아 잔차가 늘 0, 교차 확인에 9개 이상
     "cover_cos": 0.5,                                                 # 각 축 ±방향이 중력과 60° 안으로 놓인 자세가 있어야 함
     "scale_range": (0.95, 1.05), "bias_max": 0.6,                    # m/s²
     "resid_max": 0.05, "loo_resid_max": 0.08,                        # 보정 후 |a| − g, 한 자세 빼고 맞춘 뒤 그 자세의 |a| − g (m/s²)
     "cond_max": 1e3,                                                  # 맞춤 자코비안 조건수 — 크면 방향 분포가 배율·편향을 구분하지 못함(예: 정육면체 대각선만)
     "gyro_pose_dev_max": 0.02, "gyro_bias_abs_max": 0.05}            # 자세별 자이로 평균의 다른 자세 대비 차이, 공통 편향 크기(rad/s) — 일정한 회전 거부


def read_imu(folder):
    out = {"accel": [], "gyro": []}
    for r in csv.DictReader(open(os.path.join(folder, "imu.csv"))):
        if r["kind"] in out:
            out[r["kind"]].append([float(r["ts_ms"]) / 1000.0, float(r["x"]), float(r["y"]), float(r["z"])])
    return {k: np.array(v).reshape(-1, 4) for k, v in out.items()}


def static_pose(folder):
    """기록 하나 → 공통 구간 정지 평균과 판정. 반환: (자세 정보, 거부 사유 목록)."""
    d = read_imu(folder)
    A, W = d["accel"], d["gyro"]
    bad = []
    if len(A) < Q["min_samples"] or len(W) < Q["min_samples"]:
        return {"folder": folder, "n_accel": len(A), "n_gyro": len(W)}, [f"자료 부족(가속 {len(A)}, 자이로 {len(W)})"]
    t0, t1 = max(A[0, 0], W[0, 0]), min(A[-1, 0], W[-1, 0])
    span = t1 - t0
    t0, t1 = t0 + Q["trim_frac"] * span, t1 - Q["trim_frac"] * span
    A = A[(A[:, 0] >= t0) & (A[:, 0] <= t1), 1:]
    W = W[(W[:, 0] >= t0) & (W[:, 0] <= t1), 1:]
    info = {"folder": folder, "window_s": float(t1 - t0), "n_accel": len(A), "n_gyro": len(W)}
    if t1 - t0 < Q["min_window_s"] or len(A) < Q["min_samples"] or len(W) < Q["min_samples"]:
        return info, [f"정지 구간 부족({t1 - t0:.2f} s)"]
    if not (np.isfinite(A).all() and np.isfinite(W).all()):
        return info, ["비유한 값"]
    a, w = A.mean(0), W.mean(0)
    info.update(accel_mean=a.tolist(), accel_std=A.std(0).tolist(), gyro_mean=w.tolist(), gyro_std=W.std(0).tolist(),
                gyro_dev_max=float(np.abs(W - w).max()))
    if A.std(0).max() > Q["accel_std_max"]:
        bad.append(f"가속 흔들림(표준편차 {A.std(0).max():.3f} m/s²)")
    if W.std(0).max() > Q["gyro_std_max"] or info["gyro_dev_max"] > Q["gyro_dev_max"]:
        bad.append(f"회전 중(자이로 표준편차 {W.std(0).max():.4f}, 최대 편차 {info['gyro_dev_max']:.3f} rad/s)")
    return info, bad


def fit(a, iters=50):
    """정지 평균들(N×3) → 편향 b, 배율 s. 잔차 r_i = |(a_i − b)/s| − g, 가우스–뉴턴."""
    b, s = np.zeros(3), np.ones(3)
    for _ in range(iters):
        u = (a - b) / s
        n = np.linalg.norm(u, axis=1)
        r = n - G
        du = u / n[:, None]                        # ∂|u|/∂u
        J = np.hstack([-du / s, -du * u / s])      # ∂r/∂b, ∂r/∂s
        step, *_ = np.linalg.lstsq(J, -r, rcond=None)
        b, s = b + step[:3], s + step[3:]
        if np.abs(step).max() < 1e-10:
            break
    u = (a - b) / s
    return b, s, np.linalg.norm(u, axis=1) - G, float(np.linalg.cond(J))


def calibrate(folders):
    poses, bad = [], []
    for f in folders:
        info, why = static_pose(f)
        poses.append(info)
        bad += [f"{os.path.basename(f.rstrip('/'))}: {x}" for x in why]
    if len(folders) < Q["min_poses"]:
        bad.append(f"자세 수 {len(folders)} < {Q['min_poses']}(6개는 미지수와 같아 잔차·교차 확인이 무의미)")
    ok = [p for p in poses if "accel_mean" in p]
    res = {"g": G, "quality_limits": Q, "poses": poses, "valid": False, "reasons": bad,
           "model": "대각 배율 + 편향(비직교·교차축·좌표 회전 미보정)", "apply": "a_corr = (a_raw − bias) / scale (축별), gyro_corr = gyro_raw − gyro_bias"}
    if len(ok) < 6:
        res["reasons"] = bad or ["유효 자세 < 6"]
        return res
    a = np.array([p["accel_mean"] for p in ok])
    dirs = a / np.linalg.norm(a, axis=1)[:, None]
    for k, c in enumerate("xyz"):
        if dirs[:, k].max() < Q["cover_cos"] or dirs[:, k].min() > -Q["cover_cos"]:
            bad.append(f"자세 분포 부족: {c}축 ±방향 중 하나가 중력과 60° 안에 놓인 적 없음(최대 {dirs[:, k].max():.2f}, 최소 {dirs[:, k].min():.2f})")
    b, s, r, cond = fit(a)
    if not np.isfinite(cond) or cond > Q["cond_max"]:
        bad.append(f"식별 불가: 자세 방향 분포가 배율·편향을 구분하지 못함(조건수 {cond:.3g} > {Q['cond_max']:g}) — 서로 다른 방향을 더 기록")
    loo = []
    for i in range(len(a) if len(a) >= Q["min_poses"] else 0):
        m = np.arange(len(a)) != i
        bi, si, _, _ = fit(a[m])
        loo.append(float(np.linalg.norm((a[i] - bi) / si) - G))
    if not (Q["scale_range"][0] <= s.min() and s.max() <= Q["scale_range"][1]):
        bad.append(f"배율 범위 밖 {np.round(s, 4).tolist()}")
    if np.abs(b).max() > Q["bias_max"]:
        bad.append(f"편향 범위 밖 {np.round(b, 3).tolist()}")
    if np.abs(r).max() > Q["resid_max"]:
        bad.append(f"보정 후 잔차 최대 {np.abs(r).max():.3f} m/s²")
    if loo and np.abs(loo).max() > Q["loo_resid_max"]:
        bad.append(f"한 자세 빼고 맞춘 잔차 최대 {np.abs(loo).max():.3f} m/s²")
    gm = np.array([p["gyro_mean"] for p in ok])
    gmed = np.median(gm, 0)
    dev = np.linalg.norm(gm - gmed, axis=1)
    for p, dv in zip(ok, dev):
        if dv > Q["gyro_pose_dev_max"]:
            bad.append(f"{os.path.basename(p['folder'].rstrip('/'))}: 회전 중(자이로 평균이 다른 자세와 {dv:.3f} rad/s 다름 — 일정한 회전)")
    if np.linalg.norm(gmed) > Q["gyro_bias_abs_max"]:
        bad.append(f"자이로 공통 평균 {np.linalg.norm(gmed):.3f} rad/s — 편향으로 보기에 큼(모든 자세에서 회전?)")
    gyro_bias = gmed
    res.update({"accel_bias_m_s2": b.tolist(), "accel_scale": s.tolist(), "gyro_bias_rad_s": gyro_bias.tolist(),
                "gyro_bias_spread_rad_s": np.ptp([p["gyro_mean"] for p in ok], 0).tolist(), "jacobian_cond": cond,
                "norm_before": [float(np.linalg.norm(x)) for x in a], "resid_after": r.tolist(), "loo_resid": loo,
                "reasons": bad, "valid": not bad})
    return res


def apply(a, cal):
    return (np.asarray(a, float) - np.asarray(cal["accel_bias_m_s2"])) / np.asarray(cal["accel_scale"])


def main():
    ap = argparse.ArgumentParser(description="L515 가속도계 다자세 보정(축 정렬 불필요)")
    ap.add_argument("folders", nargs="+")
    ap.add_argument("--out", default="imu_calib.json")
    ap.add_argument("--force", action="store_true", help="거부돼도 valid=false로 저장")
    a = ap.parse_args()
    cal = calibrate(a.folders)
    if "accel_bias_m_s2" in cal:
        print("편향 m/s²", np.round(cal["accel_bias_m_s2"], 4), "배율", np.round(cal["accel_scale"], 4))
        print("|a| 보정 전", np.round(cal["norm_before"], 3).tolist())
        print("보정 후 잔차", np.round(cal["resid_after"], 4).tolist(), "한 자세 빼고", np.round(cal["loo_resid"], 4).tolist())
    if not cal["valid"]:
        print("❌ 거부:", *cal["reasons"], sep="\n  ")
        if not a.force:
            sys.exit(2)
    json.dump(cal, open(a.out, "w"), ensure_ascii=False, indent=1)
    print("→", a.out, "(valid)" if cal["valid"] else "(valid=false, --force)")


if __name__ == "__main__":
    main()
