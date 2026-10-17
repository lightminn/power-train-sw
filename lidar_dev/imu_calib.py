# imu_calib.py — L515 내장 가속도계 6자세 정지 보정(축별 편향·배율). 기록 6개(각 자세 3~5 s 정지)를 받아
# 축마다 +g·−g 평균으로 편향 b = (a⁺ + a⁻)/2, 배율 s = (a⁺ − a⁻)/(2g)를 구한다. 자이로는 정지 평균을 편향으로.
# 사용: python imu_calib.py <기록1> ... <기록6> [--out imu_calib.json]   (record_seq.py 기록, imu.csv 필요)
#       자세 순서는 상관없다 — 기록마다 가장 큰 축과 부호로 자동 분류한다. 6자세(±x, ±y, ±z)가 모두 있어야 한다.
import argparse, csv, json, os, sys
import numpy as np

G = 9.80665


def static_mean(folder, kind, gyro_max=0.02):
    rows = [r for r in csv.DictReader(open(os.path.join(folder, "imu.csv"))) if r["kind"] == kind]
    A = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in rows])
    if kind == "accel" and len(A) > 40:
        A = A[len(A) // 10: -len(A) // 10]   # 앞뒤 10%(손 떼는 순간) 제외
    return A.mean(0), A.std(0), len(A)


def calibrate(folders):
    poses = {}
    for f in folders:
        a, sd, n = static_mean(f, "accel")
        ax = int(np.argmax(np.abs(a)))
        sign = "+" if a[ax] > 0 else "-"
        key = f"{sign}{'xyz'[ax]}"
        if key in poses:
            raise SystemExit(f"자세 {key}가 두 번 나옴: {poses[key]['folder']}, {f}")
        g_mean, g_sd, gn = static_mean(f, "gyro")
        poses[key] = {"folder": f, "accel_mean": a.tolist(), "accel_std": sd.tolist(), "n": n, "gyro_mean": g_mean.tolist()}
    missing = [k for k in ("+x", "-x", "+y", "-y", "+z", "-z") if k not in poses]
    if missing:
        raise SystemExit(f"빠진 자세: {missing}")
    bias, scale = np.zeros(3), np.ones(3)
    for i, c in enumerate("xyz"):
        ap, am = poses[f"+{c}"]["accel_mean"][i], poses[f"-{c}"]["accel_mean"][i]
        bias[i] = (ap + am) / 2
        scale[i] = (ap - am) / (2 * G)
    gyro_bias = np.mean([p["gyro_mean"] for p in poses.values()], 0)
    # 보정 전·후 |a| (자세별)
    before = {k: float(np.linalg.norm(p["accel_mean"])) for k, p in poses.items()}
    after = {k: float(np.linalg.norm((np.array(p["accel_mean"]) - bias) / scale)) for k, p in poses.items()}
    return {"accel_bias_m_s2": bias.tolist(), "accel_scale": scale.tolist(), "gyro_bias_rad_s": gyro_bias.tolist(),
            "norm_before": before, "norm_after": after, "g": G, "poses": poses,
            "apply": "a_corr = (a_raw − bias) / scale (축별), gyro_corr = gyro_raw − gyro_bias"}


def apply(a, cal):
    return (np.asarray(a, float) - np.asarray(cal["accel_bias_m_s2"])) / np.asarray(cal["accel_scale"])


def main():
    ap = argparse.ArgumentParser(description="L515 가속도계 6자세 보정")
    ap.add_argument("folders", nargs=6)
    ap.add_argument("--out", default="imu_calib.json")
    a = ap.parse_args()
    cal = calibrate(a.folders)
    json.dump(cal, open(a.out, "w"), ensure_ascii=False, indent=1)
    print("편향 m/s²", np.round(cal["accel_bias_m_s2"], 4), "배율", np.round(cal["accel_scale"], 4))
    print("|a| 보정 전", {k: round(v, 3) for k, v in cal["norm_before"].items()})
    print("|a| 보정 후", {k: round(v, 3) for k, v in cal["norm_after"].items()})
    print("→", a.out)


if __name__ == "__main__":
    main()
