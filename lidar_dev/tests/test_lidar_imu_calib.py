# imu_calib 시험: 편향·배율을 아는 합성 6자세 → 되찾기, 자세가 빠지면 거부.
import csv, os, sys

import pytest

np = pytest.importorskip("numpy")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import imu_calib as ic  # noqa: E402

BIAS = np.array([0.12, -0.21, 0.08])
SCALE = np.array([0.985, 1.012, 0.994])


def write_pose(folder, g_dir, rng):
    os.makedirs(folder, exist_ok=True)
    with open(os.path.join(folder, "imu.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, ["host_mono_s", "kind", "ts_ms", "ts_domain", "fn", "x", "y", "z"])
        w.writeheader()
        for k in range(400):
            a = np.asarray(g_dir) * ic.G * SCALE + BIAS + rng.normal(0, 0.02, 3)
            w.writerow({"host_mono_s": k * 0.005, "kind": "accel", "ts_ms": k * 5, "ts_domain": "s", "fn": k, "x": a[0], "y": a[1], "z": a[2]})
            g = rng.normal(0, 0.002, 3) + 0.001
            w.writerow({"host_mono_s": k * 0.005, "kind": "gyro", "ts_ms": k * 5, "ts_domain": "s", "fn": k, "x": g[0], "y": g[1], "z": g[2]})


def test_recovers_bias_and_scale(tmp_path):
    rng = np.random.default_rng(1)
    dirs = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
    folders = []
    for i, d in enumerate(dirs):
        f = str(tmp_path / f"p{i}")
        write_pose(f, d, rng)
        folders.append(f)
    cal = ic.calibrate(folders[::-1])   # 순서 상관없음
    assert np.allclose(cal["accel_bias_m_s2"], BIAS, atol=0.01)
    assert np.allclose(cal["accel_scale"], SCALE, atol=0.002)
    assert all(abs(v - ic.G) < 0.02 for v in cal["norm_after"].values())


def test_missing_pose_rejected(tmp_path):
    rng = np.random.default_rng(2)
    folders = []
    for i, d in enumerate([(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, 1)]):  # −z 빠짐(+z 두 번)
        f = str(tmp_path / f"p{i}")
        write_pose(f, d, rng)
        folders.append(f)
    with pytest.raises(SystemExit):
        ic.calibrate(folders)
