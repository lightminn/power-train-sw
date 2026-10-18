# imu_calib 시험: 편향·배율을 아는 합성 자세 → 되찾기(축 정렬 불필요), 잘못된 입력 거부(교차검토 4차 반례 포함).
import csv, math, os, sys

import pytest

np = pytest.importorskip("numpy")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import imu_calib as ic  # noqa: E402

BIAS = np.array([0.12, -0.21, 0.08])
SCALE = np.array([0.985, 1.012, 0.994])
AXES = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
POSES9 = AXES + [(1, 1, 1), (-1, 1, -1), (1, -1, -1)]   # 축 6 + 비스듬히 3(교차 확인용 여분)


def write_pose(folder, g_dir, rng, gyro_fn=None, n=1000, gyro=True):
    """5 ms 간격 n개(기본 5 s — 기록 절차와 같음). gyro_fn(k) → 그 시각 자이로 추가 성분(움직임 주입)."""
    os.makedirs(folder, exist_ok=True)
    g_dir = np.asarray(g_dir, float) / np.linalg.norm(g_dir)
    with open(os.path.join(folder, "imu.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, ["host_mono_s", "kind", "ts_ms", "ts_domain", "fn", "x", "y", "z"])
        w.writeheader()
        for k in range(n):
            a = g_dir * ic.G * SCALE + BIAS + rng.normal(0, 0.02, 3)
            w.writerow({"host_mono_s": k * 0.005, "kind": "accel", "ts_ms": k * 5, "ts_domain": "s", "fn": k, "x": a[0], "y": a[1], "z": a[2]})
            if gyro:
                g = rng.normal(0, 0.002, 3) + 0.001 + (gyro_fn(k) if gyro_fn else 0)
                w.writerow({"host_mono_s": k * 0.005, "kind": "gyro", "ts_ms": k * 5, "ts_domain": "s", "fn": k, "x": g[0], "y": g[1], "z": g[2]})


def rot(axis, deg):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    t = math.radians(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K


def make(tmp_path, dirs, **kw):
    rng = np.random.default_rng(len(dirs))
    out = []
    for i, d in enumerate(dirs):
        f = str(tmp_path / f"p{i}")
        write_pose(f, d, rng, **kw)
        out.append(f)
    return out


def test_recovers_bias_and_scale_axis_poses(tmp_path):
    cal = ic.calibrate(make(tmp_path, POSES9)[::-1])   # 순서 상관없음
    assert cal["valid"], cal["reasons"]
    assert np.allclose(cal["accel_bias_m_s2"], BIAS, atol=0.01)
    assert np.allclose(cal["accel_scale"], SCALE, atol=0.002)
    assert max(abs(r) for r in cal["resid_after"]) < 0.01


def test_tilted_poses_do_not_bias_scale(tmp_path):
    """반례: 6자세를 모두 20° 틀어 놓아도 배율이 0.94로 흡수되지 않는다(타원체 맞춤은 자세 방향과 무관)."""
    R = rot((1, 1, 0.3), 20)
    cal = ic.calibrate(make(tmp_path, [R @ np.array(d) for d in POSES9]))
    assert cal["valid"], cal["reasons"]
    assert np.allclose(cal["accel_scale"], SCALE, atol=0.003), cal["accel_scale"]


def test_arbitrary_orientations_recover(tmp_path):
    """원통형 몸체라 옆면 각도를 못 맞춰도: 임의 방향 9자세(각 축 ± 포함)로 되찾는다."""
    rng = np.random.default_rng(7)
    dirs = [np.array(d) + rng.normal(0, 0.35, 3) for d in AXES] + [rng.normal(0, 1, 3) for _ in range(4)]
    cal = ic.calibrate(make(tmp_path, dirs))
    assert cal["valid"], cal["reasons"]
    assert np.allclose(cal["accel_bias_m_s2"], BIAS, atol=0.015) and np.allclose(cal["accel_scale"], SCALE, atol=0.003)


def test_poor_coverage_rejected(tmp_path):
    """자세가 +z 근처에만 몰리면(−z·±x·±y 없음) 거부."""
    dirs = [rot((1, 0, 0), a) @ np.array([0, 0, 1.0]) for a in (-30, -15, 0, 15, 30)] + [rot((0, 1, 0), 25) @ np.array([0, 0, 1.0])]
    cal = ic.calibrate(make(tmp_path, dirs))
    assert not cal["valid"] and any("분포" in r for r in cal["reasons"]), cal["reasons"]


def test_motion_inside_window_rejected(tmp_path):
    """반례: 한 자세에서 가운데 구간에 0.5 rad/s 회전 → 정지 아님으로 거부(평균에 섞이지 않게)."""
    folders = make(tmp_path, POSES9)
    write_pose(folders[2], AXES[2], np.random.default_rng(9), gyro_fn=lambda k: 0.5 if 400 < k < 600 else 0.0)
    cal = ic.calibrate(folders)
    assert not cal["valid"] and any("회전" in r for r in cal["reasons"]), cal["reasons"]


def test_motion_at_ends_is_trimmed(tmp_path):
    """앞뒤 10%(손 떼는 순간)의 회전은 공통 구간 자르기로 빠져 자이로 편향을 오염시키지 않는다."""
    folders = make(tmp_path, POSES9, gyro_fn=lambda k: 0.5 if (k < 80 or k > 920) else 0.0)
    cal = ic.calibrate(folders)
    assert cal["valid"], cal["reasons"]
    assert np.allclose(cal["gyro_bias_rad_s"], 0.001, atol=0.001), cal["gyro_bias_rad_s"]


def test_missing_gyro_rejected(tmp_path):
    folders = make(tmp_path, POSES9)
    write_pose(folders[0], AXES[0], np.random.default_rng(3), gyro=False)
    cal = ic.calibrate(folders)
    assert not cal["valid"] and any("자료 부족" in r for r in cal["reasons"]), cal["reasons"]


def test_six_exact_poses_not_valid(tmp_path):
    """6자세는 미지수와 같아 잔차가 늘 0 — 값은 내도 유효 판정은 하지 않는다."""
    cal = ic.calibrate(make(tmp_path, AXES))
    assert not cal["valid"] and any("자세 수" in r for r in cal["reasons"]), cal["reasons"]
    assert np.allclose(cal["accel_scale"], SCALE, atol=0.002)


def test_wrong_pose_caught_by_cross_check(tmp_path):
    """한 자세의 기록이 다른 센서처럼 어긋나면(배율 3% 다름) 잔차·교차 확인으로 거부."""
    folders = make(tmp_path, POSES9)
    rng = np.random.default_rng(11)
    with open(os.path.join(folders[4], "imu.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, ["host_mono_s", "kind", "ts_ms", "ts_domain", "fn", "x", "y", "z"])
        w.writeheader()
        for k in range(1000):
            a = np.array([0, 0, 1.0]) * ic.G * SCALE * 1.03 + BIAS + rng.normal(0, 0.02, 3)
            w.writerow({"host_mono_s": k * 0.005, "kind": "accel", "ts_ms": k * 5, "ts_domain": "s", "fn": k, "x": a[0], "y": a[1], "z": a[2]})
            g = rng.normal(0, 0.002, 3)
            w.writerow({"host_mono_s": k * 0.005, "kind": "gyro", "ts_ms": k * 5, "ts_domain": "s", "fn": k, "x": g[0], "y": g[1], "z": g[2]})
    cal = ic.calibrate(folders)
    assert not cal["valid"] and any("잔차" in r for r in cal["reasons"]), cal["reasons"]


def test_cube_diagonals_not_identifiable(tmp_path):
    """5차 반례: 정육면체 대각선 8방향 + 반복 1 — 모든 축 성분 크기가 같아 배율을 구분 못 함 → 조건수로 거부."""
    import itertools
    dirs = [np.array(d, float) for d in itertools.product((1, -1), repeat=3)] + [np.array([1.0, 1.0, 1.0])]
    cal = ic.calibrate(make(tmp_path, dirs))
    assert not cal["valid"] and any("식별" in r for r in cal["reasons"]), cal["reasons"]


def test_constant_rotation_rejected(tmp_path):
    """5차 반례: 한 자세에서 중력축 둘레로 일정하게 0.5 rad/s 회전 — 분산은 작지만 자이로 평균이 다른 자세와 달라 거부."""
    folders = make(tmp_path, POSES9)
    write_pose(folders[3], AXES[3], np.random.default_rng(5), gyro_fn=lambda k: np.array([0.0, 0.5, 0.0]))
    cal = ic.calibrate(folders)
    assert not cal["valid"] and any("일정한 회전" in r for r in cal["reasons"]), cal["reasons"]


def test_good_set_is_well_conditioned(tmp_path):
    cal = ic.calibrate(make(tmp_path, POSES9))
    assert cal["valid"] and cal["jacobian_cond"] < ic.Q["cond_max"], cal["jacobian_cond"]
